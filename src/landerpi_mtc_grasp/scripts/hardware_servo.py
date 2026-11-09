#!/usr/bin/env python3
"""Real bus feedback, cancellable FJT and independent base-command watchdog."""
from pathlib import Path
from threading import Lock
from time import monotonic,sleep
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool,Header
from geometry_msgs.msg import Twist
from control_msgs.action import FollowJointTrajectory
from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import GetBusServoCmd,ServosPosition,ServoPosition,SetBusServoState,BusServoState
from hardware_geometry import to_angle,to_pulse,validate,interpolate

class HardwareServo(Node):
    def __init__(self):
        super().__init__('mtc_hardware_servo')
        self.declare_parameter('servo_config','')
        path=Path(self.get_parameter('servo_config').value)
        raw=yaml.safe_load(path.read_text())['/**']['ros__parameters']
        self.names=['joint1','joint2','joint3','joint4','joint5','r_joint']
        self.config={n:raw[n] for n in self.names}
        self.group=ReentrantCallbackGroup();self.lock=Lock();self.busy=False
        self.positions={};self.feedback_time=0.;self.enabled=False;self.enable_time=0.
        self.base=Twist();self.base_time=0.;self.query=None;self.query_time=0.
        self.state=self.create_publisher(JointState,'/joint_states',10)
        self.ready=self.create_publisher(Bool,'/mtc_grasp/hardware_ready',10)
        self.commands=self.create_publisher(ServosPosition,'/ros_robot_controller/bus_servo/set_position',10)
        self.stop_pub=self.create_publisher(SetBusServoState,'/ros_robot_controller/bus_servo/set_state',10)
        self.base_pub=self.create_publisher(Twist,'/mtc_grasp/base_cmd_vel',10)
        self.client=self.create_client(GetBusServoState,'/ros_robot_controller/bus_servo/get_state',callback_group=self.group)
        self.create_subscription(Bool,'/mtc_grasp/operator_enabled',self.enable,10,callback_group=self.group)
        self.create_subscription(Twist,'/mtc_grasp/manual_cmd_vel',self.base_command,10,callback_group=self.group)
        self.servers=[]
        for name,joints in [('arm_controller',self.names[:5]),('gripper_controller',['r_joint'])]:
            self.servers.append(ActionServer(self,FollowJointTrajectory,'/'+name+'/follow_joint_trajectory',
                execute_callback=self.execute,goal_callback=lambda goal,n=joints:self.accept(goal,n),
                cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group))
        self.create_timer(.1,self.poll,callback_group=self.group)

    def enable(self,msg):
        with self.lock:
            self.enabled=msg.data;self.enable_time=monotonic()
            if not msg.data and self.busy:self.stop_servos()

    def authorized(self):
        return self.enabled and monotonic()-self.enable_time<.6

    def base_command(self,msg):self.base=msg;self.base_time=monotonic()

    def stop_servos(self):
        self.stop_pub.publish(SetBusServoState(state=[BusServoState(present_id=[1,self.config[n]['id']],stop=[1]) for n in self.names]))

    def poll(self):
        now=monotonic()
        command=Twist()
        if not self.busy and not self.authorized() and now-self.base_time<.3:command=self.base
        self.base_pub.publish(command)
        self.ready.publish(Bool(data=now-self.feedback_time<.8 and len(self.positions)==6))
        if self.query:
            if not self.query.done():return
            try:
                response=self.query.result()
                if not response.success or len(response.state)!=6:raise ValueError('incomplete servo feedback')
                positions={}
                for name,entry in zip(self.names,response.state):
                    if not entry.position:raise ValueError('servo position unavailable: '+name)
                    positions[name]=to_angle(entry.position[0],self.config[name])
                self.positions=positions;self.feedback_time=now
                self.state.publish(JointState(header=Header(stamp=self.get_clock().now().to_msg()),
                    name=self.names,position=[positions[n] for n in self.names]))
            except Exception as exc:self.get_logger().warning(str(exc))
            self.query=None
        if self.client.service_is_ready():
            self.query=self.client.call_async(GetBusServoState.Request(cmd=[GetBusServoCmd(id=self.config[n]['id'],get_position=1) for n in self.names]))

    def accept(self,goal,joints):
        with self.lock:
            if self.busy or not self.authorized() or monotonic()-self.feedback_time>.8:return GoalResponse.REJECT
            try:
                validate(goal.trajectory,joints,self.config,self.positions)
                stamp=goal.trajectory.header.stamp.sec+goal.trajectory.header.stamp.nanosec*1e-9
                if stamp and stamp<self.get_clock().now().nanoseconds/1e9-.1:return GoalResponse.REJECT
            except (ValueError,KeyError):return GoalResponse.REJECT
            self.busy=True
        return GoalResponse.ACCEPT

    def execute(self,handle):
        result=FollowJointTrajectory.Result()
        names=handle.request.trajectory.joint_names;points=handle.request.trajectory.points
        initial=[self.positions[n] for n in names]
        stamp=handle.request.trajectory.header.stamp
        requested=stamp.sec+stamp.nanosec*1e-9
        start=monotonic()+max(0.,requested-self.get_clock().now().nanoseconds/1e9) if requested else monotonic()
        end=points[-1].time_from_start.sec+points[-1].time_from_start.nanosec*1e-9
        try:
            while rclpy.ok():
                if handle.is_cancel_requested:
                    self.stop_servos();handle.canceled();return result
                if not self.authorized() or monotonic()-self.feedback_time>.8:
                    raise RuntimeError('operator authorization or actual servo feedback lost')
                elapsed=monotonic()-start
                if elapsed>=end:
                    errors=[abs(self.positions[n]-v) for n,v in zip(names,points[-1].positions)]
                    if max(errors)<.04:handle.succeed();return result
                    if elapsed>end+3.:raise RuntimeError('actual servo did not reach trajectory endpoint')
                if elapsed>=0:
                    positions=interpolate(points,initial,elapsed)
                    with self.lock:
                        if not self.authorized():raise RuntimeError('operator authorization lost')
                        self.commands.publish(ServosPosition(duration=.06,position=[ServoPosition(id=self.config[n]['id'],position=to_pulse(v,self.config[n])) for n,v in zip(names,positions)]))
                    feedback=FollowJointTrajectory.Feedback(joint_names=names)
                    feedback.header.stamp=self.get_clock().now().to_msg()
                    feedback.desired.positions=positions;feedback.actual.positions=[self.positions[n] for n in names]
                    feedback.error.positions=[a-b for a,b in zip(positions,feedback.actual.positions)]
                    handle.publish_feedback(feedback)
                sleep(.05)
            raise RuntimeError('hardware executor shutting down')
        except Exception as exc:
            self.stop_servos();result.error_code=result.PATH_TOLERANCE_VIOLATED;result.error_string=str(exc);handle.abort();return result
        finally:
            self.busy=False;self.base=Twist();self.base_time=0.

def main():
    rclpy.init();node=HardwareServo();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.base_pub.publish(Twist());node.stop_servos()
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
