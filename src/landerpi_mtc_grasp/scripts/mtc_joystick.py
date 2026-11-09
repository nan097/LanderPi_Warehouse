#!/usr/bin/env python3
"""Manual base driving and explicit start/cancel for the real MTC task."""
from time import monotonic
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,String
from std_srvs.srv import Trigger

class MtcJoystick(Node):
    def __init__(self):
        super().__init__('mtc_joystick')
        for key,value in dict(start_button=9,back_button=8,linear_axis=1,lateral_axis=0,angular_axis=2,
                max_linear=.15,max_angular=.5,deadzone=.12,joy_timeout=.6).items():self.declare_parameter(key,value)
        self.mode='manual';self.buttons=None;self.joy_time=0.;self.manual=Twist()
        self.hardware_time=0.;self.vision_time=0.;self.task_ready=False;self.pending=None
        self.cmd=self.create_publisher(Twist,'/mtc_grasp/manual_cmd_vel',10)
        self.enable=self.create_publisher(Bool,'/mtc_grasp/operator_enabled',10)
        self.mode_pub=self.create_publisher(String,'/mtc_grasp/control_mode',10)
        self.start=self.create_client(Trigger,'/mtc_grasp/start');self.cancel=self.create_client(Trigger,'/mtc_grasp/cancel')
        self.create_subscription(Joy,'/joy',self.joy,qos_profile_sensor_data)
        self.create_subscription(Bool,'/mtc_grasp/hardware_ready',lambda m:setattr(self,'hardware_time',monotonic() if m.data else 0.),10)
        self.create_subscription(Bool,'/mtc_grasp/vision_ready',lambda m:setattr(self,'vision_time',monotonic() if m.data else 0.),10)
        self.create_subscription(String,'/mtc_grasp/status',self.status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05,self.tick)

    def param(self,key):return self.get_parameter(key).value
    def status(self,msg):
        self.task_ready=msg.data.startswith('WAITING_FOR_START')
        if self.task_ready and self.mode in ('stopping','automatic') and self.pending is None:
            self.mode='manual';self.manual=Twist()
        if msg.data.startswith('SUCCESS'):
            self.mode='manual';self.manual=Twist()
        elif msg.data.startswith(('FAULT','CANCELLED','GRASP_FAILED')):
            self.mode='fault';self.manual=Twist()

    def joy(self,msg):
        now=monotonic();fresh=now-self.joy_time<self.param('joy_timeout')
        self.joy_time=now
        def edge(key):
            i=self.param(key)
            return fresh and self.buttons is not None and 0<=i<len(msg.buttons) and bool(msg.buttons[i]) and not (i<len(self.buttons) and self.buttons[i])
        self.manual=Twist()
        for field,axis,limit in [('x','linear_axis','max_linear'),('y','lateral_axis','max_linear'),('z','angular_axis','max_angular')]:
            i=self.param(axis);value=msg.axes[i] if 0<=i<len(msg.axes) else 0.
            value=0. if not math.isfinite(value) or abs(value)<self.param('deadzone') else max(-1.,min(1.,value))*self.param(limit)
            if field=='x':self.manual.linear.x=value
            elif field=='y':self.manual.linear.y=value
            else:self.manual.angular.z=value
        if edge('back_button'):
            self.manual=Twist()
            if self.mode in ('arming','automatic'):
                self.mode='stopping'
                if self.cancel.service_is_ready():self.cancel.call_async(Trigger.Request())
        elif edge('start_button') and self.mode=='manual':
            if self.task_ready and now-self.hardware_time<.8 and now-self.vision_time<1. and self.start.service_is_ready():
                self.mode='arming';self.arm_time=now;self.manual=Twist()
            else:self.get_logger().warning('Waiting for real joint feedback, aligned RGB-D and MTC readiness.')
        self.buttons=list(msg.buttons)

    def started(self,future):
        self.pending=None
        try:
            if not future.result().success:self.mode='manual'
            elif self.mode=='arming':self.mode='automatic'
            elif self.cancel.service_is_ready():self.cancel.call_async(Trigger.Request())
        except Exception:self.mode='fault'

    def tick(self):
        now=monotonic();connected=now-self.joy_time<self.param('joy_timeout')
        if not connected:
            self.manual=Twist();self.buttons=None
            if self.mode in ('arming','automatic'):
                self.mode='stopping'
                if self.cancel.service_is_ready():self.cancel.call_async(Trigger.Request())
        if self.mode=='arming' and self.pending is None and now-self.arm_time>.3:
            self.pending=self.start.call_async(Trigger.Request());self.pending.add_done_callback(self.started)
        self.enable.publish(Bool(data=connected and self.mode in ('arming','automatic')))
        self.cmd.publish(self.manual if connected and self.mode=='manual' else Twist())
        self.mode_pub.publish(String(data=self.mode))

def main():
    rclpy.init();node=MtcJoystick()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.cmd.publish(Twist());node.enable.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
