#!/usr/bin/env python3
"""Isolated ROS tests with a fake board: never opens serial or camera devices."""
import sys
from pathlib import Path
from time import monotonic,sleep
from threading import Thread
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.task import Future
from std_msgs.msg import Bool,String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import BusServoState,ServosPosition,SetBusServoState
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/landerpi_mtc_grasp/scripts'))
from hardware_servo import HardwareServo
from mtc_joystick import MtcJoystick
CONFIG=ROOT/'src/driver/servo_controller/config/servo_controller.yaml'

def until(predicate,timeout=5.):
    deadline=monotonic()+timeout
    while not predicate():
        if monotonic()>deadline:raise AssertionError('ROS test timeout')
        sleep(.02)

def result(future,timeout=5.):until(future.done,timeout);return future.result()

class FakeBoard(Node):
    def __init__(self):
        super().__init__('fake_mtc_board')
        raw=yaml.safe_load(CONFIG.read_text())['/**']['ros__parameters']
        self.positions={raw[n]['id']:raw[n]['init'] for n in ['joint1','joint2','joint3','joint4','joint5','r_joint']}
        self.feedback=True;self.authorize=False;self.stops=[];self.commands=[];self.base=[]
        self.create_service(GetBusServoState,'/ros_robot_controller/bus_servo/get_state',self.query)
        self.create_subscription(ServosPosition,'/ros_robot_controller/bus_servo/set_position',self.command,10)
        self.create_subscription(SetBusServoState,'/ros_robot_controller/bus_servo/set_state',lambda m:self.stops.append(m),10)
        self.create_subscription(Twist,'/mtc_grasp/base_cmd_vel',lambda m:self.base.append(m),10)
        self.enable=self.create_publisher(Bool,'/mtc_grasp/operator_enabled',10)
        self.manual=self.create_publisher(Twist,'/mtc_grasp/manual_cmd_vel',10)
        self.create_timer(.05,lambda:self.enable.publish(Bool(data=self.authorize)))
    def query(self,req,res):
        res.success=self.feedback
        res.state=[BusServoState(position=[self.positions[c.id]]) for c in req.cmd] if self.feedback else []
        return res
    def command(self,msg):
        self.commands.append(msg)
        for point in msg.position:self.positions[point.id]=point.position

class Interfaces(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args','-p','servo_config:='+str(CONFIG)])
        cls.board=FakeBoard();cls.servo=HardwareServo()
        cls.executor=MultiThreadedExecutor(num_threads=6)
        cls.executor.add_node(cls.board);cls.executor.add_node(cls.servo)
        cls.thread=Thread(target=cls.executor.spin,daemon=True);cls.thread.start()
        cls.client=ActionClient(cls.board,FollowJointTrajectory,'/arm_controller/follow_joint_trajectory')
        until(lambda:cls.client.server_is_ready() and len(cls.servo.positions)==6)
    @classmethod
    def tearDownClass(cls):
        cls.board.authorize=False;cls.executor.shutdown();cls.thread.join(3)
        cls.client.destroy();cls.servo.destroy_node();cls.board.destroy_node();rclpy.shutdown()
    def setUp(self):
        self.board.feedback=True;self.board.authorize=False
        until(lambda:len(self.servo.positions)==6 and monotonic()-self.servo.feedback_time<.4)
        sleep(.15)
    def goal(self,delta=.1,duration=.6):
        goal=FollowJointTrajectory.Goal();goal.trajectory.joint_names=self.servo.names[:5]
        positions=[self.servo.positions[n] for n in goal.trajectory.joint_names];positions[0]+=delta
        point=JointTrajectoryPoint(positions=positions);point.time_from_start.sec=int(duration)
        point.time_from_start.nanosec=round(duration%1*1e9);goal.trajectory.points=[point];return goal
    def authorize(self):self.board.authorize=True;until(self.servo.authorized)
    def test_01_unarmed_rejects(self):
        self.assertFalse(result(self.client.send_goal_async(self.goal())).accepted)
        self.assertEqual(len(self.board.commands),0)
    def test_02_actual_feedback_success(self):
        self.authorize();goal=self.goal();handle=result(self.client.send_goal_async(goal));self.assertTrue(handle.accepted)
        answer=result(handle.get_result_async());self.assertEqual(answer.status,4)
        self.assertEqual(answer.result.error_code,0)
        self.assertAlmostEqual(self.servo.positions['joint1'],goal.trajectory.points[-1].positions[0],delta=.04)
    def test_03_cancel_stops_servos(self):
        self.authorize();count=len(self.board.commands);handle=result(self.client.send_goal_async(self.goal(.3,2.)))
        until(lambda:len(self.board.commands)>count)
        self.assertTrue(result(handle.cancel_goal_async()).goals_canceling)
        self.assertEqual(result(handle.get_result_async()).status,5)
        until(lambda:len(self.board.stops)>0)
        self.assertEqual(list(self.board.stops[-1].state[0].present_id),[1,1])
        self.assertEqual(list(self.board.stops[-1].state[0].stop),[1])
    def test_04_feedback_loss_aborts(self):
        self.authorize();handle=result(self.client.send_goal_async(self.goal(.3,3.)))
        self.board.feedback=False
        answer=result(handle.get_result_async());self.assertEqual(answer.status,6)
        self.assertNotEqual(answer.result.error_code,0)
        self.board.feedback=True
    def test_05_operator_loss_aborts(self):
        self.authorize();handle=result(self.client.send_goal_async(self.goal(.2,2.)))
        self.board.authorize=False
        self.assertEqual(result(handle.get_result_async()).status,6)
    def test_06_base_watchdog(self):
        msg=Twist();msg.linear.x=.1
        for _ in range(6):self.board.manual.publish(msg);sleep(.05)
        until(lambda:bool(self.board.base) and self.board.base[-1].linear.x>.09)
        until(lambda:self.board.base[-1].linear.x==0.,2.)
        self.authorize();self.board.manual.publish(msg);sleep(.15)
        self.assertEqual(self.board.base[-1].linear.x,0.)

class Joystick(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.node=MtcJoystick();self.node.cmd=Mock();self.node.enable=Mock();self.node.mode_pub=Mock()
        self.node.start=Mock();self.node.start.service_is_ready.return_value=True
        self.node.cancel=Mock();self.node.cancel.service_is_ready.return_value=True
    def tearDown(self):self.node.destroy_node()
    def joy(self,button=None,axes=None):
        buttons=[0]*12
        if button is not None:buttons[button]=1
        self.node.joy(Joy(axes=axes or [0.,0.,0.],buttons=buttons))
    def test_manual_axes(self):
        self.joy(axes=[.5,1.,-.6]);self.node.tick();cmd=self.node.cmd.publish.call_args.args[0]
        self.assertAlmostEqual(cmd.linear.x,.15);self.assertAlmostEqual(cmd.linear.y,.075);self.assertAlmostEqual(cmd.angular.z,-.3)
    def test_start_and_back(self):
        self.node.task_ready=True;self.node.hardware_time=self.node.vision_time=monotonic()
        self.joy();self.joy(9);self.assertEqual(self.node.mode,'arming')
        pending=Future();self.node.start.call_async.return_value=pending
        self.node.arm_time-=.4;self.node.tick();self.node.start.call_async.assert_called_once()
        self.assertEqual(self.node.cmd.publish.call_args.args[0].linear.x,0.)
        pending.set_result(NS(success=True));self.assertEqual(self.node.mode,'automatic')
        self.joy(8);self.node.tick();self.node.cancel.call_async.assert_called_once()
        self.assertFalse(self.node.enable.publish.call_args.args[0].data)
    def test_start_requires_sensors(self):
        self.joy();self.joy(9);self.assertEqual(self.node.mode,'manual');self.node.start.call_async.assert_not_called()
    def test_disconnect_and_no_held_start_restart(self):
        self.node.mode='automatic';self.node.joy_time=monotonic()-1;self.node.tick()
        self.node.cancel.call_async.assert_called_once();self.assertEqual(self.node.mode,'stopping')
        self.node.status(String(data='WAITING_FOR_START'));self.joy(9)
        self.assertEqual(self.node.mode,'manual')
    def test_completion_waiting_resets_manual(self):
        self.node.mode='automatic';self.node.status(String(data='WAITING_FOR_START real_hardware=true'))
        self.assertEqual(self.node.mode,'manual')
        self.assertTrue(self.node.task_ready)
    def test_fault_blocks_manual(self):
        self.node.status(String(data='FAULT: feedback lost'));self.joy(axes=[1.,1.,1.]);self.node.tick()
        self.assertEqual(self.node.cmd.publish.call_args.args[0].linear.x,0.)
class Watchdogs(unittest.TestCase):
    def test_base_driver_watchdog(self):
        from controller.odom_publisher_node import Controller
        obj=NS(get_parameter=lambda _:NS(value=.4),last_velocity_command=monotonic()-1.,cmd_vel_callback=Mock())
        Controller.command_watchdog(obj)
        obj.cmd_vel_callback.assert_called_once()
        self.assertEqual(obj.cmd_vel_callback.call_args.args[0].linear.x,0.)
        obj.get_parameter=lambda _:NS(value=0.);obj.cmd_vel_callback.reset_mock()
        Controller.command_watchdog(obj);obj.cmd_vel_callback.assert_not_called()
    def test_board_watchdog(self):
        from ros_robot_controller.ros_robot_controller_node import RosRobotController
        obj=NS(motor_timeout=.6,motor_active=True,last_motor_time=monotonic()-1.,board=Mock())
        RosRobotController.motor_watchdog(obj)
        obj.board.set_motor_speed.assert_called_once_with([[1,0],[2,0],[3,0],[4,0]])
        self.assertFalse(obj.motor_active)
        obj.board.reset_mock();RosRobotController.motor_watchdog(obj)
        obj.board.set_motor_speed.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
