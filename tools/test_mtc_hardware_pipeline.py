#!/usr/bin/env python3
"""Real MTC launch against a synthetic board and targets; no physical devices."""
import os
import argparse
import signal
import subprocess
import sys
from pathlib import Path
from time import monotonic
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import Bool,String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped,Pose
from moveit_msgs.msg import PlanningScene,CollisionObject
from shape_msgs.msg import SolidPrimitive
from test_mtc_hardware_interfaces import FakeBoard
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--domain',type=int,default=197);parser.add_argument('--cancel',action='store_true');args=parser.parse_args()
os.environ['ROS_DOMAIN_ID']=str(args.domain)
log=Path(f'/tmp/mtc_hardware_pipeline_{os.getpid()}.log')
rclpy.init();node=FakeBoard();node.authorize=True
state={'status':'','hardware_ready':False}
node.create_subscription(Bool,'/mtc_grasp/hardware_ready',lambda m:state.update(hardware_ready=m.data),10)
node.create_subscription(String,'/mtc_grasp/status',lambda m:state.update(status=m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
ready=node.create_publisher(Bool,'/mtc_grasp/vision_ready',10)
target=node.create_publisher(PoseStamped,'/mtc_grasp/target_pose',10)
scene=node.create_publisher(PlanningScene,'/mtc_grasp/observed_scene',10)

def input_sample():
    ready.publish(Bool(data=True))
    if not state['status'].startswith('WAITING_FOR_RGBD'):return
    msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
    msg.pose.orientation.w=1.;msg.pose.position.x=.24;msg.pose.position.z=.025
    target.publish(msg)
    world=PlanningScene(is_diff=True);world.robot_state.is_diff=True
    for color,y in [('red',0.),('green',-.07),('blue',.07)]:
        pose=Pose();pose.orientation.w=1.;pose.position.x=.24;pose.position.y=y;pose.position.z=.025
        box=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[.04,.04,.05])
        world.world.collision_objects.append(CollisionObject(header=msg.header,id='cargo_'+color,primitives=[box],primitive_poses=[pose],operation=CollisionObject.ADD))
    scene.publish(world)
node.create_timer(.05,input_sample)
start=node.create_client(Trigger,'/mtc_grasp/start')
stream=log.open('w')
process=subprocess.Popen(['ros2','launch','landerpi_mtc_grasp','color_grasp_hardware.launch.py',
 'start_drivers:=false','start_joystick:=false','start_camera:=false','start_vision:=false',f'ros_domain_id:={args.domain}',
 'velocity_scale:=0.4','acceleration_scale:=0.4'],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
print('Log:',log,flush=True)
def until(predicate,timeout):
    deadline=monotonic()+timeout
    while not predicate():
        if process.poll() is not None:raise RuntimeError('launch exited: '+str(log))
        if state['status'].startswith('FAULT'):raise RuntimeError(state['status'])
        if monotonic()>deadline:raise TimeoutError(state['status']+' '+str(log))
        rclpy.spin_once(node,timeout_sec=.05)
try:
    until(lambda:start.service_is_ready() and state['status'].startswith('WAITING_FOR_START') and state['hardware_ready'],90)
    deadline=monotonic()+2.
    while monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    assert not node.commands,'arm moved before START'
    request=start.call_async(Trigger.Request());until(request.done,10);assert request.result().success,request.result().message
    if args.cancel:
        until(lambda:len(node.commands)>0,90)
        cancel=node.create_client(Trigger,'/mtc_grasp/cancel');until(cancel.service_is_ready,5)
        stopped=cancel.call_async(Trigger.Request());node.authorize=False
        until(stopped.done,5);assert stopped.result().success
        until(lambda:state['status'].startswith('FAULT') and bool(node.stops),10)
        count=len(node.commands);deadline=monotonic()+.5
        while monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        assert len(node.commands)==count,'commands continued after cancel'
        print('HARDWARE CANCEL PASSED: MTC cancel/authorization loss stops real FJT interface and latches fault. Board is synthetic.',flush=True)
    else:
        success='SUCCESS color=red real_hardware=true execution_complete=true grasp_verified=false'
        until(lambda:success in log.read_text(errors='replace') and state['status'].startswith('WAITING_FOR_START'),240)
        assert not any(name.startswith(('/gazebo/','/simulation/')) for name,_ in node.get_service_names_and_types())
        print('HARDWARE PIPELINE PASSED: idle gate, real launch + feedback FJT, complete MTC pick/place, no Gazebo services. Sensors and board are synthetic.',flush=True)

finally:
    node.authorize=False
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    stream.close()
