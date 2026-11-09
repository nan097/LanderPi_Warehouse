#!/usr/bin/env python3
"""Physical warehouse acceptance: all selected cargo correct, no teleport/reset."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
from time import monotonic
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from landerpi_task_interfaces.action import ExecuteSortTask
from landerpi_task_interfaces.msg import RobotStatus
from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String,Bool

ROOT=Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0,str(ROOT/'src/landerpi_simulation/scripts'))
from warehouse_layout import validate,inside


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--quantity',type=int,default=0);parser.add_argument('--timeout',type=float,default=8000);parser.add_argument('--domain',type=int,default=78)
    parser.add_argument('--motion-profile',choices=['fast','legacy'],default='fast')
    args=parser.parse_args();config=yaml.safe_load((ROOT/'src/landerpi_simulation/config/warehouse.yaml').read_text());goods=validate(config)
    run=f'/tmp/landerpi_warehouse_acceptance_{os.getpid()}';log=Path(run+'.log');csv=run+'.csv'
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    stream=log.open('w')
    process=subprocess.Popen(['bash','tools/run_warehouse_simulation.sh','gui:=false','rviz:=false',f'ros_domain_id:={args.domain}',f'csv_path:={csv}',f'motion_profile:={args.motion_profile}'],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    node=None;started=monotonic();rclpy.init()
    try:
        node=Node('warehouse_acceptance');data={}
        observed={'peak_base_speed_mps':0.0,'peak_arm_joint_speed_radps':0.0}
        def receive(key,message):
            data[key]=message
            if key=='odom':
                velocity=message.twist.twist.linear
                observed['peak_base_speed_mps']=max(observed['peak_base_speed_mps'],math.hypot(velocity.x,velocity.y))
            if key=='joints':
                observed['peak_arm_joint_speed_radps']=max(observed['peak_arm_joint_speed_radps'],
                    max((abs(v) for name,v in zip(message.name,message.velocity) if name in [f'joint{i}' for i in range(1,6)]),default=0.))
        for typ,topic,key in [(RobotStatus,'/robot/status','status'),(Odometry,'/odom','odom'),(JointState,'/joint_states','joints'),(String,'/warehouse/progress','progress'),(Bool,'/simulation/object_attached','attached')]:
            node.create_subscription(typ,topic,lambda m,k=key:receive(k,m),10)
        client=ActionClient(node,ExecuteSortTask,'/task/execute_sort');entities=node.create_client(GetEntityState,'/gazebo/get_entity_state')
        last=None
        def until(predicate,timeout):
            nonlocal last
            end=monotonic()+timeout
            while not predicate():
                if process.poll() is not None:raise RuntimeError(f'warehouse launch exited {process.returncode}; {log}')
                if monotonic()>end:raise TimeoutError(f'warehouse acceptance timeout; {log}')
                rclpy.spin_once(node,timeout_sec=.1)
                if 'progress' in data:
                    state=json.loads(data['progress'].data);key=(state['completed'],state['phase'],state['current_object'])
                    if key!=last:
                        print(f"WAREHOUSE progress={state['completed']}/{state['total']} {state['phase']} {state['current_object']} {state['source']} -> {state['destination']}",flush=True);last=key
        until(lambda:client.server_is_ready() and entities.service_is_ready() and 'status' in data and data['status'].robot_ready,240)
        def entity(name):
            future=entities.call_async(GetEntityState.Request(name=name,reference_frame='world'));until(future.done,10)
            res=future.result()
            if not res.success:raise RuntimeError('missing physical entity '+name)
            return res.state.pose
        initial={g['id']:entity(g['id']) for g in goods}
        for g in goods:
            p=initial[g['id']].position
            assert math.dist([p.x,p.y,p.z],g['position'])<.008,(g['id'],'cargo unstable before sorting')
        goal=ExecuteSortTask.Goal(task_id=f'acceptance_{os.getpid()}',object_class='all',pick_station='warehouse',place_station='by_color',quantity=args.quantity,return_home=True)
        future=client.send_goal_async(goal);until(future.done,15);handle=future.result();assert handle.accepted,'goal rejected'
        future=handle.get_result_async();until(future.done,args.timeout)
        response=future.result();result=response.result
        if response.status!=4 or not result.success:
            diagnostic={}
            for name in [g['id'] for g in goods]+['landerpi','landerpi::link4','landerpi::link5','landerpi::end_effector_link']:
                try:
                    p=entity(name);diagnostic[name]=dict(position=[p.position.x,p.position.y,p.position.z],orientation=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
                except RuntimeError:pass
            Path(run+'_failure.json').write_text(json.dumps(dict(error_code=result.error_code,message=result.message,entities=diagnostic),indent=2))
        assert response.status==4 and result.success,(result.error_code,result.message)
        count=0;placements={};remaining=[]
        for g in goods:
            p=entity(g['id']);xyz=[p.position.x,p.position.y,p.position.z]
            destination=next((name for name in config['outputs'] if inside(config,name,xyz)),None)
            if destination:
                assert config['outputs'][destination]['color']==g['color'],(g['id'],'wrong color bin')
                count+=1;placements[g['id']]={'destination':destination,'position':xyz}
            else:
                assert math.dist(xyz,g['position'])<.008,(g['id'],'unsorted cargo displaced')
                remaining.append(g['id'])
        expected=args.quantity or len(goods)
        assert count==expected==result.completed_quantity,(count,expected,result.completed_quantity)
        assert not data['attached'].data,'cargo still attached'
        odom=data['odom'].pose.pose.position
        assert math.dist([odom.x,odom.y],config['start'])<.015,'robot not returned to start'
        state=dict(zip(data['joints'].name,data['joints'].position))
        assert max(abs(state[f'joint{i}']) for i in range(1,6))<.03,'arm not in transport pose'
        for color in config['colors']:
            points=[v['position'] for key,v in placements.items() if next(g['color'] for g in goods if g['id']==key)==color]
            for i,p in enumerate(points):
                for q in points[i+1:]:assert math.dist(p[:2],q[:2])>config['cargo_min_spacing'],'cargo spacing violated'
        summary=dict(success=True,total=len(goods),completed=count,remaining=remaining,placements=placements,duration_s=monotonic()-started,task_duration_s=result.total_duration.sec+result.total_duration.nanosec/1e9,motion_profile=args.motion_profile,**observed,csv=csv,log=str(log))
        Path(run+'.json').write_text(json.dumps(summary,indent=2));print(f'WAREHOUSE ACCEPTANCE PASSED {count}/{expected}; summary={run}.json',flush=True)
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
        stream.close()

if __name__=='__main__':main()
