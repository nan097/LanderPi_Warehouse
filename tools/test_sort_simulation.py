"""Physical acceptance: sorting, grasp repeats, cancellation and missing target.

SetEntityState is used only to arrange independent trials and inject failures.
The production task never teleports the box and detects it from rendered RGB-D.
"""
import argparse
import csv
import math
import os
import random
import signal
import subprocess
import time
from pathlib import Path
os.environ['ROS_DOMAIN_ID'] = '75'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from landerpi_task_interfaces.action import ExecuteSortTask, DetectObject, PickObject, PlaceObject, MoveNamedPose, NavigateStation, FineAlign
from landerpi_task_interfaces.msg import RobotStatus
from landerpi_task_interfaces.srv import ResetFaults
from gazebo_msgs.srv import GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
import tf2_ros


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--grasp-runs', type=int, default=0)
    parser.add_argument('--fault-check', action='store_true')
    parser.add_argument('--debug-failure', action='store_true')
    args = parser.parse_args()
    assert 1 <= args.runs <= 100 and 0 <= args.grasp_runs <= 100
    log = Path(f'/tmp/landerpi_sort_test_{os.getpid()}.log')
    csv_path = Path(f'/tmp/landerpi_sort_acceptance_{os.getpid()}.csv')
    random_positions = random.Random(42)
    stream = log.open('w')
    process = subprocess.Popen(['bash', 'tools/run_sort_simulation.sh', 'gui:=false', 'rviz:=false',
                                f'csv_path:={csv_path}'], stdout=stream, stderr=subprocess.STDOUT,
                               start_new_session=True)
    try:
        rclpy.init()
        node = Node('sort_integration_test')
    except Exception:
        os.kill(process.pid, signal.SIGINT)
        try: process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait()
        stream.close()
        raise
    data = {}
    transform_buffer = tf2_ros.Buffer()
    transform_listener = tf2_ros.TransformListener(transform_buffer, node)
    for kind, topic, key in [(RobotStatus, '/robot/status', 'status'), (Odometry, '/odom', 'odom'),
                            (Clock, '/clock', 'clock'), (Bool, '/simulation/object_attached', 'attached'),
                            (JointState, '/joint_states', 'joints')]:
        node.create_subscription(kind, topic, lambda msg, k=key:data.update({k:msg}), qos_profile_sensor_data)
    cmd = node.create_publisher(Twist, '/cmd_vel', 10)
    clients = {}
    for kind, name in [(ExecuteSortTask, '/task/execute_sort'), (DetectObject, '/perception/detect_object'),
                       (PickObject, '/manipulation/pick_object'), (PlaceObject, '/manipulation/place_object'),
                       (MoveNamedPose, '/manipulation/move_named_pose'), (NavigateStation, '/navigation/navigate_station'),
                       (FineAlign, '/fine_alignment/align')]:
        clients[name] = ActionClient(node, kind, name)

    last_position_log = [0.]

    def until(predicate, timeout=90):
        deadline = time.monotonic()+timeout
        while not predicate():
            if 'process has died' in log.read_text(errors='replace') and any('sort_runtime.py' in line and 'process has died' in line for line in log.read_text(errors='replace').splitlines()):
                raise AssertionError(f'simulation runtime exited; {log}')
            if process.poll() is not None:
                raise AssertionError(f'launch exited; {log}')
            if time.monotonic()>deadline:
                raise AssertionError(f'timeout; status={data.get("status")}; {log}')
            rclpy.spin_once(node, timeout_sec=.05)
            if 'odom' in data and time.monotonic()-last_position_log[0] > 10:
                p = data['odom'].pose.pose.position
                try:
                    t = transform_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                    print(f'ODOM x={p.x:.3f} y={p.y:.3f}; TF x={t.transform.translation.x:.3f} y={t.transform.translation.y:.3f}', flush=True)
                except tf2_ros.TransformException:
                    print(f'ODOM x={p.x:.3f} y={p.y:.3f}; TF unavailable', flush=True)
                last_position_log[0] = time.monotonic()

    def service(kind, name, request):
        client = node.create_client(kind, name)
        try:
            until(client.service_is_ready)
            future = client.call_async(request)
            until(future.done, 20)
            return future.result()
        finally:
            node.destroy_client(client)

    def action(name, goal, callback=None, expected_success=True):
        client = clients[name]
        until(client.server_is_ready)
        future = client.send_goal_async(goal, feedback_callback=callback)
        until(future.done, 10)
        handle = future.result()
        assert handle.accepted, f'{name} rejected'
        future = handle.get_result_async()
        until(future.done, 600)
        result = future.result()
        if expected_success:
            assert result.status == 4 and result.result.success, result
        return result

    def ready():
        until(lambda:'status' in data and data['status'].robot_ready, 180)

    def arrange(x=.67, y=0.):
        state = EntityState(name='box', reference_frame='world')
        state.pose.position.x=x; state.pose.position.y=y; state.pose.position.z=.185
        state.pose.orientation.w=1.
        assert service(SetEntityState, '/gazebo/set_entity_state', SetEntityState.Request(state=state)).success
        begin = data['clock'].clock.sec + data['clock'].clock.nanosec/1e9
        until(lambda:data['clock'].clock.sec+data['clock'].clock.nanosec/1e9-begin>.4, 10)

    def entity():
        reply = service(GetEntityState, '/gazebo/get_entity_state',
                        GetEntityState.Request(name='box', reference_frame='world'))
        assert reply.success
        return reply.state.pose.position

    def task_goal(tag, quantity=1):
        return ExecuteSortTask.Goal(task_id=f'{tag}_{os.getpid()}',object_class='red_box',
                                   pick_station='pick',place_station='place',quantity=quantity,return_home=True)

    def reset_fault():
        reply = service(ResetFaults, '/robot/reset_faults', ResetFaults.Request(reason='acceptance test recovery'))
        assert reply.success, reply
        ready()

    try:
        ready()
        until(lambda:all(k in data for k in ['clock','odom','joints','attached']))
        def brake(name):
            assert service(Trigger,name,Trigger.Request()).success
        def lateral(seconds):
            initial = data['clock'].clock.sec + data['clock'].clock.nanosec/1e9
            deadline = time.monotonic()+15
            velocity=Twist();velocity.linear.y=.05
            while data['clock'].clock.sec+data['clock'].clock.nanosec/1e9-initial<seconds:
                assert time.monotonic()<deadline
                cmd.publish(velocity);rclpy.spin_once(node,timeout_sec=.03)
            cmd.publish(Twist())
        brake('/simulation/hold_base')
        before = data['odom'].pose.pose.position.y
        lateral(.6)
        assert abs(data['odom'].pose.pose.position.y-before)<.003, 'brake failed to hold'
        brake('/simulation/release_base')
        before=data['odom'].pose.pose.position.y;lateral(.6)
        delta=data['odom'].pose.pose.position.y-before
        assert delta>.015,f'brake remained locked after release: {delta}'
        print(f'PASS base brake hold/release delta_y={delta:.4f}',flush=True)

        # Planning-only Action must not send a trajectory to the arm.
        before=dict(zip(data['joints'].name,data['joints'].position))
        action('/manipulation/move_named_pose',MoveNamedPose.Goal(name='home',velocity_scale=.3,
                                                                acceleration_scale=.3,execute=False))
        after=dict(zip(data['joints'].name,data['joints'].position))
        assert max(abs(after[n]-before[n]) for n in ['joint1','joint2','joint3','joint4','joint5'])<.015
        print('PASS named pose planning without execution',flush=True)

        if args.fault_check:
            client=clients['/task/execute_sort']
            future=client.send_goal_async(task_goal('invalid',2));until(future.done,10)
            assert not future.result().accepted
            stages=[]
            future=client.send_goal_async(task_goal('cancel'),feedback_callback=lambda msg:stages.append(msg.feedback.current_state))
            until(future.done,10);handle=future.result();assert handle.accepted
            until(lambda:'NAVIGATE_PICK' in stages,45)
            duplicate=client.send_goal_async(task_goal('duplicate'));until(duplicate.done,10)
            assert not duplicate.result().accepted
            cancel=handle.cancel_goal_async();until(cancel.done,10);assert cancel.result().goals_canceling
            future=handle.get_result_async();until(future.done,15);result=future.result()
            assert result.status==5 and result.result.error_code==502,result
            assert not any(s in stages for s in ['DETECT_OBJECT','PICK_OBJECT','NAVIGATE_PLACE'])
            reset_fault()
            print('PASS invalid quantity, task exclusivity and cancellation propagation',flush=True)

        grasp_success=0
        alignment = action('/fine_alignment/align', FineAlign.Goal(station='home', mode=1, xy_tolerance=.006, yaw_tolerance=.015))
        assert alignment.result.final_xy_error < .006
        print('PASS public fine alignment', flush=True)
        action('/manipulation/move_named_pose', MoveNamedPose.Goal(name='transport', execute=True, velocity_scale=.4, acceleration_scale=.4))

        if args.grasp_runs:
            action('/navigation/navigate_station',NavigateStation.Goal(station='pick'))
            for run in range(args.grasp_runs):
                x=.67+random_positions.uniform(-.006, .006);y=random_positions.uniform(-.006, .006)
                arrange(x,y)
                detection=action('/perception/detect_object',DetectObject.Goal(target_class='red_box',output_frame='map',min_confidence=.5)).result
                actual=entity();observed=detection.pose.pose.position
                assert math.dist([actual.x,actual.y,actual.z],[observed.x,observed.y,observed.z])<.025
                action('/manipulation/pick_object',PickObject.Goal(task_id=f'grasp_{run}',object_id='box',
                    object_class='red_box',object_pose=detection.pose,object_size=Vector3(x=.04,y=.04,z=.05),update_planning_scene=True))
                assert data['attached'].data
                target=PoseStamped();target.header.frame_id='map';target.pose.position.x=x
                target.pose.position.y=y;target.pose.position.z=.185;target.pose.orientation.w=1.
                action('/manipulation/place_object',PlaceObject.Goal(task_id=f'grasp_{run}',object_id='box',
                    station='pick',place_pose=target,update_planning_scene=True))
                until(lambda:not data['attached'].data,5)
                grasp_success+=1
                print(f'PASS standalone RGB-D pick/place {run+1}/{args.grasp_runs}',flush=True)
            action('/navigation/navigate_station',NavigateStation.Goal(station='home'))

        for run in range(args.runs):
            arrange(.67+random_positions.uniform(-.006, .006), random_positions.uniform(-.006, .006))
            result=action('/task/execute_sort',task_goal(f'integration_{run}'),
                          callback=lambda msg:print('STATE',msg.feedback.current_state,flush=True)).result
            assert result.completed_quantity==1
            pos=entity()
            assert abs(pos.x-.67)<.09 and abs(pos.y-1.5)<.09 and abs(pos.z-.185)<.04,pos
            until(lambda:not data['attached'].data,5)
            base=data['odom'].pose.pose.position
            assert math.hypot(base.x,base.y)<.015,base
            joints=dict(zip(data['joints'].name,data['joints'].position))
            assert max(abs(joints[n]) for n in ['joint1','joint2','joint3','joint4','joint5'])<.03
            scene=service(GetPlanningScene,'/get_planning_scene',
                          GetPlanningScene.Request(components=PlanningSceneComponents(components=1023))).scene
            assert not scene.robot_state.attached_collision_objects
            assert any(obj.id=='box' for obj in scene.world.collision_objects)
            acm=scene.allowed_collision_matrix
            for table in ['pick_table','place_table']:
                if 'box' in acm.entry_names and table in acm.entry_names:
                    assert not acm.entry_values[acm.entry_names.index('box')].enabled[acm.entry_names.index(table)]
                defaults = dict(zip(acm.default_entry_names, acm.default_entry_values))
                assert not defaults.get('box', False) and not defaults.get(table, False)
            print(f'PASS full sorting {run+1}/{args.runs}; final box={pos}; duration={result.total_duration.sec}s',flush=True)

        if args.fault_check:
            arrange(5.,0.)
            result=action('/task/execute_sort',task_goal('missing_target'),expected_success=False)
            assert result.status==6 and result.result.error_code==301 and result.result.completed_quantity==0,result
            reset_fault();arrange()
            print('PASS missing RGB target, bounded retries and fault reset',flush=True)
        rows=list(csv.DictReader(csv_path.open()))
        successful=[row for row in rows if row['task_id'].startswith('integration_') and row['success']=='True']
        assert len(successful)==args.runs
        print(f'ACCEPTANCE PASSED: full_tasks={args.runs}/{args.runs}, additional_grasps={grasp_success}/{args.grasp_runs}',flush=True)
        print(f'CSV: {csv_path}\nLog: {log}',flush=True)
    except Exception:
        if args.debug_failure:
            print(f'DEBUG: retaining failed simulation for 240 s; launch={process.pid}; log={log}', flush=True)
            deadline = time.monotonic()+240
            while time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=.1)
        raise
    finally:
        try:
            if rclpy.ok():cmd.publish(Twist())
            for client in clients.values():client.destroy()
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
        finally:
            if process.poll() is None:
                os.kill(process.pid,signal.SIGINT)
                try:process.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            stream.close()
            print('Log:',log,flush=True)

if __name__=='__main__':main()
