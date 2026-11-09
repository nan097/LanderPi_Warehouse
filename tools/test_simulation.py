"""Launch headless Gazebo and verify simulated feedback and controllers."""
import os
import signal
import subprocess
import time
from pathlib import Path

# Isolate from any already running ROS/Gazebo session.
os.environ['ROS_DOMAIN_ID'] = '74'
os.environ['ROS_LOG_DIR'] = '/tmp/landerpi_sim_ros_logs'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint


def main():
    log = Path(f'/tmp/landerpi_simulation_smoke_{os.getpid()}.log')
    stream = log.open('w')
    process = subprocess.Popen(['ros2', 'launch', 'landerpi_bringup', 'simulation.launch.py',
                                'gui:=false', 'rviz:=false', 'ros_domain_id:=74'], stdout=stream,
                               stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('simulation_smoke_test')
    data = {}
    subscriptions = [node.create_subscription(kind, topic, lambda msg, key=key: data.__setitem__(key, msg), qos_profile_sensor_data)
                     for kind, topic, key in [(JointState, '/joint_states', 'joints'),
                                               (LaserScan, '/scan', 'scan'),
                                               (Odometry, '/odom', 'odom'),
                                               (Clock, '/clock', 'clock'),
                                               (JointTrajectoryControllerState, '/arm_controller/controller_state', 'arm_active'),
                                               (JointTrajectoryControllerState, '/gripper_controller/controller_state', 'gripper_active')]]
    cmd = node.create_publisher(Twist, '/cmd_vel', 10)
    clients = []
    def until(predicate, timeout=60):
        deadline = time.monotonic() + timeout
        while not predicate():
            if process.poll() is not None:
                raise AssertionError(f'launch exited: {process.returncode}; see {log}')
            if time.monotonic() > deadline:
                raise AssertionError(f'timeout; received {list(data)}; see {log}')
            rclpy.spin_once(node, timeout_sec=0.1)
    def action(name, joints, positions):
        client = ActionClient(node, FollowJointTrajectory, name)
        clients.append(client)
        until(client.server_is_ready)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        point = JointTrajectoryPoint(positions=positions)
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        until(future.done, 10)
        handle = future.result()
        assert handle.accepted
        result = handle.get_result_async()
        until(result.done, 20)
        outcome = result.result()
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED, outcome
        assert outcome.result.error_code == 0, outcome.result
    try:
        until(lambda: all(key in data for key in ('joints', 'odom', 'scan', 'clock')), 80)
        assert len(data['scan'].ranges) == 360
        assert all(name in data['joints'].name for name in ['joint1','joint2','joint3','joint4','joint5','r_joint'])
        until(lambda: 'arm_active' in data and 'gripper_active' in data, 60)
        print('PASS Gazebo clock, lidar, odometry and six controlled-joint feedback', flush=True)
        names = ['joint1','joint2','joint3','joint4','joint5']
        current = dict(zip(data['joints'].name, data['joints'].position))
        positions = [current[name] for name in names]
        positions[0] += 0.15
        action('/arm_controller/follow_joint_trajectory', names, positions)
        until(lambda: abs(dict(zip(data['joints'].name, data['joints'].position))['joint1'] - positions[0]) < 0.03, 5)
        print('PASS arm trajectory execution and observed position', flush=True)
        action('/gripper_controller/follow_joint_trajectory', ['r_joint'], [-0.4])
        print('PASS gripper trajectory execution', flush=True)
        initial_y = data['odom'].pose.pose.position.y
        first_clock = data['clock'].clock.sec + data['clock'].clock.nanosec / 1e9
        velocity = Twist()
        velocity.linear.y = 0.1
        deadline = time.monotonic() + 10
        while data['clock'].clock.sec + data['clock'].clock.nanosec / 1e9 - first_clock < 1.0:
            assert time.monotonic() < deadline
            cmd.publish(velocity)
            rclpy.spin_once(node, timeout_sec=0.05)
        cmd.publish(Twist())
        delta_y = data['odom'].pose.pose.position.y - initial_y
        assert delta_y > 0.04, delta_y
        print(f'PASS holonomic lateral base motion: delta_y={delta_y:.3f} m', flush=True)
        plan_client = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        until(plan_client.service_is_ready, 30)
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = 'arm'
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.2
        motion.max_acceleration_scaling_factor = 0.2
        motion.start_state.is_diff = True
        current = dict(zip(data['joints'].name, data['joints'].position))
        constraints = Constraints()
        for name in names:
            constraints.joint_constraints.append(JointConstraint(joint_name=name, position=current[name] + (0.1 if name == 'joint1' else 0.0), tolerance_above=0.005, tolerance_below=0.005, weight=1.0))
        motion.goal_constraints = [constraints]
        future = plan_client.call_async(request)
        until(future.done, 15)
        response = future.result().motion_plan_response
        assert response.error_code.val == 1, f'MoveIt error={response.error_code.val}; see {log}'
        assert response.trajectory.joint_trajectory.points
        print('PASS MoveIt collision-aware joint-space planning', flush=True)
        print(f'Log: {log}', flush=True)
    finally:
        cmd.publish(Twist())
        for client in clients:
            client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.kill(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        stream.close()


if __name__ == '__main__':
    main()
