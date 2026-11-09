#!/usr/bin/env python3
"""Optional visual preview using simulated Action feedback, never hardware."""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

class MotionDemo(Node):

    def __init__(self):
        super().__init__('simulation_motion_demo')
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joints = {}
        self.ready = set()
        self.subs = [self.create_subscription(JointState, '/joint_states', self.joint_callback, qos_profile_sensor_data)]
        for name in ('arm_controller', 'gripper_controller'):
            self.subs.append(self.create_subscription(JointTrajectoryControllerState, f'/{name}/controller_state', lambda msg, key=name: self.ready.add(key), qos_profile_sensor_data))
        self.phase = 0
        self.future = None
        self.goal = None
        self.motion_start = None
        self.baseline = None
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info('SIMULATION PREVIEW: waiting for active controllers; stop demo before manual planning')

    def joint_callback(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def send(self, client, names, positions):
        request = FollowJointTrajectory.Goal()
        request.trajectory.joint_names = names
        point = JointTrajectoryPoint(positions=positions)
        point.time_from_start.sec = 3
        request.trajectory.points = [point]
        self.future = client.send_goal_async(request)

    def tick(self):
        names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        if len(self.ready) < 2 or not all((name in self.joints for name in names)):
            return
        if self.baseline is None:
            self.baseline = [self.joints[name] for name in names]
        if self.future is not None:
            if not self.future.done():
                return
            result = self.future.result()
            if self.goal is None:
                if not result.accepted:
                    self.get_logger().error('Preview goal rejected; stopping preview')
                    self.timer.cancel()
                    return
                self.goal = result
                self.future = self.goal.get_result_async()
                return
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error('Preview trajectory failed; stopping preview')
                self.timer.cancel()
                return
            self.future = None
            self.goal = None
            self.phase = (self.phase + 1) % 6
        if self.phase in (0, 3):
            positions = list(self.baseline)
            positions[0] = self.baseline[0] + (0.35 if self.phase == 0 else -0.35)
            self.send(self.arm, names, positions)
            self.get_logger().info(f'PREVIEW arm joint1 -> {positions[0]:.2f}')
        elif self.phase in (1, 4):
            position = -0.5 if self.phase == 1 else 0.0
            self.send(self.gripper, ['r_joint'], [position])
            self.get_logger().info(f'PREVIEW gripper -> {position:.2f}')
        else:
            now = self.get_clock().now().nanoseconds
            if self.motion_start is None:
                self.motion_start = now
                self.get_logger().info('PREVIEW base lateral motion')
            velocity = Twist()
            if now - self.motion_start < 2000000000:
                velocity.linear.y = 0.08 if self.phase == 2 else -0.08
            else:
                self.motion_start = None
                self.phase = (self.phase + 1) % 6
            self.cmd.publish(velocity)

def main(args=None):
    rclpy.init(args=args)
    node = MotionDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd.publish(Twist())
            if node.goal is not None:
                node.goal.cancel_goal_async()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()
