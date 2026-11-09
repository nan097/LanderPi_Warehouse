#!/usr/bin/env python3
"""Physical gamepad, exclusive base command routing and warehouse task trigger."""
from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from landerpi_task_interfaces.action import ExecuteSortTask
from landerpi_task_interfaces.msg import RobotStatus


class WarehouseJoystick(Node):
    def __init__(self):
        super().__init__('warehouse_joystick')
        for key, value in dict(start_button=7, back_button=6, linear_axis=1,
                               angular_axis=0, max_linear=0.2, max_angular=0.8,
                               deadzone=0.12, joy_timeout=0.6, quantity=0).items():
            self.declare_parameter(key, value)
        self.mode = 'manual'
        self.buttons = None
        self.joy_time = self.auto_time = 0.0
        self.manual = Twist()
        self.auto = Twist()
        self.ready = False
        self.goal_handle = None
        self.pending = None
        self.cancel_sent = False
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.enable = self.create_publisher(Bool, '/warehouse/operator_enabled', 10)
        self.mode_pub = self.create_publisher(String, '/warehouse/control_mode', 10)
        self.client = ActionClient(self, ExecuteSortTask, '/task/execute_sort')
        self.create_subscription(Joy, '/joy', self.joy, qos_profile_sensor_data)
        self.create_subscription(Twist, '/warehouse/auto_cmd_vel', self.velocity, 10)
        self.create_subscription(RobotStatus, '/robot/status', self.status, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('MANUAL: left stick drives base; START begins sorting; BACK cancels. Keep gamepad connected.')

    def param(self, name):
        return self.get_parameter(name).value

    def status(self, msg):
        self.ready = msg.robot_ready

    def velocity(self, msg):
        self.auto, self.auto_time = msg, monotonic()

    def joy(self, msg):
        self.joy_time = monotonic()
        def pressed(index):
            return 0 <= index < len(msg.buttons) and bool(msg.buttons[index])
        def edge(index):
            return self.buttons is not None and pressed(index) and not (
                index < len(self.buttons) and self.buttons[index])
        self.manual = Twist()
        for field, axis, limit in [('linear', 'linear_axis', 'max_linear'),
                                   ('angular', 'angular_axis', 'max_angular')]:
            index = self.param(axis)
            value = msg.axes[index] if 0 <= index < len(msg.axes) else 0.0
            value = max(-1.0, min(1.0, value))
            value = value * self.param(limit) if abs(value) > self.param('deadzone') else 0.0
            if field == 'linear':
                self.manual.linear.x = value
            else:
                self.manual.angular.z = value
        if edge(self.param('back_button')):
            self.stop()
        elif edge(self.param('start_button')) and self.mode == 'manual':
            if self.ready and self.client.server_is_ready():
                self.mode = 'arming'
                self.arm_time = monotonic()
                self.auto = Twist()
                self.cmd.publish(Twist())
            else:
                self.get_logger().warning('Task is not ready; press START again after readiness.')
        self.buttons = list(msg.buttons)

    def stop(self):
        if self.mode != 'manual':
            self.mode = 'stopping'
        self.manual = Twist()
        self.cmd.publish(Twist())

    def sent(self, future):
        self.pending = None
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.goal_handle = None
                self.mode = 'manual'
                self.get_logger().warning('Warehouse goal rejected.')
                return
            self.goal_handle.get_result_async().add_done_callback(self.finished)
            if self.mode != 'stopping':
                self.mode = 'automatic'
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.mode = 'manual'

    def finished(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f'Task finished: {result.success}: {result.message}')
        except Exception as exc:
            self.get_logger().error(str(exc))
        self.goal_handle = None
        self.cancel_sent = False
        self.mode = 'manual'
        self.manual = Twist()
        self.cmd.publish(Twist())

    def tick(self):
        now = monotonic()
        connected = now - self.joy_time < self.param('joy_timeout')
        if not connected:
            self.stop()
            self.buttons = None
        self.enable.publish(Bool(data=connected and self.mode in ('arming', 'automatic')))
        if self.mode == 'arming' and now - self.arm_time > 0.3:
            goal = ExecuteSortTask.Goal(task_id=f'gamepad_{int(now * 1000)}',
                object_class='all', pick_station='warehouse', place_station='by_color',
                quantity=self.param('quantity'), return_home=True)
            self.mode = 'automatic'
            self.pending = self.client.send_goal_async(goal)
            self.pending.add_done_callback(self.sent)
        if self.mode == 'stopping':
            if self.goal_handle and not self.cancel_sent:
                self.goal_handle.cancel_goal_async()
                self.cancel_sent = True
            elif self.goal_handle is None and self.pending is None:
                self.mode = 'manual'
        command = Twist()
        if connected and self.mode == 'manual':
            command = self.manual
        elif connected and self.mode == 'automatic' and now - self.auto_time < 0.3:
            command = self.auto
        self.cmd.publish(command)
        self.mode_pub.publish(String(data=self.mode))


def main():
    rclpy.init()
    node = WarehouseJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
