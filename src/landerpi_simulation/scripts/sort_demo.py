#!/usr/bin/env python3
"""Submit one real task once the full simulation reports ready."""
from time import monotonic
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from landerpi_task_interfaces.action import ExecuteSortTask
from landerpi_task_interfaces.msg import RobotStatus

def main():
    rclpy.init()
    node = Node('sort_demo_client')
    status = {}
    node.create_subscription(RobotStatus, '/robot/status', lambda msg: status.update(robot=msg), 10)
    client = ActionClient(node, ExecuteSortTask, '/task/execute_sort')

    def until(predicate, timeout):
        deadline = monotonic() + timeout
        while not predicate():
            if monotonic() > deadline:
                raise RuntimeError('full simulation did not become ready or task exceeded deadline')
            rclpy.spin_once(node, timeout_sec=0.1)
    try:
        until(lambda : client.server_is_ready() and 'robot' in status and status['robot'].robot_ready, 180)
        goal = ExecuteSortTask.Goal(task_id=f'demo_{int(monotonic())}', object_class='red_box', pick_station='pick', place_station='place', quantity=1, return_home=True)
        future = client.send_goal_async(goal, feedback_callback=lambda msg: node.get_logger().info('[SORT] ' + msg.feedback.current_state))
        until(future.done, 10)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('task rejected')
        future = handle.get_result_async()
        try:
            until(future.done, 600)
        except Exception:
            cancellation = handle.cancel_goal_async()
            until(cancellation.done, 5)
            raise
        result = future.result()
        node.get_logger().info(f'[SORT RESULT] success={result.result.success}; {result.result.message}')
        if not result.result.success:
            raise RuntimeError(result.result.message)
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
