#!/usr/bin/env python3
"""Run warehouse sorting until all remaining cargo is placed by color."""
from time import monotonic
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from landerpi_task_interfaces.action import ExecuteSortTask
from landerpi_task_interfaces.msg import RobotStatus


def main():
    rclpy.init();node=Node('warehouse_demo_client');node.declare_parameter('quantity',0)
    status={};node.create_subscription(RobotStatus,'/robot/status',lambda m:status.update(robot=m),10)
    client=ActionClient(node,ExecuteSortTask,'/task/execute_sort')
    def until(predicate,timeout):
        deadline=monotonic()+timeout
        while not predicate():
            if monotonic()>deadline:raise RuntimeError('warehouse readiness or task timeout')
            rclpy.spin_once(node,timeout_sec=.1)
    try:
        until(lambda:client.server_is_ready() and 'robot' in status and status['robot'].robot_ready,240)
        goal=ExecuteSortTask.Goal(task_id=f'warehouse_{int(monotonic())}',object_class='all',pick_station='warehouse',place_station='by_color',quantity=node.get_parameter('quantity').value,return_home=True)
        future=client.send_goal_async(goal,feedback_callback=lambda m:node.get_logger().info(f'[WAREHOUSE FEEDBACK] {m.feedback.current_state} {m.feedback.detail}'))
        until(future.done,15);handle=future.result()
        if not handle.accepted:raise RuntimeError('warehouse goal rejected')
        future=handle.get_result_async()
        try:until(future.done,8000)
        except Exception:
            cancel=handle.cancel_goal_async();until(cancel.done,10);raise
        result=future.result();node.get_logger().info(f'[WAREHOUSE RESULT] success={result.result.success}; completed={result.result.completed_quantity}; {result.result.message}')
        if not result.result.success:raise RuntimeError(result.result.message)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
