"""Exercise the actual Action transport using in-process ROS nodes, no motion."""
import csv
import tempfile
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatus
from landerpi_task_interfaces.action import ExecuteSortTask
from landerpi_task_manager.task_manager_node import TaskManager


def main():
    rclpy.init()
    node = Node('framework_action_test')
    executor = SingleThreadedExecutor()
    server = TaskManager()
    directory = tempfile.TemporaryDirectory(prefix='landerpi_test_')
    from landerpi_experiment_tools.experiment_logger import ExperimentLogger
    csv_path = directory.name + '/tasks.csv'
    server.logger = ExperimentLogger(csv_path)
    executor.add_node(node)
    executor.add_node(server)
    client = ActionClient(node, ExecuteSortTask, '/task/execute_sort')

    def wait(future, timeout=10):
        executor.spin_until_future_complete(future, timeout_sec=timeout)
        assert future.done(), 'future timed out'
        return future.result()

    def goal(quantity=1, home=False):
        return ExecuteSortTask.Goal(task_id='framework_test', object_class='box',
                                    pick_station='pick_station', place_station='place_station',
                                    quantity=quantity, return_home=home)

    try:
        assert client.wait_for_server(timeout_sec=5)
        feedback = []
        handle = wait(client.send_goal_async(goal(2, True), feedback_callback=lambda msg: feedback.append(msg.feedback.current_state)))
        assert handle.accepted
        second = wait(client.send_goal_async(goal()))
        assert not second.accepted, 'concurrent task must be rejected'
        outcome = wait(handle.get_result_async())
        assert outcome.status == GoalStatus.STATUS_SUCCEEDED
        assert outcome.result.success and outcome.result.completed_quantity == 2
        assert feedback.count('PICK_OBJECT') == 2 and feedback.count('NAV_HOME') == 1
        print('PASS success, multi-object, return-home, concurrent rejection')
        # A second task verifies transition back to IDLE and cancel acceptance.
        handle = wait(client.send_goal_async(goal(100)))
        assert handle.accepted
        cancellation = wait(handle.cancel_goal_async())
        assert cancellation.goals_canceling
        outcome = wait(handle.get_result_async())
        assert outcome.status == GoalStatus.STATUS_CANCELED
        assert outcome.result.error_code == 502 and not outcome.result.success
        print('PASS cancellation and repeat task')
        server.timeout = 0.05
        handle = wait(client.send_goal_async(goal()))
        outcome = wait(handle.get_result_async())
        assert outcome.status == GoalStatus.STATUS_ABORTED
        assert outcome.result.error_code == 501
        print('PASS task timeout')
        server.mode = 'live'
        assert not wait(client.send_goal_async(goal())).accepted
        print('PASS live execution rejection')
        server.mode = 'dry_run'
        assert not wait(client.send_goal_async(goal(0))).accepted
        print('PASS invalid quantity rejection')
        with open(csv_path, newline='') as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 3 and all(row['execution_mode'] == 'dry_run' for row in rows)
        assert [row['final_error_code'] for row in rows] == ['0', '502', '501']
        print('PASS CSV terminal results and dry-run labels')
    finally:
        client.destroy()
        server.server.destroy()
        executor.remove_node(server)
        executor.remove_node(node)
        server.destroy_node()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        directory.cleanup()


if __name__ == '__main__':
    main()
