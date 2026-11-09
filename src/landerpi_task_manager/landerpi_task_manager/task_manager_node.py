"""Action endpoint for dry-run FSM validation. Live execution is fail-closed."""
import math
from time import monotonic, time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from landerpi_task_interfaces.action import ExecuteSortTask
from landerpi_experiment_tools.experiment_logger import ExperimentLogger
from .error_codes import ErrorCode
from .state_machine import State, StateMachine
from .task_context import TaskContext


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.declare_parameter('execution_mode', 'dry_run')
        self.declare_parameter('dry_run_step_sec', 0.1)
        self.declare_parameter('total_timeout_sec', 360.0)
        self.declare_parameter('csv_path', '/tmp/landerpi_experiments/tasks.csv')
        self.mode = self.get_parameter('execution_mode').value
        self.timeout = float(self.get_parameter('total_timeout_sec').value)
        period = float(self.get_parameter('dry_run_step_sec').value)
        if not math.isfinite(period) or period <= 0 or not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('step interval and timeout must be finite and positive')
        self.logger = ExperimentLogger(self.get_parameter('csv_path').value)
        self.fsm = StateMachine()
        self.fsm.initialize()
        self.get_logger().info('BOOT -> SELF_CHECK')
        if self.mode == 'dry_run':
            self.fsm.ready()
            self.get_logger().warning('DRY_RUN: IDLE; states are simulated, no motion or sensing')
        else:
            self.get_logger().error('Live backend NOT_IMPLEMENTED; goals will be rejected')
        self.busy = False
        self.active_goal = None
        self.pending = None
        self.task_context = None
        group = ReentrantCallbackGroup()
        self.server = ActionServer(self, ExecuteSortTask, '/task/execute_sort',
                                   execute_callback=self.execute,
                                   goal_callback=self.accept_goal,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=group)
        self.timer = self.create_timer(period, self.tick, callback_group=group)

    def accept_goal(self, goal):
        if (self.busy or self.mode != 'dry_run' or self.fsm.state != State.IDLE or
                not 1 <= goal.quantity <= 100 or
                not all((goal.task_id, goal.object_class, goal.pick_station, goal.place_station))):
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    async def execute(self, handle):
        self.task_context = TaskContext(handle.request.task_id, handle.request.quantity)
        self.started_at = time()
        self.pending = Future()
        self.fsm.start(handle.request.quantity, handle.request.return_home)
        self.active_goal = handle
        return await self.pending

    def tick(self):
        if self.active_goal is None:
            return
        handle = self.active_goal
        if handle.is_cancel_requested:
            self.finish(ErrorCode.TASK_CANCELLED, 'DRY_RUN cancelled')
            return
        if monotonic() - self.task_context.start_time >= self.timeout:
            self.finish(ErrorCode.TASK_TIMEOUT, 'DRY_RUN task timeout')
            return
        state, index = self.fsm.advance()
        if state == State.RETURN_TRANSPORT:
            self.task_context.completed_quantity = index
        feedback = ExecuteSortTask.Feedback()
        feedback.current_state = state.value
        feedback.progress = self.fsm.cursor / len(self.fsm.steps)
        feedback.current_index = index
        feedback.detail = 'DRY_RUN: simulated state; no hardware operation'
        handle.publish_feedback(feedback)
        self.get_logger().info(f'[DRY_RUN][{self.task_context.task_id}][{state.value}] index={index}')
        if state == State.FINISHED:
            self.finish(ErrorCode.SUCCESS, 'DRY_RUN sequence completed; hardware not executed')

    def finish(self, code, message):
        duration = monotonic() - self.task_context.start_time
        result = ExecuteSortTask.Result()
        result.success = code == ErrorCode.SUCCESS
        result.error_code = int(code)
        result.message = message
        result.completed_quantity = self.task_context.completed_quantity
        result.total_duration.sec = int(duration)
        result.total_duration.nanosec = int((duration % 1) * 1e9)
        try:
            self.logger.record(task_id=self.task_context.task_id, execution_mode=self.mode,
                               start_time=self.started_at, end_time=time(), total_duration_s=duration,
                               quantity=self.task_context.quantity,
                               completed_quantity=self.task_context.completed_quantity,
                               task_success=result.success, final_error_code=int(code),
                               final_error_message=message)
        except OSError as exc:
            result.success = False
            result.error_code = int(ErrorCode.CONFIG_INVALID)
            result.message = f'CSV write failed: {exc}'
        if code == ErrorCode.TASK_CANCELLED:
            self.active_goal.canceled()
        elif result.success:
            self.active_goal.succeed()
        else:
            self.active_goal.abort()
        if not result.success:
            self.fsm.stop()
        self.fsm.reset()
        pending = self.pending
        self.active_goal = None
        self.pending = None
        self.busy = False
        pending.set_result(result)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    try:
        rclpy.spin(node)
    finally:
        node.server.destroy()
        node.destroy_node()
        rclpy.shutdown()
