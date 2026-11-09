"""Run with sourced ROS workspace: python3 -m unittest discover -s this_directory."""
import importlib.util
from pathlib import Path
from time import monotonic
import unittest
import sys
from types import SimpleNamespace
from unittest.mock import Mock
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

spec = importlib.util.spec_from_file_location('warehouse_joystick', Path(__file__).parents[1]/'scripts/warehouse_joystick.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ControlsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = module.WarehouseJoystick()
        self.node.cmd = Mock()
        self.node.enable = Mock()
        self.original_client = self.node.client
        self.node.client = Mock()
        self.node.client.server_is_ready.return_value = True

    def tearDown(self):
        self.original_client.destroy()
        self.node.destroy_node()

    def joy(self, forward=0., turn=0., button=None):
        msg = Joy(axes=[turn, forward], buttons=[0]*8)
        if button is not None:
            msg.buttons[button] = 1
        self.node.joy(msg)

    def test_manual_motion_and_deadzone(self):
        for forward, turn in [(1.,0.),(-1.,0.),(0.,1.),(0.,-1.),(.05,.05)]:
            self.joy(forward, turn)
            self.node.tick()
            cmd = self.node.cmd.publish.call_args.args[0]
            self.assertAlmostEqual(cmd.linear.x, forward*.2 if abs(forward)>.12 else 0.)
            self.assertAlmostEqual(cmd.angular.z, turn*.8 if abs(turn)>.12 else 0.)
        self.node.client.send_goal_async.assert_not_called()

    def test_start_edge_readiness_and_no_repeat(self):
        self.joy(button=7)  # held at connection must not start
        self.assertEqual(self.node.mode, 'manual')
        self.joy(); self.joy(button=7)  # not ready
        self.assertEqual(self.node.mode, 'manual')
        self.node.ready = True
        self.joy(); self.joy(button=7)
        self.assertEqual(self.node.mode, 'arming')
        self.node.arm_time = monotonic()-1.
        self.node.tick()
        self.node.client.send_goal_async.assert_called_once()
        self.joy(button=7); self.node.tick()
        self.node.client.send_goal_async.assert_called_once()

    def test_manual_blocks_navigation_and_stale_joy_stops(self):
        auto = Twist(); auto.linear.x = .9
        self.node.velocity(auto)
        self.joy(); self.node.tick()
        self.assertEqual(self.node.cmd.publish.call_args.args[0].linear.x, 0.)
        self.node.mode = 'automatic'
        self.node.goal_handle = Mock()
        self.node.tick()
        self.assertEqual(self.node.cmd.publish.call_args.args[0].linear.x, .9)
        self.node.joy_time = monotonic()-2.
        self.node.tick()
        self.assertEqual(self.node.cmd.publish.call_args.args[0].linear.x, 0.)
        self.node.goal_handle.cancel_goal_async.assert_called_once()
        self.assertFalse(self.node.enable.publish.call_args.args[0].data)

    def test_back_cancels_and_waits_before_manual(self):
        self.joy()
        self.node.mode = 'automatic'
        self.node.goal_handle = Mock()
        self.joy(button=6); self.node.tick()
        self.assertEqual(self.node.mode, 'stopping')
        self.node.goal_handle.cancel_goal_async.assert_called_once()
        self.node.tick()
        self.node.goal_handle.cancel_goal_async.assert_called_once()


class AuthorizationTest(unittest.TestCase):
    def test_runtime_requires_fresh_operator_authorization(self):
        sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
        from warehouse_runtime import WarehouseRuntime
        from rclpy.action import GoalResponse
        state = SimpleNamespace(busy=set(), fault='', ready=True, require_operator=True,
            operator_enabled=False, operator_stamp=monotonic(),
            config={'colors':['red']}, goods=[{}])
        goal = SimpleNamespace(task_id='test', object_class='all',
            pick_station='warehouse', place_station='by_color', quantity=0)
        accept = lambda: WarehouseRuntime.accept(state, '/task/execute_sort', goal)
        self.assertEqual(accept(), GoalResponse.REJECT)
        state.require_operator = False
        self.assertEqual(accept(), GoalResponse.ACCEPT)
        state.busy.clear()
        state.require_operator = True
        state.operator_enabled = True
        state.operator_stamp = monotonic()-2
        self.assertEqual(accept(), GoalResponse.REJECT)
        state.operator_stamp = monotonic()
        self.assertEqual(accept(), GoalResponse.ACCEPT)


if __name__ == '__main__':
    unittest.main()
