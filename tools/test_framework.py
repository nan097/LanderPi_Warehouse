"""Structural and pure-code checks for the foundation, without hardware."""
import ast
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src/landerpi_task_manager'),
                str(ROOT / 'src/landerpi_perception')]
from landerpi_task_manager.state_machine import State, StateMachine, task_steps
from landerpi_perception.depth_localizer import project_pixel


class FrameworkTests(unittest.TestCase):
    def test_packages_and_syntax(self):
        packages = list((ROOT / 'src').glob('landerpi_*/package.xml'))
        # Includes the retained description package plus the 12 new packages.
        self.assertEqual(len(packages), 13)
        for manifest in packages:
            ET.parse(manifest)
        for base in (ROOT / 'src').glob('landerpi_*'):
            for path in base.rglob('*.py'):
                ast.parse(path.read_text(), filename=str(path))

    def test_interface_contract(self):
        import re
        doc = (ROOT / 'deep-research-report.md').read_text()
        base = ROOT / 'src/landerpi_task_interfaces'
        files = list(base.glob('action/*.action')) + list(base.glob('msg/*.msg')) + list(base.glob('srv/*.srv'))
        self.assertEqual(len(files), 11)
        for path in files:
            pattern = r'\*\*`' + re.escape(path.name) + r'`\*\*\s*```text\n(.*?)```'
            self.assertEqual(path.read_text(), re.search(pattern, doc, re.S).group(1))

    def test_order_and_quantity(self):
        steps = task_steps(2, True)
        self.assertEqual(sum(state == State.NAV_HOME for state, _ in steps), 1)
        self.assertFalse(any(state == State.NAV_HOME for state, _ in task_steps(2, False)))
        self.assertEqual(sum(state == State.PICK_OBJECT for state, _ in steps), 2)
        self.assertEqual(steps[-2:], [(State.NAV_HOME, 2), (State.FINISHED, 2)])
        for index in (1, 2):
            cycle = [state for state, n in steps if n == index]
            self.assertLess(cycle.index(State.ARM_TRANSPORT), cycle.index(State.NAV_TO_PLACE))

    def test_busy_and_reset(self):
        fsm = StateMachine()
        with self.assertRaises(ValueError):
            fsm.start(1)
        fsm.initialize(); fsm.ready(); fsm.start(1)
        while fsm.cursor < len(fsm.steps):
            fsm.advance()
        fsm.reset()
        self.assertEqual(fsm.state, State.IDLE)
        with self.assertRaises(ValueError):
            task_steps(0)

    def test_depth_refuses_invalid_data(self):
        for value in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                project_pixel(320, 240, value, 500, 500, 320, 240)
        self.assertEqual(project_pixel(320, 240, 1, 500, 500, 320, 240), (0, 0, 1))
        with self.assertRaises(ValueError):
            project_pixel(320, 240, 1, 0, 500, 320, 240)


if __name__ == '__main__':
    unittest.main()
