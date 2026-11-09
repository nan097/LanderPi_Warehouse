"""Regression for cancellation arriving before child-goal acceptance."""
import asyncio
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

path = Path(__file__).resolve().parents[1] / 'src/landerpi_simulation/scripts/sort_runtime.py'
spec = importlib.util.spec_from_file_location('simulation_runtime_under_test', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class DelayedActionClient:
    def __init__(self):
        self.active = False
        self.cancelled = False
        self.result = asyncio.get_running_loop().create_future()

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        async def accept():
            await asyncio.sleep(.05)
            self.active = True
            return SimpleNamespace(accepted=True, get_result_async=lambda:self.result,
                                   cancel_goal_async=self.cancel_goal_async)
        return asyncio.create_task(accept())

    def cancel_goal_async(self):
        async def cancel():
            await asyncio.sleep(.01)
            self.cancelled = True
            self.active = False
            self.result.set_result(SimpleNamespace(status=5, result=None))
            return SimpleNamespace()
        return asyncio.create_task(cancel())

class Harness:
    wait = module.Runtime.wait
    check = module.Runtime.check
    action = module.Runtime.action
    deadline = None

    def __init__(self, client):
        self.action_clients = {'/child':client}
        self.stops = []
        self.cmd = SimpleNamespace(publish=self.stops.append)

    async def pause(self, seconds=.005):
        await asyncio.sleep(seconds)

class CancellationTests(unittest.IsolatedAsyncioTestCase):
    async def verify_cancel(self, delay):
        client = DelayedActionClient()
        runtime = Harness(client)
        parent = SimpleNamespace(is_cancel_requested=False)
        async def request_cancel():
            await asyncio.sleep(delay)
            parent.is_cancel_requested = True
        cancellation = asyncio.create_task(request_cancel())
        with self.assertRaises(module.Interrupted):
            await runtime.action('/child', object(), parent, timeout=1)
        await cancellation
        await asyncio.sleep(.06)
        self.assertTrue(client.cancelled)
        self.assertFalse(client.active, 'accepted child must not remain moving')
        self.assertTrue(client.result.done(), 'wait for child terminal result')
        self.assertTrue(runtime.stops)

    async def test_cancel_during_send_handshake(self):
        await self.verify_cancel(.02)

    async def test_cancel_after_acceptance(self):
        await self.verify_cancel(.08)

if __name__ == '__main__':
    unittest.main()
