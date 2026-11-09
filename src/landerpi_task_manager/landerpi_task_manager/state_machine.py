"""Pure task sequencing for explicit dry-run validation; no hardware commands."""
from enum import Enum


class State(str, Enum):
    BOOT = 'BOOT'
    SELF_CHECK = 'SELF_CHECK'
    IDLE = 'IDLE'
    PREPARE = 'PREPARE'
    NAV_TO_PICK = 'NAV_TO_PICK'
    FINE_ALIGN_PICK = 'FINE_ALIGN_PICK'
    ARM_OBSERVE = 'ARM_OBSERVE'
    DETECT_OBJECT = 'DETECT_OBJECT'
    COMPUTE_GRASP = 'COMPUTE_GRASP'
    PICK_OBJECT = 'PICK_OBJECT'
    VERIFY_PICK = 'VERIFY_PICK'
    ARM_TRANSPORT = 'ARM_TRANSPORT'
    NAV_TO_PLACE = 'NAV_TO_PLACE'
    FINE_ALIGN_PLACE = 'FINE_ALIGN_PLACE'
    PLACE_OBJECT = 'PLACE_OBJECT'
    VERIFY_PLACE = 'VERIFY_PLACE'
    RETURN_TRANSPORT = 'RETURN_TRANSPORT'
    NAV_HOME = 'NAV_HOME'
    FINISHED = 'FINISHED'
    SAFE_STOP = 'SAFE_STOP'


CYCLE = tuple(State)[3:list(State).index(State.NAV_HOME)]


def task_steps(quantity, return_home=False):
    if quantity < 1 or quantity > 100:
        raise ValueError('quantity must be in [1, 100]')
    steps = [(state, index) for index in range(1, quantity + 1) for state in CYCLE]
    if return_home:
        steps.append((State.NAV_HOME, quantity))
    steps.append((State.FINISHED, quantity))
    return steps


class StateMachine:
    def __init__(self):
        self.state = State.BOOT
        self.steps = []
        self.cursor = 0

    def initialize(self):
        if self.state != State.BOOT:
            raise ValueError('already initialized')
        self.state = State.SELF_CHECK

    def ready(self):
        if self.state != State.SELF_CHECK:
            raise ValueError('not checking readiness')
        self.state = State.IDLE

    def start(self, quantity, return_home=False):
        if self.state != State.IDLE:
            raise ValueError('task manager is busy')
        self.steps = task_steps(quantity, return_home)
        self.cursor = 0

    def advance(self):
        if self.cursor >= len(self.steps):
            raise ValueError('no pending step')
        self.state, index = self.steps[self.cursor]
        self.cursor += 1
        return self.state, index

    def stop(self):
        self.state = State.SAFE_STOP

    def reset(self):
        if self.state not in (State.FINISHED, State.SAFE_STOP):
            raise ValueError('task has not ended')
        self.state = State.IDLE
