"""Pure state machine - no ROS, no I/O, no sleeps. Testable with plain
pytest. The pipeline node (trash_sorter_pipeline.py) is the only thing that
drives this against real hardware; test/test_state_machine.py drives it
against nothing at all.
"""

from enum import Enum


class State(Enum):
    IDLE = 'IDLE'
    SCAN = 'SCAN'
    DETECTED = 'DETECTED'
    APPROACH = 'APPROACH'
    ALIGN = 'ALIGN'
    GRASP = 'GRASP'
    TRANSPORT = 'TRANSPORT'
    DROP = 'DROP'
    RETURN = 'RETURN'
    SAFE_STOP = 'SAFE_STOP'


ALLOWED_TRANSITIONS = {
    State.IDLE: {State.SCAN},
    State.SCAN: {State.DETECTED, State.SAFE_STOP},
    State.DETECTED: {State.APPROACH, State.SCAN, State.SAFE_STOP},
    State.APPROACH: {State.ALIGN, State.SCAN, State.SAFE_STOP},
    State.ALIGN: {State.GRASP, State.SCAN, State.SAFE_STOP},
    # Grasp verification (servo-feedback deficit check) happens synchronously
    # inside the GRASP state's own handler - there's no separate async check
    # to wait on, so no dedicated VERIFY state.
    State.GRASP: {State.TRANSPORT, State.SAFE_STOP},
    State.TRANSPORT: {State.DROP, State.SAFE_STOP},
    State.DROP: {State.RETURN, State.SAFE_STOP},
    State.RETURN: {State.SCAN, State.IDLE},
    State.SAFE_STOP: {State.IDLE},
}


class InvalidTransition(Exception):
    pass


class StateMachine:
    def __init__(self, initial=State.IDLE):
        self.state = initial
        self.history = [initial]

    def can_transition(self, to_state):
        return to_state in ALLOWED_TRANSITIONS.get(self.state, set())

    def transition(self, to_state):
        if not self.can_transition(to_state):
            raise InvalidTransition(f'{self.state.value} -> {to_state.value} is not allowed')
        self.state = to_state
        self.history.append(to_state)
        return self.state
