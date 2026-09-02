import pytest

from trash_modular.pipeline.state_machine import InvalidTransition, State, StateMachine


def test_initial_state_is_idle():
    sm = StateMachine()
    assert sm.state == State.IDLE


def test_full_happy_path():
    sm = StateMachine()
    path = [
        State.SCAN, State.DETECTED, State.APPROACH, State.ALIGN,
        State.GRASP, State.TRANSPORT, State.DROP,
        State.RETURN, State.SCAN,
    ]
    for state in path:
        sm.transition(state)
    assert sm.state == State.SCAN


def test_invalid_transition_raises():
    sm = StateMachine()
    with pytest.raises(InvalidTransition):
        sm.transition(State.GRASP)


def test_any_active_state_can_reach_safe_stop_or_scan():
    for state in (State.SCAN, State.DETECTED, State.APPROACH, State.ALIGN):
        sm = StateMachine(initial=state)
        assert sm.can_transition(State.SAFE_STOP) or sm.can_transition(State.SCAN)


def test_safe_stop_only_returns_to_idle():
    sm = StateMachine(initial=State.SAFE_STOP)
    assert sm.can_transition(State.IDLE)
    assert not sm.can_transition(State.SCAN)
