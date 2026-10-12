"""Tests for the mission graph independently of ROS servers and hardware."""

import pytest
from yasmin import Blackboard, State
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED, TIMEOUT

from nav_grasp_mission.mission_fsm import (
    MISSION_ABORTED,
    MISSION_CANCELED,
    MISSION_SUCCEEDED,
    STATE_ORDER,
    assemble_mission,
)


class RecordedState(State):
    """Return one outcome and record that the state ran."""

    def __init__(self, name, outcome, calls):
        super().__init__({SUCCEED, ABORT, CANCEL, TIMEOUT})
        self._name = name
        self._outcome = outcome
        self._calls = calls

    def execute(self, _blackboard):
        self._calls.append(self._name)
        return self._outcome


def make_states(calls, outcome_by_name=None):
    outcomes = outcome_by_name or {}
    return {
        name: RecordedState(name, outcomes.get(name, SUCCEED), calls)
        for name in STATE_ORDER
    }


def test_success_path_runs_every_state_in_order():
    calls = []
    machine = assemble_mission(make_states(calls))

    assert machine(Blackboard()) == MISSION_SUCCEEDED
    assert calls == list(STATE_ORDER)


@pytest.mark.parametrize('failure_outcome', (ABORT, TIMEOUT))
def test_failure_stops_before_later_states(failure_outcome):
    calls = []
    failed_state = 'PLAN_PICK'
    machine = assemble_mission(
        make_states(calls, {failed_state: failure_outcome}))

    assert machine(Blackboard()) == MISSION_ABORTED
    assert calls == list(STATE_ORDER[:STATE_ORDER.index(failed_state) + 1])


def test_cancel_stops_before_later_states():
    calls = []
    canceled_state = 'WAIT_BASE_STILL'
    machine = assemble_mission(
        make_states(calls, {canceled_state: CANCEL}))

    assert machine(Blackboard()) == MISSION_CANCELED
    assert calls == list(STATE_ORDER[:STATE_ORDER.index(canceled_state) + 1])


def test_missing_state_is_rejected_during_assembly():
    calls = []
    states = make_states(calls)
    states.pop('RETURN_TO_START')

    with pytest.raises(ValueError, match='RETURN_TO_START'):
        assemble_mission(states)
