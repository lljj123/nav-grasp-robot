"""Focused tests for mission state synchronization behavior."""

from types import SimpleNamespace

from yasmin import Blackboard
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED, TIMEOUT

from nav_grasp_mission.states import (
    DependencyCheckState,
    PlanPickWithinWindowState,
    WaitPickResultState,
)


class CanceledNode:
    """Minimal node double for an early cancellation request."""

    def emit_event(self, _code, _message):
        pass

    def mission_cancel_requested(self):
        return True


class PickNode:
    """Feed deterministic grasp-state snapshots to a waiting state."""

    def __init__(self, snapshots, canceled=False):
        self._snapshots = list(snapshots)
        self._last_snapshot = self._snapshots[-1]
        self._canceled = canceled
        self.events = []

    def emit_event(self, code, message):
        self.events.append((code, message))

    def mission_cancel_requested(self):
        return self._canceled

    def get_pick_snapshot(self):
        if self._snapshots:
            self._last_snapshot = self._snapshots.pop(0)
        return self._last_snapshot

    def wait_for_telemetry(self, _timeout_s):
        pass


class ImmediateFuture:
    """Minimal already-completed service future."""

    def __init__(self, response):
        self._response = response

    def done(self):
        return True

    def result(self):
        return self._response

    def cancel(self):
        pass


class PlanClient:
    """Return deterministic Trigger-like responses."""

    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = 0

    def service_is_ready(self):
        return True

    def call_async(self, _request):
        index = min(self.calls, len(self.responses) - 1)
        self.calls += 1
        success, message = self.responses[index]
        return ImmediateFuture(SimpleNamespace(
            success=success, message=message))


class PlanNode:
    """Node double for the bounded target-detection state."""

    io_callback_group = None

    def __init__(self, responses):
        self.client = PlanClient(responses)
        self.events = []

    def create_client(self, *_args, **_kwargs):
        return self.client

    def emit_event(self, code, message):
        self.events.append((code, message))

    def mission_cancel_requested(self):
        return False


def test_dependency_wait_honors_cancel_requested_before_machine_runs():
    state = DependencyCheckState(
        node=CanceledNode(),
        action_clients=[],
        service_clients=[],
        timeout_s=1.0,
        poll_period_s=0.01,
        odom_stale_timeout_s=1.0,
    )

    assert state(Blackboard()) == CANCEL


def test_pick_wait_ignores_stale_success_then_accepts_new_success():
    node = PickNode([
        (10, 'SUCCEEDED: stale result', 0.0),
        (11, 'OPENING: active', 0.1),
        (12, 'SUCCEEDED: carry posture reached', 0.2),
    ])
    blackboard = Blackboard()
    blackboard['pick_wait_after_sequence'] = 10

    outcome = WaitPickResultState(node, timeout_s=1.0)(blackboard)

    assert outcome == SUCCEED
    assert ('PICK', 'SUCCEEDED: stale result') not in node.events
    assert ('PICK', 'OPENING: active') in node.events


def test_pick_wait_propagates_new_executor_error():
    node = PickNode([(8, 'ERROR: gripper feedback lost', 0.1)])
    blackboard = Blackboard()
    blackboard['pick_wait_after_sequence'] = 7

    outcome = WaitPickResultState(node, timeout_s=1.0)(blackboard)

    assert outcome == ABORT
    assert blackboard['last_error'] == 'ERROR: gripper feedback lost'


def test_pick_wait_honors_mission_cancel_flag():
    node = PickNode([(1, 'OPENING: active', 0.1)], canceled=True)
    blackboard = Blackboard()
    blackboard['pick_wait_after_sequence'] = 0

    assert WaitPickResultState(node, timeout_s=1.0)(blackboard) == CANCEL


def test_plan_pick_retries_empty_detection_then_succeeds():
    node = PlanNode([
        (False, 'No any detection inside configured workspace'),
        (True, 'Selected red, confidence=0.91'),
    ])
    state = PlanPickWithinWindowState(
        node=node,
        service_name='/visual_grasp/plan_pick',
        detection_window_s=0.1,
        retry_period_s=0.001,
        response_timeout_s=0.05,
    )

    assert state(Blackboard()) == SUCCEED
    assert node.client.calls == 2
    assert not any(code == 'ERROR' for code, _ in node.events)


def test_plan_pick_times_out_only_after_detection_window():
    node = PlanNode([(False, 'No cube detections available')])
    blackboard = Blackboard()
    state = PlanPickWithinWindowState(
        node=node,
        service_name='/visual_grasp/plan_pick',
        detection_window_s=0.03,
        retry_period_s=0.005,
        response_timeout_s=0.02,
    )

    assert state(blackboard) == TIMEOUT
    assert node.client.calls > 1
    assert 'within 0.0 s' in blackboard['last_error']


def test_plan_pick_aborts_immediately_for_non_transient_rejection():
    node = PlanNode([
        (False, 'Corrected grasp target is outside configured workspace'),
    ])
    state = PlanPickWithinWindowState(
        node=node,
        service_name='/visual_grasp/plan_pick',
        detection_window_s=0.1,
        retry_period_s=0.001,
        response_timeout_s=0.05,
    )

    assert state(Blackboard()) == ABORT
    assert node.client.calls == 1
