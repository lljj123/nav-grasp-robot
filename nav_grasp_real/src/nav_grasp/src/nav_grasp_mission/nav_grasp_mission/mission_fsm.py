"""Construction of the YASMIN navigation-and-grasp state machine."""

from __future__ import annotations

from collections.abc import Mapping

from yasmin import State, StateMachine
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED, TIMEOUT

from .states import (
    CaptureStartPoseState,
    CheckedTriggerState,
    DependencyCheckState,
    NavigateToPoseState,
    PlanPickWithinWindowState,
    WaitBaseStillState,
    WaitPickResultState,
)


MISSION_SUCCEEDED = 'mission_succeeded'
MISSION_ABORTED = 'mission_aborted'
MISSION_CANCELED = 'mission_canceled'

STATE_ORDER = (
    'WAIT_DEPENDENCIES',
    'CAPTURE_START_POSE',
    'NAVIGATE_TO_TARGET',
    'WAIT_BASE_STILL',
    'PLAN_PICK',
    'EXECUTE_PICK',
    'WAIT_PICK_RESULT',
    'RETURN_TO_START',
)


def _transitions(next_state: str) -> dict[str, str]:
    return {
        SUCCEED: next_state,
        ABORT: MISSION_ABORTED,
        TIMEOUT: MISSION_ABORTED,
        CANCEL: MISSION_CANCELED,
    }


def assemble_mission(states: Mapping[str, State]) -> StateMachine:
    """Assemble a mission from concrete or test-double states."""
    missing = set(STATE_ORDER).difference(states)
    if missing:
        raise ValueError(f'Missing mission states: {sorted(missing)}')

    machine = StateMachine(
        'NAV_GRASP_MISSION',
        [MISSION_SUCCEEDED, MISSION_ABORTED, MISSION_CANCELED],
    )
    for current, following in zip(STATE_ORDER, STATE_ORDER[1:]):
        machine.add_state(current, states[current], _transitions(following))
    machine.add_state(
        STATE_ORDER[-1],
        states[STATE_ORDER[-1]],
        _transitions(MISSION_SUCCEEDED),
    )
    machine.validate(strict_mode=True)
    return machine


def build_mission(node) -> StateMachine:
    """Create the concrete state machine bound to a mission ROS node."""
    plan_pick = PlanPickWithinWindowState(
        node=node,
        service_name=node.plan_pick_service,
        detection_window_s=node.target_detection_window_s,
        retry_period_s=node.target_detection_retry_period_s,
        response_timeout_s=node.service_response_timeout_s,
    )
    execute_pick = CheckedTriggerState(
        node=node,
        service_name=node.execute_pick_service,
        label='starting the existing full-pick executor',
        response_timeout_s=node.service_response_timeout_s,
        record_pick_sequence=True,
    )

    states = {
        'WAIT_DEPENDENCIES': DependencyCheckState(
            node=node,
            action_clients=[node.navigation_client],
            service_clients=[plan_pick.service_client,
                             execute_pick.service_client],
            timeout_s=node.dependency_timeout_s,
            poll_period_s=node.dependency_poll_period_s,
            odom_stale_timeout_s=node.odom_stale_timeout_s,
        ),
        'CAPTURE_START_POSE': CaptureStartPoseState(node),
        'NAVIGATE_TO_TARGET': NavigateToPoseState(
            node=node,
            pose_key='target_pose',
            destination='grasp target',
            goal_response_timeout_s=node.nav_goal_response_timeout_s,
            result_timeout_s=node.navigation_timeout_s,
        ),
        'WAIT_BASE_STILL': WaitBaseStillState(
            node=node,
            settle_time_s=node.base_settle_time_s,
            timeout_s=node.base_settle_timeout_s,
            odom_stale_timeout_s=node.odom_stale_timeout_s,
            max_linear_speed_mps=node.max_base_linear_speed_mps,
            max_angular_speed_rps=node.max_base_angular_speed_rps,
        ),
        'PLAN_PICK': plan_pick,
        'EXECUTE_PICK': execute_pick,
        'WAIT_PICK_RESULT': WaitPickResultState(
            node=node,
            timeout_s=node.pick_execution_timeout_s,
        ),
        'RETURN_TO_START': NavigateToPoseState(
            node=node,
            pose_key='start_pose',
            destination='captured start pose',
            goal_response_timeout_s=node.nav_goal_response_timeout_s,
            result_timeout_s=node.navigation_timeout_s,
        ),
    }
    return assemble_mission(states)
