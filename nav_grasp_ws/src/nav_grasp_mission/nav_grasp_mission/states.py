"""YASMIN states that adapt the already working ROS interfaces."""

from __future__ import annotations

import copy
import math
import threading
import time
from collections.abc import Sequence
from typing import Any

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from yasmin import Blackboard, State
from yasmin_ros import ServiceState
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED, TIMEOUT

from .logic import execution_state_token, is_transient_detection_failure


STANDARD_OUTCOMES = {SUCCEED, ABORT, CANCEL, TIMEOUT}


def _set_error(blackboard: Blackboard, message: str) -> None:
    blackboard['last_error'] = str(message)


class DependencyCheckState(State):
    """Wait until the existing navigation and grasp stacks are available."""

    def __init__(
        self,
        node,
        action_clients: Sequence[Any],
        service_clients: Sequence[Any],
        timeout_s: float,
        poll_period_s: float,
        odom_stale_timeout_s: float,
    ) -> None:
        super().__init__(STANDARD_OUTCOMES)
        self._node = node
        self._action_clients = list(action_clients)
        self._service_clients = list(service_clients)
        self._timeout_s = float(timeout_s)
        self._poll_period_s = float(poll_period_s)
        self._odom_stale_timeout_s = float(odom_stale_timeout_s)

    def execute(self, blackboard: Blackboard) -> str:
        self._node.emit_event(
            'WAIT', 'waiting for existing navigation and grasp stacks')
        deadline = time.monotonic() + self._timeout_s
        missing: list[str] = []

        while time.monotonic() < deadline:
            if self.is_canceled() or self._node.mission_cancel_requested():
                return CANCEL

            missing = []
            if not all(client.server_is_ready()
                       for client in self._action_clients):
                missing.append(self._node.navigation_action_name)
            for client in self._service_clients:
                if not client.service_is_ready():
                    missing.append(client.srv_name)
            if not self._node.transform_available():
                missing.append(
                    f'TF {self._node.map_frame}->{self._node.base_frame}')
            if not self._node.has_fresh_odom(self._odom_stale_timeout_s):
                missing.append(self._node.odom_topic)
            if not self._node.pick_state_publisher_available():
                missing.append(self._node.execution_state_topic)

            if not missing:
                self._node.emit_event(
                    'OK', 'all mission dependencies are ready')
                return SUCCEED
            time.sleep(self._poll_period_s)

        message = 'dependencies not ready: ' + ', '.join(sorted(set(missing)))
        _set_error(blackboard, message)
        self._node.emit_event('ERROR', message)
        return TIMEOUT


class CaptureStartPoseState(State):
    """Capture the current map-frame base pose for the return navigation."""

    def __init__(self, node) -> None:
        super().__init__(STANDARD_OUTCOMES)
        self._node = node

    def execute(self, blackboard: Blackboard) -> str:
        if self.is_canceled() or self._node.mission_cancel_requested():
            return CANCEL
        try:
            pose = self._node.capture_base_pose()
        except Exception as error:  # tf2 raises several concrete subclasses.
            message = f'failed to capture start pose: {error}'
            _set_error(blackboard, message)
            self._node.emit_event('ERROR', message)
            return ABORT

        blackboard['start_pose'] = pose
        self._node.emit_event(
            'POSE',
            f'captured start pose x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}',
        )
        return SUCCEED


class NavigateToPoseState(State):
    """Send a Nav2 goal and cancel it safely on timeout or mission cancel."""

    def __init__(
        self,
        node,
        pose_key: str,
        destination: str,
        goal_response_timeout_s: float,
        result_timeout_s: float,
    ) -> None:
        super().__init__(STANDARD_OUTCOMES)
        self._node = node
        self._client = node.navigation_client
        self._pose_key = pose_key
        self._destination = destination
        self._goal_response_timeout_s = float(goal_response_timeout_s)
        self._result_timeout_s = float(result_timeout_s)
        self._run_lock = threading.RLock()
        self._run_cancel_event: threading.Event | None = None
        self._goal_handle = None

    def cancel_state(self) -> None:
        with self._run_lock:
            cancel_event = self._run_cancel_event
            goal_handle = self._goal_handle
        if cancel_event is not None:
            cancel_event.set()
        if goal_handle is not None and goal_handle.accepted:
            goal_handle.cancel_goal_async()
        self._node.stop_base()
        super().cancel_state()

    def execute(self, blackboard: Blackboard) -> str:
        if self.is_canceled() or self._node.mission_cancel_requested():
            return CANCEL
        if not blackboard.contains(self._pose_key):
            message = f'blackboard has no {self._pose_key}'
            _set_error(blackboard, message)
            return ABORT

        run_cancel = threading.Event()
        with self._run_lock:
            self._run_cancel_event = run_cancel
            self._goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = copy.deepcopy(blackboard[self._pose_key])
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        self._node.emit_event(
            'NAV',
            f'navigating to {self._destination}: '
            f'x={goal.pose.pose.position.x:.3f}, '
            f'y={goal.pose.pose.position.y:.3f}',
        )

        acceptance_event = threading.Event()
        acceptance: dict[str, Any] = {}

        def goal_response_callback(future) -> None:
            try:
                handle = future.result()
                acceptance['handle'] = handle
                with self._run_lock:
                    if self._run_cancel_event is run_cancel:
                        self._goal_handle = handle
                if (
                    run_cancel.is_set()
                    and handle is not None
                    and handle.accepted
                ):
                    handle.cancel_goal_async()
            except Exception as error:
                acceptance['error'] = error
            finally:
                acceptance_event.set()

        try:
            future = self._client.send_goal_async(goal)
            future.add_done_callback(goal_response_callback)
        except Exception as error:
            return self._navigation_failure(
                blackboard, f'failed to send navigation goal: {error}', ABORT)

        outcome = self._wait_event(
            acceptance_event,
            run_cancel,
            self._goal_response_timeout_s,
        )
        if outcome is not None:
            run_cancel.set()
            if outcome == TIMEOUT:
                return self._navigation_failure(
                    blackboard,
                    'navigation goal acceptance timed out for '
                    f'{self._destination}',
                    TIMEOUT,
                )
            self._node.emit_event(
                'CANCEL', 'navigation goal canceled before acceptance')
            self._node.stop_base()
            return CANCEL

        if 'error' in acceptance:
            return self._navigation_failure(
                blackboard,
                f'navigation goal request failed: {acceptance["error"]}',
                ABORT,
            )
        handle = acceptance.get('handle')
        if handle is None or not handle.accepted:
            return self._navigation_failure(
                blackboard,
                f'Nav2 rejected the goal for {self._destination}',
                ABORT,
            )

        result_event = threading.Event()
        result: dict[str, Any] = {}

        def result_callback(future) -> None:
            try:
                result['wrapped'] = future.result()
            except Exception as error:
                result['error'] = error
            finally:
                result_event.set()

        result_future = handle.get_result_async()
        result_future.add_done_callback(result_callback)
        outcome = self._wait_event(
            result_event,
            run_cancel,
            self._result_timeout_s,
        )
        if outcome is not None:
            run_cancel.set()
            handle.cancel_goal_async()
            if outcome == TIMEOUT:
                return self._navigation_failure(
                    blackboard,
                    f'navigation timed out while going to {self._destination}',
                    TIMEOUT,
                )
            self._node.emit_event(
                'CANCEL', 'active Nav2 goal cancellation requested')
            self._node.stop_base()
            return CANCEL

        with self._run_lock:
            if self._run_cancel_event is run_cancel:
                self._goal_handle = None
                self._run_cancel_event = None

        if 'error' in result:
            return self._navigation_failure(
                blackboard,
                f'navigation result failed: {result["error"]}',
                ABORT,
            )
        wrapped = result.get('wrapped')
        if wrapped is None or wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            status = getattr(wrapped, 'status', 'missing')
            return self._navigation_failure(
                blackboard,
                f'navigation to {self._destination} ended with status '
                f'{status}',
                ABORT,
            )

        self._node.emit_event('OK', f'reached {self._destination}')
        return SUCCEED

    @staticmethod
    def _wait_event(
        event: threading.Event,
        cancel_event: threading.Event,
        timeout_s: float,
    ) -> str | None:
        deadline = time.monotonic() + timeout_s
        while not event.wait(0.05):
            if cancel_event.is_set():
                return CANCEL
            if time.monotonic() >= deadline:
                return TIMEOUT
        if cancel_event.is_set():
            return CANCEL
        return None

    def _navigation_failure(
        self,
        blackboard: Blackboard,
        message: str,
        outcome: str,
    ) -> str:
        _set_error(blackboard, message)
        self._node.emit_event('ERROR', message)
        self._node.stop_base()
        return outcome


class WaitBaseStillState(State):
    """Require fresh odometry to remain below velocity limits."""

    def __init__(
        self,
        node,
        settle_time_s: float,
        timeout_s: float,
        odom_stale_timeout_s: float,
        max_linear_speed_mps: float,
        max_angular_speed_rps: float,
    ) -> None:
        super().__init__(STANDARD_OUTCOMES)
        self._node = node
        self._settle_time_s = float(settle_time_s)
        self._timeout_s = float(timeout_s)
        self._odom_stale_timeout_s = float(odom_stale_timeout_s)
        self._max_linear_speed_mps = float(max_linear_speed_mps)
        self._max_angular_speed_rps = float(max_angular_speed_rps)

    def execute(self, blackboard: Blackboard) -> str:
        self._node.emit_event(
            'WAIT', 'waiting for the mobile base to remain still')
        initial_sequence, _, _, _ = self._node.get_odom_snapshot()
        stable_since: float | None = None
        deadline = time.monotonic() + self._timeout_s

        while time.monotonic() < deadline:
            if self.is_canceled() or self._node.mission_cancel_requested():
                return CANCEL
            sequence, received, linear_speed, angular_speed = \
                self._node.get_odom_snapshot()
            now = time.monotonic()
            fresh = (
                sequence > initial_sequence
                and now - received <= self._odom_stale_timeout_s
            )
            stopped = (
                fresh
                and math.isfinite(linear_speed)
                and math.isfinite(angular_speed)
                and linear_speed <= self._max_linear_speed_mps
                and angular_speed <= self._max_angular_speed_rps
            )
            if stopped:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= self._settle_time_s:
                    self._node.emit_event(
                        'OK', 'mobile base is continuously still')
                    return SUCCEED
            else:
                stable_since = None
            self._node.wait_for_telemetry(0.05)

        message = 'mobile base did not become continuously still in time'
        _set_error(blackboard, message)
        self._node.emit_event('ERROR', message)
        self._node.stop_base()
        return TIMEOUT


class CheckedTriggerState(ServiceState):
    """Call an existing Trigger service and require ``success=true``."""

    def __init__(
        self,
        node,
        service_name: str,
        label: str,
        response_timeout_s: float,
        record_pick_sequence: bool = False,
    ) -> None:
        self._mission_node = node
        self._label = label
        self._record_pick_sequence = bool(record_pick_sequence)
        super().__init__(
            Trigger,
            service_name,
            self._make_request,
            outcomes=set(),
            response_handler=self._handle_response,
            callback_group=node.io_callback_group,
            node=node,
            wait_timeout=1.0,
            response_timeout=float(response_timeout_s),
            maximum_retry=0,
        )

    @property
    def service_client(self):
        return self._service_client

    @staticmethod
    def _make_request(_blackboard: Blackboard) -> Trigger.Request:
        return Trigger.Request()

    def execute(self, blackboard: Blackboard) -> str:
        self._mission_node.emit_event('CALL', self._label)
        if (
            self.is_canceled()
            or self._mission_node.mission_cancel_requested()
        ):
            return CANCEL
        outcome = super().execute(blackboard)
        if (
            outcome in {TIMEOUT, ABORT}
            and not blackboard.contains('last_error')
        ):
            message = f'{self._label} failed with outcome {outcome}'
            _set_error(blackboard, message)
            self._mission_node.emit_event('ERROR', message)
        return outcome

    def _handle_response(self, blackboard: Blackboard, response) -> str:
        if not response.success:
            message = f'{self._label} was rejected: {response.message}'
            _set_error(blackboard, message)
            self._mission_node.emit_event('ERROR', message)
            return ABORT
        if self._record_pick_sequence:
            sequence, _, _ = self._mission_node.get_pick_snapshot()
            blackboard['pick_wait_after_sequence'] = sequence
        self._mission_node.emit_event('OK', response.message or self._label)
        return SUCCEED


class PlanPickWithinWindowState(State):
    """Retry transient no-detection responses during a bounded window."""

    def __init__(
        self,
        node,
        service_name: str,
        detection_window_s: float,
        retry_period_s: float,
        response_timeout_s: float,
    ) -> None:
        super().__init__(STANDARD_OUTCOMES)
        self._node = node
        self._service_name = str(service_name)
        self._detection_window_s = float(detection_window_s)
        self._retry_period_s = float(retry_period_s)
        self._response_timeout_s = float(response_timeout_s)
        self._service_client = node.create_client(
            Trigger,
            self._service_name,
            callback_group=node.io_callback_group,
        )

    @property
    def service_client(self):
        return self._service_client

    def _cancel_requested(self) -> bool:
        return self.is_canceled() or self._node.mission_cancel_requested()

    def _wait_interruptibly(self, duration_s: float) -> bool:
        deadline = time.monotonic() + max(float(duration_s), 0.0)
        while time.monotonic() < deadline:
            if self._cancel_requested():
                return False
            time.sleep(min(0.05, max(deadline - time.monotonic(), 0.0)))
        return not self._cancel_requested()

    def execute(self, blackboard: Blackboard) -> str:
        if self._cancel_requested():
            return CANCEL

        deadline = time.monotonic() + self._detection_window_s
        attempt = 0
        last_detail = 'no detector response received'
        self._node.emit_event(
            'WAIT',
            'searching for a valid grasp target for up to '
            f'{self._detection_window_s:.1f} s',
        )

        while time.monotonic() < deadline:
            if self._cancel_requested():
                return CANCEL

            if not self._service_client.service_is_ready():
                last_detail = (
                    f'{self._service_name} is temporarily unavailable')
            else:
                attempt += 1
                try:
                    future = self._service_client.call_async(Trigger.Request())
                except Exception as error:
                    last_detail = f'service call failed: {error}'
                else:
                    response_deadline = min(
                        deadline,
                        time.monotonic() + self._response_timeout_s,
                    )
                    while (
                        not future.done()
                        and time.monotonic() < response_deadline
                    ):
                        if self._cancel_requested():
                            future.cancel()
                            return CANCEL
                        time.sleep(0.02)

                    if not future.done():
                        future.cancel()
                        last_detail = 'grasp-plan response timed out'
                    else:
                        try:
                            response = future.result()
                        except Exception as error:
                            last_detail = f'grasp-plan service failed: {error}'
                        else:
                            if response.success:
                                self._node.emit_event(
                                    'OK',
                                    response.message
                                    or 'fresh visual grasp plan is ready',
                                )
                                return SUCCEED
                            last_detail = (
                                response.message
                                or 'grasp planner rejected the request')
                            if not is_transient_detection_failure(last_detail):
                                message = (
                                    'grasp planning was rejected: '
                                    f'{last_detail}')
                                _set_error(blackboard, message)
                                self._node.emit_event('ERROR', message)
                                return ABORT

            remaining = max(deadline - time.monotonic(), 0.0)
            if remaining <= 0.0:
                break
            self._node.emit_event(
                'WAIT',
                f'grasp target not ready (attempt {attempt}); '
                f'continuing detection for {remaining:.1f} s: {last_detail}',
            )
            if not self._wait_interruptibly(
                    min(self._retry_period_s, remaining)):
                return CANCEL

        message = (
            f'no valid grasp target detected within '
            f'{self._detection_window_s:.1f} s: {last_detail}')
        _set_error(blackboard, message)
        self._node.emit_event('ERROR', message)
        return TIMEOUT


class WaitPickResultState(State):
    """Wait for the existing grasp executor to report success or error."""

    def __init__(self, node, timeout_s: float) -> None:
        super().__init__(STANDARD_OUTCOMES)
        self._node = node
        self._timeout_s = float(timeout_s)

    def execute(self, blackboard: Blackboard) -> str:
        baseline = (
            int(blackboard['pick_wait_after_sequence'])
            if blackboard.contains('pick_wait_after_sequence')
            else self._node.get_pick_snapshot()[0]
        )
        deadline = time.monotonic() + self._timeout_s
        last_token = ''
        self._node.emit_event(
            'WAIT', 'waiting for the existing grasp executor')

        while time.monotonic() < deadline:
            if self.is_canceled() or self._node.mission_cancel_requested():
                self._node.emit_event(
                    'CANCEL',
                    'mission canceled during grasp; the grasp subsystem was '
                    'not interrupted',
                )
                return CANCEL
            sequence, message, _ = self._node.get_pick_snapshot()
            if sequence > baseline:
                token = execution_state_token(message)
                if token and token != last_token:
                    self._node.emit_event('PICK', message)
                    last_token = token
                if token == 'SUCCEEDED':
                    self._node.emit_event(
                        'OK', 'grasp succeeded and carry posture was reached')
                    return SUCCEED
                if token == 'ERROR':
                    detail = message or 'grasp executor reported ERROR'
                    _set_error(blackboard, detail)
                    self._node.emit_event('ERROR', detail)
                    return ABORT
            self._node.wait_for_telemetry(0.05)

        message = 'timed out waiting for the grasp executor result'
        _set_error(blackboard, message)
        self._node.emit_event('ERROR', message)
        return TIMEOUT
