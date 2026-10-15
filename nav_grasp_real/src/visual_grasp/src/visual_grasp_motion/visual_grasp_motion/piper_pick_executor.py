"""Feedback-checked staged and one-call execution for the real PiPER arm."""

from __future__ import annotations

import copy
import math
import time
from enum import Enum
from typing import Optional

import rclpy
from agx_arm_msgs.msg import AgxArmStatus, GripperStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .gripper_safety import load_gripper_range_report
from .motion_math import (
    position_distance,
    quaternion_distance_rad,
    step_toward,
    tcp_to_flange,
    tool_down_tilt_rad,
)


class PickState(str, Enum):
    IDLE = 'IDLE'
    OPENING = 'OPENING'
    MOVING_PREGRASP = 'MOVING_PREGRASP'
    HOLDING_PREGRASP = 'HOLDING_PREGRASP'
    AWAITING_CONFIRMATION = 'AWAITING_CONFIRMATION'
    DESCENDING = 'DESCENDING'
    CLOSING = 'CLOSING'
    LIFTING = 'LIFTING'
    STRAIGHTENING = 'STRAIGHTENING'
    MOVING_TO_CARRY = 'MOVING_TO_CARRY'
    RECOVERING_EMPTY_GRASP = 'RECOVERING_EMPTY_GRASP'
    WAITING_RETRY_DETECTION = 'WAITING_RETRY_DETECTION'
    RETRY_PLANNING = 'RETRY_PLANNING'
    SUCCEEDED = 'SUCCEEDED'
    ERROR = 'ERROR'


ACTIVE_STATES = {
    PickState.OPENING,
    PickState.MOVING_PREGRASP,
    PickState.HOLDING_PREGRASP,
    PickState.DESCENDING,
    PickState.CLOSING,
    PickState.LIFTING,
    PickState.STRAIGHTENING,
    PickState.MOVING_TO_CARRY,
    PickState.RECOVERING_EMPTY_GRASP,
    PickState.WAITING_RETRY_DETECTION,
    PickState.RETRY_PLANNING,
}


class PiperPickExecutor(Node):
    """Execute a checked pick manually in two phases or as one full sequence."""

    def __init__(self) -> None:
        super().__init__('piper_pick_executor')
        self.allow_robot_commands = bool(self.declare_parameter(
            'allow_robot_commands', False).value)
        self.base_frame = self.declare_parameter(
            'base_frame', 'piper_base_link').value
        self.pick_sequence_topic = self.declare_parameter(
            'pick_sequence_topic', '/visual_grasp/pick_sequence').value
        self.move_p_topic = self.declare_parameter(
            'move_p_topic', '/arm/control/move_p').value
        self.move_l_topic = self.declare_parameter(
            'move_l_topic', '/arm/control/move_l').value
        self.move_j_topic = self.declare_parameter(
            'move_j_topic', '/arm/control/move_j').value
        self.gripper_command_topic = self.declare_parameter(
            'gripper_command_topic', '/arm/control/joint_states').value
        self.joint_feedback_topic = self.declare_parameter(
            'joint_feedback_topic', '/arm/feedback/joint_states').value
        self.flange_feedback_topic = self.declare_parameter(
            'flange_feedback_topic', '/arm/feedback/tcp_pose').value
        self.arm_status_topic = self.declare_parameter(
            'arm_status_topic', '/arm/feedback/arm_status').value
        self.gripper_status_topic = self.declare_parameter(
            'gripper_status_topic', '/arm/feedback/gripper_status').value

        self.tcp_offset = self._three_values('tcp_offset_xyz_m', [0.0, 0.0, 0.142])
        self.tcp_workspace_min = self._three_values(
            'tcp_workspace_min_xyz_m', [0.15, -0.35, -0.120])
        self.tcp_workspace_max = self._three_values(
            'tcp_workspace_max_xyz_m', [0.60, 0.35, 0.35])
        self.flange_workspace_min = self._three_values(
            'flange_workspace_min_xyz_m', [0.05, -0.45, 0.020])
        self.flange_workspace_max = self._three_values(
            'flange_workspace_max_xyz_m', [0.70, 0.45, 0.70])
        self.grasp_z_min_m = float(self.declare_parameter(
            'grasp_z_min_m', -0.120).value)
        self.grasp_z_max_m = float(self.declare_parameter(
            'grasp_z_max_m', 0.200).value)
        self.min_approach_height_m = float(self.declare_parameter(
            'min_approach_height_m', 0.060).value)
        self.max_tool_tilt_rad = math.radians(float(self.declare_parameter(
            'max_tool_tilt_deg', 12.0).value))

        self.max_plan_age_s = float(self.declare_parameter(
            'max_plan_age_s', 2.0).value)
        self.max_feedback_age_s = float(self.declare_parameter(
            'max_feedback_age_s', 0.6).value)
        self.motion_timeout_s = float(self.declare_parameter(
            'motion_timeout_s', 20.0).value)
        self.gripper_timeout_s = float(self.declare_parameter(
            'gripper_timeout_s', 5.0).value)
        self.gripper_command_repeat_s = float(self.declare_parameter(
            'gripper_command_repeat_s', 0.5).value)
        self.require_gripper_homed = bool(self.declare_parameter(
            'require_gripper_homed', True).value)
        self.allow_verified_unhomed_gripper = bool(self.declare_parameter(
            'allow_verified_unhomed_gripper', False).value)
        self.gripper_range_report_path = str(self.declare_parameter(
            'gripper_range_report_path', '').value)
        self.gripper_range_expected_firmware = str(self.declare_parameter(
            'gripper_range_expected_firmware', 'S-V1.9-1').value)
        self.gripper_range_closed_abs_limit_m = float(self.declare_parameter(
            'gripper_range_closed_abs_limit_m', 0.002).value)
        self.gripper_range_min_span_m = float(self.declare_parameter(
            'gripper_range_min_span_m', 0.070).value)
        self.gripper_range_max_span_m = float(self.declare_parameter(
            'gripper_range_max_span_m', 0.085).value)
        self.gripper_range_max_jitter_m = float(self.declare_parameter(
            'gripper_range_max_jitter_m', 0.001).value)
        self.motion_settle_s = float(self.declare_parameter(
            'motion_settle_s', 0.4).value)
        self.automatic_pregrasp_hold_s = float(self.declare_parameter(
            'automatic_pregrasp_hold_s', 1.0).value)
        if self.automatic_pregrasp_hold_s < 0.0:
            raise ValueError('automatic_pregrasp_hold_s must be non-negative')
        self.position_tolerance_m = float(self.declare_parameter(
            'position_tolerance_m', 0.006).value)
        self.orientation_tolerance_rad = math.radians(float(self.declare_parameter(
            'orientation_tolerance_deg', 5.0).value))

        self.move_to_carry_after_pick = bool(self.declare_parameter(
            'move_to_carry_after_pick', True).value)
        self.carry_joint_names = [str(value) for value in self.declare_parameter(
            'carry_joint_names', [
                'joint1', 'joint2', 'joint3',
                'joint4', 'joint5', 'joint6',
            ]).value]
        self.carry_joint_positions = [float(value) for value in self.declare_parameter(
            'carry_joint_positions_rad', [
                0.0, 0.46, -0.74, 0.0, 1.05, 0.0,
            ]).value]
        self.carry_joint_tolerance_rad = math.radians(float(
            self.declare_parameter('carry_joint_tolerance_deg', 6.0).value))
        self.carry_joint_limits_min = [float(value) for value in self.declare_parameter(
            'carry_joint_limits_min_rad', [
                -2.61799, 0.0, -2.96706,
                -1.74533, -1.22173, -2.09440,
            ]).value]
        self.carry_joint_limits_max = [float(value) for value in self.declare_parameter(
            'carry_joint_limits_max_rad', [
                2.61799, 3.14159, 0.0,
                1.74533, 1.22173, 2.09440,
            ]).value]
        self._validate_carry_configuration()

        self.gripper_open_width_m = float(self.declare_parameter(
            'gripper_open_width_m', 0.050).value)
        self.gripper_close_width_m = float(self.declare_parameter(
            'gripper_close_width_m', 0.0).value)
        self.gripper_force_n = float(self.declare_parameter(
            'gripper_force_n', 1.0).value)
        self.gripper_open_tolerance_m = float(self.declare_parameter(
            'gripper_open_tolerance_m', 0.006).value)
        self.gripper_close_settle_s = float(self.declare_parameter(
            'gripper_close_settle_s', 1.5).value)
        self.gripper_close_step_m = float(self.declare_parameter(
            'gripper_close_step_m', 0.004).value)
        self.gripper_close_step_interval_s = float(self.declare_parameter(
            'gripper_close_step_interval_s', 0.2).value)
        if self.gripper_close_step_m <= 0.0:
            raise ValueError('gripper_close_step_m must be positive')
        if self.gripper_close_step_interval_s <= 0.0:
            raise ValueError('gripper_close_step_interval_s must be positive')
        self.object_width_min_m = float(self.declare_parameter(
            'object_width_min_m', 0.020).value)
        self.object_width_max_m = float(self.declare_parameter(
            'object_width_max_m', 0.040).value)
        self.max_grasp_retries = int(self.declare_parameter(
            'max_grasp_retries', 1).value)
        self.retry_detection_settle_s = float(self.declare_parameter(
            'retry_detection_settle_s', 0.8).value)
        self.retry_plan_timeout_s = float(self.declare_parameter(
            'retry_plan_timeout_s', 4.0).value)
        if self.max_grasp_retries < 0:
            raise ValueError('max_grasp_retries must be non-negative')
        if self.retry_detection_settle_s < 0.0:
            raise ValueError('retry_detection_settle_s must be non-negative')
        if self.retry_plan_timeout_s <= 0.0:
            raise ValueError('retry_plan_timeout_s must be positive')
        if not 0.0 <= self.object_width_min_m < self.object_width_max_m:
            raise ValueError(
                'object_width_min_m/object_width_max_m define an invalid range')
        self.gripper_command_min_width_m = float(self.declare_parameter(
            'gripper_command_min_width_m', 0.0).value)
        self.gripper_command_max_width_m = float(self.declare_parameter(
            'gripper_command_max_width_m', 0.070).value)

        for label, width in (
                ('gripper_open_width_m', self.gripper_open_width_m),
                ('gripper_close_width_m', self.gripper_close_width_m)):
            if not self.gripper_command_min_width_m <= width <= \
                    self.gripper_command_max_width_m:
                raise ValueError(
                    f'{label}={width:.6f} is outside configured command range '
                    f'[{self.gripper_command_min_width_m:.6f}, '
                    f'{self.gripper_command_max_width_m:.6f}] m')

        self.verified_gripper_range = None
        self.gripper_range_verification_error: Optional[str] = None
        if self.allow_verified_unhomed_gripper:
            try:
                self.verified_gripper_range = load_gripper_range_report(
                    self.gripper_range_report_path,
                    expected_firmware=self.gripper_range_expected_firmware,
                    closed_abs_limit_m=self.gripper_range_closed_abs_limit_m,
                    min_span_m=self.gripper_range_min_span_m,
                    max_span_m=self.gripper_range_max_span_m,
                    max_jitter_m=self.gripper_range_max_jitter_m,
                )
            except ValueError as error:
                self.gripper_range_verification_error = str(error)

        self.latest_sequence: Optional[PoseArray] = None
        self.latest_sequence_received = 0.0
        self.flange_feedback: Optional[PoseStamped] = None
        self.flange_feedback_received = 0.0
        self.arm_status: Optional[AgxArmStatus] = None
        self.arm_status_received = 0.0
        self.gripper_status: Optional[GripperStatus] = None
        self.gripper_status_received = 0.0
        self.joint_feedback: Optional[JointState] = None
        self.joint_feedback_received = 0.0
        self.latched_tcp_poses: list[Pose] = []
        self.latched_flange_poses: list[Pose] = []
        self.state = PickState.IDLE
        self.state_detail = 'Waiting for a planned pick sequence'
        self.deadline = 0.0
        self.state_started = time.monotonic()
        self.target_reached_since: Optional[float] = None
        self.last_gripper_command = 0.0
        self.active_gripper_target: Optional[float] = None
        self.automatic_pick = False
        self.close_command_width: Optional[float] = None
        self.close_next_step_time = 0.0
        self.close_final_command_time: Optional[float] = None
        self.retry_allowed = False
        self.grasp_retry_count = 0
        self.retry_open_reached_at: Optional[float] = None
        self.retry_plan_requested_at = 0.0
        self.retry_plan_future = None
        self.retry_plan_response_ok = False

        self.move_p_publisher = self.create_publisher(
            PoseStamped, self.move_p_topic, 1)
        self.move_l_publisher = self.create_publisher(
            PoseStamped, self.move_l_topic, 1)
        self.move_j_publisher = self.create_publisher(
            JointState, self.move_j_topic, 1)
        self.gripper_publisher = self.create_publisher(
            JointState, self.gripper_command_topic, 1)
        self.state_publisher = self.create_publisher(
            String, '/visual_grasp/execution_state', 10)
        self.command_pose_publisher = self.create_publisher(
            PoseStamped, '/visual_grasp/commanded_flange_pose', 10)
        self.plan_pick_client = self.create_client(
            Trigger, '/visual_grasp/plan_pick')

        self.create_subscription(
            PoseArray, self.pick_sequence_topic, self._sequence_callback, 10)
        self.create_subscription(
            PoseStamped, self.flange_feedback_topic,
            self._flange_feedback_callback, 10)
        self.create_subscription(
            AgxArmStatus, self.arm_status_topic, self._arm_status_callback, 10)
        self.create_subscription(
            GripperStatus, self.gripper_status_topic,
            self._gripper_status_callback, 10)
        self.create_subscription(
            JointState, self.joint_feedback_topic,
            self._joint_feedback_callback, 10)

        self.create_service(
            Trigger, '/visual_grasp/move_to_pregrasp', self._move_to_pregrasp)
        self.create_service(
            Trigger, '/visual_grasp/execute_pick', self._execute_pick)
        self.create_service(
            Trigger, '/visual_grasp/execute_full_pick', self._execute_full_pick)
        self.create_service(
            Trigger, '/visual_grasp/reset_pick', self._reset_pick)
        self.create_service(
            Trigger, '/visual_grasp/pick_status', self._pick_status)
        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._publish_state)

        if self.allow_robot_commands:
            self.get_logger().warning(
                'REAL ROBOT COMMANDS ENABLED: use the guarded two-stage services '
                'or explicitly call execute_full_pick for automatic continuation')
        else:
            self.get_logger().warning(
                'Robot commands disabled. Relaunch with '
                'allow_robot_commands:=true only after checking dry-run poses')
        if self.verified_gripper_range is not None:
            measured = self.verified_gripper_range
            self.get_logger().warning(
                'Verified unhomed gripper fallback is available: '
                f'closed={measured.closed_width_m:.6f} m, '
                f'open={measured.open_width_m:.6f} m, '
                f'span={measured.span_m:.6f} m; commands remain limited to '
                f'[{self.gripper_command_min_width_m:.3f}, '
                f'{self.gripper_command_max_width_m:.3f}] m')
        elif self.allow_verified_unhomed_gripper:
            self.get_logger().error(
                'Verified unhomed gripper fallback is unavailable: '
                + str(self.gripper_range_verification_error))
        self._publish_state()

    def _three_values(self, name: str, default) -> list[float]:
        values = [float(value) for value in self.declare_parameter(name, default).value]
        if len(values) != 3:
            raise ValueError(f'{name} must contain exactly three values')
        return values

    def _validate_carry_configuration(self) -> None:
        count = len(self.carry_joint_names)
        expected_names = [f'joint{index}' for index in range(1, 7)]
        if self.carry_joint_names != expected_names:
            raise ValueError(
                'carry_joint_names must be ordered exactly as '
                '[joint1, joint2, joint3, joint4, joint5, joint6]')
        vectors = (
            ('carry_joint_positions_rad', self.carry_joint_positions),
            ('carry_joint_limits_min_rad', self.carry_joint_limits_min),
            ('carry_joint_limits_max_rad', self.carry_joint_limits_max),
        )
        for label, values in vectors:
            if len(values) != count or not all(math.isfinite(value) for value in values):
                raise ValueError(f'{label} must contain six finite values')
        if self.carry_joint_tolerance_rad <= 0.0:
            raise ValueError('carry_joint_tolerance_deg must be positive')
        for name, target, lower, upper in zip(
                self.carry_joint_names,
                self.carry_joint_positions,
                self.carry_joint_limits_min,
                self.carry_joint_limits_max):
            if lower >= upper:
                raise ValueError(f'Invalid configured limits for {name}')
            if not lower <= target <= upper:
                raise ValueError(
                    f'Carry target {name}={target:.4f} rad is outside '
                    f'[{lower:.4f}, {upper:.4f}] rad')

    def _sequence_callback(self, message: PoseArray) -> None:
        self.latest_sequence = message
        self.latest_sequence_received = time.monotonic()

    def _flange_feedback_callback(self, message: PoseStamped) -> None:
        self.flange_feedback = message
        self.flange_feedback_received = time.monotonic()

    def _arm_status_callback(self, message: AgxArmStatus) -> None:
        self.arm_status = message
        self.arm_status_received = time.monotonic()

    def _gripper_status_callback(self, message: GripperStatus) -> None:
        self.gripper_status = message
        self.gripper_status_received = time.monotonic()

    def _joint_feedback_callback(self, message: JointState) -> None:
        self.joint_feedback = message
        self.joint_feedback_received = time.monotonic()

    def _move_to_pregrasp(self, _request, response):
        return self._start_pick(response, automatic=False)

    def _execute_full_pick(self, _request, response):
        return self._start_pick(response, automatic=True)

    def _start_pick(self, response, automatic: bool):
        if not self.allow_robot_commands:
            return self._service_failure(
                response, 'Robot commands are disabled by allow_robot_commands=false')
        if self.state in ACTIVE_STATES:
            return self._service_failure(
                response, f'Executor is busy in state {self.state.value}')
        if self.state == PickState.AWAITING_CONFIRMATION:
            return self._service_failure(
                response, 'Already at pregrasp; call execute_pick or reset_pick')

        # A manual range measurement intentionally leaves the gripper
        # disabled.  The first width command enables it, so allow only this
        # phase to start from disabled feedback; OPENING enforces a short
        # enable grace period below.
        error = self._ready_error(
            require_plan=True, allow_gripper_disabled=True)
        if error:
            return self._service_failure(response, error)
        try:
            tcp_poses, flange_poses = self._validated_sequence(self.latest_sequence)
        except ValueError as error:
            return self._service_failure(response, str(error))
        if self.move_p_publisher.get_subscription_count() < 1:
            return self._service_failure(response, 'PiPER move_p subscriber is missing')
        if self.gripper_publisher.get_subscription_count() < 1:
            return self._service_failure(response, 'PiPER gripper subscriber is missing')
        if automatic and self.move_l_publisher.get_subscription_count() < 1:
            return self._service_failure(response, 'PiPER move_l subscriber is missing')
        if self.move_to_carry_after_pick and \
                self.move_j_publisher.get_subscription_count() < 1:
            return self._service_failure(response, 'PiPER move_j subscriber is missing')

        self.latched_tcp_poses = tcp_poses
        self.latched_flange_poses = flange_poses
        self.automatic_pick = automatic
        self.retry_allowed = automatic
        self.grasp_retry_count = 0
        self.retry_open_reached_at = None
        self.retry_plan_future = None
        self.retry_plan_response_ok = False
        self._command_gripper(self.gripper_open_width_m)
        self._transition(
            PickState.OPENING, 'Opening gripper before pregrasp',
            self.gripper_timeout_s)
        response.success = True
        response.message = (
            'Accepted full pick: open, align above target, descend, close, '
            'lift, straighten, and move to the configured carry posture'
            if automatic else
            'Accepted phase 1: opening gripper, then moving only to pregrasp')
        return response

    def _execute_pick(self, _request, response):
        if not self.allow_robot_commands:
            return self._service_failure(
                response, 'Robot commands are disabled by allow_robot_commands=false')
        if self.state != PickState.AWAITING_CONFIRMATION:
            return self._service_failure(
                response, 'Not awaiting confirmation at the pregrasp pose')
        error = self._ready_error(require_plan=False)
        if error:
            return self._service_failure(response, error)
        if not self._pose_reached(self.latched_flange_poses[0], settle=False):
            return self._service_failure(
                response, 'Arm has moved away from the confirmed pregrasp pose')
        if self.move_to_carry_after_pick and \
                self.move_j_publisher.get_subscription_count() < 1:
            return self._service_failure(response, 'PiPER move_j subscriber is missing')
        self.automatic_pick = False
        # Calling phase 2 explicitly authorizes the requested one-time
        # re-detect/retry policy after an empty grasp.  The first descent still
        # requires the operator's pregrasp confirmation.
        self.retry_allowed = True
        self.grasp_retry_count = 0
        error = self._begin_descent()
        if error:
            return self._service_failure(response, error)
        response.success = True
        response.message = (
            'Accepted phase 2: descend, verify, lift, straighten, and move '
            'to the carry posture; one retry is allowed')
        return response

    def _reset_pick(self, _request, response):
        if self.state in ACTIVE_STATES:
            return self._service_failure(
                response, f'Cannot reset while motion state {self.state.value} is active')
        self.latched_tcp_poses = []
        self.latched_flange_poses = []
        self.automatic_pick = False
        self.retry_allowed = False
        self.grasp_retry_count = 0
        self.retry_open_reached_at = None
        self.retry_plan_future = None
        self.retry_plan_response_ok = False
        self._transition(PickState.IDLE, 'Reset; request a new plan_pick', 0.0)
        response.success = True
        response.message = self.state_detail
        return response

    def _pick_status(self, _request, response):
        response.success = self.state not in {PickState.ERROR}
        response.message = f'{self.state.value}: {self.state_detail}'
        return response

    @staticmethod
    def _service_failure(response, message: str):
        response.success = False
        response.message = message
        return response

    def _ready_error(
            self, require_plan: bool,
            allow_gripper_disabled: bool = False) -> Optional[str]:
        now = time.monotonic()
        if require_plan:
            if self.latest_sequence is None:
                return 'No pick sequence; call /visual_grasp/plan_pick first'
            age = now - self.latest_sequence_received
            if age > self.max_plan_age_s:
                return f'Pick sequence is stale ({age:.2f} s); plan again'
        if self.flange_feedback is None or (
                now - self.flange_feedback_received > self.max_feedback_age_s):
            return 'Flange pose feedback is missing or stale'
        if self.arm_status is None or (
                now - self.arm_status_received > self.max_feedback_age_s):
            return 'Arm status feedback is missing or stale'
        arm_error = self._arm_error()
        if arm_error:
            return arm_error
        if self.gripper_status is None or (
                now - self.gripper_status_received > self.max_feedback_age_s):
            return 'Gripper feedback is missing or stale'
        if self.move_to_carry_after_pick and (
                self.joint_feedback is None or
                now - self.joint_feedback_received > self.max_feedback_age_s):
            return 'Joint feedback is missing or stale'
        return self._gripper_error(
            allow_disabled=allow_gripper_disabled)

    def _arm_error(self) -> Optional[str]:
        status = self.arm_status
        if status is None:
            return 'Arm status is unavailable'
        if status.ctrl_mode != 1:
            return (
                f'PiPER ctrl_mode={status.ctrl_mode}, expected 1 '
                '(CAN control mode)')
        if status.arm_status != 0:
            return f'PiPER arm_status={status.arm_status}, expected 0'
        if status.err_status != 0:
            return f'PiPER err_status={status.err_status}'
        if any(status.joint_angle_limit):
            return 'At least one PiPER joint reports an angle limit'
        if any(status.communication_status_joint):
            return 'At least one PiPER joint reports a communication fault'
        if status.teach_status in (1, 3, 4, 5, 7):
            return f'PiPER teach operation is active (teach_status={status.teach_status})'
        return None

    def _gripper_error(self, allow_disabled: bool = False) -> Optional[str]:
        status = self.gripper_status
        if status is None:
            return 'Gripper status is unavailable'
        fault_names = [
            name for name in (
                'voltage_too_low', 'motor_overheating', 'driver_overcurrent',
                'driver_overheating', 'sensor_status', 'driver_error_status')
            if bool(getattr(status, name))
        ]
        if fault_names:
            return 'Gripper fault: ' + ', '.join(fault_names)
        if not status.driver_enable_status and not allow_disabled:
            return 'Gripper driver is not enabled'
        if self.require_gripper_homed and not status.homing_status:
            if self.verified_gripper_range is None:
                detail = self.gripper_range_verification_error or (
                    'verified manual range fallback is disabled')
                return (
                    'Gripper reports homing_status=false and no valid manual '
                    f'range verification is available: {detail}')
            live_width = float(status.width)
            measured = self.verified_gripper_range
            margin = 0.003
            if not (
                    measured.closed_width_m - margin <= live_width
                    <= measured.open_width_m + margin):
                return (
                    f'Gripper width {live_width:.6f} m is outside the verified '
                    f'manual range [{measured.closed_width_m:.6f}, '
                    f'{measured.open_width_m:.6f}] m')
        return None

    def _validated_sequence(self, sequence: PoseArray) -> tuple[list[Pose], list[Pose]]:
        if len(sequence.poses) != 4:
            raise ValueError(
                'Pick sequence must contain pregrasp, grasp, lift, and '
                'straighten poses')
        if sequence.header.frame_id and sequence.header.frame_id != self.base_frame:
            raise ValueError(
                f'Pick sequence frame is {sequence.header.frame_id}, expected {self.base_frame}')
        tcp_poses = [copy.deepcopy(pose) for pose in sequence.poses]
        labels = ('pregrasp', 'grasp', 'lift', 'straighten')
        for label, pose in zip(labels, tcp_poses):
            values = self._pose_values(pose)
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f'{label} pose contains a non-finite value')
            if not self._inside(
                    values[:3], self.tcp_workspace_min, self.tcp_workspace_max):
                raise ValueError(f'{label} TCP pose is outside the configured workspace')
            tilt = tool_down_tilt_rad(values[3:])
            if tilt > self.max_tool_tilt_rad:
                raise ValueError(
                    f'{label} tool tilt is {math.degrees(tilt):.1f} deg; '
                    f'maximum is {math.degrees(self.max_tool_tilt_rad):.1f} deg')

        pregrasp, grasp, lift, straighten = tcp_poses
        if not self.grasp_z_min_m <= grasp.position.z <= self.grasp_z_max_m:
            raise ValueError(
                f'Grasp TCP z={grasp.position.z:.4f} m is outside '
                f'[{self.grasp_z_min_m:.4f}, {self.grasp_z_max_m:.4f}] m')
        if pregrasp.position.z - grasp.position.z < self.min_approach_height_m:
            raise ValueError('Pregrasp is not high enough above the grasp pose')
        if lift.position.z - grasp.position.z < self.min_approach_height_m:
            raise ValueError('Lift is not high enough above the grasp pose')
        for other, label in ((pregrasp, 'pregrasp'), (lift, 'lift')):
            xy_error = math.hypot(
                other.position.x - grasp.position.x,
                other.position.y - grasp.position.y)
            if xy_error > 0.005:
                raise ValueError(f'{label} is not vertically aligned with grasp')

        for other, label in ((pregrasp, 'pregrasp'), (lift, 'lift')):
            orientation_error = quaternion_distance_rad(
                self._quaternion_values(other),
                self._quaternion_values(grasp),
            )
            if orientation_error > self.orientation_tolerance_rad:
                raise ValueError(
                    f'{label} changes orientation before the safe lift is complete')
        if position_distance(
                self._position_values(straighten),
                self._position_values(lift)) > 0.005:
            raise ValueError(
                'Straighten pose must keep the lifted TCP position fixed')

        flange_poses = [self._tcp_pose_to_flange(pose) for pose in tcp_poses]
        for label, pose in zip(labels, flange_poses):
            position = self._position_values(pose)
            if not self._inside(
                    position, self.flange_workspace_min, self.flange_workspace_max):
                raise ValueError(f'{label} flange pose is outside the configured workspace')
        return tcp_poses, flange_poses

    def _tcp_pose_to_flange(self, tcp_pose: Pose) -> Pose:
        position, quaternion = tcp_to_flange(
            self._position_values(tcp_pose),
            self._quaternion_values(tcp_pose),
            self.tcp_offset,
        )
        flange_pose = copy.deepcopy(tcp_pose)
        flange_pose.position.x, flange_pose.position.y, flange_pose.position.z = position
        (
            flange_pose.orientation.x,
            flange_pose.orientation.y,
            flange_pose.orientation.z,
            flange_pose.orientation.w,
        ) = quaternion
        return flange_pose

    @staticmethod
    def _inside(values, lower, upper) -> bool:
        return all(
            minimum <= value <= maximum
            for value, minimum, maximum in zip(values, lower, upper)
        )

    @staticmethod
    def _position_values(pose: Pose) -> tuple[float, float, float]:
        return pose.position.x, pose.position.y, pose.position.z

    @staticmethod
    def _quaternion_values(pose: Pose) -> tuple[float, float, float, float]:
        return (
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )

    def _pose_values(self, pose: Pose) -> tuple[float, ...]:
        return self._position_values(pose) + self._quaternion_values(pose)

    def _command_gripper(self, width_m: float) -> None:
        if not self.gripper_command_min_width_m <= width_m <= \
                self.gripper_command_max_width_m:
            raise ValueError(
                f'Gripper command {width_m:.6f} m is outside safe range '
                f'[{self.gripper_command_min_width_m:.6f}, '
                f'{self.gripper_command_max_width_m:.6f}] m')
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ['gripper']
        message.position = [float(width_m)]
        message.effort = [self.gripper_force_n]
        self.gripper_publisher.publish(message)
        self.active_gripper_target = float(width_m)
        self.last_gripper_command = time.monotonic()

    def _repeat_gripper_command_if_due(self, now: float) -> None:
        if self.active_gripper_target is None:
            return
        if now - self.last_gripper_command >= self.gripper_command_repeat_s:
            self._command_gripper(self.active_gripper_target)

    def _start_slow_close(self) -> None:
        current_width = min(
            self.gripper_command_max_width_m,
            max(self.gripper_close_width_m, float(self.gripper_status.width)),
        )
        first_target = step_toward(
            current_width,
            self.gripper_close_width_m,
            self.gripper_close_step_m,
        )
        self.close_command_width = first_target
        self._command_gripper(first_target)
        now = time.monotonic()
        self.close_next_step_time = now + self.gripper_close_step_interval_s
        self.close_final_command_time = (
            now if first_target == self.gripper_close_width_m else None)

    def _advance_slow_close(self, now: float) -> None:
        if self.close_command_width is None or self.close_final_command_time is not None:
            return
        if now < self.close_next_step_time:
            return
        target = step_toward(
            self.close_command_width,
            self.gripper_close_width_m,
            self.gripper_close_step_m,
        )
        self.close_command_width = target
        self._command_gripper(target)
        self.close_next_step_time = now + self.gripper_close_step_interval_s
        if target == self.gripper_close_width_m:
            self.close_final_command_time = now

    def _command_pose(self, publisher, pose: Pose) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.pose = copy.deepcopy(pose)
        publisher.publish(message)
        self.command_pose_publisher.publish(message)

    def _command_carry_posture(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self.carry_joint_names)
        message.position = list(self.carry_joint_positions)
        self.move_j_publisher.publish(message)

    def _transition(self, state: PickState, detail: str, timeout_s: float) -> None:
        self.state = state
        self.state_detail = detail
        self.state_started = time.monotonic()
        self.deadline = self.state_started + timeout_s if timeout_s > 0.0 else 0.0
        self.target_reached_since = None
        self.get_logger().info(f'{state.value}: {detail}')
        self._publish_state()

    def _fail(self, detail: str) -> None:
        self.automatic_pick = False
        self.retry_allowed = False
        self._transition(PickState.ERROR, detail, 0.0)
        self.get_logger().error(detail)

    def _start_retry_planning(self) -> None:
        if not self.plan_pick_client.service_is_ready():
            self._fail(
                'Arm retracted, but /visual_grasp/plan_pick is unavailable '
                'for the retry')
            return
        self.retry_plan_requested_at = time.monotonic()
        self.retry_plan_response_ok = False
        self.retry_plan_future = self.plan_pick_client.call_async(
            Trigger.Request())
        self._transition(
            PickState.RETRY_PLANNING,
            f'Re-detecting and replanning retry '
            f'{self.grasp_retry_count}/{self.max_grasp_retries}',
            self.retry_plan_timeout_s)

    def _accept_retry_plan_if_ready(self) -> None:
        future = self.retry_plan_future
        if future is None:
            self._fail('Retry planner request was not created')
            return
        if not self.retry_plan_response_ok:
            if not future.done():
                return
            try:
                result = future.result()
            except Exception as error:
                self._fail(f'Retry planning service failed: {error}')
                return
            if result is None or not result.success:
                message = 'no response' if result is None else result.message
                self._fail(f'Retry planning rejected: {message}')
                return
            self.retry_plan_response_ok = True

        # The Trigger response and the published PoseArray use separate ROS
        # channels.  Accept only the sequence received after this request.
        if self.latest_sequence_received < self.retry_plan_requested_at:
            return
        try:
            tcp_poses, flange_poses = self._validated_sequence(
                self.latest_sequence)
        except ValueError as error:
            self._fail(f'Retry plan is invalid: {error}')
            return
        self.latched_tcp_poses = tcp_poses
        self.latched_flange_poses = flange_poses
        self.automatic_pick = True
        self._command_gripper(self.gripper_open_width_m)
        self._transition(
            PickState.OPENING,
            f'Retry {self.grasp_retry_count}/{self.max_grasp_retries}: '
            'opening before the new pregrasp',
            self.gripper_timeout_s)

    def _object_width_confirms_block(self) -> bool:
        width = float(self.gripper_status.width)
        return self.object_width_min_m <= width <= self.object_width_max_m

    def _retry_or_fail_after_retract(self, width: float) -> None:
        if (self.retry_allowed
                and self.grasp_retry_count < self.max_grasp_retries):
            self.grasp_retry_count += 1
            self.retry_open_reached_at = None
            self._command_gripper(self.gripper_open_width_m)
            self._transition(
                PickState.WAITING_RETRY_DETECTION,
                f'Width {width:.4f} m did not confirm the block after '
                f'retraction; opening and stabilizing for retry '
                f'{self.grasp_retry_count}/{self.max_grasp_retries}',
                self.gripper_timeout_s + self.retry_detection_settle_s + 1.0)
            return
        self._fail(
            f'Arm retracted, but gripper width {width:.4f} m did not '
            'confirm a 30 mm block; retry limit reached')

    def _begin_descent(self) -> Optional[str]:
        if self.move_l_publisher.get_subscription_count() < 1:
            return 'PiPER move_l subscriber is missing'
        self._command_pose(self.move_l_publisher, self.latched_flange_poses[1])
        self._transition(
            PickState.DESCENDING, 'Descending linearly to grasp height',
            self.motion_timeout_s)
        return None

    def _publish_state(self) -> None:
        message = String()
        message.data = f'{self.state.value}: {self.state_detail}'
        self.state_publisher.publish(message)

    def _tick(self) -> None:
        if self.state not in ACTIVE_STATES:
            return
        allow_disabled = (
            self.state == PickState.OPENING
            and time.monotonic() - self.state_started <= 2.0)
        error = self._ready_error(
            require_plan=False,
            allow_gripper_disabled=allow_disabled,
        )
        if error:
            self._fail(error)
            return
        now = time.monotonic()
        if self.deadline and now > self.deadline:
            self._fail(f'{self.state.value} timed out; no further command was sent')
            return

        if self.state == PickState.OPENING:
            self._repeat_gripper_command_if_due(now)
            if self.gripper_status.width >= (
                    self.gripper_open_width_m - self.gripper_open_tolerance_m):
                self._command_pose(
                    self.move_p_publisher, self.latched_flange_poses[0])
                self._transition(
                    PickState.MOVING_PREGRASP,
                    ('Moving to the pregrasp pose; automatic descent is armed'
                     if self.automatic_pick else
                     'Moving to the pregrasp pose; final descent remains locked'),
                    self.motion_timeout_s)
            return

        if self.state == PickState.MOVING_PREGRASP:
            if self._pose_reached(self.latched_flange_poses[0]):
                if self.automatic_pick:
                    self._transition(
                        PickState.HOLDING_PREGRASP,
                        'Pregrasp reached and stable; holding before automatic descent',
                        self.automatic_pregrasp_hold_s + 1.0)
                else:
                    self._transition(
                        PickState.AWAITING_CONFIRMATION,
                        'Pregrasp reached; visually verify and call execute_pick', 0.0)
            return

        if self.state == PickState.HOLDING_PREGRASP:
            if not self._pose_reached(self.latched_flange_poses[0], settle=False):
                self._fail('Arm moved away while holding at the pregrasp pose')
                return
            if now - self.state_started >= self.automatic_pregrasp_hold_s:
                error = self._begin_descent()
                if error:
                    self._fail(error)
            return

        if self.state == PickState.DESCENDING:
            if self._pose_reached(self.latched_flange_poses[1]):
                self._start_slow_close()
                self._transition(
                    PickState.CLOSING,
                    'Closing gripper slowly and checking object width',
                    self.gripper_timeout_s)
            return

        if self.state == PickState.CLOSING:
            self._advance_slow_close(now)
            if self.close_final_command_time is None:
                return
            self._repeat_gripper_command_if_due(now)
            if now - self.close_final_command_time < self.gripper_close_settle_s:
                return
            width = float(self.gripper_status.width)
            object_detected = self._object_width_confirms_block()
            next_state = (
                PickState.LIFTING if object_detected
                else PickState.RECOVERING_EMPTY_GRASP)
            detail = (
                f'30 mm object confirmed at gripper width {width:.4f} m; lifting'
                if object_detected else
                f'Width {width:.4f} m is outside the configured 30 mm '
                'object band; retracting safely')
            self._command_pose(self.move_l_publisher, self.latched_flange_poses[2])
            self._transition(next_state, detail, self.motion_timeout_s)
            return

        if self.state in {PickState.LIFTING, PickState.RECOVERING_EMPTY_GRASP}:
            if not self._pose_reached(self.latched_flange_poses[2]):
                return
            width = float(self.gripper_status.width)
            # Check again after lift: if a block slips out during retraction,
            # the continuing close command can reduce the width toward zero.
            if self.state == PickState.LIFTING and \
                    self._object_width_confirms_block():
                self._command_pose(
                    self.move_p_publisher, self.latched_flange_poses[3])
                self._transition(
                    PickState.STRAIGHTENING,
                    'Lift is stable; restoring the nominal tool angle in air',
                    self.motion_timeout_s)
            else:
                self._retry_or_fail_after_retract(width)
            return

        if self.state == PickState.STRAIGHTENING:
            if not self._pose_reached(self.latched_flange_poses[3]):
                return
            width = float(self.gripper_status.width)
            if self._object_width_confirms_block():
                if self.move_to_carry_after_pick:
                    self._command_carry_posture()
                    target_degrees = ', '.join(
                        f'{math.degrees(value):.1f}'
                        for value in self.carry_joint_positions)
                    self._transition(
                        PickState.MOVING_TO_CARRY,
                        'Block is secure; moving to configured carry posture '
                        f'[{target_degrees}] deg',
                        self.motion_timeout_s)
                else:
                    self._complete_success(
                        'Pick completed and tool angle restored', width)
            else:
                self._retry_or_fail_after_retract(width)
            return

        if self.state == PickState.MOVING_TO_CARRY:
            if not self._carry_posture_reached():
                return
            width = float(self.gripper_status.width)
            if self._object_width_confirms_block():
                self._complete_success(
                    'Pick completed and configured carry posture reached',
                    width)
            else:
                # The camera is no longer over the original target after the
                # carry motion, so an automatic visual retry is not safe here.
                self._fail(
                    f'Carry posture reached, but gripper width {width:.4f} m '
                    'no longer confirms the block')
            return

        if self.state == PickState.WAITING_RETRY_DETECTION:
            self._repeat_gripper_command_if_due(now)
            gripper_is_open = self.gripper_status.width >= (
                self.gripper_open_width_m - self.gripper_open_tolerance_m)
            if not gripper_is_open:
                self.retry_open_reached_at = None
                return
            if self.retry_open_reached_at is None:
                self.retry_open_reached_at = now
                return
            if now - self.retry_open_reached_at >= self.retry_detection_settle_s:
                self._start_retry_planning()
            return

        if self.state == PickState.RETRY_PLANNING:
            self._accept_retry_plan_if_ready()

    def _complete_success(self, detail: str, width: float) -> None:
        self.automatic_pick = False
        self.retry_allowed = False
        self._transition(
            PickState.SUCCEEDED,
            f'{detail}; gripper width={width:.4f} m',
            0.0)

    def _carry_posture_reached(self, settle: bool = True) -> bool:
        if self.joint_feedback is None:
            return False
        positions = {
            name: float(self.joint_feedback.position[index])
            for index, name in enumerate(self.joint_feedback.name)
            if index < len(self.joint_feedback.position)
        }
        if any(name not in positions for name in self.carry_joint_names):
            self.target_reached_since = None
            return False
        errors = [
            abs(positions[name] - target)
            for name, target in zip(
                self.carry_joint_names, self.carry_joint_positions)
        ]
        inside = (
            all(math.isfinite(error) for error in errors)
            and max(errors) <= self.carry_joint_tolerance_rad)
        if not settle:
            return inside
        now = time.monotonic()
        if not inside:
            self.target_reached_since = None
            return False
        if self.target_reached_since is None:
            self.target_reached_since = now
            return False
        return now - self.target_reached_since >= self.motion_settle_s

    def _pose_reached(self, target: Pose, settle: bool = True) -> bool:
        if self.flange_feedback is None:
            return False
        current = self.flange_feedback.pose
        position_error = position_distance(
            self._position_values(current), self._position_values(target))
        orientation_error = quaternion_distance_rad(
            self._quaternion_values(current), self._quaternion_values(target))
        inside = (
            position_error <= self.position_tolerance_m
            and orientation_error <= self.orientation_tolerance_rad)
        if not settle:
            return inside
        now = time.monotonic()
        if not inside:
            self.target_reached_since = None
            return False
        if self.target_reached_since is None:
            self.target_reached_since = now
            return False
        return now - self.target_reached_since >= self.motion_settle_s


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PiperPickExecutor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
