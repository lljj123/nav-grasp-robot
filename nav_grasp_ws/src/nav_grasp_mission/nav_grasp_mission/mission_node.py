"""ROS node hosting the YASMIN navigation-and-grasp mission."""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from yasmin import Blackboard
from yasmin_ros import set_ros_loggers

from .logic import validate_xyyaw, vector_norm, yaw_to_quaternion
from .mission_fsm import (
    MISSION_CANCELED,
    MISSION_SUCCEEDED,
    STATE_ORDER,
    build_mission,
)


class NavGraspMissionNode(Node):
    """Coordinate existing Nav2 and visual-grasp interfaces only."""

    def __init__(self) -> None:
        super().__init__('nav_grasp_mission')
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        self.io_callback_group = ReentrantCallbackGroup()
        self._telemetry_condition = threading.Condition(threading.RLock())
        self._odom_sequence = 0
        self._odom_received = 0.0
        self._linear_speed = math.inf
        self._angular_speed = math.inf
        self._pick_sequence = 0
        self._pick_state = ''
        self._pick_state_received = 0.0

        self._mission_lock = threading.RLock()
        self._mission_thread: Optional[threading.Thread] = None
        self._mission_running = False
        self._cancel_requested = threading.Event()
        self._state = 'IDLE'
        self._state_detail = 'waiting for /nav_grasp/start'
        self._blackboard: Optional[Blackboard] = None

        transient_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = self.create_publisher(
            String, '/nav_grasp/state', transient_qos)
        self._event_publisher = self.create_publisher(
            String, '/nav_grasp/event', transient_qos)
        self._stop_publisher = self.create_publisher(
            Twist, self.cmd_vel_topic, 10)

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            20,
            callback_group=self.io_callback_group,
        )
        self.create_subscription(
            String,
            self.execution_state_topic,
            self._pick_state_callback,
            20,
            callback_group=self.io_callback_group,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False)
        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            self.navigation_action_name,
            callback_group=self.io_callback_group,
        )

        self._machine = build_mission(self)
        self._machine.add_start_cb(self._on_machine_start)
        self._machine.add_transition_cb(self._on_machine_transition)

        self.create_service(
            Trigger,
            '/nav_grasp/start',
            self._start_callback,
            callback_group=self.io_callback_group,
        )
        self.create_service(
            Trigger,
            '/nav_grasp/cancel',
            self._cancel_callback,
            callback_group=self.io_callback_group,
        )
        self.create_service(
            Trigger,
            '/nav_grasp/reset',
            self._reset_callback,
            callback_group=self.io_callback_group,
        )
        self.create_service(
            Trigger,
            '/nav_grasp/status',
            self._status_callback,
            callback_group=self.io_callback_group,
        )

        set_ros_loggers(self)
        self._publish_state()
        self.emit_event(
            'READY',
            'mission coordinator ready; existing navigation and grasp stacks '
            'remain externally managed',
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('target_pose_configured', False)
        self.declare_parameter('target_pose_xyyaw', [0.0, 0.0, 0.0])
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('navigation_action_name', '/navigate_to_pose')
        self.declare_parameter('plan_pick_service', '/visual_grasp/plan_pick')
        self.declare_parameter(
            'execute_pick_service', '/visual_grasp/execute_full_pick')
        self.declare_parameter(
            'execution_state_topic', '/visual_grasp/execution_state')
        self.declare_parameter('dependency_timeout_s', 30.0)
        self.declare_parameter('dependency_poll_period_s', 0.2)
        self.declare_parameter('nav_goal_response_timeout_s', 10.0)
        self.declare_parameter('navigation_timeout_s', 240.0)
        self.declare_parameter('service_response_timeout_s', 5.0)
        self.declare_parameter('target_detection_window_s', 15.0)
        self.declare_parameter('target_detection_retry_period_s', 0.5)
        self.declare_parameter('base_settle_time_s', 1.5)
        self.declare_parameter('base_settle_timeout_s', 20.0)
        self.declare_parameter('odom_stale_timeout_s', 1.0)
        self.declare_parameter('max_base_linear_speed_mps', 0.02)
        self.declare_parameter('max_base_angular_speed_rps', 0.03)
        self.declare_parameter('pick_execution_timeout_s', 120.0)

    def _load_parameters(self) -> None:
        self.target_pose_configured = bool(
            self.get_parameter('target_pose_configured').value)
        self.target_pose_xyyaw = validate_xyyaw(
            self.get_parameter('target_pose_xyyaw').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.navigation_action_name = str(
            self.get_parameter('navigation_action_name').value)
        self.plan_pick_service = str(
            self.get_parameter('plan_pick_service').value)
        self.execute_pick_service = str(
            self.get_parameter('execute_pick_service').value)
        self.execution_state_topic = str(
            self.get_parameter('execution_state_topic').value)

        numeric_names = (
            'dependency_timeout_s',
            'dependency_poll_period_s',
            'nav_goal_response_timeout_s',
            'navigation_timeout_s',
            'service_response_timeout_s',
            'target_detection_window_s',
            'target_detection_retry_period_s',
            'base_settle_time_s',
            'base_settle_timeout_s',
            'odom_stale_timeout_s',
            'max_base_linear_speed_mps',
            'max_base_angular_speed_rps',
            'pick_execution_timeout_s',
        )
        for name in numeric_names:
            setattr(self, name, float(self.get_parameter(name).value))

    def _validate_parameters(self) -> None:
        string_values = {
            'map_frame': self.map_frame,
            'base_frame': self.base_frame,
            'odom_topic': self.odom_topic,
            'cmd_vel_topic': self.cmd_vel_topic,
            'navigation_action_name': self.navigation_action_name,
            'plan_pick_service': self.plan_pick_service,
            'execute_pick_service': self.execute_pick_service,
            'execution_state_topic': self.execution_state_topic,
        }
        for name, value in string_values.items():
            if not value.strip():
                raise ValueError(f'{name} must not be empty')

        numeric_names = (
            'dependency_timeout_s',
            'dependency_poll_period_s',
            'nav_goal_response_timeout_s',
            'navigation_timeout_s',
            'service_response_timeout_s',
            'target_detection_window_s',
            'target_detection_retry_period_s',
            'base_settle_time_s',
            'base_settle_timeout_s',
            'odom_stale_timeout_s',
            'max_base_linear_speed_mps',
            'max_base_angular_speed_rps',
            'pick_execution_timeout_s',
        )
        for name in numeric_names:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if self.base_settle_time_s >= self.base_settle_timeout_s:
            raise ValueError(
                'base_settle_time_s must be less than base_settle_timeout_s')
        if self.target_detection_retry_period_s >= \
                self.target_detection_window_s:
            raise ValueError(
                'target_detection_retry_period_s must be less than '
                'target_detection_window_s')

    def _odom_callback(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        with self._telemetry_condition:
            self._odom_sequence += 1
            self._odom_received = time.monotonic()
            self._linear_speed = vector_norm(linear.x, linear.y, linear.z)
            self._angular_speed = vector_norm(angular.x, angular.y, angular.z)
            self._telemetry_condition.notify_all()

    def _pick_state_callback(self, message: String) -> None:
        with self._telemetry_condition:
            self._pick_sequence += 1
            self._pick_state = str(message.data)
            self._pick_state_received = time.monotonic()
            self._telemetry_condition.notify_all()

    def get_odom_snapshot(self) -> tuple[int, float, float, float]:
        with self._telemetry_condition:
            return (
                self._odom_sequence,
                self._odom_received,
                self._linear_speed,
                self._angular_speed,
            )

    def get_pick_snapshot(self) -> tuple[int, str, float]:
        with self._telemetry_condition:
            return (
                self._pick_sequence,
                self._pick_state,
                self._pick_state_received,
            )

    def wait_for_telemetry(self, timeout_s: float) -> None:
        with self._telemetry_condition:
            self._telemetry_condition.wait(timeout=float(timeout_s))

    def has_fresh_odom(self, stale_timeout_s: float) -> bool:
        sequence, received, _, _ = self.get_odom_snapshot()
        return sequence > 0 and time.monotonic() - received <= stale_timeout_s

    def pick_state_publisher_available(self) -> bool:
        return self.count_publishers(self.execution_state_topic) > 0

    def transform_available(self) -> bool:
        try:
            return self._tf_buffer.can_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return False

    def capture_base_pose(self) -> PoseStamped:
        transform = self._tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.5),
        )
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _target_pose(self) -> PoseStamped:
        x, y, yaw = self.target_pose_xyyaw
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _start_callback(self, _request, response):
        with self._mission_lock:
            if self._mission_running or self._state != 'IDLE':
                response.success = False
                response.message = (
                    f'mission is {self._state}; cancel or reset before '
                    'starting')
                return response
            if not self.target_pose_configured:
                response.success = False
                response.message = (
                    'target_pose_configured=false; set a verified map-frame '
                    'target_pose_xyyaw before starting')
                return response

            blackboard = Blackboard()
            blackboard['target_pose'] = self._target_pose()
            self._blackboard = blackboard
            self._cancel_requested.clear()
            self._mission_running = True
            self._mission_thread = threading.Thread(
                target=self._run_mission,
                args=(blackboard,),
                name='nav-grasp-mission-worker',
                daemon=True,
            )
            self._mission_thread.start()

        response.success = True
        response.message = 'navigation-grasp-return mission started'
        self.emit_event('START', response.message)
        return response

    def _cancel_callback(self, _request, response):
        with self._mission_lock:
            running = self._mission_running
            current_state = self._state
        if not running:
            response.success = False
            response.message = 'no mission is running'
            return response

        self._cancel_requested.set()
        self._machine.cancel_state()
        self.stop_base()
        response.success = True
        if current_state in {'PLAN_PICK', 'EXECUTE_PICK', 'WAIT_PICK_RESULT'}:
            response.message = (
                'mission cancellation requested; the grasp subsystem is not '
                'interrupted and return navigation will not start')
        else:
            response.message = (
                'mission and any active Nav2 goal cancellation requested')
        self.emit_event('CANCEL', response.message)
        return response

    def _reset_callback(self, _request, response):
        with self._mission_lock:
            if self._mission_running:
                response.success = False
                response.message = 'cannot reset while the mission is running'
                return response
            self._blackboard = None
            self._cancel_requested.clear()
        self.set_state(
            'IDLE', 'mission state reset; subsystem state is unchanged')
        response.success = True
        response.message = (
            'mission reset to IDLE; this does not reset or move the grasp '
            'subsystem')
        self.emit_event('RESET', response.message)
        return response

    def _status_callback(self, _request, response):
        with self._mission_lock:
            state = self._state
            running = self._mission_running
            detail = self._state_detail
        response.success = state != 'ERROR'
        response.message = f'state={state}; running={running}; detail={detail}'
        return response

    def mission_cancel_requested(self) -> bool:
        """Return whether cancellation was requested for the current run."""
        return self._cancel_requested.is_set()

    def _run_mission(self, blackboard: Blackboard) -> None:
        try:
            outcome = self._machine(blackboard)
            if outcome == MISSION_SUCCEEDED:
                self.set_state(
                    'DONE',
                    'grasp completed and robot returned to the captured '
                    'start pose')
                self.emit_event('DONE', self._state_detail)
            elif outcome == MISSION_CANCELED:
                self.set_state(
                    'CANCELED',
                    'mission canceled; no later stage was started')
                self.emit_event('CANCELED', self._state_detail)
            else:
                detail = self._blackboard_error(blackboard)
                self.set_state('ERROR', detail)
                self.emit_event('ERROR', detail)
                self.stop_base()
        except Exception as error:
            detail = f'unhandled mission exception: {error}'
            self.get_logger().error(detail)
            self.set_state('ERROR', detail)
            self.emit_event('ERROR', detail)
            self.stop_base()
        finally:
            with self._mission_lock:
                self._mission_running = False

    @staticmethod
    def _blackboard_error(blackboard: Blackboard) -> str:
        if blackboard.contains('last_error'):
            return str(blackboard['last_error'])
        return 'mission aborted without a subsystem error message'

    def _on_machine_start(
        self,
        _blackboard: Blackboard,
        state_name: str,
    ) -> None:
        if state_name in STATE_ORDER:
            self.set_state(state_name, f'entered {state_name}')

    def _on_machine_transition(
        self,
        _blackboard: Blackboard,
        _from_state: str,
        _outcome: str,
        to_state: str,
    ) -> None:
        if to_state in STATE_ORDER:
            self.set_state(to_state, f'entered {to_state}')

    def set_state(self, state: str, detail: str) -> None:
        with self._mission_lock:
            self._state = str(state)
            self._state_detail = str(detail)
        self._publish_state()
        self.get_logger().info(f'Mission state: {state} - {detail}')

    def _publish_state(self) -> None:
        try:
            self._state_publisher.publish(String(data=self._state))
        except Exception:
            pass

    def emit_event(self, code: str, message: str) -> None:
        text = f'{code}|{message}'
        with self._mission_lock:
            self._state_detail = str(message)
        try:
            self._event_publisher.publish(String(data=text))
        except Exception:
            pass
        self.get_logger().info(text)

    def stop_base(self) -> None:
        try:
            self._stop_publisher.publish(Twist())
        except Exception:
            pass

    def stop_worker(self) -> None:
        with self._mission_lock:
            running = self._mission_running
            worker = self._mission_thread
        if running:
            self._cancel_requested.set()
            self._machine.cancel_state()
            self.stop_base()
        if worker is not None and worker.is_alive():
            worker.join(timeout=5.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavGraspMissionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_worker()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
