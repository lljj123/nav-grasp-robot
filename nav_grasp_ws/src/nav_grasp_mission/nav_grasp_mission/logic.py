"""ROS-independent validation helpers for the mission coordinator."""

from __future__ import annotations

import math
from collections.abc import Sequence


TERMINAL_PICK_STATES = {'SUCCEEDED', 'ERROR'}


def validate_xyyaw(values: Sequence[float]) -> tuple[float, float, float]:
    """Validate and normalize a planar map pose."""
    if len(values) != 3:
        raise ValueError('target_pose_xyyaw must contain exactly [x, y, yaw]')
    pose = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in pose):
        raise ValueError('target_pose_xyyaw must contain only finite values')
    return pose


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Return an XYZW quaternion for a planar yaw angle."""
    if not math.isfinite(yaw):
        raise ValueError('yaw must be finite')
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def vector_norm(x: float, y: float, z: float) -> float:
    """Return a finite three-dimensional vector norm."""
    values = (float(x), float(y), float(z))
    if not all(math.isfinite(value) for value in values):
        return math.inf
    return math.sqrt(sum(value * value for value in values))


def execution_state_token(message: str) -> str:
    """Extract the executor state token from ``STATE: detail`` text."""
    return str(message).partition(':')[0].strip().upper()


def is_pick_terminal(message: str) -> bool:
    """Return whether an execution-state message is terminal."""
    return execution_state_token(message) in TERMINAL_PICK_STATES


def is_transient_detection_failure(message: str) -> bool:
    """Return whether a grasp-plan rejection should be retried briefly."""
    text = str(message).strip().lower()
    return (
        'no cube detection' in text
        or 'detections are stale' in text
        or ('no ' in text and ' detection inside configured workspace' in text)
    )
