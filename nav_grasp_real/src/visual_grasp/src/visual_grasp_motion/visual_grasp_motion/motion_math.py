"""Pure geometry helpers used by the guarded PiPER executor."""

from __future__ import annotations

import math
from typing import Iterable, Tuple


def normalize_quaternion(quaternion: Iterable[float]) -> Tuple[float, ...]:
    values = tuple(float(value) for value in quaternion)
    if len(values) != 4:
        raise ValueError('A quaternion must contain four values')
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        raise ValueError('Quaternion has zero length')
    return tuple(value / norm for value in values)


def multiply_quaternions(
    first_xyzw: Iterable[float], second_xyzw: Iterable[float]
) -> Tuple[float, ...]:
    """Compose two ROS-order quaternions as first * second."""
    ax, ay, az, aw = normalize_quaternion(first_xyzw)
    bx, by, bz, bw = normalize_quaternion(second_xyzw)
    return normalize_quaternion((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def rotate_orientation_about_base_z(
    quaternion_xyzw: Iterable[float], angle_rad: float
) -> Tuple[float, ...]:
    """Pre-multiply an orientation by a rotation about the base-frame Z."""
    half_angle = 0.5 * float(angle_rad)
    base_z_rotation = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))
    return multiply_quaternions(base_z_rotation, quaternion_xyzw)


def horizontal_axis_yaw(
    quaternion_xyzw: Iterable[float], axis_xyz: Iterable[float] = (1.0, 0.0, 0.0)
) -> float:
    """Return the base-frame yaw of a rotated tool axis."""
    axis_in_base = rotate_vector(quaternion_xyzw, axis_xyz)
    if math.hypot(axis_in_base[0], axis_in_base[1]) < 1e-9:
        raise ValueError('The selected tool axis has no horizontal projection')
    return math.atan2(axis_in_base[1], axis_in_base[0])


def square_alignment_delta(
    object_axis_yaw_rad: float,
    tool_axis_yaw_rad: float,
    alignment_offset_rad: float = 0.0,
) -> float:
    """Smallest yaw correction for a square, whose axes repeat every 90 deg."""
    period = 0.5 * math.pi
    difference = (
        float(object_axis_yaw_rad) + float(alignment_offset_rad)
        - float(tool_axis_yaw_rad)
    )
    return (difference + 0.5 * period) % period - 0.5 * period


def rotate_vector(
    quaternion_xyzw: Iterable[float], vector_xyz: Iterable[float]
) -> Tuple[float, float, float]:
    """Rotate a vector using an ROS-order (x, y, z, w) quaternion."""
    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    vx, vy, vz = (float(value) for value in vector_xyz)
    # q * [v, 0] * conjugate(q), written without temporary quaternions.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def tcp_to_flange(
    tcp_position_xyz: Iterable[float],
    tcp_quaternion_xyzw: Iterable[float],
    flange_to_tcp_xyz: Iterable[float],
) -> tuple[Tuple[float, float, float], Tuple[float, ...]]:
    """Convert a desired TCP pose to a flange pose.

    The current installation has no rotational TCP offset.  The translation
    is expressed in the flange frame, so it must first be rotated into the
    base frame before it is subtracted from the desired TCP position.
    """
    tcp_position = tuple(float(value) for value in tcp_position_xyz)
    if len(tcp_position) != 3:
        raise ValueError('TCP position must contain three values')
    quaternion = normalize_quaternion(tcp_quaternion_xyzw)
    offset_in_base = rotate_vector(quaternion, flange_to_tcp_xyz)
    flange_position = tuple(
        tcp_value - offset_value
        for tcp_value, offset_value in zip(tcp_position, offset_in_base)
    )
    return flange_position, quaternion


def position_distance(first: Iterable[float], second: Iterable[float]) -> float:
    return math.sqrt(sum(
        (float(a_value) - float(b_value)) ** 2
        for a_value, b_value in zip(first, second)
    ))


def plane_height_at_xy(
        x: float, y: float, reference_xyz: Iterable[float],
        slope_xy: Iterable[float]) -> float:
    """Return z on a plane expressed as z0 + dz/dx*dx + dz/dy*dy."""
    x0, y0, z0 = (float(value) for value in reference_xyz)
    dz_dx, dz_dy = (float(value) for value in slope_xy)
    return z0 + dz_dx * (float(x) - x0) + dz_dy * (float(y) - y0)


def step_toward(current: float, target: float, maximum_step: float) -> float:
    """Move a scalar toward a target without overshooting it."""
    if maximum_step <= 0.0:
        raise ValueError('maximum_step must be positive')
    delta = float(target) - float(current)
    if abs(delta) <= maximum_step:
        return float(target)
    return float(current) + math.copysign(maximum_step, delta)


def quaternion_distance_rad(
    first_xyzw: Iterable[float], second_xyzw: Iterable[float]
) -> float:
    first = normalize_quaternion(first_xyzw)
    second = normalize_quaternion(second_xyzw)
    dot = abs(sum(a_value * b_value for a_value, b_value in zip(first, second)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def tool_down_tilt_rad(quaternion_xyzw: Iterable[float]) -> float:
    """Return the angle between TCP +Z and base -Z."""
    tool_z = rotate_vector(quaternion_xyzw, (0.0, 0.0, 1.0))
    cosine = max(-1.0, min(1.0, -tool_z[2]))
    return math.acos(cosine)
