import math

import pytest

from visual_grasp_motion.motion_math import (
    horizontal_axis_yaw,
    plane_height_at_xy,
    position_distance,
    quaternion_distance_rad,
    rotate_orientation_about_base_z,
    rotate_vector,
    square_alignment_delta,
    tcp_to_flange,
    tool_down_tilt_rad,
    step_toward,
)


def test_tcp_offset_for_tool_down_pose_moves_flange_up():
    flange_position, flange_quaternion = tcp_to_flange(
        (0.30, 0.0, 0.01),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 0.142),
    )
    assert flange_position == pytest.approx((0.30, 0.0, 0.152), abs=1e-9)
    assert flange_quaternion == pytest.approx((0.0, 1.0, 0.0, 0.0))


def test_identity_tcp_offset_moves_flange_back_along_base_z():
    flange_position, _ = tcp_to_flange(
        (0.30, 0.10, 0.20),
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.142),
    )
    assert flange_position == pytest.approx((0.30, 0.10, 0.058), abs=1e-9)


def test_rotation_and_tool_down_tilt():
    assert rotate_vector(
        (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0)
    ) == pytest.approx((0.0, 0.0, -1.0), abs=1e-9)
    assert tool_down_tilt_rad((0.0, 1.0, 0.0, 0.0)) == pytest.approx(0.0)
    assert tool_down_tilt_rad((0.0, 0.0, 0.0, 1.0)) == pytest.approx(math.pi)


def test_quaternion_sign_is_equivalent():
    assert quaternion_distance_rad(
        (0.0, 1.0, 0.0, 0.0), (0.0, -1.0, 0.0, 0.0)
    ) == pytest.approx(0.0)


def test_position_distance():
    assert position_distance((0.0, 0.0, 0.0), (0.03, 0.04, 0.0)) \
        == pytest.approx(0.05)


def test_tilted_support_plane_height():
    assert plane_height_at_xy(
        0.435512, 0.018063,
        (0.260290, -0.009180, -0.004454),
        (-0.136599, -0.004044),
    ) == pytest.approx(-0.02849, abs=1e-5)


def test_step_toward_without_overshoot():
    assert step_toward(0.050, 0.0, 0.004) == pytest.approx(0.046)
    assert step_toward(0.003, 0.0, 0.004) == pytest.approx(0.0)
    assert step_toward(0.0, 0.050, 0.004) == pytest.approx(0.004)
    with pytest.raises(ValueError):
        step_toward(0.0, 1.0, 0.0)


def test_square_alignment_uses_smallest_equivalent_wrist_rotation():
    nominal_tool_down = (0.0, 1.0, 0.0, 0.0)
    nominal_axis_yaw = horizontal_axis_yaw(nominal_tool_down)

    plus_twenty = square_alignment_delta(
        math.radians(20.0), nominal_axis_yaw)
    minus_ten = square_alignment_delta(
        math.radians(80.0), nominal_axis_yaw)

    assert math.degrees(plus_twenty) == pytest.approx(20.0)
    assert math.degrees(minus_ten) == pytest.approx(-10.0)
    aligned = rotate_orientation_about_base_z(
        nominal_tool_down, plus_twenty)
    assert math.degrees(horizontal_axis_yaw(aligned)) == pytest.approx(-160.0)
    assert tool_down_tilt_rad(aligned) == pytest.approx(0.0)
