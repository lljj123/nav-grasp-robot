"""Unit tests for ROS-independent mission helpers."""

import math

import pytest

from nav_grasp_mission.logic import (
    execution_state_token,
    is_pick_terminal,
    is_transient_detection_failure,
    validate_xyyaw,
    vector_norm,
    yaw_to_quaternion,
)


def test_validate_xyyaw_accepts_three_finite_values():
    assert validate_xyyaw([1, -2.5, 0.25]) == (1.0, -2.5, 0.25)


@pytest.mark.parametrize('values', ([1.0, 2.0], [1.0, 2.0, 3.0, 4.0]))
def test_validate_xyyaw_rejects_wrong_length(values):
    with pytest.raises(ValueError):
        validate_xyyaw(values)


@pytest.mark.parametrize('bad_value', (math.inf, -math.inf, math.nan))
def test_validate_xyyaw_rejects_non_finite_values(bad_value):
    with pytest.raises(ValueError):
        validate_xyyaw([0.0, bad_value, 0.0])


def test_planar_yaw_quaternion():
    qx, qy, qz, qw = yaw_to_quaternion(math.pi)
    assert qx == 0.0
    assert qy == 0.0
    assert qz == pytest.approx(1.0)
    assert qw == pytest.approx(0.0, abs=1e-12)


def test_vector_norm_and_invalid_input():
    assert vector_norm(3.0, 4.0, 12.0) == pytest.approx(13.0)
    assert math.isinf(vector_norm(math.nan, 0.0, 0.0))


@pytest.mark.parametrize(
    ('message', 'token'),
    (
        ('SUCCEEDED: carry posture reached', 'SUCCEEDED'),
        (' error : motion timed out', 'ERROR'),
        ('OPENING', 'OPENING'),
        ('', ''),
    ),
)
def test_execution_state_token(message, token):
    assert execution_state_token(message) == token


def test_terminal_pick_states_are_exact():
    assert is_pick_terminal('SUCCEEDED: done')
    assert is_pick_terminal('ERROR: stopped')
    assert not is_pick_terminal('LIFTING: active')


@pytest.mark.parametrize(
    'message',
    (
        'No cube detections available',
        'Detections are stale (0.72 s)',
        'No any detection inside configured workspace',
        'No red detection inside configured workspace',
    ),
)
def test_transient_detection_failures_are_retryable(message):
    assert is_transient_detection_failure(message)


def test_invalid_corrected_target_is_not_a_transient_detection_failure():
    assert not is_transient_detection_failure(
        'Corrected grasp target is outside configured workspace')
