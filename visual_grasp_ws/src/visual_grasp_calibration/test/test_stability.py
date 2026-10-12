import math

import numpy as np

from visual_grasp_calibration.stability import pose_motion_span


def _pose(x=0.0, yaw_deg=0.0):
    angle = math.radians(yaw_deg)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    transform = np.eye(4)
    transform[:3, :3] = [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]
    transform[0, 3] = x
    return transform


def test_pose_motion_span_reports_pairwise_extremes():
    translation_m, rotation_deg = pose_motion_span([
        _pose(x=-0.0005, yaw_deg=-0.2),
        _pose(),
        _pose(x=0.0005, yaw_deg=0.2),
    ])
    assert math.isclose(translation_m, 0.001, abs_tol=1e-12)
    assert math.isclose(rotation_deg, 0.4, abs_tol=1e-8)


def test_pose_motion_span_rejects_empty_input():
    try:
        pose_motion_span([])
    except ValueError:
        return
    raise AssertionError('empty input must raise ValueError')
