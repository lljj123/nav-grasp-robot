import math

import numpy as np

from visual_grasp_calibration.geometry import (
    matrix_to_xyz_quaternion,
    quaternion_to_rotation,
    xyz_quaternion_to_matrix,
)


def test_quaternion_round_trip():
    quaternion = np.array([0.2, -0.3, 0.1, 0.9], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    transform = xyz_quaternion_to_matrix([0.1, -0.2, 0.3], quaternion)
    translation_out, quaternion_out = matrix_to_xyz_quaternion(transform)
    assert np.allclose(translation_out, [0.1, -0.2, 0.3])
    assert np.allclose(
        quaternion_to_rotation(quaternion_out),
        quaternion_to_rotation(quaternion),
        atol=1e-9,
    )
    assert math.isclose(np.linalg.det(transform[:3, :3]), 1.0, abs_tol=1e-9)
