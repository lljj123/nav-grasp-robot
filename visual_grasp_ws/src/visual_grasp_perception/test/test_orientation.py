import math

import numpy as np
import pytest

from visual_grasp_perception.orientation import box_axis_yaw_in_output


def test_box_axis_is_projected_from_image_into_output_frame():
    box = np.array([
        [10.0, 20.0],
        [110.0, 20.0],
        [110.0, 70.0],
        [10.0, 70.0],
    ])
    assert box_axis_yaw_in_output(
        box, 600.0, 600.0, np.eye(3)) == pytest.approx(0.0)

    output_rotation_camera = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    assert box_axis_yaw_in_output(
        box, 600.0, 600.0, output_rotation_camera
    ) == pytest.approx(0.5 * math.pi)
