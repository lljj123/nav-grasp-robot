"""Pure orientation helpers for RGB-D block detections."""

from __future__ import annotations

import math

import numpy as np


def box_axis_yaw_in_output(
    box_xy: np.ndarray,
    fx: float,
    fy: float,
    output_rotation_camera: np.ndarray,
) -> float:
    """Project a dominant image-box edge into the output frame and return yaw.

    The image edge is interpreted as a tangent at constant depth.  Its scale
    and sign do not matter because a square is symmetric under 90/180 degree
    rotations; the planner later selects the equivalent axis requiring the
    smallest wrist rotation.
    """
    box = np.asarray(box_xy, dtype=np.float64)
    rotation = np.asarray(output_rotation_camera, dtype=np.float64)
    if box.shape != (4, 2):
        raise ValueError('box_xy must have shape (4, 2)')
    if rotation.shape != (3, 3):
        raise ValueError('output_rotation_camera must have shape (3, 3)')
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('Camera focal lengths must be positive')

    edges = [box[(index + 1) % 4] - box[index] for index in range(4)]
    edge = max(edges, key=lambda value: float(np.dot(value, value)))
    camera_tangent = np.array([edge[0] / fx, edge[1] / fy, 0.0])
    output_tangent = rotation @ camera_tangent
    if math.hypot(output_tangent[0], output_tangent[1]) < 1e-9:
        raise ValueError('Projected box edge has no horizontal component')
    return math.atan2(output_tangent[1], output_tangent[0])
