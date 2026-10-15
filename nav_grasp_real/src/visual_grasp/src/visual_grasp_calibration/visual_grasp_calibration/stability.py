"""Pose-window checks used to reject hand-eye samples taken while moving."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np


def pose_motion_span(transforms: Iterable[np.ndarray]) -> Tuple[float, float]:
    """Return maximum pairwise translation (m) and rotation (deg)."""
    poses = [np.asarray(transform, dtype=np.float64) for transform in transforms]
    if not poses:
        raise ValueError('At least one transform is required')
    if any(pose.shape != (4, 4) for pose in poses):
        raise ValueError('Every transform must be 4x4')

    translation_span_m = 0.0
    rotation_span_rad = 0.0
    for first_index, first in enumerate(poses):
        for second in poses[first_index + 1:]:
            translation_span_m = max(
                translation_span_m,
                float(np.linalg.norm(first[:3, 3] - second[:3, 3])),
            )
            relative_rotation = first[:3, :3].T @ second[:3, :3]
            cosine = float(np.clip(
                (np.trace(relative_rotation) - 1.0) * 0.5, -1.0, 1.0))
            rotation_span_rad = max(rotation_span_rad, math.acos(cosine))

    return translation_span_m, math.degrees(rotation_span_rad)
