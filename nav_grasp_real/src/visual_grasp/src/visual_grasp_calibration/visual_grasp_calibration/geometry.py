"""Small SE(3) helpers with ROS quaternion order x, y, z, w."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np


def quaternion_to_rotation(quaternion: Iterable[float]) -> np.ndarray:
    x, y, z, w = np.asarray(list(quaternion), dtype=np.float64)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('Quaternion has zero length')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def rotation_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError('Rotation matrix must be 3x3')

    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale

    quaternion = np.array([x, y, z, w], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return quaternion


def make_transform(rotation: np.ndarray, translation: Iterable[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64)
    transform[:3, 3] = np.asarray(list(translation), dtype=np.float64).reshape(3)
    return transform


def xyz_quaternion_to_matrix(
    translation: Iterable[float], quaternion: Iterable[float]
) -> np.ndarray:
    return make_transform(quaternion_to_rotation(quaternion), translation)


def matrix_to_xyz_quaternion(transform: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    matrix = np.asarray(transform, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError('Transform must be 4x4')
    return matrix[:3, 3].copy(), rotation_to_quaternion(matrix[:3, :3])


def transform_message_to_matrix(message) -> np.ndarray:
    translation = message.transform.translation
    rotation = message.transform.rotation
    return xyz_quaternion_to_matrix(
        [translation.x, translation.y, translation.z],
        [rotation.x, rotation.y, rotation.z, rotation.w],
    )


def rotation_angle(rotation: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return math.acos(cosine)


def average_rotation(rotations: Iterable[np.ndarray]) -> np.ndarray:
    summed = np.sum(np.asarray(list(rotations), dtype=np.float64), axis=0)
    u_matrix, _, vt_matrix = np.linalg.svd(summed)
    mean = u_matrix @ vt_matrix
    if np.linalg.det(mean) < 0.0:
        u_matrix[:, -1] *= -1.0
        mean = u_matrix @ vt_matrix
    return mean
