"""OpenCV eye-in-hand solver plus closed-loop quality scoring."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List

import cv2
import numpy as np

from .geometry import average_rotation, make_transform, rotation_angle


@dataclass
class HandEyeSolution:
    method: str
    flange_T_camera: np.ndarray
    translation_rms_m: float
    rotation_rms_deg: float
    score: float


METHODS = {
    'tsai': cv2.CALIB_HAND_EYE_TSAI,
    'park': cv2.CALIB_HAND_EYE_PARK,
    'horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
    'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def evaluate_solution(
    base_T_flange: Iterable[np.ndarray],
    camera_T_target: Iterable[np.ndarray],
    flange_T_camera: np.ndarray,
) -> tuple[float, float, float]:
    base_T_targets = [
        base_T_gripper @ flange_T_camera @ camera_T_board
        for base_T_gripper, camera_T_board in zip(base_T_flange, camera_T_target)
    ]
    translations = np.asarray([item[:3, 3] for item in base_T_targets])
    translation_mean = np.mean(translations, axis=0)
    translation_rms = float(np.sqrt(np.mean(np.sum(
        (translations - translation_mean) ** 2, axis=1))))

    mean_rotation = average_rotation(item[:3, :3] for item in base_T_targets)
    angles = np.asarray([
        rotation_angle(mean_rotation.T @ item[:3, :3])
        for item in base_T_targets
    ])
    rotation_rms_rad = float(np.sqrt(np.mean(angles ** 2)))
    rotation_rms_deg = float(np.degrees(rotation_rms_rad))

    # A 1 degree rotational residual is weighted like roughly 1 mm.
    score = translation_rms + 0.001 * rotation_rms_deg
    return translation_rms, rotation_rms_deg, score


def solve_all(
    base_T_flange: List[np.ndarray], camera_T_target: List[np.ndarray]
) -> Dict[str, HandEyeSolution]:
    if len(base_T_flange) != len(camera_T_target):
        raise ValueError('Robot and camera sample counts differ')
    if len(base_T_flange) < 3:
        raise ValueError('At least three samples are required by OpenCV')

    rotations_gripper_to_base = [item[:3, :3] for item in base_T_flange]
    translations_gripper_to_base = [item[:3, 3].reshape(3, 1) for item in base_T_flange]
    rotations_target_to_camera = [item[:3, :3] for item in camera_T_target]
    translations_target_to_camera = [item[:3, 3].reshape(3, 1) for item in camera_T_target]

    solutions: Dict[str, HandEyeSolution] = {}
    for name, method in METHODS.items():
        try:
            rotation, translation = cv2.calibrateHandEye(
                rotations_gripper_to_base,
                translations_gripper_to_base,
                rotations_target_to_camera,
                translations_target_to_camera,
                method=method,
            )
        except cv2.error:
            continue
        if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
            continue
        if abs(float(np.linalg.det(rotation)) - 1.0) > 1e-3:
            continue

        transform = make_transform(rotation, translation.reshape(3))
        translation_rms, rotation_rms_deg, score = evaluate_solution(
            base_T_flange, camera_T_target, transform)
        solutions[name] = HandEyeSolution(
            method=name,
            flange_T_camera=transform,
            translation_rms_m=translation_rms,
            rotation_rms_deg=rotation_rms_deg,
            score=score,
        )
    return solutions


def best_solution(
    base_T_flange: List[np.ndarray], camera_T_target: List[np.ndarray]
) -> tuple[HandEyeSolution, Dict[str, HandEyeSolution]]:
    solutions = solve_all(base_T_flange, camera_T_target)
    if not solutions:
        raise RuntimeError('Every OpenCV hand-eye method failed')
    return min(solutions.values(), key=lambda item: item.score), solutions


def motion_diversity(base_T_flange: List[np.ndarray]) -> tuple[float, float]:
    max_translation = 0.0
    max_rotation = 0.0
    for first_index, first in enumerate(base_T_flange):
        for second in base_T_flange[first_index + 1:]:
            max_translation = max(
                max_translation,
                float(np.linalg.norm(first[:3, 3] - second[:3, 3])),
            )
            max_rotation = max(
                max_rotation,
                rotation_angle(first[:3, :3].T @ second[:3, :3]),
            )
    return max_translation, float(np.degrees(max_rotation))
