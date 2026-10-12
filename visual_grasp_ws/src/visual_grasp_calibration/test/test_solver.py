import cv2
import numpy as np

from visual_grasp_calibration.geometry import make_transform, rotation_angle
from visual_grasp_calibration.solver import best_solution


def random_transform(rng, translation_scale=0.35):
    rvec = rng.normal(size=3)
    rvec *= rng.uniform(0.2, 1.2) / np.linalg.norm(rvec)
    rotation, _ = cv2.Rodrigues(rvec)
    translation = rng.uniform(-translation_scale, translation_scale, size=3)
    return make_transform(rotation, translation)


def test_synthetic_eye_in_hand_solution():
    rng = np.random.default_rng(42)
    flange_T_camera = random_transform(rng, 0.08)
    base_T_target = random_transform(rng, 0.25)
    base_T_target[2, 3] = 0.2

    base_T_flange = []
    camera_T_target = []
    for _ in range(24):
        base_T_gripper = random_transform(rng, 0.3)
        base_T_flange.append(base_T_gripper)
        camera_T_target.append(
            np.linalg.inv(flange_T_camera)
            @ np.linalg.inv(base_T_gripper)
            @ base_T_target)

    selected, _ = best_solution(base_T_flange, camera_T_target)
    translation_error = np.linalg.norm(
        selected.flange_T_camera[:3, 3] - flange_T_camera[:3, 3])
    rotation_error = rotation_angle(
        selected.flange_T_camera[:3, :3].T @ flange_T_camera[:3, :3])
    assert translation_error < 1e-5
    assert rotation_error < 1e-5
