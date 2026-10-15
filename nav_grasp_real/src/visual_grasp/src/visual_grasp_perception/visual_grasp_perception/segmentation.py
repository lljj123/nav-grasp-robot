"""Pure OpenCV HSV segmentation, kept separate for offline testing."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import cv2
import numpy as np


@dataclass
class ColorRegion:
    color: str
    contour: np.ndarray
    center_uv: Tuple[float, float]
    box: np.ndarray
    angle_deg: float
    area_px: float
    square_ratio: float
    score: float


def default_hsv_ranges() -> Dict[str, List[Tuple[np.ndarray, np.ndarray]]]:
    return {
        'red': [
            # Keep the low-hue red band narrow enough to reject orange
            # fixtures/support boxes (typically H≈8 in the current D435
            # lighting) while retaining the dark-red cubes (H≈2--6).
            (np.array([0, 90, 55]), np.array([6, 255, 255])),
            (np.array([170, 90, 55]), np.array([179, 255, 255])),
        ],
        'green': [
            (np.array([35, 65, 45]), np.array([88, 255, 255])),
        ],
        'blue': [
            (np.array([90, 75, 45]), np.array([138, 255, 255])),
        ],
    }


def color_mask(
    hsv: np.ndarray,
    ranges: List[Tuple[np.ndarray, np.ndarray]],
    morphology_kernel: int,
) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
    kernel_size = max(1, int(morphology_kernel))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_color_regions(
    bgr: np.ndarray,
    min_area_px: float = 100.0,
    max_area_px: float = 30000.0,
    min_square_ratio: float = 0.55,
    morphology_kernel: int = 5,
    hsv_ranges=None,
) -> tuple[List[ColorRegion], Dict[str, np.ndarray]]:
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError('Expected an HxWx3 BGR image')
    ranges_by_color = hsv_ranges or default_hsv_ranges()
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    regions: List[ColorRegion] = []
    masks: Dict[str, np.ndarray] = {}

    for color, ranges in ranges_by_color.items():
        mask = color_mask(hsv, ranges, morphology_kernel)
        masks[color] = mask
        contours_result = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_result[-2]
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area_px or area > max_area_px:
                continue
            rectangle = cv2.minAreaRect(contour)
            (center_u, center_v), (width_px, height_px), angle_deg = rectangle
            longer = max(width_px, height_px)
            shorter = min(width_px, height_px)
            if longer < 1.0:
                continue
            square_ratio = float(shorter / longer)
            if square_ratio < min_square_ratio:
                continue
            fill_ratio = float(np.clip(area / max(width_px * height_px, 1.0), 0.0, 1.0))
            score = float(np.clip(0.65 * square_ratio + 0.35 * fill_ratio, 0.0, 1.0))
            regions.append(ColorRegion(
                color=color,
                contour=contour,
                center_uv=(float(center_u), float(center_v)),
                box=cv2.boxPoints(rectangle),
                angle_deg=float(angle_deg),
                area_px=area,
                square_ratio=square_ratio,
                score=score,
            ))
    regions.sort(key=lambda item: item.score, reverse=True)
    return regions, masks
