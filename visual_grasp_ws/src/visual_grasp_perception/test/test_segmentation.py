import cv2
import numpy as np
import pytest

from visual_grasp_perception.segmentation import detect_color_regions


def test_three_synthetic_colored_squares():
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (60, 80), (140, 160), (0, 0, 255), -1)
    cv2.rectangle(image, (260, 90), (340, 170), (0, 255, 0), -1)
    cv2.rectangle(image, (460, 100), (540, 180), (255, 0, 0), -1)
    regions, _ = detect_color_regions(image, min_area_px=500)
    assert {region.color for region in regions} == {'red', 'green', 'blue'}
    assert all(region.square_ratio > 0.95 for region in regions)


def test_dark_red_cube_is_separated_from_orange_support():
    image = np.full((300, 400, 3), 120, dtype=np.uint8)
    # Representative D435 colours from the current scene: orange support is
    # HSV H=8 and the darker red cube is H=2.
    cv2.rectangle(image, (40, 40), (360, 260), (66, 100, 190), -1)
    cv2.rectangle(image, (175, 125), (225, 175), (45, 51, 151), -1)

    regions, masks = detect_color_regions(image, min_area_px=500)
    red_regions = [region for region in regions if region.color == 'red']

    assert len(red_regions) == 1
    assert red_regions[0].center_uv == pytest.approx((200.0, 150.0), abs=1.0)
    assert np.count_nonzero(masks['red']) < 4000

