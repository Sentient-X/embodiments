"""A calibration board reserves marker IDs without changing physical corner IDs."""

from __future__ import annotations

import cv2
import numpy as np
import pytest

from sx_embodiments.calibrate import CalibrationSolveError, CharucoBoard


def test_default_board_matches_vendor_image_and_keeps_minimum_corners_positional() -> None:
    board = CharucoBoard(5, 7, 0.025, 0.018, "DICT_4X4_50", 8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    vendor = cv2.aruco.CharucoBoard((5, 7), 0.025, 0.018, dictionary)
    assert board.minimum_corners == 8 and board.marker_start_id == 0
    np.testing.assert_array_equal(board.image(500, 700), vendor.generateImage((500, 700)))
    np.testing.assert_array_equal(board.points(), vendor.getChessboardCorners())


@pytest.mark.parametrize("start", [True, False, -1, 1.0, "1", 34])
def test_invalid_or_overflowing_marker_ranges_are_refused(start) -> None:
    with pytest.raises(CalibrationSolveError, match="marker"):
        CharucoBoard(5, 7, 0.025, 0.018, "DICT_4X4_50", marker_start_id=start)


def test_last_dictionary_range_and_its_foreign_scene_markers_are_handled() -> None:
    board = CharucoBoard(5, 7, 0.025, 0.018, "DICT_4X4_50", marker_start_id=33)
    np.testing.assert_array_equal(board.board().getIds().reshape(-1), np.arange(33, 50))
    image = np.full((900, 900), 255, dtype=np.uint8)
    image[100:800, 40:540] = board.image(500, 700)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    image[100:200, 650:750] = cv2.aruco.generateImageMarker(dictionary, 1, 100)
    image[350:450, 650:750] = cv2.aruco.generateImageMarker(dictionary, 20, 100)
    foreign_only = np.full_like(image, 255)
    foreign_only[100:200, 650:750] = image[100:200, 650:750]
    assert board.detect(foreign_only) is None
    detected = board.detect(image)
    assert detected is not None
    np.testing.assert_array_equal(np.sort(detected.ids), np.arange(24))
    expected = np.column_stack(
        ((detected.ids % 4 + 1) * 100 + 40, (detected.ids // 4 + 1) * 100 + 100)
    )
    np.testing.assert_allclose(detected.pixels, expected, atol=1.0)
