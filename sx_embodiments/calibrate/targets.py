"""Calibration targets: where their corners are, and where an image shows them.

A target's corners have ids; a :class:`Detection` is the pixels of the corners one image
shows, by id. A checkerboard shows every corner but cannot say which way round it is (its
detector's order is ambiguous by a half turn), so a solve tries both orders; a ChArUco board's
markers name each corner, so it can show a part of the board and is never ambiguous.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import cast

import cv2
import numpy as np
from numpy.typing import NDArray

from ..calibration import CalibrationError, CalibrationMethod

Gray = NDArray[np.uint8]


class CalibrationSolveError(CalibrationError):
    """The evidence given cannot determine the calibration asked for."""


@dataclass(frozen=True, slots=True, eq=False)
class Detection:
    """The pixels of the target corners one image shows: ``pixels[i]`` is corner ``ids[i]``."""

    ids: NDArray[np.int64]
    pixels: NDArray[np.float64]

    def __post_init__(self) -> None:
        if self.ids.ndim != 1 or self.pixels.shape != (self.ids.shape[0], 2):
            raise CalibrationSolveError("a detection pairs each corner id with one pixel")
        if len(set(self.ids.tolist())) != self.ids.shape[0]:
            raise CalibrationSolveError("a detection names each corner once")
        if not np.all(np.isfinite(self.pixels)):
            raise CalibrationSolveError("detected pixels are finite")

    @classmethod
    def of(cls, pixels: NDArray[np.float64], ids: NDArray[np.int64] | None = None) -> Detection:
        found = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
        return cls(
            np.arange(found.shape[0], dtype=np.int64) if ids is None else ids.astype(np.int64),
            found,
        )

    def reversed(self) -> Detection:
        """The same pixels assigned to the corners in the opposite order."""
        return Detection(self.ids, self.pixels[::-1].copy())


@dataclass(frozen=True, slots=True)
class Checkerboard:
    """Inner-corner grid; corner ``i`` sits at column ``i % columns``, row ``i // columns``."""

    columns: int
    rows: int
    square_m: float

    def __post_init__(self) -> None:
        if self.columns < 2 or self.rows < 2:
            raise CalibrationSolveError("a checkerboard needs at least 2x2 inner corners")
        if not math.isfinite(self.square_m) or self.square_m <= 0.0:
            raise CalibrationSolveError("a checkerboard square has a positive size")

    @property
    def method(self) -> CalibrationMethod:
        return CalibrationMethod.CHECKERBOARD

    @property
    def ambiguous(self) -> bool:
        """Whether a detection's corner order can be the half-turn of the true one."""
        return True

    @property
    def corner_count(self) -> int:
        return self.columns * self.rows

    def points(self) -> NDArray[np.float64]:
        """Every corner in the board frame (z = 0), by id."""
        grid = np.mgrid[0 : self.columns, 0 : self.rows].T.reshape(-1, 2)
        points = np.zeros((self.corner_count, 3), dtype=np.float64)
        points[:, :2] = grid * self.square_m
        return points

    def detect(self, gray: Gray) -> Detection | None:
        """Every inner corner, by the distortion-robust sector detector, or ``None``."""
        found, corners = cv2.findChessboardCornersSB(
            gray,
            (self.columns, self.rows),
            flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
        )
        if not found:
            return None
        return Detection.of(np.asarray(corners, dtype=np.float64))


@dataclass(frozen=True, slots=True)
class CharucoBoard:
    """A chessboard of ``columns`` x ``rows`` squares with an ArUco marker in each white one.

    ``dictionary`` is an OpenCV predefined dictionary name (``"DICT_5X5_100"``). Corner ids are
    OpenCV's: the inner chessboard corners, row by row.
    """

    columns: int
    rows: int
    square_m: float
    marker_m: float
    dictionary: str
    minimum_corners: int = 6

    def __post_init__(self) -> None:
        if self.columns < 3 or self.rows < 3:
            raise CalibrationSolveError("a ChArUco board needs at least 3x3 squares")
        if not (0.0 < self.marker_m < self.square_m) or not math.isfinite(self.square_m):
            raise CalibrationSolveError("a ChArUco marker is smaller than its square")
        if not self.dictionary.startswith("DICT_") or not hasattr(cv2.aruco, self.dictionary):
            raise CalibrationSolveError(f"no ArUco dictionary {self.dictionary!r}")
        if self.minimum_corners < 4:
            raise CalibrationSolveError("a pose needs at least four corners")

    @property
    def method(self) -> CalibrationMethod:
        return CalibrationMethod.CHARUCO

    @property
    def ambiguous(self) -> bool:
        return False

    @property
    def corner_count(self) -> int:
        return (self.columns - 1) * (self.rows - 1)

    def board(self) -> cv2.aruco.CharucoBoard:
        dictionary = cv2.aruco.getPredefinedDictionary(int(getattr(cv2.aruco, self.dictionary)))
        return cv2.aruco.CharucoBoard(
            (self.columns, self.rows), self.square_m, self.marker_m, dictionary
        )

    def points(self) -> NDArray[np.float64]:
        return np.asarray(self.board().getChessboardCorners(), dtype=np.float64).reshape(-1, 3)

    def image(self, width_px: int, height_px: int, margin_px: int = 0) -> Gray:
        """The board as printed, ``width_px`` x ``height_px``."""
        return np.asarray(
            self.board().generateImage((width_px, height_px), marginSize=margin_px),
            dtype=np.uint8,
        )

    def detect(self, gray: Gray) -> Detection | None:
        """The corners whose markers the image shows, or ``None`` below ``minimum_corners``."""
        detector = cv2.aruco.CharucoDetector(self.board())
        corners, found, _marker_corners, _marker_ids = detector.detectBoard(gray)
        ids = cast("NDArray[np.int32] | None", found)  # OpenCV answers None when it saw none
        if ids is None or len(ids) < self.minimum_corners:
            return None
        return Detection.of(
            np.asarray(corners, dtype=np.float64),
            np.asarray(ids, dtype=np.int64).reshape(-1),
        )


Target = Checkerboard | CharucoBoard
