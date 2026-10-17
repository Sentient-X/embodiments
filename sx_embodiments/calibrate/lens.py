"""A camera's intrinsics as the solvers use them: project camera points, find a target's pose.

One model per wire distortion name (:data:`~sx_embodiments.calibration.DISTORTION_COEFFICIENTS`):
``none``; ``plumb_bob`` and ``rational_polynomial``, OpenCV's Brown-Conrady (``k1 k2 p1 p2 k3``
and ``k1 k2 p1 p2 k3 k4 k5 k6``); ``equidistant`` and ``kannala_brandt``, OpenCV's four-
coefficient fisheye. The projection is OpenCV's, written vectorised so a solve can evaluate
thousands of corners per step.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from numpy.typing import NDArray

from ..calibration import DISTORTION_COEFFICIENTS, CameraUnitCalibration
from ..parts import CameraOptics
from . import _rigid
from .targets import CalibrationSolveError, Detection, Target

FISHEYE_MODELS = frozenset({"equidistant", "kannala_brandt"})


@dataclass(frozen=True, slots=True, eq=False)
class Lens:
    width: int
    height: int
    k: NDArray[np.float64]
    model: str
    coefficients: NDArray[np.float64]

    def __post_init__(self) -> None:
        expected = DISTORTION_COEFFICIENTS.get(self.model)
        if expected is None:
            raise CalibrationSolveError(f"unknown distortion model {self.model!r}")
        if self.k.shape != (3, 3) or self.coefficients.shape != (expected,):
            raise CalibrationSolveError(
                f"a {self.model} lens is a 3x3 matrix and {expected} coefficients"
            )

    @classmethod
    def of(cls, source: CameraUnitCalibration | CameraOptics) -> Lens:
        return cls(
            source.width,
            source.height,
            np.asarray(source.image_from_camera, dtype=np.float64).reshape(3, 3),
            source.distortion_model,
            np.asarray(source.distortion_coefficients, dtype=np.float64),
        )

    @property
    def fisheye(self) -> bool:
        return self.model in FISHEYE_MODELS

    def project(self, points: NDArray[np.float64]) -> NDArray[np.float64]:
        """Camera-frame points ``(n, 3)`` to pixels ``(n, 2)``; points behind the camera are NaN."""
        z = points[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            x = np.where(z > 1e-9, points[:, 0] / z, np.nan)
            y = np.where(z > 1e-9, points[:, 1] / z, np.nan)
        d = self.coefficients
        if self.fisheye:
            radius = np.sqrt(x * x + y * y)
            theta = np.arctan(radius)
            t2 = theta * theta
            distorted = theta * (1.0 + d[0] * t2 + d[1] * t2**2 + d[2] * t2**3 + d[3] * t2**4)
            scale = np.where(radius > 1e-12, distorted / np.maximum(radius, 1e-12), 1.0)
            u, v = x * scale, y * scale
        elif self.model == "none":
            u, v = x, y
        else:
            r2 = x * x + y * y
            radial = 1.0 + d[0] * r2 + d[1] * r2**2 + d[4] * r2**3
            if self.model == "rational_polynomial":
                radial = radial / (1.0 + d[5] * r2 + d[6] * r2**2 + d[7] * r2**3)
            u = x * radial + 2.0 * d[2] * x * y + d[3] * (r2 + 2.0 * x * x)
            v = y * radial + d[2] * (r2 + 2.0 * y * y) + 2.0 * d[3] * x * y
        k = self.k
        return np.column_stack((k[0, 0] * u + k[0, 1] * v + k[0, 2], k[1, 1] * v + k[1, 2]))

    def normalized(self, pixels: NDArray[np.float64]) -> NDArray[np.float64]:
        """Pixels to ideal (undistorted, unit-focal) image coordinates."""
        observed = pixels.reshape(-1, 1, 2)
        if self.model == "none":
            homogeneous = np.column_stack((pixels.reshape(-1, 2), np.ones(observed.shape[0])))
            ideal = (np.linalg.inv(self.k) @ homogeneous.T).T
            return ideal[:, :2] / ideal[:, 2:3]
        if self.fisheye:
            out = cv2.fisheye.undistortPoints(observed, self.k, self.coefficients.reshape(-1, 1))
        else:
            out = cv2.undistortPoints(observed, self.k, self.coefficients.reshape(-1, 1))
        return np.asarray(out, dtype=np.float64).reshape(-1, 2)

    def target_pose(self, detection: Detection, target: Target) -> _rigid.Matrix:
        """``camera_from_target`` from one detection: undistort, then perspective-n-point."""
        if detection.ids.shape[0] < 4:
            raise CalibrationSolveError("a target pose needs at least four corners")
        ok, rvec, tvec = cv2.solvePnP(
            target.points()[detection.ids],
            self.normalized(detection.pixels),
            np.eye(3, dtype=np.float64),
            np.zeros(5, dtype=np.float64),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            raise CalibrationSolveError("no target pose fits a detected view")
        return _rigid.from_vector(
            np.concatenate(
                (
                    np.asarray(rvec, dtype=np.float64).reshape(3),
                    np.asarray(tvec, dtype=np.float64).reshape(3),
                )
            )
        )
