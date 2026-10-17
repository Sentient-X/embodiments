"""One camera unit's intrinsics from views of a target: a :class:`CameraUnitCalibration`.

OpenCV fits the model (``cv2.calibrateCamera`` for the pinhole models, ``cv2.fisheye.calibrate``
for the fisheye ones); this keeps what the fit used and why it dropped anything. A view is
dropped only when the fisheye fit names it ill-conditioned or when it is the worst view and
above the per-view limit, never more than the limits allow, so a poor collection fails rather
than being trimmed until it looks good.
"""

from __future__ import annotations

import re
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime

import cv2
import numpy as np
from numpy.typing import NDArray

from ..calibration import (
    DISTORTION_COEFFICIENTS,
    CameraUnitCalibration,
    Evidence,
    EvidenceKind,
    Measurement,
    Residual,
    ResidualUnit,
    UnitSerial,
)
from . import _rigid
from .lens import FISHEYE_MODELS, Lens
from .targets import CalibrationSolveError, Detection, Target

_ILL_CONDITIONED_VIEW = re.compile(r"input array (\d+)")
_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-10)


@dataclass(frozen=True, slots=True)
class IntrinsicsLimits:
    minimum_views: int = 20
    max_view_rms_px: float = 0.75
    max_rejected_fraction: float = 0.20


@dataclass(frozen=True, slots=True, eq=False)
class View:
    """One calibration image's detection, and the sha256 of the image it came from."""

    detection: Detection
    image_sha256: str


@dataclass(frozen=True, slots=True, eq=False)
class IntrinsicsSolve:
    calibration: CameraUnitCalibration
    kept: tuple[int, ...]  # indices into the views given
    rejected: tuple[int, ...]
    per_view_rms_px: tuple[float, ...]  # one per kept view
    camera_from_target: tuple[_rigid.Matrix, ...]  # one per kept view


@dataclass(frozen=True, slots=True, eq=False)
class _Fit:
    lens: Lens
    poses: tuple[_rigid.Matrix, ...]


def _fit(target: Target, views: Sequence[View], width: int, height: int, model: str) -> _Fit:
    objects = [target.points()[view.detection.ids] for view in views]
    images = [view.detection.pixels for view in views]
    if model in FISHEYE_MODELS:
        _rms, k, d, rvecs, tvecs = cv2.fisheye.calibrate(
            [points.reshape(1, -1, 3) for points in objects],
            [pixels.reshape(1, -1, 2) for pixels in images],
            (width, height),
            np.zeros((3, 3), dtype=np.float64),
            np.zeros((4, 1), dtype=np.float64),
            flags=(
                cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                | cv2.fisheye.CALIB_CHECK_COND
                | cv2.fisheye.CALIB_FIX_SKEW
            ),
            criteria=_CRITERIA,
        )
    else:
        flags = {
            "none": cv2.CALIB_FIX_K1
            | cv2.CALIB_FIX_K2
            | cv2.CALIB_FIX_K3
            | cv2.CALIB_ZERO_TANGENT_DIST,
            "plumb_bob": 0,
            "rational_polynomial": cv2.CALIB_RATIONAL_MODEL,
        }[model]
        _rms, k, d, rvecs, tvecs = cv2.calibrateCamera(
            [points.astype(np.float32) for points in objects],
            [pixels.astype(np.float32) for pixels in images],
            (width, height),
            np.eye(3, dtype=np.float64),
            np.zeros(8 if model == "rational_polynomial" else 5, dtype=np.float64),
            flags=flags,
            criteria=_CRITERIA,
        )
    coefficients = np.asarray(d, dtype=np.float64).reshape(-1)[: DISTORTION_COEFFICIENTS[model]]
    lens = Lens(width, height, np.asarray(k, dtype=np.float64), model, coefficients)
    poses = tuple(
        _rigid.from_vector(
            np.concatenate(
                (
                    np.asarray(rvec, dtype=np.float64).reshape(3),
                    np.asarray(tvec, dtype=np.float64).reshape(3),
                )
            )
        )
        for rvec, tvec in zip(rvecs, tvecs, strict=True)
    )
    return _Fit(lens, poses)


def view_errors(
    lens: Lens, target: Target, detection: Detection, camera_from_target: _rigid.Matrix
) -> NDArray[np.float64]:
    """Each corner's reprojection error, in pixels."""
    points = target.points()[detection.ids]
    camera = (camera_from_target[:3, :3] @ points.T).T + camera_from_target[:3, 3]
    delta = lens.project(camera) - detection.pixels
    return np.sqrt(np.sum(delta * delta, axis=1))


def solve_intrinsics(
    target: Target,
    views: Sequence[View],
    *,
    width: int,
    height: int,
    model: str,
    unit: UnitSerial,
    measured_at: datetime,
    limits: IntrinsicsLimits | None = None,
) -> IntrinsicsSolve:
    """Fit ``model`` for ``unit`` at ``width`` x ``height`` from views of ``target``."""
    bounds = limits or IntrinsicsLimits()
    if model not in DISTORTION_COEFFICIENTS:
        raise CalibrationSolveError(f"unknown distortion model {model!r}")
    if len(views) < bounds.minimum_views:
        raise CalibrationSolveError(
            f"{len(views)} views; an intrinsics solve needs at least {bounds.minimum_views}"
        )
    if any(view.detection.ids.shape[0] < 6 for view in views):
        raise CalibrationSolveError("every view shows at least six corners")
    active = list(range(len(views)))
    rejected: list[int] = []
    allowed = max(1, int(len(active) * bounds.max_rejected_fraction))
    while True:
        chosen = [views[index] for index in active]
        try:
            fit = _fit(target, chosen, width, height, model)
        except cv2.error as error:
            match = _ILL_CONDITIONED_VIEW.search(str(error))
            if (
                match is None
                or len(active) <= bounds.minimum_views
                or len(rejected) >= allowed
                or int(match.group(1)) >= len(active)
            ):
                raise CalibrationSolveError(f"the {model} fit failed: {error}") from error
            rejected.append(active.pop(int(match.group(1))))
            continue
        errors = [
            view_errors(fit.lens, target, view.detection, pose)
            for view, pose in zip(chosen, fit.poses, strict=True)
        ]
        per_view = [float(np.sqrt(np.mean(error * error))) for error in errors]
        worst = max(range(len(per_view)), key=per_view.__getitem__)
        if (
            per_view[worst] <= bounds.max_view_rms_px
            or len(active) <= bounds.minimum_views
            or len(rejected) >= allowed
        ):
            break
        rejected.append(active.pop(worst))
    every = np.concatenate(errors)
    calibration = CameraUnitCalibration(
        unit=unit,
        width=width,
        height=height,
        image_from_camera=tuple(float(value) for value in fit.lens.k.reshape(-1)),
        distortion_model=model,
        distortion_coefficients=tuple(float(value) for value in fit.lens.coefficients),
        measurement=Measurement(
            target.method,
            measured_at,
            Residual(float(np.sqrt(np.mean(every * every))), ResidualUnit.PIXEL),
            tuple(
                Evidence(EvidenceKind.IMAGE, views[index].image_sha256) for index in sorted(active)
            ),
        ),
    )
    return IntrinsicsSolve(
        calibration=calibration,
        kept=tuple(active),
        rejected=tuple(sorted(rejected)),
        per_view_rms_px=tuple(per_view),
        camera_from_target=fit.poses,
    )
