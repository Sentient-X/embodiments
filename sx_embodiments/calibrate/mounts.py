"""Where cameras sit on the links that carry them, from views of a target another link holds.

One model covers eye-in-hand (a wrist camera, a target fixed in the world), eye-to-base (a
fixed camera, a target in the gripper), a head camera watching a target in the hand, and a
tracked rig's cameras watching a static board. Each camera ``c`` rides rigidly on a carrier
link and the target rides rigidly on one holder link; at each view the carrier-from-holder
transform is known (forward kinematics at the recorded joint positions, or a tracked pose):

    camera_from_target = inv(carrier_from_camera[c]) · carrier_from_holder · holder_from_target

The unknowns are ``carrier_from_camera`` per camera and the one ``holder_from_target``; the
residual is every corner's reprojection in every view, under a soft-L1 loss so a few bad
corners cannot steer it. Cameras that saw the same target are solved together, because each
pins the target the others see. A checkerboard's half-turn order ambiguity is resolved per
view against the running target estimate and re-resolved after each solve until stable.
"""

from __future__ import annotations

import importlib
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import Protocol, cast

import cv2
import numpy as np
from numpy.typing import NDArray

from . import _rigid
from .lens import Lens
from .targets import CalibrationSolveError, Detection, Target

#: Corner errors beyond this many pixels lose quadratic weight.
SOFT_L1_SCALE_PX = 1.5
MAX_ORIENTATION_PASSES = 5
#: Below this spread of carrier-from-holder rotations a mount and the target trade off freely.
MIN_ROTATION_SPREAD_RAD = math.radians(10.0)
#: Relative motions must turn about axes at least this far apart for the mount to be seen.
MIN_AXIS_SPREAD_RAD = math.radians(10.0)


@dataclass(frozen=True, slots=True, eq=False)
class MountView:
    """One image: the target's detected corners, and the carrier-from-holder transform then."""

    carrier_from_holder: _rigid.Matrix
    detection: Detection

    def reversed(self) -> MountView:
        return MountView(self.carrier_from_holder, self.detection.reversed())


@dataclass(frozen=True, slots=True, eq=False)
class CameraViews:
    camera: str
    lens: Lens
    views: tuple[MountView, ...]

    def with_views(self, views: Sequence[MountView]) -> CameraViews:
        return CameraViews(self.camera, self.lens, tuple(views))


@dataclass(frozen=True, slots=True)
class SolverStatus:
    success: bool
    message: str
    cost: float
    evaluations: int


@dataclass(frozen=True, slots=True, eq=False)
class MountSolve:
    """The solved mounts and target, with each camera's views in the order the fit used."""

    cameras: tuple[CameraViews, ...]
    holder_from_target: _rigid.Matrix
    carrier_from_camera: Mapping[str, _rigid.Matrix]
    orientation_passes: int
    flipped_views: Mapping[str, int]
    status: SolverStatus

    def per_view_rms_px(self, target: Target) -> dict[str, NDArray[np.float64]]:
        return {
            camera.camera: per_view_rms_px(
                camera, self.holder_from_target, self.carrier_from_camera[camera.camera], target
            )
            for camera in self.cameras
        }


Mounts = Mapping[str, _rigid.Matrix]


def predict(
    view: MountView,
    holder_from_target: _rigid.Matrix,
    carrier_from_camera: _rigid.Matrix,
    target: Target,
    lens: Lens,
) -> NDArray[np.float64]:
    """Where the view's detected corners land under the given mount and target."""
    camera_from_target = (
        _rigid.inverse(carrier_from_camera) @ view.carrier_from_holder @ holder_from_target
    )
    points = target.points()[view.detection.ids]
    camera = (camera_from_target[:3, :3] @ points.T).T + camera_from_target[:3, 3]
    return lens.project(camera)


def _rms(delta: NDArray[np.float64]) -> float:
    return math.sqrt(float((delta * delta).sum(axis=1).mean()))


def per_view_rms_px(
    camera: CameraViews,
    holder_from_target: _rigid.Matrix,
    carrier_from_camera: _rigid.Matrix,
    target: Target,
) -> NDArray[np.float64]:
    return np.asarray(
        [
            _rms(
                predict(view, holder_from_target, carrier_from_camera, target, camera.lens)
                - view.detection.pixels
            )
            for view in camera.views
        ],
        dtype=np.float64,
    )


Objective = Callable[[NDArray[np.float64]], NDArray[np.float64]]


class _LeastSquaresResult(Protocol):
    x: NDArray[np.float64]
    success: bool
    message: str
    cost: float
    nfev: int


class _LeastSquares(Protocol):
    def __call__(
        self,
        fun: Objective,
        x0: NDArray[np.float64],
        *,
        loss: str,
        f_scale: float,
        x_scale: str,
        max_nfev: int,
    ) -> _LeastSquaresResult: ...


def _soft_l1(
    objective: Objective,
    x0: NDArray[np.float64],
    *,
    max_evaluations: int,
) -> tuple[NDArray[np.float64], SolverStatus]:
    """SciPy's trust-region least squares, soft-L1 loss, Jacobian scaling; SciPy ships no stubs,
    so the slice used is pinned by the protocol above."""
    least_squares = cast(_LeastSquares, importlib.import_module("scipy.optimize").least_squares)
    result = least_squares(
        objective,
        x0,
        loss="soft_l1",
        f_scale=SOFT_L1_SCALE_PX,
        x_scale="jac",
        max_nfev=max_evaluations,
    )
    return np.asarray(result.x, dtype=np.float64), SolverStatus(
        bool(result.success), str(result.message), float(result.cost), int(result.nfev)
    )


def _residuals(
    cameras: Sequence[CameraViews],
    holder_from_target: _rigid.Matrix,
    mounts: Mounts,
    target: Target,
) -> NDArray[np.float64]:
    values = [
        predict(view, holder_from_target, mounts[camera.camera], target, camera.lens)
        - view.detection.pixels
        for camera in cameras
        for view in camera.views
    ]
    stacked = np.concatenate(values).reshape(-1)
    return np.nan_to_num(stacked, nan=1e4, posinf=1e4, neginf=-1e4)


def _unpack(
    x: NDArray[np.float64], names: Sequence[str]
) -> tuple[_rigid.Matrix, dict[str, _rigid.Matrix]]:
    return _rigid.from_vector(x[:6]), {
        name: _rigid.from_vector(x[6 + 6 * index : 12 + 6 * index])
        for index, name in enumerate(names)
    }


def _joint_objective(
    cameras: tuple[CameraViews, ...], names: Sequence[str], target: Target
) -> Objective:
    """Residuals of the target (first six) and each named camera's mount (six each)."""

    def objective(x: NDArray[np.float64]) -> NDArray[np.float64]:
        holder_from_target, mounts = _unpack(x, names)
        return _residuals(cameras, holder_from_target, mounts, target)

    return objective


def _target_objective(
    cameras: tuple[CameraViews, ...], mounts: Mounts, target: Target
) -> Objective:
    def objective(x: NDArray[np.float64]) -> NDArray[np.float64]:
        return _residuals(cameras, _rigid.from_vector(x), mounts, target)

    return objective


def _distance_score(first: _rigid.Matrix, second: _rigid.Matrix) -> float:
    translation = float(np.linalg.norm(first[:3, 3] - second[:3, 3]))
    return translation / 0.05 + _rigid.rotation_angle(first, second) / math.radians(15.0)


def orient(
    camera: CameraViews,
    carrier_from_camera: _rigid.Matrix,
    target: Target,
    seed: _rigid.Matrix | None = None,
) -> tuple[CameraViews, _rigid.Matrix]:
    """Each view's corner order nearest the running target estimate, and that estimate.

    Every view proposes a holder-from-target through the given mount by perspective-n-point,
    in each order an ambiguous target allows; without a seed the first view's detected order
    sets the convention.
    """
    if not camera.views:
        raise CalibrationSolveError(f"{camera.camera}: no views to solve from")
    candidates: list[tuple[_rigid.Matrix, _rigid.Matrix | None]] = []
    for view in camera.views:
        holder_from_camera = _rigid.inverse(view.carrier_from_holder) @ carrier_from_camera
        direct = holder_from_camera @ camera.lens.target_pose(view.detection, target)
        reverse = (
            holder_from_camera @ camera.lens.target_pose(view.detection.reversed(), target)
            if target.ambiguous
            else None
        )
        candidates.append((direct, reverse))
    estimate = candidates[0][0] if seed is None else seed
    flags = [False] * len(candidates)
    for _iteration in range(4):
        chosen: list[_rigid.Matrix] = []
        for index, (direct, reverse) in enumerate(candidates):
            flags[index] = reverse is not None and _distance_score(
                estimate, reverse
            ) < _distance_score(estimate, direct)
            chosen.append(reverse if flags[index] and reverse is not None else direct)
        estimate = _rigid.average(chosen)
    views = tuple(
        view.reversed() if flag else view for view, flag in zip(camera.views, flags, strict=True)
    )
    return camera.with_views(views), estimate


def orient_cameras(
    target: Target,
    cameras: Sequence[CameraViews],
    mounts: Mounts,
    seed: _rigid.Matrix | None = None,
) -> tuple[tuple[CameraViews, ...], _rigid.Matrix]:
    """Every camera's views in one target convention, and the target they agree on.

    The first camera (or ``seed``) sets the convention, each later camera is oriented against
    the running estimate, and the target returned is the mean over all cameras.
    """
    oriented: list[CameraViews] = []
    estimates: list[_rigid.Matrix] = []
    running = seed
    for camera in cameras:
        views, estimate = orient(camera, mounts[camera.camera], target, running)
        oriented.append(views)
        estimates.append(estimate)
        running = estimate
    return tuple(oriented), _rigid.average(estimates)


def _reorient(
    cameras: Sequence[CameraViews],
    holder_from_target: _rigid.Matrix,
    mounts: Mounts,
    target: Target,
) -> tuple[tuple[CameraViews, ...], dict[str, int]]:
    """Flip each view to whichever corner order the current solve predicts better."""
    output: list[CameraViews] = []
    flips: dict[str, int] = {}
    for camera in cameras:
        views: list[MountView] = []
        flipped = 0
        for view in camera.views:
            if target.ambiguous:
                projected = predict(
                    view, holder_from_target, mounts[camera.camera], target, camera.lens
                )
                direct = float(np.sum((projected - view.detection.pixels) ** 2))
                reverse = float(np.sum((projected - view.detection.pixels[::-1]) ** 2))
                if reverse < direct:
                    view = view.reversed()
                    flipped += 1
            views.append(view)
        output.append(camera.with_views(views))
        flips[camera.camera] = flipped
    return tuple(output), flips


def hand_eye(camera: CameraViews, target: Target) -> _rigid.Matrix:
    """A closed-form ``carrier_from_camera`` (Park and Martin) to start a solve from.

    OpenCV's ``calibrateHandEye`` solves ``holder_from_carrier · X · camera_from_target``
    constant, which is this model with ``X = carrier_from_camera``. A target whose corner order
    is ambiguous gives each view's pose up to a half turn, so it needs a nominal mount instead.
    """
    if target.ambiguous:
        raise CalibrationSolveError(
            f"{camera.camera}: a checkerboard's corner order is ambiguous; give a nominal mount"
        )
    require_motion(camera)
    holder_from_carrier = [_rigid.inverse(view.carrier_from_holder) for view in camera.views]
    camera_from_target = [camera.lens.target_pose(view.detection, target) for view in camera.views]
    rotation, translation = cv2.calibrateHandEye(
        [pose[:3, :3] for pose in holder_from_carrier],
        [pose[:3, 3].reshape(3, 1) for pose in holder_from_carrier],
        [pose[:3, :3] for pose in camera_from_target],
        [pose[:3, 3].reshape(3, 1) for pose in camera_from_target],
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = np.asarray(rotation, dtype=np.float64)
    out[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return out


def require_motion(camera: CameraViews) -> None:
    """Refuse views that leave the mount unobservable.

    A rotation about one axis commutes with any transform along that axis, so a mount can only
    be seen through relative motions about at least two non-parallel axes (Tsai and Lenz);
    each counts only when it turns at least :data:`MIN_ROTATION_SPREAD_RAD`.
    """
    rotations = [view.carrier_from_holder[:3, :3] for view in camera.views]
    axes: list[NDArray[np.float64]] = []
    for index, first in enumerate(rotations):
        for second in rotations[index + 1 :]:
            vector = _rigid.vector_from_rotation(first.T @ second)
            angle = float(np.linalg.norm(vector))
            if angle >= MIN_ROTATION_SPREAD_RAD:
                axes.append(vector / angle)
    if not axes:
        raise CalibrationSolveError(
            f"{camera.camera}: the views never turn the target "
            f"{math.degrees(MIN_ROTATION_SPREAD_RAD):.0f} degrees relative to the camera's "
            "carrier; the mount is unobservable"
        )
    stacked = np.stack(axes)
    if float(np.min(np.abs(stacked @ stacked.T))) > math.cos(MIN_AXIS_SPREAD_RAD):
        raise CalibrationSolveError(
            f"{camera.camera}: every motion relative to the camera's carrier turns about one "
            f"axis; a mount needs turns about two axes at least "
            f"{math.degrees(MIN_AXIS_SPREAD_RAD):.0f} degrees apart"
        )


def solve_mounts(
    target: Target,
    cameras: Sequence[CameraViews],
    *,
    initial: Mounts | None = None,
    max_evaluations: int = 600,
) -> MountSolve:
    """Every camera's mount and the one target, jointly, re-orienting views until stable.

    ``initial`` gives a starting mount per camera (the description's nominal one); a camera
    without one starts from :func:`hand_eye`.
    """
    if not cameras:
        raise CalibrationSolveError("a mount solve needs at least one camera")
    names = [camera.camera for camera in cameras]
    if len(set(names)) != len(names):
        raise CalibrationSolveError("each camera is solved once")
    for camera in cameras:
        require_motion(camera)
    given = initial or {}
    mounts: dict[str, _rigid.Matrix] = {
        camera.camera: given[camera.camera] if camera.camera in given else hand_eye(camera, target)
        for camera in cameras
    }
    current, holder_from_target = orient_cameras(target, cameras, mounts)
    passes = 0
    flips_total = dict.fromkeys(names, 0)
    status = SolverStatus(False, "not run", math.inf, 0)
    while True:
        passes += 1
        x0 = np.concatenate(
            (_rigid.to_vector(holder_from_target), *(_rigid.to_vector(mounts[n]) for n in names))
        )
        solution, status = _soft_l1(
            _joint_objective(current, names, target), x0, max_evaluations=max_evaluations
        )
        holder_from_target, mounts = _unpack(solution, names)
        current, flips = _reorient(current, holder_from_target, mounts, target)
        for name, count in flips.items():
            flips_total[name] += count
        if not any(flips.values()) or passes >= MAX_ORIENTATION_PASSES:
            break
    return MountSolve(current, holder_from_target, mounts, passes, flips_total, status)


def solve_target(
    target: Target,
    cameras: Sequence[CameraViews],
    *,
    mounts: Mounts,
    holder_from_target: _rigid.Matrix,
    max_evaluations: int = 300,
) -> tuple[tuple[CameraViews, ...], _rigid.Matrix]:
    """The target alone with every mount frozen: how a held-out collection checks a solve."""
    current = tuple(cameras)
    estimate = holder_from_target
    for _pass in range(MAX_ORIENTATION_PASSES):
        current, flips = _reorient(current, estimate, mounts, target)
        solution, _status = _soft_l1(
            _target_objective(current, mounts, target),
            _rigid.to_vector(estimate),
            max_evaluations=max_evaluations,
        )
        estimate = _rigid.from_vector(solution)
        if not any(flips.values()):
            break
    return current, estimate
