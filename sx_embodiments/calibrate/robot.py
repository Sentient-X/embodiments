"""Calibrate one built robot's cameras: the solvers bound to a robot instance and its body.

A camera is named by the body (``embodiment.cameras``) and filled by a unit the instance
places in that camera's slot. Intrinsics are measured at the raster the body declares; a mount
is measured on the link that carries the camera rigidly, against a target held by another
link, with every view's carrier-from-holder transform from the body's own description at the
joint positions recorded with that view. The results are sealed ``sx.calibration/v1`` kinds
for the instance to select; nothing here moves the embodiment's id.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime

import numpy as np
from sx_contracts import Sha256Digest

from ..calibration import (
    Calibration,
    CalibrationMethod,
    Evidence,
    EvidenceKind,
    Measurement,
    Residual,
    ResidualUnit,
    RigidCalibration,
    RigidReference,
    UnitSerial,
)
from ..embodiment import Embodiment
from ..instance import InstanceError, RobotInstance, Slot, SlotKind, resolve
from ..kinematics import Kinematics
from ..parts import CameraBinding
from . import _rigid
from .intrinsics import IntrinsicsLimits, IntrinsicsSolve, View, solve_intrinsics
from .lens import Lens
from .mounts import CameraViews, MountSolve, MountView, predict, solve_mounts
from .targets import CalibrationSolveError, Detection, Target


def _camera(embodiment: Embodiment, name: str) -> CameraBinding:
    found = next((camera for camera in embodiment.cameras if camera.name == name), None)
    if found is None:
        raise InstanceError(f"{embodiment.name} declares no camera {name!r}")
    return found


def _unit(instance: RobotInstance, camera: str) -> UnitSerial:
    unit = instance.unit_in(Slot(SlotKind.CAMERA, camera))
    if unit is None:
        raise InstanceError(f"the instance places no unit in camera slot {camera!r}")
    return unit


def measure_intrinsics(
    embodiment: Embodiment,
    instance: RobotInstance,
    camera: str,
    target: Target,
    views: Sequence[View],
    *,
    measured_at: datetime,
    model: str | None = None,
    limits: IntrinsicsLimits | None = None,
) -> IntrinsicsSolve:
    """The camera unit's intrinsics at the body's declared raster.

    ``model`` defaults to the declared optics' distortion model, or ``plumb_bob`` where the
    body declares an undistorted nominal.
    """
    if instance.embodiment != embodiment.id:
        raise InstanceError("the instance is of a different embodiment")
    binding = _camera(embodiment, camera)
    optics = binding.camera.optics
    declared = optics.distortion_model
    return solve_intrinsics(
        target,
        views,
        width=optics.width,
        height=optics.height,
        model=model or ("plumb_bob" if declared == "none" else declared),
        unit=_unit(instance, camera),
        measured_at=measured_at,
        limits=limits,
    )


@dataclass(frozen=True, slots=True, eq=False)
class JointView:
    """One image of the target: its detection, the joint positions then, and the image's sha256."""

    positions: Mapping[str, float]
    detection: Detection
    image_sha256: str


@dataclass(frozen=True, slots=True, eq=False)
class MountMeasurement:
    calibration: RigidCalibration
    solve: MountSolve
    per_view_rms_px: tuple[float, ...]


def carrier_of(kinematics: Kinematics, frame: str) -> str:
    """The highest link joined to ``frame`` by fixed joints only: the rigid body it rides."""
    link = frame
    while (name := kinematics.parent_joint.get(link)) is not None:
        joint = kinematics.joints[name]
        if not kinematics.is_rigid(joint.parent, frame):
            break
        link = joint.parent
    return link


def measure_mount(
    embodiment: Embodiment,
    instance: RobotInstance,
    calibrations: Iterable[Calibration],
    camera: str,
    *,
    holder: str,
    target: Target,
    views: Sequence[JointView],
    measured_at: datetime,
    carrier: str | None = None,
    reference: RigidReference = RigidReference.DESCRIPTION,
) -> MountMeasurement:
    """Where the camera sits on its carrier link, from views of a target ``holder`` carries.

    The lens is the unit's selected intrinsics when the instance selects one, else the body's
    nominal optics. ``carrier`` defaults to the rigid body the camera frame rides; the result
    is ``carrier → camera frame``, a correction of the description's own transform when
    ``reference`` is ``DESCRIPTION``.
    """
    resolved = resolve(embodiment, instance, calibrations)
    binding = _camera(embodiment, camera)
    unit = _unit(instance, camera)
    measured = resolved.camera(camera).calibration
    lens = Lens.of(measured if measured is not None else binding.camera.optics)
    kinematics = Kinematics.of(embodiment.urdf_bytes)
    frame = binding.frame
    link = carrier or carrier_of(kinematics, frame)
    if not kinematics.is_rigid(link, frame):
        raise CalibrationSolveError(
            f"{frame!r} moves relative to {link!r}; a mount is measured on the link it rides"
        )
    if holder not in kinematics.links:
        raise CalibrationSolveError(f"{holder!r} is not a link of {embodiment.name}")
    if kinematics.is_rigid(link, holder):
        raise CalibrationSolveError(
            f"no joint moves {holder!r} relative to {link!r}; the views cannot see the mount"
        )
    nominal = _rigid.matrix(kinematics.pose(link, frame))
    camera_views = CameraViews(
        camera,
        lens,
        tuple(
            MountView(_rigid.matrix(kinematics.pose(link, holder, view.positions)), view.detection)
            for view in views
        ),
    )
    solve = solve_mounts(target, (camera_views,), initial={camera: nominal})
    solved = solve.carrier_from_camera[camera]
    per_view = solve.per_view_rms_px(target)[camera]
    deltas = np.concatenate(
        [
            predict(view, solve.holder_from_target, solved, target, lens) - view.detection.pixels
            for view in solve.cameras[0].views
        ]
    )
    transform = (
        _rigid.inverse(nominal) @ solved if reference is RigidReference.DESCRIPTION else solved
    )
    calibration = RigidCalibration(
        parent_frame=link,
        child_frame=frame,
        parent_from_child=_rigid.flat(transform),
        reference=reference,
        description_sha256=(
            Sha256Digest(embodiment.urdf.sha256)
            if reference is RigidReference.DESCRIPTION
            else None
        ),
        units=(unit,),
        measurement=Measurement(
            CalibrationMethod.HAND_EYE,
            measured_at,
            Residual(float(np.sqrt((deltas * deltas).sum(axis=1).mean())), ResidualUnit.PIXEL),
            tuple(
                Evidence(EvidenceKind.IMAGE, sha256)
                for sha256 in sorted({view.image_sha256 for view in views})
            ),
        ),
    )
    return MountMeasurement(calibration, solve, tuple(float(value) for value in per_view))
