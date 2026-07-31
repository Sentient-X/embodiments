"""Typed part records: the atoms embodiments are composed from.

Joint names belong to parts because they come from the description asset itself (URDF/MJCF)
and are pinned against it by ``tests/test_urdf_parity.py``. Names minted by an engine or
scene registry (``robot0_joint1``, robosuite handles) stay consumer-local — a name that
appears in the description asset belongs to the part; a name minted by an engine belongs to
the consumer.

There is deliberately no TeleopDevice part type: a leader arm IS an :class:`ArmSpec` — what
makes it a leader is its role in a composition, not its nature. Hardware whose kinematic
description has not been captured yet is a :class:`DeviceSpec`; its presence in a body role
marks the embodiment's channel layout as not yet declared, so layout-derived enforcement
skips rather than invents.
"""

from dataclasses import dataclass
from enum import StrEnum

from .assets import PackagedAsset
from .curves import Curve1D
from .errors import PartValidationError
from .identity import PartId
from .layout import CoordinateUnit


class SensorModel(StrEnum):
    """The closed sensor-product vocabulary (byte-equal to the factory hardware strings)."""

    UVC_MONO = "uvc_mono"
    REALSENSE_D405 = "realsense_d405"
    REALSENSE_D435 = "realsense_d435"
    QUEST3_RGB = "quest3_rgb"
    INSTA360_X5 = "insta360_x5"


class CameraModality(StrEnum):
    RGB = "rgb"
    DEPTH = "depth"
    RGBD = "rgbd"


class LensProjection(StrEnum):
    """The lens's nominal projection family — the calibration MODEL, never a calibration.

    Per-unit calibrated intrinsics (K/D matrices) are capture data and travel with the
    recording (MCAP ``camera_info``, Quest characteristics JSONs); the registry states only
    which projection family the optics follow. ``EQUIDISTANT`` is byte-equal to the
    Foxglove/OpenCV fisheye ``distortion_model`` wire string.
    """

    PINHOLE = "pinhole"
    EQUIDISTANT = "equidistant"


@dataclass(frozen=True, slots=True)
class PhysicalSpec:
    """Verified datasheet facts — the registry as spec-sheet-of-record.

    Every field is optional; a populated value MUST come from a manufacturer datasheet and
    is pinned by tests. An unverifiable fact stays ``None`` — never estimated.
    """

    payload_kg: float | None = None
    reach_m: float | None = None
    mass_kg: float | None = None

    def __post_init__(self) -> None:
        for name in ("payload_kg", "reach_m", "mass_kg"):
            value: float | None = getattr(self, name)
            if value is not None and value <= 0.0:
                raise PartValidationError(PartId("physical"), f"{name} must be positive")


def _validate_joint_box(
    part_id: PartId,
    joint_names: tuple[str, ...],
    joint_units: tuple[CoordinateUnit, ...],
    lower: tuple[float, ...],
    upper: tuple[float, ...],
) -> None:
    if len({*joint_names}) != len(joint_names):
        raise PartValidationError(part_id, "joint names must be unique")
    if not (len(joint_names) == len(joint_units) == len(lower) == len(upper)):
        raise PartValidationError(part_id, "joint names, units, and limits must have equal length")
    if any(lo >= hi for lo, hi in zip(lower, upper, strict=True)):
        raise PartValidationError(part_id, "joint lower limits must be < upper limits")


def _validate_joint_part(
    part_id: PartId,
    noun: str,
    joint_names: tuple[str, ...],
    joint_units: tuple[CoordinateUnit, ...],
    lower: tuple[float, ...],
    upper: tuple[float, ...],
    home_joints: tuple[float, ...],
) -> None:
    """The shared invariants of any actuated joint chain (arm, group, gripper homes)."""
    if not joint_names:
        raise PartValidationError(part_id, f"{noun} needs at least one joint")
    _validate_joint_box(part_id, joint_names, joint_units, lower, upper)
    if len(home_joints) != len(joint_names):
        raise PartValidationError(part_id, "home_joints must match the joint count")
    if any(home < lo or home > hi for home, lo, hi in zip(home_joints, lower, upper, strict=True)):
        raise PartValidationError(part_id, "home_joints must lie within the limits")


@dataclass(frozen=True, slots=True)
class ArmSpec:
    """A serial arm; ``joint_names`` order defines this part's channel order.

    ``ready_joints`` is a work-ready configuration — tool over the workspace, poised to
    act — distinct from ``home_joints`` (the driver's calibrated zero, which may point
    the arm straight up). Consumers that stage motion (scene composition, bench bring-up)
    read ``ready``; drivers home to ``home``.
    """

    part_id: PartId
    joint_names: tuple[str, ...]
    joint_units: tuple[CoordinateUnit, ...]
    joint_lower: tuple[float, ...]
    joint_upper: tuple[float, ...]
    home_joints: tuple[float, ...]
    ready_joints: tuple[float, ...] | None = None
    assets: tuple[PackagedAsset, ...] = ()
    physical: PhysicalSpec | None = None

    def __post_init__(self) -> None:
        _validate_joint_part(
            self.part_id,
            "an arm",
            self.joint_names,
            self.joint_units,
            self.joint_lower,
            self.joint_upper,
            self.home_joints,
        )
        if self.ready_joints is not None:
            if len(self.ready_joints) != len(self.joint_names):
                raise PartValidationError(self.part_id, "ready_joints must match the joint count")
            if any(
                ready < lo or ready > hi
                for ready, lo, hi in zip(
                    self.ready_joints, self.joint_lower, self.joint_upper, strict=True
                )
            ):
                raise PartValidationError(self.part_id, "ready_joints must lie within the limits")

    @property
    def dof(self) -> int:
        return len(self.joint_names)


@dataclass(frozen=True, slots=True)
class JointGroupSpec:
    """An articulated body group that is not an arm, gripper, or mobile base.

    Legs, torso/head chains, and linear lifts use this atom. ``joint_names`` contains
    executed action channels only; passive/mimic joints stay in the description asset.
    """

    part_id: PartId
    joint_names: tuple[str, ...]
    joint_units: tuple[CoordinateUnit, ...]
    joint_lower: tuple[float, ...]
    joint_upper: tuple[float, ...]
    home_joints: tuple[float, ...]
    assets: tuple[PackagedAsset, ...] = ()

    def __post_init__(self) -> None:
        _validate_joint_part(
            self.part_id,
            "a joint group",
            self.joint_names,
            self.joint_units,
            self.joint_lower,
            self.joint_upper,
            self.home_joints,
        )

    @property
    def dof(self) -> int:
        return len(self.joint_names)


@dataclass(frozen=True, slots=True)
class MimicJoint:
    """A passive joint driven by another joint's value (URDF ``<mimic>``)."""

    joint_name: str
    of: str  # the mimicked joint; may itself be a mimic (the DAS two-level chain)
    multiplier: float


@dataclass(frozen=True, slots=True)
class GripperSpec:
    """An end-effector; ``joint_names`` lists ACTUATED joints only.

    ``travel_m`` is the physical aperture range ``(closed, full_open)`` in meters when it is
    known; ``gap_curve`` is the measured aperture(q) forward-kinematic table for linkages
    where aperture is not linear in the joint angle (the DAS jaw). ``grasp_centre_m`` is
    the grasp centre — the midpoint between the closed finger pads — in the gripper's
    mount frame, so tool-frame consumers (scene composition, TCP feedback, Cartesian
    preflight) share one measured offset instead of each re-deriving it.
    """

    part_id: PartId
    joint_names: tuple[str, ...]
    joint_units: tuple[CoordinateUnit, ...]
    joint_lower: tuple[float, ...]
    joint_upper: tuple[float, ...]
    travel_m: tuple[float, float] | None = None
    grasp_centre_m: tuple[float, float, float] | None = None
    mimic_joints: tuple[MimicJoint, ...] = ()
    gap_curve: Curve1D | None = None
    assets: tuple[PackagedAsset, ...] = ()
    physical: PhysicalSpec | None = None

    def __post_init__(self) -> None:
        if not self.joint_names:
            raise PartValidationError(self.part_id, "a gripper needs at least one actuated joint")
        _validate_joint_box(
            self.part_id,
            self.joint_names,
            self.joint_units,
            self.joint_lower,
            self.joint_upper,
        )
        if self.travel_m is not None:
            lo, hi = self.travel_m
            if not 0.0 <= lo < hi:
                raise PartValidationError(self.part_id, "travel must satisfy 0 <= closed < open")
        known = {*self.joint_names}
        for mimic in self.mimic_joints:
            if mimic.joint_name in known:
                raise PartValidationError(
                    self.part_id, f"mimic joint {mimic.joint_name!r} duplicates a declared joint"
                )
            known.add(mimic.joint_name)
        resolvable = {*self.joint_names} | {m.joint_name for m in self.mimic_joints}
        for mimic in self.mimic_joints:
            if mimic.of not in resolvable:
                raise PartValidationError(
                    self.part_id, f"mimic joint {mimic.joint_name!r} follows unknown {mimic.of!r}"
                )

    @property
    def dof(self) -> int:
        return len(self.joint_names)


@dataclass(frozen=True, slots=True)
class CameraSpec:
    """A camera product; per-instance name and mount live on the Attachment.

    ``resolution`` is the product's native ``(width, height)`` in pixels, set only when one
    unambiguous per-product figure exists (a D435's depth and RGB streams differ — it stays
    ``None`` there until a consumer needs per-stream resolutions).
    """

    part_id: PartId
    model: SensorModel
    modality: CameraModality
    fps: float
    projection: LensProjection = LensProjection.PINHOLE
    resolution: tuple[int, int] | None = None

    def __post_init__(self) -> None:
        if self.fps <= 0.0:
            raise PartValidationError(self.part_id, "fps must be positive")
        if self.resolution is not None:
            width, height = self.resolution
            if width <= 0 or height <= 0:
                raise PartValidationError(self.part_id, "resolution must be positive")


@dataclass(frozen=True, slots=True)
class MobileBaseSpec:
    """A mobile base; ``channel_names`` is empty when the base is commanded outside the
    joint-space vector (the Panda-on-Omron case today)."""

    part_id: PartId
    channel_names: tuple[str, ...] = ()
    channel_units: tuple[CoordinateUnit, ...] = ()
    assets: tuple[PackagedAsset, ...] = ()

    def __post_init__(self) -> None:
        if len(self.channel_names) != len(self.channel_units):
            raise PartValidationError(
                self.part_id, "base channel names and units must have equal length"
            )


@dataclass(frozen=True, slots=True)
class ForceTorqueSpec:
    """A wrist force-torque sensor; rate unknown until measured."""

    part_id: PartId
    rate_hz: float | None = None


@dataclass(frozen=True, slots=True)
class DeviceSpec:
    """Hardware identified but not yet kinematically described (no captured URDF/limits).

    In a body role it contributes an UNKNOWN number of channels: the embodiment's flat
    layout is then undeclared and layout-derived enforcement skips (the registry cannot
    state a law it does not contain). Replace with a real part spec when the description
    is captured.
    """

    part_id: PartId
    description: str = ""


@dataclass(frozen=True, slots=True)
class CameraBinding:
    """A camera instance on an embodiment: canonical stream name + the sensor's facts."""

    name: str
    camera: CameraSpec
    parent_instance: str = ""
    frame: str = ""


@dataclass(frozen=True, slots=True)
class ControlRates:
    """Nominal control rates of the hardware (per-recording rates travel with episodes)."""

    policy_hz: float
    low_level_hz: float | None = None

    def __post_init__(self) -> None:
        if self.policy_hz <= 0.0:
            raise PartValidationError(PartId("rates"), "policy_hz must be positive")
        if self.low_level_hz is not None and self.low_level_hz <= 0.0:
            raise PartValidationError(PartId("rates"), "low_level_hz must be positive")


Part = (
    ArmSpec
    | JointGroupSpec
    | GripperSpec
    | CameraSpec
    | MobileBaseSpec
    | ForceTorqueSpec
    | DeviceSpec
)
