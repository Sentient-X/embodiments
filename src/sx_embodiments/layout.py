"""Canonical joint layouts and the derived native body-state order."""

import math
from dataclasses import dataclass
from enum import StrEnum

from sx_contracts import ComponentId

from .errors import LayoutError
from .identity import PartId


class ChannelKind(StrEnum):
    ARM_JOINT = "arm_joint"
    BODY_JOINT = "body_joint"
    GRIPPER = "gripper"
    BASE = "base"


class CoordinateUnit(StrEnum):
    RADIAN = "rad"
    METER = "m"
    RADIANS_PER_SECOND = "rad/s"
    METERS_PER_SECOND = "m/s"
    NEWTON_METER = "N*m"
    UNITLESS = "unitless"


@dataclass(frozen=True, slots=True)
class Unbounded:
    """A coordinate whose physical range is intentionally not declared."""


@dataclass(frozen=True, slots=True)
class Bounds:
    lower: float
    upper: float

    def __post_init__(self) -> None:
        if (
            isinstance(self.lower, bool)
            or isinstance(self.upper, bool)
            or not math.isfinite(self.lower)
            or not math.isfinite(self.upper)
            or self.lower >= self.upper
        ):
            raise LayoutError("joint", "bounds require finite lower < upper")
        object.__setattr__(self, "lower", float(self.lower))
        object.__setattr__(self, "upper", float(self.upper))


CoordinateBounds = Unbounded | Bounds


@dataclass(frozen=True, slots=True)
class JointAxis:
    """One joint coordinate; name, unit, and bounds cannot drift apart."""

    name: str
    unit: CoordinateUnit
    bounds: CoordinateBounds

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise LayoutError("joint", "axis name must not be empty")


@dataclass(frozen=True, slots=True)
class JointLayout:
    """A joint vector whose tuple order is the only coordinate order."""

    axes: tuple[JointAxis, ...]

    def __post_init__(self) -> None:
        names = tuple(axis.name for axis in self.axes)
        if len(set(names)) != len(names):
            raise LayoutError("joint", "axis names must be unique")

    @property
    def width(self) -> int:
        return len(self.axes)


def bounded_joint_layout(
    rows: tuple[tuple[str, CoordinateUnit, float, float], ...],
) -> JointLayout:
    """Compact source-authoring helper; the returned value has one canonical form."""

    return JointLayout(
        tuple(JointAxis(name, unit, Bounds(lower, upper)) for name, unit, lower, upper in rows)
    )


@dataclass(frozen=True, slots=True)
class StateCoordinate:
    """One coordinate in a derived state vector; tuple position is its index."""

    instance: ComponentId
    part_id: PartId
    axis: JointAxis
    kind: ChannelKind

    @property
    def joint_name(self) -> str:
        return self.axis.name

    @property
    def unit(self) -> CoordinateUnit:
        return self.axis.unit

    @property
    def lower(self) -> float | None:
        return self.axis.bounds.lower if isinstance(self.axis.bounds, Bounds) else None

    @property
    def upper(self) -> float | None:
        return self.axis.bounds.upper if isinstance(self.axis.bounds, Bounds) else None


@dataclass(frozen=True, slots=True)
class StateSpace:
    """Ordered native coordinates; dense tensors are projections of this tuple."""

    coordinates: tuple[StateCoordinate, ...]

    @property
    def width(self) -> int:
        return len(self.coordinates)

    @property
    def arm_joint_count(self) -> int:
        return sum(1 for coordinate in self.coordinates if coordinate.kind is ChannelKind.ARM_JOINT)

    @property
    def gripper_count(self) -> int:
        return sum(1 for coordinate in self.coordinates if coordinate.kind is ChannelKind.GRIPPER)

    @property
    def joint_count(self) -> int:
        return self.width - self.gripper_count

    def indices(self, kind: ChannelKind) -> tuple[int, ...]:
        return tuple(
            index for index, coordinate in enumerate(self.coordinates) if coordinate.kind is kind
        )

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(
            f"{coordinate.instance}/{coordinate.axis.name}" for coordinate in self.coordinates
        )

    def validate_widths(self, *, joint_dim: int, gripper_dim: int) -> None:
        if joint_dim != self.joint_count or gripper_dim != self.gripper_count:
            raise LayoutError(
                "state",
                f"episode widths (joints={joint_dim}, grippers={gripper_dim}) do not match "
                f"the declared layout (joints={self.joint_count}, "
                f"grippers={self.gripper_count})",
            )
