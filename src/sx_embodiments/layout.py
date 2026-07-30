"""The ordered native body-state space of an embodiment, and its exactness laws.

THE ORDERING LAW: the flat body-state vector of an embodiment is the concatenation, over
its attachments in declared tuple order, restricted to body-role attachments, of each
part's joint/channel names in the part's declared order. Nothing reorders; declaration
order IS wire order. Mount frames are informational and never affect channel order.
``tests/test_layout_laws.py`` pins exact index tuples for every registry entry.
"""

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
    """Physical units admitted by native embodiment and action coordinates."""

    RADIAN = "rad"
    METER = "m"
    RADIANS_PER_SECOND = "rad/s"
    METERS_PER_SECOND = "m/s"
    NEWTON_METER = "N*m"
    UNITLESS = "unitless"


@dataclass(frozen=True, slots=True)
class StateCoordinate:
    """One named coordinate in the embodiment's native body-state vector."""

    index: int
    instance: ComponentId  # attachment instance, such as "left_arm"
    part_id: PartId
    joint_name: str  # description joint or physical base channel
    kind: ChannelKind
    unit: CoordinateUnit
    lower: float | None = None
    upper: float | None = None

    def __post_init__(self) -> None:
        """Bounds are either both known or both absent for unbounded native state."""
        if (self.lower is None) is not (self.upper is None):
            raise LayoutError(self.instance, f"{self.joint_name}: incomplete coordinate bounds")
        if self.lower is not None and self.upper is not None and self.lower >= self.upper:
            raise LayoutError(self.instance, f"{self.joint_name}: lower bound must be < upper")


@dataclass(frozen=True, slots=True)
class StateSpace:
    """Ordered native coordinates; dense tensors are projections of this table."""

    coordinates: tuple[StateCoordinate, ...]

    def __post_init__(self) -> None:
        if [coordinate.index for coordinate in self.coordinates] != list(
            range(len(self.coordinates))
        ):
            raise LayoutError("state", "slot indices must be 0..n-1 in order")

    @property
    def width(self) -> int:
        """Number of physical state channels; this says nothing about a controller."""
        return len(self.coordinates)

    @property
    def arm_joint_count(self) -> int:
        return sum(1 for coordinate in self.coordinates if coordinate.kind is ChannelKind.ARM_JOINT)

    @property
    def gripper_count(self) -> int:
        return sum(1 for coordinate in self.coordinates if coordinate.kind is ChannelKind.GRIPPER)

    @property
    def joint_count(self) -> int:
        """All non-gripper channels in the episode joint-state vector."""
        return self.width - self.gripper_count

    def indices(self, kind: ChannelKind) -> tuple[int, ...]:
        return tuple(coordinate.index for coordinate in self.coordinates if coordinate.kind is kind)

    @property
    def names(self) -> tuple[str, ...]:
        """``instance/joint_name`` per slot — the unambiguous wire order."""
        return tuple(
            f"{coordinate.instance}/{coordinate.joint_name}" for coordinate in self.coordinates
        )

    def validate_widths(self, *, joint_dim: int, gripper_dim: int) -> None:
        """Fail closed unless an episode's joint/gripper widths match this body."""
        if joint_dim != self.joint_count or gripper_dim != self.gripper_count:
            raise LayoutError(
                "state",
                f"episode widths (joints={joint_dim}, grippers={gripper_dim}) do not match "
                f"the declared layout (joints={self.joint_count}, "
                f"grippers={self.gripper_count})",
            )
