"""The ordered native body-state space of an embodiment, and its exactness laws.

THE ORDERING LAW: the flat body-state vector of an embodiment is the concatenation, over
its attachments in declared tuple order, restricted to body-role attachments, of each
part's joint/channel names in the part's declared order. Nothing reorders; declaration
order IS wire order. Mount frames are informational and never affect channel order.
``tests/test_layout_laws.py`` pins exact index tuples for every registry entry.
"""

from dataclasses import dataclass
from enum import StrEnum

from .errors import LayoutError
from .identity import PartId


class ChannelKind(StrEnum):
    ARM_JOINT = "arm_joint"
    BODY_JOINT = "body_joint"
    GRIPPER = "gripper"
    BASE = "base"


@dataclass(frozen=True, slots=True)
class StateCoordinate:
    """One named coordinate in the embodiment's native body-state vector."""

    index: int
    instance: str  # attachment instance, such as "left_arm"
    part_id: PartId
    joint_name: str  # description joint or physical base channel
    kind: ChannelKind


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

    def uniform_arm_blocks(self) -> tuple[int, int, int]:
        """``(arms, block, gripper_index)`` — succeeds ONLY when the layout is N identical
        ``[arm joints…, one gripper]`` blocks and nothing else. The supervisors bridge."""
        grippers = self.gripper_count
        if grippers == 0:
            raise LayoutError("state", "no gripper channels; not an arm-block layout")
        if self.width % grippers != 0:
            raise LayoutError("state", "channels do not divide into equal arm blocks")
        block = self.width // grippers
        first = [coordinate.kind for coordinate in self.coordinates[:block]]
        gripper_positions = [i for i, kind in enumerate(first) if kind is ChannelKind.GRIPPER]
        if len(gripper_positions) != 1 or any(
            kind not in (ChannelKind.ARM_JOINT, ChannelKind.GRIPPER) for kind in first
        ):
            raise LayoutError("state", "block is not [arm joints…, one gripper]")
        for arm in range(1, grippers):
            kinds = [
                coordinate.kind for coordinate in self.coordinates[arm * block : (arm + 1) * block]
            ]
            if kinds != first:
                raise LayoutError("state", "arm blocks are not identical")
        return grippers, block, gripper_positions[0]
