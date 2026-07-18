"""The flat-vector channel layout of an embodiment, and its exactness laws.

THE ORDERING LAW: the flat joint-space vector of an embodiment is the concatenation, over
its attachments in declared tuple order, restricted to body-role attachments, of each
part's joint/channel names in the part's declared order. Nothing reorders; declaration
order IS wire order. Mount frames are informational and never affect channel order.
``tests/test_layout_laws.py`` pins exact index tuples for every registry entry.
"""

from dataclasses import dataclass
from enum import StrEnum

from .errors import LayoutError
from .identity import EmbodimentId, PartId


class ChannelKind(StrEnum):
    ARM_JOINT = "arm_joint"
    GRIPPER = "gripper"
    BASE = "base"


@dataclass(frozen=True, slots=True)
class ChannelSlot:
    """One position in the flat vector."""

    index: int
    instance: str  # attachment instance ("left_arm")
    part_id: PartId
    joint_name: str  # name in the description asset ("gripper", "joint_1")
    kind: ChannelKind


@dataclass(frozen=True, slots=True)
class FlatLayout:
    """The derived flat-vector layout; every view of it is arithmetic over ``slots``."""

    embodiment_id: EmbodimentId
    slots: tuple[ChannelSlot, ...]

    def __post_init__(self) -> None:
        if [slot.index for slot in self.slots] != list(range(len(self.slots))):
            raise LayoutError(self.embodiment_id, "slot indices must be 0..n-1 in order")

    @property
    def action_dim(self) -> int:
        return len(self.slots)

    @property
    def arm_joint_count(self) -> int:
        return sum(1 for slot in self.slots if slot.kind is ChannelKind.ARM_JOINT)

    @property
    def gripper_count(self) -> int:
        return sum(1 for slot in self.slots if slot.kind is ChannelKind.GRIPPER)

    def indices(self, kind: ChannelKind) -> tuple[int, ...]:
        return tuple(slot.index for slot in self.slots if slot.kind is kind)

    def channel_names(self) -> tuple[str, ...]:
        """``instance/joint_name`` per slot — the unambiguous wire order."""
        return tuple(f"{slot.instance}/{slot.joint_name}" for slot in self.slots)

    def validate_widths(self, *, joint_dim: int, gripper_dim: int) -> None:
        """Fail closed unless the split (joint, gripper) widths match this layout."""
        if joint_dim != self.arm_joint_count or gripper_dim != self.gripper_count:
            raise LayoutError(
                self.embodiment_id,
                f"action widths (joints={joint_dim}, grippers={gripper_dim}) do not match "
                f"the declared layout (joints={self.arm_joint_count}, "
                f"grippers={self.gripper_count})",
            )

    def uniform_arm_blocks(self) -> tuple[int, int, int]:
        """``(arms, block, gripper_index)`` — succeeds ONLY when the layout is N identical
        ``[arm joints…, one gripper]`` blocks and nothing else. The supervisors bridge."""
        grippers = self.gripper_count
        if grippers == 0:
            raise LayoutError(self.embodiment_id, "no gripper channels; not an arm-block layout")
        if self.action_dim % grippers != 0:
            raise LayoutError(self.embodiment_id, "channels do not divide into equal arm blocks")
        block = self.action_dim // grippers
        first = [slot.kind for slot in self.slots[:block]]
        gripper_positions = [i for i, kind in enumerate(first) if kind is ChannelKind.GRIPPER]
        if len(gripper_positions) != 1 or any(k is ChannelKind.BASE for k in first):
            raise LayoutError(self.embodiment_id, "block is not [arm joints…, one gripper]")
        for arm in range(1, grippers):
            kinds = [slot.kind for slot in self.slots[arm * block : (arm + 1) * block]]
            if kinds != first:
                raise LayoutError(self.embodiment_id, "arm blocks are not identical")
        return grippers, block, gripper_positions[0]
