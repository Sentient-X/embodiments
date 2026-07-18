"""The ordering law, pinned with literal expected layouts (no property-only testing)."""

import pytest

from sx_embodiments import (
    EMBODIMENTS,
    ChannelKind,
    EmbodimentId,
    LayoutError,
    UnknownEmbodimentError,
    flat_layout,
    layout_for,
    validate_action_widths,
)
from sx_embodiments.known.das import DAS_UMI_V4_SPEC
from sx_embodiments.known.piper import PIPER_SPEC
from sx_embodiments.known.so101 import BIMANUAL_SO101_SPEC
from sx_embodiments.known.stations import B601_DM_SPEC


def test_bimanual_so101_layout_is_the_supervisors_wire_convention() -> None:
    layout = flat_layout(BIMANUAL_SO101_SPEC)
    assert layout.channel_names() == (
        "left_arm/shoulder_pan",
        "left_arm/shoulder_lift",
        "left_arm/elbow_flex",
        "left_arm/wrist_flex",
        "left_arm/wrist_roll",
        "left_jaw/gripper",
        "right_arm/shoulder_pan",
        "right_arm/shoulder_lift",
        "right_arm/elbow_flex",
        "right_arm/wrist_flex",
        "right_arm/wrist_roll",
        "right_jaw/gripper",
    )
    assert layout.action_dim == 12
    assert layout.arm_joint_count == 10
    assert layout.gripper_count == 2
    # The supervisors ChannelLayout(arms=2, block=6, gripper_index=5), derived not declared.
    assert layout.uniform_arm_blocks() == (2, 6, 5)
    assert layout.indices(ChannelKind.GRIPPER) == (5, 11)


def test_piper_layout() -> None:
    layout = flat_layout(PIPER_SPEC)
    assert layout.channel_names() == (
        "arm/joint1",
        "arm/joint2",
        "arm/joint3",
        "arm/joint4",
        "arm/joint5",
        "arm/joint6",
        "gripper/joint7",
    )
    assert layout.uniform_arm_blocks() == (1, 7, 6)


def test_das_capture_rig_layout_is_two_jaw_channels() -> None:
    layout = flat_layout(DAS_UMI_V4_SPEC)
    assert layout.channel_names() == ("left_jaw/joint_1", "right_jaw/joint_1")
    assert layout.arm_joint_count == 0
    assert layout.gripper_count == 2
    # Degenerate-but-coherent: two one-channel jaw "blocks".
    assert layout.uniform_arm_blocks() == (2, 1, 0)


def test_undeclared_layout_fails_closed_but_widths_skip() -> None:
    with pytest.raises(LayoutError):
        flat_layout(B601_DM_SPEC)  # DeviceSpec follower: channels unknown
    # ...and the width validator therefore SKIPS rather than inventing a law.
    validate_action_widths(B601_DM_SPEC.embodiment_id, joint_dim=99, gripper_dim=99)


def test_width_validation_enforces_declared_layouts() -> None:
    validate_action_widths(EmbodimentId("bimanual-so101"), joint_dim=10, gripper_dim=2)
    with pytest.raises(LayoutError):
        validate_action_widths(EmbodimentId("bimanual-so101"), joint_dim=12, gripper_dim=0)
    # Unknown ids pass through: the registry cannot enforce a law it does not state.
    validate_action_widths(EmbodimentId("not-a-robot"), joint_dim=1, gripper_dim=1)


def test_layout_for_unknown_id_fails_closed() -> None:
    with pytest.raises(UnknownEmbodimentError):
        layout_for(EmbodimentId("not-a-robot"))


def test_every_declared_registry_layout_derives() -> None:
    for spec in EMBODIMENTS.values():
        if spec.layout_declared():
            layout = flat_layout(spec)
            assert layout.action_dim > 0 or spec.kind.value != "robot"
