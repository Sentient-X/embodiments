"""The ordering law, pinned with literal expected layouts (no property-only testing)."""

import pytest

from sx_embodiments import ChannelKind, LayoutError, UnknownEmbodimentError, embodiments


def test_bimanual_so101_state_is_the_supervisors_wire_convention() -> None:
    state = embodiments["bimanual-so101"].state
    assert state.names == (
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
    assert state.width == 12
    assert state.arm_joint_count == 10
    assert state.gripper_count == 2
    assert state.indices(ChannelKind.ARM_JOINT) == (0, 1, 2, 3, 4, 6, 7, 8, 9, 10)
    assert state.indices(ChannelKind.GRIPPER) == (5, 11)


def test_piper_state() -> None:
    state = embodiments["piper"].state
    assert state.names == (
        "arm/joint1",
        "arm/joint2",
        "arm/joint3",
        "arm/joint4",
        "arm/joint5",
        "arm/joint6",
        "gripper/joint7",
    )
    assert state.indices(ChannelKind.ARM_JOINT) == (0, 1, 2, 3, 4, 5)
    assert state.indices(ChannelKind.GRIPPER) == (6,)


def test_das_capture_rig_state_is_two_jaw_channels() -> None:
    from sx_embodiments.embodiment import embodiment_from_definition
    from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS

    state = embodiment_from_definition(DEVELOPMENT_EMBODIMENTS["das-umi-v4"].spec).state
    assert state.names == ("left_jaw/joint_1", "right_jaw/joint_1")
    assert state.arm_joint_count == 0
    assert state.gripper_count == 2
    assert state.indices(ChannelKind.GRIPPER) == (0, 1)


def test_width_validation_enforces_declared_layouts() -> None:
    state = embodiments["bimanual-so101"].state
    state.validate_widths(joint_dim=10, gripper_dim=2)
    with pytest.raises(LayoutError):
        state.validate_widths(joint_dim=12, gripper_dim=0)


def test_unknown_id_fails_closed() -> None:
    with pytest.raises(UnknownEmbodimentError):
        embodiments["not-a-robot"]


def test_every_registered_robot_has_native_state() -> None:
    for embodiment in embodiments.values():
        assert embodiment.state.width > 0 or embodiment.kind.value != "robot"


def test_new_robot_layout_widths_follow_upstream_action_order() -> None:
    expected = {
        "aloha": 14,
        "rby1": 28,
        "unitree-g1": 29,
        "ur10e": 6,
        "ur5e": 6,
        "yor": 22,
        "sentient-humanoid": 25,
    }
    assert {name: embodiments[name].state.width for name in expected} == expected


def test_whole_body_layout_exposes_exact_coordinate_kinds() -> None:
    state = embodiments["rby1"].state
    assert state.indices(ChannelKind.BASE) == tuple(range(4))
    assert state.indices(ChannelKind.BODY_JOINT) == (*range(4, 10), 24, 25)
    assert state.indices(ChannelKind.ARM_JOINT) == tuple(range(10, 24))
    assert state.indices(ChannelKind.GRIPPER) == (26, 27)
