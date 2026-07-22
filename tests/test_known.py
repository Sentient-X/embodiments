"""The public registry is the only route from a name to a complete embodiment."""

from sx_embodiments import ComponentRole, Embodiment, EmbodimentKind, embodiments


def test_piper_kinematics_are_derived_from_the_registry() -> None:
    piper = embodiments["piper"]
    assert piper.single_arm.dof == 6
    assert piper.single_arm.joint_lower == (-2.6179, 0.0, -2.967, -1.745, -1.22, -2.09439)
    assert piper.single_arm.joint_upper == (2.6179, 3.14, 0.0, 1.745, 1.22, 2.09439)
    assert piper.single_arm.home_joints == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert piper.single_gripper.travel_m == (0.0, 0.07)
    assert piper.policy_hz == 30.0
    assert piper.has_mobile_base is False


def test_panda_omron_kinematics_are_derived_from_the_registry() -> None:
    panda = embodiments["panda_omron"]
    assert panda.single_arm.dof == 7
    assert panda.single_arm.joint_lower == (
        -2.8973,
        -1.7628,
        -2.8973,
        -3.0718,
        -2.8973,
        -0.0175,
        -2.8973,
    )
    assert panda.single_arm.joint_upper == (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
    assert panda.single_arm.home_joints == (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)
    assert panda.single_gripper.travel_m == (0.0, 0.08)
    assert panda.policy_hz == 20.0
    assert panda.has_mobile_base is True


def test_registry_ids_are_byte_stable() -> None:
    assert set(embodiments) == {
        "piper",
        "nero",
        "aloha",
        "rby1",
        "unitree-g1",
        "ur10e",
        "ur5e",
        "yor",
        "sentient-humanoid",
        "panda_omron",
        "franka",
        "so101",
        "bimanual-so101",
        "das-umi-v4",
        "quest-ego",
        "yubi-mono",
        "yubi-depth",
        "yubi-widejaw",
        "piperx-station",
    }


def test_camera_names_are_properties_of_the_embodiment() -> None:
    assert tuple(camera.name for camera in embodiments["das-umi-v4"].cameras) == (
        "left_wrist",
        "right_wrist",
        "base",
    )
    assert tuple(camera.name for camera in embodiments["yubi-mono"].cameras) == (
        "wrist_left",
        "wrist_right",
    )
    assert embodiments["piper"].cameras == ()


def test_every_entry_is_complete_and_round_trips() -> None:
    for name in embodiments:
        embodiment = embodiments[name]
        assert isinstance(embodiment, Embodiment)
        assert str(embodiment.name) == name
        assert len(embodiment.id) == 64
        assert embodiment.schema_version == 7
        assert Embodiment.from_json(embodiment.to_json()) == embodiment
        assert embodiment.urdf.local_path().is_file()
        if embodiment.kind is EmbodimentKind.TELEOP_STATION:
            assert any(
                component.role is ComponentRole.LEADER for component in embodiment.components
            )
            assert embodiment.cameras
        if embodiment.kind is EmbodimentKind.CAPTURE_RIG:
            assert embodiment.cameras
