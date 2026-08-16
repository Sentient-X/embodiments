"""The public registry is the only route from a name to a complete embodiment."""

from sx_embodiments import Embodiment, development_embodiments, embodiments
from sx_embodiments.identity import EmbodimentKind
from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS, DevelopmentReason


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
        "quest-ego",
        "b601-dm",
        "bimanual-b601-dm",
    }


def test_camera_names_are_properties_of_the_embodiment() -> None:
    assert tuple(camera.name for camera in embodiments["quest-ego"].cameras) == (
        "head_left",
        "head_right",
    )
    assert embodiments["piper"].cameras == ()


def test_incomplete_camera_rigs_are_development_only() -> None:
    expected = {
        "das-umi-v4": DevelopmentReason.MISSING_CAMERA_CALIBRATION,
        "yubi": DevelopmentReason.MISSING_CAMERA_CALIBRATION,
        "piperx-station": DevelopmentReason.MISSING_CAMERA_INSTALLATION,
        "b601-dm-station": DevelopmentReason.MISSING_CAMERA_INSTALLATION,
    }
    for name, reason in expected.items():
        entry = DEVELOPMENT_EMBODIMENTS[name]
        assert entry.reason is reason
        assert name not in set(embodiments)

    yubi_mounts = DEVELOPMENT_EMBODIMENTS["yubi"].spec.operator_mounts
    assert tuple(mount.site.value for mount in yubi_mounts) == (
        "left_hand",
        "right_hand",
    )
    assert tuple(mount.attachment_frame for mount in yubi_mounts) == (
        "quest_left_controller",
        "quest_right_controller",
    )


def test_every_entry_is_complete_and_round_trips() -> None:
    for name in embodiments:
        embodiment = embodiments[name]
        assert isinstance(embodiment, Embodiment)
        assert str(embodiment.name) == name
        assert len(embodiment.id) == 64
        assert embodiment.schema_version == 12
        assert Embodiment.from_json(embodiment.to_json()) == embodiment
        assert embodiment.urdf_path.is_file()
        if embodiment.kind is EmbodimentKind.CAPTURE_RIG:
            assert embodiment.cameras


def test_yubi_embodiment_has_only_yubi_description_assets_and_parts() -> None:
    yubi = development_embodiments["yubi"]
    assert "das" not in yubi.to_json().lower()
    assert all(
        str(asset.asset.logical_path).startswith("yubi_description/") for asset in yubi.assets
    )
