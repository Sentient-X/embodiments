"""Registry pins: derived constants can never drift from the deployed values."""

import pytest

from sx_embodiments import (
    EMBODIMENTS,
    PANDA_OMRON,
    PIPER,
    EmbodimentId,
    EmbodimentKind,
    MissingUrdfError,
    camera_names,
    kinematic_view,
    manifest_for,
)
from sx_embodiments.known.panda import PANDA_OMRON_SPEC
from sx_embodiments.known.piper import PIPER_SPEC


def test_piper_kinematic_view_is_the_deployed_constant() -> None:
    assert kinematic_view(PIPER_SPEC) == PIPER
    assert PIPER.embodiment_id == "piper"
    assert PIPER.dof == 6
    assert PIPER.joint_lower == (-2.6179, 0.0, -2.967, -1.745, -1.22, -2.09439)
    assert PIPER.joint_upper == (2.6179, 3.14, 0.0, 1.745, 1.22, 2.09439)
    assert PIPER.home_joints == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert PIPER.gripper_travel_m == (0.0, 0.07)
    assert PIPER.gripper_max_width_m == 0.07
    assert PIPER.policy_hz == 30.0
    assert PIPER.mobile_base is False


def test_panda_omron_kinematic_view_is_the_deployed_constant() -> None:
    assert kinematic_view(PANDA_OMRON_SPEC) == PANDA_OMRON
    assert PANDA_OMRON.embodiment_id == "panda_omron"
    assert PANDA_OMRON.dof == 7
    assert PANDA_OMRON.joint_lower == (
        -2.8973,
        -1.7628,
        -2.8973,
        -3.0718,
        -2.8973,
        -0.0175,
        -2.8973,
    )
    assert PANDA_OMRON.joint_upper == (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
    assert PANDA_OMRON.home_joints == (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)
    assert PANDA_OMRON.gripper_travel_m == (0.0, 0.08)
    assert PANDA_OMRON.policy_hz == 20.0
    assert PANDA_OMRON.mobile_base is True


def test_registry_ids_are_byte_stable() -> None:
    """These strings are live wire data (DB rows, .rrd meta, exports). Renames break data."""
    assert {str(eid) for eid in EMBODIMENTS} == {
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
        "libero_panda",
        "so101",
        "bimanual-so101",
        "das-umi-v4",
        "quest-ego",
        "insta360-umi",
        "yubi-mono",
        "yubi-depth",
        "yubi-widejaw",
        "piperx-station",
    }


def test_camera_name_sets() -> None:
    assert camera_names(EMBODIMENTS[EmbodimentId("das-umi-v4")]) == (
        "left_wrist",
        "right_wrist",
        "base",
    )
    assert camera_names(EMBODIMENTS[EmbodimentId("yubi-mono")]) == ("wrist_left", "wrist_right")
    assert camera_names(EMBODIMENTS[EmbodimentId("piper")]) == ()


def test_every_entry_validates_and_kinds_are_coherent() -> None:
    """Character laws: every entry knows what it is, and what it is implies what it has.

    A data-collection device without a camera cannot collect; a teleop station
    without a leader cannot be driven and without a camera cannot be supervised.
    Robots deliberately carry no cameras of their own — eyes arrive by composing
    the body into a rig or station (piperx-station's `front` over piper's none).
    """
    for eid, spec in EMBODIMENTS.items():
        assert spec.embodiment_id == eid
        assert spec.kind is not None, f"{eid} does not declare its character"
        if eid == EmbodimentId("insta360-umi"):
            with pytest.raises(MissingUrdfError):
                manifest_for(spec)
            continue
        manifest = manifest_for(spec)
        assert manifest.schema_version == 3
        assert (
            sum(
                asset.format.value == "urdf" and asset.role.value == "description"
                for asset in manifest.assets
            )
            == 1
        )
        if spec.kind is EmbodimentKind.TELEOP_STATION:
            assert any(a.role.value == "leader" for a in spec.attachments)
            assert camera_names(spec), f"teleop station {eid} declares no camera"
        if spec.kind is EmbodimentKind.CAPTURE_RIG:
            assert camera_names(spec), f"capture rig {eid} declares no camera"
