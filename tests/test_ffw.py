"""FFW-BG2 mirrors the vendored `ai_worker` URDF (names, order, limits, mimic graph)."""

import xml.etree.ElementTree as ET

from sx_embodiments import development_embodiments
from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS, DevelopmentReason
from sx_embodiments.known.ffw import (
    FFW_BG2_HEAD,
    FFW_BG2_LEFT_ARM,
    FFW_BG2_LEFT_GRIPPER,
    FFW_BG2_RIGHT_ARM,
    FFW_BG2_RIGHT_GRIPPER,
    FFW_BG2_SPEC,
    FFW_BG2_TORSO,
    FFW_BG2_URDF,
)


def _movable_joints(root: ET.Element) -> dict[str, ET.Element]:
    return {
        joint.get("name") or "": joint
        for joint in root.iter("joint")
        if joint.get("type") not in (None, "fixed")
    }


def test_ffw_bg2_spec_matches_the_vendored_urdf() -> None:
    joints = _movable_joints(ET.parse(FFW_BG2_URDF.path()).getroot())
    actuated = (
        *FFW_BG2_TORSO.joint_names,
        *FFW_BG2_HEAD.joint_names,
        *FFW_BG2_LEFT_ARM.joint_names,
        *FFW_BG2_RIGHT_ARM.joint_names,
        *FFW_BG2_LEFT_GRIPPER.joint_names,
        *FFW_BG2_RIGHT_GRIPPER.joint_names,
    )
    mimics = {
        mimic.joint_name: mimic
        for gripper in (FFW_BG2_LEFT_GRIPPER, FFW_BG2_RIGHT_GRIPPER)
        for mimic in gripper.mimic_joints
    }
    assert set(joints) == {*actuated, *mimics}

    for part in (FFW_BG2_TORSO, FFW_BG2_HEAD, FFW_BG2_LEFT_ARM, FFW_BG2_RIGHT_ARM):
        for i, name in enumerate(part.joint_names):
            limit = joints[name].find("limit")
            assert limit is not None
            assert (float(limit.get("lower", "0")), float(limit.get("upper", "0"))) == (
                part.joint_lower[i],
                part.joint_upper[i],
            )
    for gripper in (FFW_BG2_LEFT_GRIPPER, FFW_BG2_RIGHT_GRIPPER):
        limit = joints[gripper.joint_names[0]].find("limit")
        assert limit is not None
        assert (float(limit.get("lower", "0")), float(limit.get("upper", "0"))) == (
            gripper.joint_lower[0],
            gripper.joint_upper[0],
        )

    for name, declared in mimics.items():
        mimic = joints[name].find("mimic")
        assert mimic is not None, f"{name} is declared a mimic but the URDF disagrees"
        assert mimic.get("joint") == declared.of
        assert float(mimic.get("multiplier", "1")) == declared.multiplier


def test_ffw_bg2_state_order_is_the_urdf_declaration_order() -> None:
    ffw = development_embodiments["ffw-bg2"]
    root = ET.parse(FFW_BG2_URDF.path()).getroot()
    urdf_actuated_order = tuple(
        joint.get("name")
        for joint in root.iter("joint")
        if joint.get("type") not in (None, "fixed") and joint.find("mimic") is None
    )
    assert tuple(c.joint_name for c in ffw.state.coordinates) == urdf_actuated_order
    assert ffw.state.width == 19
    assert ffw.has_mobile_base is False  # rev4 roots at world_fixed; SG2/F2/SH5 carry the swerve
    assert ffw.cameras == ()


def test_ffw_bg2_is_development_tier_until_cameras_are_calibrated() -> None:
    entry = DEVELOPMENT_EMBODIMENTS[FFW_BG2_SPEC.name]
    assert entry.reason is DevelopmentReason.MISSING_CAMERA_CALIBRATION


def test_ffw_bg2_id_is_byte_stable() -> None:
    assert (
        development_embodiments["ffw-bg2"].id
        == "395ea05c86939a1849ed20c5a4f4a3a3f490cd5d141f7b54f629f56fa60d2c7f"
    )
