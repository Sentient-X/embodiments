"""Part specs mirror the description assets they claim to mirror (stdlib xml only).

Scoped to the parts whose docstrings assert URDF parity: SO-101 and the DAS jaw. The
Piper spec deliberately diverges from its menagerie MJCF (deployed limits win; see
``known/piper.py``), so it is NOT pinned here.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

from sx_embodiments import (
    AssetFormat,
    embodiments,
)
from sx_embodiments.known.das import DAS_JAW_V4
from sx_embodiments.known.humanoid import (
    SENTIENT_HUMANOID_LEFT_ARM,
    SENTIENT_HUMANOID_LEFT_LEG,
    SENTIENT_HUMANOID_RIGHT_ARM,
    SENTIENT_HUMANOID_RIGHT_LEG,
    SENTIENT_HUMANOID_TORSO,
    SENTIENT_HUMANOID_URDF,
)
from sx_embodiments.known.so101 import SO101_ARM, SO101_JAW, SO101_URDF


def _movable_joints(path: Path) -> dict[str, tuple[float, float, ET.Element]]:
    joints: dict[str, tuple[float, float, ET.Element]] = {}
    for joint in ET.parse(path).getroot().iter("joint"):
        if joint.get("type") in (None, "fixed"):
            continue
        limit = joint.find("limit")
        assert limit is not None
        name = joint.get("name")
        assert name is not None
        joints[name] = (float(limit.get("lower", "0")), float(limit.get("upper", "0")), joint)
    return joints


def test_so101_spec_matches_urdf() -> None:
    joints = _movable_joints(SO101_URDF.path())
    assert set(joints) == {*SO101_ARM.joint_names, *SO101_JAW.joint_names}
    for i, name in enumerate(SO101_ARM.joint_names):
        lower, upper, _ = joints[name]
        assert (lower, upper) == (SO101_ARM.joint_lower[i], SO101_ARM.joint_upper[i])
    lower, upper, _ = joints[SO101_JAW.joint_names[0]]
    assert (lower, upper) == (SO101_JAW.joint_lower[0], SO101_JAW.joint_upper[0])


def test_das_jaw_matches_urdf_mimic_chain() -> None:
    from sx_embodiments.known.das import DAS_GRIPPER_URDF

    joints = _movable_joints(DAS_GRIPPER_URDF.path())
    assert set(joints) == {
        *DAS_JAW_V4.joint_names,
        *(m.joint_name for m in DAS_JAW_V4.mimic_joints),
    }
    lower, upper, _ = joints["joint_1"]
    assert (lower, upper) == (0.0, 0.925)
    for mimic in DAS_JAW_V4.mimic_joints:
        _, _, element = joints[mimic.joint_name]
        declared = element.find("mimic")
        assert declared is not None, mimic.joint_name
        assert declared.get("joint") == mimic.of
        assert float(declared.get("multiplier", "1")) == mimic.multiplier


def test_das_gap_curve_shape() -> None:
    curve = DAS_JAW_V4.gap_curve
    assert curve is not None
    assert curve.at(0.0) == 0.105  # geometric full-open
    assert curve.at(0.925) == 0.00011  # closed
    # inverse round-trips through the measured region
    for q in (0.1, 0.42, 0.8):
        assert abs(curve.inverse_at(curve.at(q)) - q) < 1e-9


def test_sentient_humanoid_executed_groups_match_hardware_urdf() -> None:
    joints = _movable_joints(SENTIENT_HUMANOID_URDF.path())
    parts = (
        SENTIENT_HUMANOID_TORSO,
        SENTIENT_HUMANOID_RIGHT_ARM,
        SENTIENT_HUMANOID_LEFT_ARM,
        SENTIENT_HUMANOID_RIGHT_LEG,
        SENTIENT_HUMANOID_LEFT_LEG,
    )
    for part in parts:
        for index, name in enumerate(part.joint_names):
            lower, upper, _ = joints[name]
            assert (lower, upper) == (part.joint_lower[index], part.joint_upper[index])

    executed = {name for part in parts for name in part.joint_names}
    assert "waist_rod_joint" not in executed  # feedback/passive, never an action row
    assert {"neck_yaw_joint", "head_joint"}.isdisjoint(executed)  # no motor assignment


def test_every_episode_ready_layout_is_declared_by_its_authoritative_urdf() -> None:
    for embodiment in embodiments.values():
        urdf = next(asset for asset in embodiment.assets if asset.format is AssetFormat.URDF)
        movable = {
            joint.get("name")
            for joint in ET.parse(urdf.local_path()).getroot().iter("joint")
            if joint.get("type") not in (None, "fixed")
        }
        description_channels = {
            coordinate.joint_name for coordinate in embodiment.state.coordinates
        }
        assert description_channels <= movable
        assert all(asset.provenance is not None for asset in embodiment.assets)


def test_yubi_hands_urdf_matches_upstream_hand_model() -> None:
    """The vendored yubi-sw description: per hand one encoder-driven finger joint
    with a -1 mimic mirror, the hand-camera links the registry mounts cameras on,
    the Quest controller mount frames, and a closed mesh set."""
    from sx_embodiments.assets import asset_root
    from sx_embodiments.known.yubi import YUBI_HANDS_URDF

    root = ET.parse(YUBI_HANDS_URDF.path()).getroot()
    links = {link.get("name") for link in root.iter("link")}
    assert {
        "quest_origin",
        "quest_left_controller",
        "quest_right_controller",
        "left_hand_cam_link",
        "right_hand_cam_link",
    } <= links

    revolute = {
        joint.get("name"): joint for joint in root.iter("joint") if joint.get("type") == "revolute"
    }
    hands = ("left", "right")
    assert set(revolute) == {f"{hand}_{finger}_finger_joint" for hand in hands for finger in hands}
    for hand in ("left", "right"):
        driven = revolute[f"{hand}_right_finger_joint"]
        limit = driven.find("limit")
        assert limit is not None and (limit.get("lower"), limit.get("upper")) == ("0.0", "0.94")
        assert driven.find("mimic") is None
        mirror = revolute[f"{hand}_left_finger_joint"]
        declared = mirror.find("mimic")
        assert declared is not None
        assert declared.get("joint") == f"{hand}_right_finger_joint"
        assert float(declared.get("multiplier", "1")) == -1.0

    package_root = asset_root()
    meshes = {
        mesh.get("filename") for mesh in root.iter("mesh") if mesh.get("filename") is not None
    }
    assert meshes, "yubi hands URDF references no meshes"
    for filename in meshes:
        assert filename is not None and filename.startswith("package://yubi_description/")
        relative = filename.removeprefix("package://")
        assert (package_root / relative).is_file(), f"unresolved mesh {filename}"


def test_yubi_camera_mounts_exist_in_authoritative_urdf() -> None:
    for name in ("yubi-mono", "yubi-depth", "yubi-widejaw"):
        embodiment = embodiments[name]
        links = {link.get("name") for link in ET.fromstring(embodiment.urdf_bytes).iter("link")}
        for camera in embodiment.cameras:
            assert camera.frame in links, f"{name}: camera {camera.name} frame {camera.frame}"
