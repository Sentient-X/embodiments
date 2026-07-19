"""Part specs mirror the description assets they claim to mirror (stdlib xml only).

Scoped to the parts whose docstrings assert URDF parity: SO-101 and the DAS jaw. The
Piper spec deliberately diverges from its menagerie MJCF (deployed limits win; see
``known/piper.py``), so it is NOT pinned here.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

from sx_embodiments import (
    EPISODE_READY_EMBODIMENTS,
    AssetFormat,
    asset_root,
    manifest_for,
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
    for spec in EPISODE_READY_EMBODIMENTS.values():
        manifest = manifest_for(spec)
        urdf = next(asset for asset in manifest.assets if asset.format is AssetFormat.URDF)
        relpath = urdf.uri.removeprefix("package://sx-embodiments/")
        movable = {
            joint.get("name")
            for joint in ET.parse(asset_root() / relpath).getroot().iter("joint")
            if joint.get("type") not in (None, "fixed")
        }
        description_channels = {slot.joint_name for slot in manifest.layout.slots}
        assert description_channels <= movable
        assert all(asset.provenance is not None for asset in manifest.assets)
