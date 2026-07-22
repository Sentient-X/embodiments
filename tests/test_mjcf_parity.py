"""New registry parts match the joint names and limits in their pinned MJCFs."""

import xml.etree.ElementTree as ET

from sx_embodiments.assets import PackagedAsset
from sx_embodiments.known.aloha import (
    ALOHA_LEFT_ARM,
    ALOHA_LEFT_GRIPPER,
    ALOHA_MJCF,
    ALOHA_RIGHT_ARM,
    ALOHA_RIGHT_GRIPPER,
)
from sx_embodiments.known.g1 import (
    UNITREE_G1_LEFT_ARM,
    UNITREE_G1_LEFT_LEG,
    UNITREE_G1_MJCF,
    UNITREE_G1_RIGHT_ARM,
    UNITREE_G1_RIGHT_LEG,
    UNITREE_G1_TORSO,
)
from sx_embodiments.known.rby1 import (
    RBY1_HEAD,
    RBY1_LEFT_ARM,
    RBY1_LEFT_GRIPPER,
    RBY1_MJCF,
    RBY1_RIGHT_ARM,
    RBY1_RIGHT_GRIPPER,
    RBY1_TORSO,
)
from sx_embodiments.known.universal_robots import (
    UR5E_ARM,
    UR5E_MJCF,
    UR10E_ARM,
    UR10E_MJCF,
)
from sx_embodiments.known.yor import YOR_LEFT_ARM, YOR_LIFT, YOR_MJCF, YOR_RIGHT_ARM
from sx_embodiments.parts import ArmSpec, GripperSpec, JointGroupSpec

JointPart = ArmSpec | JointGroupSpec | GripperSpec


def _joint_ranges(asset: PackagedAsset) -> dict[str, tuple[float, float]]:
    root = ET.parse(asset.path()).getroot()
    defaults: dict[str, tuple[float, float]] = {}

    def visit_default(default: ET.Element, inherited: tuple[float, float] | None = None) -> None:
        class_name = default.get("class")
        joint = default.find("joint")
        raw_range = joint.get("range") if joint is not None else None
        current = inherited
        if raw_range is not None:
            lower, upper = (float(value) for value in raw_range.split())
            current = (lower, upper)
        if class_name is not None and current is not None:
            defaults[class_name] = current
        for child in default.findall("default"):
            visit_default(child, current)

    for default in root.findall("default"):
        visit_default(default)

    ranges: dict[str, tuple[float, float]] = {}
    for joint in root.iter("joint"):
        name = joint.get("name")
        if name is None:
            continue
        raw_range = joint.get("range")
        if raw_range is not None:
            lower, upper = (float(value) for value in raw_range.split())
            ranges[name] = (lower, upper)
            continue
        class_name = joint.get("class")
        if class_name is not None and class_name in defaults:
            ranges[name] = defaults[class_name]
    return ranges


def _assert_part_matches(part: JointPart, ranges: dict[str, tuple[float, float]]) -> None:
    for index, name in enumerate(part.joint_names):
        assert ranges[name] == (part.joint_lower[index], part.joint_upper[index])


def test_aloha_parts_match_mjcf() -> None:
    ranges = _joint_ranges(ALOHA_MJCF)
    for part in (ALOHA_LEFT_ARM, ALOHA_LEFT_GRIPPER, ALOHA_RIGHT_ARM, ALOHA_RIGHT_GRIPPER):
        _assert_part_matches(part, ranges)


def test_rby1_parts_match_mjcf() -> None:
    ranges = _joint_ranges(RBY1_MJCF)
    for part in (
        RBY1_TORSO,
        RBY1_RIGHT_ARM,
        RBY1_LEFT_ARM,
        RBY1_HEAD,
        RBY1_RIGHT_GRIPPER,
        RBY1_LEFT_GRIPPER,
    ):
        _assert_part_matches(part, ranges)


def test_unitree_g1_parts_match_mjcf() -> None:
    ranges = _joint_ranges(UNITREE_G1_MJCF)
    for part in (
        UNITREE_G1_LEFT_LEG,
        UNITREE_G1_RIGHT_LEG,
        UNITREE_G1_TORSO,
        UNITREE_G1_LEFT_ARM,
        UNITREE_G1_RIGHT_ARM,
    ):
        _assert_part_matches(part, ranges)


def test_universal_robot_arms_match_mjcf() -> None:
    _assert_part_matches(UR5E_ARM, _joint_ranges(UR5E_MJCF))
    _assert_part_matches(UR10E_ARM, _joint_ranges(UR10E_MJCF))


def test_yor_lift_and_arms_match_mjcf() -> None:
    ranges = _joint_ranges(YOR_MJCF)
    for part in (YOR_LIFT, YOR_LEFT_ARM, YOR_RIGHT_ARM):
        _assert_part_matches(part, ranges)
