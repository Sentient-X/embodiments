"""The one URDF chain reader: poses compose, mimics follow, malformed trees are refused."""

from __future__ import annotations

import math

import pytest

from sx_embodiments.kinematics import IDENTITY, Kinematics, KinematicsError, compose, invert

URDF = b"""<robot name="t">
  <link name="base"/><link name="a"/><link name="b"/><link name="c"/><link name="d"/>
  <joint name="j1" type="revolute"><parent link="base"/><child link="a"/>
    <origin xyz="0 0 1" rpy="0 0 0"/><axis xyz="0 0 1"/></joint>
  <joint name="j2" type="prismatic"><parent link="a"/><child link="b"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/></joint>
  <joint name="j3" type="revolute"><parent link="base"/><child link="c"/>
    <origin xyz="0 1 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <mimic joint="j1" multiplier="-1" offset="0"/></joint>
  <joint name="f" type="fixed"><parent link="c"/><child link="d"/>
    <origin xyz="0 0 0.5" rpy="0 0 1.5707963267948966"/></joint>
</robot>"""


def _point(transform: tuple[float, ...]) -> tuple[float, float, float]:
    return (transform[3], transform[7], transform[11])


def test_a_chain_composes_down_and_across_branches() -> None:
    kinematics = Kinematics.of(URDF)
    positions = {"j1": math.pi / 2, "j2": 0.25}
    # base -> a turns 90 degrees about z, so a's x axis is base's y.
    assert _point(kinematics.pose("base", "b", positions)) == pytest.approx((0.0, 1.25, 1.0))
    across = kinematics.pose("b", "d", positions)
    assert compose(kinematics.pose("base", "b", positions), across) == pytest.approx(
        kinematics.pose("base", "d", positions)
    )
    assert compose(across, kinematics.pose("d", "b", positions)) == pytest.approx(IDENTITY)
    assert invert(across) == pytest.approx(kinematics.pose("d", "b", positions))


def test_a_mimic_follows_its_leader_and_cannot_be_given() -> None:
    kinematics = Kinematics.of(URDF)
    assert kinematics.independent == ("j1", "j2")
    turned = kinematics.pose("base", "c", {"j1": 0.3})
    assert math.atan2(turned[4], turned[0]) == pytest.approx(-0.3)
    with pytest.raises(KinematicsError, match="follows"):
        kinematics.pose("base", "c", {"j1": 0.3, "j3": 0.1})


def test_a_rigid_path_needs_no_positions_and_a_moving_one_does() -> None:
    kinematics = Kinematics.of(URDF)
    assert kinematics.is_rigid("c", "d") and not kinematics.is_rigid("base", "b")
    assert _point(kinematics.pose("c", "d")) == pytest.approx((0.0, 0.0, 0.5))
    with pytest.raises(KinematicsError, match="no position for movable joint 'j1'"):
        kinematics.pose("base", "a")


@pytest.mark.parametrize(
    ("urdf", "message"),
    [
        (
            b"""<robot name="t"><link name="a"/><link name="b"/>
            <joint name="x" type="fixed"><parent link="a"/><child link="b"/></joint>
            <joint name="y" type="fixed"><parent link="b"/><child link="a"/></joint></robot>""",
            "cycle",
        ),
        (
            b"""<robot name="t"><link name="a"/><link name="b"/><link name="c"/>
            <joint name="x" type="revolute"><parent link="a"/><child link="b"/>
              <mimic joint="y"/></joint>
            <joint name="y" type="revolute"><parent link="a"/><child link="c"/>
              <mimic joint="x"/></joint></robot>""",
            "cycle",
        ),
        (
            b"""<robot name="t"><link name="a"/><link name="b"/>
            <joint name="x" type="hinge"><parent link="a"/><child link="b"/></joint></robot>""",
            "does not define",
        ),
    ],
    ids=["link-cycle", "mimic-cycle", "unknown-type"],
)
def test_a_malformed_tree_is_refused(urdf: bytes, message: str) -> None:
    with pytest.raises(KinematicsError, match=message):
        Kinematics.of(urdf)
