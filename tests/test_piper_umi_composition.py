"""Piper composition preserves the supplied per-hand geometry and all meshes."""

import copy
import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "assets/piper_umi_description"
spec = importlib.util.spec_from_file_location(
    "compose_piper_umi_urdf", ROOT / "tools/compose_piper_umi_urdf.py"
)
assert spec is not None and spec.loader is not None
composer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(composer)


def test_generated_description_is_current() -> None:
    assert (DESCRIPTION / "urdf/piper_umi_hands.urdf").read_bytes() == composer.render()


def test_both_hands_and_mesh_closure() -> None:
    robot = ET.fromstring(composer.render())
    assert len(robot.findall("link")) == 117
    assert len(robot.findall("joint")) == 116
    composer.validate_tree(robot)
    for mesh in robot.findall(".//mesh"):
        uri = mesh.attrib["filename"]
        assert uri.startswith("package://piper_umi_description/")
        assert (DESCRIPTION / uri.removeprefix("package://piper_umi_description/")).is_file()


@pytest.mark.parametrize("side", ["left", "right"])
def test_source_joint_geometry_is_unchanged(side: str) -> None:
    source = ET.parse(DESCRIPTION / "source" / f"piper_umi_{side}.urdf").getroot()
    robot = ET.fromstring(composer.render())
    for original in source.findall("joint"):
        composed = robot.find(f"./joint[@name='{side}_{original.attrib['name']}']")
        assert composed is not None
        assert composed.attrib["type"] == original.attrib["type"]
        for tag in ("origin", "axis", "limit", "dynamics"):
            before, after = original.find(tag), composed.find(tag)
            if tag == "limit" and original.attrib["name"] in ("slider_1", "slider_2"):
                assert before is not None and after is not None
                driver = original.attrib["name"] == "slider_1"
                assert after.attrib == {
                    **before.attrib,
                    "lower": "0" if driver else "-0.05",
                    "upper": "0.05" if driver else "0",
                }
                continue
            assert (None if before is None else before.attrib) == (
                None if after is None else after.attrib
            )
        for tag in ("parent", "child"):
            before, after = original.find(tag), composed.find(tag)
            assert before is not None and after is not None
            assert after.attrib["link"] == composer.link_name(side, before.attrib["link"])
        before, after = original.find("mimic"), composed.find("mimic")
        if before is not None:
            assert after is not None
            assert after.attrib == {**before.attrib, "joint": f"{side}_{before.attrib['joint']}"}


def _element_signature(element: ET.Element) -> tuple:
    return (
        element.tag,
        sorted(element.attrib.items()),
        (element.text or "").strip(),
        tuple(_element_signature(child) for child in element),
    )


@pytest.mark.parametrize("side", ["left", "right"])
def test_link_geometry_inertias_and_materials_are_preserved(side: str) -> None:
    source = ET.parse(DESCRIPTION / "source" / f"piper_umi_{side}.urdf").getroot()
    robot = ET.fromstring(composer.render())
    for original in source.findall("link"):
        expected = copy.deepcopy(original)
        name = composer.link_name(side, original.attrib["name"])
        expected.set("name", name)
        for mesh in expected.iter("mesh"):
            relative = mesh.attrib["filename"].removeprefix("package://umi/meshes/")
            mesh.set("filename", f"package://piper_umi_description/meshes/{side}/{relative}")
        for material in expected.iter("material"):
            if material.get("name"):
                material.set("name", f"{side}_{material.attrib['name']}")
        actual = robot.find(f"./link[@name='{name}']")
        assert actual is not None
        assert _element_signature(actual) == _element_signature(expected)


def test_declared_optical_basis_and_no_invented_tracking_alias() -> None:
    robot = ET.fromstring(composer.render())
    for side in ("left", "right"):
        assert robot.find(f"./link[@name='{side}_vrcontroller']") is not None
        camera = robot.find(f"./link[@name='{side}_camera_link']")
        assert camera is not None
        assert camera.attrib["data-frame-convention"] == "camera_optical"
        assert robot.find(f"./link[@name='quest_{side}_controller']") is None


@pytest.mark.parametrize("side", ["left", "right"])
def test_opening_coordinates_and_pinion_rebase(side: str) -> None:
    robot = ET.fromstring(composer.render())
    for name in ("slider_1", "slider_2"):
        joint = robot.find(f"joint[@name='{side}_{name}']")
        assert joint is not None
        axis = joint.find("axis")
        assert axis is not None
        assert tuple(float(x) for x in axis.attrib["xyz"].split()) == (-1.0, 0.0, 0.0)
        origin = joint.find("origin")
        assert origin is not None
        expected_x = -0.0875 if name == "slider_1" else -0.0525
        assert tuple(float(x) for x in origin.attrib["xyz"].split()) == pytest.approx(
            (expected_x, 0.00407, 0.00477), abs=1e-12
        )
    mimic = robot.find(f"joint[@name='{side}_revolute_1']/mimic")
    assert mimic is not None
    assert float(mimic.attrib["multiplier"]) == -100
    assert float(mimic.attrib["offset"]) == 0
    pinion = robot.find(f"joint[@name='{side}_revolute_1']/origin")
    assert pinion is not None
    assert tuple(float(x) for x in pinion.attrib["rpy"].split()) == pytest.approx(
        (-0.87037469282041346, 0, 0), abs=1e-12
    )


@pytest.mark.parametrize("side", ["left", "right"])
def test_camera_joint_retains_its_authored_pose(side: str) -> None:
    robot = ET.fromstring(composer.render())
    origin = robot.find(f"joint[@name='{side}_camera_link']/origin")
    assert origin is not None
    assert tuple(float(x) for x in origin.attrib["xyz"].split()) == pytest.approx(
        (0.013999999999999974, -0.0089926909146001502, -0.015801650662099095),
        abs=1e-12,
    )
    assert tuple(float(x) for x in origin.attrib["rpy"].split()) == pytest.approx(
        (-1.7453292519944341, 0.0, -3.1415926535897931), abs=1e-12
    )


def test_disconnected_description_is_rejected() -> None:
    robot = ET.fromstring(composer.render())
    ET.SubElement(robot, "link", {"name": "orphan"})
    with pytest.raises(composer.CompositionError):
        composer.validate_tree(robot)


@pytest.mark.parametrize("two_joint_cycle", [False, True])
def test_cyclic_mimic_is_rejected(two_joint_cycle: bool) -> None:
    robot = ET.fromstring(composer.render())
    follower = robot.find("./joint[@name='left_slider_2']/mimic")
    assert follower is not None
    if two_joint_cycle:
        driver = robot.find("./joint[@name='left_slider_1']")
        assert driver is not None
        ET.SubElement(driver, "mimic", {"joint": "left_slider_2"})
    else:
        follower.set("joint", "left_slider_2")
    with pytest.raises(composer.CompositionError, match="cyclic mimic"):
        composer.validate_tree(robot)
