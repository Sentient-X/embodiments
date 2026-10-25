"""The static upper body is its generator's rendering of the received file and its motor table."""

import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from sx_embodiments.compose import MountKind
from sx_embodiments.embodiment import embodiment_from_definition
from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS
from sx_embodiments.known.static_upper_body import (
    STATIC_UPPER_BODY_LEFT_ARM,
    STATIC_UPPER_BODY_MASTER_ID_OFFSET,
    STATIC_UPPER_BODY_RIGHT_ARM,
    STATIC_UPPER_BODY_SPEC,
    STATIC_UPPER_BODY_TORSO,
)
from sx_embodiments.layout import Bounds, DirectDrive

ROOT = Path(__file__).resolve().parents[1]


def test_the_description_is_the_generators_rendering() -> None:
    pytest.importorskip("scipy")
    spec = importlib.util.spec_from_file_location(
        "compose_static_upper_body_urdf", ROOT / "tools/compose_static_upper_body_urdf.py"
    )
    assert spec is not None and spec.loader is not None
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    for path, data in tool.render().items():
        assert path.read_bytes() == data, path.relative_to(ROOT)


def test_the_body_is_a_table_clamped_development_body() -> None:
    entry = DEVELOPMENT_EMBODIMENTS[STATIC_UPPER_BODY_SPEC.name]
    assert STATIC_UPPER_BODY_SPEC.base_mount is not None
    assert STATIC_UPPER_BODY_SPEC.base_mount.kind is MountKind.CLAMP_EDGE
    robot = embodiment_from_definition(entry.spec)
    assert len(robot.joint_names) == 25


def test_the_arm_bindings_are_the_hardware_profile() -> None:
    """Ids, models and signs are humanoid_pkg's hardware profile for allisa_bot; each arm
    joint's range is the rendered description's limit."""
    source = ROOT / "assets/sentient_static_upper_body/source"
    motors = json.loads((source / "humanoid_allisa_bot_hardware.json").read_text())["hardware"][
        "motors"
    ]
    limits = {
        joint.attrib["name"]: joint.find("limit")
        for joint in ET.parse(
            ROOT / "assets/sentient_static_upper_body/sentient_static_upper_body.urdf"
        )
        .getroot()
        .findall("joint")
    }
    axes = [
        axis
        for chain in (STATIC_UPPER_BODY_LEFT_ARM, STATIC_UPPER_BODY_RIGHT_ARM)
        for axis in chain.layout.axes
    ]
    assert sorted(axis.name for axis in axes) == sorted(motors)
    for axis in axes:
        motor = motors[axis.name]
        assert isinstance(axis.actuation, DirectDrive) and isinstance(axis.bounds, Bounds)
        binding = axis.actuation.actuator
        assert binding.model.value == f"damiao_{motor['model_candidate'].lower()}", axis.name
        assert binding.bus_id == motor["send_id"], axis.name
        assert motor["recv_id"] == binding.bus_id + STATIC_UPPER_BODY_MASTER_ID_OFFSET
        assert (binding.sign, motor["zero_rad"]) == (motor["sign"], 0.0), axis.name
        limit = limits[axis.name]
        assert limit is not None
        assert axis.bounds == Bounds(float(limit.attrib["lower"]), float(limit.attrib["upper"]))
    # The profile has no torso motor; its binding is the superseded description's table.
    torso = STATIC_UPPER_BODY_TORSO.layout.axes[0].actuation
    assert isinstance(torso, DirectDrive) and torso.actuator.bus_id == 0x1F


def test_the_description_is_allisa_bot_with_the_wrists_in_motor_limits() -> None:
    """Every source joint but the neck's renders with its origin and axis; the four reversed
    wrist axes carry the donor's motor-coordinate limits negated, every other joint its limits
    unchanged."""
    asset = ROOT / "assets/sentient_static_upper_body"
    source = {
        j.attrib["name"]: j
        for j in ET.parse(asset / "source/allisa_bot.urdf").getroot().findall("joint")
    }
    rendered = {
        j.attrib["name"]: j
        for j in ET.parse(asset / "sentient_static_upper_body.urdf").getroot().findall("joint")
    }
    reversed_axes = {
        "left_wrist_pitch_joint",
        "left_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_roll_joint",
    }
    neck = {name for name in source if name.startswith("so101_") and name != "so101_mount_joint"}
    assert set(source) <= set(rendered)
    for name, joint in source.items():
        if name in neck or name == "zed2i_left_optical_joint" or name == "so101_mount_joint":
            continue  # the official neck's, below
        ours = rendered[name]
        for tag in ("origin", "axis", "parent", "child"):
            theirs, mine = joint.find(tag), ours.find(tag)
            assert (theirs is None) == (mine is None), (name, tag)
            if theirs is not None and mine is not None:
                assert theirs.attrib == mine.attrib, (name, tag)
        theirs, mine = joint.find("limit"), ours.find("limit")
        if theirs is None or mine is None:
            assert theirs is mine, name
            continue
        lower, upper = float(theirs.attrib["lower"]), float(theirs.attrib["upper"])
        expected = (-upper, -lower) if name in reversed_axes else (lower, upper)
        assert (float(mine.attrib["lower"]), float(mine.attrib["upper"])) == expected, name


def test_every_link_off_the_neck_draws_allisa_bots_meshes_where_it_does() -> None:
    asset = ROOT / "assets/sentient_static_upper_body"

    def visuals(path: Path) -> dict[str, list[tuple[str, dict[str, str]]]]:
        out: dict[str, list[tuple[str, dict[str, str]]]] = {}
        for link in ET.parse(path).getroot().findall("link"):
            parts: list[tuple[str, dict[str, str]]] = []
            for visual in link.findall("visual"):
                mesh, origin = visual.find("geometry/mesh"), visual.find("origin")
                assert mesh is not None and origin is not None
                parts.append((mesh.attrib["filename"].rsplit("/", 1)[-1], dict(origin.attrib)))
            out[link.attrib["name"]] = parts
        return out

    source = visuals(asset / "source/allisa_bot.urdf")
    rendered = visuals(asset / "sentient_static_upper_body.urdf")
    for name, parts in source.items():
        if name.startswith("so101_") or name == "camera_link":
            continue  # the official neck, carrying allisa_bot's base, holder, bracket and ZED
        assert rendered[name] == parts, name


def test_the_neck_is_bound_provisionally_as_an_so101_on_its_own_port() -> None:
    from sx_embodiments.known.static_upper_body import (
        STATIC_UPPER_BODY_NECK,
        STATIC_UPPER_BODY_NECK_PORT,
    )
    from sx_embodiments.layout import ActuatorBus, ActuatorModel

    bindings = [axis.actuator for axis in STATIC_UPPER_BODY_NECK.layout.axes]
    assert [binding.bus_id for binding in bindings if binding is not None] == [1, 2, 3, 4, 5, 6]
    for binding in bindings:
        assert binding is not None
        assert binding.model is ActuatorModel.FEETECH_STS3215
        assert binding.bus is ActuatorBus.FEETECH_SERIAL
        assert binding.bus_port == STATIC_UPPER_BODY_NECK_PORT == "neck"
    robot = embodiment_from_definition(DEVELOPMENT_EMBODIMENTS[STATIC_UPPER_BODY_SPEC.name].spec)
    assert all(c.axis.actuator is not None for c in robot.state.coordinates)


def test_each_gripper_is_two_feetech_servos_on_a_bus_of_its_own() -> None:
    from sx_embodiments.known.static_upper_body import (
        STATIC_UPPER_BODY_LEFT_GRIPPER,
        STATIC_UPPER_BODY_RIGHT_GRIPPER,
    )
    from sx_embodiments.layout import ActuatorModel

    for gripper, port, ids in (
        (STATIC_UPPER_BODY_RIGHT_GRIPPER, "right_gripper", [1, 2]),
        (STATIC_UPPER_BODY_LEFT_GRIPPER, "left_gripper", [3, 4]),
    ):
        bindings = [axis.actuator for axis in gripper.layout.axes]
        assert [b.bus_id for b in bindings if b is not None] == ids
        assert all(
            b is not None and b.model is ActuatorModel.FEETECH_STS3215 and b.bus_port == port
            for b in bindings
        )


def test_the_neck_is_the_official_new_calibration_so101() -> None:
    """Below its base the neck is TheRobotStudio's so101_new_calib chain under our names, so a
    LeRobot degree reading is its joint angle; the camera tilt keeps the official origin and
    axis and declares the free arc its camera rests in."""
    asset = ROOT / "assets/sentient_static_upper_body"
    official = {
        j.attrib["name"]: j
        for j in ET.parse(asset / "source/so101_new_calib.urdf").getroot().findall("joint")
    }
    ours = {
        j.attrib["name"]: j
        for j in ET.parse(asset / "sentient_static_upper_body.urdf").getroot().findall("joint")
    }
    names = {
        "shoulder_pan": "so101_shoulder_pan_joint",
        "shoulder_lift": "so101_shoulder_lift_joint",
        "elbow_flex": "so101_elbow_flex_joint",
        "wrist_flex": "so101_wrist_flex_joint",
        "wrist_roll": "so101_wrist_roll_joint",
        "gripper": "so101_camera_tilt_joint",
    }

    def numbers(joint: ET.Element, tag: str, keys: tuple[str, ...]) -> list[float]:
        element = joint.find(tag)
        assert element is not None, tag
        return [float(v) for key in keys for v in element.attrib[key].split()]

    from sx_embodiments.known.static_upper_body import STATIC_UPPER_BODY_NECK

    bounds = {axis.name: axis.bounds for axis in STATIC_UPPER_BODY_NECK.layout.axes}
    for upstream, name in names.items():
        theirs, mine = official[upstream], ours[name]
        tags = [("origin", ("xyz", "rpy")), ("axis", ("xyz",))]
        if upstream != "gripper":
            tags.append(("limit", ("lower", "upper")))
        for tag, keys in tags:
            assert numbers(mine, tag, keys) == pytest.approx(
                numbers(theirs, tag, keys), abs=1e-6
            ), (name, tag)
        lower, upper = numbers(mine, "limit", ("lower", "upper"))
        assert bounds[name] == Bounds(lower, upper), name
