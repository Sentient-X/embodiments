"""The static upper body is its generator's rendering of the received file and its motor table."""

import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from sx_embodiments.compose import BaseMount, Direction, MountKind, SupportEdge
from sx_embodiments.embodiment import Embodiment, embodiment_from_definition
from sx_embodiments.errors import ComponentGraphError, EmbodimentSchemaError
from sx_embodiments.kinematics import Kinematics
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
        if path.suffix == ".urdf":
            _assert_same_description(path.read_bytes(), data, path.relative_to(ROOT))
        else:
            assert path.read_bytes() == data, path.relative_to(ROOT)


def _numbers(text: str) -> list[float] | None:
    try:
        return [float(token) for token in text.split()]
    except ValueError:
        return None


def _assert_same_description(committed: bytes, rendered: bytes, where: object) -> None:
    """The committed description is the rendering, element for element and attribute for
    attribute, its numbers to 1e-6 relative (a micrometre in a metre). The hull-estimated
    inertials come from qhull, whose last bits differ between macOS and Linux (CI renders on
    Linux), so a byte comparison fails on one of them; re-rendering the committed bytes instead
    would re-mint the body's identity and the dynamics evidence measured against it, for noise
    far below any physics."""
    ours, theirs = ET.fromstring(committed), ET.fromstring(rendered)
    mine, others = list(ours.iter()), list(theirs.iter())
    assert [e.tag for e in mine] == [e.tag for e in others], where
    for a, b in zip(mine, others, strict=True):
        assert a.attrib.keys() == b.attrib.keys(), (where, a.tag)
        for key, value in a.attrib.items():
            left, right = _numbers(value), _numbers(b.attrib[key])
            if left is None or right is None or len(left) != len(right):
                assert value == b.attrib[key], (where, a.tag, key)
            else:
                assert left == pytest.approx(right, rel=1e-6, abs=1e-9), (where, a.tag, key)
    assert committed.count(b"\n") == rendered.count(b"\n"), where


def test_the_body_is_a_table_clamped_development_body() -> None:
    entry = DEVELOPMENT_EMBODIMENTS[STATIC_UPPER_BODY_SPEC.name]
    assert STATIC_UPPER_BODY_SPEC.base_mount is not None
    assert STATIC_UPPER_BODY_SPEC.base_mount.kind is MountKind.CLAMP_EDGE
    robot = embodiment_from_definition(entry.spec)
    assert len(robot.joint_names) == 25


def _hull_xy(path: Path) -> tuple[float, float, float, float]:
    import numpy as np

    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    rows = np.frombuffer(
        data[84 : 84 + 50 * count],
        dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]),
    )
    vertices = rows["v"].reshape(-1, 3)
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    return float(low[0]), float(high[0]), float(low[1]), float(high[1])


def test_the_table_ends_at_the_clamp_plates_front_and_lies_behind_the_body() -> None:
    """The footprint is base_link's collision hull; the body faces +x, out from the table, so
    the table's edge is the plate's front and the table lies on -x (2026-10-26 bench)."""
    pytest.importorskip("numpy")
    mount = STATIC_UPPER_BODY_SPEC.base_mount
    assert mount is not None
    edge = mount.edge
    assert edge is not None
    assert edge == SupportEdge(out=Direction.POS_X, at_m=0.0549)
    x0, x1, y0, y1 = _hull_xy(
        ROOT / "assets/sentient_static_upper_body/meshes/collision/base_link.stl"
    )
    for got, want in zip(
        (x0, x1, y0, y1),
        (
            mount.centre[0] - mount.half_extents[0],
            mount.centre[0] + mount.half_extents[0],
            mount.centre[1] - mount.half_extents[1],
            mount.centre[1] + mount.half_extents[1],
        ),
        strict=True,
    ):
        assert abs(got - want) < 1e-4
    assert abs(x1 - edge.at_m) < 1e-4
    # Facing +x: both tool points sit well in front of the edge with every joint at zero.
    urdf = (ROOT / "assets/sentient_static_upper_body/sentient_static_upper_body.urdf").read_bytes()
    kinematics = Kinematics.of(urdf)
    zero = dict.fromkeys(kinematics.independent, 0.0)
    for tcp in ("left_gripper_tcp", "right_gripper_tcp"):
        assert kinematics.pose("base_link", tcp, zero)[3] > edge.at_m + 0.3


def test_a_support_edge_round_trips_and_a_footprint_past_it_is_refused() -> None:
    robot = embodiment_from_definition(STATIC_UPPER_BODY_SPEC)
    document = json.loads(robot.to_json())
    assert document["base_mount"]["edge"] == {"out": "pos_x", "at_m": 0.0549}
    assert Embodiment.from_json(robot.to_json()) == robot
    document["base_mount"]["edge"] = None
    with pytest.raises(EmbodimentSchemaError):
        Embodiment.from_json(json.dumps(document))
    with pytest.raises(ComponentGraphError, match="past its support's edge"):
        BaseMount(
            kind=MountKind.CLAMP_EDGE,
            frame="base_link",
            half_extents=(0.06, 0.05),
            edge=SupportEdge(out=Direction.POS_X, at_m=0.0549),
        )
    with pytest.raises(ComponentGraphError, match="past its support's edge"):
        BaseMount(
            kind=MountKind.CLAMP_EDGE,
            frame="base_link",
            half_extents=(0.05, 0.05),
            centre=(-0.02, 0.0),
            edge=SupportEdge(out=Direction.NEG_X, at_m=0.06),
        )


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
        # The profile writes the torso's and shoulders' DM-J8009P-2EC as "DM8009".
        candidate = {"dm8009": "dm8009p"}.get(motor["model_candidate"].lower())
        candidate = candidate or motor["model_candidate"].lower()
        assert binding.model.value == f"damiao_{candidate}", axis.name
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
        if name == "base_link":  # the camera tilt horn moves onto the bracket it turns with
            parts = [part for part in parts if part[0] != "_________step__Body1_1_.stl"]
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
