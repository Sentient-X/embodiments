"""The humanoid's bodies are assemblies of parts cut from one description, and its drive."""

import math
import random
import xml.etree.ElementTree as ET

import pytest

from sx_embodiments import PlacedEmbodiment, compose_embodiments, embodiments
from sx_embodiments.assembly import Assembly, Member, Pose, Subtree
from sx_embodiments.errors import CompositionError
from sx_embodiments.instance import (
    InstanceError,
    drive_document,
    nominal_instance,
    resolve,
)
from sx_embodiments.known.humanoid import (
    HEAD,
    SENTIENT_HUMANOID_ASSEMBLY,
    SENTIENT_HUMANOID_SPEC,
    SENTIENT_HUMANOID_UPPER_BODY_ASSEMBLY,
    SENTIENT_HUMANOID_UPPER_BODY_SPEC,
    SENTIENT_HUMANOID_URDF,
)
from sx_embodiments.known.so101 import SO101_URDF
from sx_embodiments.layout import ActuatorBus, ActuatorModel, DirectDrive, UndocumentedDrive
from sx_embodiments.parts import Limb

Matrix = list[list[float]]


def _matmul(a: Matrix, b: Matrix) -> Matrix:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _transform(xyz: list[float], rpy: list[float]) -> Matrix:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    rotation = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    return [[*rotation[i], xyz[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def _about(axis: list[float], angle: float) -> Matrix:
    norm = math.sqrt(sum(a * a for a in axis))
    x, y, z = (a / norm for a in axis)
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    rotation = [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
    return [[*rotation[i], 0.0] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def _numbers(element: ET.Element | None, key: str) -> list[float]:
    text = "0 0 0" if element is None else element.get(key, "0 0 0")
    return [float(v) for v in text.split()]


def link_poses(urdf: bytes, joints: dict[str, float], frame: str) -> dict[str, Matrix]:
    """Every link's pose in ``frame`` at the given joint values: plain forward kinematics."""
    robot = ET.fromstring(urdf)
    below: dict[str, list[ET.Element]] = {}
    for joint in robot.findall("joint"):
        parent = joint.find("parent")
        assert parent is not None
        below.setdefault(parent.attrib["link"], []).append(joint)
    children = {c.attrib["link"] for c in robot.iter("child")}
    (root,) = {link.attrib["name"] for link in robot.findall("link")} - children
    identity = _transform([0.0] * 3, [0.0] * 3)
    poses = {root: identity}
    stack = [root]
    while stack:
        parent = stack.pop()
        for joint in below.get(parent, []):
            origin = joint.find("origin")
            moved = _matmul(
                poses[parent], _transform(_numbers(origin, "xyz"), _numbers(origin, "rpy"))
            )
            value = joints.get(joint.attrib["name"], 0.0)
            axis = (
                _numbers(joint.find("axis"), "xyz")
                if joint.find("axis") is not None
                else [
                    1.0,
                    0.0,
                    0.0,
                ]
            )
            if joint.get("type") in ("revolute", "continuous"):
                moved = _matmul(moved, _about(axis, value))
            child = joint.find("child")
            assert child is not None
            poses[child.attrib["link"]] = moved
            stack.append(child.attrib["link"])
    base = poses[frame]
    inverse = _invert(base)
    return {name: _matmul(inverse, pose) for name, pose in poses.items()}


def _invert(pose: Matrix) -> Matrix:
    rotation = [[pose[j][i] for j in range(3)] for i in range(3)]
    translation = [-sum(rotation[i][k] * pose[k][3] for k in range(3)) for i in range(3)]
    return [[*rotation[i], translation[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def _movable(urdf: bytes) -> list[str]:
    return [
        joint.attrib["name"]
        for joint in ET.fromstring(urdf).findall("joint")
        if joint.get("type") != "fixed"
    ]


def test_the_reassembled_parts_are_the_vendors_robot() -> None:
    """Every link of the source sits where the source puts it, at any joint values."""
    source = SENTIENT_HUMANOID_URDF.path().read_bytes()
    rejoined = SENTIENT_HUMANOID_ASSEMBLY.urdf()
    assert sorted(_movable(source)) == sorted(_movable(rejoined))
    source_links = {
        link.attrib["name"]: ET.canonicalize(ET.tostring(link), strip_text=True)
        for link in ET.fromstring(source).findall("link")
    }
    rejoined_links = {
        link.attrib["name"]: ET.canonicalize(ET.tostring(link), strip_text=True)
        for link in ET.fromstring(rejoined).findall("link")
    }
    for name, element in source_links.items():
        assert rejoined_links[name] == element, name  # geometry, inertia and meshes untouched
    rng = random.Random(15)
    for _ in range(4):
        joints = {name: rng.uniform(-1.0, 1.0) for name in _movable(source)}
        expected = link_poses(source, joints, "pelvis")
        found = link_poses(rejoined, joints, "pelvis")
        for name, pose in expected.items():
            worst = max(
                abs(a - b)
                for row, other in zip(pose, found[name], strict=True)
                for a, b in zip(row, other, strict=True)
            )
            assert worst < 1e-12, name


def test_the_upper_body_is_the_humanoid_without_its_legs() -> None:
    full = {
        link.attrib["name"]
        for link in ET.fromstring(SENTIENT_HUMANOID_ASSEMBLY.urdf()).iter("link")
    }
    upper = {
        link.attrib["name"]
        for link in ET.fromstring(SENTIENT_HUMANOID_UPPER_BODY_ASSEMBLY.urdf()).iter("link")
    }
    legs = full - upper
    assert upper < full
    assert all(
        name.startswith(("left_", "right_"))
        and any(part in name for part in ("hip", "knee", "foot", "ankle"))
        for name in legs
    ), legs


def test_the_full_humanoid_is_the_upper_body_plus_legs() -> None:
    upper = SENTIENT_HUMANOID_UPPER_BODY_SPEC.attachments
    full = SENTIENT_HUMANOID_SPEC.attachments
    assert full[: len(upper)] == upper
    assert {component.limb for component in full[len(upper) :]} == {Limb.LEG}


def test_the_upper_body_drives_its_arms_and_torso_yaw_on_one_damiao_bus() -> None:
    body = embodiments["sentient-humanoid-upper-body"]
    driven = {
        coordinate.joint_name: coordinate.axis.actuator
        for coordinate in body.state.coordinates
        if coordinate.axis.actuator is not None
    }
    assert set(driven) == {
        "torso_joint",
        *(
            f"{side}_{name}_joint"
            for side in ("right", "left")
            for name in (
                "shoulder_pitch",
                "shoulder_roll",
                "elbow_yaw",
                "elbow_forearm_yaw",
                "wrist_yaw",
            )
        ),
    }
    assert {binding.bus for binding in driven.values()} == {ActuatorBus.DAMIAO_CAN}
    assert {binding.bus_id for binding in driven.values()} == {
        0x0F,
        *range(0x10, 0x15),
        *range(0x18, 0x1D),
    }
    shoulders = [b for name, b in driven.items() if "shoulder" in name or name == "torso_joint"]
    assert {binding.model for binding in shoulders} == {ActuatorModel.DAMIAO_DM8009}
    waist = [c for c in body.state.coordinates if c.joint_name.startswith("waist_")]
    assert waist and all(isinstance(c.axis.actuation, UndocumentedDrive) for c in waist)
    assert all(
        isinstance(c.axis.actuation, DirectDrive) and c.axis.actuator is not None
        for c in body.state.coordinates
        if c.joint_name in driven
    )


def test_an_so101_neck_composes_on_the_head_and_rides_with_the_torso() -> None:
    upper = embodiments["sentient-humanoid-upper-body"]
    neck = embodiments["so101"]
    variant = compose_embodiments(
        "sentient-humanoid-so101-neck",
        {"body": PlacedEmbodiment(upper), "neck": PlacedEmbodiment(neck, parent="body/head_link")},
        label="Sentient humanoid upper body with an SO-101 neck",
    )
    assert len(variant.state.coordinates) == len(upper.state.coordinates) + len(
        neck.state.coordinates
    )
    poses = link_poses(variant.urdf_bytes, {}, "body/pelvis")
    on_head = poses["neck/base_link"]
    head = poses["body/head_link"]
    assert [v for row in on_head for v in row] == pytest.approx([v for row in head for v in row])
    arm = next(c for c in variant.components if c.instance == "neck.arm")
    assert getattr(arm.mount, "parent", None) == "body.torso"
    drivers = {
        c.axis.actuator.bus for c in variant.state.coordinates if c.axis.actuator is not None
    }
    assert drivers == {ActuatorBus.DAMIAO_CAN, ActuatorBus.FEETECH_SERIAL}


def test_a_cut_names_joints_the_description_has() -> None:
    with pytest.raises(CompositionError):
        Subtree(SENTIENT_HUMANOID_URDF, below="no_such_joint").urdf()
    with pytest.raises(CompositionError):
        Subtree(SENTIENT_HUMANOID_URDF, without=("no_such_joint",)).urdf()
    with pytest.raises(CompositionError):
        Member("head", HEAD, Pose((0.0, 0.0, 1.0)), seat=HEAD)


def test_parts_keeping_their_names_still_cannot_collide() -> None:
    twice = Assembly(
        "twice",
        "origin",
        (Member("a", SO101_URDF, namespaced=False), Member("b", SO101_URDF, namespaced=False)),
    )
    with pytest.raises(CompositionError, match="collide"):
        twice.urdf()


def test_a_drive_is_rendered_from_the_resolved_instance_and_only_for_its_body() -> None:
    body = embodiments["sentient-humanoid-upper-body"]
    document = drive_document(body, resolve(body, nominal_instance(body), ()))
    assert document["embodiment"] == str(body.id)
    joints = document["joints"]
    assert isinstance(joints, list)
    assert [joint["joint"] for joint in joints if isinstance(joint, dict)] == [
        c.joint_name for c in body.state.coordinates if c.axis.actuator is not None
    ]
    other = embodiments["b601-dm"]
    with pytest.raises(InstanceError):
        drive_document(other, resolve(body, nominal_instance(body), ()))


def test_a_cut_that_leaves_a_mimic_leader_out_is_refused() -> None:
    from sx_embodiments.known.b601 import B601_DM_URDF

    with pytest.raises(CompositionError, match="mimics"):
        Subtree(B601_DM_URDF, without=("gripper_joint1",)).urdf()
