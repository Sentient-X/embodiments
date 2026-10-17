"""The humanoid's bodies are assemblies of parts cut from one description, and its drive."""

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
from sx_embodiments.kinematics import IDENTITY, Kinematics
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
        expected = Kinematics.of(source).poses("pelvis", joints)
        found = Kinematics.of(rejoined).poses("pelvis", joints)
        for name, pose in expected.items():
            worst = max(abs(a - b) for a, b in zip(pose, found[name], strict=True))
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
    kinematics = Kinematics.of(variant.urdf_bytes)
    assert kinematics.is_rigid("body/head_link", "neck/base_link")
    assert kinematics.pose("body/head_link", "neck/base_link") == pytest.approx(IDENTITY)
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
