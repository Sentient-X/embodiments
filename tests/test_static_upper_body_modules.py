"""Corrected upper-body parts detach without changing physical facts or old recordings."""

import random

import pytest

from sx_embodiments.embodiment import embodiment_from_definition
from sx_embodiments.identity import EmbodimentId, EmbodimentName
from sx_embodiments.kinematics import Kinematics
from sx_embodiments.known.static_upper_body import (
    STATIC_UPPER_BODY_HEAD_MODULE,
    STATIC_UPPER_BODY_LEFT_ARM_MODULE,
    STATIC_UPPER_BODY_MODULE,
    STATIC_UPPER_BODY_RIGHT_ARM_MODULE,
    STATIC_UPPER_BODY_SPEC,
    STATIC_UPPER_BODY_URDF,
)
from sx_embodiments.mechanical import Build, Mate, Placed
from sx_embodiments.recorded import historical_document, read_recorded
from sx_embodiments.successor import successor

PREVIOUS = EmbodimentId("51db79c80289f76c9f20f2043c114836dfcb80baca88e8f5094762c925a37402")


def test_previous_recordings_keep_their_native_order_and_physical_facts() -> None:
    old = read_recorded(historical_document(EmbodimentName("sentient-static-upper-body"), PREVIOUS))
    current = embodiment_from_definition(STATIC_UPPER_BODY_SPEC)
    assert old.recorded_id == old.embodiment.id == PREVIOUS
    assert current.id != PREVIOUS
    assert successor(str(PREVIOUS)) is None
    assert (
        old.embodiment.urdf_bytes
        == current.urdf_bytes
        == STATIC_UPPER_BODY_URDF.path().read_bytes()
    )
    assert tuple(c.instance for c in old.embodiment.components) == (
        "torso",
        "left_arm",
        "right_arm",
        "left_gripper",
        "right_gripper",
        "neck",
        "head_left",
        "head_right",
        "head_imu",
    )
    assert tuple(c.instance for c in current.components) == (
        "torso",
        "left_arm",
        "left_gripper",
        "right_arm",
        "right_gripper",
        "neck",
        "head_left",
        "head_right",
        "head_imu",
    )
    assert old.embodiment.joint_names != current.joint_names
    assert {c.instance: c for c in old.embodiment.components} == {
        c.instance: c for c in current.components
    }
    assert {site.name: site for site in old.embodiment.sites} == {
        site.name: site for site in current.sites
    }


@pytest.mark.parametrize(
    ("socket", "module"),
    [
        ("left_shoulder", STATIC_UPPER_BODY_LEFT_ARM_MODULE),
        ("right_shoulder", STATIC_UPPER_BODY_RIGHT_ARM_MODULE),
        ("neck", STATIC_UPPER_BODY_HEAD_MODULE),
    ],
)
def test_corrected_parts_rejoin_the_source_with_identical_kinematics(socket, module) -> None:
    assert STATIC_UPPER_BODY_MODULE.socket(socket).fitted is module
    source = Kinematics.of(STATIC_UPPER_BODY_URDF.path().read_bytes())
    rejoined = Build(
        robot="corrected_upper_body",
        root="origin",
        placed=(Placed("body", STATIC_UPPER_BODY_MODULE, namespaced=False),),
        mates=(Mate("part", module, "body", socket, namespaced=False),),
    )
    found = Kinematics.of(rejoined.urdf())
    random_values = random.Random(42)
    for _ in range(3):
        positions = {name: random_values.uniform(-0.2, 0.2) for name in source.independent}
        expected = source.poses("base_link", positions)
        actual = found.poses("base_link", positions)
        for link, pose in expected.items():
            assert actual[link] == pytest.approx(pose, abs=1e-12), link
    assert {c.instance: c for c in rejoined.components()} == {
        c.instance: c for c in STATIC_UPPER_BODY_SPEC.build.components()
    }


@pytest.mark.parametrize(
    "arm", [STATIC_UPPER_BODY_LEFT_ARM_MODULE, STATIC_UPPER_BODY_RIGHT_ARM_MODULE]
)
def test_reusable_corrected_arm_owns_its_same_gripper(arm) -> None:
    gripper = arm.socket("wrist").fitted
    assert gripper is not None
    body = Build.of(arm)
    assert len(body.components()) == 2
    current = {c.instance: c for c in STATIC_UPPER_BODY_SPEC.build.components()}
    for component in body.components():
        assert component.part is current[component.instance].part
    reopened = Build(
        robot="corrected_arm",
        root="origin",
        placed=(Placed("arm", arm, namespaced=False),),
        mates=(Mate("gripper", gripper, "arm", "wrist", namespaced=False),),
    )
    source = Kinematics.of(body.urdf())
    found = Kinematics.of(reopened.urdf())
    frame = f"{body.components()[0].instance.removesuffix('_arm')}_shoulder_pitch_joint_mount"
    positions = dict.fromkeys(source.independent, 0.1)
    for link, pose in source.poses(frame, positions).items():
        assert found.pose(frame, link, positions) == pytest.approx(pose, abs=1e-12)
    assert body.components() == reopened.components()
