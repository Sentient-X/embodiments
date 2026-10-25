"""The media plan: every declared camera is a track, a pair is one track, nothing is invented."""

import math

import pytest

from sx_embodiments import embodiments
from sx_embodiments.embodiment import Embodiment, embodiment_from_definition
from sx_embodiments.errors import EmbodimentSchemaError
from sx_embodiments.instance import InstanceError, nominal_instance, resolve
from sx_embodiments.known import definitions
from sx_embodiments.media import (
    NO_RIG,
    MediaRig,
    MediaRigError,
    MicrophoneSpec,
    StereoPair,
    media_document,
    mount_pose,
    mount_pose_at,
)

QUEST_RIG = MediaRig(
    stereo=(StereoPair("head", "head_left", "head_right"),),
    microphones=(MicrophoneSpec("head_mic", "quest3_microphone", 48_000, 1, "quest3s_head"),),
)


def _bodies() -> list[Embodiment]:
    bodies: list[Embodiment] = []
    for definition in definitions():
        try:
            bodies.append(embodiment_from_definition(definition))
        except EmbodimentSchemaError:
            continue
    return bodies


def _plan(body: Embodiment, rig: MediaRig = NO_RIG) -> dict[str, object]:
    return media_document(body, resolve(body, nominal_instance(body), ()), rig)


@pytest.mark.parametrize("body", _bodies(), ids=lambda body: str(body.name))
def test_every_body_publishes_exactly_the_cameras_it_declares(body: Embodiment) -> None:
    plan = _plan(body)
    tracks = plan["tracks"]
    assert isinstance(tracks, list)
    assert [track["track"] for track in tracks] == [camera.name for camera in body.cameras]
    assert plan["microphones"] == []


def test_a_stereo_pair_is_one_track_left_then_right() -> None:
    quest = embodiments["quest-ego"]
    plan = _plan(quest, QUEST_RIG)
    (track,) = plan["tracks"]  # type: ignore[misc]
    assert track["track"] == "head"
    assert [view["camera"] for view in track["views"]] == ["head_left", "head_right"]
    (microphone,) = plan["microphones"]  # type: ignore[misc]
    assert microphone["sample_rate_hz"] == 48_000


def test_a_pair_must_be_one_exposure_and_name_declared_cameras() -> None:
    quest = embodiments["quest-ego"]
    with pytest.raises(MediaRigError, match="declares no"):
        _plan(quest, MediaRig(stereo=(StereoPair("head", "head_left", "nose"),)))
    with pytest.raises(MediaRigError, match="two stereo pairs"):
        _plan(
            quest,
            MediaRig(
                stereo=(
                    StereoPair("a", "head_left", "head_right"),
                    StereoPair("b", "head_right", "head_left"),
                )
            ),
        )
    with pytest.raises(MediaRigError, match="not a link"):
        _plan(quest, MediaRig(microphones=(MicrophoneSpec("m", "x", 48_000, 1, "nowhere"),)))
    with pytest.raises(MediaRigError, match="Hz"):
        MicrophoneSpec("m", "x", 44_100, 1, "quest3s_head")
    for name in ("../escape", "/tmp/x", "Head", ""):
        with pytest.raises(MediaRigError, match="not a track name"):
            StereoPair(name, "head_left", "head_right")
        with pytest.raises(MediaRigError, match="not a track name"):
            MicrophoneSpec(name, "x", 48_000, 1, "quest3s_head")


def test_a_plan_is_rendered_only_for_its_own_body() -> None:
    quest, so101 = embodiments["quest-ego"], embodiments["so101"]
    with pytest.raises(InstanceError):
        media_document(so101, resolve(quest, nominal_instance(quest), ()))


def test_a_fixed_mount_carries_its_pose_and_a_moving_one_names_its_joints() -> None:
    quest = embodiments["quest-ego"]
    fixed = mount_pose(quest.urdf_bytes, quest.cameras[0].frame)
    assert fixed.pose is not None and fixed.moves_with == ()
    _, rotation = fixed.pose
    assert math.isclose(math.sqrt(sum(value * value for value in rotation)), 1.0)
    so101 = embodiments["so101"]
    moving = mount_pose(so101.urdf_bytes, "gripper_link")
    assert moving.pose is None and moving.moves_with


_MIMIC_URDF = b"""<robot name="mimic">
  <link name="base"/><link name="a"/><link name="b"/><link name="c"/>
  <joint name="lead" type="revolute"><parent link="base"/><child link="a"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/></joint>
  <joint name="first" type="prismatic"><parent link="a"/><child link="b"/>
    <origin xyz="0.2 0 0"/><axis xyz="1 0 0"/><mimic joint="lead" multiplier="2" offset="0.1"/></joint>
  <joint name="second" type="prismatic"><parent link="b"/><child link="c"/>
    <axis xyz="0 1 0"/><mimic joint="first" multiplier="-0.5" offset="0.3"/></joint>
</robot>"""


def test_a_chain_names_every_joint_root_first_and_folds_a_mimic_of_a_mimic() -> None:
    mount = mount_pose(_MIMIC_URDF, "c")
    assert mount.pose is None
    assert [step.joint for step in mount.chain] == ["lead", "first", "second"]
    assert mount.moves_with == ("lead",)
    # second = -0.5 * (2 * lead + 0.1) + 0.3 = -1 * lead + 0.25
    follows = mount.chain[2].follows
    assert follows is not None and follows[0] == "lead"
    assert math.isclose(follows[1], -1.0) and math.isclose(follows[2], 0.25)
    translation, _ = mount_pose_at(_MIMIC_URDF, "c", {"lead": 0.0})
    assert math.isclose(translation[0], 0.2 + 0.1) and math.isclose(translation[1], 0.25)


def test_every_registered_camera_view_renders_its_chain() -> None:
    for body in _bodies():
        plan = media_document(body, resolve(body, nominal_instance(body), ()))
        for track in plan["tracks"]:  # type: ignore[union-attr]
            for view in track["views"]:  # type: ignore[index]
                chain = view["chain"]  # type: ignore[index]
                moving = [step for step in chain if step["type"] != "fixed"]  # type: ignore[index]
                assert bool(moving) == bool(view["moves_with"])  # type: ignore[index]


def test_an_inertial_sensor_is_listed_with_its_axes_unit_and_mount() -> None:
    from sx_embodiments import development_embodiments
    from sx_embodiments.calibration import UnitSerial
    from sx_embodiments.instance import RobotInstance, Slot, SlotKind
    from sx_embodiments.known.static_upper_body import STATIC_UPPER_BODY_RIG

    body = development_embodiments["sentient-static-upper-body"]
    instance = RobotInstance.of(
        body.id, {Slot(SlotKind.COMPONENT, "head_imu"): UnitSerial("30371653")}, ()
    )
    plan = media_document(body, resolve(body, instance, ()), STATIC_UPPER_BODY_RIG)
    (sensor,) = plan["inertial"]  # type: ignore[misc]
    assert sensor["sensor"] == "head_imu"
    assert sensor["model"] == "zed_2i"
    assert sensor["unit"] == "30371653"
    assert sensor["mount_frame"] == "ZED_camera"
    assert sensor["rate_hz"] == 400.0
    assert {"acceleration", "angular_velocity"} <= set(sensor["measures"])
    # Stereolabs' RAW axes: x down, y left, z forward, a quarter turn about the ZED frame's y.
    half = math.sqrt(0.5)
    assert sensor["mount_from_sensor"]["rotation_xyzw"] == [0.0, half, 0.0, half]
    assert "so101_camera_tilt_joint" in sensor["moves_with"]
    assert sensor["root_from_mount"] is None
    assert Embodiment.from_dict(body.to_dict()) == body


def test_an_inertial_part_measures_motion_on_a_rigid_pose() -> None:
    from sx_embodiments.errors import PartValidationError
    from sx_embodiments.identity import PartId
    from sx_embodiments.known.zed import zed_2i_imu
    from sx_embodiments.parts import InertialQuantity, InertialSpec, SensorModel

    source = zed_2i_imu().source
    with pytest.raises(PartValidationError):
        InertialSpec(
            PartId("x"),
            SensorModel("x"),
            100.0,
            (InertialQuantity.ACCELERATION,),
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            source,
        )
    with pytest.raises(PartValidationError):
        InertialSpec(
            PartId("x"),
            SensorModel("x"),
            100.0,
            (InertialQuantity.ACCELERATION, InertialQuantity.ANGULAR_VELOCITY),
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 2.0)),
            source,
        )
