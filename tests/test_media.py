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
