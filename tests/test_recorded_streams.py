"""A capture rig's bindings are the body's facts, typed and resolved against the built body."""

import pytest

from sx_embodiments import (
    EmbodimentError,
    development_embodiments,
    embodiments,
    recorded_streams_of,
)
from sx_embodiments.errors import UnknownEmbodimentError
from sx_embodiments.identity import EmbodimentKind
from sx_embodiments.recorded_streams import (
    DeviceStream,
    Measurand,
    RecordedCamera,
    RecordedCoordinate,
    RecordedPose,
    RecordedStreams,
    RecordedStreamsError,
)


def _every_body() -> list[str]:
    return sorted({*embodiments, *development_embodiments})


def _built(name: str):  # noqa: ANN202 - the registry decides which one holds it
    try:
        return embodiments[name]
    except EmbodimentError:
        return development_embodiments[name]


@pytest.mark.parametrize("name", _every_body())
def test_every_bodys_bindings_resolve_against_it(name: str) -> None:
    streams = recorded_streams_of(name)
    if streams == RecordedStreams():
        return
    body = _built(name)
    assert body.kind is EmbodimentKind.CAPTURE_RIG, "only a capture rig records device streams"
    streams.resolve(body)


def test_both_pod_rigs_bind_their_hands_and_four_cameras() -> None:
    for name, link in (("yubi", "quest_left_controller"), ("piper-umi", "left_vrcontroller")):
        streams = recorded_streams_of(name)
        assert {(p.site, p.stream.topic) for p in streams.poses} == {
            ("left_hand", "/quest3s/left_controller_pose"),
            ("right_hand", "/quest3s/right_controller_pose"),
        }
        assert streams.poses[0].link == link
        assert {(c.name, c.video.topic, c.calibration.topic) for c in streams.cameras} == {
            ("head_left", "/quest3s/head_left/camera/compressed", "/quest3s/head_left/camera/camera_info"),
            ("head_right", "/quest3s/head_right/camera/compressed", "/quest3s/head_right/camera/camera_info"),
            ("left_wrist", "/gripper/left/camera0/compressed", "/gripper/left/camera0/camera_info"),
            ("right_wrist", "/gripper/right/camera0/compressed", "/gripper/right/camera0/camera_info"),
        }


def test_a_yubi_jaw_is_its_measured_opening() -> None:
    coordinates = recorded_streams_of("yubi").coordinates
    assert {(c.component, c.stream.topic, c.measurand) for c in coordinates} == {
        ("left_jaw", "/gripper/left/magnetic_encoder", Measurand.JAW_APERTURE_M),
        ("right_jaw", "/gripper/right/magnetic_encoder", Measurand.JAW_APERTURE_M),
    }
    # A Piper jaw has no gap curve, so binding its opening cannot resolve.
    unmapped = RecordedStreams(
        coordinates=(
            RecordedCoordinate(
                "left_jaw", DeviceStream("/gripper/left/magnetic_encoder"), Measurand.JAW_APERTURE_M
            ),
        )
    )
    with pytest.raises(RecordedStreamsError, match="no gap curve"):
        unmapped.resolve(_built("piper-umi"))


def test_a_binding_to_a_link_the_body_lacks_or_contradicts_is_refused() -> None:
    yubi = _built("yubi")
    stream = DeviceStream("/quest3s/head_pose")
    with pytest.raises(RecordedStreamsError, match="absent link"):
        RecordedStreams(poses=(RecordedPose("head", stream, "quest3s_head"),)).resolve(yubi)
    with pytest.raises(RecordedStreamsError, match="in the body"):
        RecordedStreams(poses=(RecordedPose("left_hand", stream, "left_hand_root"),)).resolve(yubi)


def test_a_device_stream_names_its_device_and_its_evidence() -> None:
    stream = DeviceStream("/gripper/left/magnetic_encoder")
    assert (stream.device, stream.evidence_topic) == (
        "gripper",
        "/evidence/gripper/left/magnetic_encoder",
    )
    for topic in ("gripper", "/gripper", "/gripper//x", "/evidence/gripper/x", "/a /b"):
        with pytest.raises(RecordedStreamsError):
            DeviceStream(topic)


def test_a_body_no_rig_records_declares_none() -> None:
    assert recorded_streams_of("so101") == RecordedStreams()
    with pytest.raises(UnknownEmbodimentError):
        recorded_streams_of("no-such-body")


def test_one_stream_binds_one_thing() -> None:
    stream = DeviceStream("/a/b")
    with pytest.raises(RecordedStreamsError):
        RecordedStreams(
            poses=(RecordedPose("left_hand", stream, "l"),),
            coordinates=(RecordedCoordinate("left_jaw", stream, Measurand.JAW_APERTURE_M),),
        )
    with pytest.raises(RecordedStreamsError):
        RecordedStreams(
            cameras=(
                RecordedCamera("head", "a", DeviceStream("/c/a"), DeviceStream("/c/a_info")),
                RecordedCamera("head", "b", DeviceStream("/c/b"), DeviceStream("/c/b_info")),
            )
        )


def test_the_bindings_have_their_own_identity_outside_the_body() -> None:
    yubi, piper = recorded_streams_of("yubi"), recorded_streams_of("piper-umi")
    assert len(yubi.digest) == 64 and yubi.digest != piper.digest
    assert yubi.digest == RecordedStreams(**{f: getattr(yubi, f) for f in ("poses", "coordinates", "cameras")}).digest
