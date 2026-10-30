"""A capture rig's recorded names for a body's devices are the body's facts, checked against it."""

import pytest

from sx_embodiments import development_embodiments, recorded_streams_of
from sx_embodiments.errors import UnknownEmbodimentError
from sx_embodiments.recorded_streams import (
    RecordedCamera,
    RecordedCoordinate,
    RecordedPose,
    RecordedStreams,
    RecordedStreamsError,
)


def test_yubi_names_each_hands_controller_and_each_jaws_encoder() -> None:
    yubi = development_embodiments["yubi"]
    streams = recorded_streams_of("yubi")
    sites = {mount.site.value for mount in yubi.operator_mounts}
    assert {pose.site for pose in streams.poses} == sites
    jaws = {str(c.component_id) for c in yubi.components}
    assert {item.component for item in streams.coordinates} <= jaws
    assert {pose.frame for pose in streams.poses} == {
        "quest3s_left_controller",
        "quest3s_right_controller",
    }


def test_a_body_no_rig_records_declares_none() -> None:
    assert recorded_streams_of("so101") == RecordedStreams()
    with pytest.raises(UnknownEmbodimentError):
        recorded_streams_of("no-such-body")


def test_one_frame_names_one_device() -> None:
    with pytest.raises(RecordedStreamsError):
        RecordedStreams(
            poses=(RecordedPose(site="left_hand", frame="a"),),
            coordinates=(RecordedCoordinate(component="left_jaw", frame="a"),),
        )


def test_a_pods_cameras_are_named_as_reviewers_read_them() -> None:
    for body in ("yubi", "piper-umi"):
        names = {camera.frame: camera.name for camera in recorded_streams_of(body).cameras}
        assert names == {
            "quest3s_left_camera_optical": "head_left",
            "quest3s_right_camera_optical": "head_right",
            "left_hand_cam_optical": "left_wrist",
            "right_hand_cam_optical": "right_wrist",
        }


def test_two_cameras_never_share_a_name() -> None:
    with pytest.raises(RecordedStreamsError):
        RecordedStreams(
            cameras=(RecordedCamera(frame="a", name="head"), RecordedCamera(frame="b", name="head"))
        )
