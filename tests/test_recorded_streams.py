"""A capture rig's recorded names for a body's devices are the body's facts, checked against it."""

import pytest

from sx_embodiments import development_embodiments, recorded_streams_of
from sx_embodiments.errors import UnknownEmbodimentError
from sx_embodiments.recorded_streams import (
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
