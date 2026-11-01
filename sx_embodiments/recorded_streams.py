"""A capture rig's bindings: which recorded device stream is which of the body's sites,
coordinates and cameras.

A pod records a handheld body by its devices, not by its description: the recorder writes each
device's readings on the device's own stream (``/quest3s/left_controller_pose``,
``/gripper/left/magnetic_encoder``). The one episode format
(docs/plans/one-episode-format-2026-10.md) writes the body instead: ``/body/site/<site>/pose``,
``/body/joint_state`` and ``/camera/<camera>/…``, keeping each raw stream byte for byte on
``/evidence/<device>/<stream>``. Only the recorder and the seal name a device, and only through
these bindings, declared once beside the body, so no reader matches topics or name prefixes.

* :class:`DeviceStream`: one stream a recorder writes, by its topic.
* :class:`RecordedPose`: the stream whose poses are world-from-``link`` for one site.
* :class:`RecordedCoordinate`: the stream that measures one component's single coordinate,
  and what it measures (:class:`Measurand`), which says how it becomes the joint position.
* :class:`RecordedCamera`: the video and calibration streams of one camera, the optical frame
  its pictures are in, and the name a person reads for it.

Like viewing facts, these are outside an embodiment's content identity: declaring or correcting
one moves no id. Their own identity is :attr:`RecordedStreams.digest`, which an episode in the
one format records beside the document it was bound against.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING

from sx_contracts.identity import JsonObject, content_digest

from .errors import EmbodimentError

if TYPE_CHECKING:
    from .embodiment import Embodiment

EVIDENCE_PREFIX = "/evidence"
"""Where the one episode format keeps a device stream's readings, byte for byte."""


class RecordedStreamsError(EmbodimentError):
    """A recorded-stream fact is malformed, or does not resolve against its body."""


class Measurand(StrEnum):
    """What a recorded coordinate stream's value is, and so how it becomes the coordinate."""

    JAW_APERTURE_M = "jaw_aperture_m"
    """A jaw's opening in metres; the component's gap curve maps it to the joint position."""


@dataclass(frozen=True, slots=True)
class DeviceStream:
    """One stream a capture device's recorder writes, named by its topic: ``/<device>/<stream>``,
    the device being its first segment."""

    topic: str

    def __post_init__(self) -> None:
        segments = self.topic.split("/")
        if (
            len(segments) < 3
            or segments[0]
            or not all(s.strip() == s and s for s in segments[1:])
            or self.topic.startswith(EVIDENCE_PREFIX + "/")
        ):
            raise RecordedStreamsError(f"a device stream is /<device>/<stream>: {self.topic!r}")

    @property
    def device(self) -> str:
        return self.topic.split("/")[1]

    @property
    def evidence_topic(self) -> str:
        """Where the one episode format keeps this stream's readings, byte for byte."""
        return EVIDENCE_PREFIX + self.topic


@dataclass(frozen=True, slots=True)
class RecordedPose:
    """``stream``'s poses are world-from-``link``, the description link the tracked device
    coincides with; the one format writes them as ``site``'s pose."""

    site: str
    stream: DeviceStream
    link: str

    def __post_init__(self) -> None:
        if not (self.site.strip() and self.link.strip()) or "/" in self.site:
            raise RecordedStreamsError("a recorded pose names a plain site and its link")


@dataclass(frozen=True, slots=True)
class RecordedCoordinate:
    """``stream`` measures ``component``'s one coordinate as ``measurand``."""

    component: str
    stream: DeviceStream
    measurand: Measurand

    def __post_init__(self) -> None:
        if not self.component.strip():
            raise RecordedStreamsError("a recorded coordinate names its component")


@dataclass(frozen=True, slots=True)
class RecordedCamera:
    """``name`` is what a person reads for the camera whose pictures are recorded in optical
    ``frame``, on ``video`` with its calibration on ``calibration``."""

    name: str
    frame: str
    video: DeviceStream
    calibration: DeviceStream

    def __post_init__(self) -> None:
        if not (self.frame.strip() and self.name.strip()) or "/" in self.name:
            raise RecordedStreamsError("a recorded camera names its frame and a plain name")


@dataclass(frozen=True, slots=True)
class RecordedStreams:
    """One body's capture bindings; a body no rig records declares none."""

    poses: tuple[RecordedPose, ...] = ()
    coordinates: tuple[RecordedCoordinate, ...] = ()
    cameras: tuple[RecordedCamera, ...] = ()

    def __post_init__(self) -> None:
        streams = self.streams
        if len(set(streams)) != len(streams):
            raise RecordedStreamsError("each device stream is bound once")
        for label, values in (
            ("site", [pose.site for pose in self.poses]),
            ("component", [item.component for item in self.coordinates]),
            ("camera name", [camera.name for camera in self.cameras]),
            ("camera frame", [camera.frame for camera in self.cameras]),
        ):
            if len(set(values)) != len(values):
                raise RecordedStreamsError(f"each {label} is bound once")

    @property
    def streams(self) -> tuple[DeviceStream, ...]:
        """Every device stream a binding names, in declaration order."""
        return (
            *(pose.stream for pose in self.poses),
            *(item.stream for item in self.coordinates),
            *(s for camera in self.cameras for s in (camera.video, camera.calibration)),
        )

    def to_dict(self) -> JsonObject:
        return {
            "poses": [
                {"site": p.site, "stream": p.stream.topic, "link": p.link} for p in self.poses
            ],
            "coordinates": [
                {"component": c.component, "stream": c.stream.topic, "measurand": c.measurand.value}
                for c in self.coordinates
            ],
            "cameras": [
                {
                    "name": c.name,
                    "frame": c.frame,
                    "video": c.video.topic,
                    "calibration": c.calibration.topic,
                }
                for c in self.cameras
            ],
        }

    @property
    def digest(self) -> str:
        """The bindings' content identity: what an episode was bound with, beside its body."""
        return content_digest(self.to_dict())

    def resolve(self, embodiment: Embodiment) -> None:
        """Raise `RecordedStreamsError` unless every binding names what ``embodiment`` has: a
        pose's link is a description link and agrees with any site or operator mount of the
        same name; a coordinate's component owns exactly one coordinate and its measurand maps
        to it; a camera's name is not another frame's declared camera."""
        from .instance import description_links
        from .parts import GripperSpec

        name = str(embodiment.name)
        links = description_links(embodiment.urdf_bytes)
        sites = {site.name: site.link for site in embodiment.sites}
        mounts = {m.site.value: m.attachment_frame for m in embodiment.operator_mounts}
        for pose in self.poses:
            if pose.link not in links:
                raise RecordedStreamsError(f"{name}: {pose.site} binds absent link {pose.link!r}")
            for declared in (sites.get(pose.site), mounts.get(pose.site)):
                if declared not in (None, pose.link):
                    raise RecordedStreamsError(
                        f"{name}: {pose.site} is {declared!r} in the body, bound to {pose.link!r}"
                    )
        owned: dict[str, int] = {}
        for coordinate in embodiment.state.coordinates:
            owned[str(coordinate.instance)] = owned.get(str(coordinate.instance), 0) + 1
        parts = {str(c.component_id): c.part for c in embodiment.components}
        for item in self.coordinates:
            if owned.get(item.component) != 1:
                raise RecordedStreamsError(
                    f"{name}: {item.component} owns no single coordinate to bind"
                )
            part = parts[item.component]
            if item.measurand is Measurand.JAW_APERTURE_M and not (
                isinstance(part, GripperSpec) and part.gap_curve is not None
            ):
                raise RecordedStreamsError(
                    f"{name}: {item.component} has no gap curve to map its aperture"
                )
        declared_cameras = {camera.name: camera.frame for camera in embodiment.cameras}
        for camera in self.cameras:
            if declared_cameras.get(camera.name, camera.frame) != camera.frame:
                raise RecordedStreamsError(
                    f"{name}: camera {camera.name!r} is a declared camera of another frame"
                )
