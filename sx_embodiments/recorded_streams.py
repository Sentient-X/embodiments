"""What a capture rig's recording names each of a body's tracked devices and measured coordinates.

A pod records a handheld body by its devices, not by its description: a tracked controller's
pose is a ``foxglove.PoseInFrame`` whose ``frame_id`` is the device's own name, a jaw's opening
a ``foxglove.MagneticEncoderMeasurement`` whose ``frame_id`` is the encoder's. A reader that
draws such a recording with the body must know which recorded frame is which operator site and
which component's coordinate, and those names are facts about the rig, declared once beside the
body here, so no reader matches topics or name prefixes.

* :class:`RecordedPose`: the recorded frame that places one operator site's device. The site's
  operator mount (``Embodiment.operator_mounts``) says which description frame it is.
* :class:`RecordedCoordinate`: the recorded frame that measures one component's single
  coordinate. A jaw's encoder measures its gap in metres, which the component's gap curve maps
  to the joint position.

Like viewing facts, these are outside an embodiment's content identity: declaring or correcting
one moves no id.
"""

from __future__ import annotations

from dataclasses import dataclass

from .errors import EmbodimentError


class RecordedStreamsError(EmbodimentError):
    """A recorded-stream fact is malformed."""


@dataclass(frozen=True, slots=True)
class RecordedPose:
    """``frame`` is the ``frame_id`` a recording gives the device worn or held at ``site``."""

    site: str
    frame: str

    def __post_init__(self) -> None:
        if not (self.site.strip() and self.frame.strip()):
            raise RecordedStreamsError("a recorded pose names its site and its frame")


@dataclass(frozen=True, slots=True)
class RecordedCoordinate:
    """``frame`` is the ``frame_id`` a recording measures ``component``'s one coordinate under."""

    component: str
    frame: str

    def __post_init__(self) -> None:
        if not (self.component.strip() and self.frame.strip()):
            raise RecordedStreamsError("a recorded coordinate names its component and its frame")


@dataclass(frozen=True, slots=True)
class RecordedStreams:
    """One body's recorded-stream facts; a body a rig does not record declares none."""

    poses: tuple[RecordedPose, ...] = ()
    coordinates: tuple[RecordedCoordinate, ...] = ()

    def __post_init__(self) -> None:
        frames = [item.frame for item in (*self.poses, *self.coordinates)]
        if len(set(frames)) != len(frames):
            raise RecordedStreamsError("each recorded frame names one device")
        sites = [pose.site for pose in self.poses]
        if len(set(sites)) != len(sites):
            raise RecordedStreamsError("one recorded pose per site")
