"""What one built robot streams: its declared cameras as tracks, and its microphones.

A station publishes exactly the cameras its body declares, through the optics its instance
resolved (:func:`sx_embodiments.instance.resolve`), so no track, raster or lens is spelled in
the station. :func:`media_document` renders that as ``sx.media-plan/v3``, which
``sx_embodiment_drivers::MediaPlan`` parses for one instance and refuses for any other.

A :class:`MediaRig` is the body's media wiring that the design itself does not carry: which
two declared cameras are one stereo pair (published as one side-by-side track so the two eyes
can never desynchronise) and which microphones are fitted. A pair is two cameras of one
exposure: the same raster, rate and modality. Every other camera is its own track.

Each view carries where its camera sits: the description link it is mounted on, the joint
chain from the description's root down to it, and, when only fixed joints lie on that chain, the
link's pose in the root frame. A camera that rides on a moving joint names the joints whose
positions place it instead, and its pose at capture is the chain evaluated at the joint state of
that instant: ``sx_embodiment_drivers::kinematics`` composes the rendered chain exactly as
:meth:`~sx_embodiments.kinematics.Kinematics.pose` does, so the station never reads a URDF.

Every declared inertial sensor (:class:`~sx_embodiments.parts.InertialSpec`) is listed under
``inertial`` with the same placement, its sensing axes at ``mount_from_sensor`` in its mount link,
and the unit the instance placed in its component slot (the device serial a driver opens it by).
Its samples are on those axes in REP-103 units; nothing in the plan names a vendor protocol.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Final

from sx_contracts.identity import JsonValue

from .embodiment import Embodiment
from .errors import EmbodimentSchemaError
from .instance import InstanceError, ResolvedCamera, ResolvedInstance, Slot, SlotKind
from .kinematics import Joint, JointType, Kinematics, KinematicsError, Transform
from .parts import InertialSpec

MEDIA_SCHEMA: Final = "sx.media-plan/v3"
#: A track or microphone name: it names a published track and the journal file beside it.
TRACK_NAME: Final = re.compile(r"[a-z0-9][a-z0-9_-]{0,62}")
#: The rates a published microphone track may run at: what WebRTC's audio path accepts.
MICROPHONE_RATES_HZ: Final = frozenset({16_000, 24_000, 32_000, 48_000})


class MediaRigError(EmbodimentSchemaError):
    """A media rig names cameras or microphones its body cannot stream."""


def _require_track_name(name: str) -> None:
    if TRACK_NAME.fullmatch(name) is None:
        raise MediaRigError(f"{name!r} is not a track name ({TRACK_NAME.pattern})")


@dataclass(frozen=True, slots=True)
class StereoPair:
    """Two declared cameras of one exposure, published as one ``left | right`` track."""

    track: str
    left: str
    right: str

    def __post_init__(self) -> None:
        _require_track_name(self.track)


@dataclass(frozen=True, slots=True)
class MicrophoneSpec:
    """One fitted microphone and the link it is mounted on."""

    name: str
    model: str
    sample_rate_hz: int
    channels: int
    mount_frame: str

    def __post_init__(self) -> None:
        _require_track_name(self.name)
        if not self.model.strip() or not self.mount_frame.strip():
            raise MediaRigError("a microphone names itself, its model and its mount link")
        if self.sample_rate_hz not in MICROPHONE_RATES_HZ:
            raise MediaRigError(
                f"{self.name}: {self.sample_rate_hz} Hz is not one of {sorted(MICROPHONE_RATES_HZ)}"
            )
        if self.channels not in (1, 2):
            raise MediaRigError(f"{self.name}: a microphone track is mono or stereo")


@dataclass(frozen=True, slots=True)
class MediaRig:
    stereo: tuple[StereoPair, ...] = ()
    microphones: tuple[MicrophoneSpec, ...] = ()


#: A body with no stereo pair and no microphone: every declared camera its own track.
NO_RIG: Final = MediaRig()


@dataclass(frozen=True, slots=True)
class ChainStep:
    """One joint on the chain from the description's root down to a mount link."""

    joint: str
    type: JointType
    #: ``parent_from_child`` at zero, row-major 4x4 in metres.
    origin: Transform
    axis: tuple[float, float, float]
    #: For a mimic joint: the independent joint it follows, and its multiplier and offset
    #: (a mimic of a mimic is folded into one).
    follows: tuple[str, float, float] | None


@dataclass(frozen=True, slots=True)
class MountPose:
    """Where a link sits relative to the description's root."""

    root_frame: str
    #: The link's pose in ``root_frame`` as (translation m, rotation quaternion xyzw), when
    #: only fixed joints lie between them.
    pose: tuple[tuple[float, float, float], tuple[float, float, float, float]] | None
    #: The independent joints whose positions place the link, in chain order, root first.
    moves_with: tuple[str, ...]
    #: Every joint from the root down to the link, root first.
    chain: tuple[ChainStep, ...]


def _follows(kinematics: Kinematics, joint: Joint) -> tuple[str, float, float] | None:
    if joint.mimic is None:
        return None
    multiplier, offset, leader = 1.0, 0.0, joint
    while leader.mimic is not None:
        # position = m * leader + o, folded outward: m1 * (m2 * x + o2) + o1.
        offset += multiplier * leader.mimic.offset
        multiplier *= leader.mimic.multiplier
        leader = kinematics.joints[leader.mimic.leader]
    return leader.name, multiplier, offset


def mount_pose(urdf: bytes, link: str) -> MountPose:
    """The chain from the description's root to ``link``, and its fixed pose when rigid."""
    try:
        kinematics = Kinematics.of(urdf)
        root = link
        while (joint := kinematics.parent_joint.get(root)) is not None:
            root = kinematics.joints[joint].parent
        _, down = kinematics.path(root, link)
    except KinematicsError as error:
        raise MediaRigError(str(error)) from error
    chain: list[ChainStep] = []
    moves_with: list[str] = []
    for joint in down:
        if joint.type in (JointType.FLOATING, JointType.PLANAR):
            raise MediaRigError(f"{link!r} rides a {joint.type} joint; no position places it")
        follows = _follows(kinematics, joint)
        chain.append(ChainStep(joint.name, joint.type, joint.origin, joint.axis, follows))
        if joint.type is JointType.FIXED:
            continue
        needed = joint.name if follows is None else follows[0]
        if needed not in moves_with:
            moves_with.append(needed)
    if moves_with:
        return MountPose(root, None, tuple(moves_with), tuple(chain))
    return MountPose(
        root, _translation_rotation(kinematics.pose(root, link)), (), tuple(chain)
    )


def mount_pose_at(
    urdf: bytes, link: str, positions: Mapping[str, float]
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Where ``link`` sits in the description's root frame at ``positions``: the pose a
    station's evaluation of the rendered chain must reproduce."""
    try:
        kinematics = Kinematics.of(urdf)
        root = link
        while (joint := kinematics.parent_joint.get(root)) is not None:
            root = kinematics.joints[joint].parent
        return _translation_rotation(kinematics.pose(root, link, positions))
    except KinematicsError as error:
        raise MediaRigError(str(error)) from error


def _translation_rotation(
    pose: Transform,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """The translation and the unit quaternion (x, y, z, w) of a row-major rigid transform."""

    def m(row: int, column: int) -> float:
        return pose[row * 4 + column]

    trace = m(0, 0) + m(1, 1) + m(2, 2)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = (
            0.25 * s,
            (m(2, 1) - m(1, 2)) / s,
            (m(0, 2) - m(2, 0)) / s,
            (m(1, 0) - m(0, 1)) / s,
        )
    elif m(0, 0) > m(1, 1) and m(0, 0) > m(2, 2):
        s = math.sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2.0
        w, x, y, z = (
            (m(2, 1) - m(1, 2)) / s,
            0.25 * s,
            (m(0, 1) + m(1, 0)) / s,
            (m(0, 2) + m(2, 0)) / s,
        )
    elif m(1, 1) > m(2, 2):
        s = math.sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2.0
        w, x, y, z = (
            (m(0, 2) - m(2, 0)) / s,
            (m(0, 1) + m(1, 0)) / s,
            0.25 * s,
            (m(1, 2) + m(2, 1)) / s,
        )
    else:
        s = math.sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2.0
        w, x, y, z = (
            (m(1, 0) - m(0, 1)) / s,
            (m(0, 2) + m(2, 0)) / s,
            (m(1, 2) + m(2, 1)) / s,
            0.25 * s,
        )
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    return (m(0, 3), m(1, 3), m(2, 3)), (x, y, z, w)


def _view(embodiment: Embodiment, camera: ResolvedCamera) -> dict[str, JsonValue]:
    binding = next(binding for binding in embodiment.cameras if binding.name == camera.name)
    optics = camera.nominal
    measured = camera.calibration
    return {
        "camera": camera.name,
        "model": str(binding.camera.model),
        "modality": binding.camera.modality.value,
        "width": optics.width,
        "height": optics.height,
        "fps": binding.camera.fps,
        "image_from_camera": list(
            measured.image_from_camera if measured is not None else optics.image_from_camera
        ),
        "distortion_model": (
            measured.distortion_model if measured is not None else optics.distortion_model
        ),
        "distortion_coefficients": list(
            measured.distortion_coefficients
            if measured is not None
            else optics.distortion_coefficients
        ),
        "unit": None if camera.unit is None else str(camera.unit),
        "calibration": None if camera.measured is None else str(camera.measured),
    } | _placement(embodiment, binding.frame)


def _pose(
    pose: tuple[tuple[float, float, float], tuple[float, float, float, float]],
) -> dict[str, JsonValue]:
    translation, rotation = pose
    return {"translation_m": list(translation), "rotation_xyzw": list(rotation)}


def _placement(embodiment: Embodiment, frame: str) -> dict[str, JsonValue]:
    """Where a sensor's mount link sits: the chain to it, and its pose when rigid."""
    mount = mount_pose(embodiment.urdf_bytes, frame)
    return {
        "mount_frame": frame,
        "root_frame": mount.root_frame,
        "root_from_mount": None if mount.pose is None else _pose(mount.pose),
        "moves_with": list(mount.moves_with),
        "chain": [_step(step) for step in mount.chain],
    }


def _inertial(
    embodiment: Embodiment, resolved: ResolvedInstance, names: set[str]
) -> list[JsonValue]:
    units = {entry.slot: entry.unit for entry in resolved.instance.units}
    sensors: list[JsonValue] = []
    for component in embodiment.components:
        part = component.part
        if not isinstance(part, InertialSpec):
            continue
        _require_track_name(component.instance)
        if component.instance in names:
            raise MediaRigError(f"{component.instance!r} names a track already")
        names.add(component.instance)
        unit = units.get(Slot(SlotKind.COMPONENT, component.instance))
        sensors.append(
            {
                "sensor": component.instance,
                "model": str(part.model),
                "unit": None if unit is None else str(unit),
                "rate_hz": part.rate_hz,
                "measures": [quantity.value for quantity in part.measures],
                "mount_from_sensor": _pose(part.mount_from_sensor),
            }
            | _placement(embodiment, component.mount.frame)
        )
    return sensors


def _step(step: ChainStep) -> dict[str, JsonValue]:
    return {
        "joint": step.joint,
        "type": step.type.value,
        "origin": list(step.origin),
        "axis": list(step.axis),
        "follows": (
            None
            if step.follows is None
            else {"joint": step.follows[0], "multiplier": step.follows[1], "offset": step.follows[2]}
        ),
    }


def media_document(
    embodiment: Embodiment, resolved: ResolvedInstance, rig: MediaRig = NO_RIG
) -> dict[str, JsonValue]:
    """What a station publishes for one built robot: one track per declared camera, a stereo
    pair's two cameras as one track, and every fitted microphone."""
    if resolved.instance.embodiment != embodiment.id:
        raise InstanceError("the instance is of a different embodiment")
    cameras = {camera.name: camera for camera in resolved.cameras}
    bindings = {binding.name: binding for binding in embodiment.cameras}
    paired: set[str] = set()
    names: set[str] = set()
    tracks: list[JsonValue] = []
    for pair in rig.stereo:
        for name in (pair.left, pair.right):
            if name not in cameras:
                raise MediaRigError(f"stereo track {pair.track!r}: the body declares no {name!r}")
            if name in paired:
                raise MediaRigError(f"{name!r} is in two stereo pairs")
            paired.add(name)
        left, right = bindings[pair.left].camera, bindings[pair.right].camera
        left_optics, right_optics = cameras[pair.left].nominal, cameras[pair.right].nominal
        if (
            (left_optics.width, left_optics.height) != (right_optics.width, right_optics.height)
            or left.fps != right.fps
            or left.modality is not right.modality
        ):
            raise MediaRigError(
                f"stereo track {pair.track!r}: a pair is one exposure of one raster, rate and "
                "modality"
            )
        tracks.append(
            {
                "track": pair.track,
                "fps": left.fps,
                "views": [
                    _view(embodiment, cameras[pair.left]),
                    _view(embodiment, cameras[pair.right]),
                ],
            }
        )
        names.add(pair.track)
    for camera in resolved.cameras:
        if camera.name in paired:
            continue
        _require_track_name(camera.name)
        tracks.append(
            {
                "track": camera.name,
                "fps": bindings[camera.name].camera.fps,
                "views": [_view(embodiment, camera)],
            }
        )
        names.add(camera.name)
    if len(names) != len(tracks):
        raise MediaRigError("two tracks share a name")
    description = ET.fromstring(embodiment.urdf_bytes)
    links = {element.attrib["name"] for element in description.iter("link")}
    microphones: list[JsonValue] = []
    for microphone in rig.microphones:
        if microphone.mount_frame not in links:
            raise MediaRigError(
                f"{microphone.name}: {microphone.mount_frame!r} is not a link of the body"
            )
        if microphone.name in names:
            raise MediaRigError(f"{microphone.name!r} names a track already")
        names.add(microphone.name)
        microphones.append(
            {
                "microphone": microphone.name,
                "model": microphone.model,
                "sample_rate_hz": microphone.sample_rate_hz,
                "channels": microphone.channels,
                "mount_frame": microphone.mount_frame,
            }
        )
    inertial = _inertial(embodiment, resolved, names)
    return {
        "schema": MEDIA_SCHEMA,
        "embodiment": str(embodiment.id),
        "instance": str(resolved.instance.id),
        "tracks": tracks,
        "microphones": microphones,
        "inertial": inertial,
    }
