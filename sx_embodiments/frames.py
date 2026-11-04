"""Every frame of a body: its axes and its roles, read from what the body declares.

A body's frames are its description's links. Each one carries :class:`~sx_geometry.frames.Axes`
on the facing of the thing it is attached to (``docs/plans/frames-2026-11.md``):

* The **root link**'s axes are on the body's own facing: its registry module declares the
  body's front (:attr:`~sx_embodiments.compose.EmbodimentDefinition.front`) along the root's
  x or y, and the root's z is up (:func:`root_axes`; ``FLU`` when the front is +x, as ROS
  REP-103 names a base). ``tests/test_frames.py`` checks every declaration against the body's
  geometry: a body's two tools lie mirrored across its centre plane, so they differ along its
  left.
* Every other link is **FLU** on its own facing (x forward, y left, z up), as URDF and REP-103
  define a link.
* A link the description declares a camera's optical frame (``data-frame-convention=
  "camera_optical"``, :attr:`~sx_embodiments.assembly.Frame.camera_optical`) is **RDF** (x
  image-right, y image-down, z along the view). Its rotation on its camera's body frame is
  computed from the two axes (:data:`~sx_embodiments.assembly.OPTICAL_RPY`), never typed in.

Each frame's **roles** (:class:`FrameRole`) come from the component graph, so no consumer reads
a role from a link's name (``*_gripper_tcp``, ``ZED_camera``):

* :attr:`FrameRole.TOOL`: a tool's frame, the one :mod:`~sx_embodiments.tool_frames` states its
  grip in: a gripper's mount frame, or the last link of an arm that carries no gripper.
* :attr:`FrameRole.HEAD`: the last link of a chain serving the head limb: where the head
  ends, with that link's own axes. The head looks through its ``VIEW`` frames (along their z),
  not along ``HEAD``'s x (the static upper body's ``camera_link`` x points up at rest).
* :attr:`FrameRole.VIEW`: a camera's frame (always optical).
* :attr:`FrameRole.SENSOR`: any other sensor's frame (an inertial unit, a force-torque cell).

Where a body meets the world or a person is already declared, by its base mount and operator
mounts, so it is not a role here.

:func:`sides` says which side of the body (``LEFT`` or ``RIGHT`` of its facing) a frame is on,
from the operator mount that carries it or, for two frames, their places along the body's left
at rest. Needs the ``render`` extra (NumPy, through ``sx_geometry``).
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from typing import Final

import numpy as np
from sx_contracts.identity import JsonValue
from sx_geometry.frames import FLU, RDF, Axes, Direction, Frame, basis

from .compose import Component, ComponentRole, OperatorSite
from .compose import Direction as PlaneDirection
from .embodiment import Embodiment
from .errors import EmbodimentError
from .kinematics import Kinematics, KinematicsError
from .known import front_of
from .parts import CameraSpec, ChainSpec, GripperSpec, Limb

#: A description link's axes on its own facing (URDF, REP-103).
LINK_AXES: Final = FLU
#: A link the description declares a camera's optical frame.
OPTICAL_AXES: Final = RDF
#: The description's mark on a camera's optical frame.
OPTICAL_MARK: Final = ("data-frame-convention", "camera_optical")
#: Each declared front as a unit vector in the root frame (its z is up).
_PLANE: Final = {
    PlaneDirection.POS_X: (1.0, 0.0, 0.0),
    PlaneDirection.NEG_X: (-1.0, 0.0, 0.0),
    PlaneDirection.POS_Y: (0.0, 1.0, 0.0),
    PlaneDirection.NEG_Y: (0.0, -1.0, 0.0),
}
#: Two frames closer than this along the body's left are on neither side, metres.
_CENTRE_M: Final = 1e-3

_OPERATOR_SIDES: Final = {
    OperatorSite.LEFT_HAND: Direction.LEFT,
    OperatorSite.RIGHT_HAND: Direction.RIGHT,
}


class FrameDeclarationError(EmbodimentError):
    """A body that does not declare what its frames need."""


class FrameRole(StrEnum):
    """What a body's frame is for, as its component graph declares it."""

    TOOL = "tool"
    HEAD = "head"
    VIEW = "view"
    SENSOR = "sensor"


@dataclass(frozen=True, slots=True)
class BodyFrame:
    """One link of a body's description: its frame (name and axes) and its roles."""

    frame: Frame
    roles: frozenset[FrameRole]

    @property
    def name(self) -> str:
        return self.frame.name

    def to_json(self) -> JsonValue:
        axes: list[JsonValue] = [*self.frame.axes.to_document()]
        roles: list[JsonValue] = [*sorted(role.value for role in self.roles)]
        return {"name": self.frame.name, "axes": axes, "roles": roles}


def _description(embodiment: Embodiment) -> ET.Element:
    return ET.fromstring(embodiment.urdf_bytes)  # nosec B314 - registry-owned description bytes


def link_axes(link: ET.Element) -> Axes:
    """A description link's axes: optical when the description declares it, else a link's."""
    key, value = OPTICAL_MARK
    return OPTICAL_AXES if link.get(key) == value else LINK_AXES


def _last_link(
    embodiment: Embodiment, component: Component, part: ChainSpec, kinematics: Kinematics
) -> str:
    """The link a chain's last axis moves."""
    joints = zip(embodiment.state.coordinates, embodiment.joint_names, strict=True)
    resolved = {(str(c.instance), c.joint_name): name for c, name in joints}
    return kinematics.joints[resolved[(component.instance, part.layout.axes[-1].name)]].child


def tools(embodiment: Embodiment) -> tuple[tuple[Component, str], ...]:
    """Every tool of a body and the frame it is stated in, in component order: each gripper at
    its mount frame, and each arm carrying no gripper at its last link."""
    kinematics = Kinematics.of(embodiment.urdf_bytes)
    found: list[tuple[Component, str]] = []
    for component in embodiment.components:
        if component.role is not ComponentRole.BODY:
            continue
        part = component.part
        if isinstance(part, GripperSpec):
            found.append((component, component.mount.frame))
        elif isinstance(part, ChainSpec) and component.limb is Limb.ARM:
            carries_gripper = any(
                getattr(other.mount, "parent", None) == component.instance
                and other.role is ComponentRole.BODY
                and isinstance(other.part, GripperSpec)
                for other in embodiment.components
            )
            if not carries_gripper:
                found.append((component, _last_link(embodiment, component, part, kinematics)))
    return tuple(found)


def frame_roles(embodiment: Embodiment) -> Mapping[str, frozenset[FrameRole]]:
    """Each frame the component graph gives a role, and its roles."""
    kinematics = Kinematics.of(embodiment.urdf_bytes)
    roles: dict[str, set[FrameRole]] = {}

    def add(frame: str, role: FrameRole) -> None:
        roles.setdefault(frame, set()).add(role)

    for _, frame in tools(embodiment):
        add(frame, FrameRole.TOOL)
    for component in embodiment.components:
        part = component.part
        if component.role is ComponentRole.SENSOR:
            add(
                component.mount.frame,
                FrameRole.VIEW if isinstance(part, CameraSpec) else FrameRole.SENSOR,
            )
        elif isinstance(part, ChainSpec) and component.limb is Limb.HEAD:
            add(_last_link(embodiment, component, part, kinematics), FrameRole.HEAD)
    return {frame: frozenset(found) for frame, found in roles.items()}


def body_frames(embodiment: Embodiment) -> tuple[BodyFrame, ...]:
    """Every link of the body's description, in description order, with its axes and roles."""
    roles = frame_roles(embodiment)
    root, facing = root_link(embodiment), root_axes(embodiment)
    frames = tuple(
        BodyFrame(
            Frame(name, facing if name == root else link_axes(link)),
            roles.get(name, frozenset()),
        )
        for link in _description(embodiment).findall("link")
        if (name := link.get("name")) is not None
    )
    if missing := sorted(set(roles) - {frame.name for frame in frames}):
        raise KinematicsError(f"{embodiment.name}: roles name links it lacks: {missing}")
    return frames


def root_link(embodiment: Embodiment) -> str:
    """The description's root link, whose facing is the body's."""
    kinematics = Kinematics.of(embodiment.urdf_bytes)
    roots = sorted(kinematics.links - set(kinematics.parent_joint))
    if len(roots) != 1:
        raise KinematicsError(f"{embodiment.name}: the description has roots {roots}")
    return roots[0]


def root_axes(embodiment: Embodiment) -> Axes:
    """The root link's axes on the body's facing, from the front its registry module declares
    (:attr:`~sx_embodiments.compose.EmbodimentDefinition.front`) with the root's z up.

    Raises :class:`~sx_embodiments.errors.UnknownEmbodimentError` for a body no registry holds
    and :class:`FrameDeclarationError` for one that declares no front: a facing is never
    assumed.
    """
    front = front_of(str(embodiment.name))
    if front is None:
        raise FrameDeclarationError(f"{embodiment.name} declares no front")
    forward = np.asarray(_PLANE[front], dtype=np.float64)
    up = np.array([0.0, 0.0, 1.0])
    # Each root axis written in the body's forward-left-up coordinates names its direction.
    body_from_root = np.vstack((forward, np.cross(up, forward), up))
    by_vector = {tuple(direction.vector): direction for direction in Direction}
    x, y, z = (by_vector[tuple(column)] for column in body_from_root.T)
    return Axes(x, y, z)


def sides(embodiment: Embodiment, frames: Sequence[str]) -> dict[str, Direction]:
    """Which side of the body (``LEFT`` or ``RIGHT`` of its facing) each of ``frames`` is on.

    An operator mount whose subtree carries a frame says it (a handheld rig's root floats).
    Otherwise two frames are told apart where their branches of the description part (the two
    links just below the last link both hang from: two arms' bases), by those links' places
    along the body's left at rest: the one further left is ``LEFT``. Where the branches part
    decides, not where the frames are, so two arms crossed at rest keep their sides. A frame
    neither rule places has no side.
    """
    kinematics = Kinematics.of(embodiment.urdf_bytes)
    found: dict[str, Direction] = {}
    for frame in frames:
        for mount in embodiment.operator_mounts:
            direction = _OPERATOR_SIDES.get(mount.site)
            if direction is not None and (
                frame == mount.root_frame or mount.root_frame in _ancestors(kinematics, frame)
            ):
                found[frame] = direction
    rest_frames = [frame for frame in frames if frame not in found]
    if len(rest_frames) != 2:
        return found
    axes = root_axes(embodiment)
    left = basis(axes, FLU) @ Direction.LEFT.vector  # the body's left, in root coordinates
    root = root_link(embodiment)
    rest = dict.fromkeys(kinematics.independent, 0.0)
    branches = _branches(kinematics, rest_frames[0], rest_frames[1])
    if branches is None:
        return found
    try:
        along = [
            float(left @ np.asarray(pose[3:12:4], dtype=np.float64))
            for branch in branches
            for pose in (kinematics.pose(root, branch, rest),)
        ]
    except KinematicsError:  # a floating joint places it, and no operator mount does
        return found
    if abs(along[0] - along[1]) < _CENTRE_M:
        return found
    further = 0 if along[0] > along[1] else 1
    found[rest_frames[further]] = Direction.LEFT
    found[rest_frames[1 - further]] = Direction.RIGHT
    return found


def _lineage(kinematics: Kinematics, link: str) -> list[str]:
    """``link`` and every link above it, up to the root."""
    found = [link]
    while (joint := kinematics.parent_joint.get(link)) is not None:
        link = kinematics.joints[joint].parent
        found.append(link)
    return found


def _ancestors(kinematics: Kinematics, link: str) -> set[str]:
    return set(_lineage(kinematics, link)[1:])


def _branches(kinematics: Kinematics, a: str, b: str) -> tuple[str, str] | None:
    """The two links just below the last link ``a`` and ``b`` both hang from; ``None`` when
    one hangs from the other."""
    up_a, up_b = _lineage(kinematics, a)[::-1], _lineage(kinematics, b)[::-1]
    shared = 0
    while shared < min(len(up_a), len(up_b)) and up_a[shared] == up_b[shared]:
        shared += 1
    if shared in (len(up_a), len(up_b)):
        return None
    return up_a[shared], up_b[shared]
