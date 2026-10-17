"""Forward kinematics over a body's description: where one link is in another's frame.

The one URDF chain reader. A calibration solve, a delivery adapter and a test all ask the same
question, the transform ``parent_from_child`` between two links at given joint positions, and
answer it here from the description bytes, never from a second parser.

Transforms are row-major 4x4 tuples in metres, the form :class:`~sx_embodiments.calibration.
RigidCalibration` stores. A path may climb from ``parent`` to a common ancestor and descend to
``child``; every movable joint on it needs a position, a mimic joint follows its leader, and a
rigid path (fixed joints only) needs none.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from typing import Final

from .errors import EmbodimentError

Transform = tuple[float, ...]

IDENTITY: Final[Transform] = (
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
)  # fmt: skip


class KinematicsError(EmbodimentError):
    """A description cannot answer a kinematic question as asked."""


class JointType(StrEnum):
    FIXED = "fixed"
    REVOLUTE = "revolute"
    CONTINUOUS = "continuous"
    PRISMATIC = "prismatic"
    FLOATING = "floating"  # a free six-dof placement: no position answers it
    PLANAR = "planar"


_ONE_DOF: Final = frozenset({JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC})


@dataclass(frozen=True, slots=True)
class Mimic:
    leader: str
    multiplier: float
    offset: float


@dataclass(frozen=True, slots=True)
class Joint:
    name: str
    type: JointType
    parent: str
    child: str
    origin: Transform
    axis: tuple[float, float, float]
    mimic: Mimic | None


def compose(*transforms: Transform) -> Transform:
    """The product of row-major 4x4 transforms, left to right."""
    result = IDENTITY
    for transform in transforms:
        result = tuple(
            sum(result[4 * row + k] * transform[4 * k + column] for k in range(4))
            for row in range(4)
            for column in range(4)
        )
    return result


def invert(transform: Transform) -> Transform:
    """The inverse of a rigid transform."""
    r = (transform[0:3], transform[4:7], transform[8:11])
    t = (transform[3], transform[7], transform[11])
    rt = tuple(tuple(r[column][row] for column in range(3)) for row in range(3))
    translation = tuple(-sum(rt[row][k] * t[k] for k in range(3)) for row in range(3))
    return (
        *rt[0], translation[0],
        *rt[1], translation[1],
        *rt[2], translation[2],
        0.0, 0.0, 0.0, 1.0,
    )  # fmt: skip


def from_xyz_rpy(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> Transform:
    """A URDF origin: translation, then fixed-axis roll, pitch, yaw."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0],
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1],
        -sp, cp * sr, cp * cr, xyz[2],
        0.0, 0.0, 0.0, 1.0,
    )  # fmt: skip


def _about(axis: tuple[float, float, float], angle: float) -> Transform:
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return (
        t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0,
        t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0,
        t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0,
        0.0, 0.0, 0.0, 1.0,
    )  # fmt: skip


def _along(axis: tuple[float, float, float], distance: float) -> Transform:
    x, y, z = axis
    return (
        1.0, 0.0, 0.0, x * distance,
        0.0, 1.0, 0.0, y * distance,
        0.0, 0.0, 1.0, z * distance,
        0.0, 0.0, 0.0, 1.0,
    )  # fmt: skip


def _triple(element: ET.Element | None, key: str, default: str) -> tuple[float, float, float]:
    text = default if element is None else element.get(key, default)
    try:
        values = tuple(float(value) for value in text.split())
    except ValueError as error:
        raise KinematicsError(f"{key}={text!r} is not three numbers") from error
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise KinematicsError(f"{key}={text!r} is not three finite numbers")
    return (values[0], values[1], values[2])


@dataclass(frozen=True, slots=True)
class Kinematics:
    """A description's joint tree, parsed once."""

    links: frozenset[str]
    joints: Mapping[str, Joint]
    parent_joint: Mapping[str, str]  # child link -> the joint above it

    @classmethod
    def of(cls, urdf: bytes) -> Kinematics:
        try:
            root = ET.fromstring(urdf)  # nosec B314 - registry-owned description bytes
        except ET.ParseError as error:
            raise KinematicsError(f"the description is not XML: {error}") from error
        links = frozenset(
            name for link in root.findall("link") if (name := link.get("name")) is not None
        )
        joints: dict[str, Joint] = {}
        parent_of: dict[str, str] = {}
        for element in root.findall("joint"):
            name = element.get("name")
            parent = element.find("parent")
            child = element.find("child")
            if name is None or parent is None or child is None:
                raise KinematicsError("a joint names itself, its parent and its child")
            parent_link, child_link = parent.get("link"), child.get("link")
            if parent_link not in links or child_link not in links:
                raise KinematicsError(f"joint {name!r} joins links the description lacks")
            if child_link in parent_of:
                raise KinematicsError(f"link {child_link!r} has two parent joints")
            parent_of[child_link] = name
            try:
                kind = JointType(element.get("type", ""))
            except ValueError as error:
                raise KinematicsError(
                    f"joint {name!r} has type {element.get('type')!r}, which URDF does not define"
                ) from error
            origin = element.find("origin")
            axis = _triple(element.find("axis"), "xyz", "1 0 0")
            norm = math.sqrt(sum(value * value for value in axis))
            if kind in _ONE_DOF and norm < 1e-12:
                raise KinematicsError(f"joint {name!r} moves about a zero axis")
            mimic_element = element.find("mimic")
            mimic = None
            if mimic_element is not None:
                leader = mimic_element.get("joint")
                if leader is None:
                    raise KinematicsError(f"joint {name!r} mimics no joint")
                mimic = Mimic(
                    leader,
                    float(mimic_element.get("multiplier", "1")),
                    float(mimic_element.get("offset", "0")),
                )
            joints[name] = Joint(
                name=name,
                type=kind,
                parent=parent_link,
                child=child_link,
                origin=from_xyz_rpy(
                    _triple(origin, "xyz", "0 0 0"), _triple(origin, "rpy", "0 0 0")
                ),
                axis=(
                    (axis[0] / norm, axis[1] / norm, axis[2] / norm)
                    if norm >= 1e-12
                    else (1.0, 0.0, 0.0)
                ),
                mimic=mimic,
            )
        for joint in joints.values():
            if joint.mimic is not None and joint.mimic.leader not in joints:
                raise KinematicsError(f"joint {joint.name!r} mimics an absent joint")
        return cls(links, joints, parent_of)

    @property
    def independent(self) -> tuple[str, ...]:
        """Every movable joint whose position is its own (not a mimic), in document order."""
        return tuple(
            joint.name
            for joint in self.joints.values()
            if joint.type is not JointType.FIXED and joint.mimic is None
        )

    def _parent_joint(self, link: str) -> Joint | None:
        name = self.parent_joint.get(link)
        return self.joints[name] if name is not None else None

    def _ancestry(self, link: str) -> list[Joint]:
        if link not in self.links:
            raise KinematicsError(f"{link!r} is not a link of the description")
        chain: list[Joint] = []
        while (joint := self._parent_joint(link)) is not None:
            chain.append(joint)
            link = joint.parent
        return chain

    def path(self, parent: str, child: str) -> tuple[tuple[Joint, ...], tuple[Joint, ...]]:
        """The joints climbed from ``parent`` to the common ancestor, then down to ``child``."""
        up, down = self._ancestry(parent), self._ancestry(child)
        while up and down and up[-1] is down[-1]:
            up.pop()
            down.pop()
        if (up[-1].parent if up else parent) != (down[-1].parent if down else child):
            raise KinematicsError(f"{parent!r} and {child!r} are in different trees")
        return tuple(up), tuple(reversed(down))

    def is_rigid(self, parent: str, child: str) -> bool:
        """Whether only fixed joints lie between the two links."""
        up, down = self.path(parent, child)
        return all(joint.type is JointType.FIXED for joint in (*up, *down))

    def _position(self, joint: Joint, positions: Mapping[str, float]) -> float:
        if joint.mimic is not None:
            if joint.name in positions:
                raise KinematicsError(f"{joint.name!r} follows {joint.mimic.leader!r}; give that")
            leader = self.joints[joint.mimic.leader]
            return joint.mimic.multiplier * self._position(leader, positions) + joint.mimic.offset
        if joint.name not in positions:
            raise KinematicsError(f"no position for movable joint {joint.name!r}")
        value = positions[joint.name]
        if not math.isfinite(value):
            raise KinematicsError(f"{joint.name!r} has a non-finite position")
        return value

    def _local(self, joint: Joint, positions: Mapping[str, float]) -> Transform:
        """``parent_from_child`` across one joint."""
        match joint.type:
            case JointType.FIXED:
                return joint.origin
            case JointType.REVOLUTE | JointType.CONTINUOUS:
                return compose(joint.origin, _about(joint.axis, self._position(joint, positions)))
            case JointType.PRISMATIC:
                return compose(joint.origin, _along(joint.axis, self._position(joint, positions)))
            case JointType.FLOATING | JointType.PLANAR:
                raise KinematicsError(
                    f"{joint.name!r} is a {joint.type} joint; one position cannot place it"
                )

    def pose(
        self, parent: str, child: str, positions: Mapping[str, float] | None = None
    ) -> Transform:
        """``parent_from_child`` at ``positions``; a rigid path needs none."""
        given = positions or {}
        unknown = set(given) - set(self.joints)
        if unknown:
            raise KinematicsError(f"positions name absent joints: {', '.join(sorted(unknown))}")
        up, down = self.path(parent, child)
        climbed = compose(*(self._local(joint, given) for joint in reversed(up)))
        descended = compose(*(self._local(joint, given) for joint in down))
        return compose(invert(climbed), descended)

    def poses(self, frame: str, positions: Mapping[str, float]) -> dict[str, Transform]:
        """Every link's pose in ``frame`` at ``positions``."""
        return {link: self.pose(frame, link, positions) for link in sorted(self.links)}
