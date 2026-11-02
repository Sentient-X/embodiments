"""Each tool a body carries, and how it grips, derived from its description.

A body's **tools** are its grippers (each :class:`~sx_embodiments.parts.GripperSpec` component)
and every arm that carries none (its flange). A tool's :class:`ToolFrame` names the description
frame it is stated in (the gripper's mount frame, or the arm's last link) and its grip: either
:class:`Jaws`, the line its jaws close on, the direction it approaches along and the point its
jaws meet, or :class:`NoJaws` with the reason a closing line does not exist. A closing line is
never guessed: a multi-finger hand, a bare flange, a device whose jaw no source describes, and
jaws the description draws no geometry for each say so.

One rule for every body, from the description's joints and collision geometry, in the tool's
frame:

1. The **jaws** are the links the gripper's joints (its actuated joints and their mimics) move.
   Each joint whose parent the gripper does not move starts one jaw piece (a linkage has
   several), and a piece's side is the sign of its geometry's displacement along the principal
   direction of every jaw vertex's displacement between the joints' two ends. Pieces on both
   sides are **parallel** jaws; pieces on one side are a **single moving** jaw against a fixed
   one.
2. The jaws are **open** at the end where they stand further apart (for a single moving jaw,
   further from the fixed jaw's geometry), and closed at the other.
3. Each side's **fingertip** is its vertex furthest from the mount with the jaws closed, and
   the link carrying it is that side's jaw. The **grasp point** is the gripper's declared
   ``grasp_centre_m``, else the midpoint of the fingertips.
4. The **closing line** is the jaws' relative velocity at the grasp point as the gripper opens
   from closed (a fixed jaw stands still): the finger joints' own motion, so a prismatic jaw
   closes along its axis and a revolute one across its hinge. Parallel jaws' line is signed so
   its largest component is positive; a single moving jaw's runs from the fixed jaw to the
   moving one.
5. The **approach** runs from the mount to the grasp point, made normal to the closing line.

``tests/test_tool_frames.py`` holds every body in the registry to this, and parallel jaws'
closing line to the line between their fingertips with the jaws open. Needs the ``render``
extra (NumPy), as the collision geometry does.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Final

import numpy as np
import numpy.typing as npt
from sx_contracts.identity import JsonValue

from .collision_proxies import (
    _collision_meshes,  # pyright: ignore[reportPrivateUsage]  # the one mesh source
)
from .compose import Component, ComponentRole
from .embodiment import Embodiment
from .kinematics import Kinematics
from .layout import Bounds, Unbounded
from .parts import ChainSpec, DeviceSpec, GraspKind, GripperSpec, Limb

Vec3 = tuple[float, float, float]
_Floats = npt.NDArray[np.float64]

#: A piece whose displacement along the closing direction is under this share of the largest
#: one moves across the jaws (a linkage's connector), not with either side.
_SIDE_SHARE: Final = 0.1
#: The step, as a share of the joints' range, the jaws' velocity is measured over.
_STEP: Final = 1e-4
#: How far back from a fingertip's furthest point its pad reaches, m.
_PAD_M: Final = 0.005


class JawKind(StrEnum):
    """How a gripper's jaws meet."""

    #: Two jaws move toward each other.
    PARALLEL = "parallel"
    #: One jaw moves against a fixed one.
    SINGLE_MOVING = "single_moving"


class NoJawsReason(StrEnum):
    """Why a tool has no closing line."""

    #: A hand of more than two fingers (``GraspKind.DEXTEROUS``): no one line.
    MULTI_FINGER_HAND = "multi_finger_hand"
    #: An arm the registry mounts no gripper on: a flange closes on nothing.
    BARE_FLANGE = "bare_flange"
    #: A device mounted on an arm whose jaw no source describes.
    DEVICE_UNDESCRIBED = "device_undescribed"
    #: The description draws no collision geometry on the moving jaws: no fingertip to place.
    JAWS_UNDESCRIBED = "jaws_undescribed"
    #: The gripper's joints move no geometry between their two ends.
    NO_JAW_MOTION = "no_jaw_motion"


@dataclass(frozen=True, slots=True)
class Jaws:
    """A tool whose jaws close on one line; every vector is in the tool's frame."""

    kind: JawKind
    #: Unit; normal to ``approach``.
    closing_axis: Vec3
    #: Unit; from where the jaws attach toward the grasp point.
    approach_axis: Vec3
    grasp_point_m: Vec3


@dataclass(frozen=True, slots=True)
class NoJaws:
    """A tool with no closing line, and why."""

    reason: NoJawsReason
    detail: str


@dataclass(frozen=True, slots=True)
class ToolFrame:
    """One tool of a body: the component it is, the frame it is stated in, and its grip."""

    component: str
    frame: str
    grip: Jaws | NoJaws

    def to_json(self) -> JsonValue:
        grip: dict[str, JsonValue]
        match self.grip:
            case Jaws(kind, closing, approach, point):
                grip = {
                    "jaws": kind.value,
                    "closing_axis": _rounded(closing),
                    "approach_axis": _rounded(approach),
                    "grasp_point_m": _rounded(point),
                }
            case NoJaws(reason, detail):
                grip = {"none": reason.value, "detail": detail}
        return {"component": self.component, "frame": self.frame, "grip": grip}


def tool_frames(embodiment: Embodiment) -> tuple[ToolFrame, ...]:
    """Every tool of ``embodiment``, in component order: each gripper, and each arm carrying
    none."""
    kinematics = Kinematics.of(embodiment.urdf_bytes)
    joints = _resolved_joints(embodiment)
    mounts = {component.instance: component.mount.frame for component in embodiment.components}
    meshes: dict[str, list[tuple[_Floats, _Floats]]] | None = None
    tools: list[ToolFrame] = []
    for component in embodiment.components:
        if component.role is not ComponentRole.BODY:
            continue
        part = component.part
        if isinstance(part, GripperSpec):
            if meshes is None:
                meshes = _collision_meshes(embodiment, embodiment.urdf_bytes)
            tools.append(_gripper(component, part, kinematics, joints, mounts, meshes))
        elif isinstance(part, ChainSpec) and component.limb is Limb.ARM:
            if (tool := _flange(embodiment, component, part, joints)) is not None:
                tools.append(tool)
    return tuple(tools)


def _flange(
    embodiment: Embodiment,
    arm: Component,
    part: ChainSpec,
    joints: dict[tuple[str, str], str],
) -> ToolFrame | None:
    """An arm's flange tool, or ``None`` when a gripper rides on the arm."""
    carried = [
        component
        for component in embodiment.components
        if getattr(component.mount, "parent", None) == arm.instance
        and component.role is ComponentRole.BODY
    ]
    if any(isinstance(component.part, GripperSpec) for component in carried):
        return None
    last = joints[(arm.instance, part.layout.axes[-1].name)]
    frame = Kinematics.of(embodiment.urdf_bytes).joints[last].child
    devices = [c.instance for c in carried if isinstance(c.part, DeviceSpec)]
    if devices:
        return ToolFrame(
            arm.instance,
            frame,
            NoJaws(
                NoJawsReason.DEVICE_UNDESCRIBED,
                f"{', '.join(devices)} rides on {arm.instance}; no source describes its jaw",
            ),
        )
    return ToolFrame(
        arm.instance,
        frame,
        NoJaws(NoJawsReason.BARE_FLANGE, f"the registry mounts no gripper on {arm.instance}"),
    )


def _subtree_joints(kinematics: Kinematics, frame: str, boundaries: set[str]) -> set[str]:
    """Every joint below ``frame`` short of another component's mount frame."""
    reachable = {frame}
    scoped: set[str] = set()
    changed = True
    while changed:
        changed = False
        for name, joint in kinematics.joints.items():
            if joint.parent in reachable and joint.parent not in boundaries and name not in scoped:
                scoped.add(name)
                reachable.add(joint.child)
                changed = True
    return scoped


@dataclass(frozen=True, slots=True)
class _Jaw:
    """One gripper's jaws as the rule reads them, in its mount frame."""

    kinematics: Kinematics
    frame: str
    meshes: dict[str, list[tuple[_Floats, _Floats]]]
    kind: JawKind
    #: Each side's links: the first moves along +``direction`` as the jaws open.
    sides: tuple[frozenset[str], ...]
    hand: frozenset[str]
    direction: _Floats
    opened: dict[str, float]
    closed: dict[str, float]

    def pose(self, link: str, positions: dict[str, float]) -> _Floats:
        return np.asarray(
            self.kinematics.pose(self.frame, link, positions), dtype=np.float64
        ).reshape(4, 4)

    def vertices(self, links: frozenset[str], positions: dict[str, float]) -> _Floats:
        points = [
            mesh @ transform[:3, :3].T + transform[:3, 3]
            for link in sorted(links)
            for transform in (self.pose(link, positions),)
            for mesh, _ in self.meshes.get(link, [])
        ]
        return np.vstack(points) if points else np.zeros((0, 3))

    def tip(self, links: frozenset[str], positions: dict[str, float]) -> tuple[str, _Floats]:
        """The jaw's fingertip: its vertex furthest from the mount, and the link carrying it."""
        best: tuple[str, _Floats, float] | None = None
        for link in sorted(links):
            points = self.vertices(frozenset((link,)), positions)
            if not points.size:
                continue
            far = int(np.argmax(np.linalg.norm(points, axis=1)))
            reach = float(np.linalg.norm(points[far]))
            if best is None or reach > best[2]:
                best = (link, points[far], reach)
        assert best is not None, "every side carries geometry"
        return best[0], best[1]


def _gripper(
    component: Component,
    part: GripperSpec,
    kinematics: Kinematics,
    joints: dict[tuple[str, str], str],
    mounts: dict[str, str],
    meshes: dict[str, list[tuple[_Floats, _Floats]]],
) -> ToolFrame:
    frame = component.mount.frame
    jaw = _jaw(component, part, kinematics, joints, mounts, meshes)
    if isinstance(jaw, NoJaws):
        return ToolFrame(component.instance, frame, jaw)
    tips = [jaw.tip(side, jaw.closed) for side in jaw.sides]
    if part.grasp_centre_m is not None:
        point = np.asarray(part.grasp_centre_m, dtype=np.float64)
    else:
        point = np.mean([tip for _, tip in tips], axis=0)
    step = {
        name: closed + _STEP * (jaw.opened[name] - closed) for name, closed in jaw.closed.items()
    }

    def velocity(link: str) -> _Floats:
        """How the jaw link carries the grasp point as the jaws open from closed."""
        local = np.linalg.solve(jaw.pose(link, jaw.closed), np.append(point, 1.0))
        return (jaw.pose(link, step) @ local)[:3] - point

    relative = velocity(tips[0][0]) - (
        velocity(tips[1][0]) if jaw.kind is JawKind.PARALLEL else np.zeros(3)
    )
    closing = _unit(relative)
    if jaw.kind is JawKind.PARALLEL and closing[int(np.argmax(np.abs(closing)))] < 0:
        closing = -closing
    approach = _unit(point - float(point @ closing) * closing)
    return ToolFrame(
        component.instance, frame, Jaws(jaw.kind, _vec(closing), _vec(approach), _vec(point))
    )


def _jaw(
    component: Component,
    part: GripperSpec,
    kinematics: Kinematics,
    joints: dict[tuple[str, str], str],
    mounts: dict[str, str],
    meshes: dict[str, list[tuple[_Floats, _Floats]]],
) -> _Jaw | NoJaws:
    frame = component.mount.frame
    if part.grasp is GraspKind.DEXTEROUS:
        return NoJaws(NoJawsReason.MULTI_FINGER_HAND, f"{part.part_id} is a multi-finger hand")
    scoped = _subtree_joints(kinematics, frame, set(mounts.values()) - {frame})
    actuated = {
        joints[(component.instance, axis.name)]: _range(axis.bounds) for axis in part.layout.axes
    }
    gripper = set(actuated) | {
        _resolve(scoped, mimic.joint_name, component.instance) for mimic in part.mimic_joints
    }
    moving = {
        link
        for link in kinematics.links
        if any(
            joint.name in gripper
            for joint in kinematics._ancestry(link)  # pyright: ignore[reportPrivateUsage]
        )
    }
    if not any(meshes.get(link) for link in moving):
        return NoJaws(
            NoJawsReason.JAWS_UNDESCRIBED,
            f"the description draws no collision geometry on {part.part_id}'s jaws",
        )
    tops = sorted(name for name in gripper if kinematics.joints[name].parent not in moving)
    pieces = {top: frozenset(_below(kinematics, kinematics.joints[top].child)) for top in tops}
    near = {name: min(max(0.0, lo), hi) for name, (lo, hi) in actuated.items()}
    far = {name: lo if abs(lo) >= abs(hi) else hi for name, (lo, hi) in actuated.items()}
    hand = frozenset(
        link
        for link in kinematics.links
        if link not in moving
        and _within(kinematics, frame, link)
        and kinematics.is_rigid(frame, link)
    )
    probe = _Jaw(kinematics, frame, meshes, JawKind.PARALLEL, (), hand, np.zeros(3), far, near)
    everything = frozenset(moving)
    moved = probe.vertices(everything, far) - probe.vertices(everything, near)
    if float(np.abs(moved).max()) < 1e-6:
        return NoJaws(NoJawsReason.NO_JAW_MOTION, f"{part.part_id}'s joints move none of its jaws")
    direction = _principal(moved)
    shift = {
        top: float(np.mean((probe.vertices(links, far) - probe.vertices(links, near)) @ direction))
        for top, links in pieces.items()
        if any(meshes.get(link) for link in links)
    }
    largest = max(abs(value) for value in shift.values())
    plus = [top for top, value in shift.items() if value > _SIDE_SHARE * largest]
    minus = [top for top, value in shift.items() if value < -_SIDE_SHARE * largest]
    if not plus:
        plus, minus, direction = minus, plus, -direction
    kind = JawKind.PARALLEL if minus else JawKind.SINGLE_MOVING
    sides = tuple(
        frozenset[str]().union(*(pieces[top] for top in side)) for side in (plus, minus) if side
    )

    def gap(positions: dict[str, float]) -> float:
        first = probe.vertices(sides[0], positions) @ direction
        other = (
            probe.vertices(sides[1], positions) @ direction
            if kind is JawKind.PARALLEL
            else probe.vertices(hand, positions) @ direction
        )
        return abs(float(first.mean()) - (float(other.mean()) if other.size else 0.0))

    opened, closed = (far, near) if gap(far) >= gap(near) else (near, far)
    return _Jaw(kinematics, frame, meshes, kind, sides, hand, direction, opened, closed)


def fingertip_line(embodiment: Embodiment, tool: ToolFrame) -> Vec3 | None:
    """For parallel jaws, the unit line between their fingertips with the jaws open (the
    centre of each side's vertices within :data:`_PAD_M` of its furthest out along the
    approach), made normal to the approach and signed as
    the closing line: the check the derived closing line is held to. ``None`` for any other
    tool."""
    if not isinstance(tool.grip, Jaws) or tool.grip.kind is not JawKind.PARALLEL:
        return None
    component = next(c for c in embodiment.components if c.instance == tool.component)
    part = component.part
    assert isinstance(part, GripperSpec)
    kinematics = Kinematics.of(embodiment.urdf_bytes)
    joints = _resolved_joints(embodiment)
    mounts = {c.instance: c.mount.frame for c in embodiment.components}
    meshes = _collision_meshes(embodiment, embodiment.urdf_bytes)
    jaw = _jaw(component, part, kinematics, joints, mounts, meshes)
    assert isinstance(jaw, _Jaw)
    approach = np.asarray(tool.grip.approach_axis)

    def tip(links: frozenset[str]) -> _Floats:
        """The centre of the jaw's pad: its vertices within :data:`_PAD_M` of its furthest
        out along the approach."""
        points = jaw.vertices(links, jaw.opened)
        out = points @ approach
        return points[out >= out.max() - _PAD_M].mean(axis=0)

    line = tip(jaw.sides[0]) - tip(jaw.sides[1])
    line = _unit(line - float(line @ approach) * approach)
    return _vec(line if float(line @ np.asarray(tool.grip.closing_axis)) >= 0 else -line)


def _resolved_joints(embodiment: Embodiment) -> dict[tuple[str, str], str]:
    """Each component's joint, by its instance and local name, as the description names it."""
    resolved = zip(embodiment.state.coordinates, embodiment.joint_names, strict=True)
    return {(str(c.instance), c.joint_name): name for c, name in resolved}


def _range(bounds: Bounds | Unbounded) -> tuple[float, float]:
    if not isinstance(bounds, Bounds):
        raise TypeError("a gripper's axes are bounded")  # GripperSpec holds it
    return bounds.lower, bounds.upper


def _resolve(scoped: set[str], local: str, instance: str) -> str:
    candidates = [
        name for name in scoped if name == local or name.endswith(("/" + local, "_" + local))
    ]
    if len(candidates) != 1:
        raise ValueError(f"{instance}/{local} needs one joint below its mount; got {candidates}")
    return candidates[0]


def _below(kinematics: Kinematics, link: str) -> set[str]:
    links = {link}
    changed = True
    while changed:
        changed = False
        for joint in kinematics.joints.values():
            if joint.parent in links and joint.child not in links:
                links.add(joint.child)
                changed = True
    return links


def _within(kinematics: Kinematics, frame: str, link: str) -> bool:
    if link == frame:
        return True
    return any(
        joint.parent == frame
        for joint in kinematics._ancestry(link)  # pyright: ignore[reportPrivateUsage]
    )


def _principal(displacements: _Floats) -> _Floats:
    _, vectors = np.linalg.eigh(displacements.T @ displacements)
    axis = vectors[:, -1]
    return axis if axis[int(np.argmax(np.abs(axis)))] > 0 else -axis


def _unit(vector: _Floats) -> _Floats:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError("a tool axis of no length")
    return vector / norm


def _vec(vector: _Floats) -> Vec3:
    return (float(vector[0]), float(vector[1]), float(vector[2]))


def _rounded(vector: Vec3) -> JsonValue:
    """To nanometres (nano-units of an axis), with no negative zero."""
    return [round(value, 9) + 0.0 for value in vector]


__all__ = [
    "JawKind",
    "Jaws",
    "NoJaws",
    "NoJawsReason",
    "ToolFrame",
    "fingertip_line",
    "tool_frames",
]
