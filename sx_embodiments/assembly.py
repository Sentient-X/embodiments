"""One renderer for every description built from other descriptions.

An :class:`Assembly` mounts whole member descriptions (a vendor's arm-with-gripper URDF, a
complete registered body) on a root link or on any link of a member declared before it, and
adds named frames (camera mounts, leader sites) at nominal poses. Its URDF is a total
function of the declaration, so a registered multi-device body and a runtime composition are
the same operation, and a committed description is checked against the declaration that
renders it rather than kept in step by hand.

A member may be a :class:`Subtree` of a description rather than the whole of it: the parts a
vendor ships as one file (an upper body, a head, a leg) are cut from it by naming the joint each
hangs from, and an assembly of the parts is the vendor's robot again. A member that keeps its
own names (``namespaced=False``) is how a part rejoins the body it was cut from.

The component graph follows the same declaration: :meth:`Assembly.components_of` namespaces a
member's own components exactly as the renderer namespaces its links, so a body cannot name a
frame its description does not have. Poses here are nominal design facts; a measured mount
belongs to the unit's calibration, never to the assembly.
"""

from __future__ import annotations

import copy
import math
import re
import xml.etree.ElementTree as ET
from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import StrEnum
from typing import cast

from .assets import PackagedAsset
from .compose import Component, MountedOn, RootMount
from .errors import CompositionError

#: The note the registry's inspection layouts carry in their committed bytes.
INSPECTION_NOTE = (
    " Independent device roots use a nominal inspection layout; measured station "
    "extrinsics remain recording-time facts. "
)
_NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
_NAMED = frozenset({"link", "joint", "transmission", "actuator", "material"})


class MountJoint(StrEnum):
    """How a member or frame meets its parent link."""

    FIXED = "fixed"
    FLOATING = "floating"  # placed per installation; the nominal pose is only a default


def _number(value: float) -> str:
    text = repr(float(value))
    return text.removesuffix(".0")


@dataclass(frozen=True, slots=True)
class Pose:
    """A nominal child-in-parent pose, metres and radians (URDF ``xyz``/``rpy``)."""

    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if any(
            len(vector) != 3 or not all(map(math.isfinite, vector))
            for vector in (self.xyz, self.rpy)
        ):
            raise CompositionError("pose", "xyz and rpy must each contain three finite values")

    def origin(self) -> dict[str, str]:
        return {
            "xyz": " ".join(map(_number, self.xyz)),
            "rpy": " ".join(map(_number, self.rpy)),
        }


@dataclass(frozen=True, slots=True)
class Subtree:
    """The part of one description below a joint, or from its root without some joints.

    ``below`` names the joint the part hangs from. The part keeps that joint, so a revolute
    hip still moves, under a root link named ``<joint>_mount``; the joint's origin moves onto
    the member's mount (:meth:`pose`) and the joint itself sits at the identity, which leaves
    every link where the source put it. Without ``below`` the part is rooted where the source
    is. ``without`` names joints whose whole subtrees stay out: the legs an upper body leaves,
    the head a neck replaces.
    """

    source: PackagedAsset
    below: str | None = None
    without: tuple[str, ...] = ()

    def _tree(self) -> ET.Element:
        return ET.fromstring(self.source.path().read_bytes())

    def _joint(self, robot: ET.Element, name: str) -> ET.Element:
        found = next((j for j in robot.findall("joint") if j.get("name") == name), None)
        if found is None:
            raise CompositionError(self.source.relpath, f"the description has no joint {name!r}")
        return found

    @property
    def mount_link(self) -> str | None:
        return None if self.below is None else f"{self.below}_mount"

    def parent(self) -> str | None:
        """The source link the part hung from; ``None`` for a part rooted where the source is."""
        if self.below is None:
            return None
        parent = self._joint(self._tree(), self.below).find("parent")
        assert parent is not None
        return parent.attrib["link"]

    def pose(self) -> Pose:
        """Where the part's root sits on :meth:`parent`: the cut joint's own origin."""
        if self.below is None:
            return Pose()
        origin = self._joint(self._tree(), self.below).find("origin")
        if origin is None:
            return Pose()
        xyz, rpy = (
            tuple(float(v) for v in origin.get(key, "0 0 0").split()) for key in ("xyz", "rpy")
        )
        return Pose(
            cast("tuple[float, float, float]", xyz), cast("tuple[float, float, float]", rpy)
        )

    def urdf(self) -> bytes:
        robot = self._tree()
        joints = robot.findall("joint")
        children: dict[str, list[ET.Element]] = {}
        for joint in joints:
            parent = joint.find("parent")
            assert parent is not None
            children.setdefault(parent.attrib["link"], []).append(joint)
        for name in self.without:
            self._joint(robot, name)
        cut = None if self.below is None else self._joint(robot, self.below)
        start = _root_link(robot) if cut is None else _child(cut)
        kept_links = {start}
        kept_joints: list[ET.Element] = [] if cut is None else [cut]
        stack = [start]
        while stack:
            for joint in children.get(stack.pop(), []):
                if joint.get("name") in self.without:
                    continue
                kept_joints.append(joint)
                kept_links.add(_child(joint))
                stack.append(_child(joint))
        part = ET.Element("robot", dict(robot.attrib))
        if cut is not None:
            assert self.mount_link is not None
            ET.SubElement(part, "link", {"name": self.mount_link})
        for item in robot:
            if item.tag == "link" and item.get("name") in kept_links:
                part.append(copy.deepcopy(item))
            elif item.tag == "joint" and item in kept_joints:
                joint = copy.deepcopy(item)
                if item is cut:
                    origin = joint.find("origin")
                    if origin is not None:
                        joint.remove(origin)
                    parent = joint.find("parent")
                    assert parent is not None and self.mount_link is not None
                    parent.set("link", self.mount_link)
                part.append(joint)
            elif item.tag not in ("link", "joint") and _keeps(item, kept_links, kept_joints):
                part.append(copy.deepcopy(item))
        return ET.tostring(part, encoding="utf-8")


def _child(joint: ET.Element) -> str:
    child = joint.find("child")
    assert child is not None
    return child.attrib["link"]


def _keeps(item: ET.Element, links: set[str], joints: list[ET.Element]) -> bool:
    """Materials and other top-level elements stay unless they reference what was cut."""
    names = {joint.get("name") for joint in joints}
    if item.tag == "transmission":
        return all(j.get("name") in names for j in item.iter("joint"))
    if item.tag == "gazebo" and item.get("reference"):
        return item.get("reference") in links
    return True


@dataclass(frozen=True, slots=True)
class Member:
    """One whole description mounted under ``name``.

    ``parent`` is a link already in the assembly (the root when ``None``), so a member can
    sit on another member: an arm on a torso, a camera head on a neck. ``mesh_uri`` rewrites
    mesh and texture references when the rendered bytes will not sit beside the source.
    ``seat`` puts the member where a cut part hangs from its source, on the cut joint's
    parent link at its origin, read when the assembly renders: the vendor head rejoining the
    chest, or another body taking the head's place. ``namespaced`` is false for a part
    rejoining the body it was cut from, which keeps the source's names; the assembly still
    refuses any name two members share.
    """

    name: str
    description: PackagedAsset | bytes | Subtree
    pose: Pose = Pose()
    parent: str | None = None
    joint: MountJoint = MountJoint.FIXED
    mesh_uri: Callable[[str], str] | None = None
    namespaced: bool = True
    seat: Subtree | None = None

    def __post_init__(self) -> None:
        if _NAME.fullmatch(self.name) is None:
            raise CompositionError(self.name, f"invalid member name {self.name!r}")
        if self.seat is not None and (self.pose != Pose() or self.parent is not None):
            raise CompositionError(self.name, "a seated member takes its pose from the seat")

    def mount(self) -> tuple[str | None, Pose]:
        """The link this member mounts on (the root when ``None``) and its pose there."""
        if self.seat is None:
            return self.parent, self.pose
        return self.seat.parent(), self.seat.pose()

    def source(self) -> ET.Element:
        match self.description:
            case bytes():
                data = self.description
            case Subtree():
                data = self.description.urdf()
            case _:
                data = self.description.path().read_bytes()
        return ET.fromstring(data)


@dataclass(frozen=True, slots=True)
class Frame:
    """A named link at a nominal pose on ``parent``: a camera mount or a leader site."""

    name: str
    parent: str
    pose: Pose = Pose()
    joint: MountJoint = MountJoint.FIXED
    preview_kind: str | None = None
    preview_label: str | None = None


@dataclass(frozen=True, slots=True)
class Assembly:
    """Members and frames under one root link; ``separator`` joins a member's name to its links."""

    robot: str
    root: str
    members: tuple[Member, ...]
    frames: tuple[Frame, ...] = ()
    separator: str = "_"
    note: str | None = None  # an XML comment heading the rendering

    def __post_init__(self) -> None:
        if not self.members:
            raise CompositionError(self.robot, "an assembly needs at least one member")
        names = [member.name for member in self.members]
        if len(set(names)) != len(names):
            raise CompositionError(self.robot, "member names must be unique")

    def prefix(self, member: str) -> str:
        found = next((m for m in self.members if m.name == member), None)
        if found is None:
            raise CompositionError(self.robot, f"{member!r} is not a member of this assembly")
        return member + self.separator if found.namespaced else ""

    def components_of(
        self,
        member: str,
        components: tuple[Component, ...],
        *,
        instance_separator: str = "_",
        carrier: str | None = None,
    ) -> tuple[Component, ...]:
        """A member body's components, namespaced exactly as its links are rendered.

        carrier is the already-declared component that owns the link a member mounts on;
        the member's root-mounted components then ride on it, so the graph says what the
        description says. Without one they stay root-mounted, which is only true for a member
        on a link no component moves.
        """
        prefix = self.prefix(member)
        instance_prefix = member + instance_separator if prefix else ""
        named = (namespaced(component, instance_prefix, prefix) for component in components)
        if carrier is None:
            return tuple(named)
        return tuple(
            replace(component, mount=MountedOn(carrier, component.mount.frame))
            if isinstance(component.mount, RootMount)
            else component
            for component in named
        )

    def urdf(self) -> bytes:
        """The rendered description: members in order, then frames, deterministically."""
        robot = ET.Element("robot", {"name": self.robot})
        if self.note is not None:
            robot.append(ET.Comment(self.note))
        ET.SubElement(robot, "link", {"name": self.root})
        links = {self.root}
        for member in self.members:
            source = member.source()
            prefix = self.prefix(member.name)
            items = _prefixed(source, prefix, member.mesh_uri)
            parent, pose = member.mount()
            parent = parent if parent is not None else self.root
            if parent not in links:
                raise CompositionError(
                    self.robot,
                    f"{member.name!r} mounts on {parent!r}, not a link declared before it",
                )
            robot.extend(items)
            links |= {item.attrib["name"] for item in items if item.tag == "link"}
            _mount(
                robot,
                parent=parent,
                child=prefix + _root_link(source),
                name=f"{parent}_to_{member.name}",
                pose=pose,
                joint=member.joint,
            )
        for frame in self.frames:
            if frame.parent not in links:
                raise CompositionError(
                    self.robot, f"frame {frame.name!r} mounts on unknown link {frame.parent!r}"
                )
            attributes = {"name": frame.name}
            if frame.preview_kind is not None:
                attributes["data-preview-kind"] = frame.preview_kind
            if frame.preview_label is not None:
                attributes["data-preview-label"] = frame.preview_label
            ET.SubElement(robot, "link", attributes)
            links.add(frame.name)
            _mount(
                robot,
                parent=frame.parent,
                child=frame.name,
                name=f"{frame.parent}_to_{frame.name}",
                pose=frame.pose,
                joint=frame.joint,
            )
        _require_unique_names(self.robot, robot)
        ET.indent(robot, space="  ")
        return ET.tostring(robot, encoding="utf-8", xml_declaration=True) + b"\n"


def namespaced(component: Component, instance_prefix: str, frame_prefix: str) -> Component:
    """Rename one component and its mount; the part keeps its own identity."""
    mount = component.mount
    return replace(
        component,
        instance=instance_prefix + component.instance,
        mount=(
            MountedOn(instance_prefix + mount.parent, frame_prefix + mount.frame)
            if isinstance(mount, MountedOn)
            else RootMount(frame_prefix + mount.frame)
        ),
    )


def _require_unique_names(name: str, robot: ET.Element) -> None:
    for tag in ("link", "joint"):
        names = [element.attrib["name"] for element in robot.findall(tag)]
        repeated = sorted({n for n in names if names.count(n) > 1})
        if repeated:
            raise CompositionError(name, f"{tag} names collide: {repeated}")


def _root_link(source: ET.Element) -> str:
    links = {link.attrib["name"] for link in source.findall("link")}
    children = {child.attrib["link"] for child in source.findall("joint/child")}
    roots = links - children
    if len(roots) != 1:
        raise CompositionError("assembly", f"a member needs one root link, got {sorted(roots)}")
    return roots.pop()


def _prefixed(
    source: ET.Element, prefix: str, mesh_uri: Callable[[str], str] | None
) -> list[ET.Element]:
    output: list[ET.Element] = []
    for child in source:
        item = copy.deepcopy(child)
        for node in item.iter():
            if node.tag in _NAMED and node.get("name"):
                node.set("name", prefix + node.attrib["name"])
            if node.tag in {"parent", "child"} and node.get("link"):
                node.set("link", prefix + node.attrib["link"])
            if node.tag == "mimic" and node.get("joint"):
                node.set("joint", prefix + node.attrib["joint"])
            if node.tag == "gazebo" and node.get("reference"):
                node.set("reference", prefix + node.attrib["reference"])
            if mesh_uri is not None and node.tag in {"mesh", "texture"} and node.get("filename"):
                node.set("filename", mesh_uri(node.attrib["filename"]))
        output.append(item)
    return output


def _mount(
    robot: ET.Element, *, parent: str, child: str, name: str, pose: Pose, joint: MountJoint
) -> None:
    element = ET.SubElement(robot, "joint", {"name": name, "type": joint.value})
    ET.SubElement(element, "origin", pose.origin())
    ET.SubElement(element, "parent", {"link": parent})
    ET.SubElement(element, "child", {"link": child})
