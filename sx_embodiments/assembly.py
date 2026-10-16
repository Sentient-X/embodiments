"""One renderer for every description built from other descriptions.

An :class:`Assembly` mounts whole member descriptions (a vendor's arm-with-gripper URDF, a
complete registered body) on a root link or on any link of a member declared before it, and
adds named frames (camera mounts, leader sites) at nominal poses. Its URDF is a total
function of the declaration, so a registered multi-device body and a runtime composition are
the same operation, and a committed description is checked against the declaration that
renders it rather than kept in step by hand.

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
class Member:
    """One whole description mounted under ``name``.

    ``parent`` is a link already in the assembly (the root when ``None``), so a member can
    sit on another member: an arm on a torso, a camera head on a neck. ``mesh_uri`` rewrites
    mesh and texture references when the rendered bytes will not sit beside the source.
    """

    name: str
    description: PackagedAsset | bytes
    pose: Pose = Pose()
    parent: str | None = None
    joint: MountJoint = MountJoint.FIXED
    mesh_uri: Callable[[str], str] | None = None

    def __post_init__(self) -> None:
        if _NAME.fullmatch(self.name) is None:
            raise CompositionError(self.name, f"invalid member name {self.name!r}")

    def source(self) -> ET.Element:
        data = (
            self.description
            if isinstance(self.description, bytes)
            else self.description.path().read_bytes()
        )
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
        if member not in {m.name for m in self.members}:
            raise CompositionError(self.robot, f"{member!r} is not a member of this assembly")
        return member + self.separator

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
        named = (
            namespaced(component, member + instance_separator, prefix) for component in components
        )
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
            parent = member.parent if member.parent is not None else self.root
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
                pose=member.pose,
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
