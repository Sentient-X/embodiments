"""Modules that mate: the mechanical units every body is built from, and how they join.

A **mechanical interface** is the pattern two parts join by: an ISO 9409-1 tool flange, a
vendor's wrist flange, the 1/4-20 thread under a camera, a humanoid's neck seat. A **module** is
one mechanical unit (an arm, a gripper, a camera, a pan-tilt neck, an adapter plate, a fingertip
pad): a description, the interface its root link joins by (its plug, or none for a body that
stands on the floor), the **sockets** it offers (an interface at a pose on one of its links), and
the components it carries, named in its own links. A passive element is a module with no
components; its mass and geometry are its description (:func:`rigid_module`).

A **build** places modules on a root at installation poses and mates modules plug-to-socket,
and renders through :class:`~sx_embodiments.assembly.Assembly`, so the description, the component
graph and which component carries which are one declaration. Two sites join only when their
interface ids are equal; a plug onto a socket of another interface, a second module on one socket
and a socket its module does not have are typed refusals, never a silent placement.

A vendor often ships an arm and its gripper as one file. The arm's socket is then **fitted**: its
own description already carries a module there, cut from that description below the joint the
module hangs from. A build that keeps every fitted module renders the vendor's bytes unchanged;
mating something else onto a fitted socket cuts the fitted module out, which is a gripper swap.
``tests/test_mechanical.py`` holds that a fitted module is detachable: cut out and mated back, it
puts every link where the vendor put it and carries the same components.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Final, NewType

from .assembly import Assembly, Frame, Member, MountJoint, Pose, Subtree, namespaced
from .assets import PackagedAsset
from .compose import Component, MountedOn, RootMount
from .errors import CompositionError, InterfaceMismatchError, SocketError

if TYPE_CHECKING:
    from .mobile_base import MobileBase

InterfaceId = NewType("InterfaceId", str)
"""A mechanical interface's id in the open interface registry (:data:`MECHANICAL_INTERFACES`)."""

ModuleId = NewType("ModuleId", str)

_NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*")


@dataclass(frozen=True, slots=True)
class MechanicalInterface:
    """One joining pattern and what defines it (a standard, or the vendor drawing it comes from)."""

    interface: InterfaceId
    label: str
    standard: str


MECHANICAL_INTERFACES: Final[Mapping[InterfaceId, MechanicalInterface]] = {
    record.interface: record
    for record in (
        MechanicalInterface(
            InterfaceId("iso-9409-1-50-4-m6"),
            "ISO 9409-1 tool flange, 50 mm pitch circle, four M6",
            "ISO 9409-1:2004, form A, d1 = 50 mm",
        ),
        MechanicalInterface(
            InterfaceId("unc-1-4-20"),
            "1/4-20 UNC camera thread",
            "ISO 1222:2010 tripod connection, 1/4-20 UNC",
        ),
        MechanicalInterface(
            InterfaceId("agilex-piper-flange"),
            "AgileX Piper wrist flange",
            "agilexrobotics/piper_ros piper_description, joint6_to_gripper_base",
        ),
        MechanicalInterface(
            InterfaceId("seeed-b601-flange"),
            "Seeed reBot B601-DM wrist flange",
            "Seeed-Projects/reBotArmController_ROS2 reBot_B601_DM_with_gripper, gripper_joint",
        ),
        MechanicalInterface(
            InterfaceId("rainbow-rby1-flange"),
            "Rainbow Robotics RB-Y1 wrist flange",
            "RainbowRobotics/rby1-sdk model_v1.3, END_right and END_left",
        ),
        MechanicalInterface(
            InterfaceId("interbotix-xs-gripper"),
            "Interbotix X-Series gripper seat on the wrist-rotate link",
            "Interbotix/interbotix_ros_manipulators vx300s, ee_arm",
        ),
        MechanicalInterface(
            InterfaceId("robotis-rh-p12-rn"),
            "ROBOTIS RH-P12-RN gripper mount",
            "ROBOTIS-GIT/ai_worker ffw_bg2_rev4_follower, gripper_l_joint and gripper_r_joint",
        ),
        MechanicalInterface(
            InterfaceId("seeed-b601-base"),
            "Seeed reBot B601-DM base plate",
            "Seeed-Projects/reBotArmController_ROS2 reBot_B601_DM_with_gripper, base_link",
        ),
        MechanicalInterface(
            InterfaceId("so101-base"),
            "SO-101 base plate",
            "TheRobotStudio/SO-ARM100 SO-101 base, the plate the arm is clamped or screwed by",
        ),
        MechanicalInterface(
            InterfaceId("sentient-humanoid-wrist"),
            "Sentient humanoid wrist, the wrist-yaw link's frame",
            "Sentient-X/humanoid_pkg urdf_robot_acc, *_wrist_yaw_link",
        ),
        MechanicalInterface(
            InterfaceId("sentient-humanoid-neck"),
            "Sentient humanoid neck seat on the chest",
            "Sentient-X/humanoid_pkg urdf_robot_acc, neck_joint",
        ),
        MechanicalInterface(
            InterfaceId("sentient-humanoid-camera-holder"),
            "Sentient humanoid head camera holder",
            "Sentient-X/humanoid_pkg humanoid_pkg.csv, Camera_Holder-1 on head_link",
        ),
        MechanicalInterface(
            InterfaceId("sentient-humanoid-hip"),
            "Sentient humanoid hip seat on the pelvis",
            "Sentient-X/humanoid_pkg urdf_robot_acc, *_hip_pitch_joint",
        ),
    )
}


def interface_label(interface: InterfaceId) -> str:
    """The interface's name, or its id when the registry does not list it (a tenant's own)."""
    record = MECHANICAL_INTERFACES.get(interface)
    return record.label if record is not None else interface.replace("-", " ")


Description = PackagedAsset | Subtree | bytes


@dataclass(frozen=True, slots=True)
class Socket:
    """A site a module offers: ``interface`` at ``pose`` on ``link`` of the module's description.

    ``carrier`` is the module's own component whose joints move ``link`` (``None`` when nothing
    does), so what mates here rides on it; ``tests/test_mechanical.py`` holds it to the
    description. ``fitted`` is the module the description already carries here, cut from it
    below the joint it hangs from; ``link`` and ``pose`` are then that joint's parent and
    origin. ``FLOATING`` is a site whose pose is set per installation, the nominal one a default.
    """

    name: str
    interface: InterfaceId
    link: str
    pose: Pose = field(default_factory=Pose)
    carrier: str | None = None
    joint: MountJoint = MountJoint.FIXED
    fitted: Module | None = None

    def __post_init__(self) -> None:
        if _NAME.fullmatch(self.name) is None:
            raise SocketError(self.name, "a socket name is an identifier")
        if not self.interface.strip() or not self.link.strip():
            raise SocketError(self.name, "a socket names its interface and its link")
        if self.fitted is not None:
            _require_fits(self.fitted, self)
            if not isinstance(self.fitted.description, Subtree) or (
                self.fitted.description.below is None
            ):
                raise SocketError(
                    self.name, "a fitted module is the cut of its host below one joint"
                )


@dataclass(frozen=True, slots=True)
class Slot:
    """Where, in a module's component order, the components of the module fitted on ``socket``
    go. Without one they follow the module's own, in socket order; with one, a vendor's state
    order (left arm, left gripper, right arm, right gripper) stays the vendor's."""

    socket: str


@dataclass(frozen=True, slots=True)
class Module:
    """One mechanical unit: a description, its plug, the sockets it offers and its components.

    ``components`` are named in the module's own links and instances; a build namespaces them
    with the module's place, and a root-mounted one rides on whatever carries that place.
    ``plug`` is the interface the description's root link joins by; ``None`` is a body that
    stands on the floor or is held, placed rather than mated. A ``None`` description is hardware
    whose geometry is not captured yet: it is a body on its own, never placed or mated.
    ``frames`` are named sites the module adds on its own links (a camera's optical frame); they
    render in a rooted build, after every member, named like the module's links.
    ``base`` is the mobile base a drivable module is made from (:attr:`MobileBase.module`),
    whose ratings and support the composition checks read.
    """

    module_id: ModuleId
    label: str
    description: Description | None
    plug: InterfaceId | None = None
    sockets: tuple[Socket, ...] = ()
    components: tuple[Component | Slot, ...] = ()
    frames: tuple[Frame, ...] = ()
    base: MobileBase | None = None

    def __post_init__(self) -> None:
        if not self.module_id.strip() or not self.label.strip():
            raise CompositionError(str(self.module_id), "a module needs an id and a label")
        names = [socket.name for socket in self.sockets]
        if len(set(names)) != len(names):
            raise SocketError(str(self.module_id), "socket names must be unique")
        slotted = [entry.socket for entry in self.components if isinstance(entry, Slot)]
        fitted = {socket.name for socket in self.sockets if socket.fitted is not None}
        if len(set(slotted)) != len(slotted) or not set(slotted) <= fitted:
            raise SocketError(str(self.module_id), "a slot names each fitted socket at most once")
        instances = {entry.instance for entry in self.components if not isinstance(entry, Slot)}
        for socket in self.sockets:
            if socket.carrier is not None and socket.carrier not in instances:
                raise SocketError(
                    str(self.module_id),
                    f"{socket.name!r} rides on {socket.carrier!r}, not one of its components",
                )
            if socket.fitted is None:
                continue
            cut = socket.fitted.description
            assert isinstance(cut, Subtree)
            if cut.source != source(self.description):
                raise SocketError(
                    str(self.module_id),
                    f"{socket.name!r} is fitted with a cut of another description",
                )

    def socket(self, name: str) -> Socket:
        found = next((socket for socket in self.sockets if socket.name == name), None)
        if found is None:
            offered = ", ".join(socket.name for socket in self.sockets) or "none"
            raise SocketError(str(self.module_id), f"has no socket {name!r}; it offers {offered}")
        return found

    def links(self) -> frozenset[str]:
        """Every link of the module's description; reading it reads the file."""
        if self.description is None:
            return frozenset()
        robot = ET.fromstring(_bytes(self.description))
        return frozenset(link.attrib["name"] for link in robot.findall("link"))

    def without(self, sockets: frozenset[str]) -> Description:
        """The description with the modules fitted on ``sockets`` cut out."""
        if self.description is None:
            raise CompositionError(
                str(self.module_id), "a module with no captured description is not placed"
            )
        cuts = tuple(
            cut.below
            for socket in self.sockets
            if socket.name in sockets
            and socket.fitted is not None
            and isinstance(cut := socket.fitted.description, Subtree)
            and cut.below is not None
        )
        if not cuts:
            return self.description
        match self.description:
            case Subtree():
                return replace(self.description, without=(*self.description.without, *cuts))
            case PackagedAsset():
                return Subtree(self.description, without=cuts)
            case _:
                raise SocketError(str(self.module_id), "generated bytes carry no fitted module")


def source(description: Description | None) -> PackagedAsset | bytes | None:
    """The file a description is, or is cut from; generated bytes are their own source."""
    return description.source if isinstance(description, Subtree) else description


def _require_fits(module: Module, socket: Socket) -> None:
    if module.plug is None:
        raise SocketError(
            str(module.module_id), "has no plug: it stands on its own and is placed, not mated"
        )
    if module.plug != socket.interface:
        raise InterfaceMismatchError(
            str(module.module_id), module.plug, socket.name, socket.interface
        )


@dataclass(frozen=True, slots=True)
class Placed:
    """A module set down on the build's root at an installation pose (a bench layout, a floating
    camera). Not a mechanical join; see :class:`Mate`."""

    name: str
    module: Module
    pose: Pose = field(default_factory=Pose)
    joint: MountJoint = MountJoint.FIXED
    namespaced: bool = True


@dataclass(frozen=True, slots=True)
class Mate:
    """``module``'s plug joined onto ``socket`` of the earlier placed or mated module ``host``.

    ``namespaced`` is false for a part rejoining the description it was cut from, which keeps
    that description's link and component names.
    """

    name: str
    module: Module
    host: str
    socket: str
    namespaced: bool = True


@dataclass(frozen=True, slots=True)
class Build:
    """Modules placed and mated into one body, rendered by one :class:`Assembly`.

    With no ``root`` the build is one module as it was made: its description is the module's
    own, byte for byte, and its components are the module's and its fitted modules'. With a
    root, ``placed`` and ``mates`` render in order (member order is state order), ``frames``
    add named sites, and ``devices`` are components on those sites with no description of their
    own (a leader arm whose kinematics are not captured), listed first.
    """

    placed: tuple[Placed, ...]
    mates: tuple[Mate, ...] = ()
    robot: str | None = None
    root: str | None = None
    frames: tuple[Frame, ...] = ()
    devices: tuple[Component, ...] = ()
    separator: str = "_"
    note: str | None = None

    def __post_init__(self) -> None:
        if not self.placed:
            raise CompositionError(str(self.robot), "a build places at least one module")
        if self.root is None and (
            len(self.placed) != 1 or self.mates or self.frames or self.devices or self.robot
        ):
            raise CompositionError(
                str(self.robot), "a build without a root is one module as it was made"
            )
        if self.root is not None and self.robot is None:
            raise CompositionError(self.root, "a rooted build names its robot")
        if self.root is None and _frames(self.placed[0].module, frozenset(), ""):
            raise CompositionError(
                str(self.placed[0].module.module_id),
                "a module that adds frames renders only in a rooted build",
            )
        names = [entry.name for entry in (*self.placed, *self.mates)]
        if len(set(names)) != len(names):
            raise CompositionError(str(self.robot), "placed and mated names must be unique")
        modules = {entry.name: entry.module for entry in self.placed}
        taken: set[tuple[str, str]] = set()
        for mate in self.mates:
            host = modules.get(mate.host)
            if host is None:
                raise SocketError(
                    str(self.robot),
                    f"{mate.name!r} mates on {mate.host!r}, not a module placed or mated before it",
                )
            _require_fits(mate.module, host.socket(mate.socket))
            if (mate.host, mate.socket) in taken:
                raise SocketError(
                    str(self.robot), f"{mate.host}.{mate.socket} already carries a module"
                )
            taken.add((mate.host, mate.socket))
            modules[mate.name] = mate.module

    @classmethod
    def of(cls, module: Module) -> Build:
        """A body that is one module as its maker shipped it."""
        return cls(placed=(Placed(str(module.module_id), module, namespaced=False),))

    def _opened(self, name: str) -> frozenset[str]:
        return frozenset(mate.socket for mate in self.mates if mate.host == name)

    def assembly(
        self, mesh_uri: Callable[[Module], Callable[[str], str] | None] | None = None
    ) -> Assembly:
        """The rendering: placed modules, then mates, each on its socket's link and pose.

        ``mesh_uri`` gives each module's mesh-reference rewrite, for bytes that will not sit
        beside their sources (a runtime composition); the registry's sit beside them.
        """
        if self.root is None or self.robot is None:
            raise CompositionError(str(self.robot), "a build without a root renders no assembly")
        prefixes: dict[str, str] = {}
        members: list[Member] = []
        modules: dict[str, Module] = {}
        for entry in self.placed:
            prefixes[entry.name] = entry.name + self.separator if entry.namespaced else ""
            modules[entry.name] = entry.module
            members.append(
                Member(
                    entry.name,
                    entry.module.without(self._opened(entry.name)),
                    entry.pose,
                    joint=entry.joint,
                    mesh_uri=None if mesh_uri is None else mesh_uri(entry.module),
                    namespaced=entry.namespaced,
                )
            )
        for mate in self.mates:
            socket = modules[mate.host].socket(mate.socket)
            prefixes[mate.name] = mate.name + self.separator if mate.namespaced else ""
            modules[mate.name] = mate.module
            members.append(
                Member(
                    mate.name,
                    mate.module.without(self._opened(mate.name)),
                    socket.pose,
                    parent=prefixes[mate.host] + socket.link,
                    joint=socket.joint,
                    mesh_uri=None if mesh_uri is None else mesh_uri(mate.module),
                    namespaced=mate.namespaced,
                )
            )
        added = tuple(
            frame
            for entry in (*self.placed, *self.mates)
            for frame in _frames(entry.module, self._opened(entry.name), prefixes[entry.name])
        )
        return Assembly(
            robot=self.robot,
            root=self.root,
            members=tuple(members),
            frames=(*self.frames, *added),
            separator=self.separator,
            note=self.note,
        )

    def urdf(self) -> bytes:
        """The rendered description; a mate on a socket whose link its module's description
        lacks is refused here, where the descriptions are read."""
        if self.root is None:
            (only,) = self.placed
            if only.module.description is None:
                raise CompositionError(str(only.module.module_id), "has no captured description")
            return _bytes(only.module.description)
        modules = {entry.name: entry.module for entry in (*self.placed, *self.mates)}
        for mate in self.mates:
            host = modules[mate.host]
            socket = host.socket(mate.socket)
            if socket.link not in host.links():
                raise SocketError(
                    str(self.robot),
                    f"{host.module_id}.{socket.name} sits on {socket.link!r}, "
                    "which its description does not have",
                )
        return self.assembly().urdf()

    def components(self) -> tuple[Component, ...]:
        """Every module's components in build order, namespaced, each riding on its carrier.

        Read from the declarations alone, without the descriptions; :meth:`urdf` is what proves
        each socket's link exists.

        A mated module's own root-mounted components ride on its socket's carrier, and a fitted
        module's on its socket's carrier in the host; a fitted module's root link is named by
        its socket's link, since the host's description has no separate one, and so is a cut
        part's that rejoins its source under the source's names.
        """
        out = list(self.devices)
        modules: dict[str, Module] = {}
        prefixes: dict[str, str] = {}
        carriers: dict[str, str | None] = {}
        for entry in self.placed:
            prefix = entry.name + self.separator if entry.namespaced and self.root else ""
            modules[entry.name], prefixes[entry.name], carriers[entry.name] = (
                entry.module,
                prefix,
                None,
            )
            out.extend(_carried(entry.module, self._opened(entry.name), prefix, None, None))
        for mate in self.mates:
            socket = modules[mate.host].socket(mate.socket)
            host = prefixes[mate.host]
            # A socket nothing in its module moves rides on whatever carries that module: a
            # camera on a passive bracket rides on the arm the bracket is mated to.
            rides = carriers[mate.host] if socket.carrier is None else host + socket.carrier
            prefix = mate.name + self.separator if mate.namespaced else ""
            cut = mate.module.description
            seat = (
                (cut.mount_link, host + socket.link)
                if not mate.namespaced and isinstance(cut, Subtree) and cut.mount_link is not None
                else None
            )
            modules[mate.name], prefixes[mate.name], carriers[mate.name] = (
                mate.module,
                prefix,
                rides,
            )
            out.extend(_carried(mate.module, self._opened(mate.name), prefix, rides, seat))
        return tuple(out)


def _carried(
    module: Module,
    opened: frozenset[str],
    prefix: str,
    rides: str | None,
    seat: tuple[str, str] | None,
) -> list[Component]:
    """A module's own components and its fitted modules', in its declared order, namespaced.

    ``rides`` is the component a root-mounted one rides on; ``seat`` renames the module's root
    link (its own name) to the link it sits on (a rendered name).
    """
    out: list[Component] = []
    slotted = {entry.socket for entry in module.components if isinstance(entry, Slot)}
    for entry in module.components:
        if isinstance(entry, Slot):
            out.extend(_fitted(module.socket(entry.socket), opened, prefix, rides))
            continue
        component = namespaced(entry, prefix, prefix)
        if seat is not None and component.mount.frame == prefix + seat[0]:
            component = replace(component, mount=replace(component.mount, frame=seat[1]))
        if rides is not None and isinstance(component.mount, RootMount):
            component = replace(component, mount=MountedOn(rides, component.mount.frame))
        out.append(component)
    for socket in module.sockets:
        if socket.name not in slotted:
            out.extend(_fitted(socket, opened, prefix, rides))
    return out


def _fitted(
    socket: Socket, opened: frozenset[str], prefix: str, host: str | None
) -> list[Component]:
    """What the module fitted on ``socket`` carries, in the host's names, riding on the socket's
    carrier or else on the host's own; nothing when the socket is opened for a mate."""
    fitted = socket.fitted
    if fitted is None or socket.name in opened:
        return []
    cut = fitted.description
    assert isinstance(cut, Subtree) and cut.mount_link is not None
    rides = host if socket.carrier is None else prefix + socket.carrier
    return _carried(fitted, frozenset(), prefix, rides, (cut.mount_link, prefix + socket.link))


def _frames(module: Module, opened: frozenset[str], prefix: str) -> tuple[Frame, ...]:
    """The frames a module and the modules still fitted on it add, named like its links."""
    own = tuple(
        replace(frame, name=prefix + frame.name, parent=prefix + frame.parent)
        for frame in module.frames
    )
    return own + tuple(
        frame
        for socket in module.sockets
        if socket.fitted is not None and socket.name not in opened
        for frame in _frames(socket.fitted, frozenset(), prefix)
    )


def carrier(
    link: str, parents: Mapping[str, tuple[str, str]], owners: Mapping[str, str]
) -> str | None:
    """The component whose joints move ``link``: the owner of the nearest joint above it.

    ``None`` when no component's joint lies above the link (it hangs off a fixed root). A
    component mounted on an ancestor link does not carry it: two arms on a chest do not move
    the head above the chest; the torso below it does.
    """
    current = link
    while current in parents:
        current, joint = parents[current]
        if joint in owners:
            return owners[joint]
    return None


def _bytes(description: Description) -> bytes:
    match description:
        case bytes():
            return description
        case Subtree():
            return description.urdf()
        case _:
            return description.path().read_bytes()


@dataclass(frozen=True, slots=True)
class Box:
    """A solid box, metres along x, y and z."""

    size: tuple[float, float, float]

    def inertia(self, mass: float) -> tuple[float, float, float]:
        x, y, z = self.size
        return (
            mass * (y * y + z * z) / 12,
            mass * (x * x + z * z) / 12,
            mass * (x * x + y * y) / 12,
        )

    @property
    def height(self) -> float:
        return self.size[2]

    def element(self) -> ET.Element:
        return ET.Element("box", {"size": " ".join(_number(v) for v in self.size)})


@dataclass(frozen=True, slots=True)
class Cylinder:
    """A solid cylinder along z, metres."""

    radius: float
    length: float

    def inertia(self, mass: float) -> tuple[float, float, float]:
        side = mass * (3 * self.radius * self.radius + self.length * self.length) / 12
        return (side, side, mass * self.radius * self.radius / 2)

    @property
    def height(self) -> float:
        return self.length

    def element(self) -> ET.Element:
        return ET.Element(
            "cylinder", {"radius": _number(self.radius), "length": _number(self.length)}
        )


Shape = Box | Cylinder


def _number(value: float) -> str:
    return repr(float(value)).removesuffix(".0")


def rigid_module(
    module_id: str,
    label: str,
    *,
    shape: Shape,
    mass_kg: float,
    plug: InterfaceId | None,
    sockets: Mapping[str, InterfaceId] | None = None,
) -> Module:
    """A passive element: one rigid solid rising along +z from its plug face.

    The description has one link, ``mount``, whose frame is the plug face; the solid's inertia
    is the solid's, about its centre. Each named socket sits on the top face (an adapter plate's
    other side, a bracket's camera thread); a fingertip pad or a spacer offers none.
    """
    dimensions = shape.size if isinstance(shape, Box) else (shape.radius, shape.length)
    if (
        not math.isfinite(mass_kg)
        or mass_kg <= 0.0
        or not all(math.isfinite(value) and value > 0.0 for value in dimensions)
    ):
        raise CompositionError(module_id, "a rigid element needs a positive mass and size")
    robot = ET.Element("robot", {"name": module_id})
    link = ET.SubElement(robot, "link", {"name": "mount"})
    centre = {"xyz": f"0 0 {_number(shape.height / 2)}", "rpy": "0 0 0"}
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", centre)
    ET.SubElement(inertial, "mass", {"value": _number(mass_kg)})
    ixx, iyy, izz = shape.inertia(mass_kg)
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": _number(ixx),
            "ixy": "0",
            "ixz": "0",
            "iyy": _number(iyy),
            "iyz": "0",
            "izz": _number(izz),
        },
    )
    for tag in ("visual", "collision"):
        element = ET.SubElement(link, tag)
        ET.SubElement(element, "origin", centre)
        geometry = ET.SubElement(element, "geometry")
        geometry.append(shape.element())
    ET.indent(robot, space="  ")
    top = Pose((0.0, 0.0, shape.height))
    return Module(
        module_id=ModuleId(module_id),
        label=label,
        description=ET.tostring(robot, encoding="utf-8", xml_declaration=True) + b"\n",
        plug=plug,
        sockets=tuple(
            Socket(name, interface, "mount", top) for name, interface in (sockets or {}).items()
        ),
    )
