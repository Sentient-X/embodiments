"""Mobile bases designed from their specs: a chassis, its wheels, its drive and its deck.

A base is the module a composition stands on when it drives: four Seeed arms on one deck,
a camera mast on a cart. :class:`MobileBase` is every fact a person states about one (the
footprint, the wheels and how they drive it, the deck's height and what it may carry, and the
typed sockets on its deck), and :attr:`MobileBase.module` is the module those facts make, so a
base composes like any other module: placed on a build's root, with arms mated onto its deck
sockets plug to socket.

:class:`Drive` is the drive model, the one place twist and wheel speeds meet. It is the
interface a base driver consumes, a body twist in and wheel angular velocities out in the
module's wheel order, and its inverse reads odometry back. No base motor is qualified for
hardware, so a base's wheel axes declare :class:`~sx_embodiments.layout.UndocumentedDrive`
with :data:`NO_QUALIFIED_BASE_MOTOR`: the base drives in simulation, and binding a wheel to a
bus waits on a motor that drives and safe-stops on the bench.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from enum import StrEnum
from typing import Final

from .assembly import Pose
from .compose import RootMount, body_component
from .errors import CompositionError
from .identity import PartId
from .layout import CoordinateUnit, JointAxis, JointLayout, Unbounded, UndocumentedDrive
from .mechanical import Box, Cylinder, InterfaceId, Module, ModuleId, Socket
from .parts import MobileBaseSpec

NO_QUALIFIED_BASE_MOTOR: Final = (
    "no base motor is qualified for hardware; the wheels drive in simulation only"
)

#: The chassis link and the deck link every base module offers its sockets on.
CHASSIS: Final = "base_link"
DECK: Final = "deck"


class DriveKind(StrEnum):
    """How a base's wheels move it."""

    #: Two driven wheels on one axle and two casters: it turns in place, never sideways.
    DIFFERENTIAL = "differential"
    #: Four mecanum (or 45-degree omni) wheels at the corners: it moves in any direction.
    MECANUM = "mecanum"


class DriveError(CompositionError):
    """A twist the drive cannot produce, or wheel speeds of the wrong width."""


@dataclass(frozen=True, slots=True)
class Twist:
    """A planar body velocity in the base's frame: metres per second forward and left, and
    radians per second about the vertical."""

    vx: float
    vy: float
    wz: float


@dataclass(frozen=True, slots=True)
class Drive:
    """The drive model: wheel geometry, kind and limits, and the kinematics between them.

    ``track_m`` is the left-to-right distance between wheel centres; ``wheelbase_m`` is the
    front-to-rear distance between axles (mecanum) or casters (differential).
    """

    kind: DriveKind
    wheel_radius_m: float
    wheel_width_m: float
    track_m: float
    wheelbase_m: float
    max_speed_m_s: float
    max_acceleration_m_s2: float

    def __post_init__(self) -> None:
        values = (
            self.wheel_radius_m,
            self.wheel_width_m,
            self.track_m,
            self.wheelbase_m,
            self.max_speed_m_s,
            self.max_acceleration_m_s2,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise DriveError("drive", "wheel geometry and limits must be positive")

    @property
    def wheels(self) -> tuple[str, ...]:
        """The driven wheels' joint names, in wheel-speed order."""
        if self.kind is DriveKind.DIFFERENTIAL:
            return ("wheel_left", "wheel_right")
        return ("wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right")

    def wheel_velocities(self, twist: Twist) -> tuple[float, ...]:
        """Each wheel's angular velocity, rad/s, for ``twist``, in :attr:`wheels` order.

        A differential drive refuses a sideways velocity rather than dropping it, and every
        drive refuses a twist faster than :attr:`max_speed_m_s` at any wheel's rim.
        """
        r, half_track, half_base = (
            self.wheel_radius_m,
            self.track_m / 2,
            self.wheelbase_m / 2,
        )
        if self.kind is DriveKind.DIFFERENTIAL:
            if abs(twist.vy) > 1e-12:
                raise DriveError("drive", "a differential base cannot move sideways")
            rims = (twist.vx - twist.wz * half_track, twist.vx + twist.wz * half_track)
        else:
            turn = (half_track + half_base) * twist.wz
            rims = (
                twist.vx - twist.vy - turn,
                twist.vx + twist.vy + turn,
                twist.vx + twist.vy - turn,
                twist.vx - twist.vy + turn,
            )
        if max(abs(rim) for rim in rims) > self.max_speed_m_s + 1e-9:
            raise DriveError(
                "drive", f"the twist needs a wheel faster than {self.max_speed_m_s} m/s"
            )
        return tuple(rim / r for rim in rims)

    def twist(self, wheel_velocities: tuple[float, ...]) -> Twist:
        """The body twist the wheels' angular velocities make: odometry, the inverse of
        :meth:`wheel_velocities`."""
        if len(wheel_velocities) != len(self.wheels):
            raise DriveError("drive", f"expected {len(self.wheels)} wheel velocities")
        rims = tuple(value * self.wheel_radius_m for value in wheel_velocities)
        if self.kind is DriveKind.DIFFERENTIAL:
            left, right = rims
            return Twist((left + right) / 2, 0.0, (right - left) / self.track_m)
        front_left, front_right, rear_left, rear_right = rims
        return Twist(
            (front_left + front_right + rear_left + rear_right) / 4,
            (-front_left + front_right + rear_left - rear_right) / 4,
            (-front_left + front_right - rear_left + rear_right)
            / (4 * (self.track_m / 2 + self.wheelbase_m / 2)),
        )


@dataclass(frozen=True, slots=True)
class DeckSocket:
    """A typed seat on the deck: ``interface`` at ``xy`` metres from the deck's centre
    (x forward, y left), turned ``yaw`` radians about the deck's normal."""

    name: str
    interface: InterfaceId
    xy: tuple[float, float]
    yaw: float = 0.0


@dataclass(frozen=True, slots=True)
class MobileBase:
    """A drivable base: its footprint, drive, deck and ratings, and the module they make.

    The chassis is a box ``length_m`` by ``width_m`` whose top is the deck, ``deck_height_m``
    above the floor; ``payload_kg`` is what the deck may carry, every module mated on it
    included. Every deck socket lies on the deck and every wheel inside the footprint's
    sides, or the base is refused.
    """

    module_id: ModuleId
    label: str
    drive: Drive
    length_m: float
    width_m: float
    deck_height_m: float
    mass_kg: float
    payload_kg: float
    deck: tuple[DeckSocket, ...] = ()

    def __post_init__(self) -> None:
        name = str(self.module_id)
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (
                self.length_m,
                self.width_m,
                self.deck_height_m,
                self.mass_kg,
                self.payload_kg,
            )
        ):
            raise CompositionError(name, "a base's size, height, mass and payload are positive")
        if self.deck_height_m <= self.drive.wheel_radius_m:
            raise CompositionError(name, "the deck sits above the wheels' axles")
        if self.drive.track_m > self.width_m + 2 * self.drive.wheel_width_m:
            raise CompositionError(name, "the wheels sit beside the chassis, not beyond it")
        if self.drive.wheelbase_m > self.length_m:
            raise CompositionError(name, "the axles lie within the chassis' length")
        for socket in self.deck:
            x, y = socket.xy
            if abs(x) > self.length_m / 2 or abs(y) > self.width_m / 2:
                raise CompositionError(name, f"deck socket {socket.name!r} lies off the deck")

    @property
    def chassis_height_m(self) -> float:
        """The chassis box's height: from just above the floor clearance to the deck."""
        return self.deck_height_m - self.clearance_m

    @property
    def clearance_m(self) -> float:
        return self.drive.wheel_radius_m / 2

    @property
    def support(self) -> tuple[tuple[float, float], ...]:
        """Where the base touches the floor, in its frame, counter-clockwise: the support
        polygon its centre of mass must stay over."""
        half_track, half_base = self.drive.track_m / 2, self.drive.wheelbase_m / 2
        if self.drive.kind is DriveKind.DIFFERENTIAL:
            return ((half_base, 0.0), (0.0, half_track), (-half_base, 0.0), (0.0, -half_track))
        return (
            (half_base, half_track),
            (-half_base, half_track),
            (-half_base, -half_track),
            (half_base, -half_track),
        )

    @property
    def spec(self) -> MobileBaseSpec:
        """The drive's component part: one unbounded wheel coordinate per driven wheel, with
        no driver until a base motor is qualified."""
        return MobileBaseSpec(
            part_id=PartId(f"{self.module_id}-drive"),
            layout=JointLayout(
                tuple(
                    JointAxis(
                        wheel,
                        CoordinateUnit.RADIAN,
                        Unbounded(),
                        actuation=UndocumentedDrive(NO_QUALIFIED_BASE_MOTOR),
                    )
                    for wheel in self.drive.wheels
                )
            ),
        )

    @property
    def module(self) -> Module:
        """The base as a module: its description, its drive component and its deck sockets."""
        return Module(
            module_id=self.module_id,
            label=self.label,
            description=self._urdf(),
            components=(body_component("drive", self.spec, RootMount(CHASSIS)),),
            sockets=tuple(
                Socket(
                    socket.name,
                    socket.interface,
                    DECK,
                    Pose((socket.xy[0], socket.xy[1], 0.0), (0.0, 0.0, socket.yaw)),
                )
                for socket in self.deck
            ),
            base=self,
        )

    def _urdf(self) -> bytes:
        """A simulation-ready description: the chassis on the floor frame, a continuous joint
        per driven wheel, fixed casters for a differential drive, and the deck on top. Every
        link carries its solid's inertia; mass is split 85% chassis, 15% wheels."""
        drive = self.drive
        robot = ET.Element("robot", {"name": str(self.module_id)})
        wheels = drive.wheels
        wheel_mass = 0.15 * self.mass_kg / len(wheels)
        chassis = Box((self.length_m, self.width_m, self.chassis_height_m))
        _link(robot, CHASSIS, chassis, 0.85 * self.mass_kg, (0.0, 0.0, self.clearance_m))
        ET.SubElement(robot, "link", {"name": DECK})
        _joint(robot, "deck_joint", "fixed", CHASSIS, DECK, (0.0, 0.0, self.deck_height_m))
        tyre = Cylinder(drive.wheel_radius_m, drive.wheel_width_m)
        side = drive.track_m / 2
        axle = drive.wheelbase_m / 2
        places = (
            {"wheel_left": (0.0, side), "wheel_right": (0.0, -side)}
            if drive.kind is DriveKind.DIFFERENTIAL
            else {
                "wheel_front_left": (axle, side),
                "wheel_front_right": (axle, -side),
                "wheel_rear_left": (-axle, side),
                "wheel_rear_right": (-axle, -side),
            }
        )
        for joint in wheels:
            link = joint.replace("wheel_", "wheel_link_")
            # A wheel's cylinder runs along its axle, the link's y axis.
            _link(robot, link, tyre, wheel_mass, (0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0))
            x, y = places[joint]
            _joint(
                robot,
                joint,
                "continuous",
                CHASSIS,
                link,
                (x, y, drive.wheel_radius_m),
                axis="0 1 0",
            )
        if drive.kind is DriveKind.DIFFERENTIAL:
            caster = drive.wheel_radius_m / 2
            for end, x in (("front", axle), ("rear", -axle)):
                link = f"caster_{end}"
                _sphere_link(robot, link, caster, 0.01 * self.mass_kg)
                _joint(robot, f"{link}_joint", "fixed", CHASSIS, link, (x, 0.0, caster))
        ET.indent(robot, space="  ")
        return ET.tostring(robot, encoding="utf-8", xml_declaration=True) + b"\n"


def _number(value: float) -> str:
    return repr(float(value)).removesuffix(".0")


def _xyz(values: tuple[float, float, float]) -> str:
    return " ".join(_number(value) for value in values)


def _inertial(
    link: ET.Element,
    mass: float,
    moments: tuple[float, float, float],
    centre: tuple[float, float, float],
) -> None:
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": _xyz(centre), "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": _number(mass)})
    ixx, iyy, izz = moments
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


def _link(
    robot: ET.Element,
    name: str,
    shape: Box | Cylinder,
    mass: float,
    base: tuple[float, float, float],
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    """A link holding one solid; a box rises from ``base``, a cylinder centres on it."""
    link = ET.SubElement(robot, "link", {"name": name})
    centre = (base[0], base[1], base[2] + shape.height / 2) if isinstance(shape, Box) else base
    moments = shape.inertia(mass)
    if rpy != (0.0, 0.0, 0.0):
        # A cylinder turned onto y swaps its axial moment from z to y.
        moments = (moments[0], moments[2], moments[1])
    _inertial(link, mass, moments, centre)
    for tag in ("visual", "collision"):
        element = ET.SubElement(link, tag)
        ET.SubElement(element, "origin", {"xyz": _xyz(centre), "rpy": _xyz(rpy)})
        ET.SubElement(element, "geometry").append(shape.element())


def _sphere_link(robot: ET.Element, name: str, radius: float, mass: float) -> None:
    link = ET.SubElement(robot, "link", {"name": name})
    moment = 2 * mass * radius * radius / 5
    _inertial(link, mass, (moment, moment, moment), (0.0, 0.0, 0.0))
    for tag in ("visual", "collision"):
        element = ET.SubElement(link, tag)
        ET.SubElement(element, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(ET.SubElement(element, "geometry"), "sphere", {"radius": _number(radius)})


def _joint(
    robot: ET.Element,
    name: str,
    kind: str,
    parent: str,
    child: str,
    xyz: tuple[float, float, float],
    *,
    axis: str | None = None,
) -> None:
    joint = ET.SubElement(robot, "joint", {"name": name, "type": kind})
    ET.SubElement(joint, "origin", {"xyz": _xyz(xyz), "rpy": "0 0 0"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    if axis is not None:
        ET.SubElement(joint, "axis", {"xyz": axis})
