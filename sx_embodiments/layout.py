"""Canonical joint layouts and the derived native body-state order."""

import math
import re
from dataclasses import dataclass
from enum import StrEnum
from typing import Final, NewType

from sx_contracts import ComponentId

from .errors import LayoutError
from .identity import PartId

#: What an axis records when nothing documents how it is driven or observed. The
#: schema-13 converter records the same absence with the same words, so a body
#: authored today and the same body carried from schema 13 are one identity.
UNDOCUMENTED_DRIVE_REASON = "drive facts are not documented"
UNDOCUMENTED_OBSERVATION_REASON = "observation facts are not documented"


class ChannelKind(StrEnum):
    ARM_JOINT = "arm_joint"
    BODY_JOINT = "body_joint"
    GRIPPER = "gripper"
    BASE = "base"


class CoordinateUnit(StrEnum):
    RADIAN = "rad"
    METER = "m"
    RADIANS_PER_SECOND = "rad/s"
    METERS_PER_SECOND = "m/s"
    NEWTON_METER = "N*m"
    UNITLESS = "unitless"


@dataclass(frozen=True, slots=True)
class Unbounded:
    """A coordinate whose physical range is intentionally not declared."""


@dataclass(frozen=True, slots=True)
class Bounds:
    lower: float
    upper: float

    def __post_init__(self) -> None:
        if (
            isinstance(self.lower, bool)
            or isinstance(self.upper, bool)
            or not math.isfinite(self.lower)
            or not math.isfinite(self.upper)
            or self.lower >= self.upper
        ):
            raise LayoutError("joint", "bounds require finite lower < upper")
        object.__setattr__(self, "lower", float(self.lower))
        object.__setattr__(self, "upper", float(self.upper))


CoordinateBounds = Unbounded | Bounds


class ActuatorModel(StrEnum):
    """The closed list of tested actuator products (byte-equal to purchasable products).

    Adding one is a first-party decision that comes with testing: it must drive on the
    bench and safe-stop and torque-off on the station. This enum is the product's "clear
    motor restrictions": a body whose every actuated axis uses a tested model can be
    driven by construction; anything else is refused at authoring.

    The Dynamixel members are the exception, added untested on the bench on 2026-10-25 so
    the static upper body's grippers can be bound once the robot is probed: the
    ``sx-dynamixel-serial`` driver is ported from ROBOTIS's DynamixelSDK and checked against
    the SDK's own packets, never against a servo. A Dynamixel model is qualified when one
    servo of it has been driven on the bench through that driver and has safe-stopped with
    its torque read back off; until then no registered body binds one.
    """

    FEETECH_STS3215 = "feetech_sts3215"
    DAMIAO_DM4310 = "damiao_dm4310"
    DAMIAO_DM4340 = "damiao_dm4340"
    DAMIAO_DM8009 = "damiao_dm8009"
    #: The DM-J8009P-2EC: the DM8009's wire limits ([12.5, 45, 54]) and torque (20 N·m
    #: nominal, 40 peak) on a crossed-roller output bearing, rated 24 V and run up to 48 V.
    #: A distinct product, so a body names which one it carries.
    DAMIAO_DM8009P = "damiao_dm8009p"
    DYNAMIXEL_XL330_M288 = "dynamixel_xl330_m288"
    DYNAMIXEL_XM430_W350 = "dynamixel_xm430_w350"


@dataclass(frozen=True, slots=True)
class SupplyRange:
    """The supply voltages an actuator is specified to run on, V, both ends inclusive.

    The bring-up compares the measured rail against it before anything moves.
    """

    min_volts: float
    max_volts: float

    def __post_init__(self) -> None:
        if not (math.isfinite(self.min_volts) and math.isfinite(self.max_volts)) or not (
            0.0 < self.min_volts <= self.max_volts
        ):
            raise LayoutError("actuator", "supply range requires 0 < min_volts <= max_volts")

    def admits(self, volts: float) -> bool:
        return self.min_volts <= volts <= self.max_volts


#: The fastest servo approach a body may declare: one turn a second (an STS3215 turns about
#: 4.7 rad/s unloaded, so a limit above this limits nothing). The drivers refuse it too.
MAX_SERVO_APPROACH_RAD_S: Final = 2.0 * math.pi


@dataclass(frozen=True, slots=True)
class ServoApproach:
    """The speed a serial bus's servos approach any goal at, on the servo itself (a Feetech
    STS3215's ``Goal_Speed`` and ``Acceleration`` registers), in rad/s and rad/s² of the servo's
    output through its binding's reduction (a RANGE_0_100 finger's joint maps its calibrated
    range onto its declared one, so its joint may move up to about 15 % faster).

    A driver setting per bus, rendered into the driver bindings beside the axes and outside the
    body's identity: the governor bounds every commanded step, and this is the servo's own
    backstop for a goal its position limits clamp. A servo resting outside those limits is pulled
    in at the slower of this and the driver's bring-up approach.
    """

    speed: float
    acceleration: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) and v > 0.0 for v in (self.speed, self.acceleration)):
            raise LayoutError("actuator", "servo approach must be positive finite rad/s, rad/s²")
        if self.speed > MAX_SERVO_APPROACH_RAD_S:
            raise LayoutError("actuator", "a servo approach above one turn a second limits nothing")


@dataclass(frozen=True, slots=True)
class ActuatorRating:
    """A tested product's ratings, as its vendor publishes them.

    ``peak_torque_nm`` is what torque guards are a share of. ``supply`` is ``None`` where no
    vendor range is recorded yet, never a guess.
    """

    peak_torque_nm: float
    supply: SupplyRange | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.peak_torque_nm) or self.peak_torque_nm <= 0.0:
            raise LayoutError("actuator", "peak torque must be a positive finite N·m")


#: Every :class:`ActuatorModel`'s rating. Drivers receive it rendered into
#: ``drivers/sx-embodiment-drivers/bindings/vocabulary.json``.
ACTUATOR_RATINGS: Final[dict[ActuatorModel, ActuatorRating]] = {
    # 30 kg·cm stall at 12 V.
    ActuatorModel.FEETECH_STS3215: ActuatorRating(2.94),
    ActuatorModel.DAMIAO_DM4310: ActuatorRating(7.0),
    ActuatorModel.DAMIAO_DM4340: ActuatorRating(27.0),
    ActuatorModel.DAMIAO_DM8009: ActuatorRating(40.0),
    # openelab.io/products/damiao-brushless-servo-dm-j8009p and
    # sharingwin.com/product/dmbots-dm-j8009p-2ec-robotic-actuator: rated 24 V (100 rpm
    # output), supports 24-48 V (200 rpm at 48 V), 20/40 N·m nominal/peak, 40-50 A peak.
    ActuatorModel.DAMIAO_DM8009P: ActuatorRating(40.0, SupplyRange(24.0, 48.0)),
    # ROBOTIS e-manual: 0.52 N·m stall at 5 V, 3.7-6 V; 4.1 N·m stall at 12 V, 10-14.8 V.
    ActuatorModel.DYNAMIXEL_XL330_M288: ActuatorRating(0.52, SupplyRange(3.7, 6.0)),
    ActuatorModel.DYNAMIXEL_XM430_W350: ActuatorRating(4.1, SupplyRange(10.0, 14.8)),
}


class ActuatorBus(StrEnum):
    """The closed list of buses tested actuators are driven over."""

    FEETECH_SERIAL = "feetech_serial"
    DAMIAO_CAN = "damiao_can"
    DYNAMIXEL_SERIAL = "dynamixel_serial"


#: The name commissioning records one physical bus's bridge under, such as ``can`` or
#: ``neck``: lowercase letters, digits, ``_`` and ``-``, starting with a letter.
BusPort = NewType("BusPort", str)

_BUS_PORT: Final = re.compile(r"[a-z][a-z0-9_-]*")

#: The port of a body's one bus of each kind, when its bindings name none.
DEFAULT_PORTS: Final[dict[ActuatorBus, BusPort]] = {
    ActuatorBus.FEETECH_SERIAL: BusPort("serial"),
    ActuatorBus.DAMIAO_CAN: BusPort("can"),
    ActuatorBus.DYNAMIXEL_SERIAL: BusPort("dynamixel"),
}


@dataclass(frozen=True, slots=True)
class ActuatorBinding:
    """How one joint axis is physically driven: a tested product on a shared bus.

    ``bus_id`` is the actuator's address on its daisy chain; the same address may recur
    across chains (each bimanual side is its own serial adapter), never within one
    layout. ``sign`` and ``reduction`` map the body's joint coordinate onto the actuator's
    own axis where the drive is not identity: ``actuator = sign * (joint - zero) *
    reduction``. The zero is measured per motor, so it is a joint calibration of the unit,
    never a fact of the design.

    ``port`` names the physical bus the actuator is wired to, as commissioning records that
    bus's bridge: a body whose actuators of one kind share one bus leaves it ``None``, the
    kind's :data:`DEFAULT_PORTS` entry, and a body with more buses names each one beyond
    that. Spelling the default is refused, so one wiring has one form, and an unnamed port
    is not serialized, so a body's identity changes only when it names one.

    ``provisional`` marks a binding not yet verified on the robot it describes (its address,
    side, drive map or zero), so no driver moves it unless a person bringing the robot up opts
    in for its port. It is serialized only when set.
    """

    model: ActuatorModel
    bus: ActuatorBus
    bus_id: int
    sign: int = 1
    reduction: float = 1.0
    port: BusPort | None = None
    provisional: bool = False

    def __post_init__(self) -> None:
        if type(self.bus_id) is not int or self.bus_id < 1:
            raise LayoutError("actuator", "bus_id must be a positive integer")
        if self.sign not in (-1, 1):
            raise LayoutError("actuator", "sign must be +1 or -1")
        if not math.isfinite(self.reduction) or self.reduction <= 0.0:
            raise LayoutError("actuator", "reduction must be a positive finite ratio")
        if type(self.provisional) is not bool:
            raise LayoutError("actuator", "provisional must be a bool")
        if self.port is not None:
            if not _BUS_PORT.fullmatch(self.port):
                raise LayoutError(
                    "actuator", "port must be lowercase letters, digits, _ or -, from a letter"
                )
            if self.port == DEFAULT_PORTS[self.bus]:
                raise LayoutError(
                    "actuator", f"port {self.port!r} is the default for its bus; leave it unset"
                )

    @property
    def bus_port(self) -> BusPort:
        """The physical bus this actuator is on: its named port, or its kind's default."""
        return self.port if self.port is not None else DEFAULT_PORTS[self.bus]


@dataclass(frozen=True, slots=True)
class ActuatorFeedback:
    """The coordinate is read from the same tested actuator that drives it."""


@dataclass(frozen=True, slots=True)
class VendorReadout:
    """A vendor interface reports this physical coordinate."""

    interface: str

    def __post_init__(self) -> None:
        if not self.interface.strip():
            raise LayoutError("observation", "vendor readout interface must not be empty")


@dataclass(frozen=True, slots=True)
class EncoderReadout:
    """A named encoder reports this physical coordinate."""

    sensor: str

    def __post_init__(self) -> None:
        if not self.sensor.strip():
            raise LayoutError("observation", "encoder sensor must not be empty")


@dataclass(frozen=True, slots=True)
class Unobserved:
    reason: str

    def __post_init__(self) -> None:
        if not self.reason.strip():
            raise LayoutError("observation", "unobserved reason must not be empty")


ObservationBinding = ActuatorFeedback | VendorReadout | EncoderReadout | Unobserved


@dataclass(frozen=True, slots=True)
class DirectDrive:
    """A tested product directly drives this axis."""

    actuator: ActuatorBinding


@dataclass(frozen=True, slots=True)
class IntegratedDrive:
    """A vendor controller drives the named group without a per-axis bus binding."""

    controller: str
    group: str

    def __post_init__(self) -> None:
        if not self.controller.strip() or not self.group.strip():
            raise LayoutError(
                "actuation", "integrated drive controller and group must not be empty"
            )


@dataclass(frozen=True, slots=True)
class Passive:
    reason: str

    def __post_init__(self) -> None:
        if not self.reason.strip():
            raise LayoutError("actuation", "passive reason must not be empty")


@dataclass(frozen=True, slots=True)
class UndocumentedDrive:
    reason: str

    def __post_init__(self) -> None:
        if not self.reason.strip():
            raise LayoutError("actuation", "undocumented drive reason must not be empty")


ActuationBinding = DirectDrive | IntegratedDrive | Passive | UndocumentedDrive


@dataclass(frozen=True, slots=True, init=False)
class JointAxis:
    """One physical coordinate, independent of how a session uses it.

    The optional positional ``actuator`` argument is retained only as an authoring
    convenience and normalizes to an explicit direct drive. Observation is an
    independent fact: a motor binding does not prove that its state is readable.
    """

    name: str
    unit: CoordinateUnit
    bounds: CoordinateBounds
    observation: ObservationBinding
    actuation: ActuationBinding

    def __init__(
        self,
        name: str,
        unit: CoordinateUnit,
        bounds: CoordinateBounds,
        actuator: ActuatorBinding | None = None,
        *,
        observation: ObservationBinding | None = None,
        actuation: ActuationBinding | None = None,
    ) -> None:
        if actuator is not None and actuation is not None:
            raise LayoutError("joint", "declare actuator or actuation, not both")
        resolved_actuation: ActuationBinding = (
            DirectDrive(actuator)
            if actuator is not None
            else actuation
            if actuation is not None
            else UndocumentedDrive(UNDOCUMENTED_DRIVE_REASON)
        )
        resolved_observation: ObservationBinding = (
            observation if observation is not None else Unobserved(UNDOCUMENTED_OBSERVATION_REASON)
        )
        if isinstance(resolved_observation, ActuatorFeedback) and not isinstance(
            resolved_actuation, DirectDrive
        ):
            raise LayoutError(
                "joint", "actuator feedback needs an actuator that drives the joint directly"
            )
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "unit", unit)
        object.__setattr__(self, "bounds", bounds)
        object.__setattr__(self, "observation", resolved_observation)
        object.__setattr__(self, "actuation", resolved_actuation)
        self.__post_init__()

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise LayoutError("joint", "axis name must not be empty")

    @property
    def actuator(self) -> ActuatorBinding | None:
        """Qualified direct actuator, or none for every other explicit drive relation."""

        if isinstance(self.actuation, DirectDrive):
            return self.actuation.actuator
        return None


@dataclass(frozen=True, slots=True)
class JointLayout:
    """A joint vector whose tuple order is the only coordinate order."""

    axes: tuple[JointAxis, ...]

    def __post_init__(self) -> None:
        names = tuple(axis.name for axis in self.axes)
        if len(set(names)) != len(names):
            raise LayoutError("joint", "axis names must be unique")
        addresses = [
            (axis.actuator.bus, axis.actuator.bus_port, axis.actuator.bus_id)
            for axis in self.axes
            if axis.actuator is not None
        ]
        if len(set(addresses)) != len(addresses):
            raise LayoutError("joint", "actuator bus addresses must be unique within a layout")

    @property
    def width(self) -> int:
        return len(self.axes)


def bounded_joint_layout(
    rows: tuple[tuple[str, CoordinateUnit, float, float], ...],
) -> JointLayout:
    """Compact source-authoring helper; the returned value has one canonical form."""

    return JointLayout(
        tuple(JointAxis(name, unit, Bounds(lower, upper)) for name, unit, lower, upper in rows)
    )


@dataclass(frozen=True, slots=True)
class StateCoordinate:
    """One coordinate in a derived state vector; tuple position is its index."""

    instance: ComponentId
    part_id: PartId
    axis: JointAxis
    kind: ChannelKind

    @property
    def joint_name(self) -> str:
        return self.axis.name

    @property
    def unit(self) -> CoordinateUnit:
        return self.axis.unit

    @property
    def lower(self) -> float | None:
        return self.axis.bounds.lower if isinstance(self.axis.bounds, Bounds) else None

    @property
    def upper(self) -> float | None:
        return self.axis.bounds.upper if isinstance(self.axis.bounds, Bounds) else None


@dataclass(frozen=True, slots=True)
class StateSpace:
    """Ordered native coordinates; dense tensors are projections of this tuple."""

    coordinates: tuple[StateCoordinate, ...]

    @property
    def width(self) -> int:
        return len(self.coordinates)

    @property
    def arm_joint_count(self) -> int:
        return sum(1 for coordinate in self.coordinates if coordinate.kind is ChannelKind.ARM_JOINT)

    @property
    def gripper_count(self) -> int:
        return sum(1 for coordinate in self.coordinates if coordinate.kind is ChannelKind.GRIPPER)

    @property
    def joint_count(self) -> int:
        return self.width - self.gripper_count

    def indices(self, kind: ChannelKind) -> tuple[int, ...]:
        return tuple(
            index for index, coordinate in enumerate(self.coordinates) if coordinate.kind is kind
        )

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(
            f"{coordinate.instance}/{coordinate.axis.name}" for coordinate in self.coordinates
        )

    def validate_widths(self, *, joint_dim: int, gripper_dim: int) -> None:
        if joint_dim != self.joint_count or gripper_dim != self.gripper_count:
            raise LayoutError(
                "state",
                f"episode widths (joints={joint_dim}, grippers={gripper_dim}) do not match "
                f"the declared layout (joints={self.joint_count}, "
                f"grippers={self.gripper_count})",
            )
