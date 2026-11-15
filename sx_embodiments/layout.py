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
    #: The DM-J10010-2EC, the humanoid's hip pitch, hip roll and knee motor: 40/150 N·m
    #: rated/peak through a 10:1 gear, driven with ``DM_CAN``'s DM10010 row ([12.5, 20, 200]).
    #: Like the Dynamixels, added untested on the bench (2026-11-15) so the humanoid's legs
    #: carry their real drive facts; every leg binding is provisional until one is driven.
    DAMIAO_DM10010 = "damiao_dm10010"
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
class SupplySpeed:
    """The vendor's output speeds on one supply, rad/s: rated (continuous, under rated torque)
    and unloaded. A Damiao joint motor's speed scales with its bus voltage, so its datasheet
    states both at 24 and at 48 V."""

    volts: float
    rated_rad_s: float
    no_load_rad_s: float

    def __post_init__(self) -> None:
        if not (math.isfinite(self.volts) and self.volts > 0.0):
            raise LayoutError("actuator", "a supply speed's volts must be positive and finite")
        if not (math.isfinite(self.rated_rad_s) and 0.0 < self.rated_rad_s <= self.no_load_rad_s):
            raise LayoutError(
                "actuator", "a supply's rated speed must be positive and at most its unloaded one"
            )
        if not math.isfinite(self.no_load_rad_s):
            raise LayoutError("actuator", "a supply's unloaded speed must be finite")


@dataclass(frozen=True, slots=True)
class ActuatorRating:
    """A tested product's ratings, as its vendor publishes them.

    ``peak_torque_nm`` is what torque guards are a share of. ``supply`` is ``None`` where no
    vendor range is recorded yet, never a guess. ``rated_speed_rad_s`` is the vendor's rated
    (continuous) output speed at its rated supply, the joint speed a description declares for
    an axis it drives (``<limit velocity>``); ``None`` where none is recorded.
    ``rated_torque_nm`` is the vendor's rated (continuous) torque, what the motor gives across
    its rated speed, so what a planned acceleration may spend
    (:mod:`sx_embodiments.calibrate.motion`); ``None`` where none is recorded.
    ``armature_kg_m2`` is the rotor's inertia reflected to the output shaft (rotor inertia
    times the internal gear ratio squared), which every joint the actuator drives moves on
    top of its links (``JointMotion.inertia``); ``None`` where none is recorded. Left out,
    a light link looks a hundred times lighter than it is (sx#3004).
    ``holding_brake`` is whether the product carries a brake that holds its shaft with the
    power off: a joint without one stays where it is unpowered only if its own friction
    outweighs gravity there, so a stop holds it under torque instead of releasing it
    (``sx_embodiment_drivers::rest``). False for every product rated today.
    ``thermal_limit_celsius`` is the hottest the vendor lets the motor run (the lower of its
    driver and coil limits), what a joint held under torque is watched against; ``None``
    where none is recorded, never a guess.
    ``supply_speeds`` are the vendor's rated and unloaded speeds at each supply it publishes
    them for; a binding that declares the rail its motor is wired to
    (``ActuatorBinding.supply_volts``) runs at that supply's (:meth:`speed_at`).
    """

    peak_torque_nm: float
    supply: SupplyRange | None = None
    rated_speed_rad_s: float | None = None
    rated_torque_nm: float | None = None
    armature_kg_m2: float | None = None
    holding_brake: bool = False
    thermal_limit_celsius: float | None = None
    supply_speeds: tuple[SupplySpeed, ...] = ()

    def speed_at(self, volts: float | None) -> SupplySpeed | None:
        """The published speeds on a ``volts`` supply; ``None`` where the vendor publishes none
        for it (the caller then keeps ``rated_speed_rad_s``, the rated supply's)."""
        if volts is None:
            return None
        return next((speed for speed in self.supply_speeds if speed.volts == volts), None)

    def __post_init__(self) -> None:
        volts = [speed.volts for speed in self.supply_speeds]
        if len(set(volts)) != len(volts):
            raise LayoutError("actuator", "a supply's speeds are published once")
        if self.supply is not None and not all(self.supply.admits(v) for v in volts):
            raise LayoutError("actuator", "a published supply speed lies outside its supply range")
        if not math.isfinite(self.peak_torque_nm) or self.peak_torque_nm <= 0.0:
            raise LayoutError("actuator", "peak torque must be a positive finite N·m")
        speed = self.rated_speed_rad_s
        if speed is not None and not (math.isfinite(speed) and speed > 0.0):
            raise LayoutError("actuator", "rated speed must be a positive finite rad/s")
        rated = self.rated_torque_nm
        if rated is not None and not (math.isfinite(rated) and 0.0 < rated <= self.peak_torque_nm):
            raise LayoutError("actuator", "rated torque must be positive and at most the peak")
        armature = self.armature_kg_m2
        if armature is not None and not (math.isfinite(armature) and armature > 0.0):
            raise LayoutError("actuator", "armature must be a positive finite kg·m²")
        limit = self.thermal_limit_celsius
        if limit is not None and not (math.isfinite(limit) and 0.0 < limit <= 200.0):
            raise LayoutError("actuator", "thermal limit must be a finite °C in (0, 200]")


def rpm(revolutions_per_minute: float) -> float:
    """An output speed in rpm, as vendors publish it, in rad/s."""
    return revolutions_per_minute * 2.0 * math.pi / 60.0


#: Damiao's 2025 product selection manual, one specification page per joint motor (rated and
#: peak torque, rated and unloaded speed at 24 and 48 V, gear ratio, weight), the source of the
#: DM8009 and DM10010 ratings below.
DAMIAO_SELECTION_MANUAL_2025: Final = (
    "https://www.worldrobotconference.com/profile/robot/download/2025/06/30/"
    "%E8%BE%BE%E5%A6%99%E7%A7%91%E6%8A%80DAMIAO%20-%202025%E5%B9%B4%E4%BA%A7%E5%93%81"
    "%E9%80%89%E5%9E%8B%E6%89%8B%E5%86%8C_20250630192546A441.pdf"
)

#: The hottest a Damiao joint motor may run, °C. The DM-J4310-2EC V1.1 user manual
#: (sharingwin.com/wp-content/uploads/2025/09/DM-J4310-2EC-V1.1-Gear-Motor-User-Manual-V1.0.pdf,
#: "Protection"): the driver shuts down at 120 °C (fault 0xB) and the motor coil's protection
#: is user-set, recommended at most 100 °C (fault 0xC); the motor leaves enable mode at either.
#: The lower, 100, applies to the hotter of the two temperatures every answer reports. The
#: DM4340 and DM8009 manuals describe the same driver's protections; none of ours has its
#: ``OT_Value`` register read yet (``fleet/station/src/motor_temperature.rs``).
DAMIAO_THERMAL_LIMIT_CELSIUS: Final = 100.0

#: Every :class:`ActuatorModel`'s rating. Drivers receive it rendered into
#: ``drivers/sx-embodiment-drivers/bindings/vocabulary.json``.
ACTUATOR_RATINGS: Final[dict[ActuatorModel, ActuatorRating]] = {
    # No product here carries a holding brake (``holding_brake`` False): the STS3215, the
    # DM-J4310/4340/8009(P)-2EC and the XL330/XM430 datasheets list none, and Damiao sells its
    # braked joints as other products. Feetech and Dynamixel publish no thermal limit our
    # command answers could be watched against (their answers carry no temperature).
    # 30 kg·cm stall at 12 V; 0.222 s per 60 deg unloaded (Feetech STS3215 datasheet), 4.72
    # rad/s. Feetech publishes no speed at rated load, so this is the unloaded figure: every
    # STS3215 port's servo approach (ServoApproach, 4 rad/s on the static upper body) is
    # below it and is what binds (sx_embodiments.calibrate.motion).
    ActuatorModel.FEETECH_STS3215: ActuatorRating(2.94, rated_speed_rad_s=math.pi / 3.0 / 0.222),
    # DM-J4310-2EC V1.1 manual (files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf): rated
    # 24 V, 3/7 N·m rated/peak, rated speed 120 rpm, 200 rpm unloaded, gear ratio 10:1.
    # Armature: Damiao's parameter table (wiki.seeedstudio.com/damiao_series, "Rotational
    # Inertia" 1.80E-05 kg·m², the rotor's) times 10², 1.8e-3 at the output; the bench
    # estimate (tools/sim_joint_tracking.py, BAM) is 2e-3.
    ActuatorModel.DAMIAO_DM4310: ActuatorRating(
        7.0,
        rated_speed_rad_s=rpm(120.0),
        rated_torque_nm=3.0,
        armature_kg_m2=1.8e-5 * 10**2,
        thermal_limit_celsius=DAMIAO_THERMAL_LIMIT_CELSIUS,
    ),
    # DM-J4340-2EC user manual: 24 V, 9/27 N·m rated/peak, rated speed 36 rpm, 52 rpm unloaded,
    # gear ratio 40:1. Armature: the same table's 2.00E-05 kg·m² rotor times 40², 0.032 at the
    # output (the bench estimate assumed 0.01; neither is measured on our motors yet).
    # Speeds per supply, the 2025 selection manual: 36 rpm rated on both versions, 52.5 rpm
    # unloaded on the 24 V version and 100 rpm on the 48 V version. One registry model covers
    # both; which one a 48 V rail carries is the unit's (the driver accepts either VMAX row).
    ActuatorModel.DAMIAO_DM4340: ActuatorRating(
        27.0,
        rated_speed_rad_s=rpm(36.0),
        rated_torque_nm=9.0,
        armature_kg_m2=2.0e-5 * 40**2,
        thermal_limit_celsius=DAMIAO_THERMAL_LIMIT_CELSIUS,
        supply_speeds=(
            SupplySpeed(24.0, rpm(36.0), rpm(52.5)),
            SupplySpeed(48.0, rpm(36.0), rpm(100.0)),
        ),
    ),
    # DM-J8009-2EC: Damiao's 2025 product selection manual (DAMIAO_SELECTION_MANUAL_2025, the
    # DM-J8009-2EC page): rated 24 V, supports 24-48 V, 20/40 N·m rated/peak, 100 rpm rated at
    # 24 V (200 at 48 V), 168 rpm unloaded at 24 V, gear ratio 9:1. The manual publishes no
    # rotor inertia for the 8009 or 10010 families: the armature is the bench estimate, 0.02
    # kg·m² (tools/sim_joint_tracking.py, BAM).
    ActuatorModel.DAMIAO_DM8009: ActuatorRating(
        40.0,
        SupplyRange(24.0, 48.0),
        rated_speed_rad_s=rpm(100.0),
        rated_torque_nm=20.0,
        armature_kg_m2=0.02,
        thermal_limit_celsius=DAMIAO_THERMAL_LIMIT_CELSIUS,
        supply_speeds=(
            SupplySpeed(24.0, rpm(100.0), rpm(168.0)),
            SupplySpeed(48.0, rpm(200.0), rpm(335.0)),
        ),
    ),
    # openelab.io/products/damiao-brushless-servo-dm-j8009p and
    # sharingwin.com/product/dmbots-dm-j8009p-2ec-robotic-actuator: rated 24 V (100 rpm
    # output), supports 24-48 V (200 rpm at 48 V), 20/40 N·m nominal/peak, 40-50 A peak.
    # Armature as the DM8009's, the bench estimate.
    ActuatorModel.DAMIAO_DM8009P: ActuatorRating(
        40.0,
        SupplyRange(24.0, 48.0),
        rated_speed_rad_s=rpm(100.0),
        rated_torque_nm=20.0,
        armature_kg_m2=0.02,
        thermal_limit_celsius=DAMIAO_THERMAL_LIMIT_CELSIUS,
        # The selection manual's DM-J8009P-2EC page: the DM8009's speeds.
        supply_speeds=(
            SupplySpeed(24.0, rpm(100.0), rpm(168.0)),
            SupplySpeed(48.0, rpm(200.0), rpm(335.0)),
        ),
    ),
    # DM-J10010-2EC: the same manual's DM-J10010-2EC page: rated 24 V, supports 24-48 V, 40/150
    # N·m rated/peak, 50 rpm rated at 24 V (100 at 48 V), 75 rpm unloaded at 24 V (150 at 48 V),
    # gear ratio 10:1, 1485 g. No rotor inertia is published and none is measured on our motors,
    # so the armature is left unrecorded, never a guess: Damiao's driver identifies it itself,
    # which is the bench's to read (robot-bench). The ``DM_CAN`` limit row's 20 rad/s and 200
    # N·m are the MIT encoding's ranges, not ratings.
    ActuatorModel.DAMIAO_DM10010: ActuatorRating(
        150.0,
        SupplyRange(24.0, 48.0),
        rated_speed_rad_s=rpm(50.0),
        rated_torque_nm=40.0,
        thermal_limit_celsius=DAMIAO_THERMAL_LIMIT_CELSIUS,
        supply_speeds=(
            SupplySpeed(24.0, rpm(50.0), rpm(75.0)),
            SupplySpeed(48.0, rpm(100.0), rpm(150.0)),
        ),
    ),
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
class CommandAnswer:
    """What a bus's answer to a position command reports beyond position, declared once per
    bus and rendered into the drivers' vocabulary (``tools/render_driver_bindings.py``), whose
    ``Reports`` each driver's answer is held to: whether it measures the motor's velocity and
    torque (``reports_motion``) and carries its temperatures (``reports_temperature``)."""

    reports_motion: bool
    reports_temperature: bool


#: Each bus's command answer: a Damiao motor answers every MIT command with its position,
#: velocity, torque and both temperatures; a Feetech or Dynamixel sync write answers nothing.
COMMAND_ANSWERS: Final[dict[ActuatorBus, CommandAnswer]] = {
    ActuatorBus.FEETECH_SERIAL: CommandAnswer(reports_motion=False, reports_temperature=False),
    ActuatorBus.DAMIAO_CAN: CommandAnswer(reports_motion=True, reports_temperature=True),
    ActuatorBus.DYNAMIXEL_SERIAL: CommandAnswer(reports_motion=False, reports_temperature=False),
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

    ``supply_volts`` is the rail the design wires this motor to, where the design states one
    (the humanoid's legs on 48 V): the motor's speeds are its rating's at that supply
    (:func:`rated_speed`). A robot's measured rail is still the commissioning's (``--rail``),
    which derates a motor over its range. Serialized only when set.
    """

    def speeds(self) -> SupplySpeed | None:
        """The motor's published speeds on its declared supply, at the motor shaft."""
        return ACTUATOR_RATINGS[self.model].speed_at(self.supply_volts)

    model: ActuatorModel
    bus: ActuatorBus
    bus_id: int
    sign: int = 1
    reduction: float = 1.0
    port: BusPort | None = None
    provisional: bool = False
    supply_volts: float | None = None

    def __post_init__(self) -> None:
        volts = self.supply_volts
        if volts is not None:
            if not (math.isfinite(volts) and volts > 0.0):
                raise LayoutError("actuator", "supply volts must be positive and finite")
            supply = ACTUATOR_RATINGS[self.model].supply
            if supply is not None and not supply.admits(volts):
                raise LayoutError(
                    "actuator", f"{self.model.value} is not specified for a {volts:g} V supply"
                )
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


@dataclass(frozen=True, slots=True)
class LinkageDrive:
    """Several tested motors drive this axis together with others through a parallel linkage
    (a humanoid's ankle pitch and roll from two motors on push rods) whose map from motor to
    joint angles is not documented.

    The motors and their addresses are facts; the map is not, so no driver opens these motors
    and no reading of one is a joint observation. What simulation takes from it is
    :func:`joint_rating`: the joint can get at most every motor's torque at once, through a lever
    ratio taken as one until the linkage is measured.
    """

    actuators: tuple[ActuatorBinding, ...]

    def __post_init__(self) -> None:
        if len(self.actuators) < 2:
            raise LayoutError("actuation", "a linkage drive has at least two motors")
        if not all(binding.provisional for binding in self.actuators):
            raise LayoutError(
                "actuation", "a linkage's motors are provisional until its map is documented"
            )


ActuationBinding = DirectDrive | IntegratedDrive | Passive | UndocumentedDrive | LinkageDrive


def rated_speed(binding: ActuatorBinding) -> float | None:
    """The joint speed a binding's motor is rated at on its declared supply (its rating's rated
    supply where it declares none), through its reduction; ``None`` where none is recorded."""
    on_supply = binding.speeds()
    speed = (
        on_supply.rated_rad_s
        if on_supply is not None
        else ACTUATOR_RATINGS[binding.model].rated_speed_rad_s
    )
    return None if speed is None else speed / binding.reduction


@dataclass(frozen=True, slots=True)
class JointRating:
    """What an axis's drive gives it in joint space: the effort a description declares as
    ``<limit effort>`` (N·m), the speed as ``<limit velocity>`` (rad/s), and the reflected rotor
    inertia a simulator adds as armature (kg·m², ``None`` where a motor's is unrecorded)."""

    effort: float
    velocity: float
    armature: float | None
    #: The unloaded speed on the declared supply (rad/s), where the vendor publishes one: what a
    #: torque-speed curve falls to zero at.
    no_load_velocity: float | None = None


def joint_rating(actuation: ActuationBinding) -> JointRating | None:
    """The joint-space rating of an axis's drive, from its motors' :data:`ACTUATOR_RATINGS`;
    ``None`` where the drive is not by tested motors or a motor has no rated speed.

    A direct drive through ``reduction`` (actuator turns per joint turn) multiplies torque and
    divides speed by it and reflects inertia by its square. A linkage sums its motors' peak
    torque and armature and takes the slowest one's speed, each at a unit lever ratio: an upper
    bound to use until the linkage's geometry is measured, not a measured joint limit.
    """
    if isinstance(actuation, DirectDrive):
        binding = actuation.actuator
        rating = ACTUATOR_RATINGS[binding.model]
        speed = rated_speed(binding)
        if speed is None:
            return None
        on_supply = binding.speeds()
        return JointRating(
            effort=rating.peak_torque_nm * binding.reduction,
            velocity=speed,
            armature=None
            if rating.armature_kg_m2 is None
            else rating.armature_kg_m2 * binding.reduction**2,
            no_load_velocity=None
            if on_supply is None
            else on_supply.no_load_rad_s / binding.reduction,
        )
    if isinstance(actuation, LinkageDrive):
        bindings = actuation.actuators
        ratings = [ACTUATOR_RATINGS[binding.model] for binding in bindings]
        speeds = [rated_speed(binding) for binding in bindings]
        if any(speed is None for speed in speeds):
            return None
        armatures = [rating.armature_kg_m2 for rating in ratings]
        unloaded = [binding.speeds() for binding in bindings]
        return JointRating(
            effort=sum(rating.peak_torque_nm for rating in ratings),
            velocity=min(speed for speed in speeds if speed is not None),
            armature=None
            if any(armature is None for armature in armatures)
            else sum(armature for armature in armatures if armature is not None),
            no_load_velocity=None
            if any(speed is None for speed in unloaded)
            else min(speed.no_load_rad_s for speed in unloaded if speed is not None),
        )
    return None


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
