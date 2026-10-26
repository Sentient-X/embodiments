//! Any registered body's actuator chain, opened from its robot instance and the station's
//! commissioned ports.
//!
//! A station is one robot instance. The registry renders that instance's drive
//! (`sx.instance-drive/v1`, [`InstanceDrive`]), and commissioning records which physical bridge
//! carries each of the body's buses ([`Commissioning`], `sx.station-commissioning/v1`): a port
//! name the body's opener declares ([`ports`]) mapped to the bridge's identity: its USB serial
//! number, or for a CAN controller on the board itself its interface name.
//!
//! [`open`] picks the opener by the body the drive was resolved for. The B601-DM keeps its own
//! (its gripper is a second chain with its own transmission). Every other body is opened from
//! its rendered bindings alone, across any number of buses: each axis names the port its bus is
//! commissioned under, [`BodyBindings::buses`] splits the native state order into one bus per
//! port, and each bus is opened by its vendor's crate from that crate's data for the body: a
//! [`DamiaoBody`] for its Damiao CAN bus (the humanoid and static upper bodies), a
//! [`FeetechBody`] for each Feetech serial bus (the SO-101 follower, the static upper body's
//! neck and grippers),
//! a [`DynamixelBody`] for a Dynamixel bus (none registered yet). The chains compose in state
//! order into one [`Chains`]: every target checked before any bus is written, a stop reaching
//! every chain and reporting each, its budget the sum of theirs. A serial servo's `LeRobot`
//! calibration is per unit and is supplied as [`ServoCalibrations`] to [`open_calibrated`];
//! [`open`] supplies none, so it refuses a body with serial servos it would open.
//!
//! A port the registry marks provisional (bindings not yet verified on the robot: the static
//! upper body's neck and grippers) is left closed: its bridge is not opened and its axes are
//! not in the body, so a station drives the verified rest. A supervised bring-up opens one
//! through [`open_with`] and [`OpenOptions::allow_provisional`], which names each port it
//! takes responsibility for.
//!
//! Each named bridge is opened once through a [`PortSource`]: two CAN port names on one serial
//! share one bridge, as the B601-DM's arm and gripper can; a serial bus shares its bridge with
//! nothing. `usb_serial` (feature `usb-serial`) opens Damiao USB2CAN bridges and serial-servo
//! adapters, `socketcan` (feature `socketcan`, Linux) `SocketCAN` interfaces, and
//! `system::SystemPorts` (either feature) looks an identity up across every transport compiled
//! in and is what a station opens through. It returns one [`Body`], an [`ActuatorChain`] the
//! station drives without naming a vendor, a joint or a limit.
//!
//! Every mismatch between the drive, the commissioning and the opener is refused before any
//! bridge is opened: commissioning for another instance, a drive for a body no opener
//! composes or whose joints are not the ones it composes, a port the opener does not declare,
//! a declared port commissioning does not name, a serial bus commissioned on a bridge another
//! port also names, a bus no vendor crate composes for the body, and a serial servo whose
//! calibration is not supplied. A bridge that is not attached is refused as it is opened. A motor that does not answer as the body binds it is found only by talking to
//! it; the vendor crate then sends every motor of that chain its disable before refusing, and
//! names any whose disable could not be sent, which may still hold torque.

use std::collections::BTreeMap;
use std::time::Duration;

use serde::Deserialize;
use sx_damiao_can::b601::{self, B601};
use sx_damiao_can::body::DamiaoBody;
use sx_damiao_can::chain::{DAMIAO_CAN, DamiaoChainError, MotorFeedback};
use sx_dynamixel_serial::DYNAMIXEL_SERIAL;
use sx_dynamixel_serial::body::DynamixelBody;
use sx_dynamixel_serial::chain::DynamixelChainError;
use sx_embodiment_drivers::can::{CanPort, SharedCanPort};
use sx_embodiment_drivers::lerobot::MotorCalibration;
use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{
    ActuatorBus, ActuatorChain, AxisReading, BodyBindings, BodyError, BoundAxis, ChainError,
    ChainFailure, Chains, ChainsError, CommandRange, CompositeError, InstanceDrive, Receipt,
    StopReport,
};
use sx_feetech_serial::FEETECH_SERIAL;
use sx_feetech_serial::body::FeetechBody;
use sx_feetech_serial::chain::FeetechChainError;
use thiserror::Error;

#[cfg(feature = "socketcan")]
pub mod socketcan;
#[cfg(any(feature = "usb-serial", feature = "socketcan", feature = "zlg"))]
pub mod system;
#[cfg(feature = "usb-serial")]
pub mod usb_serial;

/// The commissioning document's schema.
pub const COMMISSIONING_SCHEMA: &str = "sx.station-commissioning/v1";

/// A boxed bridge to one CAN bus.
pub type Port = Box<dyn CanPort>;

/// A boxed bridge to one serial-servo bus.
pub type SerialLink = Box<dyn SerialPort>;

/// What commissioning recorded for one station's robot: the instance it is, the Fleet pod
/// revision it was committed as, and which bridge carries each of the body's ports.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Commissioning {
    /// The robot instance's content id.
    pub instance: String,
    pub pod_id: String,
    pub pod_revision: u32,
    /// Port name, as the body's opener declares it, to the bridge's USB serial number (or a
    /// board CAN interface's name, which has none).
    pub ports: BTreeMap<String, String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CommissioningDocument {
    schema: String,
    instance: String,
    pod_id: String,
    pod_revision: u32,
    ports: BTreeMap<String, String>,
}

impl Commissioning {
    /// Parse a station's commissioning for `expected`, the instance the station is deployed as.
    ///
    /// # Errors
    ///
    /// Refuses anything but this schema, commissioning recorded for another instance, an empty
    /// pod, and a port or serial that is empty or carries whitespace.
    pub fn parse(document: &str, expected: &str) -> Result<Self, OpenError> {
        let parsed: CommissioningDocument = serde_json::from_str(document)
            .map_err(|error| OpenError::Document(error.to_string()))?;
        if parsed.schema != COMMISSIONING_SCHEMA {
            return Err(OpenError::Document(format!(
                "schema {} is not {COMMISSIONING_SCHEMA}",
                parsed.schema
            )));
        }
        if parsed.instance != expected {
            return Err(OpenError::Instance {
                expected: expected.to_owned(),
                found: parsed.instance,
            });
        }
        let invalid = |value: &str| value.is_empty() || value.chars().any(char::is_whitespace);
        if invalid(&parsed.pod_id)
            || parsed.pod_revision == 0
            || parsed
                .ports
                .iter()
                .any(|(name, serial)| invalid(name) || invalid(serial))
        {
            return Err(OpenError::Document(
                "a pod, its revision and every port and serial are non-empty identities".to_owned(),
            ));
        }
        Ok(Self {
            instance: parsed.instance,
            pod_id: parsed.pod_id,
            pod_revision: parsed.pod_revision,
            ports: parsed.ports,
        })
    }
}

/// Opens the bridge with a USB serial number.
pub trait PortSource {
    /// Open the CAN bridge whose USB serial number is `serial` (for `SocketCAN`, a board
    /// interface without one is named by its interface name).
    ///
    /// # Errors
    ///
    /// Returns [`OpenError::NotAttached`] when no such bridge is attached, or
    /// [`OpenError::Port`] when it cannot be opened.
    fn open(&mut self, serial: &str) -> Result<Port, OpenError>;

    /// Open the serial-servo bridge whose USB serial number is `serial`, at `baud`. A source
    /// that carries no serial buses refuses every one.
    ///
    /// # Errors
    ///
    /// Returns [`OpenError::NotAttached`] when no such bridge is attached, or
    /// [`OpenError::Port`] when it cannot be opened.
    fn open_serial(&mut self, serial: &str, baud: u32) -> Result<SerialLink, OpenError> {
        Err(OpenError::Port {
            serial: serial.to_owned(),
            reason: format!("this port source opens no serial bus (asked at {baud} baud)"),
        })
    }
}

/// Each serial servo's `LeRobot` calibration, by the port its bus is commissioned under and its
/// address on that bus. A unit's calibration is measured on the robot, so a station supplies
/// it beside the drive.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ServoCalibrations(BTreeMap<String, BTreeMap<u16, MotorCalibration>>);

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct LeRobotMotor {
    id: u16,
    drive_mode: u8,
    homing_offset: i16,
    range_min: u16,
    range_max: u16,
}

impl ServoCalibrations {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// These calibrations and `calibration` for the servo at `bus_id` on `port`.
    #[must_use]
    pub fn with(mut self, port: &str, bus_id: u16, calibration: MotorCalibration) -> Self {
        self.0
            .entry(port.to_owned())
            .or_default()
            .insert(bus_id, calibration);
        self
    }

    /// These calibrations and one bus's `LeRobot` calibration file, the JSON `LeRobot` writes
    /// per robot (`{motor: {id, drive_mode, homing_offset, range_min, range_max}}`, its
    /// `MotorCalibration`), for the servos on `port`.
    ///
    /// # Errors
    ///
    /// Refuses a document that does not parse, a drive mode other than 0 or 1, a value outside
    /// its register, and two motors at one id.
    pub fn with_lerobot(mut self, port: &str, document: &str) -> Result<Self, OpenError> {
        let motors: BTreeMap<String, LeRobotMotor> = serde_json::from_str(document)
            .map_err(|error| OpenError::Document(format!("LeRobot calibration: {error}")))?;
        let bus = self.0.entry(port.to_owned()).or_default();
        let mut seen = Vec::new();
        for (name, motor) in motors {
            if motor.drive_mode > 1 || seen.contains(&motor.id) {
                return Err(OpenError::Document(format!(
                    "LeRobot calibration: motor {name} has drive mode {} at id {}, which is \
                     not one calibration",
                    motor.drive_mode, motor.id
                )));
            }
            seen.push(motor.id);
            bus.insert(
                motor.id,
                MotorCalibration {
                    drive_mode: motor.drive_mode == 1,
                    homing_offset: motor.homing_offset,
                    range_min: motor.range_min,
                    range_max: motor.range_max,
                },
            );
        }
        Ok(self)
    }

    /// The calibration of each axis, in order, all on `port`.
    fn of(&self, port: &str, axes: &[BoundAxis]) -> Result<Vec<MotorCalibration>, OpenError> {
        axes.iter()
            .map(|axis| {
                self.0
                    .get(port)
                    .and_then(|bus| bus.get(&axis.binding.bus_id))
                    .copied()
                    .ok_or_else(|| OpenError::Uncalibrated {
                        port: port.to_owned(),
                        bus_id: axis.binding.bus_id,
                    })
            })
            .collect()
    }
}

/// A body this crate composes, by the content id its bindings carry.
#[derive(Clone, Copy, Debug)]
enum Opener {
    B601,
    /// Any other registered body, opened bus by bus from its rendered bindings.
    Buses(&'static str),
}

/// One bus of a body, as its vendor's crate opens it.
enum Part {
    Damiao(&'static DamiaoBody),
    Feetech(&'static FeetechBody, Vec<MotorCalibration>),
    Dynamixel(&'static DynamixelBody, Vec<MotorCalibration>),
}

/// How a port's bridge is opened.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Wire {
    Can,
    Serial { baud: u32 },
}

impl Opener {
    fn of(drive: &InstanceDrive) -> Result<Self, OpenError> {
        if BodyBindings::parse(b601::BINDINGS)?.id == drive.embodiment {
            return Ok(Self::B601);
        }
        let embodiment = drive.embodiment.as_str();
        DamiaoBody::of(embodiment)?
            .map(|body| body.bindings)
            .or(FeetechBody::bindings_of(embodiment)?)
            .or(DynamixelBody::of(embodiment)?.map(|body| body.bindings))
            .map(Self::Buses)
            .ok_or_else(|| OpenError::Body(drive.embodiment.clone()))
    }

    /// Each port in the order its chain is composed, with the bus its bindings put on it and
    /// whether the registry marks it provisional.
    fn buses(self, drive: &InstanceDrive) -> Result<Vec<(String, ActuatorBus, bool)>, OpenError> {
        match self {
            // Seeed's controller puts the arm and the gripper behind their own bridge; one
            // bridge carrying both is commissioned as the same serial under both names.
            Self::B601 => Ok(["arm", "gripper"]
                .into_iter()
                .map(|port| (port.to_owned(), DAMIAO_CAN, false))
                .collect()),
            Self::Buses(bindings) => Ok(BodyBindings::parse(bindings)?
                .calibrated(drive)?
                .buses()?
                .into_iter()
                .map(|bus| (bus.port, bus.kind, bus.provisional))
                .collect()),
        }
    }

    /// The provisional ports `options` leaves closed, refusing an opt-in for a port that is
    /// not one of this body's provisional ports.
    fn closed(self, drive: &InstanceDrive, options: OpenOptions) -> Result<Vec<String>, OpenError> {
        let buses = self.buses(drive)?;
        if let Some(port) = options.allow_provisional.iter().find(|port| {
            !buses
                .iter()
                .any(|(name, _, provisional)| *provisional && name == *port)
        }) {
            return Err(OpenError::NotProvisional((*port).to_owned()));
        }
        Ok(buses
            .into_iter()
            .filter(|(port, _, provisional)| {
                *provisional && !options.allow_provisional.contains(&port.as_str())
            })
            .map(|(port, _, _)| port)
            .collect())
    }

    /// Compose every bus's chain without a bridge, refusing a drive whose joints are not the
    /// ones the vendors' data compose, a bus no vendor composes, and a serial servo whose
    /// calibration is not supplied.
    fn parts(
        self,
        drive: &InstanceDrive,
        calibrations: &ServoCalibrations,
        closed: &[String],
    ) -> Result<Vec<(String, Part, Wire)>, OpenError> {
        let Self::Buses(_) = self else {
            b601::b601(drive)?;
            return Ok(Vec::new());
        };
        let embodiment = drive.embodiment.as_str();
        let unopened = |port: &str, kind: ActuatorBus| OpenError::Bus {
            port: port.to_owned(),
            bus: kind.wire(),
        };
        let parts: Vec<(String, Part, Wire)> = self
            .buses(drive)?
            .into_iter()
            .filter(|(port, _, _)| !closed.contains(port))
            .map(|(port, kind, _)| {
                let (part, wire) = if kind == DAMIAO_CAN {
                    let body = DamiaoBody::of(embodiment)?.ok_or_else(|| unopened(&port, kind))?;
                    body.axes(drive)?;
                    (Part::Damiao(body), Wire::Can)
                } else if kind == FEETECH_SERIAL {
                    let body =
                        FeetechBody::of(embodiment, &port)?.ok_or_else(|| unopened(&port, kind))?;
                    let calibration = calibrations.of(&port, &body.calibrated(drive)?)?;
                    let baud = body.baud;
                    (Part::Feetech(body, calibration), Wire::Serial { baud })
                } else if kind == DYNAMIXEL_SERIAL {
                    let body =
                        DynamixelBody::of(embodiment)?.ok_or_else(|| unopened(&port, kind))?;
                    let calibration = calibrations.of(&port, &body.calibrated(drive)?)?;
                    let baud = body.baud;
                    (Part::Dynamixel(body, calibration), Wire::Serial { baud })
                } else {
                    return Err(unopened(&port, kind));
                };
                Ok((port, part, wire))
            })
            .collect::<Result<_, OpenError>>()?;
        if parts.is_empty() {
            return Err(OpenError::Provisional(closed.to_vec()));
        }
        Ok(parts)
    }
}

/// The ports the opener for `drive`'s body declares, in the order it opens them.
///
/// # Errors
///
/// Returns [`OpenError::Body`] when no opener composes the drive's body, and why its bindings
/// cannot be split into buses.
pub fn ports(drive: &InstanceDrive) -> Result<Vec<String>, OpenError> {
    Ok(Opener::of(drive)?
        .buses(drive)?
        .into_iter()
        .map(|(port, _, _)| port)
        .collect())
}

/// How a body is opened beyond its drive, commissioning and calibrations.
#[derive(Clone, Copy, Debug, Default)]
pub struct OpenOptions<'a> {
    /// Provisional ports to open anyway, for a supervised bring-up only: a port whose
    /// bindings the registry has not verified on the robot (its ids, side, sign or zero).
    /// Every provisional port not named here is left closed, its axes outside the body.
    pub allow_provisional: &'a [&'a str],
}

/// The provisional ports opening `drive`'s body with `options` leaves closed, in state order.
///
/// # Errors
///
/// Returns [`OpenError::Body`] when no opener composes the drive's body, and
/// [`OpenError::NotProvisional`] for an opt-in naming a port that is not provisional.
pub fn closed_ports(drive: &InstanceDrive, options: OpenOptions) -> Result<Vec<String>, OpenError> {
    Opener::of(drive)?.closed(drive, options)
}

/// A chain of one bus, behind the one error and feedback type every bus shares.
pub type BusChain = Box<dyn ActuatorChain<Error = BusError, Feedback = MotorFeedback> + Send>;

/// One opened body. It is opened once per station process and lives for it, so its variants
/// are held inline rather than boxed.
#[allow(clippy::large_enum_variant)]
pub enum Body {
    B601(B601<Port, Port>),
    /// Any other body: one chain per bus, in native state order.
    Buses(Chains<BusChain>),
}

/// Open `drive`'s body on the bridges `commissioning` names, through `source`, with no serial
/// servo calibrated: a body with serial servos is refused.
///
/// # Errors
///
/// What [`open_calibrated`] refuses.
pub fn open(
    drive: &InstanceDrive,
    commissioning: &Commissioning,
    source: &mut dyn PortSource,
) -> Result<Body, OpenError> {
    open_calibrated(drive, commissioning, &ServoCalibrations::new(), source)
}

/// Open `drive`'s body on the bridges `commissioning` names, through `source`, each serial
/// servo calibrated by `calibrations`.
///
/// # Errors
///
/// Refuses, before any bridge is opened, commissioning for another instance than the drive's,
/// a body no opener composes, a bus no vendor composes for it, a serial servo with no
/// calibration, a commissioned port the opener does not declare, a declared port
/// commissioning does not name, and a serial bus on a bridge another port names; then a bridge
/// that is not attached, and any chain its vendor crate refuses to open.
pub fn open_calibrated(
    drive: &InstanceDrive,
    commissioning: &Commissioning,
    calibrations: &ServoCalibrations,
    source: &mut dyn PortSource,
) -> Result<Body, OpenError> {
    open_with(
        drive,
        commissioning,
        calibrations,
        OpenOptions::default(),
        source,
    )
}

/// [`open_calibrated`], opening also the provisional ports `options` opts in to. Every other
/// provisional port is left closed: its bridge is not opened, it needs no calibration, and its
/// axes are not in the body, so the body's axes are its verified ones in native state order.
///
/// # Errors
///
/// What [`open_calibrated`] refuses; [`OpenError::NotProvisional`] for an opt-in naming a port
/// that is not provisional, and [`OpenError::Provisional`] when every port is provisional and
/// left closed.
pub fn open_with(
    drive: &InstanceDrive,
    commissioning: &Commissioning,
    calibrations: &ServoCalibrations,
    options: OpenOptions,
    source: &mut dyn PortSource,
) -> Result<Body, OpenError> {
    if commissioning.instance != drive.instance {
        return Err(OpenError::Instance {
            expected: drive.instance.clone(),
            found: commissioning.instance.clone(),
        });
    }
    let opener = Opener::of(drive)?;
    let closed = opener.closed(drive, options)?;
    let parts = opener.parts(drive, calibrations, &closed)?;
    let declared: Vec<(String, Wire)> = match opener {
        Opener::B601 => opener
            .buses(drive)?
            .into_iter()
            .map(|(port, _, _)| (port, Wire::Can))
            .collect(),
        Opener::Buses(_) => parts
            .iter()
            .map(|(port, _, wire)| (port.clone(), *wire))
            .collect(),
    };
    let mut links = bridges(&declared, &closed, commissioning, source)?.into_iter();
    let mut next = || links.next().ok_or(OpenError::MissingPort(String::new()));
    if let Opener::B601 = opener {
        let (Link::Can(arm), Link::Can(gripper)) = (next()?, next()?) else {
            return Err(OpenError::MissingPort("arm".to_owned()));
        };
        return Ok(Body::B601(b601::open_b601(arm, gripper, drive)?));
    }
    let mut chains: Vec<BusChain> = Vec::with_capacity(parts.len());
    for (port, part, _) in parts {
        chains.push(match (part, next()?) {
            (Part::Damiao(body), Link::Can(link)) => Box::new(OnBus(body.open(link, drive)?)),
            (Part::Feetech(body, calibration), Link::Serial(link)) => {
                Box::new(OnBus(body.open(link, drive, &calibration)?))
            }
            (Part::Dynamixel(body, calibration), Link::Serial(link)) => {
                Box::new(OnBus(body.open(link, drive, &calibration)?))
            }
            _ => return Err(OpenError::MissingPort(port)),
        });
    }
    Ok(Body::Buses(Chains::new(chains)))
}

/// One opened bridge.
enum Link {
    Can(Port),
    Serial(SerialLink),
}

/// Open each declared port's bridge once, refusing before opening any a commissioned port
/// that is not declared, a declared port that is not commissioned, and a serial bus whose
/// bridge another port also names.
fn bridges(
    declared: &[(String, Wire)],
    closed: &[String],
    commissioning: &Commissioning,
    source: &mut dyn PortSource,
) -> Result<Vec<Link>, OpenError> {
    if let Some(extra) = commissioning
        .ports
        .keys()
        .find(|name| !declared.iter().any(|(port, _)| port == *name) && !closed.contains(name))
    {
        return Err(OpenError::UndeclaredPort(extra.clone()));
    }
    let serials: Vec<(&str, Wire)> = declared
        .iter()
        .map(|(name, wire)| {
            commissioning
                .ports
                .get(name)
                .map(|serial| (serial.as_str(), *wire))
                .ok_or_else(|| OpenError::MissingPort(name.clone()))
        })
        .collect::<Result<_, _>>()?;
    let shared = |serial: &str| serials.iter().filter(|(s, _)| *s == serial).count() > 1;
    if let Some((serial, _)) = serials
        .iter()
        .find(|(serial, wire)| *wire != Wire::Can && shared(serial))
    {
        return Err(OpenError::SharedBridge((*serial).to_owned()));
    }
    let mut can_bridges: BTreeMap<&str, SharedCanPort<Port>> = BTreeMap::new();
    let mut links = Vec::with_capacity(serials.len());
    for (serial, wire) in &serials {
        links.push(match wire {
            Wire::Serial { baud } => Link::Serial(source.open_serial(serial, *baud)?),
            Wire::Can if shared(serial) => {
                let port = if let Some(port) = can_bridges.get(serial) {
                    port.clone()
                } else {
                    let port = SharedCanPort::new(source.open(serial)?);
                    can_bridges.insert(serial, port.clone());
                    port
                };
                Link::Can(Box::new(port))
            }
            Wire::Can => Link::Can(source.open(serial)?),
        });
    }
    Ok(links)
}

/// A body that could not be opened.
#[derive(Debug, Error)]
pub enum OpenError {
    #[error("the commissioning document does not parse: {0}")]
    Document(String),
    #[error("commissioning of robot instance {found} cannot open instance {expected}")]
    Instance { expected: String, found: String },
    #[error("no opener composes body {0}")]
    Body(String),
    #[error("no driver here composes the {bus} bus on port {port} for this body")]
    Bus { port: String, bus: &'static str },
    #[error("the servo at bus_id {bus_id} on port {port} has no calibration supplied")]
    Uncalibrated { port: String, bus_id: u16 },
    #[error("port {0} is not one of this body's provisional ports, so there is nothing to opt in")]
    NotProvisional(String),
    #[error("every port of this body is provisional and left closed: {0:?}")]
    Provisional(Vec<String>),
    #[error("commissioning names port {0}, which this body's opener does not declare")]
    UndeclaredPort(String),
    #[error("this body's opener needs port {0}, which commissioning does not name")]
    MissingPort(String),
    #[error("bridge {0} carries a serial bus and is commissioned under more than one port")]
    SharedBridge(String),
    #[error("no bridge with USB serial {0} is attached")]
    NotAttached(String),
    #[error("more than one bridge carries identity {identity}: {}", bridges.join(", "))]
    Ambiguous {
        identity: String,
        bridges: Vec<String>,
    },
    #[error("the bridge with USB serial {serial} cannot be opened: {reason}")]
    Port { serial: String, reason: String },
    #[error(transparent)]
    Bindings(#[from] BodyError),
    #[error(transparent)]
    Chain(#[from] BusError),
}

impl From<DamiaoChainError> for OpenError {
    fn from(error: DamiaoChainError) -> Self {
        Self::Chain(error.into())
    }
}

impl From<FeetechChainError> for OpenError {
    fn from(error: FeetechChainError) -> Self {
        Self::Chain(error.into())
    }
}

impl From<DynamixelChainError> for OpenError {
    fn from(error: DynamixelChainError) -> Self {
        Self::Chain(error.into())
    }
}

/// What one bus's chain refused or failed, by its vendor's crate.
#[derive(Debug, Error)]
pub enum BusError {
    #[error(transparent)]
    Damiao(#[from] DamiaoChainError),
    #[error(transparent)]
    Feetech(#[from] FeetechChainError),
    #[error(transparent)]
    Dynamixel(#[from] DynamixelChainError),
}

impl ChainError for BusError {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::Damiao(error) => error.failure(),
            Self::Feetech(error) => error.failure(),
            Self::Dynamixel(error) => error.failure(),
        }
    }
}

/// A feedback type a bus's chain answers in, as the body's one feedback type: a Damiao motor's
/// own answer, or none from a bus whose command has no answer.
trait Answer: Sized {
    fn into_feedback(answers: Vec<Self>) -> Vec<MotorFeedback>;
}

impl Answer for MotorFeedback {
    fn into_feedback(answers: Vec<Self>) -> Vec<MotorFeedback> {
        answers
    }
}

impl Answer for () {
    fn into_feedback(_answers: Vec<Self>) -> Vec<MotorFeedback> {
        Vec::new()
    }
}

/// One vendor's chain behind the body's shared error and feedback types.
struct OnBus<C>(C);

impl<C> ActuatorChain for OnBus<C>
where
    C: ActuatorChain,
    C::Error: Into<BusError>,
    C::Feedback: Answer,
{
    type Error = BusError;
    type Feedback = MotorFeedback;

    fn axes(&self) -> &[BoundAxis] {
        self.0.axes()
    }

    fn commandable(&self) -> &[CommandRange] {
        self.0.commandable()
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt<MotorFeedback>, BusError> {
        let receipt = self.0.command(joints).map_err(Into::into)?;
        Ok(Receipt {
            evidence: receipt.evidence,
            observed: receipt.observed,
            feedback: Answer::into_feedback(receipt.feedback),
        })
    }

    fn stop(&mut self) -> StopReport {
        self.0.stop()
    }

    fn stop_budget(&self) -> Duration {
        self.0.stop_budget()
    }

    fn read_positions(&mut self) -> Vec<AxisReading> {
        self.0.read_positions()
    }

    fn last_observed(&self) -> Option<Vec<f64>> {
        self.0.last_observed()
    }

    fn command_readings(&self) -> Option<Vec<AxisReading>> {
        self.0.command_readings()
    }

    fn approach_speed_limited(&self, axis: usize) -> bool {
        self.0.approach_speed_limited(axis)
    }

    fn clear_faults(&mut self) {
        self.0.clear_faults();
    }
}

/// A command or stop one opened body refused or failed.
#[derive(Debug, Error)]
pub enum BodyChainError {
    #[error(transparent)]
    B601(#[from] CompositeError<DamiaoChainError, DamiaoChainError>),
    #[error(transparent)]
    Buses(#[from] ChainsError<BusError>),
}

impl ChainError for BodyChainError {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::B601(error) => error.failure(),
            Self::Buses(error) => error.failure(),
        }
    }
}

impl ActuatorChain for Body {
    type Error = BodyChainError;
    type Feedback = MotorFeedback;

    fn axes(&self) -> &[BoundAxis] {
        match self {
            Self::B601(chain) => chain.axes(),
            Self::Buses(chain) => chain.axes(),
        }
    }

    fn commandable(&self) -> &[CommandRange] {
        match self {
            Self::B601(chain) => chain.commandable(),
            Self::Buses(chain) => chain.commandable(),
        }
    }

    fn read_positions(&mut self) -> Vec<AxisReading> {
        match self {
            Self::B601(chain) => chain.read_positions(),
            Self::Buses(chain) => chain.read_positions(),
        }
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt<Self::Feedback>, Self::Error> {
        Ok(match self {
            Self::B601(chain) => chain.command(joints)?,
            Self::Buses(chain) => chain.command(joints)?,
        })
    }

    fn stop(&mut self) -> StopReport {
        match self {
            Self::B601(chain) => chain.stop(),
            Self::Buses(chain) => chain.stop(),
        }
    }

    fn stop_budget(&self) -> Duration {
        match self {
            Self::B601(chain) => chain.stop_budget(),
            Self::Buses(chain) => chain.stop_budget(),
        }
    }

    fn last_observed(&self) -> Option<Vec<f64>> {
        match self {
            Self::B601(chain) => chain.last_observed(),
            Self::Buses(chain) => chain.last_observed(),
        }
    }

    fn command_readings(&self) -> Option<Vec<AxisReading>> {
        match self {
            Self::B601(chain) => chain.command_readings(),
            Self::Buses(chain) => chain.command_readings(),
        }
    }

    fn approach_speed_limited(&self, axis: usize) -> bool {
        match self {
            Self::B601(chain) => chain.approach_speed_limited(axis),
            Self::Buses(chain) => chain.approach_speed_limited(axis),
        }
    }

    fn clear_faults(&mut self) {
        match self {
            Self::B601(chain) => chain.clear_faults(),
            Self::Buses(chain) => chain.clear_faults(),
        }
    }
}
