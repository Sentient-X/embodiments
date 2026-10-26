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
//! Commissioning also records each power rail's measured voltage ([`Commissioning::rails`],
//! `sx_embodiment_drivers::Rails`). An axis whose rail is over its model's supported supply is
//! opened derated (`ActuatorChain::derated`, which a `Governor` applies: lower speed, an
//! acceleration cap, a feed-forward torque cap); one more than `REFUSE_ABOVE` over is refused
//! ([`OpenError::OverRail`]) unless commissioning puts it out of service. Both lift once
//! commissioning records a rail in range.
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

use std::collections::{BTreeMap, BTreeSet};
use std::time::Duration;

use serde::Deserialize;
use sx_damiao_can::b601;
use sx_damiao_can::body::DamiaoBody;
use sx_damiao_can::chain::{DAMIAO_CAN, DamiaoAxis, DamiaoChain, DamiaoChainError, MotorFeedback};
use sx_damiao_can::observe::DamiaoObserver;
use sx_dynamixel_serial::DYNAMIXEL_SERIAL;
use sx_dynamixel_serial::body::{DynamixelBody, ServoSetup as DynamixelSetup};
use sx_dynamixel_serial::chain::{DynamixelChain, DynamixelChainError, DynamixelServo};
use sx_dynamixel_serial::observe::DynamixelObserver;
use sx_embodiment_drivers::can::{CanPort, SharedCanPort};
use sx_embodiment_drivers::lerobot::MotorCalibration;
use sx_embodiment_drivers::rail::{AxisDerate, RailCheck, Rails, derates};
use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{
    ActuatorBus, ActuatorChain, Axes, AxisReading, BodyBindings, BodyError, BoundAxis, ChainError,
    ChainFailure, Chains, ChainsError, CommandRange, Feedforward, FeedforwardLimit, InstanceDrive,
    Masked, Observe, Receipt, StopReport,
};
use sx_feetech_serial::FEETECH_SERIAL;
use sx_feetech_serial::body::{FeetechBody, ServoSetup as FeetechSetup};
use sx_feetech_serial::chain::{FeetechChain, FeetechChainError, FeetechServo};
use sx_feetech_serial::observe::FeetechObserver;
use thiserror::Error;

#[cfg(feature = "socketcan")]
pub mod socketcan;
#[cfg(any(feature = "usb-serial", feature = "socketcan", feature = "zlg"))]
pub mod system;
#[cfg(feature = "usb-serial")]
pub mod usb_serial;

/// The commissioning document's schema. `rails` is optional, so a document without it is
/// read as before: no axis derated.
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
    /// Joints whose actuator this robot does not run (a motor known faulty, removed or not yet
    /// fitted). The actuator is never addressed, so the rest opens, and the axis keeps its
    /// place in the body's vector as data ([`Masked`]): read as unavailable, observed as `NaN`,
    /// never commanded, named by `ActuatorChain::unavailable`. A port whose every axis is out
    /// of service is left closed.
    pub out_of_service: BTreeSet<String>,
    /// Each power rail's measured voltage, by selector (`*`, a port, a model, or
    /// `<port>/<model>`): what derates or refuses an axis over its model's supported supply;
    /// with the derate declared for this robot (`derate`: `max_speed` and `decided`, who
    /// decided it and why), else the rail module's automatic one.
    pub rails: Rails,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CommissioningDocument {
    schema: String,
    instance: String,
    pod_id: String,
    pod_revision: u32,
    ports: BTreeMap<String, String>,
    #[serde(default)]
    out_of_service: BTreeSet<String>,
    #[serde(default)]
    rails: BTreeMap<String, f64>,
    #[serde(default)]
    derate: Option<DeclaredDerate>,
}

/// A derate commissioning declares for this robot's over-rail axes.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct DeclaredDerate {
    max_speed: f64,
    /// Who decided it, and why.
    decided: String,
}

impl Commissioning {
    /// Parse a station's commissioning for `expected`, the instance the station is deployed as.
    ///
    /// # Errors
    ///
    /// Refuses anything but this schema, commissioning recorded for another instance, an empty
    /// pod, a port or serial that is empty or carries whitespace, and a rail whose selector or
    /// voltage is not one.
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
            || parsed.out_of_service.iter().any(|joint| invalid(joint))
        {
            return Err(OpenError::Document(
                "a pod, its revision and every port, serial and joint are non-empty identities"
                    .to_owned(),
            ));
        }
        let rails = Rails::new(parsed.rails)
            .and_then(|rails| match parsed.derate {
                Some(derate) => rails.with_derate(derate.max_speed, &derate.decided),
                None => Ok(rails),
            })
            .map_err(|error| OpenError::Document(error.to_string()))?;
        Ok(Self {
            instance: parsed.instance,
            pod_id: parsed.pod_id,
            pod_revision: parsed.pod_revision,
            ports: parsed.ports,
            out_of_service: parsed.out_of_service,
            rails,
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

    /// The calibration supplied for the servo at `bus_id` on `port`.
    #[must_use]
    pub fn of(&self, port: &str, bus_id: u16) -> Option<MotorCalibration> {
        self.0.get(port).and_then(|bus| bus.get(&bus_id)).copied()
    }

    /// These calibrations and, for every port `commissioning` names that these do not cover,
    /// the robot directory's own calibration of it, [`calibration_file`] in `directory`, where
    /// there is one: the robot instance's commissioned unit calibrations, which
    /// `sx-commission calibrate apply --robot` writes once the servos hold them. Returns the
    /// ports read from the directory with them.
    ///
    /// # Errors
    ///
    /// A calibration file that exists but cannot be read or parsed.
    pub fn with_robot_directory(
        mut self,
        directory: &std::path::Path,
        commissioning: &Commissioning,
    ) -> Result<(Self, Vec<String>), OpenError> {
        let mut read = Vec::new();
        for port in commissioning.ports.keys() {
            if self.0.contains_key(port) {
                continue;
            }
            let path = directory.join(calibration_file(port));
            let document = match std::fs::read_to_string(&path) {
                Ok(document) => document,
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => continue,
                Err(error) => {
                    return Err(OpenError::Document(format!("{}: {error}", path.display())));
                }
            };
            self = self
                .with_lerobot(port, &document)
                .map_err(|error| OpenError::Document(format!("{}: {error}", path.display())))?;
            read.push(port.clone());
        }
        Ok((self, read))
    }
}

/// The file a robot directory keeps a serial port's unit calibration in, beside its
/// `commissioning.json`: `<port>.calibration.json`, a `LeRobot` calibration keyed by joint.
#[must_use]
pub fn calibration_file(port: &str) -> String {
    format!("{port}.calibration.json")
}

/// A body this crate composes, by the content id its bindings carry.
#[derive(Clone, Copy, Debug)]
enum Opener {
    /// The reBot B601-DM: its arm and its gripper, each a Damiao chain on its own port.
    B601,
    /// Any other registered body, opened bus by bus from its rendered bindings.
    Buses(&'static str),
}

/// How a port's bridge is opened.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Wire {
    Can,
    Serial { baud: u32 },
}

/// One bus of a body, its axes resolved for one robot instance, as its vendor's crate opens
/// or observes it.
enum Part {
    Damiao(Vec<DamiaoAxis>),
    Feetech {
        baud: u32,
        axes: Vec<BoundAxis>,
        setups: Vec<FeetechSetup>,
        calibrations: Vec<Option<MotorCalibration>>,
    },
    Dynamixel {
        baud: u32,
        axes: Vec<BoundAxis>,
        setups: Vec<DynamixelSetup>,
        calibrations: Vec<Option<MotorCalibration>>,
    },
}

impl Part {
    const fn wire(&self) -> Wire {
        match self {
            Self::Damiao(_) => Wire::Can,
            Self::Feetech { baud, .. } | Self::Dynamixel { baud, .. } => {
                Wire::Serial { baud: *baud }
            }
        }
    }

    /// The axes this bus drives, in order.
    fn axes(&self) -> Vec<&BoundAxis> {
        match self {
            Self::Damiao(motors) => motors.iter().map(|motor| &motor.axis).collect(),
            Self::Feetech { axes, .. } | Self::Dynamixel { axes, .. } => axes.iter().collect(),
        }
    }

    /// The first axis whose rail is too far over its model's supported supply to enable.
    fn over_rail(&self, port: &str, rails: &Rails) -> Option<OpenError> {
        self.axes()
            .into_iter()
            .find_map(|axis| match rails.check(axis) {
                RailCheck::Refused(over) => Some(OpenError::OverRail {
                    port: port.to_owned(),
                    joint: axis.joint.clone(),
                    model: axis.binding.model.wire(),
                    rail_volts: over.rail_volts,
                    max_volts: over.max_volts,
                }),
                _ => None,
            })
    }

    /// The first servo with no calibration supplied, as the refusal naming it.
    fn uncalibrated(&self, port: &str) -> Option<OpenError> {
        let (axes, calibrations) = match self {
            Self::Damiao(_) => return None,
            Self::Feetech {
                axes, calibrations, ..
            }
            | Self::Dynamixel {
                axes, calibrations, ..
            } => (axes, calibrations),
        };
        axes.iter()
            .zip(calibrations)
            .find(|(_, calibration)| calibration.is_none())
            .map(|(axis, _)| OpenError::Uncalibrated {
                port: port.to_owned(),
                bus_id: axis.binding.bus_id,
            })
    }

    /// Open this bus's chain on `link`, each axis derated as `rails` put it; every serial
    /// servo's calibration was required first.
    fn open(self, port: &str, link: Link, rails: &Rails) -> Result<BusChain, OpenError> {
        if let Some(refused) = self.uncalibrated(port) {
            return Err(refused);
        }
        Ok(match (self, link) {
            (Self::Damiao(axes), Link::Can(link)) => {
                OnBus::boxed(DamiaoChain::open(link, axes)?, rails)
            }
            (
                Self::Feetech {
                    axes,
                    setups,
                    calibrations,
                    ..
                },
                Link::Serial(link),
            ) => {
                let servos = axes
                    .into_iter()
                    .zip(setups)
                    .zip(calibrations.into_iter().flatten())
                    .map(|((axis, setup), calibration)| FeetechServo {
                        axis,
                        norm_mode: setup.norm_mode,
                        calibration,
                        configuration: setup.configuration,
                    })
                    .collect();
                OnBus::boxed(FeetechChain::open(link, servos)?, rails)
            }
            (
                Self::Dynamixel {
                    axes,
                    setups,
                    calibrations,
                    ..
                },
                Link::Serial(link),
            ) => {
                let servos = axes
                    .into_iter()
                    .zip(setups)
                    .zip(calibrations.into_iter().flatten())
                    .map(|((axis, setup), calibration)| DynamixelServo {
                        axis,
                        norm_mode: setup.norm_mode,
                        calibration,
                        operating_mode: setup.operating_mode,
                        approach: setup.approach,
                        goal_current: setup.goal_current,
                        multi_turn: setup.multi_turn,
                    })
                    .collect();
                OnBus::boxed(DynamixelChain::open(link, servos)?, rails)
            }
            _ => return Err(OpenError::MissingPort(port.to_owned())),
        })
    }

    /// Observe this bus on `link`, a serial servo without a supplied calibration read with its
    /// registers' own.
    fn observe(self, port: &str, link: Link) -> Result<BusObserver, OpenError> {
        Ok(match (self, link) {
            (Self::Damiao(axes), Link::Can(link)) => Box::new(DamiaoObserver::open(link, axes)?),
            (
                Self::Feetech {
                    axes,
                    setups,
                    calibrations,
                    ..
                },
                Link::Serial(link),
            ) => Box::new(FeetechObserver::open(link, axes, &setups, &calibrations)?),
            (
                Self::Dynamixel {
                    axes,
                    setups,
                    calibrations,
                    ..
                },
                Link::Serial(link),
            ) => Box::new(DynamixelObserver::open(link, axes, &setups, &calibrations)?),
            _ => return Err(OpenError::MissingPort(port.to_owned())),
        })
    }
}

/// Where a bus's out-of-service axes sit in its whole vector.
struct Mask {
    whole: Vec<BoundAxis>,
    out: Vec<usize>,
}

impl Mask {
    fn of(whole: Vec<BoundAxis>, out_of_service: &BTreeSet<String>) -> Self {
        let out = whole
            .iter()
            .enumerate()
            .filter(|(_, axis)| out_of_service.contains(&axis.joint))
            .map(|(index, _)| index)
            .collect();
        Self { whole, out }
    }

    /// `chain`, which drives the bus's axes in service, as the bus's whole vector.
    fn chain(self, chain: BusChain) -> Result<BusChain, OpenError> {
        if self.out.is_empty() {
            return Ok(chain);
        }
        Ok(Box::new(
            Masked::new(chain, self.whole, &self.out)
                .map_err(DamiaoChainError::from)?
                .answering(|axis| MotorFeedback::unavailable(axis.binding.bus_id)),
        ))
    }

    /// `observer`, which reads the bus's axes in service, as the bus's whole vector.
    fn observer(self, observer: BusObserver) -> Result<BusObserver, OpenError> {
        if self.out.is_empty() {
            return Ok(observer);
        }
        Ok(Box::new(
            Masked::observing(observer, self.whole, &self.out).map_err(DamiaoChainError::from)?,
        ))
    }
}

/// Keep the axes (and what is listed beside each) whose joint is in service.
fn in_service<T>(
    items: impl IntoIterator<Item = (BoundAxis, T)>,
    out_of_service: &BTreeSet<String>,
) -> (Vec<BoundAxis>, Vec<T>) {
    items
        .into_iter()
        .filter(|(axis, _)| !out_of_service.contains(&axis.joint))
        .unzip()
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

    /// Every joint of the body.
    fn joints(self, drive: &InstanceDrive) -> Result<Vec<String>, OpenError> {
        let bindings = match self {
            Self::B601 => b601::BINDINGS,
            Self::Buses(bindings) => bindings,
        };
        Ok(BodyBindings::parse(bindings)?
            .calibrated(drive)?
            .axes
            .into_iter()
            .map(|axis| axis.joint)
            .collect())
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

    /// The Damiao motors on `port`, as `drive`'s robot drives them.
    fn damiao(self, drive: &InstanceDrive, port: &str) -> Result<Vec<DamiaoAxis>, OpenError> {
        Ok(match self {
            Self::B601 if port == "arm" => b601::b601_arm(drive)?,
            Self::B601 => vec![b601::b601_gripper(drive)?],
            Self::Buses(_) => DamiaoBody::of(&drive.embodiment)?
                .ok_or_else(|| unopened(port, DAMIAO_CAN))?
                .axes(drive)?,
        })
    }

    /// Resolve every open bus's axes for `drive` without a bridge, leaving out the joints out
    /// of service and closing a port left with none. Refuses a drive whose joints are not the
    /// ones the vendors' data compose, a bus no vendor composes, and a joint out of service
    /// that the body does not have.
    fn parts(
        self,
        drive: &InstanceDrive,
        calibrations: &ServoCalibrations,
        closed: &mut Vec<String>,
        out_of_service: &BTreeSet<String>,
    ) -> Result<Vec<(String, Part, Mask)>, OpenError> {
        let joints = self.joints(drive)?;
        if let Some(unknown) = out_of_service.iter().find(|joint| !joints.contains(joint)) {
            return Err(OpenError::OutOfService(unknown.clone()));
        }
        let embodiment = drive.embodiment.as_str();
        let mut parts = Vec::new();
        for (port, kind, _) in self.buses(drive)? {
            if closed.contains(&port) {
                continue;
            }
            let supplied = |axes: &[BoundAxis]| -> Vec<Option<MotorCalibration>> {
                axes.iter()
                    .map(|axis| calibrations.of(&port, axis.binding.bus_id))
                    .collect()
            };
            let whole: Vec<BoundAxis>;
            let part = if kind == DAMIAO_CAN {
                let all = self.damiao(drive, &port)?;
                whole = all.iter().map(|motor| motor.axis.clone()).collect();
                let motors: Vec<DamiaoAxis> = all
                    .into_iter()
                    .filter(|motor| !out_of_service.contains(&motor.axis.joint))
                    .collect();
                (!motors.is_empty()).then_some(Part::Damiao(motors))
            } else if kind == FEETECH_SERIAL {
                let body =
                    FeetechBody::of(embodiment, &port)?.ok_or_else(|| unopened(&port, kind))?;
                let calibrated = body.calibrated(drive)?;
                whole = calibrated.clone();
                let (axes, setups) =
                    in_service(calibrated.into_iter().zip(body.setups()?), out_of_service);
                (!axes.is_empty()).then(|| Part::Feetech {
                    baud: body.baud,
                    calibrations: supplied(&axes),
                    axes,
                    setups,
                })
            } else if kind == DYNAMIXEL_SERIAL {
                let body = DynamixelBody::of(embodiment)?.ok_or_else(|| unopened(&port, kind))?;
                let calibrated = body.calibrated(drive)?;
                whole = calibrated.clone();
                let (axes, setups) = in_service(
                    calibrated.into_iter().zip(body.servos.iter().copied()),
                    out_of_service,
                );
                (!axes.is_empty()).then(|| Part::Dynamixel {
                    baud: body.baud,
                    calibrations: supplied(&axes),
                    axes,
                    setups,
                })
            } else {
                return Err(unopened(&port, kind));
            };
            match part {
                Some(part) => parts.push((port, part, Mask::of(whole, out_of_service))),
                None => closed.push(port),
            }
        }
        if parts.is_empty() {
            return Err(OpenError::Provisional(closed.clone()));
        }
        Ok(parts)
    }
}

/// The refusal of a bus no vendor composes for this body.
fn unopened(port: &str, kind: ActuatorBus) -> OpenError {
    OpenError::Bus {
        port: port.to_owned(),
        bus: kind.wire(),
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

/// One opened body: one chain per bus, in native state order. Every target is checked
/// before any bus is written, and a stop reaches every chain.
pub type Body = Chains<BusChain>;

/// What a body's command or stop refused or failed.
pub type BodyChainError = ChainsError<BusError>;

/// What resolving a body for `drive` on `commissioning` settled before any bridge is opened.
struct Plan {
    closed: Vec<String>,
    parts: Vec<(String, Part, Mask)>,
}

/// Refuse a mismatch between the drive and commissioning, then resolve every open bus.
fn plan(
    drive: &InstanceDrive,
    commissioning: &Commissioning,
    calibrations: &ServoCalibrations,
    options: OpenOptions,
) -> Result<Plan, OpenError> {
    if commissioning.instance != drive.instance {
        return Err(OpenError::Instance {
            expected: drive.instance.clone(),
            found: commissioning.instance.clone(),
        });
    }
    let opener = Opener::of(drive)?;
    let declared = ports(drive)?;
    if let Some(selector) = commissioning.rails.volts().keys().find(|selector| {
        let port = selector
            .split_once('/')
            .map_or(selector.as_str(), |(port, _)| port);
        let is_model = sx_embodiment_drivers::ActuatorModel::parse(selector).is_some();
        *selector != "*" && !is_model && !declared.iter().any(|name| name == port)
    }) {
        return Err(OpenError::RailPort(selector.clone()));
    }
    let mut closed = opener.closed(drive, options)?;
    let parts = opener.parts(
        drive,
        calibrations,
        &mut closed,
        &commissioning.out_of_service,
    )?;
    Ok(Plan { closed, parts })
}

impl Plan {
    /// Open each part's bridge, after `before` has had its say on the plan.
    fn links(
        self,
        commissioning: &Commissioning,
        source: &mut dyn PortSource,
    ) -> Result<Vec<(String, Part, Mask, Link)>, OpenError> {
        let declared: Vec<(String, Wire)> = self
            .parts
            .iter()
            .map(|(port, part, _)| (port.clone(), part.wire()))
            .collect();
        let links = bridges(&declared, &self.closed, commissioning, source)?;
        Ok(self
            .parts
            .into_iter()
            .zip(links)
            .map(|((port, part, mask), link)| (port, part, mask, link))
            .collect())
    }
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
/// a body no opener composes, a bus no vendor composes for it, a joint out of service the body
/// does not have, a serial servo with no calibration, a commissioned port the opener does not
/// declare, a declared port commissioning does not name, and a serial bus on a bridge another
/// port names; then a bridge that is not attached, and any chain its vendor crate refuses to
/// open.
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
/// that is not provisional, and [`OpenError::Provisional`] when every port is left closed.
pub fn open_with(
    drive: &InstanceDrive,
    commissioning: &Commissioning,
    calibrations: &ServoCalibrations,
    options: OpenOptions,
    source: &mut dyn PortSource,
) -> Result<Body, OpenError> {
    let plan = plan(drive, commissioning, calibrations, options)?;
    if let Some(refused) = plan.parts.iter().find_map(|(port, part, _)| {
        part.uncalibrated(port)
            .or_else(|| part.over_rail(port, &commissioning.rails))
    }) {
        return Err(refused);
    }
    let mut chains: Vec<BusChain> = Vec::with_capacity(plan.parts.len());
    for (port, part, mask, link) in plan.links(commissioning, source)? {
        chains.push(mask.chain(part.open(&port, link, &commissioning.rails)?)?);
    }
    Ok(Chains::new(chains))
}

/// One bus read only, behind the one observing surface every bus shares.
pub type BusObserver = Box<dyn Observe + Send>;

/// A body read and never commanded, from the same inputs as [`open_with`]: its axes are the
/// ones [`open_with`] opens, in the same order, and each reading is in joint coordinates
/// through the same drive map and unit calibration. It does not implement
/// [`ActuatorChain`], and each vendor's observer speaks through a port that refuses any
/// request but a read, so it cannot enable, configure or move an actuator.
pub struct Observer {
    buses: Vec<BusObserver>,
    axes: Vec<BoundAxis>,
}

impl Observer {
    /// Each bus's observer, in native state order, so each can be read on its own thread: the
    /// buses are separate links, and none waits on another.
    #[must_use]
    pub fn into_buses(self) -> Vec<BusObserver> {
        self.buses
    }
}

impl Observe for Observer {
    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    /// Every bus's readings, in native state order; a bus's missing axes stand in their place.
    fn read_positions(&mut self) -> sx_embodiment_drivers::Axes<AxisReading> {
        self.buses
            .iter_mut()
            .flat_map(Observe::read_positions)
            .collect()
    }
}

/// Observe `drive`'s body on the bridges `commissioning` names, through `source`, reading and
/// never writing: Damiao motors by register reads and status refreshes only, Feetech and
/// Dynamixel servos by `PING` and `READ` only. The ports, layout and joint coordinates are
/// [`open_with`]'s for the same inputs. A serial servo with no calibration in `calibrations`
/// is read with the calibration its own registers hold, where [`open_with`] would refuse it.
/// Observing a provisional port `options` opts in to writes nothing, so it needs no
/// supervision.
///
/// # Errors
///
/// What [`open_with`] refuses before a bridge, but a missing calibration; a bridge that is not
/// attached; a servo answering as another model or holding another calibration than the one
/// supplied. A motor or servo that does not answer is not refused: it reads missing.
pub fn observe(
    drive: &InstanceDrive,
    commissioning: &Commissioning,
    calibrations: &ServoCalibrations,
    options: OpenOptions,
    source: &mut dyn PortSource,
) -> Result<Observer, OpenError> {
    let plan = plan(drive, commissioning, calibrations, options)?;
    let mut buses: Vec<BusObserver> = Vec::with_capacity(plan.parts.len());
    for (port, part, mask, link) in plan.links(commissioning, source)? {
        buses.push(mask.observer(part.observe(&port, link)?)?);
    }
    let axes = buses
        .iter()
        .flat_map(|bus| bus.axes().iter().cloned())
        .collect();
    Ok(Observer { buses, axes })
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
    #[error(
        "every port of this body is left closed, provisional or with every axis out of service: \
         {0:?}"
    )]
    Provisional(Vec<String>),
    #[error("commissioning puts joint {0} out of service, which this body does not have")]
    OutOfService(String),
    #[error(
        "joint {joint} ({model}) on port {port} is on a {rail_volts} V rail, more than {} times its {max_volts} V supported maximum; it is not enabled (put it out of service to open the rest)",
        sx_embodiment_drivers::REFUSE_ABOVE
    )]
    OverRail {
        port: String,
        joint: String,
        model: &'static str,
        rail_volts: f64,
        max_volts: f64,
    },
    #[error("commissioning records rail {0}, which names no port of this body")]
    RailPort(String),
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
trait Answer: Copy + Default {
    fn into_feedback(answers: Axes<Self>) -> Axes<MotorFeedback>;
}

impl Answer for MotorFeedback {
    fn into_feedback(answers: Axes<Self>) -> Axes<MotorFeedback> {
        answers
    }
}

impl Answer for () {
    fn into_feedback(_answers: Axes<Self>) -> Axes<MotorFeedback> {
        Axes::new()
    }
}

/// One vendor's chain behind the body's shared error and feedback types, with the axes its
/// rails derate.
struct OnBus<C>(C, Vec<AxisDerate>);

impl<C> OnBus<C>
where
    C: ActuatorChain + Send + 'static,
    C::Error: Into<BusError>,
    C::Feedback: Answer,
{
    /// `chain` as a bus of the body, its axes derated as `rails` put them.
    fn boxed(chain: C, rails: &Rails) -> BusChain {
        let derated = derates(chain.axes(), rails);
        Box::new(Self(chain, derated))
    }
}

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

    fn command_with(
        &mut self,
        joints: &[f64],
        feedforward: &[Feedforward],
    ) -> Result<Receipt<MotorFeedback>, BusError> {
        let receipt = self
            .0
            .command_with(joints, feedforward)
            .map_err(Into::into)?;
        Ok(Receipt {
            evidence: receipt.evidence,
            observed: receipt.observed,
            feedback: Answer::into_feedback(receipt.feedback),
        })
    }

    fn set_feedforward_limits(&mut self, limits: &[FeedforwardLimit]) -> Result<(), BusError> {
        self.0.set_feedforward_limits(limits).map_err(Into::into)
    }

    fn stop(&mut self) -> StopReport {
        self.0.stop()
    }

    fn stop_budget(&self) -> Duration {
        self.0.stop_budget()
    }

    fn read_positions(&mut self) -> sx_embodiment_drivers::Axes<AxisReading> {
        self.0.read_positions()
    }

    fn last_observed(&self) -> Option<sx_embodiment_drivers::Axes<f64>> {
        self.0.last_observed()
    }

    fn command_readings(&self) -> Option<sx_embodiment_drivers::Axes<AxisReading>> {
        self.0.command_readings()
    }

    fn derated(&self) -> Vec<AxisDerate> {
        self.1.clone()
    }

    fn clear_faults(&mut self) {
        self.0.clear_faults();
    }
}
