//! Any registered body's actuator chain, opened from its robot instance and the station's
//! commissioned ports.
//!
//! A station is one robot instance. The registry renders that instance's drive
//! (`sx.instance-drive/v1`, [`InstanceDrive`]), and commissioning records which physical bridge
//! carries each of the body's buses ([`Commissioning`], `sx.station-commissioning/v1`): a port
//! name the body's opener declares ([`ports`]) mapped to the bridge's USB serial number. [`open`]
//! picks the opener by the body the drive was resolved for, opens each named bridge once
//! through a [`PortSource`] (two port names on one serial share one bridge, as the B601-DM's
//! arm and gripper can), and returns one [`Body`], an [`ActuatorChain`] the station drives
//! without naming a vendor, a joint or a limit.
//!
//! Every mismatch is refused before a byte reaches a bus: commissioning for another instance,
//! a drive for a body no opener composes, a port the opener does not declare, a declared port
//! commissioning does not name, and a bridge that is not attached.

use std::collections::BTreeMap;
use std::time::Duration;

use serde::Deserialize;
use sx_damiao_can::b601::{self, B601};
use sx_damiao_can::chain::{DamiaoChain, DamiaoChainError, MotorFeedback};
use sx_damiao_can::humanoid;
use sx_embodiment_drivers::can::{CanPort, SharedCanPort};
use sx_embodiment_drivers::{
    ActuatorChain, BodyBindings, BodyError, BoundAxis, ChainError, ChainFailure, CommandRange,
    CompositeError, InstanceDrive, Receipt, StopReport,
};
use thiserror::Error;

#[cfg(feature = "usb-serial")]
pub mod usb_serial;

/// The commissioning document's schema.
pub const COMMISSIONING_SCHEMA: &str = "sx.station-commissioning/v1";

/// A boxed bridge to one bus.
pub type Port = Box<dyn CanPort>;

/// What commissioning recorded for one station's robot: the instance it is, the Fleet pod
/// revision it was committed as, and which bridge carries each of the body's ports.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Commissioning {
    /// The robot instance's content id.
    pub instance: String,
    pub pod_id: String,
    pub pod_revision: u32,
    /// Port name, as the body's opener declares it, to the bridge's USB serial number.
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
    /// Open the bridge whose USB serial number is `serial`.
    ///
    /// # Errors
    ///
    /// Returns [`OpenError::NotAttached`] when no such bridge is attached, or
    /// [`OpenError::Port`] when it cannot be opened.
    fn open(&mut self, serial: &str) -> Result<Port, OpenError>;
}

/// A body this crate composes, by the registry name its bindings carry.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Opener {
    B601,
    HumanoidUpperBody,
}

impl Opener {
    const ALL: [Self; 2] = [Self::B601, Self::HumanoidUpperBody];

    const fn bindings(self) -> &'static str {
        match self {
            Self::B601 => b601::BINDINGS,
            Self::HumanoidUpperBody => humanoid::BINDINGS,
        }
    }

    const fn ports(self) -> &'static [&'static str] {
        match self {
            // Seeed's controller puts the arm and the gripper behind their own bridge; one
            // bridge carrying both is commissioned as the same serial under both names.
            Self::B601 => &["arm", "gripper"],
            Self::HumanoidUpperBody => &["can"],
        }
    }

    fn of(drive: &InstanceDrive) -> Result<Self, OpenError> {
        for opener in Self::ALL {
            if BodyBindings::parse(opener.bindings())?.id == drive.embodiment {
                return Ok(opener);
            }
        }
        Err(OpenError::Body(drive.embodiment.clone()))
    }
}

/// The ports the opener for `drive`'s body declares, in the order it opens them.
///
/// # Errors
///
/// Returns [`OpenError::Body`] when no opener composes the drive's body.
pub fn ports(drive: &InstanceDrive) -> Result<&'static [&'static str], OpenError> {
    Ok(Opener::of(drive)?.ports())
}

/// One opened body. It is opened once per station process and lives for it, so its variants
/// are held inline rather than boxed.
#[allow(clippy::large_enum_variant)]
pub enum Body {
    B601(B601<Port, Port>),
    HumanoidUpperBody(DamiaoChain<Port>),
}

/// Open `drive`'s body on the bridges `commissioning` names, through `source`.
///
/// # Errors
///
/// Refuses commissioning for another instance than the drive's, a body no opener composes, a
/// commissioned port the opener does not declare, a declared port commissioning does not name,
/// a bridge that is not attached, and any chain its vendor crate refuses to open.
pub fn open(
    drive: &InstanceDrive,
    commissioning: &Commissioning,
    source: &mut dyn PortSource,
) -> Result<Body, OpenError> {
    if commissioning.instance != drive.instance {
        return Err(OpenError::Instance {
            expected: drive.instance.clone(),
            found: commissioning.instance.clone(),
        });
    }
    let opener = Opener::of(drive)?;
    let declared = opener.ports();
    if let Some(extra) = commissioning
        .ports
        .keys()
        .find(|name| !declared.contains(&name.as_str()))
    {
        return Err(OpenError::UndeclaredPort(extra.clone()));
    }
    let serials: Vec<&str> = declared
        .iter()
        .map(|name| {
            commissioning
                .ports
                .get(*name)
                .map(String::as_str)
                .ok_or_else(|| OpenError::MissingPort((*name).to_owned()))
        })
        .collect::<Result<_, _>>()?;
    let mut bridges: BTreeMap<&str, SharedCanPort<Port>> = BTreeMap::new();
    let mut taken: Vec<Port> = Vec::with_capacity(serials.len());
    let shared = |serial: &str| serials.iter().filter(|s| **s == serial).count() > 1;
    for serial in &serials {
        if shared(serial) {
            let port = if let Some(port) = bridges.get(serial) {
                port.clone()
            } else {
                let port = SharedCanPort::new(source.open(serial)?);
                bridges.insert(serial, port.clone());
                port
            };
            taken.push(Box::new(port));
        } else {
            taken.push(source.open(serial)?);
        }
    }
    let mut taken = taken.into_iter();
    let mut next = || taken.next().ok_or(OpenError::MissingPort(String::new()));
    Ok(match opener {
        Opener::B601 => Body::B601(b601::open_b601(next()?, next()?, drive)?),
        Opener::HumanoidUpperBody => {
            Body::HumanoidUpperBody(humanoid::open_humanoid_upper_body(next()?, drive)?)
        }
    })
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
    #[error("commissioning names port {0}, which this body's opener does not declare")]
    UndeclaredPort(String),
    #[error("this body's opener needs port {0}, which commissioning does not name")]
    MissingPort(String),
    #[error("no bridge with USB serial {0} is attached")]
    NotAttached(String),
    #[error("the bridge with USB serial {serial} cannot be opened: {reason}")]
    Port { serial: String, reason: String },
    #[error(transparent)]
    Bindings(#[from] BodyError),
    #[error(transparent)]
    Chain(#[from] DamiaoChainError),
}

/// A command or stop one opened body refused or failed.
#[derive(Debug, Error)]
pub enum BodyChainError {
    #[error(transparent)]
    B601(#[from] CompositeError<DamiaoChainError, DamiaoChainError>),
    #[error(transparent)]
    Damiao(#[from] DamiaoChainError),
}

impl ChainError for BodyChainError {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::B601(error) => error.failure(),
            Self::Damiao(error) => error.failure(),
        }
    }
}

impl ActuatorChain for Body {
    type Error = BodyChainError;
    type Feedback = MotorFeedback;

    fn axes(&self) -> &[BoundAxis] {
        match self {
            Self::B601(chain) => chain.axes(),
            Self::HumanoidUpperBody(chain) => chain.axes(),
        }
    }

    fn commandable(&self) -> &[CommandRange] {
        match self {
            Self::B601(chain) => chain.commandable(),
            Self::HumanoidUpperBody(chain) => chain.commandable(),
        }
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt<Self::Feedback>, Self::Error> {
        Ok(match self {
            Self::B601(chain) => chain.command(joints)?,
            Self::HumanoidUpperBody(chain) => chain.command(joints)?,
        })
    }

    fn stop(&mut self) -> StopReport {
        match self {
            Self::B601(chain) => chain.stop(),
            Self::HumanoidUpperBody(chain) => chain.stop(),
        }
    }

    fn stop_budget(&self) -> Duration {
        match self {
            Self::B601(chain) => chain.stop_budget(),
            Self::HumanoidUpperBody(chain) => chain.stop_budget(),
        }
    }

    fn clear_faults(&mut self) {
        match self {
            Self::B601(chain) => chain.clear_faults(),
            Self::HumanoidUpperBody(chain) => chain.clear_faults(),
        }
    }
}
