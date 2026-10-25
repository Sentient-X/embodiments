//! Each declared port matched to the one bridge its units answer on.
//!
//! A bus fits a port when every unit the port declares answers on it as its model: a Feetech
//! servo with the model's number, a Damiao motor whose position and torque ranges are the
//! model's row. Of the buses that fit, one whose answering ids are exactly the port's wins; a
//! bus that also carries other ids fits only when no exact one does, so a gripper port bound to
//! ids 1 and 2 is the two-servo bus, not the neck's bus that also answers 1 and 2, and a bus
//! that is exactly another port's is never taken for a port whose ids it also carries. Zero or
//! several fitting buses refuse the port, naming every candidate; a bus where every id answers
//! but one is the wrong model refuses it as that.
//!
//! Every port is matched before any result is returned, so one run names every problem.

use std::collections::BTreeSet;

use sx_damiao_can::chain::motor_type;
use sx_embodiment_drivers::ActuatorModel;
use sx_feetech_serial::scan::ServoScan;
use sx_feetech_serial::{FEETECH_STS3215, STS3215_MODEL_NUMBER};
use thiserror::Error;

use crate::declared::{BusKind, DeclaredBody, DeclaredPort, DeclaredUnit};
use crate::probe::Probe;

/// One port matched to its bridge.
#[derive(Clone, Debug, PartialEq)]
pub struct PortMatch {
    pub port: String,
    /// What commissioning records: the bridge's USB serial number (a board CAN interface's
    /// own name).
    pub identity: String,
    /// The tty or interface the units answered on.
    pub channel: String,
    /// For a Feetech port, each unit's joint with the servo as it read, in declared order.
    pub servos: Vec<(String, ServoScan)>,
}

/// A bus some of a port's ids answered on.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Candidate {
    pub identity: String,
    pub channel: String,
    pub answered: Vec<u16>,
}

impl std::fmt::Display for Candidate {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            formatter,
            "{} ({}) answering {:?}",
            self.identity, self.channel, self.answered
        )
    }
}

fn list(candidates: &[Candidate]) -> String {
    if candidates.is_empty() {
        return "none".to_owned();
    }
    candidates
        .iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join("; ")
}

/// Why a port has no bridge.
#[derive(Clone, Debug, Error, Eq, PartialEq)]
pub enum PortError {
    #[error("port {port}: no bus answers every id {ids:?}; partial: {}", list(.partial))]
    Missing {
        port: String,
        ids: Vec<u16>,
        partial: Vec<Candidate>,
    },
    #[error("port {port}: {} buses answer as it does: {}", .candidates.len(), list(.candidates))]
    Ambiguous {
        port: String,
        candidates: Vec<Candidate>,
    },
    #[error(
        "port {port}: on {identity} ({channel}) {joint} at address {bus_id} is not a {expected}: {found}"
    )]
    WrongModel {
        port: String,
        identity: String,
        channel: String,
        joint: String,
        bus_id: u16,
        expected: String,
        found: String,
    },
    #[error("port {port}: {joint} binds model {model}, which no scan can recognise")]
    Unqualified {
        port: String,
        joint: String,
        model: String,
    },
}

/// One scanned bus seen from a port: what answers on it, and whether each unit is its model.
struct Bus {
    candidate: Candidate,
    /// Every id that answered, the port's or not.
    all: BTreeSet<u16>,
    /// The first declared unit answering as another model, and what it answered as.
    wrong: Option<(DeclaredUnit, String)>,
    servos: Vec<(String, ServoScan)>,
}

fn feetech_model(unit: &DeclaredUnit) -> Option<u16> {
    (unit.model == FEETECH_STS3215.wire()).then_some(STS3215_MODEL_NUMBER)
}

fn damiao_model(unit: &DeclaredUnit) -> Option<sx_damiao_can::dm_can::DmMotorType> {
    ActuatorModel::ALL
        .into_iter()
        .find(|model| model.wire() == unit.model)
        .and_then(motor_type)
}

fn feetech_buses(port: &DeclaredPort, probe: &Probe) -> Vec<Bus> {
    probe
        .feetech
        .iter()
        .filter_map(|bus| {
            let scan = bus.scan.as_ref().ok()?;
            let mut wrong = None;
            let mut answered = Vec::new();
            let mut servos = Vec::new();
            for unit in &port.units {
                let Some(servo) = u8::try_from(unit.bus_id).ok().and_then(|id| scan.servo(id))
                else {
                    continue;
                };
                answered.push(unit.bus_id);
                servos.push((unit.joint.clone(), *servo));
                if Some(servo.model) != feetech_model(unit) && wrong.is_none() {
                    wrong = Some((unit.clone(), format!("model number {}", servo.model)));
                }
            }
            Some(Bus {
                candidate: Candidate {
                    identity: bus.identity.clone(),
                    channel: bus.tty.clone(),
                    answered,
                },
                all: scan
                    .servos
                    .iter()
                    .map(|servo| u16::from(servo.id))
                    .collect(),
                wrong,
                servos,
            })
        })
        .collect()
}

fn damiao_buses(port: &DeclaredPort, probe: &Probe) -> Vec<Bus> {
    probe
        .damiao
        .iter()
        .filter_map(|bus| {
            let scan = bus.scan.as_ref().ok()?;
            let mut wrong = None;
            let mut answered = Vec::new();
            for unit in &port.units {
                let Some(motor) = scan.motor(unit.bus_id) else {
                    continue;
                };
                answered.push(unit.bus_id);
                let fits = damiao_model(unit).is_some_and(|model| motor.fits(model));
                if !fits && wrong.is_none() {
                    wrong = Some((
                        unit.clone(),
                        format!(
                            "PMAX {:?} VMAX {:?} TMAX {:?}",
                            motor.p_max, motor.v_max, motor.t_max
                        ),
                    ));
                }
            }
            Some(Bus {
                candidate: Candidate {
                    identity: bus.identity.clone(),
                    channel: bus.interface.clone(),
                    answered,
                },
                all: scan.motors.iter().map(|motor| motor.slave_id).collect(),
                wrong,
                servos: Vec::new(),
            })
        })
        .collect()
}

fn buses(port: &DeclaredPort, probe: &Probe) -> Vec<Bus> {
    match port.bus {
        BusKind::Feetech => feetech_buses(port, probe),
        BusKind::Damiao => damiao_buses(port, probe),
    }
}

/// The channels `port` fits exactly: every id it declares answers as its model, and no other.
fn exact(port: &DeclaredPort, probe: &Probe) -> BTreeSet<String> {
    let ids: BTreeSet<u16> = port.units.iter().map(|unit| unit.bus_id).collect();
    buses(port, probe)
        .into_iter()
        .filter(|bus| bus.wrong.is_none() && bus.all == ids)
        .map(|bus| bus.candidate.channel)
        .collect()
}

/// Match `port`, never to a bus carrying more than its ids when that bus is exactly another
/// port's (`claimed`): a gripper's ids answering on the neck's bus are not the gripper.
fn match_port(
    port: &DeclaredPort,
    probe: &Probe,
    claimed: &BTreeSet<String>,
) -> Result<PortMatch, Box<PortError>> {
    let qualified = |unit: &DeclaredUnit| match port.bus {
        BusKind::Feetech => feetech_model(unit).is_some(),
        BusKind::Damiao => damiao_model(unit).is_some(),
    };
    if let Some(unit) = port.units.iter().find(|unit| !qualified(unit)) {
        return Err(Box::new(PortError::Unqualified {
            port: port.name.clone(),
            joint: unit.joint.clone(),
            model: unit.model.clone(),
        }));
    }
    let ids: BTreeSet<u16> = port.units.iter().map(|unit| unit.bus_id).collect();
    let (complete, partial): (Vec<Bus>, Vec<Bus>) = buses(port, probe)
        .into_iter()
        .filter(|bus| !bus.candidate.answered.is_empty())
        .partition(|bus| {
            bus.candidate.answered.len() == ids.len()
                && (bus.all == ids || !claimed.contains(&bus.candidate.channel))
        });
    let (fitting, wrong): (Vec<Bus>, Vec<Bus>) =
        complete.into_iter().partition(|bus| bus.wrong.is_none());
    let exact: Vec<&Bus> = fitting.iter().filter(|bus| bus.all == ids).collect();
    let chosen: Vec<&Bus> = if exact.is_empty() {
        fitting.iter().collect()
    } else {
        exact
    };
    match chosen.as_slice() {
        [bus] => Ok(PortMatch {
            port: port.name.clone(),
            identity: bus.candidate.identity.clone(),
            channel: bus.candidate.channel.clone(),
            servos: bus.servos.clone(),
        }),
        [] => {
            if let Some(Bus {
                candidate,
                wrong: Some((unit, found)),
                ..
            }) = wrong.into_iter().next()
            {
                return Err(Box::new(PortError::WrongModel {
                    port: port.name.clone(),
                    identity: candidate.identity,
                    channel: candidate.channel,
                    joint: unit.joint,
                    bus_id: unit.bus_id,
                    expected: unit.model,
                    found,
                }));
            }
            Err(Box::new(PortError::Missing {
                port: port.name.clone(),
                ids: ids.into_iter().collect(),
                partial: partial.into_iter().map(|bus| bus.candidate).collect(),
            }))
        }
        several => Err(Box::new(PortError::Ambiguous {
            port: port.name.clone(),
            candidates: several.iter().map(|bus| bus.candidate.clone()).collect(),
        })),
    }
}

/// Match every port `body` declares to its bridge among what `probe` found.
///
/// # Errors
///
/// Returns every port that matched no bridge, or several, or a bridge of the wrong models.
pub fn commission(body: &DeclaredBody, probe: &Probe) -> Result<Vec<PortMatch>, Vec<PortError>> {
    let exact: Vec<BTreeSet<String>> = body.ports.iter().map(|port| exact(port, probe)).collect();
    let (matched, failed): (Vec<_>, Vec<_>) = body
        .ports
        .iter()
        .enumerate()
        .map(|(index, port)| {
            let claimed: BTreeSet<String> = exact
                .iter()
                .enumerate()
                .filter(|(other, _)| *other != index)
                .flat_map(|(_, channels)| channels.iter().cloned())
                .collect();
            match_port(port, probe, &claimed)
        })
        .partition(Result::is_ok);
    if failed.is_empty() {
        Ok(matched.into_iter().filter_map(Result::ok).collect())
    } else {
        Err(failed
            .into_iter()
            .filter_map(Result::err)
            .map(|error| *error)
            .collect())
    }
}
