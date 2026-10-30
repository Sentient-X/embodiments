//! A body's bound axes as the registry renders them: the one source a driver composes from.
//!
//! `tools/render_driver_bindings.py` renders, for each body a driver crate composes, its
//! `robot.state` coordinates in native state order with each one's bounds and actuator binding.
//! A driver crate embeds that document and parses it here, so no joint bound, address, model,
//! sign or reduction is spelled in Rust. What a crate adds is its vendor's controller
//! configuration, keyed by joint name.
//!
//! Every axis names the port its bus is commissioned under. A body on several buses is opened
//! port by port: [`BodyBindings::buses`] splits the native state order into one [`Bus`] per
//! port, each a contiguous run of one bus kind, and [`BodyBindings::on`] keeps one kind's axes
//! for a crate that drives only that kind.
//!
//! A body no driver crate composes (a composition that repeats a part, the bimanual SO-101's
//! two arms) names, for each joint, its twin (`of`): the same part's same joint, on the same
//! drive, in a body a crate does compose. [`BodyBindings::configured_by`] answers, for each
//! bus, the body whose controller configuration drives it (the twin, or the body itself) with
//! the bus's axes named as that body's, so every body opens one way.

use std::collections::BTreeMap;

use serde::Deserialize;
use thiserror::Error;

use crate::approach::ApproachLimit;
use crate::binding::{
    ActuatorBinding, ActuatorBus, ActuatorModel, AxisMotion, BindingError, BoundAxis,
    UNCALIBRATED_DIRECTION, UNCALIBRATED_ZERO, validate_chain,
};

/// One registered body's bound axes, in native state order, uncalibrated.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyBindings {
    /// The registry's name for the body.
    pub embodiment: String,
    /// The body's content id when it was rendered.
    pub id: String,
    /// Earlier ids of the body that drive exactly as this one (the registry's
    /// `sx_embodiments.successor`): a robot still assigned one is this body.
    pub supersedes: Vec<String>,
    pub axes: Vec<BoundAxis>,
    /// The ports whose bindings the registry marks provisional (`ActuatorBinding.provisional`):
    /// not yet verified on the robot, so not driven unless a supervised bring-up opts in.
    pub provisional: Vec<String>,
    /// Each joint's twin in a body a driver crate composes (`of`), by joint; empty for a body a
    /// driver crate composes itself.
    pub twins: BTreeMap<String, Twin>,
    /// Per serial-servo port, the speed its servos approach any goal at on the servo itself
    /// (the registry's `ServoApproach`, a driver setting outside the body's identity); a port
    /// without one keeps the servo's own default.
    pub approach: BTreeMap<String, ApproachLimit>,
}

/// A joint's twin: `joint` on `port` of the body named `body`, driven exactly as this joint is.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Twin {
    pub body: String,
    pub port: String,
    pub joint: String,
}

/// The fastest servo approach limit a registry may declare, rad/s: one turn a second. An
/// STS3215 turns about 4.7 rad/s unloaded, so a limit above this limits nothing.
pub const MAX_APPROACH_SPEED: f64 = std::f64::consts::TAU;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Approach {
    speed: f64,
    acceleration: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    embodiment: String,
    id: String,
    axes: Vec<Axis>,
    #[serde(default)]
    approach: BTreeMap<String, Approach>,
    #[serde(default)]
    supersedes: Vec<String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Axis {
    joint: String,
    lower: f64,
    upper: f64,
    model: String,
    bus: String,
    port: String,
    #[serde(default)]
    provisional: bool,
    bus_id: u16,
    sign: i8,
    reduction: f64,
    /// The chain's declared home for this joint; absent where its part declares none.
    #[serde(default)]
    home: Option<f64>,
    #[serde(default)]
    of: Option<Twin>,
    /// The fastest the hardware moves this joint (`sx_embodiments.calibrate.motion`), and the
    /// most inertia it moves.
    speed: f64,
    acceleration: f64,
    #[serde(default)]
    inertia: f64,
}

impl Axis {
    /// The joint's rendered motion, refused unless both are positive and finite.
    fn motion(&self) -> Result<AxisMotion, BodyError> {
        let positive = |value: f64| value.is_finite() && value > 0.0;
        if positive(self.speed)
            && positive(self.acceleration)
            && self.inertia.is_finite()
            && self.inertia >= 0.0
        {
            Ok(AxisMotion {
                speed: self.speed,
                acceleration: self.acceleration,
                inertia: self.inertia,
            })
        } else {
            Err(BodyError::Motion {
                joint: self.joint.clone(),
            })
        }
    }
}

#[derive(Debug, Error, PartialEq)]
pub enum BodyError {
    #[error("the bindings document does not parse: {0}")]
    Document(String),
    #[error("joint {joint} names actuator model {model}, which the registry does not carry")]
    Model { joint: String, model: String },
    #[error("joint {joint} names bus {bus}, which the registry does not carry")]
    Bus { joint: String, bus: String },
    #[error(transparent)]
    Binding(#[from] BindingError),
    #[error("a drive of body {found} cannot calibrate {embodiment}")]
    Instance { embodiment: String, found: String },
    #[error("a drive resolved for robot instance {found} cannot calibrate instance {expected}")]
    Unit { expected: String, found: String },
    #[error("{embodiment} has no joint {joint}")]
    MissingJoint { embodiment: String, joint: String },
    #[error("{embodiment}'s port {port} carries both {first} and {second} actuators")]
    MixedPort {
        embodiment: String,
        port: String,
        first: &'static str,
        second: &'static str,
    },
    #[error(
        "{embodiment}'s port {port} returns after another port in native state order, so its \
         chains cannot be composed in that order"
    )]
    InterleavedPort { embodiment: String, port: String },
    #[error("{embodiment}'s axes for this driver are on ports {ports:?}; it drives one bus")]
    Buses {
        embodiment: String,
        ports: Vec<String>,
    },
    #[error(
        "{embodiment}'s approach for port {port} names no serial-servo port of its axes, or is \
         not a positive finite speed (at most one turn a second) and acceleration"
    )]
    Approach { embodiment: String, port: String },
    #[error(
        "{embodiment}'s port {port} declares no servo approach limit, and its servos are not \
         opened at their full speed"
    )]
    NoApproach { embodiment: String, port: String },
    #[error("joint {joint}'s speed and acceleration must be positive and finite")]
    Motion { joint: String },
    #[error("{embodiment}'s port {port} has both provisional and verified bindings")]
    MixedProvisional { embodiment: String, port: String },
    #[error(
        "{embodiment}'s port {port} is not one run of joints that all repeat one port of one \
         body a driver composes"
    )]
    Twins { embodiment: String, port: String },
    #[error("{embodiment}'s joints {found:?} are not the {expected:?} this driver composes")]
    Joints {
        embodiment: String,
        found: Vec<String>,
        expected: Vec<String>,
    },
}

/// One port's axes: a contiguous run of the native state order on one bus.
#[derive(Clone, Debug, PartialEq)]
pub struct Bus {
    /// The name commissioning records this bus's bridge under.
    pub port: String,
    pub kind: ActuatorBus,
    pub axes: Vec<BoundAxis>,
    /// Whether the registry marks this bus's bindings provisional.
    pub provisional: bool,
}

impl BodyBindings {
    /// Parse a rendered document, refusing any member the registry's vocabulary lacks and any
    /// chain the registry would refuse.
    ///
    /// # Errors
    ///
    /// Returns the first unparseable or invalid fact.
    pub fn parse(document: &str) -> Result<Self, BodyError> {
        let Document {
            embodiment,
            id,
            axes,
            approach,
            supersedes,
        } = serde_json::from_str(document)
            .map_err(|error| BodyError::Document(error.to_string()))?;
        let mut provisional: Vec<String> = Vec::new();
        let mut verified: Vec<String> = Vec::new();
        for axis in &axes {
            let (marks, others) = if axis.provisional {
                (&mut provisional, &verified)
            } else {
                (&mut verified, &provisional)
            };
            if others.contains(&axis.port) {
                return Err(BodyError::MixedProvisional {
                    embodiment: embodiment.clone(),
                    port: axis.port.clone(),
                });
            }
            if !marks.contains(&axis.port) {
                marks.push(axis.port.clone());
            }
        }
        let twins: BTreeMap<String, Twin> = axes
            .iter()
            .filter_map(|axis| Some((axis.joint.clone(), axis.of.clone()?)))
            .collect();
        let axes = axes
            .into_iter()
            .map(|axis| {
                let model = ActuatorModel::parse(&axis.model).ok_or_else(|| BodyError::Model {
                    joint: axis.joint.clone(),
                    model: axis.model.clone(),
                })?;
                let bus = ActuatorBus::parse(&axis.bus).ok_or_else(|| BodyError::Bus {
                    joint: axis.joint.clone(),
                    bus: axis.bus.clone(),
                })?;
                let motion = axis.motion()?;
                Ok(BoundAxis {
                    joint: axis.joint,
                    port: axis.port,
                    lower: axis.lower,
                    upper: axis.upper,
                    binding: ActuatorBinding {
                        model,
                        bus,
                        bus_id: axis.bus_id,
                        sign: axis.sign,
                        reduction: axis.reduction,
                    },
                    zero: UNCALIBRATED_ZERO,
                    direction: UNCALIBRATED_DIRECTION,
                    home: axis.home,
                    motion,
                })
            })
            .collect::<Result<Vec<_>, BodyError>>()?;
        validate_chain(&axes)?;
        let positive = |value: f64| value.is_finite() && value > 0.0;
        let approach = approach
            .into_iter()
            .map(|(port, limit)| {
                let serial = |axis: &BoundAxis| axis.binding.bus.wire().ends_with("_serial");
                if axes.iter().any(|axis| axis.port == port && serial(axis))
                    && positive(limit.speed)
                    && positive(limit.acceleration)
                    && limit.speed <= MAX_APPROACH_SPEED
                {
                    Ok((
                        port,
                        ApproachLimit {
                            speed: limit.speed,
                            acceleration: limit.acceleration,
                        },
                    ))
                } else {
                    Err(BodyError::Approach {
                        embodiment: embodiment.clone(),
                        port,
                    })
                }
            })
            .collect::<Result<BTreeMap<_, _>, BodyError>>()?;
        Ok(Self {
            embodiment,
            id,
            supersedes,
            axes,
            provisional,
            twins,
            approach,
        })
    }

    /// Whether a drive assigned `embodiment` is this body: its id, or an earlier id it
    /// supersedes.
    #[must_use]
    pub fn is(&self, embodiment: &str) -> bool {
        self.id == embodiment || self.supersedes.iter().any(|id| id == embodiment)
    }

    /// The body's buses, in native state order: each port's axes, which must be one run of
    /// one bus kind, so that composing the buses' chains in this order is the state order.
    ///
    /// # Errors
    ///
    /// Refuses a port carrying two bus kinds, and a port whose axes are not contiguous.
    pub fn buses(&self) -> Result<Vec<Bus>, BodyError> {
        let mut buses: Vec<Bus> = Vec::new();
        for axis in &self.axes {
            if let Some(last) = buses.last_mut().filter(|bus| bus.port == axis.port) {
                if last.kind != axis.binding.bus {
                    return Err(BodyError::MixedPort {
                        embodiment: self.embodiment.clone(),
                        port: axis.port.clone(),
                        first: last.kind.wire(),
                        second: axis.binding.bus.wire(),
                    });
                }
                last.axes.push(axis.clone());
                continue;
            }
            if buses.iter().any(|bus| bus.port == axis.port) {
                return Err(BodyError::InterleavedPort {
                    embodiment: self.embodiment.clone(),
                    port: axis.port.clone(),
                });
            }
            buses.push(Bus {
                port: axis.port.clone(),
                kind: axis.binding.bus,
                axes: vec![axis.clone()],
                provisional: self.provisional.contains(&axis.port),
            });
        }
        Ok(buses)
    }

    /// The one bus these axes are on.
    ///
    /// # Errors
    ///
    /// Refuses axes on no bus or on several, and what [`BodyBindings::buses`] refuses.
    pub fn one_bus(&self) -> Result<Bus, BodyError> {
        let mut buses = self.buses()?;
        if buses.len() == 1 {
            Ok(buses.remove(0))
        } else {
            Err(BodyError::Buses {
                embodiment: self.embodiment.clone(),
                ports: buses.into_iter().map(|bus| bus.port).collect(),
            })
        }
    }

    /// The body and port the bus on `port` repeats: `None` when none of its joints has a twin.
    ///
    /// # Errors
    ///
    /// Refuses a bus some of whose joints have no twin, or whose twins are not all on one port
    /// of one body.
    pub fn twin_bus(&self, port: &str) -> Result<Option<(String, String)>, BodyError> {
        let on: Vec<Option<&Twin>> = self
            .axes
            .iter()
            .filter(|axis| axis.port == port)
            .map(|axis| self.twins.get(&axis.joint))
            .collect();
        let refused = || BodyError::Twins {
            embodiment: self.embodiment.clone(),
            port: port.to_owned(),
        };
        let Some(Some(first)) = on.first().copied() else {
            return if on.iter().all(Option::is_none) {
                Ok(None)
            } else {
                Err(refused())
            };
        };
        let same = |twin: Option<&Twin>| {
            twin.is_some_and(|twin| twin.body == first.body && twin.port == first.port)
        };
        if on.iter().copied().all(same) {
            Ok(Some((first.body.clone(), first.port.clone())))
        } else {
            Err(refused())
        }
    }

    /// The bus on `port` as its twin: only that bus's axes, each named by its twin's joint and
    /// port and keeping this body's bounds, calibration, binding and provisional mark, so the
    /// crate composing the twin configures it. Its axes are in this body's order, which the crate checks against
    /// its own; [`BodyBindings::twin_bus`] has refused a bus that is not one twin's.
    ///
    /// # Errors
    ///
    /// What [`BodyBindings::twin_bus`] refuses, and a bus with no twin.
    pub fn as_twin(self, port: &str) -> Result<Self, BodyError> {
        let Some((_, twin_port)) = self.twin_bus(port)? else {
            return Err(BodyError::Twins {
                embodiment: self.embodiment,
                port: port.to_owned(),
            });
        };
        let provisional = if self.provisional.iter().any(|name| name == port) {
            vec![twin_port]
        } else {
            Vec::new()
        };
        let axes = self
            .axes
            .iter()
            .filter(|axis| axis.port == port)
            .map(|axis| {
                let twin = &self.twins[&axis.joint];
                BoundAxis {
                    joint: twin.joint.clone(),
                    port: twin.port.clone(),
                    ..axis.clone()
                }
            })
            .collect();
        Ok(Self {
            axes,
            provisional,
            twins: BTreeMap::new(),
            // A servo approach is the twin crate's, read from its own bindings.
            approach: BTreeMap::new(),
            ..self
        })
    }

    /// The bus on `port` as the body whose controller configuration drives it: its twin's
    /// (`as_twin`) when its joints name one, else this body's own, keeping only that bus's
    /// axes. Answers that body's registry name and port with the bus.
    ///
    /// # Errors
    ///
    /// What [`BodyBindings::twin_bus`] refuses.
    pub fn configured_by(self, port: &str) -> Result<(String, String, Self), BodyError> {
        if let Some((body, twin_port)) = self.twin_bus(port)? {
            return Ok((body, twin_port, self.as_twin(port)?));
        }
        let axes = self
            .axes
            .iter()
            .filter(|axis| axis.port == port)
            .cloned()
            .collect();
        Ok((
            self.embodiment.clone(),
            port.to_owned(),
            Self {
                axes,
                twins: BTreeMap::new(),
                ..self
            },
        ))
    }

    /// The axes on buses of `kind` alone, in native state order: what a crate driving only
    /// that kind composes.
    #[must_use]
    pub fn on(self, kind: ActuatorBus) -> Self {
        Self {
            axes: self
                .axes
                .into_iter()
                .filter(|axis| axis.binding.bus == kind)
                .collect(),
            ..self
        }
    }

    /// The axis bound to `joint`.
    ///
    /// # Errors
    ///
    /// Returns [`BodyError::MissingJoint`] when the body has none.
    pub fn axis(&self, joint: &str) -> Result<&BoundAxis, BodyError> {
        self.axes
            .iter()
            .find(|axis| axis.joint == joint)
            .ok_or_else(|| BodyError::MissingJoint {
                embodiment: self.embodiment.clone(),
                joint: joint.to_owned(),
            })
    }

    /// Refuse a body whose joints, in native state order, are not exactly `expected`: a driver
    /// composing a body configures every axis, and only those.
    ///
    /// # Errors
    ///
    /// Returns [`BodyError::Joints`] on any difference.
    pub fn require_joints(&self, expected: &[&str]) -> Result<(), BodyError> {
        let found: Vec<String> = self.axes.iter().map(|axis| axis.joint.clone()).collect();
        if found
            .iter()
            .map(String::as_str)
            .eq(expected.iter().copied())
        {
            Ok(())
        } else {
            Err(BodyError::Joints {
                embodiment: self.embodiment.clone(),
                found,
                expected: expected.iter().map(|joint| (*joint).to_owned()).collect(),
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DOCUMENT: &str = r#"{"embodiment": "b", "id": "0", "axes": [
        {"joint": "j1", "lower": -1.0, "upper": 1.0, "model": "damiao_dm4310",
         "bus": "damiao_can", "port": "can", "bus_id": 1, "sign": -1, "reduction": 2.0, "speed": 1.0, "acceleration": 10.0}]}"#;

    #[test]
    fn a_joint_without_a_positive_finite_motion_is_refused() {
        let body = BodyBindings::parse(DOCUMENT).expect("parses");
        assert_eq!(
            body.axes[0].motion,
            AxisMotion {
                speed: 1.0,
                acceleration: 10.0,
                inertia: 0.0,
            }
        );
        for wrong in ["\"speed\": 0.0", "\"speed\": -1.0"] {
            assert!(matches!(
                BodyBindings::parse(&DOCUMENT.replace("\"speed\": 1.0", wrong)),
                Err(BodyError::Motion { .. })
            ));
        }
        assert!(BodyBindings::parse(&DOCUMENT.replace(", \"speed\": 1.0", "")).is_err());
    }

    #[test]
    fn a_ports_servo_approach_is_read_and_a_wrong_one_refused() {
        let serial = DOCUMENT
            .replace("damiao_dm4310", "feetech_sts3215")
            .replace("damiao_can", "feetech_serial")
            .replace("\"can\"", "\"neck\"");
        let with = |approach: &str| {
            serial.replacen(
                "\"acceleration\": 10.0}]}",
                &format!("\"acceleration\": 10.0}}], \"approach\": {approach}}}"),
                1,
            )
        };
        let body = BodyBindings::parse(&with(r#"{"neck": {"speed": 4.0, "acceleration": 16.0}}"#))
            .expect("parses");
        assert_eq!(
            body.approach.get("neck"),
            Some(&ApproachLimit {
                speed: 4.0,
                acceleration: 16.0
            })
        );
        assert!(
            BodyBindings::parse(DOCUMENT)
                .expect("parses")
                .approach
                .is_empty()
        );
        for wrong in [
            r#"{"arm": {"speed": 4.0, "acceleration": 16.0}}"#,
            r#"{"neck": {"speed": 0.0, "acceleration": 16.0}}"#,
            r#"{"neck": {"speed": 40.0, "acceleration": 16.0}}"#,
            r#"{"neck": {"speed": 4.0}}"#,
        ] {
            assert!(BodyBindings::parse(&with(wrong)).is_err(), "{wrong}");
        }
        // A CAN bus has no servo approach.
        let can = DOCUMENT.replacen(
            "\"acceleration\": 10.0}]}",
            "\"acceleration\": 10.0}], \"approach\": {\"can\": {\"speed\": 4.0, \"acceleration\": 16.0}}}",
            1,
        );
        assert!(BodyBindings::parse(&can).is_err());
    }

    #[test]
    fn a_rendered_body_parses_uncalibrated() {
        let body = BodyBindings::parse(DOCUMENT).expect("parses");
        let axis = body.axis("j1").expect("j1");
        assert_eq!(axis.binding.model.wire(), "damiao_dm4310");
        assert_eq!(axis.binding.sign, -1);
        assert!(axis.zero == UNCALIBRATED_ZERO);
        assert_eq!(body.require_joints(&["j1"]), Ok(()));
        assert!(matches!(
            body.require_joints(&["j1", "j2"]),
            Err(BodyError::Joints { .. })
        ));
        assert!(matches!(
            body.axis("j2"),
            Err(BodyError::MissingJoint { .. })
        ));
    }

    fn axis(joint: &str, bus: &str, port: &str, bus_id: u16) -> String {
        format!(
            r#"{{"joint": "{joint}", "lower": -1.0, "upper": 1.0, "model": "damiao_dm4310",
                "bus": "{bus}", "port": "{port}", "bus_id": {bus_id}, "sign": 1,
                "reduction": 1.0, "speed": 1.0, "acceleration": 10.0}}"#
        )
    }

    fn document(axes: &[String]) -> String {
        format!(
            r#"{{"embodiment": "b", "id": "0", "axes": [{}]}}"#,
            axes.join(",")
        )
    }

    #[test]
    fn a_body_splits_into_one_bus_per_port_in_state_order() {
        let body = BodyBindings::parse(&document(&[
            axis("j1", "damiao_can", "can", 1),
            axis("j2", "damiao_can", "can", 2),
            axis("n1", "feetech_serial", "neck", 1),
        ]))
        .expect("parses");
        let buses = body.buses().expect("buses");
        let ports: Vec<(&str, &str, usize)> = buses
            .iter()
            .map(|bus| (bus.port.as_str(), bus.kind.wire(), bus.axes.len()))
            .collect();
        assert_eq!(
            ports,
            [("can", "damiao_can", 2), ("neck", "feetech_serial", 1)]
        );
        let can = body.clone().on(ActuatorBus::new("damiao_can"));
        assert_eq!(can.axes.len(), 2);
        assert_eq!(can.id, body.id);
    }

    #[test]
    fn a_provisional_port_is_named_and_a_half_provisional_port_refused() {
        let provisional = |joint: &str, id: u16| {
            axis(joint, "feetech_serial", "neck", id)
                .replace("\"bus_id\"", "\"provisional\": true, \"bus_id\"")
        };
        let body = BodyBindings::parse(&document(&[
            axis("j1", "damiao_can", "can", 1),
            provisional("n1", 1),
            provisional("n2", 2),
        ]))
        .expect("parses");
        assert_eq!(body.provisional, ["neck"]);
        let buses = body.buses().expect("buses");
        assert_eq!(
            buses.iter().map(|bus| bus.provisional).collect::<Vec<_>>(),
            [false, true]
        );
        assert!(matches!(
            BodyBindings::parse(&document(&[
                provisional("n1", 1),
                axis("n2", "feetech_serial", "neck", 2),
            ])),
            Err(BodyError::MixedProvisional { port, .. }) if port == "neck"
        ));
    }

    #[test]
    fn a_port_of_two_kinds_or_out_of_order_is_refused() {
        let mixed = BodyBindings::parse(&document(&[
            axis("j1", "damiao_can", "can", 1),
            axis("j2", "feetech_serial", "can", 2),
        ]))
        .expect("parses");
        assert!(matches!(mixed.buses(), Err(BodyError::MixedPort { .. })));
        let interleaved = BodyBindings::parse(&document(&[
            axis("j1", "damiao_can", "can", 1),
            axis("n1", "feetech_serial", "neck", 1),
            axis("j2", "damiao_can", "can", 2),
        ]))
        .expect("parses");
        assert!(matches!(
            interleaved.buses(),
            Err(BodyError::InterleavedPort { port, .. }) if port == "can"
        ));
        // One address on two ports is two buses; on one port it is refused.
        assert!(
            BodyBindings::parse(&document(&[
                axis("a", "feetech_serial", "left", 1),
                axis("b", "feetech_serial", "right", 1),
            ]))
            .is_ok()
        );
        assert_eq!(
            BodyBindings::parse(&document(&[
                axis("a", "feetech_serial", "left", 1),
                axis("b", "feetech_serial", "left", 1),
            ])),
            Err(BodyError::Binding(BindingError::DuplicateAddress(1)))
        );
    }

    #[test]
    fn what_the_registry_lacks_or_refuses_is_refused() {
        let foreign = DOCUMENT.replace("damiao_dm4310", "dynamixel_xl330");
        assert!(matches!(
            BodyBindings::parse(&foreign),
            Err(BodyError::Model { .. })
        ));
        let foreign = DOCUMENT.replace("\"damiao_can\"", "\"can\"");
        assert!(matches!(
            BodyBindings::parse(&foreign),
            Err(BodyError::Bus { .. })
        ));
        let unsigned = DOCUMENT.replace("\"sign\": -1", "\"sign\": 0");
        assert_eq!(
            BodyBindings::parse(&unsigned),
            Err(BodyError::Binding(BindingError::Sign(1)))
        );
        let extra = DOCUMENT.replace("\"sign\"", "\"zero_offset\": 0.0, \"sign\"");
        assert!(matches!(
            BodyBindings::parse(&extra),
            Err(BodyError::Document(_))
        ));
    }
}
