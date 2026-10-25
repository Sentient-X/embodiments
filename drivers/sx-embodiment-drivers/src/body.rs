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

use serde::Deserialize;
use thiserror::Error;

use crate::binding::{
    ActuatorBinding, ActuatorBus, ActuatorModel, BindingError, BoundAxis, UNCALIBRATED_DIRECTION,
    UNCALIBRATED_ZERO, validate_chain,
};

/// One registered body's bound axes, in native state order, uncalibrated.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyBindings {
    /// The registry's name for the body.
    pub embodiment: String,
    /// The body's content id when it was rendered.
    pub id: String,
    pub axes: Vec<BoundAxis>,
    /// The ports whose bindings the registry marks provisional (`ActuatorBinding.provisional`):
    /// not yet verified on the robot, so not driven unless a supervised bring-up opts in.
    pub provisional: Vec<String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    embodiment: String,
    id: String,
    axes: Vec<Axis>,
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
    #[error("{embodiment}'s port {port} has both provisional and verified bindings")]
    MixedProvisional { embodiment: String, port: String },
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
                })
            })
            .collect::<Result<Vec<_>, BodyError>>()?;
        validate_chain(&axes)?;
        Ok(Self {
            embodiment,
            id,
            axes,
            provisional,
        })
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
         "bus": "damiao_can", "port": "can", "bus_id": 1, "sign": -1, "reduction": 2.0}]}"#;

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
                "reduction": 1.0}}"#
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
