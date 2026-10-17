//! A body's bound axes as the registry renders them: the one source a driver composes from.
//!
//! `tools/render_driver_bindings.py` renders, for each body a driver crate composes, its
//! `robot.state` coordinates in native state order with each one's bounds and actuator binding.
//! A driver crate embeds that document and parses it here, so no joint bound, address, model,
//! sign or reduction is spelled in Rust. What a crate adds is its vendor's controller
//! configuration, keyed by joint name.

use serde::Deserialize;
use thiserror::Error;

use crate::binding::{
    ActuatorBinding, ActuatorBus, ActuatorModel, BindingError, BoundAxis, UNCALIBRATED_ZERO,
    validate_chain,
};

/// One registered body's bound axes, in native state order, uncalibrated.
#[derive(Clone, Debug, PartialEq)]
pub struct BodyBindings {
    /// The registry's name for the body.
    pub embodiment: String,
    /// The body's content id when it was rendered.
    pub id: String,
    pub axes: Vec<BoundAxis>,
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
    #[error("{embodiment} has no joint {joint}")]
    MissingJoint { embodiment: String, joint: String },
    #[error("{embodiment}'s joints {found:?} are not the {expected:?} this driver composes")]
    Joints {
        embodiment: String,
        found: Vec<String>,
        expected: Vec<String>,
    },
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
                })
            })
            .collect::<Result<Vec<_>, BodyError>>()?;
        validate_chain(&axes)?;
        Ok(Self {
            embodiment,
            id,
            axes,
        })
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
         "bus": "damiao_can", "bus_id": 1, "sign": -1, "reduction": 2.0}]}"#;

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
