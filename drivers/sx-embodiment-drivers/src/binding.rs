//! The registry's per-axis drive facts, as a driver receives them.
//!
//! Field for field this is `sx_embodiments.layout.ActuatorBinding` and the axis it binds; the
//! validation is that class's `__post_init__` and `JointLayout`'s unique-address law, so a
//! binding the registry would refuse is refused here too. A driver never spells these facts:
//! it reads them from the document `tools/render_driver_bindings.py` renders from the registry
//! ([`crate::body::BodyBindings`]).
//!
//! The drive map is `actuator = sign * (joint - zero) * reduction`. Sign and reduction are the
//! design's; the zero is measured per motor, so it is a joint calibration of the unit and sits
//! on [`BoundAxis`], never on the binding. [`BoundAxis::calibrated`] is the one place a
//! calibration is applied; until a unit's calibration is supplied, every axis carries
//! [`UNCALIBRATED_ZERO`].

use thiserror::Error;

/// One member of the registry's closed `ActuatorModel` vocabulary, by its wire value.
///
/// [`ActuatorModel::ALL`] is the vocabulary, held equal to the Python enum by the rendered
/// `vocabulary.json`; a driver crate names the members it is qualified for as constants.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct ActuatorModel(&'static str);

impl ActuatorModel {
    /// Every member of the registry's `ActuatorModel`, by wire value.
    pub const ALL: [Self; 3] = [
        Self("feetech_sts3215"),
        Self("damiao_dm4310"),
        Self("damiao_dm4340"),
    ];

    #[must_use]
    pub const fn new(wire: &'static str) -> Self {
        Self(wire)
    }

    /// The member spelled `wire`, or `None` when the registry has none.
    #[must_use]
    pub fn parse(wire: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|model| model.0 == wire)
    }

    #[must_use]
    pub const fn wire(self) -> &'static str {
        self.0
    }
}

/// One member of the registry's closed `ActuatorBus` vocabulary, by its wire value.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct ActuatorBus(&'static str);

impl ActuatorBus {
    /// Every member of the registry's `ActuatorBus`, by wire value.
    pub const ALL: [Self; 2] = [Self("feetech_serial"), Self("damiao_can")];

    #[must_use]
    pub const fn new(wire: &'static str) -> Self {
        Self(wire)
    }

    /// The member spelled `wire`, or `None` when the registry has none.
    #[must_use]
    pub fn parse(wire: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|bus| bus.0 == wire)
    }

    #[must_use]
    pub const fn wire(self) -> &'static str {
        self.0
    }
}

/// How one joint axis is physically driven: a qualified product at an address on a bus.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorBinding {
    pub model: ActuatorModel,
    pub bus: ActuatorBus,
    /// The actuator's address on its chain.
    pub bus_id: u16,
    pub sign: i8,
    pub reduction: f64,
}

/// The zero of a joint whose unit has no joint calibration yet: the design's own zero.
pub const UNCALIBRATED_ZERO: f64 = 0.0;

impl ActuatorBinding {
    /// Refuse what `ActuatorBinding.__post_init__` refuses.
    ///
    /// # Errors
    ///
    /// Returns the first invalid field.
    pub fn validate(&self) -> Result<(), BindingError> {
        if self.bus_id < 1 {
            return Err(BindingError::BusId(self.bus_id));
        }
        if !matches!(self.sign, -1 | 1) {
            return Err(BindingError::Sign(self.bus_id));
        }
        if !self.reduction.is_finite() || self.reduction <= 0.0 {
            return Err(BindingError::Reduction(self.bus_id));
        }
        Ok(())
    }
}

/// One coordinate of the native state order with the actuator that drives it.
#[derive(Clone, Debug, PartialEq)]
pub struct BoundAxis {
    /// The registry's joint name (`JointAxis.name`).
    pub joint: String,
    /// The joint's declared bounds, in its own unit.
    pub lower: f64,
    pub upper: f64,
    pub binding: ActuatorBinding,
    /// The joint coordinate at the actuator's zero: this unit's joint calibration, or
    /// [`UNCALIBRATED_ZERO`].
    pub zero: f64,
}

impl BoundAxis {
    /// This axis with a unit's measured zero applied: the one seam a joint calibration enters.
    ///
    /// # Errors
    ///
    /// Refuses a non-finite zero.
    pub fn calibrated(self, zero: f64) -> Result<Self, BindingError> {
        if !zero.is_finite() {
            return Err(BindingError::Zero(self.joint));
        }
        Ok(Self { zero, ..self })
    }

    /// `actuator = sign * (joint - zero) * reduction`.
    #[must_use]
    pub fn actuator_from_joint(&self, joint: f64) -> f64 {
        f64::from(self.binding.sign) * (joint - self.zero) * self.binding.reduction
    }

    /// The inverse map, from the actuator's own coordinate to the joint's.
    #[must_use]
    pub fn joint_from_actuator(&self, actuator: f64) -> f64 {
        actuator / (f64::from(self.binding.sign) * self.binding.reduction) + self.zero
    }

    /// Refuse empty bounds and an invalid binding.
    ///
    /// # Errors
    ///
    /// Returns the first invalid fact.
    pub fn validate(&self) -> Result<(), BindingError> {
        if !(self.lower.is_finite() && self.upper.is_finite() && self.lower < self.upper) {
            return Err(BindingError::Bounds(self.joint.clone()));
        }
        if !self.zero.is_finite() {
            return Err(BindingError::Zero(self.joint.clone()));
        }
        self.binding.validate()
    }

    /// Whether `joint` is a finite value inside the declared bounds.
    #[must_use]
    pub fn admits(&self, joint: f64) -> bool {
        joint.is_finite() && (self.lower..=self.upper).contains(&joint)
    }
}

/// Validate every axis and `JointLayout`'s law: one address per bus within a chain.
///
/// # Errors
///
/// Returns the first invalid axis, or the first repeated address.
pub fn validate_chain(axes: &[BoundAxis]) -> Result<(), BindingError> {
    if axes.is_empty() {
        return Err(BindingError::EmptyChain);
    }
    for (index, axis) in axes.iter().enumerate() {
        axis.validate()?;
        let address = (axis.binding.bus, axis.binding.bus_id);
        if axes[..index]
            .iter()
            .any(|earlier| (earlier.binding.bus, earlier.binding.bus_id) == address)
        {
            return Err(BindingError::DuplicateAddress(axis.binding.bus_id));
        }
    }
    Ok(())
}

#[derive(Debug, Error, PartialEq)]
pub enum BindingError {
    #[error("the chain binds no axis")]
    EmptyChain,
    #[error("bus_id {0} must be a positive integer")]
    BusId(u16),
    #[error("the actuator at bus_id {0} has a sign other than +1 or -1")]
    Sign(u16),
    #[error("joint {0} has a non-finite zero")]
    Zero(String),
    #[error("the actuator at bus_id {0} has a reduction that is not a positive finite ratio")]
    Reduction(u16),
    #[error("joint {0} has bounds that are not finite lower < upper")]
    Bounds(String),
    #[error("bus_id {0} appears twice on one bus within the chain")]
    DuplicateAddress(u16),
}

#[cfg(test)]
mod tests {
    use super::*;

    const MODEL: ActuatorModel = ActuatorModel::new("model");
    const BUS: ActuatorBus = ActuatorBus::new("bus");

    fn axis(bus_id: u16) -> BoundAxis {
        BoundAxis {
            joint: format!("joint{bus_id}"),
            lower: -1.0,
            upper: 1.0,
            binding: ActuatorBinding {
                model: MODEL,
                bus: BUS,
                bus_id,
                sign: -1,
                reduction: 4.0,
            },
            zero: 0.25,
        }
    }

    #[test]
    fn the_drive_map_round_trips() {
        let bound = axis(1);
        let actuator = bound.actuator_from_joint(0.75);
        assert!((actuator - -2.0).abs() < 1e-12);
        assert!((bound.joint_from_actuator(actuator) - 0.75).abs() < 1e-12);
        let recalibrated = bound.calibrated(0.5).expect("a finite zero");
        assert!((recalibrated.actuator_from_joint(0.75) - -1.0).abs() < 1e-12);
    }

    #[test]
    fn what_the_registry_refuses_is_refused() {
        let mut bad = axis(1);
        bad.binding.bus_id = 0;
        assert_eq!(bad.validate(), Err(BindingError::BusId(0)));
        let mut bad = axis(1);
        bad.binding.sign = 0;
        assert_eq!(bad.validate(), Err(BindingError::Sign(1)));
        let mut bad = axis(1);
        bad.binding.reduction = 0.0;
        assert_eq!(bad.validate(), Err(BindingError::Reduction(1)));
        let mut bad = axis(1);
        bad.zero = f64::NAN;
        assert!(matches!(bad.validate(), Err(BindingError::Zero(_))));
        assert!(matches!(
            axis(1).calibrated(f64::INFINITY),
            Err(BindingError::Zero(_))
        ));
        let mut bad = axis(1);
        bad.upper = bad.lower;
        assert!(matches!(bad.validate(), Err(BindingError::Bounds(_))));
        assert_eq!(validate_chain(&[]), Err(BindingError::EmptyChain));
        assert_eq!(
            validate_chain(&[axis(1), axis(2), axis(1)]),
            Err(BindingError::DuplicateAddress(1))
        );
        assert_eq!(validate_chain(&[axis(1), axis(2)]), Ok(()));
    }

    #[test]
    fn the_vocabulary_parses_only_its_own_members() {
        assert_eq!(
            ActuatorModel::parse("damiao_dm4310"),
            Some(ActuatorModel::ALL[1])
        );
        assert_eq!(ActuatorModel::parse("dynamixel_xl330"), None);
        assert_eq!(ActuatorBus::parse("damiao_can"), Some(ActuatorBus::ALL[1]));
        assert_eq!(ActuatorBus::parse("can"), None);
    }
}
