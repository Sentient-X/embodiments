//! The registry's per-axis drive facts, as a driver receives them.
//!
//! Field for field this is `sx_embodiments.layout.ActuatorBinding` and the axis it binds; the
//! validation is that class's `__post_init__` and `JointLayout`'s unique-address law, so a
//! binding the registry would refuse is refused here too.

use thiserror::Error;

/// One member of the registry's closed `ActuatorModel` vocabulary, by its wire value.
///
/// A driver crate declares the members it is qualified for as constants; the registry's tests
/// hold those values equal to the Python enum's.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct ActuatorModel(&'static str);

impl ActuatorModel {
    #[must_use]
    pub const fn new(wire: &'static str) -> Self {
        Self(wire)
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
    #[must_use]
    pub const fn new(wire: &'static str) -> Self {
        Self(wire)
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
    pub zero_offset: f64,
    pub reduction: f64,
}

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
        if !self.zero_offset.is_finite() {
            return Err(BindingError::ZeroOffset(self.bus_id));
        }
        if !self.reduction.is_finite() || self.reduction <= 0.0 {
            return Err(BindingError::Reduction(self.bus_id));
        }
        Ok(())
    }

    /// `actuator = sign * (joint - zero_offset) * reduction`.
    #[must_use]
    pub fn actuator_from_joint(&self, joint: f64) -> f64 {
        f64::from(self.sign) * (joint - self.zero_offset) * self.reduction
    }

    /// The inverse map, from the actuator's own coordinate to the joint's.
    #[must_use]
    pub fn joint_from_actuator(&self, actuator: f64) -> f64 {
        actuator / (f64::from(self.sign) * self.reduction) + self.zero_offset
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
}

impl BoundAxis {
    /// Refuse empty bounds and an invalid binding.
    ///
    /// # Errors
    ///
    /// Returns the first invalid fact.
    pub fn validate(&self) -> Result<(), BindingError> {
        if !(self.lower.is_finite() && self.upper.is_finite() && self.lower < self.upper) {
            return Err(BindingError::Bounds(self.joint.clone()));
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
    #[error("the actuator at bus_id {0} has a non-finite zero_offset")]
    ZeroOffset(u16),
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
                zero_offset: 0.25,
                reduction: 4.0,
            },
        }
    }

    #[test]
    fn the_drive_map_round_trips() {
        let binding = axis(1).binding;
        let actuator = binding.actuator_from_joint(0.75);
        assert!((actuator - -2.0).abs() < 1e-12);
        assert!((binding.joint_from_actuator(actuator) - 0.75).abs() < 1e-12);
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
        bad.binding.zero_offset = f64::NAN;
        assert_eq!(bad.validate(), Err(BindingError::ZeroOffset(1)));
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
}
