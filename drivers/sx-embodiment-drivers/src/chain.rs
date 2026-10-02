//! The bound actuator chain: command in joint coordinates, stop with proof.

use thiserror::Error;

use crate::binding::BoundAxis;

/// The strongest evidence a command's own bus traffic established; never physical effect.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Evidence {
    /// Frames were written; the bus answers nothing.
    DispatchAttempted,
    /// Every actuator answered this command, reporting itself enabled.
    BusObservedComplete,
}

/// What one command established.
#[derive(Clone, Debug, PartialEq)]
pub struct Receipt {
    pub evidence: Evidence,
    /// The joint coordinates the actuators reported in their answers, in native state order;
    /// empty when the bus answers nothing.
    pub observed: Vec<f64>,
}

/// What a stop's answers proved, actuator by actuator, by bus address.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct StopReport {
    /// Actuators with no answer proving them stopped.
    pub unproven: Vec<u16>,
    /// (bus address, vendor fault code) reported while stopping.
    pub faults: Vec<(u16, u8)>,
}

impl StopReport {
    #[must_use]
    pub fn is_proven(&self) -> bool {
        self.unproven.is_empty()
    }

    /// The proven stop, or this report back when any actuator is unproven.
    ///
    /// # Errors
    ///
    /// Returns the report itself when the stop is unproven.
    pub fn prove(self) -> Result<ProvenStop, Self> {
        if self.is_proven() {
            Ok(ProvenStop {
                faults: self.faults,
            })
        } else {
            Err(self)
        }
    }
}

/// Every actuator of the chain answered that it stopped. Only [`StopReport::prove`] makes one.
#[derive(Debug, Eq, PartialEq)]
#[must_use]
pub struct ProvenStop {
    faults: Vec<(u16, u8)>,
}

impl ProvenStop {
    /// Fault codes reported while stopping; a stop can be proven and still carry faults.
    #[must_use]
    pub fn faults(&self) -> &[(u16, u8)] {
        &self.faults
    }
}

/// A command vector the chain refuses before any frame is built.
#[derive(Debug, Error, PartialEq)]
pub enum TargetError {
    #[error("expected {expected} values in native state order, got {actual}")]
    Width { expected: usize, actual: usize },
    #[error("joint {joint} target {value} is outside its declared bounds or not finite")]
    OutOfBounds { joint: String, value: f64 },
}

/// Refuse a vector of the wrong width or with a value outside its axis's bounds.
///
/// # Errors
///
/// Returns the width mismatch or the first value out of bounds.
pub fn check_targets(axes: &[BoundAxis], joints: &[f64]) -> Result<(), TargetError> {
    if axes.len() != joints.len() {
        return Err(TargetError::Width {
            expected: axes.len(),
            actual: joints.len(),
        });
    }
    match axes
        .iter()
        .zip(joints)
        .find(|(axis, target)| !axis.admits(**target))
    {
        Some((refused, target)) => Err(TargetError::OutOfBounds {
            joint: refused.joint.clone(),
            value: *target,
        }),
        None => Ok(()),
    }
}

/// A chain of bound actuators on one bus, in the embodiment's native state order.
pub trait ActuatorChain {
    type Error: std::error::Error + Send + Sync + 'static;

    /// The bound axes, in native state order: index `i` of every vector is `axes()[i]`.
    fn axes(&self) -> &[BoundAxis];

    /// Drive every axis toward `joints`, in joint coordinates, enabling the chain first if a
    /// stop left it disabled.
    ///
    /// # Errors
    ///
    /// Refuses a vector outside the axes' bounds before any frame is sent; reports the link's
    /// failure or an actuator whose answer contradicts the command.
    fn command(&mut self, joints: &[f64]) -> Result<Receipt, Self::Error>;

    /// Disable every actuator, whatever an earlier one answered, and report what the answers
    /// prove. The chain stays disabled until the next command.
    fn stop(&mut self) -> StopReport;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::binding::{ActuatorBinding, ActuatorBus, ActuatorModel};

    #[test]
    fn only_a_fully_answered_stop_is_proven() {
        let report = StopReport {
            unproven: vec![],
            faults: vec![(3, 0xA)],
        };
        let proven = report.prove().expect("every actuator answered");
        assert_eq!(proven.faults(), &[(3, 0xA)]);
        let report = StopReport {
            unproven: vec![2],
            faults: vec![],
        };
        assert_eq!(report.clone().prove(), Err(report));
    }

    #[test]
    fn targets_are_checked_against_width_and_bounds() {
        let axes = [BoundAxis {
            joint: "joint1".into(),
            lower: -1.0,
            upper: 1.0,
            binding: ActuatorBinding {
                model: ActuatorModel::new("model"),
                bus: ActuatorBus::new("bus"),
                bus_id: 1,
                sign: 1,
                zero_offset: 0.0,
                reduction: 1.0,
            },
        }];
        assert_eq!(check_targets(&axes, &[0.5]), Ok(()));
        assert_eq!(
            check_targets(&axes, &[]),
            Err(TargetError::Width {
                expected: 1,
                actual: 0
            })
        );
        for bad in [1.5, f64::NAN] {
            assert!(matches!(
                check_targets(&axes, &[bad]),
                Err(TargetError::OutOfBounds { .. })
            ));
        }
    }
}
