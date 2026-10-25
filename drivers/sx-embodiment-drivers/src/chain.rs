//! The bound actuator chain: command in joint coordinates, stop with proof.

use std::time::Duration;

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
pub struct Receipt<F> {
    pub evidence: Evidence,
    /// The joint coordinates the actuators reported in their answers, in native state order;
    /// empty when the bus answers nothing.
    pub observed: Vec<f64>,
    /// Each actuator's own answer, in its bus's terms, in native state order; empty when the
    /// bus answers nothing.
    pub feedback: Vec<F>,
}

/// The joint range a chain commands on one axis: the declared bounds intersected with what
/// the driver's configuration lets the actuator reach.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CommandRange {
    pub lower: f64,
    pub upper: f64,
}

impl CommandRange {
    #[must_use]
    pub fn admits(&self, joint: f64) -> bool {
        joint.is_finite() && (self.lower..=self.upper).contains(&joint)
    }
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

/// What a refused or failed chain operation means to the one driving it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ChainFailure {
    /// Refused before any frame changed what an actuator holds.
    Refused,
    /// An actuator did not answer in time.
    Timeout,
    /// The link itself could not carry the exchange.
    Unavailable,
    /// The bus answered something that contradicts the command, or failed mid-command.
    Fault,
}

/// A chain's error, classified once by the crate that knows its vendor's failure modes.
pub trait ChainError: std::error::Error + Send + Sync + 'static {
    fn failure(&self) -> ChainFailure;
}

/// A chain of bound actuators, in the embodiment's native state order.
pub trait ActuatorChain {
    type Error: ChainError;
    /// One actuator's answer as the bus reports it.
    type Feedback: Clone + std::fmt::Debug + PartialEq;

    /// The bound axes, in native state order: index `i` of every vector is `axes()[i]`.
    fn axes(&self) -> &[BoundAxis];

    /// The joint range each axis is commanded within, in native state order; always inside
    /// the axis's declared bounds.
    fn commandable(&self) -> &[CommandRange];

    /// Drive every axis toward `joints`, in joint coordinates, enabling the chain first if a
    /// stop left it disabled.
    ///
    /// # Errors
    ///
    /// Refuses a vector outside the commandable ranges before any frame is sent; reports the
    /// link's failure or an actuator whose answer contradicts the command.
    fn command(&mut self, joints: &[f64]) -> Result<Receipt<Self::Feedback>, Self::Error>;

    /// Disable every actuator, whatever an earlier one answered, and report what the answers
    /// prove. The chain stays disabled until the next command.
    fn stop(&mut self) -> StopReport;

    /// The longest [`ActuatorChain::stop`] can take, whatever the actuators answer.
    fn stop_budget(&self) -> Duration;

    /// The joint coordinates every actuator last reported, in native state order, or `None`
    /// when the bus answers nothing or an actuator has not answered. A chain that observes
    /// reports them from open and from each stop, so a caller can start from where the body
    /// rests before it commands anything.
    fn last_observed(&self) -> Option<Vec<f64>> {
        None
    }

    /// Whether the device itself limits how fast axis `axis` (index in native state order)
    /// moves toward a target, so a target far from where it rests is approached at the device's
    /// own pace rather than stepped to. A chain that cannot promise this says `false`.
    fn approach_speed_limited(&self, axis: usize) -> bool {
        let _ = axis;
        false
    }

    /// Release faults a chain latched from a proven stop. The caller's policy decides when; a
    /// chain that latches nothing has nothing to release.
    fn clear_faults(&mut self) {}
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
                reduction: 1.0,
            },
            zero: crate::binding::UNCALIBRATED_ZERO,
            direction: crate::binding::UNCALIBRATED_DIRECTION,
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
