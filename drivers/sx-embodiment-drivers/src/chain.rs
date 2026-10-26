//! The bound actuator chain: command in joint coordinates, stop with proof.

use std::time::{Duration, Instant};

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
///
/// The range limits motion, not state. A joint may rest outside it (left there by hand, or
/// against a stop that reads past the limit); each step may then hold it or move it toward the
/// range, never further out: the step's interval is [`CommandRange::reaching`] where the joint
/// is now, and it shrinks as the joint comes in.
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

    /// This range widened to include `at`, where the joint is now:
    /// `[min(lower, at), max(upper, at)]`. A position that is not finite widens nothing.
    #[must_use]
    pub const fn reaching(self, at: f64) -> Self {
        if !at.is_finite() {
            return self;
        }
        Self {
            lower: self.lower.min(at),
            upper: self.upper.max(at),
        }
    }
}

/// Refuse a vector of the wrong width, or a value outside its axis's range widened to where
/// the axis is now ([`CommandRange::reaching`]): `at`, one position per axis, or the plain
/// ranges when the chain observes none.
///
/// # Errors
///
/// Returns the width mismatch or the first value outside its interval or not finite.
pub fn check_reach(
    axes: &[BoundAxis],
    ranges: &[CommandRange],
    at: Option<&[f64]>,
    joints: &[f64],
) -> Result<(), TargetError> {
    if axes.len() != joints.len() {
        return Err(TargetError::Width {
            expected: axes.len(),
            actual: joints.len(),
        });
    }
    let at = at.filter(|at| at.len() == joints.len());
    for (index, ((axis, range), joint)) in axes.iter().zip(ranges).zip(joints).enumerate() {
        let reach = at.map_or(*range, |at| range.reaching(at[index]));
        if !reach.admits(*joint) {
            return Err(TargetError::OutOfBounds {
                joint: axis.joint.clone(),
                value: *joint,
            });
        }
    }
    Ok(())
}

/// What a stop's answers proved, actuator by actuator, by bus address.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct StopReport {
    /// Actuators with no answer proving them stopped.
    pub unproven: Vec<u16>,
    /// (bus address, vendor fault code) reported while stopping.
    pub faults: Vec<(u16, u8)>,
    /// Actuators counted as stopped on a weaker proof: they answered the disable frame with no
    /// fault, but their firmware does not report whether it is enabled, so the disabled state
    /// itself was not observed. A caller that needs an observed disabled state (to release a
    /// brake, to let a person near) must not treat these as equal to it.
    pub unobserved: Vec<u16>,
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
                unobserved: self.unobserved,
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
    unobserved: Vec<u16>,
}

impl ProvenStop {
    /// Fault codes reported while stopping; a stop can be proven and still carry faults.
    #[must_use]
    pub fn faults(&self) -> &[(u16, u8)] {
        &self.faults
    }

    /// Actuators proven only by answering the disable, their enable state unobservable
    /// ([`StopReport::unobserved`]).
    #[must_use]
    pub fn unobserved(&self) -> &[u16] {
        &self.unobserved
    }

    /// Whether every actuator was observed disabled, not merely answering the disable.
    #[must_use]
    pub fn observed_disabled(&self) -> bool {
        self.unobserved.is_empty()
    }
}

/// Why one axis has no fresh reading in a poll.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MissingReason {
    /// The actuator sent no answer within the poll.
    Timeout,
    /// An answer arrived incomplete or failing its checks.
    Corrupt,
    /// The actuator did not answer this poll; the newest answer the chain holds for it predates
    /// the poll, so it is never reported as fresh.
    Stale,
    /// The link could not carry the request or the answer.
    Unavailable,
}

/// One axis's part of a read: what the actuator answered to this poll, if anything.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AxisReading {
    /// The joint coordinate from an answer the chain received after the poll began (older
    /// answers are drained or cleared first), and when it received it. On a bus that cannot
    /// pair an answer with its request (CAN), a late answer to the previous poll arriving after
    /// that drain also counts: it is at most one poll interval old.
    Fresh { position: f64, received: Instant },
    /// No usable answer to this poll.
    Missing(MissingReason),
    /// The actuator answered, reporting a vendor fault code (or a state contradicting what
    /// the chain holds it in); its position is not vouched for.
    Faulted(u8),
}

impl AxisReading {
    /// The position, when the reading is fresh.
    #[must_use]
    pub const fn position(&self) -> Option<f64> {
        match self {
            Self::Fresh { position, .. } => Some(*position),
            Self::Missing(_) | Self::Faulted(_) => None,
        }
    }

    #[must_use]
    pub const fn is_fresh(&self) -> bool {
        matches!(self, Self::Fresh { .. })
    }
}

/// Every position of a read, in native state order, only when every axis is fresh.
#[must_use]
pub fn fresh_positions(readings: &[AxisReading]) -> Option<Vec<f64>> {
    readings.iter().map(AxisReading::position).collect()
}

/// A command vector the chain refuses before any frame is built.
#[derive(Debug, Error, PartialEq)]
pub enum TargetError {
    #[error("expected {expected} values in native state order, got {actual}")]
    Width { expected: usize, actual: usize },
    #[error(
        "joint {joint} target {value} is outside where it may be commanded (its range, widened to where it is now) or not finite"
    )]
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

/// What a command adds to one axis' position target, at the joint: the planned velocity
/// (unit/s) and the model's feed-forward torque (N·m, N for a prismatic joint), both in joint
/// coordinates. An actuator that takes only a position target ignores it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Feedforward {
    pub velocity: f64,
    pub torque: f64,
}

/// How much [`Feedforward`] one axis accepts, at the joint. `None` switches that part off: the
/// actuator is sent zero, as before feed-forward existed. A vendor chain clamps a value to its
/// bound, then to what the actuator itself accepts.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FeedforwardLimit {
    /// The largest planned speed sent, unit/s; the governor's speed cap is the natural bound.
    pub velocity: Option<f64>,
    /// The largest feed-forward torque sent, N·m.
    pub torque: Option<f64>,
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
    /// Refuses a vector outside the commandable ranges, each widened to where its axis is now
    /// ([`CommandRange::reaching`]), before any frame is sent; reports the
    /// link's failure or an actuator whose answer contradicts the command.
    fn command(&mut self, joints: &[f64]) -> Result<Receipt<Self::Feedback>, Self::Error>;

    /// [`ActuatorChain::command`] with each axis' planned velocity and feed-forward torque,
    /// one per axis in native state order, bounded by
    /// [`ActuatorChain::set_feedforward_limits`]. A chain whose actuators take only a position
    /// target sends the positions alone.
    ///
    /// # Errors
    ///
    /// As [`ActuatorChain::command`]; a vendor chain also refuses, before any frame, a list of
    /// another length or a non-finite entry.
    fn command_with(
        &mut self,
        joints: &[f64],
        feedforward: &[Feedforward],
    ) -> Result<Receipt<Self::Feedback>, Self::Error> {
        let _ = feedforward;
        self.command(joints)
    }

    /// Bound, or switch off, each axis' feed-forward, in native state order. Every part is off
    /// until this bounds it. A chain whose actuators take only a position target accepts any
    /// bounds and sends none.
    ///
    /// # Errors
    ///
    /// A vendor chain refuses a list of another length and a bound that is not finite and
    /// non-negative, keeping the bounds in force.
    fn set_feedforward_limits(&mut self, limits: &[FeedforwardLimit]) -> Result<(), Self::Error> {
        let _ = limits;
        Ok(())
    }

    /// Poll measured joint coordinates without enabling an actuator or changing its target:
    /// one [`AxisReading`] per axis, always `axes().len()` of them in native state order, an
    /// axis that did not answer this poll freshly reported as missing or faulted rather than
    /// dropped. A read never fails as a whole; a link that is down reads every axis
    /// [`MissingReason::Unavailable`].
    fn read_positions(&mut self) -> Vec<AxisReading>;

    /// Each axis's reading from the last command's own observation, in native state order:
    /// what the bus answered while the body moved, so a caller can keep it beside the command.
    /// `None` when the chain observes nothing with its commands, or no command has answered
    /// since open or the last stop. A value in [`Receipt::observed`] that is NaN is an axis
    /// here that is not [`AxisReading::Fresh`].
    fn command_readings(&self) -> Option<Vec<AxisReading>> {
        None
    }

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

    /// Release faults a chain latched from a proven stop. The caller's policy decides when; a
    /// chain that latches nothing has nothing to release.
    fn clear_faults(&mut self) {}
}

/// Bound actuators that are only read, never commanded: the read half of an
/// [`ActuatorChain`], for a recorder or monitor running beside a body nothing drives. A type
/// implementing only this cannot enable, configure or move an actuator, by construction; each
/// vendor crate's observer also speaks through a port that refuses any request but a read.
pub trait Observe {
    /// The bound axes, in native state order, exactly as the chain opened on the same buses
    /// lists them.
    fn axes(&self) -> &[BoundAxis];

    /// One [`AxisReading`] per axis in native state order, in joint coordinates through each
    /// axis's drive map and unit calibration, as [`ActuatorChain::read_positions`] reports them.
    /// An axis that did not answer is missing or faulted in its place, never dropped.
    fn read_positions(&mut self) -> Vec<AxisReading>;
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
            unobserved: vec![],
        };
        let proven = report.prove().expect("every actuator answered");
        assert_eq!(proven.faults(), &[(3, 0xA)]);
        assert!(proven.observed_disabled());
        let report = StopReport {
            unobserved: vec![4],
            ..StopReport::default()
        };
        let proven = report.prove().expect("answered, state unobservable");
        assert_eq!(proven.unobserved(), &[4]);
        assert!(!proven.observed_disabled());
        let report = StopReport {
            unproven: vec![2],
            ..StopReport::default()
        };
        assert_eq!(report.clone().prove(), Err(report));
    }

    #[test]
    fn a_range_reaches_where_the_joint_is_and_no_further() {
        let range = CommandRange {
            lower: -1.0,
            upper: 1.0,
        };
        assert_eq!(range.reaching(0.5), range);
        assert_eq!(
            range.reaching(-2.5),
            CommandRange {
                lower: -2.5,
                upper: 1.0
            }
        );
        assert_eq!(
            range.reaching(3.0),
            CommandRange {
                lower: -1.0,
                upper: 3.0
            }
        );
        assert_eq!(range.reaching(f64::NAN), range);
    }

    #[test]
    fn targets_are_checked_against_width_and_bounds() {
        let axes = [BoundAxis {
            joint: "joint1".into(),
            port: "can".into(),
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
