//! Two chains driven as one: a body whose native state order spans two devices or two links.
//!
//! The vector is the first chain's axes, then the second's. Every target is checked against both
//! chains' commandable ranges, each widened to where that chain observes the axis now
//! ([`CommandRange::reaching`]), before either is written; the first is commanded, then the second.
//! A second chain that fails after the first applied is a [`ChainFailure::Fault`] whatever it
//! reported, since one device already holds its new target. A stop stops both, whatever the first
//! answered, and is proven only when both are. A read reads both, whatever either answered.

use std::time::Duration;

use thiserror::Error;

use crate::axes::Axes;
use crate::binding::BoundAxis;
use crate::chain::{
    ActuatorChain, AxisReading, ChainError, ChainFailure, CommandRange, Evidence, Feedforward,
    FeedforwardLimit, Receipt, StopReport, TargetError, check_reach,
};

/// The first chain's axes, then the second's.
pub struct Composite<A, B> {
    first: A,
    second: B,
    axes: Vec<BoundAxis>,
    commandable: Vec<CommandRange>,
}

#[derive(Debug, Error)]
pub enum CompositeError<E, F> {
    #[error(transparent)]
    Target(#[from] TargetError),
    #[error("{0} (the second chain was not commanded)")]
    First(E),
    #[error("after the first chain applied, {0}")]
    Second(F),
}

impl<E: ChainError, F: ChainError> ChainError for CompositeError<E, F> {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::Target(_) => ChainFailure::Refused,
            Self::First(error) => error.failure(),
            Self::Second(_) => ChainFailure::Fault,
        }
    }
}

impl<A: ActuatorChain, B: ActuatorChain<Feedback = A::Feedback>> Composite<A, B> {
    #[must_use]
    pub fn new(first: A, second: B) -> Self {
        let axes = first.axes().iter().chain(second.axes()).cloned().collect();
        let commandable = first
            .commandable()
            .iter()
            .chain(second.commandable())
            .copied()
            .collect();
        Self {
            first,
            second,
            axes,
            commandable,
        }
    }

    #[must_use]
    pub const fn first(&self) -> &A {
        &self.first
    }

    #[must_use]
    pub const fn second(&self) -> &B {
        &self.second
    }
}

/// What driving a composite answers.
type Driven<A, B> = Result<
    Receipt<<A as ActuatorChain>::Feedback>,
    CompositeError<<A as ActuatorChain>::Error, <B as ActuatorChain>::Error>,
>;

impl<A, B> Composite<A, B>
where
    A: ActuatorChain,
    B: ActuatorChain<Feedback = A::Feedback>,
{
    /// Each chain its slice of `joints`, and of `feedforward` when given.
    fn drive(&mut self, joints: &[f64], feedforward: Option<&[Feedforward]>) -> Driven<A, B> {
        if joints.len() != self.axes.len() {
            return Err(TargetError::Width {
                expected: self.axes.len(),
                actual: joints.len(),
            }
            .into());
        }
        let (head, tail) = joints.split_at(self.first.axes().len());
        let (first_ranges, second_ranges) = self.commandable.split_at(head.len());
        check_reach(
            self.first.axes(),
            first_ranges,
            self.first.last_observed().as_deref(),
            head,
        )?;
        check_reach(
            self.second.axes(),
            second_ranges,
            self.second.last_observed().as_deref(),
            tail,
        )?;
        let (first, second) = match feedforward {
            Some(wanted) => {
                if wanted.len() != joints.len() {
                    return Err(TargetError::Width {
                        expected: joints.len(),
                        actual: wanted.len(),
                    }
                    .into());
                }
                let (mine, theirs) = wanted.split_at(head.len());
                (
                    self.first
                        .command_with(head, mine)
                        .map_err(CompositeError::First)?,
                    self.second
                        .command_with(tail, theirs)
                        .map_err(CompositeError::Second)?,
                )
            }
            None => (
                self.first.command(head).map_err(CompositeError::First)?,
                self.second.command(tail).map_err(CompositeError::Second)?,
            ),
        };
        let evidence = if first.evidence == Evidence::BusObservedComplete {
            second.evidence
        } else {
            first.evidence
        };
        Ok(Receipt {
            evidence,
            observed: first.observed.into_iter().chain(second.observed).collect(),
            feedback: first.feedback.into_iter().chain(second.feedback).collect(),
        })
    }
}

impl<A, B> ActuatorChain for Composite<A, B>
where
    A: ActuatorChain,
    B: ActuatorChain<Feedback = A::Feedback>,
{
    type Error = CompositeError<A::Error, B::Error>;
    type Feedback = A::Feedback;

    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    fn commandable(&self) -> &[CommandRange] {
        &self.commandable
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt<Self::Feedback>, Self::Error> {
        self.drive(joints, None)
    }

    fn command_with(
        &mut self,
        joints: &[f64],
        feedforward: &[Feedforward],
    ) -> Result<Receipt<Self::Feedback>, Self::Error> {
        self.drive(joints, Some(feedforward))
    }

    fn set_feedforward_limits(&mut self, limits: &[FeedforwardLimit]) -> Result<(), Self::Error> {
        if limits.len() != self.axes.len() {
            return Err(TargetError::Width {
                expected: self.axes.len(),
                actual: limits.len(),
            }
            .into());
        }
        let (mine, theirs) = limits.split_at(self.first.axes().len());
        self.first
            .set_feedforward_limits(mine)
            .map_err(CompositeError::First)?;
        self.second
            .set_feedforward_limits(theirs)
            .map_err(CompositeError::Second)
    }

    fn stop(&mut self) -> StopReport {
        let first = self.first.stop();
        let second = self.second.stop();
        StopReport {
            unproven: first.unproven.into_iter().chain(second.unproven).collect(),
            faults: first.faults.into_iter().chain(second.faults).collect(),
            unobserved: first
                .unobserved
                .into_iter()
                .chain(second.unobserved)
                .collect(),
        }
    }

    /// One stop after the other: on a shared link they could not overlap anyway.
    fn stop_budget(&self) -> Duration {
        self.first.stop_budget() + self.second.stop_budget()
    }

    /// Both chains' reads, the first's axes then the second's. Reading moves nothing.
    fn read_positions(&mut self) -> Axes<AxisReading> {
        let mut readings = self.first.read_positions();
        readings.extend(self.second.read_positions());
        readings
    }

    fn unavailable(&self) -> Vec<usize> {
        let first = self.first.axes().len();
        let mut unavailable = self.first.unavailable();
        unavailable.extend(
            self.second
                .unavailable()
                .into_iter()
                .map(|index| first + index),
        );
        unavailable
    }

    fn command_readings(&self) -> Option<Axes<AxisReading>> {
        let mut readings = self.first.command_readings()?;
        readings.extend(self.second.command_readings()?);
        Some(readings)
    }

    fn last_observed(&self) -> Option<Axes<f64>> {
        let mut observed = self.first.last_observed()?;
        observed.extend(self.second.last_observed()?);
        Some(observed)
    }

    fn clear_faults(&mut self) {
        self.first.clear_faults();
        self.second.clear_faults();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::binding::{
        ActuatorBinding, ActuatorBus, ActuatorModel, UNCALIBRATED_DIRECTION, UNCALIBRATED_ZERO,
    };

    #[derive(Debug, Error)]
    #[error("{0:?}")]
    struct Failed(ChainFailure);

    impl ChainError for Failed {
        fn failure(&self) -> ChainFailure {
            self.0
        }
    }

    /// A one-axis chain whose commands and reads succeed or fail as configured.
    struct Fake {
        axes: Vec<BoundAxis>,
        commandable: Vec<CommandRange>,
        fails: Option<ChainFailure>,
    }

    fn fake(fails: Option<ChainFailure>) -> Fake {
        Fake {
            axes: vec![BoundAxis {
                joint: "joint".into(),
                port: "port".into(),
                lower: -1.0,
                upper: 1.0,
                binding: ActuatorBinding {
                    model: ActuatorModel::new("m"),
                    bus: ActuatorBus::new("b"),
                    bus_id: 1,
                    sign: 1,
                    reduction: 1.0,
                },
                zero: UNCALIBRATED_ZERO,
                direction: UNCALIBRATED_DIRECTION,
            }],
            commandable: vec![CommandRange {
                lower: -1.0,
                upper: 1.0,
            }],
            fails,
        }
    }

    impl ActuatorChain for Fake {
        type Error = Failed;
        type Feedback = ();

        fn axes(&self) -> &[BoundAxis] {
            &self.axes
        }

        fn commandable(&self) -> &[CommandRange] {
            &self.commandable
        }

        fn command(&mut self, joints: &[f64]) -> Result<Receipt<()>, Failed> {
            self.fails.map_or_else(
                || {
                    Ok(Receipt {
                        evidence: Evidence::BusObservedComplete,
                        observed: crate::axes::Axes::from_slice(joints),
                        feedback: [()].into_iter().collect(),
                    })
                },
                |failure| Err(Failed(failure)),
            )
        }

        fn read_positions(&mut self) -> Axes<AxisReading> {
            Axes::from_slice(&[if self.fails.is_some() {
                AxisReading::Missing(crate::chain::MissingReason::Timeout)
            } else {
                AxisReading::Fresh {
                    position: 0.5,
                    received: std::time::Instant::now(),
                }
            }])
        }

        fn stop(&mut self) -> StopReport {
            StopReport::default()
        }

        fn stop_budget(&self) -> Duration {
            Duration::ZERO
        }
    }

    #[test]
    fn a_read_keeps_both_chains_axes_but_a_late_command_failure_is_a_fault() {
        let mut both = Composite::new(fake(None), fake(None));
        assert_eq!(
            crate::chain::fresh_positions(&both.read_positions()),
            Some(vec![0.5, 0.5])
        );
        let mut second = Composite::new(fake(None), fake(Some(ChainFailure::Timeout)));
        let read = second.read_positions();
        assert!(read[0].is_fresh());
        assert_eq!(
            read[1],
            AxisReading::Missing(crate::chain::MissingReason::Timeout)
        );
        let command = second.command(&[0.0, 0.0]).expect_err("second is silent");
        assert_eq!(command.failure(), ChainFailure::Fault);
    }
}
