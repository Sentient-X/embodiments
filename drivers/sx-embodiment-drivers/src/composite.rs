//! Two chains driven as one: a body whose native state order spans two devices or two links.
//!
//! The vector is the first chain's axes, then the second's. Every target is checked against both
//! chains' commandable ranges before either is written; the first is commanded, then the second.
//! A second chain that fails after the first applied is a [`ChainFailure::Fault`] whatever it
//! reported, since one device already holds its new target. A stop stops both, whatever the first
//! answered, and is proven only when both are.

use std::time::Duration;

use thiserror::Error;

use crate::binding::BoundAxis;
use crate::chain::{
    ActuatorChain, ChainError, ChainFailure, CommandRange, Evidence, Receipt, StopReport,
    TargetError,
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
        if joints.len() != self.axes.len() {
            return Err(TargetError::Width {
                expected: self.axes.len(),
                actual: joints.len(),
            }
            .into());
        }
        if let Some((axis, value)) = self
            .axes
            .iter()
            .zip(&self.commandable)
            .zip(joints)
            .find_map(|((axis, range), value)| (!range.admits(*value)).then_some((axis, *value)))
        {
            return Err(TargetError::OutOfBounds {
                joint: axis.joint.clone(),
                value,
            }
            .into());
        }
        let (head, tail) = joints.split_at(self.first.axes().len());
        let first = self.first.command(head).map_err(CompositeError::First)?;
        let second = self.second.command(tail).map_err(CompositeError::Second)?;
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

    fn stop(&mut self) -> StopReport {
        let first = self.first.stop();
        let second = self.second.stop();
        StopReport {
            unproven: first.unproven.into_iter().chain(second.unproven).collect(),
            faults: first.faults.into_iter().chain(second.faults).collect(),
        }
    }

    /// One stop after the other: on a shared link they could not overlap anyway.
    fn stop_budget(&self) -> Duration {
        self.first.stop_budget() + self.second.stop_budget()
    }

    fn clear_faults(&mut self) {
        self.first.clear_faults();
        self.second.clear_faults();
    }
}
