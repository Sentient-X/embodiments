//! Any number of chains driven as one: a body whose native state order spans several buses.
//!
//! [`Composite`](crate::Composite) is two chains of known types; [`Chains`] is the same rule
//! over a list, for a body whose buses are read from its bindings, each chain usually boxed
//! behind one error and feedback type. The vector is each chain's axes in list order. Every
//! target is checked against every chain's commandable ranges before any chain is written;
//! then each chain is commanded in order. A chain that fails after an earlier one applied is a
//! [`ChainFailure::Fault`] whatever it reported, since that earlier device already holds its
//! new target. A stop stops every chain, whatever an earlier one answered, and is proven only
//! when every chain's is. Its budget is the sum of theirs: the chains are stopped one after the
//! other.
//!
//! [`ActuatorChain::last_observed`] is every chain's in the same order, or none when any chain
//! has none, so a [`Governor`](crate::Governor) around the whole starts every axis from where it
//! rests or refuses. A receipt's evidence is the weakest any chain established. Its observed joints and
//! feedback are the chains' in order only when every chain reported a value for each of its
//! axes; otherwise both are empty, since a vector missing one chain's values would index other
//! joints than the state order says.

use std::time::Duration;

use thiserror::Error;

use crate::binding::BoundAxis;
use crate::chain::{
    ActuatorChain, ChainError, ChainFailure, CommandRange, Evidence, Receipt, StopReport,
    TargetError,
};

/// Chains in native state order, driven as one.
pub struct Chains<C> {
    members: Vec<C>,
    axes: Vec<BoundAxis>,
    commandable: Vec<CommandRange>,
}

#[derive(Debug, Error)]
pub enum ChainsError<E> {
    #[error(transparent)]
    Target(#[from] TargetError),
    /// Chain `index` failed; `applied` when an earlier chain already holds its new target.
    #[error("chain {index}: {error}{}", if *applied { " (after an earlier chain applied)" } else { "" })]
    Chain {
        index: usize,
        applied: bool,
        error: E,
    },
}

impl<E: ChainError> ChainError for ChainsError<E> {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::Target(_) => ChainFailure::Refused,
            Self::Chain { applied: true, .. } => ChainFailure::Fault,
            Self::Chain { error, .. } => error.failure(),
        }
    }
}

impl<C: ActuatorChain> Chains<C> {
    #[must_use]
    pub fn new(chains: Vec<C>) -> Self {
        let axes = chains
            .iter()
            .flat_map(|chain| chain.axes().iter().cloned())
            .collect();
        let commandable = chains
            .iter()
            .flat_map(|chain| chain.commandable().iter().copied())
            .collect();
        Self {
            members: chains,
            axes,
            commandable,
        }
    }

    /// The chains, in native state order.
    #[must_use]
    pub fn chains(&self) -> &[C] {
        &self.members
    }
}

impl<C: ActuatorChain> ActuatorChain for Chains<C> {
    type Error = ChainsError<C::Error>;
    type Feedback = C::Feedback;

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
        let mut evidence = Evidence::BusObservedComplete;
        let mut observed = Vec::with_capacity(joints.len());
        let mut feedback = Vec::with_capacity(joints.len());
        let (mut observed_whole, mut feedback_whole) = (true, true);
        let mut rest = joints;
        for (index, chain) in self.members.iter_mut().enumerate() {
            let width = chain.axes().len();
            let (head, tail) = rest.split_at(width);
            rest = tail;
            let receipt = chain.command(head).map_err(|error| ChainsError::Chain {
                index,
                applied: index > 0,
                error,
            })?;
            if receipt.evidence == Evidence::DispatchAttempted {
                evidence = Evidence::DispatchAttempted;
            }
            observed_whole &= receipt.observed.len() == width;
            feedback_whole &= receipt.feedback.len() == width;
            observed.extend(receipt.observed);
            feedback.extend(receipt.feedback);
        }
        if !observed_whole {
            observed.clear();
        }
        if !feedback_whole {
            feedback.clear();
        }
        Ok(Receipt {
            evidence,
            observed,
            feedback,
        })
    }

    fn stop(&mut self) -> StopReport {
        let mut report = StopReport::default();
        for chain in &mut self.members {
            let stopped = chain.stop();
            report.unproven.extend(stopped.unproven);
            report.faults.extend(stopped.faults);
        }
        report
    }

    /// Every chain's, in native state order, or `None` when any chain has none.
    fn last_observed(&self) -> Option<Vec<f64>> {
        let mut observed = Vec::with_capacity(self.axes.len());
        for chain in &self.members {
            observed.extend(chain.last_observed()?);
        }
        Some(observed)
    }

    /// The chain carrying `axis` answers for it.
    fn approach_speed_limited(&self, axis: usize) -> bool {
        let mut first = 0;
        for chain in &self.members {
            let width = chain.axes().len();
            if axis < first + width {
                return chain.approach_speed_limited(axis - first);
            }
            first += width;
        }
        false
    }

    /// One stop after the other.
    fn stop_budget(&self) -> Duration {
        self.members
            .iter()
            .map(ActuatorChain::stop_budget)
            .fold(Duration::ZERO, Duration::saturating_add)
    }

    fn clear_faults(&mut self) {
        for chain in &mut self.members {
            chain.clear_faults();
        }
    }
}

/// A boxed chain is its chain, so a list of differently built chains can share one type.
impl<C: ActuatorChain + ?Sized> ActuatorChain for Box<C> {
    type Error = C::Error;
    type Feedback = C::Feedback;

    fn axes(&self) -> &[BoundAxis] {
        (**self).axes()
    }

    fn commandable(&self) -> &[CommandRange] {
        (**self).commandable()
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt<Self::Feedback>, Self::Error> {
        (**self).command(joints)
    }

    fn stop(&mut self) -> StopReport {
        (**self).stop()
    }

    fn stop_budget(&self) -> Duration {
        (**self).stop_budget()
    }

    fn last_observed(&self) -> Option<Vec<f64>> {
        (**self).last_observed()
    }

    fn approach_speed_limited(&self, axis: usize) -> bool {
        (**self).approach_speed_limited(axis)
    }

    fn clear_faults(&mut self) {
        (**self).clear_faults();
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

    /// A chain that records what it was sent and answers as configured.
    struct Fake {
        axes: Vec<BoundAxis>,
        commandable: Vec<CommandRange>,
        answers: bool,
        fails: Option<ChainFailure>,
        sent: Vec<Vec<f64>>,
        stops: usize,
        unproven: Vec<u16>,
        budget: Duration,
    }

    fn fake(port: &str, ids: &[u16], answers: bool) -> Fake {
        let axes: Vec<BoundAxis> = ids
            .iter()
            .map(|id| BoundAxis {
                joint: format!("{port}{id}"),
                port: port.to_owned(),
                lower: -1.0,
                upper: 1.0,
                binding: ActuatorBinding {
                    model: ActuatorModel::new("m"),
                    bus: ActuatorBus::new("b"),
                    bus_id: *id,
                    sign: 1,
                    reduction: 1.0,
                },
                zero: UNCALIBRATED_ZERO,
                direction: UNCALIBRATED_DIRECTION,
            })
            .collect();
        let commandable = axes
            .iter()
            .map(|_| CommandRange {
                lower: -0.5,
                upper: 0.5,
            })
            .collect();
        Fake {
            axes,
            commandable,
            answers,
            fails: None,
            sent: Vec::new(),
            stops: 0,
            unproven: Vec::new(),
            budget: Duration::from_millis(10),
        }
    }

    impl ActuatorChain for Fake {
        type Error = Failed;
        type Feedback = u16;

        fn axes(&self) -> &[BoundAxis] {
            &self.axes
        }

        fn commandable(&self) -> &[CommandRange] {
            &self.commandable
        }

        fn command(&mut self, joints: &[f64]) -> Result<Receipt<u16>, Failed> {
            if let Some(failure) = self.fails {
                return Err(Failed(failure));
            }
            self.sent.push(joints.to_vec());
            Ok(if self.answers {
                Receipt {
                    evidence: Evidence::BusObservedComplete,
                    observed: joints.to_vec(),
                    feedback: self.axes.iter().map(|axis| axis.binding.bus_id).collect(),
                }
            } else {
                Receipt {
                    evidence: Evidence::DispatchAttempted,
                    observed: Vec::new(),
                    feedback: Vec::new(),
                }
            })
        }

        fn stop(&mut self) -> StopReport {
            self.stops += 1;
            StopReport {
                unproven: self.unproven.clone(),
                faults: Vec::new(),
            }
        }

        fn stop_budget(&self) -> Duration {
            self.budget
        }

        fn last_observed(&self) -> Option<Vec<f64>> {
            self.answers.then(|| vec![0.25; self.axes.len()])
        }
    }

    #[test]
    fn targets_split_across_chains_in_order_and_receipts_stay_aligned() {
        let mut chains = Chains::new(vec![fake("can", &[1, 2], true), fake("neck", &[1], true)]);
        assert_eq!(chains.axes().len(), 3);
        let receipt = chains.command(&[0.1, 0.2, 0.3]).expect("commanded");
        assert_eq!(receipt.evidence, Evidence::BusObservedComplete);
        assert_eq!(receipt.observed, [0.1, 0.2, 0.3]);
        assert_eq!(receipt.feedback, [1, 2, 1]);
        assert_eq!(chains.chains()[1].sent, [[0.3]]);
        // A chain that answers nothing leaves no partial vector.
        let mut mixed = Chains::new(vec![fake("can", &[1, 2], true), fake("neck", &[1], false)]);
        let receipt = mixed.command(&[0.1, 0.2, 0.3]).expect("commanded");
        assert_eq!(receipt.evidence, Evidence::DispatchAttempted);
        assert!(receipt.observed.is_empty() && receipt.feedback.is_empty());
    }

    #[test]
    fn a_refused_target_reaches_no_chain_and_a_late_failure_is_a_fault() {
        let mut chains = Chains::new(vec![fake("can", &[1], true), fake("neck", &[1], true)]);
        let refused = chains.command(&[0.1, 0.9]).expect_err("out of range");
        assert_eq!(refused.failure(), ChainFailure::Refused);
        assert!(matches!(
            chains.command(&[0.1]),
            Err(ChainsError::Target(TargetError::Width { .. }))
        ));
        assert!(chains.chains().iter().all(|chain| chain.sent.is_empty()));
        let mut first = fake("can", &[1], true);
        first.fails = Some(ChainFailure::Timeout);
        let mut chains = Chains::new(vec![first, fake("neck", &[1], true)]);
        let error = chains.command(&[0.0, 0.0]).expect_err("first fails");
        assert_eq!(error.failure(), ChainFailure::Timeout);
        assert!(chains.chains()[1].sent.is_empty());
        let mut second = fake("neck", &[1], true);
        second.fails = Some(ChainFailure::Refused);
        let mut chains = Chains::new(vec![fake("can", &[1], true), second]);
        let error = chains.command(&[0.0, 0.0]).expect_err("second fails");
        assert_eq!(error.failure(), ChainFailure::Fault);
    }

    #[test]
    fn a_stop_reaches_every_chain_and_is_proven_only_by_all() {
        let mut stuck = fake("can", &[1, 2], true);
        stuck.unproven = vec![2];
        let mut neck = fake("neck", &[1], false);
        neck.budget = Duration::from_millis(30);
        let mut chains = Chains::new(vec![stuck, neck]);
        assert_eq!(chains.stop_budget(), Duration::from_millis(40));
        let report = chains.stop();
        assert_eq!(report.unproven, [2]);
        assert!(chains.chains().iter().all(|chain| chain.stops == 1));
        // The neck observes nothing here, so the whole has no resting position to start from.
        assert_eq!(chains.last_observed(), None);
        let both = Chains::new(vec![fake("can", &[1, 2], true), fake("neck", &[1], true)]);
        assert_eq!(both.last_observed(), Some(vec![0.25; 3]));
        let mut boxed: Vec<Box<dyn ActuatorChain<Error = Failed, Feedback = u16>>> = vec![
            Box::new(fake("can", &[1], true)),
            Box::new(fake("neck", &[1], true)),
        ];
        assert!(boxed.iter_mut().all(|chain| chain.stop().is_proven()));
        assert!(Chains::new(boxed).stop().is_proven());
    }
}
