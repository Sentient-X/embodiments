//! A chain whose body keeps axes its actuators cannot drive: unavailable axes stay in the
//! vector, as data, and are never commanded.
//!
//! An actuator found faulty (the static upper body's torso motor, 2026-10-26) is marked
//! unavailable by the station's commissioning. Its axis keeps its place in the native state
//! order, so the layout an episode records, a policy reads and a leader maps never shrinks or
//! reorders: every read reports it [`MissingReason::OutOfService`], every receipt observes it as
//! `NaN`, its target is dropped before the bus, and [`ActuatorChain::unavailable`] names it, so a
//! [`crate::Governor`] neither steps nor tracks it. The chain underneath drives the rest, opened
//! without it, so the faulty actuator is never addressed and cannot fail the open.

use std::time::Duration;

use crate::axes::Axes;
use crate::binding::BoundAxis;
use crate::chain::{
    ActuatorChain, AxisReading, CommandRange, Feedforward, FeedforwardLimit, MissingReason,
    Observe, Receipt, StopReport, TargetError, check_targets,
};
use crate::rail::AxisDerate;

/// `inner`'s axes inside a body's full vector, the rest unavailable.
pub struct Masked<C, F = ()> {
    inner: C,
    axes: Vec<BoundAxis>,
    commandable: Vec<CommandRange>,
    /// The full vector's index of each of `inner`'s axes, in order.
    present: Vec<usize>,
    unavailable: Vec<usize>,
    /// `inner`'s targets, refilled each command.
    targets: Axes<f64>,
    /// The feedback an unavailable axis stands as, when set: then a receipt's feedback is one
    /// per axis, as every chain's is; otherwise it is `inner`'s alone.
    unanswered: Option<fn(&BoundAxis) -> F>,
}

impl<C, F> Masked<C, F> {
    /// `inner`, over `inner_axes` with `inner_ranges`, placed among `axes`, the ones at
    /// `unavailable` left out.
    fn place(
        inner: C,
        inner_axes: &[BoundAxis],
        inner_ranges: &[CommandRange],
        axes: Vec<BoundAxis>,
        unavailable: &[usize],
    ) -> Result<Self, TargetError> {
        let present: Vec<usize> = (0..axes.len())
            .filter(|index| !unavailable.contains(index))
            .collect();
        if unavailable.iter().any(|index| *index >= axes.len())
            || present.len() != inner_axes.len()
            || present
                .iter()
                .zip(inner_axes)
                .any(|(index, axis)| axes[*index].joint != axis.joint)
        {
            return Err(TargetError::Width {
                expected: axes.len(),
                actual: inner_axes.len(),
            });
        }
        let mut commandable: Vec<CommandRange> = axes
            .iter()
            .map(|axis| CommandRange {
                lower: axis.lower,
                upper: axis.upper,
            })
            .collect();
        for (index, range) in present.iter().zip(inner_ranges) {
            commandable[*index] = *range;
        }
        let mut unavailable = unavailable.to_vec();
        unavailable.sort_unstable();
        unavailable.dedup();
        let targets = Axes::new();
        Ok(Self {
            inner,
            axes,
            commandable,
            present,
            unavailable,
            targets,
            unanswered: None,
        })
    }

    /// The chain or observer behind the available axes.
    pub const fn inner(&self) -> &C {
        &self.inner
    }

    /// `inner`'s values spread into the full vector, `fill` at every unavailable axis.
    fn spread<T: Copy + Default>(&self, values: &[T], fill: T) -> Axes<T> {
        let mut whole = Axes::filled(fill, self.axes.len());
        for (index, value) in self.present.iter().zip(values) {
            whole[*index] = *value;
        }
        whole
    }
}

impl<C: ActuatorChain> Masked<C, C::Feedback> {
    /// `inner`, which drives `axes` without the axes at `unavailable` (indices into `axes`), as
    /// the whole vector. An unavailable axis is commandable within its declared bounds, which
    /// only its target's check uses.
    ///
    /// # Errors
    ///
    /// Refuses an index outside `axes`, and an `inner` whose axes are not the rest of `axes`
    /// in order.
    pub fn new(inner: C, axes: Vec<BoundAxis>, unavailable: &[usize]) -> Result<Self, TargetError> {
        let inner_axes = inner.axes().to_vec();
        let inner_ranges = inner.commandable().to_vec();
        Self::place(inner, &inner_axes, &inner_ranges, axes, unavailable)
    }

    /// This chain with each unavailable axis's feedback in its place, as `unanswered` makes
    /// it, so every receipt's feedback is one per axis.
    #[must_use]
    pub fn answering(mut self, unanswered: fn(&BoundAxis) -> C::Feedback) -> Self {
        self.unanswered = Some(unanswered);
        self
    }
}

impl<C: Observe> Masked<C> {
    /// An observer of `axes` reading every axis but those at `unavailable` through `inner`,
    /// which reports them as [`MissingReason::OutOfService`].
    ///
    /// # Errors
    ///
    /// As [`Masked::new`].
    pub fn observing(
        inner: C,
        axes: Vec<BoundAxis>,
        unavailable: &[usize],
    ) -> Result<Self, TargetError> {
        let inner_axes = inner.axes().to_vec();
        Self::place(inner, &inner_axes, &[], axes, unavailable)
    }
}

/// A boxed observer, as a body holds one per bus.
impl<O: Observe + ?Sized> Observe for Box<O> {
    fn axes(&self) -> &[BoundAxis] {
        (**self).axes()
    }

    fn read_positions(&mut self) -> Axes<AxisReading> {
        (**self).read_positions()
    }
}

impl<C: Observe, F> Observe for Masked<C, F> {
    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    fn read_positions(&mut self) -> Axes<AxisReading> {
        let readings = self.inner.read_positions();
        self.spread(&readings, AxisReading::Missing(MissingReason::OutOfService))
    }
}

impl<C: ActuatorChain> ActuatorChain for Masked<C, C::Feedback> {
    type Error = C::Error;
    type Feedback = C::Feedback;

    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    fn commandable(&self) -> &[CommandRange] {
        &self.commandable
    }

    /// The available axes' targets to `inner`; an unavailable axis's is dropped unchecked.
    fn command(&mut self, joints: &[f64]) -> Result<Receipt<C::Feedback>, C::Error> {
        self.drive(joints, None)
    }

    /// As [`ActuatorChain::command`], each available axis with its feed-forward.
    fn command_with(
        &mut self,
        joints: &[f64],
        feedforward: &[Feedforward],
    ) -> Result<Receipt<C::Feedback>, C::Error> {
        self.drive(joints, Some(feedforward))
    }

    /// The available axes' bounds to `inner`.
    fn set_feedforward_limits(&mut self, limits: &[FeedforwardLimit]) -> Result<(), C::Error> {
        let kept: Vec<FeedforwardLimit> = self
            .present
            .iter()
            .filter_map(|index| limits.get(*index).copied())
            .collect();
        self.inner.set_feedforward_limits(&kept)
    }

    fn read_positions(&mut self) -> Axes<AxisReading> {
        let readings = self.inner.read_positions();
        self.spread(&readings, AxisReading::Missing(MissingReason::OutOfService))
    }
    fn stop(&mut self) -> StopReport {
        self.inner.stop()
    }

    fn stop_budget(&self) -> Duration {
        self.inner.stop_budget()
    }

    /// `inner`'s, with `NaN` at every unavailable axis.
    fn last_observed(&self) -> Option<Axes<f64>> {
        Some(self.spread(&self.inner.last_observed()?, f64::NAN))
    }

    fn command_readings(&self) -> Option<Axes<AxisReading>> {
        Some(self.spread(
            &self.inner.command_readings()?,
            AxisReading::Missing(MissingReason::OutOfService),
        ))
    }

    fn unavailable(&self) -> Vec<usize> {
        self.unavailable.clone()
    }

    /// `inner`'s, placed in the full vector.
    fn derated(&self) -> Vec<AxisDerate> {
        self.inner
            .derated()
            .into_iter()
            .map(|derate| AxisDerate {
                axis: self.present[derate.axis],
                ..derate
            })
            .collect()
    }

    fn clear_faults(&mut self) {
        self.inner.clear_faults();
    }
}

impl<C: ActuatorChain> Masked<C, C::Feedback> {
    /// Command the available axes, with their feed-forward where given.
    fn drive(
        &mut self,
        joints: &[f64],
        feedforward: Option<&[Feedforward]>,
    ) -> Result<Receipt<C::Feedback>, C::Error> {
        self.targets.clear();
        self.targets.extend(
            self.present
                .iter()
                .map(|index| joints.get(*index).copied().unwrap_or(f64::NAN)),
        );
        // A short vector leaves `NaN` where a target is missing, which `inner` refuses.
        let receipt = match feedforward {
            Some(all) => {
                let kept: Axes<Feedforward> = self
                    .present
                    .iter()
                    .map(|index| all.get(*index).copied().unwrap_or_default())
                    .collect();
                self.inner.command_with(&self.targets, &kept)?
            }
            None => self.inner.command(&self.targets)?,
        };
        Ok(Receipt {
            evidence: receipt.evidence,
            observed: if receipt.observed.is_empty() {
                Axes::new()
            } else {
                self.spread(&receipt.observed, f64::NAN)
            },
            feedback: match self.unanswered {
                Some(unanswered) if receipt.feedback.len() == self.present.len() => {
                    let mut whole: Axes<C::Feedback> = Axes::new();
                    let mut answers = receipt.feedback.into_iter();
                    for (index, axis) in self.axes.iter().enumerate() {
                        let answer = if self.unavailable.contains(&index) {
                            None
                        } else {
                            answers.next()
                        };
                        whole.push(answer.unwrap_or_else(|| unanswered(axis)));
                    }
                    whole
                }
                _ => receipt.feedback,
            },
        })
    }
}

/// Refuse a vector of the wrong width or outside the declared bounds, skipping `unavailable`
/// axes: their targets reach no actuator. This is for a position stated outright (where a
/// body rests, [`crate::Governor::start_at`]); a command's target is held to
/// [`crate::CommandRange::admissible`] instead.
///
/// # Errors
///
/// The width mismatch, or the first available axis' target outside its bounds.
pub fn check_available(
    axes: &[BoundAxis],
    unavailable: &[usize],
    joints: &[f64],
) -> Result<(), TargetError> {
    if axes.len() != joints.len() {
        return check_targets(axes, joints);
    }
    for (index, (axis, joint)) in axes.iter().zip(joints).enumerate() {
        if !unavailable.contains(&index) && !axis.admits(*joint) {
            return Err(TargetError::OutOfBounds {
                joint: axis.joint.clone(),
                value: *joint,
            });
        }
    }
    Ok(())
}
