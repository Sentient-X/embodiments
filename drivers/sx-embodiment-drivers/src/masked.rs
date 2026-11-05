//! A chain whose body keeps axes its actuators cannot drive: unavailable axes stay in the
//! vector, as data, and are never commanded.
//!
//! An actuator found faulty (the static upper body's torso motor, 2026-10-26) is marked
//! unavailable by the station's commissioning. Its axis keeps its place in the native state
//! order, so the layout an episode records, a policy reads and a leader maps never shrinks or
//! reorders: its target is dropped before the bus, and [`ActuatorChain::unavailable`] names it,
//! so a [`crate::Governor`] neither steps nor tracks it. The chain underneath drives the rest,
//! opened without it, so the faulty actuator is never commanded and cannot fail the open.
//!
//! **An out-of-service actuator is still a sensor.** Where the body gives one, a
//! [`ReadOnly`] set reads the unavailable axes ([`Masked::reading`]): a motor that answers is
//! read [`AxisReading::Fresh`], so forward kinematics through it (the static upper body's head
//! camera and both tools hang off its torso) places what it carries; one that does not reads
//! [`MissingReason::OutOfService`], never an error. An out-of-service actuator may be faulty,
//! so a fresh reading is taken only when plausible (inside its axis's bounds, no faster than
//! its rated speed from the last one taken, its scale its model's own); one that is not reads
//! [`MissingReason::Unavailable`]. The split is in the types: the commanded axes are `inner`,
//! an [`ActuatorChain`]; the read-only ones are a boxed [`Observe`], whose whole surface is its
//! axes and a read, and nothing on a command, stop or fault path here touches it. Each
//! vendor's observer speaks through a port that refuses every frame but a read and, for a
//! Damiao motor out of service, its disable (sent once at open and whenever it answers
//! enabled: a status refresh may feed the watchdog of a motor some crashed run left enabled),
//! so no command reaches a read-only actuator even by its bytes.
//!
//! A read costs the bus one request per read-only actuator and waits for its answer (a Damiao
//! one at most its bus time and 6 ms), so on the command path it is taken at most every
//! [`SENSE_PERIOD`]; after a read that missed it waits out a back-off ([`SENSE_RETRY`],
//! doubling to [`SENSE_RETRY_MAX`]) before asking again, so a dead motor costs a control loop
//! one short answer wait now and then, not every tick.

use std::time::{Duration, Instant};

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
    /// What every read reports at an unavailable axis: [`MissingReason::OutOfService`] unless
    /// [`Masked::missing_as`] says otherwise.
    missing: MissingReason,
    /// The unavailable axes still read, when the body gives an observer of them.
    read_only: Option<ReadOnly>,
}

/// How often the command path reads the [`ReadOnly`] axes while they answer.
pub const SENSE_PERIOD: Duration = Duration::from_millis(100);
/// How long after a read that missed the read-only axes are asked again, at first.
pub const SENSE_RETRY: Duration = Duration::from_secs(1);
/// The longest the back-off after misses in a row grows to.
pub const SENSE_RETRY_MAX: Duration = Duration::from_secs(32);

/// The unavailable axes of a [`Masked`] chain that are still read, through an observer: never
/// commanded, enabled, disabled, stopped or cleared. Its type has nothing but a read to call.
pub struct ReadOnly {
    observer: Box<dyn Observe + Send>,
    /// Each of the observer's axes' index in the whole vector.
    at: Vec<usize>,
    /// The last read, one per observer axis; a reading that is not fresh is out of service.
    last: Axes<AxisReading>,
    /// When the observer was last asked.
    asked: Option<Instant>,
    /// How long after a read that missed it is asked again.
    retry: Duration,
    /// How many of its actuators answered the last read, plausibly or not: what the back-off
    /// goes by, since an implausible answer came as quickly as a good one.
    heard: usize,
    /// Per observer axis, the last reading taken as where it is, and the last one refused
    /// as a jump it could not make ([`ReadOnly::plausible`]).
    trusted: Vec<Option<(f64, Instant)>>,
    doubted: Vec<Option<f64>>,
}

/// How far past its declared bounds, or past where its speed could take it, an out-of-service
/// reading may lie and still be taken: two readings' noise.
const PLAUSIBLE_SLACK: f64 = 2.0 * crate::chain::READ_NOISE_RAD;

impl ReadOnly {
    /// Whether every read-only actuator answered the last read.
    fn answered(&self) -> bool {
        self.asked.is_some() && self.heard == self.at.len()
    }

    /// Read the observer when it is due: at `period` after the last read while every axis
    /// answers, else after the back-off, which doubles with each read in a row that nothing
    /// answered.
    fn read_if_due(&mut self, period: Duration) {
        let now = Instant::now();
        let answered = self.answered();
        let wait = if answered { period } else { self.retry };
        if self
            .asked
            .is_some_and(|asked| now.saturating_duration_since(asked) < wait)
        {
            return;
        }
        let missed_before = self.asked.is_some() && self.heard == 0;
        let readings = self.observer.read_positions();
        self.asked = Some(Instant::now());
        let whole = readings.len() == self.at.len();
        self.heard = if whole {
            readings.iter().filter(|reading| reading.is_fresh()).count()
        } else {
            0
        };
        self.last.clear();
        for index in 0..self.at.len() {
            let reading = match readings.get(index) {
                Some(reading) if whole => self.plausible(index, *reading),
                _ => AxisReading::Missing(MissingReason::OutOfService),
            };
            self.last.push(reading);
        }
        self.retry = if self.heard > 0 || !missed_before {
            SENSE_RETRY
        } else {
            (self.retry * 2).min(SENSE_RETRY_MAX)
        };
    }

    /// `reading` of observer axis `index`, as it may reach forward kinematics and a solve. An
    /// out-of-service actuator may be faulty, so a fresh reading is taken only when it is
    /// plausible: inside its axis's declared bounds, and no further from the last one taken
    /// than its rated speed could carry it since (or confirming the last one refused, which
    /// was then a real move). An implausible reading, or one whose scale the observer refused
    /// ([`MissingReason::Corrupt`]), is [`MissingReason::Unavailable`]; one that did not
    /// answer is out of service.
    fn plausible(&mut self, index: usize, reading: AxisReading) -> AxisReading {
        let unavailable = AxisReading::Missing(MissingReason::Unavailable);
        let AxisReading::Fresh { position, received } = reading else {
            return if reading == AxisReading::Missing(MissingReason::Corrupt) {
                unavailable
            } else {
                AxisReading::Missing(MissingReason::OutOfService)
            };
        };
        let axis = &self.observer.axes()[index];
        if !position.is_finite()
            || position < axis.lower - PLAUSIBLE_SLACK
            || position > axis.upper + PLAUSIBLE_SLACK
        {
            return unavailable;
        }
        let reach = |since: Instant| {
            axis.motion.speed * received.saturating_duration_since(since).as_secs_f64()
                + PLAUSIBLE_SLACK
        };
        let taken = match self.trusted[index] {
            None => true,
            Some((at, since)) => (position - at).abs() <= reach(since),
        } || self.doubted[index]
            .is_some_and(|doubted| (position - doubted).abs() <= PLAUSIBLE_SLACK);
        if taken {
            self.trusted[index] = Some((position, received));
            self.doubted[index] = None;
            reading
        } else {
            self.doubted[index] = Some(position);
            unavailable
        }
    }

    /// The last read at each read-only axis, over `whole`.
    fn place(&self, whole: &mut [AxisReading]) {
        for (index, reading) in self.at.iter().zip(&self.last) {
            if let Some(slot) = whole.get_mut(*index) {
                *slot = *reading;
            }
        }
    }

    /// The last read position at each read-only axis that answered, over `whole`.
    fn place_positions(&self, whole: &mut [f64]) {
        for (index, reading) in self.at.iter().zip(&self.last) {
            if let (Some(slot), Some(position)) = (whole.get_mut(*index), reading.position()) {
                *slot = position;
            }
        }
    }
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
            missing: MissingReason::OutOfService,
            read_only: None,
        })
    }

    /// This chain with `observer` reading its unavailable axes ([`ReadOnly`]): its axes must be
    /// the unavailable ones, in order. It is only read, never commanded.
    ///
    /// # Errors
    ///
    /// [`TargetError::Width`] when `observer`'s axes are not this chain's unavailable axes.
    pub fn reading(mut self, observer: Box<dyn Observe + Send>) -> Result<Self, TargetError> {
        let unavailable: Vec<&str> = self
            .unavailable
            .iter()
            .map(|index| self.axes[*index].joint.as_str())
            .collect();
        let read: Vec<&str> = observer
            .axes()
            .iter()
            .map(|axis| axis.joint.as_str())
            .collect();
        if unavailable != read {
            return Err(TargetError::Width {
                expected: unavailable.len(),
                actual: read.len(),
            });
        }
        let count = self.unavailable.len();
        self.read_only = Some(ReadOnly {
            observer,
            at: self.unavailable.clone(),
            last: Axes::new(),
            asked: None,
            retry: SENSE_RETRY,
            heard: 0,
            trusted: vec![None; count],
            doubted: vec![None; count],
        });
        Ok(self)
    }

    /// Whether this chain reads its unavailable axes ([`Masked::reading`]).
    #[must_use]
    pub const fn reads_unavailable(&self) -> bool {
        self.read_only.is_some()
    }

    /// This chain with every read reporting `reason` at its unavailable axes: a bus that did
    /// not open reads [`MissingReason::Unavailable`], not out of service.
    #[must_use]
    pub fn missing_as(mut self, reason: MissingReason) -> Self {
        self.missing = reason;
        self
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

    /// `inner`'s readings spread into the full vector, each read-only axis at its last read.
    fn spread_readings(&self, readings: &[AxisReading]) -> Axes<AxisReading> {
        let mut whole = self.spread(readings, AxisReading::Missing(self.missing));
        if let Some(read_only) = &self.read_only {
            read_only.place(&mut whole);
        }
        whole
    }

    /// `inner`'s positions spread into the full vector, each read-only axis at its last read
    /// position, `NaN` where it has none.
    fn spread_positions(&self, positions: &[f64]) -> Axes<f64> {
        let mut whole = self.spread(positions, f64::NAN);
        if let Some(read_only) = &self.read_only {
            read_only.place_positions(&mut whole);
        }
        whole
    }

    /// Read the read-only axes when due ([`SENSE_PERIOD`] apart on the command path, `ZERO`
    /// on a read).
    fn sense(&mut self, period: Duration) {
        if let Some(read_only) = self.read_only.as_mut() {
            read_only.read_if_due(period);
        }
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

    fn sent_disables(&self) -> Vec<u16> {
        (**self).sent_disables()
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
        self.sense(Duration::ZERO);
        self.spread_readings(&readings)
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

    /// `inner`'s readings, and the read-only axes' own read.
    fn read_positions(&mut self) -> Axes<AxisReading> {
        let readings = self.inner.read_positions();
        self.sense(Duration::ZERO);
        self.spread_readings(&readings)
    }

    /// `inner`'s stop alone: a read-only actuator was never enabled here; its observer
    /// disabled it at open and disables it whenever it answers enabled.
    fn stop(&mut self) -> StopReport {
        self.inner.stop()
    }

    /// The available axes' rests to `inner`; an unavailable axis is never held, it was never
    /// enabled here.
    fn stop_holding(&mut self, rests: &[crate::Rest]) -> StopReport {
        let kept: Axes<crate::Rest> = self
            .present
            .iter()
            .map(|index| rests.get(*index).copied().unwrap_or_default())
            .collect();
        self.inner.stop_holding(&kept)
    }

    /// `inner`'s holds, its receipt placed in the full vector, and the read-only axes read as
    /// a command reads them.
    fn keep_holding(&mut self) -> Result<Receipt<C::Feedback>, C::Error> {
        let receipt = self.inner.keep_holding()?;
        self.sense(SENSE_PERIOD);
        Ok(self.spread_receipt(receipt))
    }

    fn stop_budget(&self) -> Duration {
        self.inner.stop_budget()
    }

    /// `inner`'s, with each read-only axis's last read position and `NaN` at every other
    /// unavailable axis.
    fn last_observed(&self) -> Option<Axes<f64>> {
        Some(self.spread_positions(&self.inner.last_observed()?))
    }

    fn command_readings(&self) -> Option<Axes<AxisReading>> {
        Some(self.spread_readings(&self.inner.command_readings()?))
    }

    fn unavailable(&self) -> Vec<usize> {
        self.unavailable.clone()
    }

    /// The disables the read-only observer sent its out-of-service actuators.
    fn read_only_disables(&self) -> Vec<u16> {
        self.read_only
            .as_ref()
            .map(|read_only| read_only.observer.sent_disables())
            .unwrap_or_default()
    }

    /// `inner`'s, placed in the full vector.
    fn tracking_ceilings(&self) -> Vec<(usize, crate::chain::TrackingCeiling)> {
        self.inner
            .tracking_ceilings()
            .into_iter()
            .map(|(axis, speed)| (self.present[axis], speed))
            .collect()
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

    fn bus_timing(&self) -> Vec<crate::BusTiming> {
        self.inner.bus_timing()
    }

    fn retried(&self) -> u64 {
        self.inner.retried()
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
        // After the command's frames, so a read never delays them.
        self.sense(SENSE_PERIOD);
        Ok(self.spread_receipt(receipt))
    }

    /// `inner`'s receipt placed in the full vector: observed `NaN` and (where set) the
    /// unanswered feedback at each unavailable axis.
    fn spread_receipt(&self, receipt: Receipt<C::Feedback>) -> Receipt<C::Feedback> {
        Receipt {
            evidence: receipt.evidence,
            observed: if receipt.observed.is_empty() {
                Axes::new()
            } else {
                self.spread_positions(&receipt.observed)
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
        }
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

#[cfg(test)]
mod tests {
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};

    use super::*;
    use crate::binding::{ActuatorBinding, ActuatorBus, ActuatorModel, AxisMotion};
    use crate::chain::Evidence;

    fn axis(joint: &str, bus_id: u16) -> BoundAxis {
        BoundAxis {
            joint: joint.into(),
            port: "can".into(),
            lower: -1.0,
            upper: 1.0,
            binding: ActuatorBinding {
                model: ActuatorModel::new("model"),
                bus: ActuatorBus::new("bus"),
                bus_id,
                sign: 1,
                reduction: 1.0,
            },
            zero: crate::binding::UNCALIBRATED_ZERO,
            direction: crate::binding::UNCALIBRATED_DIRECTION,
            home: None,
            gripper: false,
            motion: AxisMotion {
                speed: 1.0,
                acceleration: 10.0,
                inertia: 0.0,
            },
        }
    }

    #[derive(Debug, thiserror::Error)]
    #[error("refused")]
    struct Refused;

    impl crate::chain::ChainError for Refused {
        fn failure(&self) -> crate::chain::ChainFailure {
            crate::chain::ChainFailure::Refused
        }
    }

    /// One commanded axis that answers where it was sent.
    struct Commanded {
        axes: Vec<BoundAxis>,
        ranges: Vec<CommandRange>,
        at: f64,
    }

    impl ActuatorChain for Commanded {
        type Error = Refused;
        type Feedback = ();

        fn axes(&self) -> &[BoundAxis] {
            &self.axes
        }

        fn commandable(&self) -> &[CommandRange] {
            &self.ranges
        }

        fn command(&mut self, joints: &[f64]) -> Result<Receipt<()>, Refused> {
            self.at = joints[0];
            Ok(Receipt {
                evidence: Evidence::BusObservedComplete,
                observed: Axes::filled(self.at, 1),
                feedback: Axes::filled((), 1),
            })
        }

        fn read_positions(&mut self) -> Axes<AxisReading> {
            Axes::filled(
                AxisReading::Fresh {
                    position: self.at,
                    received: Instant::now(),
                },
                1,
            )
        }

        fn stop(&mut self) -> StopReport {
            StopReport::default()
        }

        fn stop_budget(&self) -> Duration {
            Duration::ZERO
        }

        fn last_observed(&self) -> Option<Axes<f64>> {
            Some(Axes::filled(self.at, 1))
        }

        fn command_readings(&self) -> Option<Axes<AxisReading>> {
            Some(Axes::filled(
                AxisReading::Fresh {
                    position: self.at,
                    received: Instant::now(),
                },
                1,
            ))
        }
    }

    /// One read-only axis at `at`, silent while `silent`, counting its reads.
    struct Sensor {
        axes: Vec<BoundAxis>,
        at: f64,
        silent: Arc<std::sync::atomic::AtomicBool>,
        reads: Arc<AtomicUsize>,
    }

    impl Observe for Sensor {
        fn axes(&self) -> &[BoundAxis] {
            &self.axes
        }

        fn read_positions(&mut self) -> Axes<AxisReading> {
            self.reads.fetch_add(1, Ordering::SeqCst);
            Axes::filled(
                if self.silent.load(Ordering::SeqCst) {
                    AxisReading::Missing(MissingReason::Timeout)
                } else {
                    AxisReading::Fresh {
                        position: self.at,
                        received: Instant::now(),
                    }
                },
                1,
            )
        }
    }

    type Fixture = (
        Masked<Commanded, ()>,
        Arc<AtomicUsize>,
        Arc<std::sync::atomic::AtomicBool>,
    );

    fn masked(silent: bool) -> Fixture {
        let commanded = Commanded {
            axes: vec![axis("arm", 1)],
            ranges: vec![CommandRange {
                lower: -1.0,
                upper: 1.0,
            }],
            at: 0.0,
        };
        let reads = Arc::new(AtomicUsize::new(0));
        let silence = Arc::new(std::sync::atomic::AtomicBool::new(silent));
        let sensor = Sensor {
            axes: vec![axis("torso", 2)],
            at: 0.3,
            silent: Arc::clone(&silence),
            reads: Arc::clone(&reads),
        };
        let masked = Masked::new(commanded, vec![axis("arm", 1), axis("torso", 2)], &[1])
            .expect("masked")
            .reading(Box::new(sensor))
            .expect("the torso read");
        (masked, reads, silence)
    }

    #[test]
    fn a_read_only_axis_is_read_fresh_and_its_target_reaches_nothing() {
        let (mut masked, reads, _) = masked(false);
        assert!(masked.reads_unavailable());
        assert_eq!(masked.unavailable(), vec![1], "still never commanded");
        let readings = masked.read_positions();
        assert_eq!(readings[1].position(), Some(0.3));
        assert_eq!(reads.load(Ordering::SeqCst), 1);
        assert!((masked.last_observed().expect("observed")[1] - 0.3).abs() < f64::EPSILON);
        // Its target is dropped; its reading rides each receipt, read at most every period.
        for _ in 0..10 {
            let receipt = masked.command(&[0.5, -0.9]).expect("command");
            assert_eq!(receipt.observed.as_slice(), &[0.5, 0.3]);
        }
        assert_eq!(reads.load(Ordering::SeqCst), 1, "not read every tick");
        let readings = masked.command_readings().expect("readings");
        assert_eq!(readings[1].position(), Some(0.3));
        // Nothing but a read reaches it: stop, clear and limits are the commanded chain's.
        assert!(masked.stop().is_proven());
        masked.clear_faults();
        masked
            .set_feedforward_limits(&[FeedforwardLimit::default(); 2])
            .expect("limits");
        assert_eq!(reads.load(Ordering::SeqCst), 1);
        // A read asks it every time.
        let _ = masked.read_positions();
        assert_eq!(reads.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn a_read_only_axis_that_does_not_answer_is_out_of_service_and_asked_again_after_a_back_off() {
        let (mut masked, reads, silence) = masked(true);
        for _ in 0..5 {
            let readings = masked.read_positions();
            assert_eq!(
                readings[1],
                AxisReading::Missing(MissingReason::OutOfService)
            );
            assert!(readings[0].is_fresh(), "the rest of the bus reads");
        }
        assert_eq!(reads.load(Ordering::SeqCst), 1, "backed off");
        let receipt = masked
            .command(&[0.1, 0.0])
            .expect("the rest still commands");
        assert!(receipt.observed[1].is_nan());
        assert_eq!(reads.load(Ordering::SeqCst), 1);
        silence.store(false, Ordering::SeqCst);
        let reading = masked.read_only.as_mut().expect("read only");
        reading.asked = reading
            .asked
            .map(|asked| asked.checked_sub(SENSE_RETRY).expect("a past instant"));
        assert_eq!(masked.read_positions()[1].position(), Some(0.3));
        assert_eq!(reads.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn the_back_off_doubles_while_nothing_answers_and_resets_once_it_does() {
        let (mut masked, _, silence) = masked(true);
        let mut retries = Vec::new();
        for _ in 0..8 {
            let reading = masked.read_only.as_mut().expect("read only");
            reading.asked = reading
                .asked
                .map(|asked| asked.checked_sub(SENSE_RETRY_MAX).expect("a past instant"));
            let _ = masked.read_positions();
            retries.push(masked.read_only.as_ref().expect("read only").retry);
        }
        assert_eq!(retries[0], SENSE_RETRY);
        assert_eq!(retries[1], SENSE_RETRY * 2);
        assert_eq!(*retries.last().expect("last"), SENSE_RETRY_MAX);
        silence.store(false, Ordering::SeqCst);
        let reading = masked.read_only.as_mut().expect("read only");
        reading.asked = reading
            .asked
            .map(|asked| asked.checked_sub(SENSE_RETRY_MAX).expect("a past instant"));
        let _ = masked.read_positions();
        assert_eq!(
            masked.read_only.as_ref().expect("read only").retry,
            SENSE_RETRY
        );
    }

    #[test]
    fn an_observer_of_other_axes_than_the_unavailable_ones_is_refused() {
        let commanded = Commanded {
            axes: vec![axis("arm", 1)],
            ranges: vec![CommandRange {
                lower: -1.0,
                upper: 1.0,
            }],
            at: 0.0,
        };
        let sensor = Sensor {
            axes: vec![axis("arm", 1)],
            at: 0.0,
            silent: Arc::default(),
            reads: Arc::default(),
        };
        let masked =
            Masked::new(commanded, vec![axis("arm", 1), axis("torso", 2)], &[1]).expect("masked");
        assert!(masked.reading(Box::new(sensor)).is_err());
    }

    /// Readings handed out in order, each `received` `gap` after the one before.
    struct Scripted {
        axes: Vec<BoundAxis>,
        script: std::collections::VecDeque<Option<f64>>,
        corrupt: bool,
        clock: Instant,
        gap: Duration,
    }

    impl Observe for Scripted {
        fn axes(&self) -> &[BoundAxis] {
            &self.axes
        }

        fn read_positions(&mut self) -> Axes<AxisReading> {
            self.clock += self.gap;
            let reading = if self.corrupt {
                AxisReading::Missing(MissingReason::Corrupt)
            } else {
                match self.script.pop_front().flatten() {
                    Some(position) => AxisReading::Fresh {
                        position,
                        received: self.clock,
                    },
                    None => AxisReading::Missing(MissingReason::Timeout),
                }
            };
            Axes::filled(reading, 1)
        }
    }

    fn scripted(script: &[Option<f64>], corrupt: bool) -> Masked<Commanded, ()> {
        let commanded = Commanded {
            axes: vec![axis("arm", 1)],
            ranges: vec![CommandRange {
                lower: -1.0,
                upper: 1.0,
            }],
            at: 0.0,
        };
        // The torso turns at most 1 rad/s ([`axis`]); a read every 100 ms.
        let sensor = Scripted {
            axes: vec![axis("torso", 2)],
            script: script.iter().copied().collect(),
            corrupt,
            clock: Instant::now(),
            gap: Duration::from_millis(100),
        };
        Masked::new(commanded, vec![axis("arm", 1), axis("torso", 2)], &[1])
            .expect("masked")
            .reading(Box::new(sensor))
            .expect("read")
    }

    #[test]
    fn an_implausible_read_only_reading_is_unavailable_never_fresh() {
        let unavailable = AxisReading::Missing(MissingReason::Unavailable);
        // Outside the axis's bounds (-1..1): a faulty encoder or a wrong scale.
        let mut masked = scripted(&[Some(0.2), Some(1.5), Some(-3.0), Some(0.21)], false);
        let mut read = || masked.read_positions()[1];
        assert_eq!(read().position(), Some(0.2));
        assert_eq!(read(), unavailable);
        assert_eq!(read(), unavailable);
        assert_eq!(read().position(), Some(0.21));
        // A jump no faster than 1 rad/s could make in 100 ms is refused; one the next reading
        // confirms was a real move (a person turned it), and is taken.
        let mut masked = scripted(
            &[Some(0.0), Some(0.05), Some(0.6), Some(0.6), Some(0.62)],
            false,
        );
        let mut read = || masked.read_positions()[1];
        assert_eq!(read().position(), Some(0.0));
        assert_eq!(read().position(), Some(0.05));
        assert_eq!(read(), unavailable, "0.55 rad in 100 ms");
        assert_eq!(read().position(), Some(0.6), "confirmed");
        assert_eq!(read().position(), Some(0.62));
        // A glitch that comes back is refused alone.
        let mut masked = scripted(&[Some(0.1), Some(0.9), Some(0.1)], false);
        let mut read = || masked.read_positions()[1];
        assert_eq!(read().position(), Some(0.1));
        assert_eq!(read(), unavailable);
        assert_eq!(read().position(), Some(0.1));
        // A reading whose scale the observer refused (another model's limit row).
        let mut masked = scripted(&[], true);
        assert_eq!(masked.read_positions()[1], unavailable);
        assert!(masked.last_observed().expect("observed")[1].is_nan());
    }
}
