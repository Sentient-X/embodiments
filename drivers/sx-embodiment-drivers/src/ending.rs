//! What a governed body does when its operation ends normally.
//!
//! A chain's [`ActuatorChain::stop`] disables every actuator where it is, so a body whose
//! operation simply stops falls, unpowered, from wherever it ended. [`AtEnd`] is the choice made
//! per body or per run instead, and [`Governor::end`] carries it out:
//!
//! - [`AtEnd::Home`] (the default): return every axis to its home ([`BoundAxis::home`], the
//!   chain's declared `home`), then stop. An axis whose part declares no home (a gripper's
//!   finger) stays where it is: a guessed pose could pinch.
//! - [`AtEnd::Start`]: return to where the body rested when the governor was built over the
//!   freshly opened chain (where it sat, so a pose it can be disabled in), then stop.
//! - [`AtEnd::Release`]: stop where the body is, as before this module existed.
//!
//! There is no choice that leaves torque on after the end: a Damiao motor whose `TIMEOUT` is
//! armed (the static upper body's 0.75 s) leaves enable mode and latches fault 0xD once nothing
//! commands it, so "hold" would be a delayed drop, and a Feetech servo would hold with no
//! process watching it.
//!
//! A return is motion under the same [`Governor`]: every step is slewed within the axis's speed
//! cap (and acceleration cap, and derate), held inside its range, and tracking-checked; an axis
//! held by the caller (`keep`), unavailable, or without a home stays where it is. The return
//! carries the caller's feed-forward (the gravity a model holds the arm against), so a PD arm
//! does not sag when the operation's feed-forward stops. It moves its target from where the
//! governor last sent each axis at [`RETURN_SPEED_SHARE`] of each axis's cap, so a servo whose own approach limit
//! floors below the cap (an STS3215 set to 0.25 rad/s moves at 0.23) keeps up instead of
//! falling ever further behind. It has arrived when every axis was sent its target and observed
//! within [`RETURN_TOLERANCE`] of it, and is held there for [`RETURN_SETTLE`] before the stop.
//! It is bounded three ways:
//!
//! - it never moves a body nothing drove: one not commanded since it was opened or last stopped
//!   rests where it is, unpowered or not, and is only stopped ([`Ending::Rested`]);
//! - it never starts on a body that is not known to be well: a latched governor fault, or an
//!   available axis whose read is not fresh, ends in the plain stop with no command sent;
//! - a deadline, from the farthest axis's travel at its speed cap ([`RETURN_SLACK`] ×, plus
//!   [`RETURN_MARGIN`]), refused outright above [`MAX_RETURN`];
//! - the governor's own tracking, missed-answer and chain-failure stops.
//!
//! A return that cannot start or does not finish ends in the plain stop, and the [`Ending`]
//! says so ([`Ending::FellBack`] and its [`ReturnFailure`]); the caller logs it.
//!
//! This is the normal end only. A fault, a lost supervisor (operator, intent or command
//! stream) or an unanswered axis keeps the existing safe-stop: a safe-stop never moves the body.

use std::fmt;
use std::str::FromStr;
use std::time::{Duration, Instant};

use crate::axes::Axes;
use crate::binding::BoundAxis;
use crate::chain::{ActuatorChain, ChainError, ChainFailure, Feedforward, StopReport};
use crate::governor::{Clock, Governor, MAX_STEP_INTERVAL};

/// Writes the feed-forward for a commanded pose (both in native state order), as the
/// operation's own model does: for an arm, the gravity torque at that pose.
pub type FeedforwardFn<'a> = dyn FnMut(&[f64], &mut [Feedforward]) + 'a;

/// The share of each axis's speed cap a return moves its target at.
pub const RETURN_SPEED_SHARE: f64 = 0.5;

/// Each return's deadline is its planned time times this.
pub const RETURN_SLACK: f64 = 1.5;

/// Added to each return's deadline: the settle and the bus's own latency.
pub const RETURN_MARGIN: Duration = Duration::from_secs(2);

/// How near its target every axis must be observed before a return counts as arrived, in the
/// joint's unit.
pub const RETURN_TOLERANCE: f64 = 0.05;

/// How long a return is held at its target, sending it, before the body is stopped there.
pub const RETURN_SETTLE: Duration = Duration::from_millis(300);

/// The longest return planned; a farther one is refused and the body stops where it is.
pub const MAX_RETURN: Duration = Duration::from_secs(60);

/// What a body does when its operation ends normally; [`AtEnd::Home`] unless chosen otherwise.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum AtEnd {
    /// Return to each axis's home ([`BoundAxis::home`]) under the governor, then stop.
    #[default]
    Home,
    /// Return to where the body rested when the governor was built, then stop.
    Start,
    /// Stop where the body is: every actuator disabled in place.
    Release,
}

impl AtEnd {
    /// Every choice, by its spelling.
    pub const ALL: [Self; 3] = [Self::Home, Self::Start, Self::Release];

    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Home => "home",
            Self::Start => "start",
            Self::Release => "release",
        }
    }
}

impl fmt::Display for AtEnd {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl FromStr for AtEnd {
    type Err = String;

    fn from_str(spelled: &str) -> Result<Self, Self::Err> {
        Self::ALL
            .into_iter()
            .find(|choice| choice.as_str() == spelled)
            .ok_or_else(|| format!("{spelled} is not one of home, start, release"))
    }
}

/// Why a return did not run or did not finish; the body was stopped where it was.
#[derive(Clone, Debug, PartialEq)]
pub enum ReturnFailure {
    /// Nothing was commanded since the chain was opened or last stopped: the body rests where
    /// it is and is not moved. Not a failure of the return; [`Governor::end`] reports it as
    /// [`Ending::Rested`].
    NotDriven,
    /// The governor holds a latched fault: nothing may move.
    Latched,
    /// This available axis's read was not fresh (no answer, a fault code, a stale answer).
    Unobserved { joint: String },
    /// [`AtEnd::Start`] with no rest recorded when the governor was built.
    NoStart,
    /// The farthest axis needs longer than [`MAX_RETURN`] at its speed cap.
    TooFar { needs: Duration },
    /// The body had not settled at the target by the deadline.
    Deadline { after: Duration },
    /// The person running it asked for the body to stop where it is.
    Aborted,
    /// The governor refused a step (nothing applied) or stopped the body for a fault.
    Governor {
        failure: ChainFailure,
        error: String,
    },
}

impl fmt::Display for ReturnFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotDriven => {
                f.write_str("nothing drove the body since it was opened or last stopped")
            }
            Self::Latched => f.write_str("a governor fault is latched"),
            Self::Unobserved { joint } => write!(f, "{joint} did not answer a fresh read"),
            Self::NoStart => f.write_str("no rest was recorded when the body was opened"),
            Self::TooFar { needs } => write!(
                f,
                "the return needs {:.1} s, more than {} s",
                needs.as_secs_f64(),
                MAX_RETURN.as_secs()
            ),
            Self::Deadline { after } => {
                write!(
                    f,
                    "not settled by its {:.1} s deadline",
                    after.as_secs_f64()
                )
            }
            Self::Aborted => f.write_str("stopped in place on request"),
            Self::Governor { error, .. } => f.write_str(error),
        }
    }
}

/// How an operation's end went.
#[derive(Debug)]
#[must_use]
pub enum Ending {
    /// The body reached its target ([`AtEnd::Home`] or [`AtEnd::Start`]) and was stopped there.
    Returned {
        to: AtEnd,
        took: Duration,
        stop: StopReport,
    },
    /// Nothing drove the body since it was opened or last stopped, so it was not moved: it was
    /// stopped where it rests ([`AtEnd::Home`] and [`AtEnd::Start`]).
    Rested { stop: StopReport },
    /// The body was stopped where it was ([`AtEnd::Release`]).
    Released { stop: StopReport },
    /// The return did not run or did not finish; the body was stopped where it was.
    FellBack {
        to: AtEnd,
        failure: ReturnFailure,
        stop: StopReport,
    },
}

impl Ending {
    /// The stop's evidence: every ending stops the body.
    #[must_use]
    pub const fn stop(&self) -> &StopReport {
        match self {
            Self::Returned { stop, .. }
            | Self::Rested { stop }
            | Self::Released { stop }
            | Self::FellBack { stop, .. } => stop,
        }
    }

    /// Whether the end went as asked and every actuator proved it stopped: not a fallback,
    /// and no stop left an actuator unproven.
    #[must_use]
    pub fn is_clean(&self) -> bool {
        !matches!(self, Self::FellBack { .. }) && self.stop().is_proven()
    }
}

impl fmt::Display for Ending {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Returned { to, took, .. } => write!(
                f,
                "returned to {to} in {:.1} s, then stopped",
                took.as_secs_f64()
            ),
            Self::Rested { .. } => {
                f.write_str("nothing drove the body: stopped where it rests, not moved")
            }
            Self::Released { .. } => f.write_str("released: stopped where it was"),
            Self::FellBack { to, failure, .. } => write!(
                f,
                "the return to {to} fell back to the plain stop where the body was: {failure}"
            ),
        }
    }
}

/// One return in progress: the target, its deadline, and since when the body has sat on it.
/// [`Governor::end`] drives one to the end; a caller with its own wait can call
/// [`Return::begin`] and [`Return::tick`] itself.
#[derive(Debug)]
pub struct Return {
    to: AtEnd,
    target: Axes<f64>,
    /// What the return sends: each axis moved from where the governor last sent it toward
    /// `target` at [`RETURN_SPEED_SHARE`] of its cap.
    commanded: Axes<f64>,
    /// The feed-forward sent with `commanded`, refilled each tick.
    feedforward: Axes<Feedforward>,
    /// The axes being returned: not kept, unavailable or without a home. Only these must be
    /// observed near their target to arrive.
    returning: Axes<bool>,
    /// When `commanded` last moved.
    last: Option<Instant>,
    began: Instant,
    deadline: Instant,
    arrived: Option<Instant>,
}

/// Where a return is after one tick.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Progress {
    Moving,
    /// Every axis was sent its target, observed within [`RETURN_TOLERANCE`] of it, and held
    /// there for [`RETURN_SETTLE`].
    Settled,
}

impl Return {
    /// Plan the return to `to` ([`AtEnd::Start`], else home), each axis in `keep` (indices in
    /// native state order) held where it is read. Sends nothing.
    ///
    /// # Errors
    ///
    /// [`ReturnFailure::NotDriven`], [`ReturnFailure::Latched`], [`ReturnFailure::Unobserved`],
    /// [`ReturnFailure::NoStart`] and [`ReturnFailure::TooFar`], each before any command.
    pub fn begin<C: ActuatorChain, K: Clock>(
        governor: &mut Governor<C, K>,
        to: AtEnd,
        keep: &[usize],
    ) -> Result<Self, ReturnFailure> {
        if governor.is_latched() {
            return Err(ReturnFailure::Latched);
        }
        let Some(sent) = governor.sent().map(Axes::from_slice) else {
            return Err(ReturnFailure::NotDriven);
        };
        let unavailable = governor.unavailable();
        let readings = governor.read_positions();
        let axes = governor.axes();
        let mut at = Axes::new();
        for (index, (axis, reading)) in axes.iter().zip(&readings).enumerate() {
            match reading.position() {
                Some(position) => at.push(position),
                None if unavailable.contains(&index) => at.push(f64::NAN),
                None => {
                    return Err(ReturnFailure::Unobserved {
                        joint: axis.joint.clone(),
                    });
                }
            }
        }
        if readings.len() != axes.len() {
            return Err(ReturnFailure::Unobserved {
                joint: axes
                    .get(readings.len())
                    .map_or_else(String::new, |axis| axis.joint.clone()),
            });
        }
        let pose: Axes<f64> = match to {
            AtEnd::Start => Axes::from_slice(governor.opened_at().ok_or(ReturnFailure::NoStart)?),
            AtEnd::Home | AtEnd::Release => axes
                .iter()
                .map(|axis: &BoundAxis| axis.home.unwrap_or(f64::NAN))
                .collect(),
        };
        let mut target = Axes::new();
        let mut returning = Axes::filled(false, axes.len());
        let mut needs = 0.0_f64;
        for (index, ((pose, at), (range, limit))) in pose
            .iter()
            .zip(&at)
            .zip(governor.commandable().iter().zip(governor.limits()))
            .enumerate()
        {
            if unavailable.contains(&index) {
                target.push(range.lower.max(range.upper.min(0.0)));
                continue;
            }
            if keep.contains(&index) || !pose.is_finite() {
                // Held where it is read: the governor's range rule lets it stay.
                target.push(*at);
                continue;
            }
            let goal = pose.clamp(range.lower, range.upper);
            let travel = (goal - at).abs().max((goal - sent[index]).abs());
            let mut seconds = travel / (limit.max_speed * RETURN_SPEED_SHARE);
            if let Some(acceleration) = limit.max_acceleration {
                seconds += limit.max_speed / acceleration;
            }
            needs = needs.max(seconds);
            returning[index] = true;
            target.push(goal);
        }
        let needs = Duration::from_secs_f64(needs);
        if needs > MAX_RETURN {
            return Err(ReturnFailure::TooFar { needs });
        }
        let began = governor.clock().now();
        // From where the governor last sent each axis, so no step is faster than the share.
        let commanded = target
            .iter()
            .zip(&sent)
            .enumerate()
            .map(|(index, (target, sent))| {
                if unavailable.contains(&index) {
                    *target
                } else {
                    *sent
                }
            })
            .collect();
        let feedforward = Axes::filled(Feedforward::default(), target.len());
        Ok(Self {
            to,
            target,
            commanded,
            feedforward,
            returning,
            last: None,
            began,
            deadline: began + needs.mul_f64(RETURN_SLACK) + RETURN_MARGIN,
            arrived: None,
        })
    }

    /// The target, in native state order.
    #[must_use]
    pub fn target(&self) -> &[f64] {
        &self.target
    }

    /// What the return was asked for.
    #[must_use]
    pub const fn to(&self) -> AtEnd {
        self.to
    }

    /// Send the target once through the governor, which slews and tracks it, with the
    /// feed-forward `feedforward` writes for the commanded positions when given (every entry
    /// starts at zero each tick).
    ///
    /// # Errors
    ///
    /// [`ReturnFailure::Deadline`] once the deadline passes unsettled, and
    /// [`ReturnFailure::Governor`] for a refused step or a fault the governor stopped for.
    pub fn tick<C: ActuatorChain, K: Clock>(
        &mut self,
        governor: &mut Governor<C, K>,
        feedforward: Option<&mut FeedforwardFn<'_>>,
    ) -> Result<Progress, ReturnFailure> {
        let now = governor.clock().now();
        if now > self.deadline {
            return Err(ReturnFailure::Deadline {
                after: self.deadline.saturating_duration_since(self.began),
            });
        }
        self.advance(governor, now);
        let receipt = self.send(governor, feedforward)?;
        // Every returning axis sent its target and observed near it: a late answer (NaN) is
        // not arrival.
        let on_target = governor.sent().is_some_and(|sent| {
            sent.iter()
                .zip(&receipt.observed)
                .zip(&self.target)
                .zip(&self.returning)
                .all(|(((sent, observed), target), returning)| {
                    !returning
                        || ((sent - target).abs() <= 1e-9
                            && (observed - target).abs() <= RETURN_TOLERANCE)
                })
        });
        if !on_target {
            self.arrived = None;
            return Ok(Progress::Moving);
        }
        let since = *self.arrived.get_or_insert(now);
        Ok(if now.saturating_duration_since(since) >= RETURN_SETTLE {
            Progress::Settled
        } else {
            Progress::Moving
        })
    }

    /// Keep the body enabled at the target: each call sends it again, with `feedforward` when
    /// given (called before the return settled, it keeps approaching at the return's pace). No
    /// deadline: the caller bounds how long the body is held, and the governor's tracking,
    /// missed-answer and chain-failure stops still apply.
    ///
    /// # Errors
    ///
    /// [`ReturnFailure::Governor`] for a refused step or a fault the governor stopped for.
    pub fn hold<C: ActuatorChain, K: Clock>(
        &mut self,
        governor: &mut Governor<C, K>,
        feedforward: Option<&mut FeedforwardFn<'_>>,
    ) -> Result<(), ReturnFailure> {
        let now = governor.clock().now();
        self.advance(governor, now);
        self.send(governor, feedforward).map(|_| ())
    }

    /// Move `commanded` toward the target at [`RETURN_SPEED_SHARE`] of each cap for the time
    /// since the last call, at most [`MAX_STEP_INTERVAL`].
    fn advance<C: ActuatorChain, K: Clock>(&mut self, governor: &Governor<C, K>, now: Instant) {
        let dt = self.last.map_or(MAX_STEP_INTERVAL, |last| {
            now.saturating_duration_since(last).min(MAX_STEP_INTERVAL)
        });
        self.last = Some(now);
        for ((commanded, target), limit) in self
            .commanded
            .iter_mut()
            .zip(&self.target)
            .zip(governor.limits())
        {
            let step = limit.max_speed * RETURN_SPEED_SHARE * dt.as_secs_f64();
            *commanded += (target - *commanded).clamp(-step, step);
        }
    }

    fn send<C: ActuatorChain, K: Clock>(
        &mut self,
        governor: &mut Governor<C, K>,
        feedforward: Option<&mut FeedforwardFn<'_>>,
    ) -> Result<crate::chain::Receipt<C::Feedback>, ReturnFailure> {
        let sent = match feedforward {
            Some(write) => {
                self.feedforward.fill(Feedforward::default());
                write(&self.commanded, &mut self.feedforward);
                governor.command_with(&self.commanded, &self.feedforward)
            }
            None => governor.command(&self.commanded),
        };
        sent.map_err(|error| ReturnFailure::Governor {
            failure: error.failure(),
            error: error.to_string(),
        })
    }

    /// How long since the return began.
    pub fn elapsed<C: ActuatorChain, K: Clock>(&self, governor: &Governor<C, K>) -> Duration {
        governor.clock().now().saturating_duration_since(self.began)
    }
}

impl<C: ActuatorChain, K: Clock> Governor<C, K> {
    /// End this body's operation as `at_end` says, holding the axes in `keep` where they are,
    /// with `feedforward` written for each commanded pose ([`Return::tick`]); `pace` waits out
    /// one control period between commands (a test advances its clock), and `abort` is asked
    /// before every command (a stop request): when it answers `true` the return ends in the
    /// plain stop where the body is.
    ///
    /// Only for a normal end: a fault or a lost supervisor stops the chain directly.
    pub fn end(
        &mut self,
        at_end: AtEnd,
        keep: &[usize],
        mut feedforward: Option<&mut FeedforwardFn<'_>>,
        pace: &mut dyn FnMut(),
        abort: &dyn Fn() -> bool,
    ) -> Ending {
        match at_end {
            AtEnd::Release => Ending::Released { stop: self.stop() },
            AtEnd::Home | AtEnd::Start => {
                let failure = match Return::begin(self, at_end, keep) {
                    Err(failure) => failure,
                    Ok(mut ret) => loop {
                        if abort() {
                            break ReturnFailure::Aborted;
                        }
                        match ret.tick(self, feedforward.as_deref_mut()) {
                            Ok(Progress::Settled) => {
                                let took = ret.elapsed(self);
                                return Ending::Returned {
                                    to: at_end,
                                    took,
                                    stop: self.stop(),
                                };
                            }
                            Ok(Progress::Moving) => pace(),
                            Err(failure) => break failure,
                        }
                    },
                };
                let stop = self.stop();
                if failure == ReturnFailure::NotDriven {
                    return Ending::Rested { stop };
                }
                Ending::FellBack {
                    to: at_end,
                    failure,
                    stop,
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use std::cell::Cell;
    use std::rc::Rc;

    use thiserror::Error;

    use super::*;
    use crate::binding::{ActuatorBinding, ActuatorBus, ActuatorModel};
    use crate::chain::{AxisReading, CommandRange, Evidence, MissingReason, Receipt};
    use crate::governor::AxisLimits;

    #[derive(Debug, Error)]
    #[error("fake failure")]
    struct FakeError;

    impl ChainError for FakeError {
        fn failure(&self) -> ChainFailure {
            ChainFailure::Fault
        }
    }

    /// Two axes on -1..1, homes 0.0 and -0.4; the body is where it was last sent, and rests at
    /// `rest` until then. Records every command and stop.
    struct Fake {
        axes: Vec<BoundAxis>,
        commandable: Vec<CommandRange>,
        at: Vec<f64>,
        sent: Vec<Vec<f64>>,
        stops: usize,
        silent: Option<usize>,
        unavailable: Vec<usize>,
        /// Observed this far from what was sent.
        lag: f64,
        /// Each `command_with`'s feed-forward.
        feedforward: Vec<Vec<Feedforward>>,
        /// What every stop reports unproven.
        unproven: Vec<u16>,
    }

    fn axis(joint: &str, bus_id: u16, home: f64) -> BoundAxis {
        BoundAxis {
            joint: joint.to_owned(),
            port: "serial".to_owned(),
            lower: -1.0,
            upper: 1.0,
            binding: ActuatorBinding {
                model: ActuatorModel::new("model"),
                bus: ActuatorBus::new("bus"),
                bus_id,
                sign: 1,
                reduction: 1.0,
            },
            zero: 0.0,
            direction: 1,
            home: Some(home),
        }
    }

    fn fake(rest: [f64; 2]) -> Fake {
        Fake {
            axes: vec![axis("a", 1, 0.0), axis("b", 2, -0.4)],
            commandable: vec![
                CommandRange {
                    lower: -0.9,
                    upper: 0.9,
                };
                2
            ],
            at: rest.to_vec(),
            sent: Vec::new(),
            stops: 0,
            silent: None,
            unavailable: Vec::new(),
            lag: 0.0,
            feedforward: Vec::new(),
            unproven: Vec::new(),
        }
    }

    impl ActuatorChain for Fake {
        type Error = FakeError;
        type Feedback = ();

        fn axes(&self) -> &[BoundAxis] {
            &self.axes
        }

        fn commandable(&self) -> &[CommandRange] {
            &self.commandable
        }

        fn command(&mut self, joints: &[f64]) -> Result<Receipt<()>, FakeError> {
            self.sent.push(joints.to_vec());
            self.at = joints.iter().map(|joint| joint + self.lag).collect();
            Ok(Receipt {
                evidence: Evidence::BusObservedComplete,
                observed: Axes::from_slice(&self.at),
                feedback: Axes::new(),
            })
        }

        fn command_with(
            &mut self,
            joints: &[f64],
            feedforward: &[Feedforward],
        ) -> Result<Receipt<()>, FakeError> {
            self.feedforward.push(feedforward.to_vec());
            self.command(joints)
        }

        fn stop(&mut self) -> StopReport {
            self.stops += 1;
            StopReport {
                unproven: self.unproven.clone(),
                ..StopReport::default()
            }
        }

        fn stop_budget(&self) -> Duration {
            Duration::from_millis(5)
        }

        fn last_observed(&self) -> Option<Axes<f64>> {
            Some(Axes::from_slice(&self.at))
        }

        fn read_positions(&mut self) -> Axes<AxisReading> {
            self.at
                .iter()
                .enumerate()
                .map(|(index, position)| {
                    if self.silent == Some(index) || self.unavailable.contains(&index) {
                        AxisReading::Missing(MissingReason::Timeout)
                    } else {
                        AxisReading::Fresh {
                            position: *position,
                            received: Instant::now(),
                        }
                    }
                })
                .collect()
        }

        fn unavailable(&self) -> Vec<usize> {
            self.unavailable.clone()
        }
    }

    #[derive(Clone)]
    struct Manual(Rc<Cell<Instant>>);

    impl Clock for Manual {
        fn now(&self) -> Instant {
            self.0.get()
        }
    }

    const LIMITS: AxisLimits = AxisLimits {
        max_speed: 0.25,
        max_tracking_error: 0.2,
        max_acceleration: None,
    };

    /// `chain` governed, and driven once where it rests, as an operation leaves it.
    fn governed(chain: Fake) -> (Governor<Fake, Manual>, Manual) {
        let (mut governor, clock) = opened(chain);
        let rest = governor.last_observed().expect("rest");
        governor.command(&rest).expect("driven where it rests");
        clock.0.set(clock.0.get() + MAX_STEP);
        (governor, clock)
    }

    /// `chain` governed, nothing commanded yet.
    fn opened(chain: Fake) -> (Governor<Fake, Manual>, Manual) {
        let clock = Manual(Rc::new(Cell::new(Instant::now())));
        let governor = Governor::with_clock(chain, vec![LIMITS; 2], clock.clone()).expect("limits");
        (governor, clock)
    }

    /// End with `at_end`, each period 20 ms of the governor's clock.
    fn end(governor: &mut Governor<Fake, Manual>, clock: &Manual, at_end: AtEnd) -> Ending {
        end_keeping(governor, clock, at_end, &[])
    }

    fn end_keeping(
        governor: &mut Governor<Fake, Manual>,
        clock: &Manual,
        at_end: AtEnd,
        keep: &[usize],
    ) -> Ending {
        let tick = clock.clone();
        governor.end(
            at_end,
            keep,
            None,
            &mut || tick.0.set(tick.0.get() + MAX_STEP),
            &|| false,
        )
    }

    const MAX_STEP: Duration = crate::governor::MAX_STEP_INTERVAL;

    fn close(left: &[f64], right: &[f64]) -> bool {
        left.len() == right.len() && left.iter().zip(right).all(|(l, r)| (l - r).abs() < 1e-9)
    }

    #[test]
    fn the_default_end_returns_home_at_the_speed_cap_then_stops() {
        assert_eq!(AtEnd::default(), AtEnd::Home);
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        let ending = end(&mut governor, &clock, AtEnd::default());
        let Ending::Returned { to, took, stop } = ending else {
            panic!("expected a return, got {ending}");
        };
        assert_eq!(to, AtEnd::Home);
        assert!(stop.is_proven());
        let chain = governor.chain();
        assert_eq!(chain.stops, 1, "stopped once, at the end");
        assert!(close(&chain.at, &[0.0, -0.4]), "at home: {:?}", chain.at);
        // No step faster than the return's share of the cap: 0.125 rad/s over 20 ms.
        for pair in chain.sent.windows(2) {
            for (before, after) in pair[0].iter().zip(&pair[1]) {
                assert!((after - before).abs() <= 0.25 * RETURN_SPEED_SHARE * 0.02 + 1e-12);
            }
        }
        // 0.7 rad at 0.125 rad/s, then the settle.
        assert!(took + MAX_STEP >= Duration::from_secs_f64(5.6) + RETURN_SETTLE);
    }

    #[test]
    fn start_returns_to_where_the_body_rested_when_opened() {
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        clock.0.set(clock.0.get() + Duration::from_secs(1));
        for _ in 0..20 {
            governor.command(&[0.0, 0.0]).expect("move");
            clock.0.set(clock.0.get() + MAX_STEP);
        }
        assert_eq!(governor.opened_at(), Some(&[0.5, 0.3][..]));
        let ending = end(&mut governor, &clock, AtEnd::Start);
        assert!(
            matches!(
                ending,
                Ending::Returned {
                    to: AtEnd::Start,
                    ..
                }
            ),
            "{ending}"
        );
        assert!(close(&governor.chain().at, &[0.5, 0.3]));
    }

    #[test]
    fn release_stops_in_place_without_a_command() {
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        let ending = end(&mut governor, &clock, AtEnd::Release);
        assert!(matches!(ending, Ending::Released { .. }));
        assert!(ending.stop().is_proven() && ending.is_clean());
        assert_eq!(
            (governor.chain().sent.len(), governor.chain().stops),
            (1, 1)
        );
    }

    #[test]
    fn a_kept_axis_stays_where_it_is() {
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        let ending = end_keeping(&mut governor, &clock, AtEnd::Home, &[1]);
        assert!(matches!(ending, Ending::Returned { .. }), "{ending}");
        assert!(close(&governor.chain().at, &[0.0, 0.3]));
    }

    #[test]
    fn an_unavailable_axis_is_not_moved_or_required_to_answer() {
        let mut chain = fake([0.5, 0.3]);
        chain.unavailable = vec![1];
        let (mut governor, clock) = governed(chain);
        let ending = end(&mut governor, &clock, AtEnd::Home);
        assert!(matches!(ending, Ending::Returned { .. }), "{ending}");
        assert!((governor.chain().at[0]).abs() < 1e-9);
    }

    #[test]
    fn an_axis_that_does_not_answer_stops_in_place_with_no_command() {
        let mut chain = fake([0.5, 0.3]);
        chain.silent = Some(1);
        let (mut governor, clock) = governed(chain);
        let ending = end(&mut governor, &clock, AtEnd::Home);
        let Ending::FellBack { failure, .. } = &ending else {
            panic!("expected the fallback, got {ending}");
        };
        assert_eq!(failure, &ReturnFailure::Unobserved { joint: "b".into() });
        assert_eq!(
            (governor.chain().sent.len(), governor.chain().stops),
            (1, 1)
        );
    }

    #[test]
    fn a_latched_fault_stops_in_place_with_no_command() {
        let mut chain = fake([0.5, 0.3]);
        // Observed 0.3 from every command: the first trips the tracking stop and latches.
        chain.lag = 0.3;
        let (mut governor, clock) = opened(chain);
        assert!(governor.command(&[0.5, 0.3]).is_err());
        assert!(governor.is_latched());
        let sent = governor.chain().sent.len();
        let ending = end(&mut governor, &clock, AtEnd::Home);
        assert!(matches!(
            ending,
            Ending::FellBack {
                failure: ReturnFailure::Latched,
                ..
            }
        ));
        assert_eq!(governor.chain().sent.len(), sent, "nothing more was sent");
    }

    #[test]
    fn a_body_that_does_not_follow_falls_back_to_the_stop() {
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        governor.chain_mut().lag = 0.3;
        let ending = end(&mut governor, &clock, AtEnd::Home);
        let Ending::FellBack { failure, stop, .. } = ending else {
            panic!("expected the fallback");
        };
        assert!(matches!(failure, ReturnFailure::Governor { .. }));
        assert!(stop.is_proven());
        assert_eq!(governor.chain().sent.len(), 2);
    }

    #[test]
    fn a_return_past_its_deadline_falls_back_to_the_stop() {
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        let tick = clock.clone();
        // A caller that stalls a second per period earns 20 ms of travel per command.
        let ending = governor.end(
            AtEnd::Home,
            &[],
            None,
            &mut || tick.0.set(tick.0.get() + Duration::from_secs(1)),
            &|| false,
        );
        assert!(
            matches!(
                ending,
                Ending::FellBack {
                    failure: ReturnFailure::Deadline { .. },
                    ..
                }
            ),
            "{ending}"
        );
        assert_eq!(governor.chain().stops, 1);
    }

    #[test]
    fn a_return_too_far_for_its_cap_is_refused_before_it_moves() {
        let chain = fake([0.9, 0.3]);
        let clock = Manual(Rc::new(Cell::new(Instant::now())));
        let slow = AxisLimits {
            max_speed: 0.01,
            ..LIMITS
        };
        let mut governor = Governor::with_clock(chain, vec![slow; 2], clock.clone()).expect("ok");
        governor
            .command(&[0.9, 0.3])
            .expect("driven where it rests");
        let ending = end(&mut governor, &clock, AtEnd::Home);
        assert!(matches!(
            ending,
            Ending::FellBack {
                failure: ReturnFailure::TooFar { .. },
                ..
            }
        ));
        assert_eq!(governor.chain().sent.len(), 1);
    }

    #[test]
    fn a_body_nothing_drove_is_stopped_where_it_rests_and_not_moved() {
        let (mut governor, clock) = opened(fake([0.5, 0.3]));
        let ending = end(&mut governor, &clock, AtEnd::Home);
        assert!(matches!(ending, Ending::Rested { .. }), "{ending}");
        assert_eq!(
            (governor.chain().sent.len(), governor.chain().stops),
            (0, 1)
        );
        // The same after an operation's stop: the next end does not move it either.
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        let _ = governor.stop();
        let ending = end(&mut governor, &clock, AtEnd::Start);
        assert!(matches!(ending, Ending::Rested { .. }), "{ending}");
        assert_eq!(governor.chain().sent.len(), 1);
    }

    #[test]
    fn an_abort_stops_where_the_body_is() {
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        let tick = clock.clone();
        let asked = Cell::new(0);
        let ending = governor.end(
            AtEnd::Home,
            &[],
            None,
            &mut || tick.0.set(tick.0.get() + MAX_STEP),
            &|| {
                asked.set(asked.get() + 1);
                asked.get() > 5
            },
        );
        assert!(matches!(
            ending,
            Ending::FellBack {
                failure: ReturnFailure::Aborted,
                ..
            }
        ));
        assert_eq!(governor.chain().sent.len(), 6);
        assert_eq!(governor.chain().stops, 1);
    }

    #[test]
    fn the_choice_is_spelled_one_way() {
        for choice in AtEnd::ALL {
            assert_eq!(choice.as_str().parse::<AtEnd>(), Ok(choice));
        }
        assert!("drop".parse::<AtEnd>().is_err());
    }

    #[test]
    fn a_body_observed_short_of_its_target_is_not_arrived() {
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        // 0.1 behind every command: inside the 0.2 tracking bound, outside the tolerance.
        governor.chain_mut().lag = 0.1;
        let ending = end(&mut governor, &clock, AtEnd::Home);
        assert!(
            matches!(
                ending,
                Ending::FellBack {
                    failure: ReturnFailure::Deadline { .. },
                    ..
                }
            ),
            "{ending}"
        );
        assert!(!ending.is_clean());
    }

    #[test]
    fn a_home_outside_the_commandable_range_is_clamped_into_it() {
        let mut chain = fake([0.5, 0.3]);
        chain.axes[0].home = Some(0.95);
        let (mut governor, clock) = governed(chain);
        let ending = end(&mut governor, &clock, AtEnd::Home);
        assert!(matches!(ending, Ending::Returned { .. }), "{ending}");
        assert!(close(&governor.chain().at, &[0.9, -0.4]));
    }

    #[test]
    fn an_axis_with_no_home_stays_where_it_is() {
        let mut chain = fake([0.5, 0.3]);
        chain.axes[1].home = None;
        let (mut governor, clock) = governed(chain);
        let ending = end(&mut governor, &clock, AtEnd::Home);
        assert!(matches!(ending, Ending::Returned { .. }), "{ending}");
        assert!(close(&governor.chain().at, &[0.0, 0.3]));
    }

    #[test]
    fn the_return_carries_the_callers_feed_forward_for_each_commanded_pose() {
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        let tick = clock.clone();
        let mut gravity = |q: &[f64], out: &mut [Feedforward]| {
            for (out, q) in out.iter_mut().zip(q) {
                out.torque = 2.0 * q;
            }
        };
        let ending = governor.end(
            AtEnd::Home,
            &[],
            Some(&mut gravity),
            &mut || tick.0.set(tick.0.get() + MAX_STEP),
            &|| false,
        );
        assert!(matches!(ending, Ending::Returned { .. }), "{ending}");
        let chain = governor.chain();
        // Every return command carried it, matching the pose it was computed for.
        let returned = &chain.sent[1..];
        assert_eq!(chain.feedforward.len(), returned.len());
        for (sent, feedforward) in returned.iter().zip(&chain.feedforward) {
            for (q, ff) in sent.iter().zip(feedforward) {
                assert!((ff.torque - 2.0 * q).abs() < 0.25 * 0.02 * 2.0 + 1e-9);
            }
        }
    }

    #[test]
    fn a_joint_resting_outside_its_bounds_is_held_then_brought_in() {
        // Axis a rests at 1.2, outside its declared -1..1 and commandable -0.9..0.9.
        let (mut governor, clock) = governed(fake([1.2, 0.3]));
        assert!(close(
            governor.sent().expect("held where it rests"),
            &[1.2, 0.3]
        ));
        let ending = end(&mut governor, &clock, AtEnd::Home);
        assert!(matches!(ending, Ending::Returned { .. }), "{ending}");
        assert!(close(&governor.chain().at, &[0.0, -0.4]));
        // It never went further out.
        assert!(
            governor
                .chain()
                .sent
                .iter()
                .all(|sent| sent[0] <= 1.2 + 1e-12)
        );
    }

    #[test]
    fn a_settled_return_holds_the_body_there_without_a_stop_until_a_fault() {
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        let mut ret = Return::begin(&mut governor, AtEnd::Home, &[]).expect("begins");
        let mut ticks = 0;
        while ret.tick(&mut governor, None).expect("returns") == Progress::Moving {
            clock.0.set(clock.0.get() + MAX_STEP);
            ticks += 1;
            assert!(ticks < 10_000, "never settled");
        }
        // Held for a minute past any deadline: the body stays enabled at home.
        for _ in 0..3_000 {
            clock.0.set(clock.0.get() + MAX_STEP);
            ret.hold(&mut governor, None).expect("held");
        }
        assert!(close(&governor.chain().at, &[0.0, -0.4]));
        assert_eq!(governor.chain().stops, 0, "never disabled while held");
        // A body that stops following while held is stopped by the governor and latched.
        governor.chain_mut().lag = 0.3;
        assert!(matches!(
            ret.hold(&mut governor, None),
            Err(ReturnFailure::Governor { .. })
        ));
        assert_eq!(governor.chain().stops, 1);
        assert!(governor.is_latched());
    }

    #[test]
    fn a_return_whose_stop_is_unproven_is_not_clean() {
        let mut chain = fake([0.5, 0.3]);
        chain.unproven = vec![2];
        let (mut governor, clock) = governed(chain);
        let ending = end(&mut governor, &clock, AtEnd::Home);
        assert!(matches!(ending, Ending::Returned { .. }), "{ending}");
        assert!(!ending.is_clean(), "an actuator may still hold torque");
    }

    #[test]
    fn the_return_steps_from_what_was_last_sent_not_from_the_read() {
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        // The body reads 0.15 short of what was sent: inside the tracking bound.
        governor.chain_mut().lag = -0.15;
        governor.command(&[0.5, 0.3]).expect("held");
        clock.0.set(clock.0.get() + MAX_STEP);
        let sent = governor.sent().expect("sent").to_vec();
        let mut ret = Return::begin(&mut governor, AtEnd::Home, &[]).expect("begins");
        ret.tick(&mut governor, None).expect("first step");
        let first = governor.sent().expect("sent");
        for (after, before) in first.iter().zip(&sent) {
            assert!(
                (after - before).abs() <= 0.25 * RETURN_SPEED_SHARE * 0.02 + 1e-12,
                "stepped {before} -> {after}"
            );
        }
    }

    #[test]
    fn a_hold_before_the_return_settled_keeps_approaching_at_its_pace() {
        let (mut governor, clock) = governed(fake([0.5, 0.3]));
        let mut ret = Return::begin(&mut governor, AtEnd::Home, &[]).expect("begins");
        for _ in 0..1_000 {
            clock.0.set(clock.0.get() + MAX_STEP);
            ret.hold(&mut governor, None).expect("held");
        }
        assert!(close(&governor.chain().at, &[0.0, -0.4]));
        for pair in governor.chain().sent.windows(2) {
            for (before, after) in pair[0].iter().zip(&pair[1]) {
                assert!((after - before).abs() <= 0.25 * RETURN_SPEED_SHARE * 0.02 + 1e-12);
            }
        }
    }
}
