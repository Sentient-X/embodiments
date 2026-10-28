//! A speed and tracking-error bound around any actuator chain.
//!
//! A chain drives each axis to whatever target it is given, as fast as its gains pull. The
//! [`Governor`] stands in front of it as the same [`ActuatorChain`], and owns two bounds per axis
//! ([`AxisLimits`]), set by whoever operates the body (the static upper body's received driver
//! ran 0.25 rad/s and 0.2 rad at bring-up):
//!
//! - **Speed.** Each command moves every axis from the position last sent toward its target by
//!   at most `max_speed` times the time since the last accepted command, that time capped at
//!   [`MAX_STEP_INTERVAL`] so a caller that stalls earns no larger step. The first command after the chain was opened,
//!   stopped or failed starts from where the body rests: a position given with
//!   [`Governor::start_at`], else the chain's own [`ActuatorChain::last_observed`]. With
//!   neither, the command is refused.
//! - **Acceleration.** With `max_acceleration`, the commanded speed changes by at most that
//!   times the step's time, and never exceeds the speed from which the axis can still brake to
//!   its target at that rate: it ramps up, cruises and brakes instead of jumping to `max_speed`
//!   (an abrupt brake regenerates a spike onto the supply). The speed is held on the governor in
//!   a fixed [`Axes`], so the bound allocates nothing per tick; it restarts from rest at the
//!   first command after open, a stop or a failure.
//! - **Derate.** Each axis the chain reports derated ([`ActuatorChain::derated`]: commissioning
//!   measured its rail over its model's supported supply, [`crate::rail`]) is held to the
//!   tighter of its limits and the derate's: a lower speed, an acceleration cap, and
//!   feed-forward torque at most a share of its model's peak, whatever is asked or set by
//!   [`ActuatorChain::set_feedforward_limits`]. The derate is data on the chain, so it lifts
//!   when the body is next opened on a rail in range.
//! - **Range is a limit on motion, not on state.** Each step is held inside an interval per
//!   axis: when the governor takes the body up (after open, a stop or a failure), the chain's
//!   commandable range widened to where the joint is ([`CommandRange::reaching`]):
//!   `[min(lower, now), max(upper, now)]`, `now` being the chain's latest observation
//!   ([`ActuatorChain::last_observed`]) or the position a command starts from; after that it
//!   only narrows, to the range widened to where the joint is now, as the joint comes in. A
//!   joint resting outside its range can hold or move toward it at `max_speed`, never further
//!   out, and one pushed further out (an arm sagging, a hand) is not followed: the command stays
//!   at the interval's edge and the tracking check catches a joint that does not come back. The
//!   chains check every command against the same rule ([`CommandRange::admissible`], plus read
//!   noise). A finite target outside the interval is not refused: it is brought in to the edge
//!   ([`Governor::brought_in`] says which axis), so a caller that read the joint a moment before
//!   the chain did (a joint drifting out while idle) is not stopped for it. Nothing is refused
//!   or latched for where a joint rests.
//! - **Fail closed.** Every receipt must observe every axis: a short or empty observation stops
//!   the chain and latches [`GovernorError::Unobserved`], so a chain that observes nothing
//!   cannot be governed. A chain error other than a refusal (the chain may have applied part
//!   of the command) stops the chain and latches [`GovernorError::Faulted`]; a refusal, which
//!   changed nothing, is returned as it is.
//! - **One late answer is data, two are a fault.** An axis the receipt observes as `NaN` (its
//!   actuator's answer missed the command's deadline: [`AxisReading::Missing`] in
//!   [`ActuatorChain::command_readings`]) is not tracked that tick, and the next command steps
//!   from the position last sent. [`MAX_MISSED_IN_A_ROW`] such ticks in a row are tolerated; one
//!   more stops the chain and latches [`GovernorError::Unanswered`]. Each tick moves an axis at
//!   most `max_speed` × [`MAX_STEP_INTERVAL`] (10 mrad at 0.5 rad/s), so an axis goes
//!   unobserved for at most one such step before it is tracked again or stopped, well inside
//!   the actuators' own communication watchdogs. A late answer is usually the host, not the
//!   motor: an overloaded macOS scheduler parks a thread for tens of milliseconds (run CSVs of
//!   2026-10-26 show 50 Hz ticks up to 98 ms apart at load 30–100), and stopping a run on it
//!   threw away a good run while proving nothing about the body.
//! - **Unavailable axes** ([`ActuatorChain::unavailable`], marked by commissioning) keep their
//!   place in every vector but are neither stepped nor tracked: their targets reach no
//!   actuator, and their observations are `NaN`.
//! - **Tracking error.** When a command's receipt carries observed positions, each must lie
//!   within `max_tracking_error` of what was just sent. One that does not stops the chain and
//!   latches a fault: every later command is refused until [`ActuatorChain::clear_faults`].
//!
//! The governor bounds what the station asks; it is not a substitute for the motors' own
//! protections.

use std::time::{Duration, Instant};

use thiserror::Error;

use crate::axes::Axes;
use crate::binding::BoundAxis;
use crate::chain::{
    ActuatorChain, AxisReading, ChainError, ChainFailure, CommandRange, Feedforward,
    FeedforwardLimit, Receipt, StopReport, TargetError,
};
use crate::masked::check_available;
use crate::rail::AxisDerate;

/// The longest time one command's speed bound counts: one control period at 50 Hz.
pub const MAX_STEP_INTERVAL: Duration = Duration::from_millis(20);

/// How many commands in a row one axis may go unobserved (`NaN` in the receipt) before the
/// governor stops the chain.
pub const MAX_MISSED_IN_A_ROW: u8 = 1;
/// A joint resting outside its range that comes no closer to it by this much (rad, a few
/// encoder counts above read noise) within [`OUTSIDE_STALL`] cannot come in.
pub const OUTSIDE_PROGRESS: f64 = 0.005;
/// How far past its range (rad) a joint must be observed to count as outside it: a few
/// encoder counts and more than a servo's sag or deadband at the edge, which is not a stuck
/// joint (a joint 3 mrad past the edge it holds must never be given up on).
pub const OUTSIDE_TOLERANCE: f64 = 0.01;
/// How long a joint outside its range may make no progress toward it before the governor stops
/// pushing it in and holds it where it is.
pub const OUTSIDE_STALL: Duration = Duration::from_secs(2);

/// One axis's bounds, in the joint's own unit (rad or m) and per second.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisLimits {
    /// The fastest a command moves the axis, unit/s.
    pub max_speed: f64,
    /// The farthest an observed position may lie from the position just sent, unit.
    pub max_tracking_error: f64,
    /// The fastest the commanded speed changes, unit/s², or `None` for no bound.
    pub max_acceleration: Option<f64>,
}

/// The time the governor measures steps by.
pub trait Clock {
    fn now(&self) -> Instant;
}

/// The host's monotonic clock.
#[derive(Clone, Copy, Debug, Default)]
pub struct MonotonicClock;

impl Clock for MonotonicClock {
    fn now(&self) -> Instant {
        Instant::now()
    }
}

/// Limits the governor refuses to be built with.
#[derive(Debug, Error, PartialEq)]
pub enum LimitsError {
    #[error("expected limits for {expected} axes, got {actual}")]
    Width { expected: usize, actual: usize },
    #[error(
        "joint {0} needs a finite, positive speed and tracking-error bound, and acceleration bound where given"
    )]
    NotPositive(String),
    /// A run's chosen over-rail derate whose speed or acceleration is not finite and positive:
    /// taken as asked it would lift the derate (NaN) or stall or crash the step.
    #[error("a chosen derate needs a finite, positive speed and acceleration")]
    Derate,
}

/// A command the governor refused, or a fault it stopped the chain for.
#[derive(Debug, Error)]
pub enum GovernorError<E> {
    /// The chain refused the command before any frame changed what an actuator holds.
    #[error(transparent)]
    Chain(E),
    /// The chain failed mid-command, so something may have been applied: it was stopped and
    /// the fault is latched.
    #[error("{error}; the chain was stopped and the fault is latched")]
    Faulted { error: E, stop: StopReport },
    #[error(transparent)]
    Target(#[from] TargetError),
    #[error("no start position: the chain observes none and none was given")]
    NoStart,
    #[error(
        "joint {joint} was sent {commanded} but observed at {observed}, more than {limit} away; the chain was stopped and the fault is latched"
    )]
    Tracking {
        joint: String,
        commanded: f64,
        observed: f64,
        limit: f64,
        stop: Box<StopReport>,
    },
    #[error(
        "the receipt observed {found} of {expected} axes, so tracking could not be checked; the chain was stopped and the fault is latched"
    )]
    Unobserved {
        expected: usize,
        found: usize,
        stop: StopReport,
    },
    #[error(
        "joint {joint} went unobserved for {ticks} commands in a row; the chain was stopped and the fault is latched"
    )]
    Unanswered {
        joint: String,
        ticks: u8,
        stop: StopReport,
    },
    #[error("a fault is latched; recover the body and clear it before commanding")]
    Latched,
}

impl<E: ChainError> ChainError for GovernorError<E> {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::Chain(error) | Self::Faulted { error, .. } => error.failure(),
            Self::Target(_) | Self::NoStart | Self::Latched => ChainFailure::Refused,
            Self::Tracking { .. } | Self::Unobserved { .. } => ChainFailure::Fault,
            Self::Unanswered { .. } => ChainFailure::Timeout,
        }
    }
}

/// An axis observed outside its range: since when it last came closer, how far out it was then,
/// and whether the governor gave up pushing it in.
#[derive(Clone, Copy, Debug)]
struct Outside {
    since: Instant,
    out: f64,
    given_up: bool,
    /// Once given up on: the deepest the caller's target has asked it in since (how far past
    /// the edge it rests outside of; negative inside). Only a target deeper still approaches it
    /// again.
    asked: Option<f64>,
    /// How many times it was approached again, at most [`MAX_REAPPROACHES`].
    rearmed: u8,
}

/// How many times one joint given up on outside its range is approached again, each on a
/// deeper request: bounded, so a joint that truly cannot come in (a stop, a wrong range) is
/// not pushed again and again.
pub const MAX_REAPPROACHES: u8 = 3;

/// The position last sent, the speed it was stepped at, and when.
#[derive(Clone, Copy)]
struct Held {
    position: Axes<f64>,
    velocity: Axes<f64>,
    at: Option<Instant>,
    /// Per axis, the interval steps may go to: the range widened to where the joint was when
    /// the governor took it up, narrowed as it comes in, never widened again. A joint pushed
    /// further out (a sagging arm, a hand) is not followed out: the command stays at the edge
    /// it had, and the tracking check catches a joint that does not come back.
    lower: Axes<f64>,
    upper: Axes<f64>,
}

/// A chain whose commands are slewed and whose tracking is checked; see the module docs.
pub struct Governor<C, K = MonotonicClock> {
    chain: C,
    limits: Vec<AxisLimits>,
    clock: K,
    held: Option<Held>,
    start: Option<Axes<f64>>,
    latched: bool,
    /// The chain's unavailable axes, neither stepped nor tracked.
    unavailable: Vec<usize>,
    /// Per axis, the commands in a row whose receipt did not observe it.
    missed: Vec<u8>,
    /// The chain's derated axes, already applied to `limits` and `torque_caps`.
    derated: Vec<AxisDerate>,
    /// The largest feed-forward torque each axis is sent, N·m, where a derate bounds it.
    torque_caps: Vec<Option<f64>>,
    /// Where the chain observed the body when the governor was built over it; on the heap,
    /// read once at the end, so the governor a control future holds stays small.
    opened_at: Option<Vec<f64>>,
    /// The first axis whose last target lay outside where it may go, and was brought in.
    brought_in: Option<usize>,
    /// Per axis observed outside its range: since when it last came closer, how far out it was
    /// then, and whether it was given up on (held where it rests, not pushed against its stop).
    outside: Vec<Option<Outside>>,
    /// When the last receipt was noted: a pause in commands is not time to make progress in.
    noted: Option<Instant>,
    /// Each axis's interval (`Held::lower`), kept across a refused command, which re-seeds the
    /// position from the chain but must not widen the interval; cleared only where the governor
    /// takes the joint up anew (open, a stop, a fault, `clear_faults`, `start_at`).
    interval: Option<(Axes<f64>, Axes<f64>)>,
}

impl<C: ActuatorChain> Governor<C> {
    /// Govern `chain` with one [`AxisLimits`] per axis, in native state order.
    ///
    /// # Errors
    ///
    /// Refuses limits of the wrong width, and a bound that is not finite and positive. Each
    /// axis the chain reports derated is held to the tighter of its limits and the derate's.
    pub fn new(chain: C, limits: Vec<AxisLimits>) -> Result<Self, LimitsError> {
        Self::with_clock(chain, limits, MonotonicClock)
    }

    /// [`Governor::new`], each derated axis held to `derate`'s speed and acceleration instead
    /// of the rail's automatic ones ([`crate::rail::OVER_RAIL`]) when given: a person's
    /// declared decision for this run (the owner's 0.6 rad/s for the J8009P on the 54 V
    /// battery, 2026-10-26). The derate's feed-forward torque share is kept.
    ///
    /// # Errors
    ///
    /// As [`Governor::new`].
    pub fn with_derate(
        chain: C,
        limits: Vec<AxisLimits>,
        derate: Option<crate::rail::DerateLimits>,
    ) -> Result<Self, LimitsError> {
        Self::governed(chain, limits, MonotonicClock, derate)
    }
}

impl<C: ActuatorChain, K: Clock> Governor<C, K> {
    /// [`Governor::new`], measuring time by `clock`.
    ///
    /// # Errors
    ///
    /// As [`Governor::new`].
    pub fn with_clock(chain: C, limits: Vec<AxisLimits>, clock: K) -> Result<Self, LimitsError> {
        Self::governed(chain, limits, clock, None)
    }

    fn governed(
        chain: C,
        limits: Vec<AxisLimits>,
        clock: K,
        derate: Option<crate::rail::DerateLimits>,
    ) -> Result<Self, LimitsError> {
        let axes = chain.axes();
        if limits.len() != axes.len() {
            return Err(LimitsError::Width {
                expected: axes.len(),
                actual: limits.len(),
            });
        }
        let positive = |value: f64| value.is_finite() && value > 0.0;
        let chain_unavailable = chain.unavailable();
        let width = axes.len();
        if let Some((axis, _)) = axes.iter().zip(&limits).find(|(_, limit)| {
            !(positive(limit.max_speed)
                && positive(limit.max_tracking_error)
                && limit.max_acceleration.is_none_or(positive))
        }) {
            return Err(LimitsError::NotPositive(axis.joint.clone()));
        }
        if derate.is_some_and(|d| !(positive(d.max_speed) && positive(d.max_acceleration))) {
            return Err(LimitsError::Derate);
        }
        let opened_at = chain
            .last_observed()
            .filter(|observed| observed.len() == width)
            .map(|observed| observed.to_vec());
        let mut limits = limits;
        let mut torque_caps = vec![None; limits.len()];
        let mut derated = chain.derated();
        if let Some(chosen) = derate {
            for axis in &mut derated {
                axis.limits.max_speed = chosen.max_speed;
                axis.limits.max_acceleration = chosen.max_acceleration;
            }
        }
        for derate in &derated {
            let (Some(limit), Some(bound)) = (limits.get_mut(derate.axis), axes.get(derate.axis))
            else {
                continue;
            };
            limit.max_speed = limit.max_speed.min(derate.limits.max_speed);
            limit.max_acceleration = Some(
                limit
                    .max_acceleration
                    .map_or(derate.limits.max_acceleration, |own| {
                        own.min(derate.limits.max_acceleration)
                    }),
            );
            torque_caps[derate.axis] =
                Some(derate.limits.torque_share * bound.binding.model.rating().peak_torque_nm);
        }
        // Each derate says what its axis is held to in force: the tighter of the derate and
        // the governor's own limit, so a derate never reports a speed the axis cannot reach.
        for derate in &mut derated {
            if let Some(limit) = limits.get(derate.axis) {
                derate.limits.max_speed = limit.max_speed;
                derate.limits.max_acceleration = limit
                    .max_acceleration
                    .unwrap_or(derate.limits.max_acceleration);
            }
        }
        Ok(Self {
            chain,
            limits,
            clock,
            held: None,
            start: None,
            latched: false,
            unavailable: chain_unavailable,
            missed: vec![0; width],
            derated,
            torque_caps,
            opened_at,
            brought_in: None,
            outside: vec![None; width],
            noted: None,
            interval: None,
        })
    }

    /// Where the body rests, as the operator states it: the next command after open, a stop or
    /// a failure starts from it, ahead of the chain's own observation. It is how a chain that
    /// observes nothing is started.
    ///
    /// # Errors
    ///
    /// Refuses a vector of the wrong width or outside the declared bounds.
    pub fn start_at(&mut self, joints: &[f64]) -> Result<(), TargetError> {
        check_available(self.chain.axes(), &self.unavailable, joints)?;
        self.start = Some(Axes::from_slice(joints));
        self.held = None;
        self.interval = None;
        self.outside.fill(None);
        self.noted = None;
        Ok(())
    }

    /// What the last accepted command sent the chain, each axis slewed toward its target:
    /// `None` since open, a stop or a failure.
    #[must_use]
    pub fn sent(&self) -> Option<&[f64]> {
        self.held
            .as_ref()
            .filter(|held| held.at.is_some())
            .map(|held| &*held.position)
    }

    /// Each axis's limits in force: the ones given, tightened by any derate.
    #[must_use]
    pub fn limits(&self) -> &[AxisLimits] {
        &self.limits
    }

    /// Where the chain observed the body when this governor was built over it: the rest an
    /// [`crate::AtEnd::Start`] returns to. `None` for a chain that observed none.
    #[must_use]
    pub fn opened_at(&self) -> Option<&[f64]> {
        self.opened_at.as_deref()
    }

    /// The first axis whose last command's target lay outside where that axis may go and was
    /// brought in to the edge instead ([`CommandRange::admissible`]); `None` when every target
    /// was taken as asked. A caller that should never ask past a range (a policy, a solver
    /// bounded by the same ranges) reports it.
    #[must_use]
    pub const fn brought_in(&self) -> Option<usize> {
        self.brought_in
    }

    /// The first axis resting outside its range that made no progress toward it for
    /// [`OUTSIDE_STALL`] (a servo too weak against gravity on its stop): it is held where it
    /// rests, no longer pushed in, until it is observed inside its range again or the governor
    /// takes the body up anew. A caller reports it.
    #[must_use]
    pub fn held_outside(&self) -> Option<usize> {
        self.outside
            .iter()
            .position(|outside| outside.is_some_and(|outside| outside.given_up))
    }

    /// Whether a fault is latched: every command is refused until
    /// [`ActuatorChain::clear_faults`].
    #[must_use]
    pub const fn is_latched(&self) -> bool {
        self.latched
    }

    /// The clock steps are measured by.
    pub const fn clock(&self) -> &K {
        &self.clock
    }

    /// The governed chain.
    pub const fn chain(&self) -> &C {
        &self.chain
    }

    /// The governed chain, for what only it offers.
    pub const fn chain_mut(&mut self) -> &mut C {
        &mut self.chain
    }

    /// The governed chain, given up: everything this governor learned of it (where it rests,
    /// a latched fault, its derates) is dropped with it, so the chain is governed again only
    /// by a new governor, which takes it up from what it reads.
    pub fn into_chain(self) -> C {
        self.chain
    }

    /// Where the next command starts: the operator's start, else the chain's observation.
    fn seed(&mut self) -> Result<Held, GovernorError<C::Error>> {
        let width = self.chain.axes().len();
        let position = match self.start.take() {
            Some(start) => start,
            None => self
                .chain
                .last_observed()
                .filter(|observed| observed.len() == width)
                .ok_or(GovernorError::NoStart)?,
        };
        let (lower, upper): (Axes<f64>, Axes<f64>) = position
            .iter()
            .zip(self.chain.commandable())
            .enumerate()
            .map(|(index, (at, range))| {
                let reach = range.reaching(*at);
                match &self.interval {
                    Some((lower, upper)) => {
                        (reach.lower.max(lower[index]), reach.upper.min(upper[index]))
                    }
                    None => (reach.lower, reach.upper),
                }
            })
            .unzip();
        Ok(Held {
            velocity: Axes::filled(0.0, position.len()),
            position,
            at: None,
            lower,
            upper,
        })
    }

    /// Stop the chain and latch a fault, returning the error `make` builds from the stop.
    fn fault(
        &mut self,
        make: impl FnOnce(StopReport) -> GovernorError<C::Error>,
    ) -> GovernorError<C::Error> {
        self.latched = true;
        self.held = None;
        self.interval = None;
        self.outside.fill(None);
        self.noted = None;
        self.missed.fill(0);
        let stop = self.chain.stop();
        make(stop)
    }
}

/// Whether `observed` lies within `limit` of `sent`; never for a NaN.
fn within(observed: f64, sent: f64, limit: f64) -> bool {
    (observed - sent).abs() <= limit
}

/// `value` moved toward `target` for `dt` seconds at most `limit.max_speed`, the speed changed
/// from `speed` by at most `limit.max_acceleration * dt` and never past the speed it can brake
/// to `target` from, held inside `reach`. Returns the position and the speed it was stepped at.
fn slew(
    value: f64,
    target: f64,
    speed: f64,
    limit: &AxisLimits,
    dt: f64,
    reach: CommandRange,
) -> (f64, f64) {
    // The target brought inside `reach` first, so the axis brakes onto the edge rather than
    // meeting it at speed; every step stays inside it too.
    let target = target.clamp(reach.lower, reach.upper);
    if dt <= 0.0 {
        return (value.clamp(reach.lower, reach.upper), speed);
    }
    let gap = target - value;
    let mut wanted = (gap / dt).clamp(-limit.max_speed, limit.max_speed);
    if let Some(acceleration) = limit.max_acceleration {
        let braking = (2.0 * acceleration * gap.abs()).sqrt();
        let change = acceleration * dt;
        wanted = wanted
            .clamp(-braking, braking)
            .clamp(speed - change, speed + change);
    }
    let next = (value + wanted * dt).clamp(reach.lower, reach.upper);
    (next, (next - value) / dt)
}

impl<C: ActuatorChain, K: Clock> Governor<C, K> {
    /// Each axis moved from `held` toward `joints` for `step` seconds within its limits, inside
    /// its interval (`Held::lower`) narrowed to its range widened to where it is now; an
    /// unavailable axis at any in-range value, at rest. Returns the positions, their speeds,
    /// the narrowed intervals and the first axis whose target lay outside its interval.
    #[allow(clippy::type_complexity)] // One step's four answers, used once.
    fn step(
        &self,
        held: &Held,
        joints: &[f64],
        step: f64,
    ) -> (Axes<f64>, Axes<f64>, (Axes<f64>, Axes<f64>), Option<usize>) {
        // Where each joint is now: the chain's latest observation, else the position last sent.
        let observed = self
            .chain
            .last_observed()
            .filter(|observed| observed.len() == held.position.len());
        let mut positions = Axes::new();
        let mut speeds = Axes::new();
        let (mut lowers, mut uppers) = (Axes::new(), Axes::new());
        let mut brought_in = None;
        for (index, ((value, target), (limit, range))) in held
            .position
            .iter()
            .zip(joints)
            .zip(self.limits.iter().zip(self.chain.commandable()))
            .enumerate()
        {
            if self.unavailable.contains(&index) {
                // Never reaches an actuator: any in-range value keeps the vector whole.
                let target = if target.is_finite() {
                    *target
                } else {
                    range.lower
                };
                positions.push(target.clamp(range.lower, range.upper));
                speeds.push(0.0);
                lowers.push(range.lower);
                uppers.push(range.upper);
                continue;
            }
            let at = observed
                .as_ref()
                .map(|observed| observed[index])
                .filter(|at| at.is_finite())
                .unwrap_or(*value);
            let speed = held.velocity.get(index).copied().unwrap_or(0.0);
            // Given up on: held where it rests, never pushed against its stop.
            let target = if self.outside[index].is_some_and(|outside| outside.given_up) {
                &at
            } else {
                target
            };
            let now = range.reaching(at);
            // Never empty: both contain the range itself.
            let reach = CommandRange {
                lower: held.lower[index].max(now.lower),
                upper: held.upper[index].min(now.upper),
            };
            if brought_in.is_none() && !reach.admits(*target) {
                brought_in = Some(index);
            }
            let (mut position, mut speed) = slew(*value, *target, speed, limit, step, reach);
            // A joint being brought in from outside its range is never commanded further in
            // than half its tracking bound from where it is: enough error to move it, never a
            // tracking fault for a joint slow to come. Only ever toward where it is, and always
            // inside this step's interval: a joint pushed further out is not followed out, and
            // the tracking check still catches it.
            if self.outside[index].is_some() {
                let lead = limit.max_tracking_error / 2.0;
                let brought = if at > range.upper {
                    position.max((at - lead).min(reach.upper))
                } else if at < range.lower {
                    position.min((at + lead).max(reach.lower))
                } else {
                    position
                };
                if (brought - position).abs() > f64::EPSILON && step > 0.0 {
                    speed = (brought - value) / step;
                }
                position = brought;
            }
            positions.push(position);
            speeds.push(speed);
            lowers.push(reach.lower);
            uppers.push(reach.upper);
        }
        (positions, speeds, (lowers, uppers), brought_in)
    }

    /// The feed-forward each axis is sent: its planned velocity held to its speed, its torque to
    /// its derate's cap, and none for an axis given up on outside its range, which rests on its
    /// stop with neither planned motion nor torque lifting it.
    fn held_feedforward(&self, wanted: &[Feedforward]) -> Axes<Feedforward> {
        wanted
            .iter()
            .zip(self.limits.iter().zip(&self.torque_caps))
            .zip(&self.outside)
            .map(|((wanted, (limit, cap)), outside)| {
                if outside.is_some_and(|outside| outside.given_up) {
                    Feedforward::default()
                } else {
                    Feedforward {
                        velocity: wanted.velocity.clamp(-limit.max_speed, limit.max_speed),
                        torque: cap.map_or(wanted.torque, |cap| wanted.torque.clamp(-cap, cap)),
                    }
                }
            })
            .collect()
    }

    /// Approach again a joint given up on outside its range once the caller asks it further in
    /// than any request since, by more than [`OUTSIDE_TOLERANCE`], at most [`MAX_REAPPROACHES`]
    /// times: a steady or jittering request never re-arms it.
    /// A joint that stalled at the edge (friction or a cable holding it against the small
    /// error the edge leaves; 10-28, the Spark's right forearm yaw stuck 0.021 rad out) is
    /// pushed with the error the operator's target gives, within the tracking bound, rather
    /// than held for the rest of the session. One that again makes no progress is given up on
    /// again after [`OUTSIDE_STALL`]; nothing is ever pushed further out.
    fn rearm_outside(&mut self, joints: &[f64], now: Instant) {
        let Some(observed) = self.chain.last_observed() else {
            return;
        };
        for (index, ((target, at), range)) in joints
            .iter()
            .zip(observed.iter())
            .zip(self.chain.commandable())
            .enumerate()
        {
            let Some(outside) = self.outside.get_mut(index).and_then(Option::as_mut) else {
                continue;
            };
            if !outside.given_up || !at.is_finite() {
                continue;
            }
            // How far past the edge it rests outside of the target lies (negative: inside).
            let past = |value: f64| {
                if *at > range.upper {
                    value - range.upper
                } else {
                    range.lower - value
                }
            };
            let asked = past(*target);
            match outside.asked {
                // The deepest target asked since it was given up on: held while the caller asks
                // no further in than any request before.
                None => outside.asked = Some(asked),
                Some(deepest)
                    if asked + OUTSIDE_TOLERANCE < deepest
                        && asked < past(*at)
                        && outside.rearmed < MAX_REAPPROACHES =>
                {
                    outside.given_up = false;
                    outside.since = now;
                    outside.asked = Some(asked);
                    outside.rearmed += 1;
                }
                Some(deepest) => outside.asked = Some(deepest.min(asked)),
            }
        }
    }

    /// Note how far each available axis observed outside its range is from it, and give up on
    /// one that came no closer by [`OUTSIDE_PROGRESS`] within [`OUTSIDE_STALL`]; one observed
    /// inside is followed as usual again.
    fn note_outside(&mut self, observed: &[f64], now: Instant) {
        // After a pause in commands (longer than a step), every approach starts its wait anew:
        // nothing was asked of the joint meanwhile, so it had no chance to come in.
        let paused = self
            .noted
            .is_none_or(|noted| now.saturating_duration_since(noted) > MAX_STEP_INTERVAL);
        self.noted = Some(now);
        for (index, (at, range)) in observed.iter().zip(self.chain.commandable()).enumerate() {
            if self.unavailable.contains(&index) || !at.is_finite() {
                continue;
            }
            let out = (range.lower - at).max(at - range.upper);
            let slot = &mut self.outside[index];
            if out <= OUTSIDE_TOLERANCE {
                *slot = None;
                continue;
            }
            match slot {
                None => {
                    *slot = Some(Outside {
                        since: now,
                        out,
                        given_up: false,
                        asked: None,
                        rearmed: 0,
                    });
                }
                Some(outside) if paused || out < outside.out - OUTSIDE_PROGRESS => {
                    outside.since = now;
                    outside.out = outside.out.min(out);
                }
                Some(outside) => {
                    if now.saturating_duration_since(outside.since) >= OUTSIDE_STALL {
                        outside.given_up = true;
                    }
                }
            }
        }
    }

    /// Count each available axis' unobserved (`NaN`) commands in a row, restarting at an
    /// observed one; the first axis past [`MAX_MISSED_IN_A_ROW`], counting nothing then.
    fn count_missed(&mut self, observed: &[f64]) -> Option<usize> {
        let past = observed.iter().enumerate().find(|(index, value)| {
            !self.unavailable.contains(index)
                && value.is_nan()
                && self.missed[*index] >= MAX_MISSED_IN_A_ROW
        });
        if let Some((index, _)) = past {
            return Some(index);
        }
        for (index, value) in observed.iter().enumerate() {
            if self.unavailable.contains(&index) {
                continue;
            }
            self.missed[index] = if value.is_nan() {
                self.missed[index] + 1
            } else {
                0
            };
        }
        None
    }

    /// Refuse a vector of the wrong width, or a target that is not a number on an available
    /// axis. Any finite target is taken: each step moves toward it only as far as it is
    /// admissible ([`CommandRange::admissible`]), so one outside its axis's range is brought in
    /// to the nearest edge from where the joint is, never refused for where the joint has moved
    /// since the caller read it.
    fn check_finite(&self, joints: &[f64]) -> Result<(), TargetError> {
        let axes = self.chain.axes();
        if joints.len() != axes.len() {
            return Err(TargetError::Width {
                expected: axes.len(),
                actual: joints.len(),
            });
        }
        match axes
            .iter()
            .zip(joints)
            .enumerate()
            .find(|(index, (_, value))| !self.unavailable.contains(index) && !value.is_finite())
        {
            Some((_, (refused, value))) => Err(TargetError::OutOfBounds {
                joint: refused.joint.clone(),
                value: *value,
            }),
            None => Ok(()),
        }
    }

    /// Slew toward `joints` and command the chain, with `feedforward` when given.
    fn drive(
        &mut self,
        joints: &[f64],
        feedforward: Option<&[Feedforward]>,
    ) -> Result<Receipt<C::Feedback>, GovernorError<C::Error>> {
        if self.latched {
            return Err(GovernorError::Latched);
        }
        self.check_finite(joints)?;
        if let Some(wanted) = feedforward
            && wanted.len() != joints.len()
        {
            return Err(TargetError::Width {
                expected: joints.len(),
                actual: wanted.len(),
            }
            .into());
        }
        let now = self.clock.now();
        let held = match self.held.take() {
            Some(held) => held,
            None => self.seed()?,
        };
        let elapsed = held.at.map_or(MAX_STEP_INTERVAL, |at| {
            now.saturating_duration_since(at).min(MAX_STEP_INTERVAL)
        });
        self.rearm_outside(joints, now);
        let (next, velocity, (lower, upper), brought_in) =
            self.step(&held, joints, elapsed.as_secs_f64());
        self.brought_in = brought_in;
        self.interval = Some((lower, upper));
        let receipt = match feedforward {
            Some(wanted) => {
                let held = self.held_feedforward(wanted);
                self.chain.command_with(&next, &held)
            }
            None => self.chain.command(&next),
        };
        let receipt = match receipt {
            Ok(receipt) => receipt,
            // Refused before any frame: nothing was applied, and the next command starts again
            // from where the chain reports the body.
            Err(error) if error.failure() == ChainFailure::Refused => {
                return Err(GovernorError::Chain(error));
            }
            // Failed mid-command: something may have been applied.
            Err(error) => return Err(self.fault(|stop| GovernorError::Faulted { error, stop })),
        };
        let width = next.len();
        if receipt.observed.len() < width {
            let found = receipt.observed.len();
            return Err(self.fault(|stop| GovernorError::Unobserved {
                expected: width,
                found,
                stop,
            }));
        }
        if let Some(index) = self.count_missed(&receipt.observed[..width]) {
            let joint = self.chain.axes()[index].joint.clone();
            let ticks = self.missed[index] + 1;
            return Err(self.fault(|stop| GovernorError::Unanswered { joint, ticks, stop }));
        }
        let strayed = receipt
            .observed
            .iter()
            .zip(&next)
            .zip(self.limits.iter().zip(self.chain.axes()))
            .enumerate()
            .filter(|(index, ((observed, _), _))| {
                !(self.unavailable.contains(index) || observed.is_nan())
            })
            .map(|(_, pair)| pair)
            .find(|((observed, sent), (limit, _))| {
                !within(**observed, **sent, limit.max_tracking_error)
            })
            .map(|((observed, sent), (limit, axis))| {
                (
                    axis.joint.clone(),
                    *sent,
                    *observed,
                    limit.max_tracking_error,
                )
            });
        if let Some((joint, commanded, observed, limit)) = strayed {
            return Err(self.fault(|stop| GovernorError::Tracking {
                joint,
                commanded,
                observed,
                limit,
                stop: Box::new(stop),
            }));
        }
        self.note_outside(&receipt.observed[..width], now);
        self.held = Some(Held {
            position: next,
            velocity,
            at: Some(now),
            lower,
            upper,
        });
        Ok(receipt)
    }
}

impl<C: ActuatorChain, K: Clock> ActuatorChain for Governor<C, K> {
    type Error = GovernorError<C::Error>;
    type Feedback = C::Feedback;

    fn axes(&self) -> &[BoundAxis] {
        self.chain.axes()
    }

    fn commandable(&self) -> &[CommandRange] {
        self.chain.commandable()
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt<Self::Feedback>, Self::Error> {
        self.drive(joints, None)
    }

    /// The slewed positions with each axis' feed-forward, its planned velocity held to the
    /// axis's `max_speed`.
    fn command_with(
        &mut self,
        joints: &[f64],
        feedforward: &[Feedforward],
    ) -> Result<Receipt<Self::Feedback>, Self::Error> {
        self.drive(joints, Some(feedforward))
    }

    /// The bounds given, each derated axis's torque bound no more than its derate's cap.
    fn set_feedforward_limits(&mut self, limits: &[FeedforwardLimit]) -> Result<(), Self::Error> {
        let bounded: Vec<FeedforwardLimit> = limits
            .iter()
            .enumerate()
            .map(
                |(index, limit)| match self.torque_caps.get(index).copied().flatten() {
                    Some(cap) => FeedforwardLimit {
                        // Off stays off; a bound is held to the cap.
                        torque: limit.torque.map(|torque| torque.min(cap)),
                        ..*limit
                    },
                    None => *limit,
                },
            )
            .collect();
        self.chain
            .set_feedforward_limits(&bounded)
            .map_err(GovernorError::Chain)
    }

    fn stop(&mut self) -> StopReport {
        self.held = None;
        self.interval = None;
        self.outside.fill(None);
        self.noted = None;
        self.missed.fill(0);
        self.chain.stop()
    }

    fn stop_budget(&self) -> Duration {
        self.chain.stop_budget()
    }

    /// A read moves nothing, so it passes through even while a fault is latched.
    fn read_positions(&mut self) -> Axes<AxisReading> {
        self.chain.read_positions()
    }

    fn last_observed(&self) -> Option<Axes<f64>> {
        self.chain.last_observed()
    }

    fn command_readings(&self) -> Option<Axes<AxisReading>> {
        self.chain.command_readings()
    }

    fn unavailable(&self) -> Vec<usize> {
        self.unavailable.clone()
    }

    fn derated(&self) -> Vec<AxisDerate> {
        self.derated.clone()
    }

    fn bus_timing(&self) -> Vec<crate::BusTiming> {
        self.chain.bus_timing()
    }

    fn retried(&self) -> u64 {
        self.chain.retried()
    }

    fn clear_faults(&mut self) {
        self.latched = false;
        self.missed.fill(0);
        self.held = None;
        self.interval = None;
        self.outside.fill(None);
        self.noted = None;
        self.chain.clear_faults();
    }
}

#[cfg(test)]
mod tests {
    use std::cell::Cell;
    use std::rc::Rc;

    use super::*;
    use crate::binding::{ActuatorBinding, ActuatorBus, ActuatorModel};
    use crate::chain::check_reach;

    #[derive(Debug, Error)]
    #[error("fake chain failure: {0:?}")]
    struct FakeError(ChainFailure);

    impl ChainError for FakeError {
        fn failure(&self) -> ChainFailure {
            self.0
        }
    }

    /// Two axes on -1..1 rad, commandable -0.9..0.9; observes the target plus `lag` when
    /// `observes`, resting at `rest` until commanded and again once stopped. It refuses a target
    /// outside its ranges widened to where it observes the axis, as every chain does.
    struct Fake {
        axes: Vec<BoundAxis>,
        commandable: Vec<CommandRange>,
        observes: bool,
        rest: Vec<f64>,
        lag: f64,
        sent: Vec<Vec<f64>>,
        stops: usize,
        fail: Option<ChainFailure>,
        /// Observe only the first axis.
        short: bool,
        /// An axis whose answer misses every command while set: observed as NaN.
        silent: Option<usize>,
        /// Where the last command's answer observed each axis; `None` once stopped.
        at: Option<Vec<f64>>,
        /// What the chain reports derated.
        derated: Vec<AxisDerate>,
        /// Each `command_with`'s feed-forward, and each feed-forward bound set.
        feedforward: Vec<Vec<Feedforward>>,
        feedforward_limits: Vec<Vec<FeedforwardLimit>>,
        /// An axis that never moves: it observes this whatever it is sent.
        pinned: Option<(usize, f64)>,
    }

    fn axis(joint: &str, bus_id: u16) -> BoundAxis {
        BoundAxis {
            joint: joint.to_owned(),
            port: "can".to_owned(),
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
            home: None,
        }
    }

    fn fake(observes: bool) -> Fake {
        Fake {
            axes: vec![axis("a", 1), axis("b", 2)],
            commandable: vec![
                CommandRange {
                    lower: -0.9,
                    upper: 0.9
                };
                2
            ],
            observes,
            rest: vec![0.0, 0.5],
            lag: 0.0,
            sent: Vec::new(),
            stops: 0,
            fail: None,
            short: false,
            silent: None,
            at: None,
            derated: Vec::new(),
            feedforward: Vec::new(),
            feedforward_limits: Vec::new(),
            pinned: None,
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
            let now = self.last_observed();
            assert!(
                check_reach(&self.axes, &self.commandable, now.as_deref(), joints).is_ok(),
                "the governor sent {joints:?} outside the commandable ranges widened to {now:?}"
            );
            if let Some(failure) = self.fail {
                return Err(FakeError(failure));
            }
            self.sent.push(joints.to_vec());
            let mut observed: Vec<f64> = joints.iter().map(|joint| joint + self.lag).collect();
            if let Some((index, at)) = self.pinned {
                observed[index] = at;
            }
            self.at = Some(observed.clone());
            if let Some(index) = self.silent {
                observed[index] = f64::NAN;
            }
            Ok(Receipt {
                evidence: crate::chain::Evidence::BusObservedComplete,
                observed: if self.observes {
                    observed
                        .into_iter()
                        .take(if self.short { 1 } else { usize::MAX })
                        .collect()
                } else {
                    Axes::new()
                },
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

        fn set_feedforward_limits(&mut self, limits: &[FeedforwardLimit]) -> Result<(), FakeError> {
            self.feedforward_limits.push(limits.to_vec());
            Ok(())
        }

        fn derated(&self) -> Vec<AxisDerate> {
            self.derated.clone()
        }

        fn stop(&mut self) -> StopReport {
            self.stops += 1;
            self.at = None;
            StopReport::default()
        }

        fn stop_budget(&self) -> Duration {
            Duration::from_millis(5)
        }

        fn last_observed(&self) -> Option<Axes<f64>> {
            self.observes
                .then(|| Axes::from_slice(self.at.as_ref().unwrap_or(&self.rest)))
        }

        fn read_positions(&mut self) -> Axes<AxisReading> {
            Axes::filled(
                AxisReading::Missing(crate::chain::MissingReason::Timeout),
                self.axes.len(),
            )
        }
    }

    /// A clock the test moves.
    #[derive(Clone)]
    struct Manual(Rc<Cell<Instant>>);

    impl Manual {
        fn advance(&self, by: Duration) {
            self.0.set(self.0.get() + by);
        }
    }

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

    fn governed(chain: Fake) -> (Governor<Fake, Manual>, Manual) {
        let clock = Manual(Rc::new(Cell::new(Instant::now())));
        let governor = Governor::with_clock(chain, vec![LIMITS; 2], clock.clone()).expect("limits");
        (governor, clock)
    }

    fn close(left: &[f64], right: &[f64]) -> bool {
        left.iter().zip(right).all(|(a, b)| (a - b).abs() < 1e-12)
    }

    /// Refute pass on #123: a chosen derate's NaN speed lifted the over-rail derate, and a
    /// zero, negative or NaN one stalled or panicked the step. Each is refused at build.
    #[test]
    fn a_chosen_derate_that_is_not_a_speed_and_acceleration_is_refused() {
        let over = crate::rail::OVER_RAIL;
        for (max_speed, max_acceleration) in [
            (f64::NAN, 3.0),
            (0.0, 3.0),
            (-0.6, 3.0),
            (0.6, f64::NAN),
            (0.6, -1.0),
            (f64::INFINITY, 3.0),
        ] {
            let derate = crate::rail::DerateLimits {
                max_speed,
                max_acceleration,
                ..over
            };
            assert!(matches!(
                Governor::with_derate(fake(true), vec![LIMITS; 2], Some(derate)),
                Err(LimitsError::Derate)
            ));
        }
        assert!(Governor::with_derate(fake(true), vec![LIMITS; 2], Some(over)).is_ok());
    }

    #[test]
    fn commands_slew_from_where_the_body_rests_at_the_speed_bound() {
        let (mut governor, clock) = governed(fake(true));
        governor.command(&[0.9, -0.9]).expect("first");
        // One capped interval from rest (0, 0.5): 0.25 rad/s for 20 ms.
        assert!(close(&governor.chain().sent[0], &[0.005, 0.495]));
        clock.advance(Duration::from_millis(10));
        governor.command(&[0.9, -0.9]).expect("second");
        assert!(close(&governor.chain().sent[1], &[0.0075, 0.4925]));
        // A target closer than one step is reached exactly.
        clock.advance(Duration::from_millis(10));
        governor.command(&[0.008, 0.4925]).expect("third");
        assert!(close(&governor.chain().sent[2], &[0.008, 0.4925]));
    }

    #[test]
    fn one_late_answer_is_tolerated_and_two_in_a_row_stop_the_chain() {
        let (mut governor, clock) = governed(fake(true));
        governor.command(&[0.0, 0.5]).expect("hold");
        governor.chain_mut().silent = Some(1);
        clock.advance(Duration::from_millis(20));
        governor
            .command(&[0.0, 0.5])
            .expect("one late answer is data");
        governor.chain_mut().silent = None;
        clock.advance(Duration::from_millis(20));
        governor
            .command(&[0.0, 0.5])
            .expect("answered again: the count restarts");
        governor.chain_mut().silent = Some(1);
        clock.advance(Duration::from_millis(20));
        governor
            .command(&[0.0, 0.5])
            .expect("a single late answer again");
        clock.advance(Duration::from_millis(20));
        assert!(matches!(
            governor.command(&[0.0, 0.5]),
            Err(GovernorError::Unanswered { ref joint, ticks: 2, .. }) if joint == "b"
        ));
        assert_eq!(governor.chain().stops, 1);
        assert!(matches!(
            governor.command(&[0.0, 0.5]),
            Err(GovernorError::Latched)
        ));
    }

    #[test]
    fn a_stalled_caller_earns_no_bigger_step() {
        let (mut governor, clock) = governed(fake(true));
        governor.command(&[0.0, 0.5]).expect("hold");
        clock.advance(Duration::from_secs(5));
        governor.command(&[0.9, 0.5]).expect("after a stall");
        assert!(close(&governor.chain().sent[1], &[0.005, 0.5]));
    }

    #[test]
    fn steps_stay_inside_the_commandable_ranges() {
        let mut chain = fake(true);
        // Resting just past the commandable edge, as a joint against its stop can read: it
        // comes in at the speed bound, never jumped to the edge.
        chain.rest = vec![0.95, 0.0];
        let (mut governor, _) = governed(chain);
        governor.command(&[0.9, 0.0]).expect("from past the edge");
        assert!(close(&governor.chain().sent[0], &[0.945, 0.0]));
        // Asked past every bound: brought in to the edge of where it may go, not refused.
        governor.command(&[1.5, 0.0]).expect("brought in");
        assert!(governor.chain().sent[1][0] <= 0.95 + 1e-12);
    }

    #[test]
    fn a_body_that_does_not_follow_is_stopped_and_the_fault_latches() {
        let mut chain = fake(true);
        chain.lag = 0.3;
        let (mut governor, _) = governed(chain);
        match governor.command(&[0.1, 0.5]) {
            Err(GovernorError::Tracking { joint, limit, .. }) => {
                assert_eq!(joint, "a");
                assert!((limit - 0.2).abs() < 1e-12);
            }
            other => panic!("expected a tracking fault, got {other:?}"),
        }
        assert_eq!(governor.chain().stops, 1);
        assert!(matches!(
            governor.command(&[0.0, 0.5]),
            Err(GovernorError::Latched)
        ));
        assert_eq!(governor.chain().sent.len(), 1, "nothing sent while latched");
        governor.chain_mut().lag = 0.0;
        governor.clear_faults();
        governor.command(&[0.0, 0.5]).expect("after clearing");
    }

    #[test]
    fn an_axis_never_observed_stops_on_its_second_command() {
        let mut chain = fake(true);
        chain.lag = f64::NAN;
        let (mut governor, _) = governed(chain);
        governor.command(&[0.0, 0.5]).expect("one unobserved tick");
        assert!(matches!(
            governor.command(&[0.0, 0.5]),
            Err(GovernorError::Unanswered { ticks: 2, .. })
        ));
    }

    #[test]
    fn a_chain_that_observes_nothing_cannot_be_governed() {
        let (mut governor, _) = governed(fake(false));
        assert!(matches!(
            governor.command(&[0.0, 0.0]),
            Err(GovernorError::NoStart)
        ));
        assert!(governor.chain().sent.is_empty());
        assert!(governor.start_at(&[0.0]).is_err());
        governor.start_at(&[0.2, -0.2]).expect("start");
        // One step from the given start goes out; its receipt observes nothing, so tracking
        // cannot be checked: the chain is stopped and the fault latched.
        assert!(matches!(
            governor.command(&[0.9, -0.9]),
            Err(GovernorError::Unobserved {
                expected: 2,
                found: 0,
                ..
            })
        ));
        assert!(close(&governor.chain().sent[0], &[0.205, -0.205]));
        assert_eq!(governor.chain().stops, 1);
        assert!(matches!(
            governor.command(&[0.0, 0.0]),
            Err(GovernorError::Latched)
        ));
    }

    #[test]
    fn a_short_observation_stops_and_latches() {
        let mut chain = fake(true);
        chain.short = true;
        let (mut governor, _) = governed(chain);
        assert!(matches!(
            governor.command(&[0.0, 0.5]),
            Err(GovernorError::Unobserved {
                expected: 2,
                found: 1,
                ..
            })
        ));
        assert_eq!(governor.chain().stops, 1);
        governor.chain_mut().short = false;
        assert!(matches!(
            governor.command(&[0.0, 0.5]),
            Err(GovernorError::Latched)
        ));
        governor.clear_faults();
        governor.command(&[0.0, 0.5]).expect("after clearing");
    }

    #[test]
    fn a_chain_that_failed_mid_command_is_stopped_and_latched() {
        for failure in [
            ChainFailure::Fault,
            ChainFailure::Timeout,
            ChainFailure::Unavailable,
        ] {
            let mut chain = fake(true);
            chain.fail = Some(failure);
            let (mut governor, _) = governed(chain);
            match governor.command(&[0.0, 0.5]) {
                Err(error @ GovernorError::Faulted { .. }) => assert_eq!(error.failure(), failure),
                other => panic!("expected a latched fault, got {other:?}"),
            }
            assert_eq!(governor.chain().stops, 1, "{failure:?}");
            governor.chain_mut().fail = None;
            assert!(matches!(
                governor.command(&[0.0, 0.5]),
                Err(GovernorError::Latched)
            ));
        }
        // A refusal is not latched.
        let mut chain = fake(true);
        chain.fail = Some(ChainFailure::Refused);
        let (mut governor, _) = governed(chain);
        assert!(matches!(
            governor.command(&[0.0, 0.5]),
            Err(GovernorError::Chain(_))
        ));
        assert_eq!(governor.chain().stops, 0);
        governor.chain_mut().fail = None;
        governor.command(&[0.0, 0.5]).expect("not latched");
    }

    #[test]
    fn after_a_stop_or_a_failure_the_next_command_starts_from_the_rest_again() {
        let (mut governor, _) = governed(fake(true));
        governor.command(&[0.9, 0.5]).expect("first");
        governor.stop();
        // The body sagged while disabled.
        governor.chain_mut().rest = vec![-0.3, 0.5];
        governor.command(&[0.9, 0.5]).expect("after the stop");
        assert!(close(&governor.chain().sent[1], &[-0.295, 0.5]));
        // A refusal changed nothing: returned as it is, and the next command starts again.
        governor.chain_mut().fail = Some(ChainFailure::Refused);
        assert!(matches!(
            governor.command(&[0.9, 0.5]),
            Err(GovernorError::Chain(_))
        ));
        governor.chain_mut().fail = None;
        governor.chain_mut().at = None;
        governor.chain_mut().rest = vec![0.1, 0.5];
        governor.command(&[0.9, 0.5]).expect("after the failure");
        assert!(close(&governor.chain().sent[2], &[0.105, 0.5]));
    }

    #[test]
    fn limits_are_one_finite_positive_pair_per_axis() {
        assert!(matches!(
            Governor::new(fake(true), vec![LIMITS]),
            Err(LimitsError::Width { .. })
        ));
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let limits = vec![
                LIMITS,
                AxisLimits {
                    max_speed: bad,
                    max_tracking_error: 0.2,
                    max_acceleration: None,
                },
            ];
            assert!(matches!(
                Governor::new(fake(true), limits),
                Err(LimitsError::NotPositive(joint)) if joint == "b"
            ));
        }
    }

    /// A governor over axis "a" resting at `rest` (range -0.9..0.9, declared -1..1) and "b" at
    /// 0.5, one 20 ms step a command.
    fn resting(rest: f64) -> (Governor<Fake, Manual>, Manual) {
        let mut chain = fake(true);
        chain.rest = vec![rest, 0.5];
        governed(chain)
    }

    /// Command `target` once, one 20 ms period after the last, and return what was sent.
    fn step(governor: &mut Governor<Fake, Manual>, clock: &Manual, target: &[f64]) -> Vec<f64> {
        clock.advance(MAX_STEP_INTERVAL);
        governor
            .command(target)
            .unwrap_or_else(|error| panic!("{error}"));
        governor.chain().sent.last().expect("sent").clone()
    }

    #[test]
    fn a_joint_resting_outside_its_range_holds_where_it_is() {
        // Past the range's lower edge, inside the declared bounds; asked further out.
        let (mut governor, clock) = resting(-0.95);
        for _ in 0..10 {
            let sent = step(&mut governor, &clock, &[-1.0, 0.5]);
            assert!(
                close(&sent, &[-0.95, 0.5]),
                "held, not stepped out: {sent:?}"
            );
        }
        assert_eq!(governor.chain().stops, 0, "nothing refused or latched");
        // The edge is never jumped to.
        assert!(governor.chain().sent.iter().all(|sent| sent[0] < -0.9));
    }

    #[test]
    fn a_joint_outside_moves_inward_at_the_speed_bound_and_never_outward() {
        // 1.5 rad below its range and past its declared bounds, as an aliased joint reads.
        let (mut governor, clock) = resting(-2.4);
        let mut last = -2.4;
        for round in 0..20 {
            // Every declared target lies inward of it: a step goes in at most 0.25 rad/s for
            // 20 ms, whatever is asked.
            let target = if round % 2 == 0 {
                [0.5, 0.5]
            } else {
                [-1.0, 0.5]
            };
            let sent = step(&mut governor, &clock, &target)[0];
            assert!(
                sent >= last - 1e-12,
                "round {round}: moved out to {sent} from {last}"
            );
            assert!(
                sent - last <= 0.005 + 1e-12,
                "round {round}: {sent} from {last}"
            );
            last = sent;
        }
        assert!((last + 2.3).abs() < 1e-9, "{last}");
    }

    #[test]
    fn the_interval_shrinks_as_the_joint_comes_in() {
        let (mut governor, clock) = resting(-0.95);
        // In: 0.005 rad a step, past the edge at -0.9 on the eleventh.
        let mut sent = Vec::new();
        for _ in 0..12 {
            sent = step(&mut governor, &clock, &[0.0, 0.5]);
        }
        assert!((sent[0] + 0.89).abs() < 1e-9, "{sent:?}");
        // Asked back out, it stops at the range's edge: where it was is no longer reachable.
        for _ in 0..10 {
            sent = step(&mut governor, &clock, &[-1.0, 0.5]);
            assert!(sent[0] >= -0.9 - 1e-12, "{sent:?}");
        }
        assert!((sent[0] + 0.9).abs() < 1e-12);
    }

    /// Bench 2026-10-26: an arm resting just past its limit drifted further out while idle,
    /// the station commanded where it then read it, and the governor, checking against its
    /// older observation, refused and the run safe-stopped. A target further out than the
    /// joint is, or past every bound, is brought in to the interval's edge, never refused; a
    /// target that is not a number still is.
    #[test]
    fn a_target_further_out_than_the_joint_is_brought_in_not_refused() {
        let (mut governor, clock) = resting(1.2);
        for target in [1.2 + 0.0119, 5.0, 1.2] {
            let sent = step(&mut governor, &clock, &[target, 0.5]);
            assert!(sent[0] <= 1.2 + 1e-12, "{target} sent {sent:?}");
        }
        assert_eq!(governor.chain().stops, 0, "nothing refused or latched");
        clock.advance(MAX_STEP_INTERVAL);
        assert!(matches!(
            governor.command(&[f64::NAN, 0.5]),
            Err(GovernorError::Target(TargetError::OutOfBounds { .. }))
        ));
        assert!(matches!(
            governor.command(&[0.0]),
            Err(GovernorError::Target(TargetError::Width { .. }))
        ));
    }

    /// Refute pass on 9e58e58: with the interval widened to wherever the joint was last read,
    /// a joint that settles a little outward of each command (gravity, a hand) was followed out
    /// past every bound. The interval only narrows once the governor has the joint, so the
    /// command stays at its edge whatever the joint does.
    #[test]
    fn a_joint_pushed_outward_is_not_followed_out() {
        for rest in [0.85, 1.2] {
            let mut chain = fake(true);
            chain.rest = vec![rest, 0.5];
            chain.lag = 0.004;
            let (mut governor, clock) = governed(chain);
            let edge = rest.max(0.9);
            for _ in 0..200 {
                let sent = step(&mut governor, &clock, &[5.0, 0.5]);
                assert!(sent[0] <= edge + 1e-12, "rest {rest}: sent {sent:?}");
            }
            assert_eq!(governor.brought_in(), Some(0));
            assert_eq!(governor.chain().stops, 0);
        }
    }

    /// Refute correction pass: a refused command or a malformed feed-forward re-seeded the
    /// interval from the sagged reading, so alternating one with a good command walked the
    /// joint out again. The interval survives both; only a stop or a fault takes it up anew.
    #[test]
    fn a_refused_or_malformed_command_does_not_widen_the_interval() {
        for refuse in [true, false] {
            let mut chain = fake(true);
            chain.rest = vec![0.85, 0.5];
            chain.lag = 0.004;
            let (mut governor, clock) = governed(chain);
            for _ in 0..200 {
                clock.advance(MAX_STEP_INTERVAL);
                if refuse {
                    governor.chain_mut().fail = Some(ChainFailure::Refused);
                    assert!(governor.command(&[5.0, 0.5]).is_err());
                    governor.chain_mut().fail = None;
                } else {
                    assert!(governor.command_with(&[5.0, 0.5], &[]).is_err());
                }
                let sent = step(&mut governor, &clock, &[5.0, 0.5]);
                assert!(sent[0] <= 0.9 + 1e-12, "refuse {refuse}: sent {sent:?}");
            }
        }
    }

    #[test]
    fn a_target_past_the_edge_is_braked_onto_it_not_hit_at_speed() {
        let limits = AxisLimits {
            max_acceleration: Some(1.5),
            ..LIMITS
        };
        let clock = Manual(Rc::new(Cell::new(Instant::now())));
        let mut governor =
            Governor::with_clock(fake(true), vec![limits; 2], clock.clone()).expect("limits");
        let dt = MAX_STEP_INTERVAL.as_secs_f64();
        let mut position = 0.0_f64;
        let mut speeds = Vec::new();
        for _ in 0..400 {
            governor.command(&[5.0, 0.5]).expect("step");
            clock.advance(MAX_STEP_INTERVAL);
            let sent = governor.chain().sent.last().expect("sent")[0];
            speeds.push((sent - position) / dt);
            position = sent;
        }
        assert!((position - 0.9).abs() < 1e-9, "{position}");
        // It reached the edge decelerating: the step before arriving was slower than the cruise.
        let arrived = speeds.iter().position(|v| *v < 1e-9).expect("arrived");
        let cruise = speeds.iter().copied().fold(0.0, f64::max);
        assert!(
            speeds[arrived - 1] < 0.5 * cruise,
            "{:?}",
            &speeds[..arrived]
        );
    }

    #[test]
    fn in_range_and_out_of_range_axes_move_together_each_by_its_own_interval() {
        // "a" rests far below its range; "b" inside it.
        let (mut governor, clock) = resting(-2.4);
        let sent = step(&mut governor, &clock, &[0.9, 0.9]);
        assert!(close(&sent, &[-2.395, 0.505]), "{sent:?}");
        // "b" stays inside its range; "a" keeps coming in.
        let mut sent = sent;
        for _ in 0..200 {
            sent = step(&mut governor, &clock, &[0.9, 1.0]);
            assert!(sent[1] <= 0.9 + 1e-12);
        }
        assert!((sent[1] - 0.9).abs() < 1e-12);
        assert!((sent[0] + 1.395).abs() < 1e-9, "{sent:?}");
    }

    #[test]
    fn a_joint_outside_that_does_not_follow_is_still_a_tracking_fault() {
        let mut chain = fake(true);
        chain.rest = vec![-2.4, 0.5];
        chain.lag = -0.3;
        let (mut governor, clock) = governed(chain);
        clock.advance(MAX_STEP_INTERVAL);
        assert!(matches!(
            governor.command(&[0.0, 0.5]),
            Err(GovernorError::Tracking { joint, .. }) if joint == "a"
        ));
        assert_eq!(governor.chain().stops, 1);
        assert!(matches!(
            governor.command(&[0.0, 0.5]),
            Err(GovernorError::Latched)
        ));
    }

    #[test]
    fn acceleration_ramps_the_speed_up_and_brakes_it_down() {
        const ACCELERATION: f64 = 1.5;
        let limits = AxisLimits {
            max_acceleration: Some(ACCELERATION),
            ..LIMITS
        };
        let clock = Manual(Rc::new(Cell::new(Instant::now())));
        let mut governor =
            Governor::with_clock(fake(true), vec![limits; 2], clock.clone()).expect("limits");
        let dt = MAX_STEP_INTERVAL.as_secs_f64();
        let (mut position, mut speed) = (0.0_f64, 0.0_f64);
        let mut fastest = 0.0_f64;
        for _ in 0..200 {
            governor.command(&[0.2, 0.5]).expect("step");
            clock.advance(MAX_STEP_INTERVAL);
            let sent = governor.chain().sent.last().expect("sent")[0];
            let step_speed = (sent - position) / dt;
            assert!(
                (step_speed - speed).abs() <= ACCELERATION * dt + 1e-9,
                "the speed jumped from {speed} to {step_speed}"
            );
            assert!(step_speed <= LIMITS.max_speed + 1e-12);
            fastest = fastest.max(step_speed);
            (position, speed) = (sent, step_speed);
        }
        // The first step starts from rest: 1.5 rad/s² for 20 ms is 0.03 rad/s.
        assert!(close(&governor.chain().sent[0][..1], &[0.03 * dt]));
        assert!(
            (fastest - LIMITS.max_speed).abs() < 1e-9,
            "it cruised at {fastest}"
        );
        assert!(
            (position - 0.2).abs() < 1e-6,
            "it braked to {position}, not 0.2"
        );
        assert!(speed.abs() < 1e-6, "it ends at rest, not {speed}");
        // A stop restarts from rest, where the fake rests once stopped (0).
        governor.stop();
        governor.command(&[-0.2, 0.5]).expect("after a stop");
        let sent = governor.chain().sent.last().expect("sent")[0];
        assert!((sent + 0.03 * dt).abs() < 1e-9);
    }

    fn rated(mut chain: Fake) -> Fake {
        chain.axes[0].binding.model = ActuatorModel::new("damiao_dm8009p");
        chain.axes[1].binding.model = ActuatorModel::new("damiao_dm4340");
        chain
    }

    #[test]
    fn a_derated_axis_is_held_to_the_derate_and_reported() {
        let mut chain = rated(fake(true));
        let derate = AxisDerate {
            axis: 0,
            over: crate::rail::OverRail {
                rail_volts: 54.0,
                max_volts: 48.0,
            },
            limits: crate::rail::OVER_RAIL,
        };
        chain.derated = vec![derate];
        let fast = AxisLimits {
            max_speed: 1.0,
            ..LIMITS
        };
        let mut governor = Governor::new(chain, vec![fast; 2]).expect("limits");
        assert_eq!(governor.derated(), vec![derate]);
        assert_eq!(
            derate.to_string(),
            "derated: over-rail 54 V > 48 V, held to 0.30 rad/s and 1.5 rad/s², feed-forward at most 50 % of peak"
        );
        let held = governor.limits()[0];
        assert!((held.max_speed - 0.3).abs() < 1e-12);
        assert_eq!(held.max_acceleration, Some(1.5));
        assert_eq!(
            governor.limits()[1],
            fast,
            "the axis in range keeps its limits"
        );
        // Feed-forward torque: at most half the J8009P's 40 N·m peak; the DM4340 as asked.
        governor
            .set_feedforward_limits(&[
                FeedforwardLimit {
                    velocity: None,
                    torque: Some(30.0),
                },
                FeedforwardLimit {
                    velocity: None,
                    torque: Some(30.0),
                },
            ])
            .expect("bounds");
        governor
            .set_feedforward_limits(&[FeedforwardLimit::default(); 2])
            .expect("off");
        let bounds = &governor.chain().feedforward_limits;
        assert_eq!(bounds[0][0].torque, Some(20.0));
        assert_eq!(bounds[0][1].torque, Some(30.0));
        assert_eq!(
            bounds[1][0].torque, None,
            "feed-forward switched off stays off"
        );
        let asked = Feedforward {
            velocity: 0.0,
            torque: -30.0,
        };
        governor
            .command_with(&[0.0, 0.5], &[asked; 2])
            .expect("command");
        let sent = &governor.chain().feedforward[0];
        assert!((sent[0].torque + 20.0).abs() < 1e-12);
        assert!((sent[1].torque + 30.0).abs() < 1e-12);
    }

    #[test]
    fn a_chain_with_no_derate_keeps_every_limit() {
        let governor = Governor::new(rated(fake(true)), vec![LIMITS; 2]).expect("limits");
        assert!(governor.derated().is_empty());
        assert_eq!(governor.limits(), &[LIMITS; 2]);
    }

    /// A joint resting past its range that its servo cannot lift (a gravity stop, a weak
    /// servo) while the caller asks it inward: after [`OUTSIDE_STALL`] without progress it is
    /// held where it rests, not driven against its stop, and nothing else stops.
    #[test]
    fn a_joint_outside_that_cannot_come_in_is_held_where_it_rests() {
        let mut chain = fake(true);
        chain.rest = vec![-0.993, 0.5];
        chain.pinned = Some((0, -0.993));
        let (mut governor, clock) = governed(chain);
        for _ in 0..500 {
            clock.advance(MAX_STEP_INTERVAL);
            governor
                .command(&[-0.9, 0.6])
                .expect("a stuck joint is not a fault");
        }
        let sent = governor.chain().sent.last().expect("sent").clone();
        assert!((sent[0] + 0.993).abs() < 1e-9, "still pushed: {}", sent[0]);
        assert!(
            (sent[1] - 0.6).abs() < 1e-9,
            "the other joint follows: {}",
            sent[1]
        );
        assert_eq!(governor.held_outside(), Some(0));
    }

    /// A joint stalled just past its edge (10-28, the Spark's right forearm yaw: 0.021 rad out,
    /// friction holding it against the edge's small error) is given up on while the caller asks
    /// only for the edge, approached again once the caller asks it further in, and never sent
    /// further than half its tracking bound from where it is meanwhile.
    #[test]
    fn a_joint_given_up_on_is_approached_again_when_asked_further_in() {
        let mut chain = fake(true);
        chain.rest = vec![-0.921, 0.5];
        chain.pinned = Some((0, -0.921));
        let (mut governor, clock) = governed(chain);
        for _ in 0..200 {
            clock.advance(MAX_STEP_INTERVAL);
            governor.command(&[-0.9, 0.5]).expect("the edge");
        }
        assert_eq!(governor.held_outside(), Some(0), "stalled at the edge");
        // Asked further in: approached again, within half its tracking bound of where it is.
        clock.advance(MAX_STEP_INTERVAL);
        governor.command(&[-0.6, 0.5]).expect("further in");
        assert_eq!(governor.held_outside(), None, "approached again");
        for _ in 0..20 {
            clock.advance(MAX_STEP_INTERVAL);
            governor.command(&[-0.6, 0.5]).expect("pushed");
            let sent = governor.chain().sent.last().expect("sent")[0];
            assert!(
                (sent + 0.921).abs() <= LIMITS.max_tracking_error / 2.0 + 1e-9,
                "sent {sent} too far from where it rests"
            );
        }
        // The stall breaks: it comes in and follows.
        governor.chain_mut().pinned = None;
        for _ in 0..500 {
            clock.advance(MAX_STEP_INTERVAL);
            governor.command(&[-0.6, 0.5]).expect("comes in");
        }
        let sent = governor.chain().sent.last().expect("sent")[0];
        assert!((sent + 0.6).abs() < 1e-9, "reached its target: {sent}");
        assert_eq!(governor.held_outside(), None);
    }

    /// A joint pushed further out after take-up is not followed out by the lead toward where it
    /// is: the command stays at the interval's edge and the tracking check catches it (refute
    /// of the re-approach).
    #[test]
    fn a_joint_pushed_far_out_is_not_led_out_and_faults() {
        let mut chain = fake(true);
        chain.rest = vec![0.5, 0.5];
        let (mut governor, clock) = governed(chain);
        clock.advance(MAX_STEP_INTERVAL);
        governor.command(&[0.5, 0.5]).expect("taken up inside");
        governor.chain_mut().pinned = Some((0, 1.2));
        let mut faulted = false;
        for _ in 0..20 {
            clock.advance(MAX_STEP_INTERVAL);
            match governor.command(&[0.5, 0.5]) {
                Ok(_) => {
                    let sent = governor.chain().sent.last().expect("sent")[0];
                    assert!(sent <= 0.9 + 1e-9, "led out to {sent}");
                }
                Err(GovernorError::Tracking { .. }) => {
                    faulted = true;
                    break;
                }
                Err(other) => panic!("{other:?}"),
            }
        }
        assert!(faulted, "a joint pushed 0.3 rad out is a tracking fault");
    }

    /// One coming in, however slowly, is never given up on; once inside it is a normal joint.
    #[test]
    fn a_joint_outside_that_comes_in_is_followed_and_never_given_up_on() {
        let mut chain = fake(true);
        chain.rest = vec![-1.05, 0.5];
        let (mut governor, clock) = governed(chain);
        for _ in 0..500 {
            clock.advance(MAX_STEP_INTERVAL);
            governor.command(&[-0.5, 0.5]).expect("comes in");
            assert_eq!(governor.held_outside(), None);
        }
        let sent = governor.chain().sent.last().expect("sent")[0];
        assert!((sent + 0.5).abs() < 1e-9, "reached its target: {sent}");
    }

    // ---- refute pass: attempts to break the outside-progress hold ----

    /// A joint outside its range that has been idle (the caller sent nothing) for longer than
    /// [`OUTSIDE_STALL`] is given up on at its first tick back, though nothing was commanded.
    #[test]
    fn a_pause_in_commands_is_not_counted_as_a_stall() {
        let mut chain = fake(true);
        chain.rest = vec![-0.993, 0.5];
        let (mut governor, clock) = governed(chain);
        clock.advance(MAX_STEP_INTERVAL);
        governor.command(&[-0.5, 0.5]).expect("first");
        clock.advance(Duration::from_secs(3));
        governor.command(&[-0.5, 0.5]).expect("after idle");
        assert_eq!(governor.held_outside(), None, "given up after an idle gap");
    }

    /// A joint that settles a few mrad past its edge while the caller holds the edge (steady
    /// state error) is given up on, then ignores every later inward command for good.
    #[test]
    fn a_joint_sagging_just_past_the_edge_it_holds_is_never_given_up_on() {
        let mut chain = fake(true);
        chain.rest = vec![0.0, 0.5];
        chain.lag = -0.003; // observes 3 mrad short of what it is sent
        let (mut governor, clock) = governed(chain);
        for _ in 0..350 {
            clock.advance(MAX_STEP_INTERVAL);
            governor.command(&[-0.9, 0.5]).expect("hold the edge");
        }
        for _ in 0..500 {
            clock.advance(MAX_STEP_INTERVAL);
            governor.command(&[0.0, 0.5]).expect("inward");
        }
        let sent = governor.chain().sent.last().expect("sent")[0];
        assert!(sent > -0.5, "joint frozen near the edge: {sent}");
    }

    /// Given up: the feed-forward torque and velocity the caller planned still go out with it.
    #[test]
    fn a_joint_given_up_on_is_sent_no_feed_forward() {
        let mut chain = fake(true);
        chain.rest = vec![-0.993, 0.5];
        chain.pinned = Some((0, -0.993));
        let (mut governor, clock) = governed(chain);
        let ff = [Feedforward {
            velocity: 0.2,
            torque: 1.0,
        }; 2];
        for _ in 0..500 {
            clock.advance(MAX_STEP_INTERVAL);
            governor.command_with(&[-0.9, 0.5], &ff).expect("ok");
        }
        assert_eq!(governor.held_outside(), Some(0));
        let last = governor.chain().feedforward.last().expect("ff")[0];
        assert!(
            last.torque == 0.0 && last.velocity == 0.0,
            "still pushing: {last:?}"
        );
    }
}
