//! A speed and tracking-error bound around any actuator chain.
//!
//! A chain drives each axis to whatever target it is given, as fast as its gains pull. The
//! [`Governor`] stands in front of it as the same [`ActuatorChain`], and owns two bounds per axis
//! ([`AxisLimits`]), set by whoever operates the body (the static upper body's received driver
//! ran 0.25 rad/s and 0.2 rad at bring-up):
//!
//! - **Speed.** Each command moves every axis from the position last sent toward its target by
//!   at most `max_speed` times the time since the last accepted command, that time capped at
//!   [`MAX_STEP_INTERVAL`] so a caller that stalls earns no larger step. The result is held
//!   inside the chain's commandable range. The first command after the chain was opened,
//!   stopped or failed starts from where the body rests: a position given with
//!   [`Governor::start_at`], else the chain's own [`ActuatorChain::last_observed`]. With
//!   neither, the command is refused.
//! - **Resting outside the range.** Holding the first step inside the range must not become a
//!   jump to its edge: an axis that rests outside its commandable range by more than its
//!   `max_tracking_error` refuses the command, sends nothing, and latches
//!   [`GovernorError::RestingOutside`] until the operator recovers it — [`Governor::start_at`]
//!   after moving the joint back by hand, or [`ActuatorChain::clear_faults`] once the chain
//!   observes it inside again. Only an axis whose chain declares that the device limits its
//!   approach speed ([`ActuatorChain::approach_speed_limited`]) is instead sent the in-range
//!   position nearest where it rests, held there while the device brings it in, with its
//!   tracking check suspended until it is observed inside; then it slews as any other axis.
//!   The approach is bounded: it must finish within twice its distance at `max_speed` plus a
//!   second (at most a minute), keep getting closer (no regress beyond the tracking bound, no
//!   [`APPROACH_STALL_OBSERVATIONS`] observations in a row without progress), and never be
//!   observed as NaN; otherwise the chain is stopped and the fault latched
//!   ([`GovernorError::ApproachTimeout`], [`GovernorError::ApproachStalled`],
//!   [`GovernorError::Tracking`]).
//! - **Fail closed.** Every receipt must observe every axis: a short or empty observation stops
//!   the chain and latches [`GovernorError::Unobserved`], so a chain that observes nothing
//!   cannot be governed. A chain error other than a refusal (the chain may have applied part
//!   of the command) stops the chain and latches [`GovernorError::Faulted`]; a refusal, which
//!   changed nothing, is returned as it is.
//! - **Tracking error.** When a command's receipt carries observed positions, each must lie
//!   within `max_tracking_error` of what was just sent. One that does not stops the chain and
//!   latches a fault: every later command is refused until [`ActuatorChain::clear_faults`].
//!
//! The governor bounds what the station asks; it is not a substitute for the motors' own
//! protections.

use std::time::{Duration, Instant};

use thiserror::Error;

use crate::binding::BoundAxis;
use crate::chain::{
    ActuatorChain, ChainError, ChainFailure, CommandRange, Receipt, StopReport, TargetError,
    check_targets,
};

/// The longest time one command's speed bound counts: one control period at 50 Hz.
pub const MAX_STEP_INTERVAL: Duration = Duration::from_millis(20);

/// One axis's bounds, in the joint's own unit (rad or m) and per second.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisLimits {
    /// The fastest a command moves the axis, unit/s.
    pub max_speed: f64,
    /// The farthest an observed position may lie from the position just sent, unit.
    pub max_tracking_error: f64,
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
    #[error("joint {0} needs a finite, positive speed and tracking-error bound")]
    NotPositive(String),
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
        stop: StopReport,
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
        "joint {joint} did not reach its range within {allowed_s:.1} s of approach (observed at {observed}); the chain was stopped and the fault is latched"
    )]
    ApproachTimeout {
        joint: String,
        observed: f64,
        allowed_s: f64,
        stop: StopReport,
    },
    #[error(
        "joint {joint} stopped approaching its range (observed at {observed}, {distance} away, its closest {closest}); the chain was stopped and the fault is latched"
    )]
    ApproachStalled {
        joint: String,
        observed: f64,
        distance: f64,
        closest: f64,
        stop: StopReport,
    },
    #[error(
        "joint {joint} rests at {observed}, outside its commandable range {lower}..{upper}; nothing was sent and the fault is latched until the joint is recovered"
    )]
    RestingOutside {
        joint: String,
        observed: f64,
        lower: f64,
        upper: f64,
    },
    #[error("a fault is latched; recover the body and clear it before commanding")]
    Latched,
}

impl<E: ChainError> ChainError for GovernorError<E> {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::Chain(error) | Self::Faulted { error, .. } => error.failure(),
            Self::Target(_) | Self::NoStart | Self::Latched | Self::RestingOutside { .. } => {
                ChainFailure::Refused
            }
            Self::ApproachTimeout { .. } => ChainFailure::Timeout,
            Self::Tracking { .. } | Self::Unobserved { .. } | Self::ApproachStalled { .. } => {
                ChainFailure::Fault
            }
        }
    }
}

/// An approach allows its distance at `max_speed` this many times over...
const APPROACH_MARGIN: f64 = 2.0;
/// ...plus this, for the device to start moving...
const APPROACH_ALLOWANCE: Duration = Duration::from_secs(1);
/// ...and never more than this.
const APPROACH_CAP: Duration = Duration::from_secs(60);
/// Consecutive observations without getting closer that end an approach as stalled.
pub const APPROACH_STALL_OBSERVATIONS: usize = 50;

/// One axis its device is bringing into range: by when, and the closest it has come.
#[derive(Clone, Copy, Debug)]
struct Approach {
    started: Instant,
    deadline: Instant,
    closest: f64,
    stalls: usize,
}

/// The position last sent, and when; and the axes still being brought into range by their
/// device.
struct Held {
    position: Vec<f64>,
    at: Option<Instant>,
    approaching: Vec<Option<Approach>>,
}

/// Why commands are refused until the operator acts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Latch {
    /// The chain was stopped for a fault: tracking, approach, observation or the chain's own.
    Fault,
    RestingOutside,
}

/// How far `value` lies outside `range`; 0 inside.
fn outside(value: f64, range: CommandRange) -> f64 {
    if value < range.lower {
        range.lower - value
    } else if value > range.upper {
        value - range.upper
    } else {
        0.0
    }
}

/// A chain whose commands are slewed and whose tracking is checked; see the module docs.
pub struct Governor<C, K = MonotonicClock> {
    chain: C,
    limits: Vec<AxisLimits>,
    clock: K,
    held: Option<Held>,
    start: Option<Vec<f64>>,
    latched: Option<Latch>,
}

impl<C: ActuatorChain> Governor<C> {
    /// Govern `chain` with one [`AxisLimits`] per axis, in native state order.
    ///
    /// # Errors
    ///
    /// Refuses limits of the wrong width, and a bound that is not finite and positive.
    pub fn new(chain: C, limits: Vec<AxisLimits>) -> Result<Self, LimitsError> {
        Self::with_clock(chain, limits, MonotonicClock)
    }
}

impl<C: ActuatorChain, K: Clock> Governor<C, K> {
    /// [`Governor::new`], measuring time by `clock`.
    ///
    /// # Errors
    ///
    /// As [`Governor::new`].
    pub fn with_clock(chain: C, limits: Vec<AxisLimits>, clock: K) -> Result<Self, LimitsError> {
        let axes = chain.axes();
        if limits.len() != axes.len() {
            return Err(LimitsError::Width {
                expected: axes.len(),
                actual: limits.len(),
            });
        }
        let positive = |value: f64| value.is_finite() && value > 0.0;
        if let Some((axis, _)) = axes
            .iter()
            .zip(&limits)
            .find(|(_, limit)| !(positive(limit.max_speed) && positive(limit.max_tracking_error)))
        {
            return Err(LimitsError::NotPositive(axis.joint.clone()));
        }
        Ok(Self {
            chain,
            limits,
            clock,
            held: None,
            start: None,
            latched: None,
        })
    }

    /// Where the body rests, as the operator states it: the next command after open, a stop,
    /// a failure or a resting-outside fault starts from it, ahead of the chain's own
    /// observation. It is how a chain that observes nothing is started, and how a joint moved
    /// back into range by hand is recovered: it releases a [`GovernorError::RestingOutside`]
    /// latch (not a tracking fault). A start that is itself outside the range is refused at the
    /// next command as any other.
    ///
    /// # Errors
    ///
    /// Refuses a vector of the wrong width or outside the declared bounds.
    pub fn start_at(&mut self, joints: Vec<f64>) -> Result<(), TargetError> {
        check_targets(self.chain.axes(), &joints)?;
        self.start = Some(joints);
        self.held = None;
        if self.latched == Some(Latch::RestingOutside) {
            self.latched = None;
        }
        Ok(())
    }

    /// The governed chain.
    pub const fn chain(&self) -> &C {
        &self.chain
    }

    /// The governed chain, for what only it offers.
    pub const fn chain_mut(&mut self) -> &mut C {
        &mut self.chain
    }

    /// Where the next command starts: the operator's start, else the chain's observation.
    /// An axis resting outside its range beyond its tracking bound refuses and latches, unless
    /// its device limits its approach, which then starts from the nearest in-range position
    /// with an approach deadline.
    fn seed(&mut self, now: Instant) -> Result<Held, GovernorError<C::Error>> {
        let width = self.chain.axes().len();
        let mut position = match self.start.take() {
            Some(start) => start,
            None => self
                .chain
                .last_observed()
                .filter(|observed| observed.len() == width)
                .ok_or(GovernorError::NoStart)?,
        };
        let mut approaching = vec![None; width];
        for (index, ((value, range), limit)) in position
            .iter_mut()
            .zip(self.chain.commandable())
            .zip(&self.limits)
            .enumerate()
        {
            let distance = outside(*value, *range);
            if distance <= limit.max_tracking_error {
                continue;
            }
            if self.chain.approach_speed_limited(index) && value.is_finite() {
                *value = value.clamp(range.lower, range.upper);
                let allowed = (APPROACH_ALLOWANCE.as_secs_f64()
                    + APPROACH_MARGIN * distance / limit.max_speed)
                    .min(APPROACH_CAP.as_secs_f64());
                approaching[index] = Some(Approach {
                    started: now,
                    deadline: now + Duration::from_secs_f64(allowed),
                    closest: distance,
                    stalls: 0,
                });
                continue;
            }
            self.latched = Some(Latch::RestingOutside);
            return Err(GovernorError::RestingOutside {
                joint: self.chain.axes()[index].joint.clone(),
                observed: *value,
                lower: range.lower,
                upper: range.upper,
            });
        }
        Ok(Held {
            position,
            at: None,
            approaching,
        })
    }

    /// Stop the chain and latch a fault, returning the error `make` builds from the stop.
    fn fault(
        &mut self,
        make: impl FnOnce(StopReport) -> GovernorError<C::Error>,
    ) -> GovernorError<C::Error> {
        self.latched = Some(Latch::Fault);
        self.held = None;
        let stop = self.chain.stop();
        make(stop)
    }

    /// Advance each approaching axis by this receipt's observation: done once inside its
    /// range, a fault on a NaN, a passed deadline, moving away or no progress for
    /// [`APPROACH_STALL_OBSERVATIONS`] observations.
    fn check_approaches(
        &mut self,
        approaching: &mut [Option<Approach>],
        observed: &[f64],
        sent: &[f64],
        now: Instant,
    ) -> Result<(), GovernorError<C::Error>> {
        for (index, slot) in approaching.iter_mut().enumerate() {
            let Some(approach) = slot.as_mut() else {
                continue;
            };
            let value = observed[index];
            let range = self.chain.commandable()[index];
            let limit = self.limits[index];
            let joint = self.chain.axes()[index].joint.clone();
            if !value.is_finite() {
                let commanded = sent[index];
                return Err(self.fault(|stop| GovernorError::Tracking {
                    joint,
                    commanded,
                    observed: value,
                    limit: limit.max_tracking_error,
                    stop,
                }));
            }
            let distance = outside(value, range);
            if distance <= limit.max_tracking_error {
                *slot = None;
                continue;
            }
            if now >= approach.deadline {
                let allowed_s = (approach.deadline - approach.started).as_secs_f64();
                return Err(self.fault(|stop| GovernorError::ApproachTimeout {
                    joint,
                    observed: value,
                    allowed_s,
                    stop,
                }));
            }
            let regressed = distance > approach.closest + limit.max_tracking_error;
            if distance < approach.closest {
                approach.closest = distance;
                approach.stalls = 0;
            } else {
                approach.stalls += 1;
            }
            if regressed || approach.stalls >= APPROACH_STALL_OBSERVATIONS {
                let closest = approach.closest;
                return Err(self.fault(|stop| GovernorError::ApproachStalled {
                    joint,
                    observed: value,
                    distance,
                    closest,
                    stop,
                }));
            }
        }
        Ok(())
    }
}

/// Whether `observed` lies within `limit` of `sent`; never for a NaN.
fn within(observed: f64, sent: f64, limit: f64) -> bool {
    (observed - sent).abs() <= limit
}

/// `value` moved toward `target` by at most `step`, then held inside `range`.
fn slew(value: f64, target: f64, step: f64, range: CommandRange) -> f64 {
    (value + (target - value).clamp(-step, step)).clamp(range.lower, range.upper)
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
        if self.latched.is_some() {
            return Err(GovernorError::Latched);
        }
        check_targets(self.chain.axes(), joints)?;
        let now = self.clock.now();
        let held = match self.held.take() {
            Some(held) => held,
            None => self.seed(now)?,
        };
        let elapsed = held.at.map_or(MAX_STEP_INTERVAL, |at| {
            now.saturating_duration_since(at).min(MAX_STEP_INTERVAL)
        });
        let mut approaching = held.approaching;
        // An axis its device is still bringing into range holds the in-range position nearest
        // where it rested; every other axis slews.
        let next: Vec<f64> = held
            .position
            .iter()
            .zip(joints)
            .zip(self.limits.iter().zip(self.chain.commandable()))
            .zip(&approaching)
            .map(|(((value, target), (limit, range)), approach)| {
                if approach.is_some() {
                    *value
                } else {
                    slew(
                        *value,
                        *target,
                        limit.max_speed * elapsed.as_secs_f64(),
                        *range,
                    )
                }
            })
            .collect();
        let receipt = match self.chain.command(&next) {
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
        self.check_approaches(&mut approaching, &receipt.observed, &next, now)?;
        let strayed = receipt
            .observed
            .iter()
            .zip(&next)
            .zip(self.limits.iter().zip(self.chain.axes()))
            .zip(&approaching)
            .filter(|(_, approach)| approach.is_none())
            .map(|(pair, _)| pair)
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
                stop,
            }));
        }
        self.held = Some(Held {
            position: next,
            at: Some(now),
            approaching,
        });
        Ok(receipt)
    }

    fn stop(&mut self) -> StopReport {
        self.held = None;
        self.chain.stop()
    }

    fn stop_budget(&self) -> Duration {
        self.chain.stop_budget()
    }

    fn last_observed(&self) -> Option<Vec<f64>> {
        self.chain.last_observed()
    }

    fn approach_speed_limited(&self, axis: usize) -> bool {
        self.chain.approach_speed_limited(axis)
    }

    fn clear_faults(&mut self) {
        self.latched = None;
        self.held = None;
        self.chain.clear_faults();
    }
}

#[cfg(test)]
mod tests {
    use std::cell::Cell;
    use std::rc::Rc;

    use super::*;
    use crate::binding::{ActuatorBinding, ActuatorBus, ActuatorModel};

    #[derive(Debug, Error)]
    #[error("fake chain failure: {0:?}")]
    struct FakeError(ChainFailure);

    impl ChainError for FakeError {
        fn failure(&self) -> ChainFailure {
            self.0
        }
    }

    /// Two axes on -1..1 rad, commandable -0.9..0.9; observes the target plus `lag` when
    /// `observes`, resting at `rest` while stopped.
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
        /// Axes whose device limits its own approach speed.
        approach_limited: [bool; 2],
        /// When set, the device moves each axis toward what was sent by at most this per
        /// command, from `rest`, and reports where it is.
        device_step: Option<f64>,
        actual: Option<Vec<f64>>,
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
            approach_limited: [false; 2],
            device_step: None,
            actual: None,
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
            assert!(
                self.commandable
                    .iter()
                    .zip(joints)
                    .all(|(range, joint)| range.admits(*joint)),
                "the governor sent {joints:?} outside the commandable ranges"
            );
            if let Some(failure) = self.fail {
                return Err(FakeError(failure));
            }
            self.sent.push(joints.to_vec());
            let reached: Vec<f64> = match self.device_step {
                Some(step) => {
                    let actual = self.actual.get_or_insert_with(|| self.rest.clone());
                    for (at, target) in actual.iter_mut().zip(joints) {
                        *at += (target - *at).clamp(-step, step);
                    }
                    actual.clone()
                }
                None => joints.to_vec(),
            };
            Ok(Receipt {
                evidence: crate::chain::Evidence::BusObservedComplete,
                observed: if self.observes {
                    reached
                        .iter()
                        .map(|joint| joint + self.lag)
                        .take(if self.short { 1 } else { usize::MAX })
                        .collect()
                } else {
                    Vec::new()
                },
                feedback: Vec::new(),
            })
        }

        fn stop(&mut self) -> StopReport {
            self.stops += 1;
            StopReport::default()
        }

        fn stop_budget(&self) -> Duration {
            Duration::from_millis(5)
        }

        fn last_observed(&self) -> Option<Vec<f64>> {
            self.observes.then(|| self.rest.clone())
        }

        fn approach_speed_limited(&self, axis: usize) -> bool {
            self.approach_limited[axis]
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
    };

    fn governed(chain: Fake) -> (Governor<Fake, Manual>, Manual) {
        let clock = Manual(Rc::new(Cell::new(Instant::now())));
        let governor = Governor::with_clock(chain, vec![LIMITS; 2], clock.clone()).expect("limits");
        (governor, clock)
    }

    fn close(left: &[f64], right: &[f64]) -> bool {
        left.iter().zip(right).all(|(a, b)| (a - b).abs() < 1e-12)
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
        // Resting just past the commandable edge, as a joint against its stop can read.
        chain.rest = vec![0.95, 0.0];
        let (mut governor, _) = governed(chain);
        governor.command(&[0.9, 0.0]).expect("from past the edge");
        assert!(close(&governor.chain().sent[0], &[0.9, 0.0]));
        assert!(matches!(
            governor.command(&[1.5, 0.0]),
            Err(GovernorError::Target(_))
        ));
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
    fn a_nan_observation_is_a_tracking_fault() {
        let mut chain = fake(true);
        chain.lag = f64::NAN;
        let (mut governor, _) = governed(chain);
        assert!(matches!(
            governor.command(&[0.0, 0.5]),
            Err(GovernorError::Tracking { .. })
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
        assert!(governor.start_at(vec![0.0]).is_err());
        governor.start_at(vec![0.2, -0.2]).expect("start");
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
                },
            ];
            assert!(matches!(
                Governor::new(fake(true), limits),
                Err(LimitsError::NotPositive(joint)) if joint == "b"
            ));
        }
    }

    #[test]
    fn an_axis_resting_outside_its_range_refuses_sends_nothing_and_latches() {
        let mut chain = fake(true);
        // The joint rests 1.5 rad below its range, far past its 0.2 rad tracking bound.
        chain.rest = vec![-2.4, 0.0];
        let (mut governor, clock) = governed(chain);
        match governor.command(&[0.0, 0.0]) {
            Err(GovernorError::RestingOutside {
                joint,
                observed,
                lower,
                ..
            }) => {
                assert_eq!(joint, "a");
                assert!((observed + 2.4).abs() < 1e-12);
                assert!((lower + 0.9).abs() < 1e-12);
            }
            other => panic!("expected a refusal, got {other:?}"),
        }
        assert!(
            governor.chain().sent.is_empty(),
            "no jump to the range's edge"
        );
        assert!(matches!(
            governor.command(&[0.0, 0.0]),
            Err(GovernorError::Latched)
        ));
        // Clearing while the chain still observes it outside refuses again.
        governor.clear_faults();
        assert!(matches!(
            governor.command(&[0.0, 0.0]),
            Err(GovernorError::RestingOutside { .. })
        ));
        // Recovered by hand and stated by the operator: commands start from there.
        governor.start_at(vec![-0.5, 0.0]).expect("a start");
        clock.advance(Duration::from_millis(20));
        governor.command(&[0.0, 0.0]).expect("after recovery");
        assert!(close(&governor.chain().sent[0], &[-0.495, 0.0]));
        assert!(
            governor
                .chain()
                .sent
                .iter()
                .flatten()
                .all(|v| v.abs() <= 0.9)
        );
        // Or cleared once the chain observes it inside again.
        governor.stop();
        governor.chain_mut().rest = vec![-2.4, 0.0];
        assert!(governor.command(&[0.0, 0.0]).is_err());
        governor.chain_mut().rest = vec![-0.7, 0.0];
        governor.clear_faults();
        governor
            .command(&[0.0, 0.0])
            .expect("observed inside again");
        assert!(close(&governor.chain().sent[1], &[-0.695, 0.0]));
    }

    #[test]
    fn a_device_limited_axis_is_sent_the_nearest_edge_until_it_is_inside() {
        let mut chain = fake(true);
        chain.rest = vec![-2.4, 0.0];
        chain.approach_limited = [true, false];
        chain.device_step = Some(0.3);
        let (mut governor, clock) = governed(chain);
        // The device closes 0.3 rad per command: observed -2.1, -1.8, -1.5, -1.2, then -0.9.
        for round in 0..5 {
            clock.advance(Duration::from_millis(20));
            governor
                .command(&[0.5, 0.0])
                .unwrap_or_else(|error| panic!("round {round}: {error}"));
            let sent = governor.chain().sent.last().expect("sent").clone();
            assert!(
                (sent[0] + 0.9).abs() < 1e-12,
                "round {round}: the edge, held"
            );
            assert!(sent[1].abs() < 1e-12);
        }
        // Inside: it slews from the edge, one step, and its tracking is checked again.
        clock.advance(Duration::from_millis(20));
        governor.command(&[0.5, 0.0]).expect("inside");
        assert!(close(&governor.chain().sent[5], &[-0.895, 0.0]));
        governor.chain_mut().lag = 0.3;
        clock.advance(Duration::from_millis(20));
        assert!(matches!(
            governor.command(&[0.5, 0.0]),
            Err(GovernorError::Tracking { joint, .. }) if joint == "a"
        ));
    }

    #[test]
    fn only_device_limited_axes_approach_and_a_slight_overshoot_still_slews() {
        // The other axis of the same chain rests outside without the capability: refused.
        let mut chain = fake(true);
        chain.rest = vec![0.0, 1.5];
        chain.approach_limited = [true, false];
        let (mut governor, _) = governed(chain);
        assert!(matches!(
            governor.command(&[0.0, 0.0]),
            Err(GovernorError::RestingOutside { joint, .. }) if joint == "b"
        ));
        assert!(governor.chain().sent.is_empty());
        // Within the tracking bound of the edge (a joint against its stop) it slews as before.
        let mut chain = fake(true);
        chain.rest = vec![1.05, 0.0];
        let (mut governor, _) = governed(chain);
        governor.command(&[0.0, 0.0]).expect("a slight overshoot");
        assert!(close(&governor.chain().sent[0], &[0.9, 0.0]));
    }

    /// A governor over an axis "a" resting at -2.4, approach-limited, whose device closes
    /// `step` per command.
    fn approaching(step: f64) -> (Governor<Fake, Manual>, Manual) {
        let mut chain = fake(true);
        chain.rest = vec![-2.4, 0.0];
        chain.approach_limited = [true, false];
        chain.device_step = Some(step);
        governed(chain)
    }

    #[test]
    fn an_approach_that_stops_getting_closer_is_a_latched_fault() {
        let (mut governor, clock) = approaching(0.0);
        for round in 0..APPROACH_STALL_OBSERVATIONS - 1 {
            clock.advance(Duration::from_millis(20));
            governor
                .command(&[0.0, 0.0])
                .unwrap_or_else(|error| panic!("round {round}: {error}"));
        }
        clock.advance(Duration::from_millis(20));
        match governor.command(&[0.0, 0.0]) {
            Err(GovernorError::ApproachStalled {
                joint, distance, ..
            }) => {
                assert_eq!(joint, "a");
                assert!((distance - 1.5).abs() < 1e-9);
            }
            other => panic!("expected a stalled approach, got {other:?}"),
        }
        assert_eq!(governor.chain().stops, 1);
        assert!(matches!(
            governor.command(&[0.0, 0.0]),
            Err(GovernorError::Latched)
        ));
    }

    #[test]
    fn an_approach_moving_away_or_observed_as_nan_is_a_latched_fault() {
        let (mut governor, clock) = approaching(0.3);
        clock.advance(Duration::from_millis(20));
        governor.command(&[0.0, 0.0]).expect("closing in");
        // Observed 0.5 farther than its closest, past the 0.2 tracking bound.
        governor.chain_mut().lag = -0.8;
        clock.advance(Duration::from_millis(20));
        assert!(matches!(
            governor.command(&[0.0, 0.0]),
            Err(GovernorError::ApproachStalled { .. })
        ));
        assert_eq!(governor.chain().stops, 1);
        let (mut governor, clock) = approaching(0.3);
        governor.chain_mut().lag = f64::NAN;
        clock.advance(Duration::from_millis(20));
        assert!(matches!(
            governor.command(&[0.0, 0.0]),
            Err(GovernorError::Tracking { joint, observed, .. }) if joint == "a" && observed.is_nan()
        ));
        assert_eq!(governor.chain().stops, 1);
    }

    #[test]
    fn an_approach_slower_than_its_deadline_times_out() {
        // 1.5 rad outside at 0.25 rad/s: 2 x 6 s + 1 s allowed.
        let (mut governor, clock) = approaching(0.01);
        clock.advance(Duration::from_millis(20));
        governor.command(&[0.0, 0.0]).expect("closing in, slowly");
        clock.advance(Duration::from_secs(13));
        match governor.command(&[0.0, 0.0]) {
            Err(error @ GovernorError::ApproachTimeout { .. }) => {
                assert_eq!(error.failure(), ChainFailure::Timeout);
                let GovernorError::ApproachTimeout { allowed_s, .. } = error else {
                    unreachable!()
                };
                assert!((allowed_s - 13.0).abs() < 1e-9);
            }
            other => panic!("expected a timeout, got {other:?}"),
        }
        assert_eq!(governor.chain().stops, 1);
    }
}
