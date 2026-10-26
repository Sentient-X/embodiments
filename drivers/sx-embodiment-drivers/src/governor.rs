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
//! - **Range is a limit on motion, not on state.** Each step is held inside the chain's
//!   commandable range widened to where the joint is now ([`CommandRange::reaching`]):
//!   `[min(lower, now), max(upper, now)]`, `now` being the chain's latest observation
//!   ([`ActuatorChain::last_observed`]) or, for a chain that observes nothing, the position
//!   last sent. A joint resting outside its range can hold or move toward it at `max_speed`,
//!   never further out, and the interval shrinks as it comes in; the same rule is the chain's.
//!   Nothing is refused or latched for where a joint rests.
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
    ActuatorChain, AxisReading, ChainError, ChainFailure, CommandRange, Receipt, StopReport,
    TargetError, check_targets,
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
    #[error("a fault is latched; recover the body and clear it before commanding")]
    Latched,
}

impl<E: ChainError> ChainError for GovernorError<E> {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::Chain(error) | Self::Faulted { error, .. } => error.failure(),
            Self::Target(_) | Self::NoStart | Self::Latched => ChainFailure::Refused,
            Self::Tracking { .. } | Self::Unobserved { .. } => ChainFailure::Fault,
        }
    }
}

/// The position last sent, and when.
struct Held {
    position: Vec<f64>,
    at: Option<Instant>,
}

/// A chain whose commands are slewed and whose tracking is checked; see the module docs.
pub struct Governor<C, K = MonotonicClock> {
    chain: C,
    limits: Vec<AxisLimits>,
    clock: K,
    held: Option<Held>,
    start: Option<Vec<f64>>,
    latched: bool,
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
            latched: false,
        })
    }

    /// Where the body rests, as the operator states it: the next command after open, a stop or
    /// a failure starts from it, ahead of the chain's own observation. It is how a chain that
    /// observes nothing is started.
    ///
    /// # Errors
    ///
    /// Refuses a vector of the wrong width or outside the declared bounds.
    pub fn start_at(&mut self, joints: Vec<f64>) -> Result<(), TargetError> {
        check_targets(self.chain.axes(), &joints)?;
        self.start = Some(joints);
        self.held = None;
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
        Ok(Held { position, at: None })
    }

    /// Stop the chain and latch a fault, returning the error `make` builds from the stop.
    fn fault(
        &mut self,
        make: impl FnOnce(StopReport) -> GovernorError<C::Error>,
    ) -> GovernorError<C::Error> {
        self.latched = true;
        self.held = None;
        let stop = self.chain.stop();
        make(stop)
    }
}

/// Whether `observed` lies within `limit` of `sent`; never for a NaN.
fn within(observed: f64, sent: f64, limit: f64) -> bool {
    (observed - sent).abs() <= limit
}

/// `value` moved toward `target` by at most `step`, then held inside `range` widened to `now`,
/// where the joint is.
fn slew(value: f64, target: f64, step: f64, range: CommandRange, now: f64) -> f64 {
    let reach = range.reaching(now);
    (value + (target - value).clamp(-step, step)).clamp(reach.lower, reach.upper)
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
        if self.latched {
            return Err(GovernorError::Latched);
        }
        check_targets(self.chain.axes(), joints)?;
        let now = self.clock.now();
        let held = match self.held.take() {
            Some(held) => held,
            None => self.seed()?,
        };
        let elapsed = held.at.map_or(MAX_STEP_INTERVAL, |at| {
            now.saturating_duration_since(at).min(MAX_STEP_INTERVAL)
        });
        let step = elapsed.as_secs_f64();
        // Where each joint is now: the chain's latest observation, else the position last sent.
        let observed = self
            .chain
            .last_observed()
            .filter(|observed| observed.len() == held.position.len());
        let next: Vec<f64> = held
            .position
            .iter()
            .zip(joints)
            .zip(self.limits.iter().zip(self.chain.commandable()))
            .enumerate()
            .map(|(index, ((value, target), (limit, range)))| {
                let at = observed
                    .as_ref()
                    .map(|observed| observed[index])
                    .filter(|at| at.is_finite())
                    .unwrap_or(*value);
                slew(*value, *target, limit.max_speed * step, *range, at)
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
        let strayed = receipt
            .observed
            .iter()
            .zip(&next)
            .zip(self.limits.iter().zip(self.chain.axes()))
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

    /// A read moves nothing, so it passes through even while a fault is latched.
    fn read_positions(&mut self) -> Vec<AxisReading> {
        self.chain.read_positions()
    }

    fn last_observed(&self) -> Option<Vec<f64>> {
        self.chain.last_observed()
    }

    fn command_readings(&self) -> Option<Vec<AxisReading>> {
        self.chain.command_readings()
    }

    fn clear_faults(&mut self) {
        self.latched = false;
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
        /// Where the last command's answer observed each axis; `None` once stopped.
        at: Option<Vec<f64>>,
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
            at: None,
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
            let observed: Vec<f64> = joints.iter().map(|joint| joint + self.lag).collect();
            self.at = Some(observed.clone());
            Ok(Receipt {
                evidence: crate::chain::Evidence::BusObservedComplete,
                observed: if self.observes {
                    observed
                        .into_iter()
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
            self.at = None;
            StopReport::default()
        }

        fn stop_budget(&self) -> Duration {
            Duration::from_millis(5)
        }

        fn last_observed(&self) -> Option<Vec<f64>> {
            self.observes
                .then(|| self.at.clone().unwrap_or_else(|| self.rest.clone()))
        }

        fn read_positions(&mut self) -> Vec<AxisReading> {
            vec![AxisReading::Missing(crate::chain::MissingReason::Timeout); self.axes.len()]
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
        // Resting just past the commandable edge, as a joint against its stop can read: it
        // comes in at the speed bound, never jumped to the edge.
        chain.rest = vec![0.95, 0.0];
        let (mut governor, _) = governed(chain);
        governor.command(&[0.9, 0.0]).expect("from past the edge");
        assert!(close(&governor.chain().sent[0], &[0.945, 0.0]));
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
}
