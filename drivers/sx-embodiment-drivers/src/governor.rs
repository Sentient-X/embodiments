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
//!   stopped or failed starts from where the body rests: the chain's own
//!   [`ActuatorChain::last_observed`], or, for a chain that observes nothing, a position given
//!   with [`Governor::start_at`]. With neither, the command is refused.
//! - **Tracking error.** When a command's receipt carries observed positions, each must lie
//!   within `max_tracking_error` of what was just sent. One that does not stops the chain and
//!   latches a fault: every later command is refused until [`ActuatorChain::clear_faults`].
//!
//! The governor bounds what the station asks; it is not a substitute for the motors' own
//! protections, and on a chain that observes nothing it cannot see the body fail to follow.

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

/// A command the governor refused, or its chain's own error.
#[derive(Debug, Error)]
pub enum GovernorError<E> {
    #[error(transparent)]
    Chain(E),
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
    #[error("a tracking fault is latched; clear it before commanding")]
    Latched,
}

impl<E: ChainError> ChainError for GovernorError<E> {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::Chain(error) => error.failure(),
            Self::Target(_) | Self::NoStart | Self::Latched => ChainFailure::Refused,
            Self::Tracking { .. } => ChainFailure::Fault,
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

    /// Where the body rests, for a chain that observes nothing: the next command after open,
    /// a stop or a failure starts from it. A chain's own observation wins over it.
    ///
    /// # Errors
    ///
    /// Refuses a vector of the wrong width or outside the declared bounds.
    pub fn start_at(&mut self, joints: Vec<f64>) -> Result<(), TargetError> {
        check_targets(self.chain.axes(), &joints)?;
        self.start = Some(joints);
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

    fn seed(&mut self) -> Result<Held, GovernorError<C::Error>> {
        let width = self.chain.axes().len();
        let position = self
            .chain
            .last_observed()
            .filter(|observed| observed.len() == width)
            .or_else(|| self.start.take())
            .ok_or(GovernorError::NoStart)?;
        Ok(Held { position, at: None })
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
        if self.latched {
            return Err(GovernorError::Latched);
        }
        check_targets(self.chain.axes(), joints)?;
        let held = match self.held.take() {
            Some(held) => held,
            None => self.seed()?,
        };
        let now = self.clock.now();
        let elapsed = held.at.map_or(MAX_STEP_INTERVAL, |at| {
            now.saturating_duration_since(at).min(MAX_STEP_INTERVAL)
        });
        let next: Vec<f64> = held
            .position
            .iter()
            .zip(joints)
            .zip(self.limits.iter().zip(self.chain.commandable()))
            .map(|((value, target), (limit, range))| {
                slew(
                    *value,
                    *target,
                    limit.max_speed * elapsed.as_secs_f64(),
                    *range,
                )
            })
            .collect();
        let receipt = match self.chain.command(&next) {
            Ok(receipt) => receipt,
            // Where the body is after a failed command is the chain's to report again.
            Err(error) => return Err(GovernorError::Chain(error)),
        };
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
            self.latched = true;
            let stop = self.chain.stop();
            return Err(GovernorError::Tracking {
                joint,
                commanded,
                observed,
                limit,
                stop,
            });
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

    fn last_observed(&self) -> Option<Vec<f64>> {
        self.chain.last_observed()
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

    #[derive(Debug, Error)]
    #[error("fake chain failure")]
    struct FakeError;

    impl ChainError for FakeError {
        fn failure(&self) -> ChainFailure {
            ChainFailure::Fault
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
        fail: bool,
    }

    fn axis(joint: &str, bus_id: u16) -> BoundAxis {
        BoundAxis {
            joint: joint.to_owned(),
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
            fail: false,
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
            if self.fail {
                return Err(FakeError);
            }
            self.sent.push(joints.to_vec());
            Ok(Receipt {
                evidence: crate::chain::Evidence::BusObservedComplete,
                observed: if self.observes {
                    joints.iter().map(|joint| joint + self.lag).collect()
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
    fn a_chain_that_observes_nothing_needs_a_start_after_every_stop() {
        let (mut governor, _) = governed(fake(false));
        assert!(matches!(
            governor.command(&[0.0, 0.0]),
            Err(GovernorError::NoStart)
        ));
        assert!(governor.chain().sent.is_empty());
        assert!(governor.start_at(vec![0.0]).is_err());
        governor.start_at(vec![0.2, -0.2]).expect("start");
        governor
            .command(&[0.9, -0.9])
            .expect("from the given start");
        assert!(close(&governor.chain().sent[0], &[0.205, -0.205]));
        assert!(governor.stop().is_proven());
        assert!(matches!(
            governor.command(&[0.0, 0.0]),
            Err(GovernorError::NoStart)
        ));
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
        governor.chain_mut().fail = true;
        assert!(matches!(
            governor.command(&[0.9, 0.5]),
            Err(GovernorError::Chain(_))
        ));
        governor.chain_mut().fail = false;
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
}
