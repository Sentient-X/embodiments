//! Local hand guidance and governed replay. All hardware access stays on the caller's
//! control thread; terminal input and episode storage are separate consumers.

use std::collections::{BTreeMap, BTreeSet};
use std::time::{Duration, Instant};

use sx_embodiment_drivers::dynamics::Scratch;
use sx_embodiment_drivers::guidance::{GuidanceProfile, GuidanceScratch};
use sx_embodiment_drivers::{
    ActuatorChain, AxisLimits, AxisReading, CompliantCommand, Dynamics, Feedforward,
    FeedforwardLimit, Governor, Receipt, Reports, Rest,
};

use crate::teach_episode::Episode;

/// Replay follows the recorded timestamps. The governor retains hardware limits.
pub const PLAYBACK_RATE: f64 = 1.0;

const FRESH: Duration = Duration::from_millis(50);
const HOLD_ERROR: f64 = 0.2;
const REPLAY_POSITION_TOLERANCE: f64 = 0.05;
const REPLAY_STATIONARY_SPEED: f64 = 0.05;
const APPROACH_SETTLE_ALLOWANCE: Duration = Duration::from_secs(30);
const SUPPORT_WINDOW: Duration = Duration::from_millis(500);
const SUPPORT_SPREAD: f64 = 0.003;
const ENGAGEMENT_RAMP: Duration = Duration::from_secs(1);
const ACTIVATION_BUDGET: Duration = Duration::from_secs(1);
const GUIDANCE_RAMP: Duration = Duration::from_secs(2);

/// Local operator's choices. `held` supplies explicitly measured, mechanically fixed
/// coordinates that the body cannot observe; they are not recorded as measured axes.
#[derive(Clone, Debug)]
pub struct Config {
    pub joints: Vec<String>,
    /// Canonical gripper opening maps, remapped into the opened body's native axis order.
    pub grippers: Vec<sx_embodiment_ik::teleop::Gripper>,
    pub held: BTreeMap<String, f64>,
    pub speed: f64,
    pub share: f64,
    pub damping: f64,
    pub period: Duration,
    pub max_seconds: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Disarmed,
    Hold,
    Teach,
    Approach,
    ReadyReplay,
    Replay,
    Fault,
    Released,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Action {
    Engage,
    Teach,
    Hold,
    Stop,
    Replay,
    /// Adjust every in-service gripper by this fraction of its range, only while held.
    GripperStep(f64),
    Release,
    /// Explicit supported-arm exit: brake/hold, then confirm torque release.
    Exit,
}

#[derive(Clone, Debug)]
pub struct Snapshot {
    pub mode: Mode,
    pub measured_at: Instant,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
}

/// Read-only diagnostics in the same active-joint order as [`Controller::joints`].
/// Times are host receipt/model-observation instants; motors supply no native clock here.
#[derive(Clone, Copy, Debug, Default)]
pub struct JointDiagnostic {
    pub calculated_gravity_nm: Option<f64>,
    /// Session-local frozen correction estimated from an explicitly accepted settled Hold.
    pub support_offset_nm: Option<f64>,
    /// Requested gravity retained after a completed dispatch, not measured applied torque.
    pub completed_gravity_command_nm: Option<f64>,
    /// Controller position target; a fault brake may hold a different measured position.
    pub position_target_rad: Option<f64>,
    pub feedback_velocity_rad_s: Option<f64>,
    pub feedback_torque_nm: Option<f64>,
    pub model_observed_at: Option<Instant>,
    pub gravity_command_completed_at: Option<Instant>,
    pub feedback_received_at: Option<Instant>,
}

#[derive(Clone, Copy, Debug, Default)]
struct MeasuredMotion {
    velocity: Option<f64>,
    torque: Option<f64>,
    received_at: Option<Instant>,
}

impl MeasuredMotion {
    fn from_native(velocity: f64, torque: f64, gain: f64, received_at: Option<Instant>) -> Self {
        let gain_valid = gain.is_finite() && gain != 0.0;
        Self {
            velocity: gain_valid
                .then(|| velocity / gain)
                .filter(|value| value.is_finite()),
            torque: gain_valid
                .then_some(torque * gain)
                .filter(|value| value.is_finite()),
            received_at,
        }
    }
}

/// One contiguous stationary Hold, discarded on movement or a target/load change.
struct SettledSupport {
    began: Option<Instant>,
    last: Option<Instant>,
    count: u32,
    residual: Vec<f64>,
    q_min: Vec<f64>,
    q_max: Vec<f64>,
    torque_min: Vec<f64>,
    torque_max: Vec<f64>,
    target: Vec<f64>,
}

impl SettledSupport {
    fn new(width: usize) -> Self {
        Self {
            began: None,
            last: None,
            count: 0,
            residual: vec![0.0; width],
            q_min: vec![0.0; width],
            q_max: vec![0.0; width],
            torque_min: vec![0.0; width],
            torque_max: vec![0.0; width],
            target: vec![0.0; width],
        }
    }
    fn reset(&mut self) {
        self.began = None;
        self.last = None;
        self.count = 0;
        self.residual.fill(0.0);
    }
    fn ready(&self) -> bool {
        self.count >= 2
            && self.began.zip(self.last).is_some_and(|(first, last)| {
                last.saturating_duration_since(first) >= SUPPORT_WINDOW && last.elapsed() <= FRESH
            })
    }
}

/// Separately calibrated hand-guidance support. None of these settings change the
/// position controller or its calibrated Hold/Replay model.
struct Guidance {
    profile: GuidanceProfile,
    scratch: GuidanceScratch,
    axes: Vec<Option<usize>>,
    required: Vec<bool>,
    q: Vec<f64>,
    torque: Vec<f64>,
    damping: Vec<f64>,
    started: Option<Instant>,
    last: Option<Instant>,
    from: Vec<f64>,
    returning: bool,
}

/// The local controller. No network, terminal, filesystem or policy inference work runs here.
pub struct Controller<C: ActuatorChain> {
    governor: Governor<C>,
    model: Dynamics,
    scratch: Scratch,
    guidance: Option<Guidance>,
    config: Config,
    selected: Vec<bool>,
    active: Vec<usize>,
    model_axes: Vec<Option<usize>>,
    model_fixed: Vec<f64>,
    gravity_required: Vec<bool>,
    model_required: Vec<bool>,
    model_q: Vec<f64>,
    model_tau: Vec<f64>,
    q: Vec<f64>,
    q_received: Vec<Option<Instant>>,
    dq: Vec<f64>,
    target: Vec<f64>,
    gravity: Vec<f64>,
    feedforward: Vec<Feedforward>,
    compliant: Vec<Option<CompliantCommand>>,
    torque_caps: Vec<f64>,
    observed_speed_limits: Vec<f64>,
    last: Instant,
    read_at: Instant,
    engaged: bool,
    engagement_started: Option<Instant>,
    engagement_complete: bool,
    support_offset: Vec<f64>,
    settled_support: SettledSupport,
    applied_gravity: Vec<f64>,
    last_motion: Vec<MeasuredMotion>,
    model_observed_at: Option<Instant>,
    gravity_command_completed_at: Option<Instant>,
    hold_problem: Option<String>,
    released_axes: Vec<bool>,
    silent_since: Vec<Option<Instant>>,
    began: Instant,
    mode: Mode,
    episode: Option<Episode>,
    pending_episode: Option<(Episode, usize)>,
    playback_at: Option<Instant>,
    approach_at: Option<(Instant, Duration)>,
    reached: Option<Instant>,
}

impl<C: ActuatorChain> Controller<C> {
    /// Validate every control/model input and read the complete state before enabling.
    ///
    /// # Errors
    /// Unknown, unsupported or unobserved joints, model coverage, and unsafe limits.
    // Keep the complete pre-enable refusal pass together; none of it sends a goal.
    #[allow(clippy::too_many_lines)]
    pub fn new(chain: C, model: Dynamics, config: Config) -> Result<Self, String> {
        if !(config.speed.is_finite()
            && config.speed > 0.0
            && config.speed <= 1.0
            && config.damping.is_finite()
            && config.damping > 0.0
            && config.max_seconds.is_finite()
            && config.max_seconds > 0.0
            && config.period >= Duration::from_millis(10)
            && config.period <= FRESH)
        {
            return Err(
                "speed must be in (0,1], damping/duration positive; period 10..50 ms".into(),
            );
        }
        let names: BTreeSet<_> = config.joints.iter().collect();
        if names.is_empty() || names.len() != config.joints.len() {
            return Err("--joints must contain unique selected arm joints".into());
        }
        let unavailable: BTreeSet<_> = chain.unavailable().into_iter().collect();
        let active: Vec<_> = (0..chain.axes().len())
            .filter(|i| !unavailable.contains(i))
            .collect();
        if active.is_empty() || active.len() > crate::teach_episode::MAX_AXES {
            return Err("the recorder requires 1..32 in-service axes".into());
        }
        for name in &config.joints {
            let index = chain
                .axes()
                .iter()
                .position(|a| &a.joint == name)
                .ok_or_else(|| format!("unknown selected joint {name}"))?;
            if unavailable.contains(&index) || chain.axes()[index].gripper {
                return Err(format!(
                    "{name} is unavailable or a gripper, not a teachable arm joint"
                ));
            }
        }
        let selected: Vec<_> = chain
            .axes()
            .iter()
            .map(|a| names.contains(&a.joint))
            .collect();
        for (name, value) in &config.held {
            if !value.is_finite() {
                return Err(format!("non-finite held coordinate {name}"));
            }
            if let Some(index) = chain.axes().iter().position(|a| &a.joint == name) {
                if !unavailable.contains(&index) {
                    return Err(format!(
                        "{name} is observable; do not override it with --held"
                    ));
                }
                if !chain.axes()[index].admits(*value) {
                    return Err(format!("held coordinate {name} is out of range"));
                }
            } else if !model.tree().joints().iter().any(|a| &a.name == name) {
                return Err(format!("unknown held coordinate {name}"));
            }
        }
        let limits =
            AxisLimits::rated_all(chain.axes(), config.share).map_err(|e| e.to_string())?;
        // Share and tracking ceilings govern commands; --speed caps positioning.
        // Measured motion is checked against hardware and physical rail derates.
        let derated = chain.derated();
        let hardware_limits =
            AxisLimits::rated_all(chain.axes(), 1.0).map_err(|e| e.to_string())?;
        let observed_speed_limits = hardware_limits
            .iter()
            .enumerate()
            .map(|(index, rated)| {
                let speed = rated.max_speed;
                derated
                    .iter()
                    .find(|d| d.axis == index)
                    .map_or(speed, |d| speed.min(d.limits.max_speed))
            })
            .collect();
        let mut torque_caps = Vec::new();
        for (index, axis) in chain.axes().iter().enumerate() {
            // Position-only servos accept no torque feedforward. An unrated torque
            // actuator is refused, never assigned a guessed continuous rating.
            let cap = match axis.binding.model.rating().rated_torque_nm {
                Some(rated) => rated * axis.binding.reduction * 0.8,
                None if axis
                    .binding
                    .bus
                    .command_answer()
                    .is_some_and(|a| a.reports_motion) =>
                {
                    return Err(format!("{} has no continuous torque rating", axis.joint));
                }
                None => 0.0,
            };
            if !(cap.is_finite() && cap >= 0.0) {
                return Err("invalid continuous torque rating".into());
            }
            let cap = chain
                .derated()
                .iter()
                .find(|d| d.axis == index)
                .map_or(cap, |d| {
                    cap.min(
                        d.limits.torque_share
                            * axis.binding.model.rating().peak_torque_nm
                            * axis.binding.reduction,
                    )
                });
            torque_caps.push(cap);
        }
        let width = chain.axes().len();
        let model_required: Vec<_> = model
            .tree()
            .joints()
            .iter()
            .map(|coordinate| {
                active.iter().any(|&index| {
                    torque_caps[index] > 0.0
                        && model.gravity_depends_on(&chain.axes()[index].joint, &coordinate.name)
                })
            })
            .collect();
        let mut model_axes = Vec::new();
        let mut model_fixed = Vec::new();
        for (model_index, joint) in model.tree().joints().iter().enumerate() {
            let index = chain.axes().iter().position(|a| a.joint == joint.name);
            // An out-of-service actuator can still provide live encoder feedback.
            // Only coordinates absent from the chain require a fixed model value.
            let fixed = if index.is_none() {
                if let Some(value) = config.held.get(&joint.name) {
                    *value
                } else if model_required[model_index] {
                    return Err(format!(
                        "{} is unobserved: supply its measured, mechanically fixed --held coordinate",
                        joint.name
                    ));
                } else {
                    joint
                        .limits
                        .map_or(0.0, |[low, high]| 0.0_f64.clamp(low, high))
                }
            } else {
                joint
                    .limits
                    .map_or(0.0, |[low, high]| 0.0_f64.clamp(low, high))
            };
            if let Some([low, high]) = joint.limits
                && index.is_none()
                && !(low..=high).contains(&fixed)
            {
                return Err(format!("held {} is out of model range", joint.name));
            }
            model_axes.push(index);
            model_fixed.push(fixed);
        }
        for &index in &active {
            if !model_axes.contains(&Some(index)) {
                return Err(format!(
                    "{} is absent from the dynamics model",
                    chain.axes()[index].joint
                ));
            }
        }
        // Only coordinates affecting controlled torque axes are observation dependencies.
        // Isolated branches cannot block this side's gravity compensation.
        let gravity_required = chain
            .axes()
            .iter()
            .map(|axis| {
                model
                    .tree()
                    .joints()
                    .iter()
                    .position(|joint| joint.name == axis.joint)
                    .is_some_and(|index| model_required[index])
            })
            .collect();
        let model_width = model_axes.len();
        let scratch = model.scratch();
        let mut governor = Governor::new(chain, limits).map_err(|e| e.to_string())?;
        governor
            .set_speed_cap(Some(config.speed))
            .map_err(|e| e.to_string())?;
        let ff_limits: Vec<_> = torque_caps
            .iter()
            .map(|cap| FeedforwardLimit {
                velocity: Some(0.0),
                torque: Some(*cap),
            })
            .collect();
        governor
            .set_feedforward_limits(&ff_limits)
            .map_err(|e| e.to_string())?;
        let now = Instant::now();
        let mut this = Self {
            governor,
            model,
            scratch,
            guidance: None,
            config,
            selected,
            active,
            model_axes,
            model_fixed,
            gravity_required,
            model_required,
            model_q: vec![0.0; model_width],
            model_tau: vec![0.0; model_width],
            q: vec![0.0; width],
            q_received: vec![None; width],
            dq: vec![0.0; width],
            target: vec![0.0; width],
            gravity: vec![0.0; width],
            feedforward: vec![Feedforward::default(); width],
            compliant: vec![None; width],
            torque_caps,
            observed_speed_limits,
            last: now,
            read_at: now,
            engaged: false,
            engagement_started: None,
            engagement_complete: false,
            support_offset: vec![0.0; width],
            settled_support: SettledSupport::new(width),
            applied_gravity: vec![0.0; width],
            last_motion: vec![MeasuredMotion::default(); width],
            model_observed_at: None,
            gravity_command_completed_at: None,
            hold_problem: None,
            released_axes: vec![false; width],
            silent_since: vec![None; width],
            began: now,
            mode: Mode::Disarmed,
            episode: None,
            pending_episode: None,
            playback_at: None,
            approach_at: None,
            reached: None,
        };
        this.observe(false)?;
        this.target.copy_from_slice(&this.q);
        this.compute_gravity()?;
        this.fill_compliant();
        this.governor
            .chain()
            .validate_compliant(&this.target, &this.feedforward, &this.compliant)
            .map_err(|e| e.to_string())?;
        this.last = Instant::now();
        Ok(this)
    }

    /// Construct with an optional, independently bound hand-guidance profile.
    /// Position-control gains and the original Hold/Replay model remain unchanged.
    ///
    /// # Errors
    /// As [`Self::new`], or an uncovered selected joint, unknown profile joint or
    /// an unobservable coordinate on which selected guidance torque depends.
    pub fn with_guidance(
        chain: C,
        model: Dynamics,
        config: Config,
        profile: Option<GuidanceProfile>,
    ) -> Result<Self, String> {
        let mut this = Self::new(chain, model, config)?;
        let Some(profile) = profile else {
            return Ok(this);
        };
        for name in &this.config.joints {
            if profile.joint_settings(name).is_none() {
                return Err(format!(
                    "guidance profile does not cover selected joint {name}"
                ));
            }
        }
        let mut mapping = Vec::new();
        let mut required = Vec::new();
        let mut damping = vec![this.config.damping; this.q.len()];
        for (coordinate, settings) in profile.tree().joints().iter().zip(profile.settings()) {
            if !this
                .model
                .tree()
                .joints()
                .iter()
                .any(|j| j.name == coordinate.name)
            {
                return Err(format!(
                    "guidance profile names unknown physical joint {}",
                    coordinate.name
                ));
            }
            let needed = this
                .config
                .joints
                .iter()
                .any(|joint| profile.gravity_depends_on(joint, &coordinate.name));
            let axis = this
                .governor
                .axes()
                .iter()
                .position(|a| a.joint == coordinate.name);
            if needed && axis.is_none() {
                return Err(format!(
                    "guidance coordinate {} is unobserved",
                    coordinate.name
                ));
            }
            if let Some(index) = axis {
                this.gravity_required[index] |= needed;
                damping[index] = settings.damping;
            }
            mapping.push(axis);
            required.push(needed);
        }
        let width = mapping.len();
        let scratch = profile.scratch();
        this.guidance = Some(Guidance {
            profile,
            scratch,
            axes: mapping,
            required,
            q: vec![0.0; width],
            torque: vec![0.0; width],
            damping,
            started: None,
            last: None,
            from: vec![0.0; this.q.len()],
            returning: false,
        });
        // Validate the actual guiding damping through the driver before enabling.
        let mut commands = this.compliant.clone();
        if let Some(guidance) = &this.guidance {
            for (index, command) in commands.iter_mut().enumerate() {
                if let Some(CompliantCommand::Torque { damping, .. }) = command {
                    *damping = guidance.damping[index];
                }
            }
        }
        this.governor
            .chain()
            .validate_compliant(&this.target, &this.feedforward, &commands)
            .map_err(|error| error.to_string())?;
        this.last = Instant::now();
        Ok(this)
    }

    #[must_use]
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Gravity support has reached its full value after the first enable/hold command.
    #[must_use]
    pub fn engagement_ready(&self) -> bool {
        self.engaged
            && !matches!(self.mode, Mode::Fault | Mode::Released)
            && self.engagement_complete
    }

    /// Whether an optional stationary support correction is available. Teach does not wait for it.
    #[must_use]
    pub fn teaching_ready(&self) -> bool {
        self.mode == Mode::Hold
            && self.engagement_ready()
            && self.settled_support.ready()
            && self.support_stationary()
    }

    #[must_use]
    pub fn replay_ready(&self) -> bool {
        self.episode.is_some() && self.pending_episode.is_none()
    }

    /// Replay follows the validated recording's original timestamps.
    #[must_use]
    pub const fn playback_rate(&self) -> f64 {
        PLAYBACK_RATE
    }

    #[must_use]
    pub fn joints(&self) -> Vec<String> {
        self.active
            .iter()
            .map(|i| self.governor.axes()[*i].joint.clone())
            .collect()
    }

    /// Fixed-size diagnostic copy without hardware reads, formatting or allocation.
    #[must_use]
    pub fn diagnostics(&self) -> [JointDiagnostic; crate::teach_episode::MAX_AXES] {
        let mut result = [JointDiagnostic::default(); crate::teach_episode::MAX_AXES];
        for (column, &index) in self.active.iter().enumerate() {
            let torque_controlled = self.torque_caps[index] > 0.0;
            let motion = self.last_motion[index];
            result[column] = JointDiagnostic {
                calculated_gravity_nm: torque_controlled.then_some(self.gravity[index]),
                support_offset_nm: (torque_controlled
                    && !(self.mode == Mode::Teach && self.guidance.is_some()))
                .then_some(self.support_offset[index]),
                completed_gravity_command_nm: if torque_controlled {
                    self.gravity_command_completed_at
                        .map(|_| self.applied_gravity[index])
                } else {
                    None
                },
                position_target_rad: Some(self.target[index]),
                feedback_velocity_rad_s: motion.velocity,
                feedback_torque_nm: motion.torque,
                model_observed_at: self.model_observed_at,
                gravity_command_completed_at: self.gravity_command_completed_at,
                feedback_received_at: motion.received_at,
            };
        }
        result
    }

    /// Replace the available replay only while no motion is running.
    ///
    /// # Errors
    /// A mismatched axis order, unsafe sample, or active motion.
    pub fn set_episode(&mut self, episode: Episode) -> Result<(), String> {
        if !matches!(self.mode, Mode::Disarmed | Mode::Hold) {
            return Err("load a replay only while disarmed or held".into());
        }
        if episode.header.joints != self.joints() {
            return Err("replay joint order differs from this body".into());
        }
        if episode.samples.len() < 2 {
            return Err("replay needs at least two samples".into());
        }
        if self.pending_episode.is_some() {
            return Err("a replay is already being validated".into());
        }
        // Bound validation work on each motor tick, even for an hour-long episode.
        self.pending_episode = Some((episode, 0));
        Ok(())
    }

    /// Terminal stop: preserve hold refresh and never resume motion automatically.
    pub fn fault(&mut self, _reason: &str) {
        self.settled_support.reset();
        self.engagement_started = None;
        self.engagement_complete = false;
        if matches!(self.mode, Mode::Disarmed | Mode::Released) {
            self.mode = Mode::Fault;
            return;
        }
        if self.mode != Mode::Fault {
            self.establish_hold();
        }
        self.mode = Mode::Fault;
    }

    /// Execute one local tick. Any error latches a hold before returning.
    ///
    /// # Errors
    /// Bad/missing state, control deadline, unsafe command or motor response.
    pub fn tick(&mut self, action: Option<Action>) -> Result<Snapshot, String> {
        let result = self.advance(action);
        if let Err(reason) = &result {
            self.fault(reason);
        }
        result
    }

    // Explicit transitions and their dispatch stay in one state-machine match.
    #[allow(clippy::too_many_lines)]
    fn advance(&mut self, action: Option<Action>) -> Result<Snapshot, String> {
        if self.mode == Mode::Released {
            return Ok(self.snapshot());
        }
        let action = if action == Some(Action::Exit) {
            self.fault("operator Ctrl-C: stop and release");
            Some(Action::Release)
        } else {
            action
        };
        if action == Some(Action::Release) {
            if !matches!(self.mode, Mode::Hold | Mode::Fault | Mode::Disarmed) {
                return Err("hold before releasing torque".into());
            }
            self.settled_support.reset();
            self.engagement_started = None;
            self.engagement_complete = false;
            let stop = self.governor.stop();
            self.mode = Mode::Released;
            if !stop.unproven.is_empty() || !stop.unreached.is_empty() {
                return Err(format!("release unproven: {stop:?}"));
            }
            return Ok(self.snapshot());
        }
        if self.mode == Mode::Fault {
            if !self.engaged {
                return Ok(self.snapshot());
            }
            let receipt = match self.governor.keep_holding() {
                Ok(receipt) => receipt,
                Err(error) => {
                    let missing = Receipt {
                        evidence: sx_embodiment_drivers::Evidence::DispatchAttempted,
                        observed: sx_embodiment_drivers::Axes::new(),
                        feedback: sx_embodiment_drivers::Axes::new(),
                    };
                    self.monitor_hold(&missing);
                    return Err(error.to_string());
                }
            };
            self.monitor_hold(&receipt);
            self.check_feedback(&receipt, false)?;
            if let Some(problem) = self.hold_problem.take() {
                return Err(problem);
            }
            self.last = Instant::now();
            return Ok(self.snapshot());
        }
        let elapsed = self.last.elapsed();
        if self.mode != Mode::Disarmed && elapsed > FRESH {
            return Err(format!(
                "{:?} control tick took {:.3} ms, exceeding 50 ms",
                self.mode,
                elapsed.as_secs_f64() * 1000.0
            ));
        }
        self.last = Instant::now();
        let retries_before = self.governor.retried();
        let read_began = Instant::now();
        self.observe(self.mode != Mode::Disarmed)?;
        let read_took = read_began.elapsed();
        let gravity_began = Instant::now();
        self.compute_gravity()?;
        let gravity_took = gravity_began.elapsed();
        if self.mode == Mode::Hold && !self.support_stationary() {
            self.settled_support.reset();
        }
        self.validate_pending()?;
        if action == Some(Action::Stop) {
            return Err("operator Stop: teaching/replay ended".into());
        }
        if self.began.elapsed().as_secs_f64() > self.config.max_seconds
            && self.mode != Mode::Disarmed
        {
            return Err("local session duration reached".into());
        }
        match action {
            Some(Action::Engage) if self.mode == Mode::Disarmed => {
                self.mode = Mode::Hold;
                self.engaged = true;
                self.engagement_started = None;
                self.engagement_complete = false;
                self.settled_support.reset();
                self.began = Instant::now();
                self.activate().map_err(|error| {
                    format!(
                        "engagement failed after {:.3} ms: {error}",
                        self.began.elapsed().as_secs_f64() * 1000.0
                    )
                })?;
            }
            Some(Action::Teach) if self.mode == Mode::Hold => {
                if self.guidance.is_none() && self.teaching_ready() {
                    self.accept_settled_support()?;
                }
                self.target.copy_from_slice(&self.q);
                self.settled_support.reset();
                self.mode = Mode::Teach;
                if let Some(guidance) = &mut self.guidance {
                    let now = Instant::now();
                    guidance.returning = false;
                    guidance.started = Some(now);
                    guidance.last = Some(now);
                    guidance.from.copy_from_slice(&self.applied_gravity);
                    self.compute_gravity()?;
                }
            }
            Some(Action::Hold) if self.mode != Mode::Disarmed => {
                self.settled_support.reset();
                if self.mode == Mode::Teach
                    && self
                        .active
                        .iter()
                        .any(|i| self.selected[*i] && self.dq[*i].abs() > 0.05)
                {
                    return Err("stop moving the arm before switching from teaching to hold".into());
                }
                let leaving_guidance = self.mode == Mode::Teach && self.guidance.is_some();
                let pause_began = Instant::now();
                self.establish_hold();
                if let Some(problem) = self.hold_problem.take() {
                    return Err(problem);
                }
                self.observe(false)?;
                self.mode = Mode::Hold;
                if leaving_guidance {
                    let guidance = self.guidance.as_mut().expect("guidance selected");
                    let now = Instant::now();
                    guidance.started = Some(now);
                    guidance.last = Some(now);
                    guidance.from.copy_from_slice(&self.applied_gravity);
                    guidance.returning = true;
                }
                self.compute_gravity()?;
                self.last = Instant::now();
                self.target.copy_from_slice(&self.q);
                self.governor.start_at(&self.q).map_err(|e| e.to_string())?;
                self.governor
                    .set_speed_cap(Some(self.config.speed))
                    .map_err(|e| e.to_string())?;
                // Braking intentionally blocks while retaining the previous support.
                // Resume the ramp from that support, without catching up in one frame.
                if let Some(started) = self.engagement_started.as_mut() {
                    *started += pause_began.elapsed();
                }
                self.mode = Mode::Hold;
            }
            Some(Action::Replay) if self.mode == Mode::Hold && self.engagement_ready() => {
                self.settled_support.reset();
                if self.pending_episode.is_some() {
                    return Err("replay is still being validated".into());
                }
                let first = self
                    .episode
                    .as_ref()
                    .ok_or("no complete local episode loaded")?
                    .samples
                    .first()
                    .ok_or("empty episode")?;
                for (column, &index) in self.active.iter().enumerate() {
                    self.target[index] = first.position[column];
                }
                self.governor.start_at(&self.q).map_err(|e| e.to_string())?;
                let mut travel_s: f64 = 0.0;
                for &index in &self.active {
                    travel_s = travel_s.max(approach_travel_seconds(
                        (self.target[index] - self.q[index]).abs(),
                        self.governor.limits()[index],
                    )?);
                }
                let budget =
                    Duration::try_from_secs_f64(travel_s + APPROACH_SETTLE_ALLOWANCE.as_secs_f64())
                        .map_err(|_| "replay approach budget is not a finite duration")?;
                self.approach_at = Some((Instant::now(), budget));
                self.reached = None;
                self.mode = Mode::Approach;
            }
            Some(Action::Replay) if self.mode == Mode::ReadyReplay => {
                if !self.replay_settled() {
                    return Err(
                        self.replay_settle_error("replay starting pose is no longer settled")
                    );
                }
                self.governor.start_at(&self.q).map_err(|e| e.to_string())?;
                self.governor
                    .set_speed_cap(None)
                    .map_err(|e| e.to_string())?;
                self.playback_at = Some(Instant::now());
                self.reached = None;
                self.mode = Mode::Replay;
            }
            Some(Action::GripperStep(fraction))
                if self.mode == Mode::Hold && self.engagement_ready() =>
            {
                self.settled_support.reset();
                if !fraction.is_finite() || fraction.abs() > 0.05 {
                    return Err("gripper step is at most five percent".into());
                }
                if self.config.grippers.is_empty() {
                    return Err("this body has no canonical gripper opening map".into());
                }
                self.check_gripper_map()?;
                if self
                    .config
                    .grippers
                    .iter()
                    .any(|g| g.fingers.iter().any(|(i, _, _)| !self.active.contains(i)))
                {
                    return Err("a canonical gripper contains an out-of-service finger".into());
                }
                for gripper in &self.config.grippers {
                    let (index, closed, open) = gripper.fingers[0];
                    let opening = ((self.target[index] - closed) / (open - closed) + fraction)
                        .clamp(0.0, 1.0);
                    let width = gripper.closed_width_m
                        + opening * (gripper.open_width_m - gripper.closed_width_m);
                    gripper.write(width, &mut self.target);
                }
            }
            _ => {}
        }
        let dispatch_began = Instant::now();
        match self.mode {
            Mode::Teach => {
                for &index in &self.active {
                    if !self.selected[index]
                        && !self.governor.axes()[index].gripper
                        && (self.q[index] - self.target[index]).abs() > HOLD_ERROR
                    {
                        return Err(format!(
                            "{} moved away from its held target",
                            self.governor.axes()[index].joint
                        ));
                    }
                }
                self.fill_compliant();
                let receipt = self
                    .governor
                    .chain_mut()
                    .command_compliant(&self.target, &self.feedforward, &self.compliant)
                    .map_err(|e| e.to_string())?;
                self.remember_support();
                self.check_feedback(&receipt, true)?;
                // Teach has delivered full support; returning to Hold must not
                // restart an earlier engagement ramp or block replay/gripper keys.
                self.engagement_complete = true;
            }
            Mode::Hold | Mode::Approach | Mode::ReadyReplay | Mode::Replay => {
                let done = if self.mode == Mode::Replay {
                    let since = self
                        .playback_at
                        .ok_or("missing replay start")?
                        .elapsed()
                        .as_secs_f64();
                    let mut output: Vec<_> = self.active.iter().map(|i| self.target[*i]).collect();
                    // Follow the recorded clock; ordinary governed command limits remain.
                    let done = self
                        .episode
                        .as_ref()
                        .ok_or("missing replay")?
                        .interpolate(since, PLAYBACK_RATE, &mut output)
                        .map_err(|e| e.to_string())?;
                    for (column, &index) in self.active.iter().enumerate() {
                        self.target[index] = output[column];
                    }
                    done
                } else {
                    false
                };
                let support_scale = self.scale_engagement_support();
                let receipt = self
                    .governor
                    .command_with(&self.target, &self.feedforward)
                    .map_err(|e| e.to_string())?;
                self.remember_support();
                // Start after the first command returns: watchdog setup/enable time
                // must not consume the zero-to-full support ramp.
                self.engagement_started.get_or_insert_with(Instant::now);
                self.engagement_complete |= support_scale >= 1.0;
                self.check_feedback(&receipt, false)?;
                if self.mode == Mode::Hold && support_scale >= 1.0 {
                    self.accumulate_settled_support(&receipt);
                } else {
                    self.settled_support.reset();
                }
                if self.mode == Mode::Approach {
                    let (started, budget) = self.approach_at.ok_or("missing approach start")?;
                    if started.elapsed() > budget {
                        return Err(self.replay_settle_error(&format!(
                            "replay starting pose was not reached after {:.3} s (budget {:.3} s)",
                            started.elapsed().as_secs_f64(),
                            budget.as_secs_f64(),
                        )));
                    }
                    let close = self.replay_settled();
                    if close {
                        let since = *self.reached.get_or_insert_with(Instant::now);
                        if since.elapsed() > Duration::from_millis(300) {
                            self.mode = Mode::ReadyReplay;
                        }
                    } else {
                        self.reached = None;
                    }
                } else if done {
                    let close = self.replay_settled();
                    if close {
                        let since = *self.reached.get_or_insert_with(Instant::now);
                        if since.elapsed() > Duration::from_millis(300) {
                            self.governor.start_at(&self.q).map_err(|e| e.to_string())?;
                            self.governor
                                .set_speed_cap(Some(self.config.speed))
                                .map_err(|e| e.to_string())?;
                            self.mode = Mode::Hold;
                        }
                    } else {
                        self.reached = None;
                    }
                    let duration = self
                        .episode
                        .as_ref()
                        .and_then(|e| e.samples.last())
                        .ok_or("missing replay end")?
                        .t_s
                        / PLAYBACK_RATE;
                    if self
                        .playback_at
                        .ok_or("missing replay start")?
                        .elapsed()
                        .as_secs_f64()
                        > duration + 10.0
                    {
                        return Err(
                            self.replay_settle_error("replay did not settle at its final pose")
                        );
                    }
                }
            }
            Mode::Disarmed | Mode::Fault | Mode::Released => {}
        }
        let dispatch_took = dispatch_began.elapsed();
        let took = self.last.elapsed();
        if self.mode != Mode::Disarmed && took > FRESH {
            return Err(format!(
                "{:?} control read/compute/write took {:.3} ms, exceeding 50 ms; read_ms={:.3}, initial_gravity_ms={:.3}, dispatch_ms={:.3}, retries={}",
                self.mode,
                took.as_secs_f64() * 1000.0,
                read_took.as_secs_f64() * 1000.0,
                gravity_took.as_secs_f64() * 1000.0,
                dispatch_took.as_secs_f64() * 1000.0,
                self.governor.retried().saturating_sub(retries_before),
            ));
        }
        Ok(self.snapshot())
    }

    // Optional gripper controls must not prevent arm teaching from starting.
    fn check_gripper_map(&self) -> Result<(), String> {
        let chain = self.governor.chain();
        let mut finger_axes = BTreeSet::new();
        for gripper in &self.config.grippers {
            if !gripper.closed_width_m.is_finite()
                || gripper.closed_width_m < 0.0
                || !gripper.open_width_m.is_finite()
                || gripper.open_width_m <= gripper.closed_width_m
                || gripper.fingers.is_empty()
            {
                return Err("invalid canonical gripper opening map".into());
            }
            for &(index, closed, open) in &gripper.fingers {
                if index >= chain.axes().len()
                    || !chain.axes()[index].gripper
                    || !chain.commandable()[index].admits(closed)
                    || !chain.commandable()[index].admits(open)
                    || (closed - open).abs() < 1e-6
                    || !finger_axes.insert(index)
                {
                    return Err("gripper opening map does not match the opened finger axes".into());
                }
            }
        }
        Ok(())
    }

    fn observe(&mut self, check_speed: bool) -> Result<(), String> {
        let before = Instant::now();
        let readings = self.governor.read_positions();
        let now = Instant::now();
        if now.duration_since(before) > FRESH {
            return Err("joint read exceeded 50 ms".into());
        }
        let dt = now
            .saturating_duration_since(self.read_at)
            .as_secs_f64()
            .max(1e-6);
        if readings.len() != self.q.len() {
            return Err("incomplete body reading".into());
        }
        for (index, reading) in readings.iter().enumerate() {
            let axis = &self.governor.axes()[index];
            let active = self.active.contains(&index);
            let received = match reading {
                AxisReading::Fresh { received, .. } => Some(*received),
                _ => None,
            };
            let value = match reading {
                AxisReading::Fresh { position, received }
                    if position.is_finite()
                        && now.saturating_duration_since(*received) <= FRESH =>
                {
                    if !active
                        && let Some(fixed) = self.config.held.get(&axis.joint)
                        && (position - fixed).abs() > 0.02
                    {
                        return Err(format!("fixed {} moved", axis.joint));
                    }
                    *position
                }
                _ if !active => {
                    if let Some(fixed) = self.config.held.get(&axis.joint) {
                        *fixed
                    } else if !self.gravity_required[index] {
                        // This coordinate is not measured here and is never recorded or
                        // commanded. Any in-range value gives the same gravity torques.
                        let range = self.governor.commandable()[index];
                        self.q[index].clamp(range.lower, range.upper)
                    } else {
                        return Err(format!("{} has no fresh passive-joint reading", axis.joint));
                    }
                }
                _ => {
                    return Err(format!(
                        "{} has no fresh fault-free reading: {reading:?}",
                        axis.joint
                    ));
                }
            };
            if (active
                || self.gravity_required[index]
                || self.config.held.contains_key(&axis.joint))
                && !self.governor.commandable()[index].admits(value)
            {
                return Err(format!("{} is outside its command range", axis.joint));
            }
            self.dq[index] = if check_speed && active {
                (value - self.q[index]) / dt
            } else {
                0.0
            };
            let speed_limit = self.observed_speed_limit(index);
            if check_speed && active && self.dq[index].abs() > speed_limit {
                let receive_dt = received.zip(self.q_received[index]).map_or_else(
                    || "unavailable".to_owned(),
                    |(current, previous)| {
                        format!(
                            "{:.6}",
                            current.saturating_duration_since(previous).as_secs_f64()
                        )
                    },
                );
                return Err(format!(
                    "{} exceeds observed speed limit: {:.6} rad/s > {:.6} rad/s; previous={:.9}, current={:.9}, delta={:.9} rad, batch_dt={dt:.6} s, host_receive_dt={receive_dt} s",
                    axis.joint,
                    self.dq[index].abs(),
                    speed_limit,
                    self.q[index],
                    value,
                    value - self.q[index],
                ));
            }
            self.q[index] = value;
            self.q_received[index] = received;
        }
        self.read_at = now;
        Ok(())
    }

    fn compute_gravity(&mut self) -> Result<(), String> {
        let guiding = self.mode == Mode::Teach && self.guidance.is_some();
        // Held, nonselected axes can still depend on a guided joint's position.
        // Refresh their normal model every tick, then overlay selected guidance.
        for (i, axis) in self.model_axes.iter().enumerate() {
            self.model_q[i] = if self.model_required[i] {
                axis.map_or(self.model_fixed[i], |index| self.q[index])
            } else {
                self.model_fixed[i]
            };
        }
        self.model
            .gravity_torque_into(&self.model_q, &mut self.scratch, &mut self.model_tau)
            .map_err(|e| e.to_string())?;
        self.gravity.fill(0.0);
        for (i, axis) in self.model_axes.iter().enumerate() {
            if let Some(index) = *axis {
                self.gravity[index] = self.model_tau[i];
            }
        }
        self.model_observed_at = Some(self.read_at);
        for &index in &self.active {
            let torque = self.gravity[index];
            if self.torque_caps[index] == 0.0 {
                self.gravity[index] = 0.0;
                self.feedforward[index] = Feedforward::default();
                continue;
            }
            if guiding && self.selected[index] {
                continue;
            }
            if !torque.is_finite() || torque.abs() > self.torque_caps[index] {
                return Err(format!(
                    "{} gravity torque exceeds continuous cap",
                    self.governor.axes()[index].joint
                ));
            }
        }
        self.apply_support_offset()?;
        if guiding {
            return self.compute_guidance();
        }
        if self.guidance.as_ref().is_some_and(|guide| guide.returning) {
            self.blend_guidance_support();
        }
        Ok(())
    }

    fn compute_guidance(&mut self) -> Result<(), String> {
        let guidance = self.guidance.as_mut().expect("guidance selected");
        for (i, axis) in guidance.axes.iter().enumerate() {
            guidance.q[i] = if guidance.required[i] {
                self.q[axis.expect("required guidance coordinate mapped")]
            } else {
                0.0
            };
        }
        guidance
            .profile
            .gravity_into(&guidance.q, &mut guidance.scratch, &mut guidance.torque)
            .map_err(|error| error.to_string())?;
        for (i, axis) in guidance.axes.iter().enumerate() {
            let Some(index) = *axis else { continue };
            if !self.selected[index] {
                continue;
            }
            let torque = guidance.torque[i];
            if !torque.is_finite() || torque.abs() > self.torque_caps[index] {
                return Err(format!(
                    "{} guidance gravity exceeds continuous cap",
                    self.governor.axes()[index].joint
                ));
            }
            self.gravity[index] = torque;
            self.feedforward[index] = Feedforward {
                torque,
                gravity: torque,
                velocity: 0.0,
            };
        }
        self.blend_guidance_support();
        self.model_observed_at = Some(self.read_at);
        Ok(())
    }

    /// Both mode changes retain the last completed support command and slew toward
    /// the destination model. Position-mode stiffness and damping do not change.
    /// A guidance torque cap takes precedence over slew on entry if inherited Hold
    /// support is already outside that cap; Hold itself keeps its normal bounds.
    fn blend_guidance_support(&mut self) {
        let guidance = self.guidance.as_mut().expect("guidance selected");
        let now = Instant::now();
        let ramp = guidance.started.map_or(0.0, |started| {
            (now.duration_since(started).as_secs_f64() / GUIDANCE_RAMP.as_secs_f64()).min(1.0)
        });
        let dt = guidance
            .last
            .map_or(0.0, |last| now.duration_since(last).as_secs_f64());
        let mut finished = ramp >= 1.0;
        for (i, axis) in guidance.axes.iter().enumerate() {
            let Some(index) = *axis else { continue };
            if !self.selected[index] {
                continue;
            }
            let desired = self.feedforward[index].gravity;
            let wanted = guidance.from[index] * (1.0 - ramp) + desired * ramp;
            let delta = guidance.profile.settings()[i].torque_slew_rate * dt;
            let applied = self.applied_gravity[index];
            let torque = wanted.clamp(applied - delta, applied + delta);
            let torque = if self.mode == Mode::Teach {
                guidance.profile.settings()[i]
                    .torque_limit
                    .map_or(torque, |limit| torque.clamp(-limit, limit))
            } else {
                torque
            };
            self.feedforward[index] = Feedforward {
                torque,
                gravity: torque,
                velocity: 0.0,
            };
            finished &= (torque - desired).abs() < 1e-9;
        }
        guidance.last = Some(now);
        if finished && guidance.returning {
            guidance.returning = false;
        }
    }

    fn apply_support_offset(&mut self) -> Result<(), String> {
        for &index in &self.active {
            if self.torque_caps[index] == 0.0
                || (self.mode == Mode::Teach && self.guidance.is_some() && self.selected[index])
            {
                continue;
            }
            let torque = self.gravity[index] + self.support_offset[index];
            if !torque.is_finite() || torque.abs() > self.torque_caps[index] {
                return Err(format!(
                    "{} corrected gravity exceeds continuous cap",
                    self.governor.axes()[index].joint
                ));
            }
            self.feedforward[index] = Feedforward {
                torque,
                gravity: torque,
                velocity: 0.0,
            };
        }
        Ok(())
    }

    fn validate_pending(&mut self) -> Result<(), String> {
        let Some((episode, at)) = self.pending_episode.as_mut() else {
            return Ok(());
        };
        let end = (*at + 64).min(episode.samples.len());
        for row in *at..end {
            let sample = &episode.samples[row];
            if sample.position.len() != self.active.len()
                || sample.velocity.len() != self.active.len()
                || !sample.t_s.is_finite()
                || sample.t_s < 0.0
            {
                return Err("replay sample has invalid width/timestamp".into());
            }
            for (column, &index) in self.active.iter().enumerate() {
                if !self.governor.commandable()[index].admits(sample.position[column])
                    || !sample.velocity[column].is_finite()
                {
                    return Err(format!(
                        "recorded {} is invalid/out of range",
                        self.governor.axes()[index].joint
                    ));
                }
                if row > 0 {
                    let previous = &episode.samples[row - 1];
                    let dt = sample.t_s - previous.t_s;
                    if !dt.is_finite() || dt <= 0.0 {
                        return Err("replay timestamps are not increasing".into());
                    }
                    let speed = ((sample.position[column] - previous.position[column]) / dt).abs();
                    if !speed.is_finite() {
                        return Err("recorded segment speed is nonfinite".into());
                    }
                }
            }
        }
        *at = end;
        if end == episode.samples.len() {
            let (episode, _) = self
                .pending_episode
                .take()
                .ok_or("missing pending replay")?;
            let duration = episode.samples.last().ok_or("missing replay duration")?.t_s;
            if !duration.is_finite() {
                return Err("required replay duration is nonfinite".into());
            }
            if let Some(previous) = self.episode.replace(episode) {
                std::thread::Builder::new()
                    .name("retire-episode".into())
                    .spawn(move || drop(previous))
                    .map_err(|e| e.to_string())?;
            }
        }
        Ok(())
    }

    fn fill_compliant(&mut self) {
        for (index, selected) in self.selected.iter().enumerate() {
            self.compliant[index] = selected.then_some(CompliantCommand::Torque {
                torque: self.feedforward[index].gravity,
                damping: if self.mode == Mode::Teach {
                    self.guidance
                        .as_ref()
                        .map_or(self.config.damping, |guide| guide.damping[index])
                } else {
                    self.config.damping
                },
            });
            if self.governor.axes()[index].gripper
                && self.active.contains(&index)
                && matches!(self.mode, Mode::Disarmed | Mode::Teach)
            {
                self.compliant[index] = Some(CompliantCommand::Released);
            }
            if *selected
                || (self.mode == Mode::Teach
                    && self.governor.axes()[index].gripper
                    && self.active.contains(&index))
            {
                self.target[index] = self.q[index];
            }
        }
    }

    fn replay_settled(&self) -> bool {
        self.active.iter().all(|&index| {
            let reports_motion = self.governor.axes()[index]
                .binding
                .bus
                .command_answer()
                .is_some_and(|answer| answer.reports_motion);
            replay_axis_settled(
                self.q[index] - self.target[index],
                reports_motion,
                self.last_motion[index],
            )
        })
    }

    fn replay_settle_error(&self, message: &str) -> String {
        let Some(&index) = self.active.iter().max_by(|&&a, &&b| {
            (self.q[a] - self.target[a])
                .abs()
                .total_cmp(&(self.q[b] - self.target[b]).abs())
        }) else {
            return message.to_owned();
        };
        format!(
            "{message}; worst joint {} error={:.6} rad, tolerance={REPLAY_POSITION_TOLERANCE:.6} rad, native velocity={:?} rad/s, stationary limit={REPLAY_STATIONARY_SPEED:.6} rad/s",
            self.governor.axes()[index].joint,
            (self.q[index] - self.target[index]).abs(),
            self.last_motion[index].velocity,
        )
    }

    fn observed_speed_limit(&self, index: usize) -> f64 {
        self.observed_speed_limits[index]
    }

    fn check_feedback(
        &mut self,
        receipt: &Receipt<C::Feedback>,
        teaching: bool,
    ) -> Result<(), String> {
        // Capture every available answer before any guard returns, including a partial/faulting reply.
        self.capture_motion(receipt);
        if receipt.observed.len() != self.q.len() || receipt.feedback.len() != self.q.len() {
            return Err("command did not observe the complete body".into());
        }
        for &index in &self.active {
            let axis = &self.governor.axes()[index];
            if !receipt.observed[index].is_finite() {
                return Err(format!("{} did not answer the command", axis.joint));
            }
            let answer = receipt.feedback[index];
            if let Some((velocity, torque)) = answer.motion() {
                let gain = axis.actuator_from_joint(axis.zero + 1.0);
                let speed = velocity.abs() / gain.abs();
                let effort = torque.abs() * gain.abs();
                let speed_limit = self.observed_speed_limit(index);
                let torque_limit = self.torque_caps[index];
                if !velocity.is_finite() || !torque.is_finite() {
                    return Err(format!(
                        "{} nonfinite motor feedback: velocity={velocity}, torque={torque}, gain={gain}",
                        axis.joint
                    ));
                }
                if speed > speed_limit {
                    return Err(format!(
                        "{} feedback speed: {speed:.6} rad/s > {speed_limit:.6} rad/s; joint_torque={effort:.6} Nm, torque_limit={torque_limit:.6} Nm; raw_velocity={velocity:.6}, raw_torque={torque:.6}, gain={gain:.6}",
                        axis.joint
                    ));
                }
                if effort > torque_limit {
                    return Err(format!(
                        "{} feedback torque: {effort:.6} Nm > {torque_limit:.6} Nm; joint_speed={speed:.6} rad/s, speed_limit={speed_limit:.6} rad/s; raw_velocity={velocity:.6}, raw_torque={torque:.6}, gain={gain:.6}",
                        axis.joint
                    ));
                }
            } else if teaching && self.selected[index] {
                return Err(format!("{} supplies no motion feedback", axis.joint));
            }
            if let Some((_, temperature)) = answer.temperature()
                && let Some(limit) = axis.binding.model.rating().thermal_limit_celsius
                && f64::from(temperature.hottest()) >= limit - 10.0
            {
                return Err(format!("{} reached its temperature margin", axis.joint));
            }
        }
        Ok(())
    }

    fn capture_motion(&mut self, receipt: &Receipt<C::Feedback>) {
        let readings = self.governor.command_readings();
        for &index in &self.active {
            self.last_motion[index] = receipt
                .feedback
                .get(index)
                .and_then(Reports::motion)
                .map_or_else(MeasuredMotion::default, |(velocity, torque)| {
                    let axis = &self.governor.axes()[index];
                    let gain = axis.actuator_from_joint(axis.zero + 1.0);
                    let received_at = readings
                        .as_ref()
                        .and_then(|readings| readings.get(index))
                        .and_then(|reading| match reading {
                            AxisReading::Fresh { received, .. } => Some(*received),
                            AxisReading::Missing(_) | AxisReading::Faulted(_) => None,
                        });
                    MeasuredMotion::from_native(velocity, torque, gain, received_at)
                });
        }
    }

    fn monitor_hold(&mut self, receipt: &Receipt<C::Feedback>) {
        self.capture_motion(receipt);
        let now = Instant::now();
        let readings = self.governor.command_readings();
        let mut changed = false;
        for &index in &self.active {
            if self.released_axes[index] {
                continue;
            }
            let mut release = readings
                .as_ref()
                .is_some_and(|r| matches!(r.get(index), Some(AxisReading::Faulted(_))));
            if receipt.observed.get(index).is_some_and(|q| q.is_finite()) {
                self.silent_since[index] = None;
            } else {
                let began = *self.silent_since[index].get_or_insert(now);
                release |= now.duration_since(began) >= Duration::from_millis(300);
            }
            let rating = self.governor.axes()[index].binding.model.rating();
            if let Some(answer) = receipt.feedback.get(index) {
                if let Some((_, temperature)) = answer.temperature() {
                    release |= rating
                        .thermal_limit_celsius
                        .is_some_and(|limit| f64::from(temperature.hottest()) >= limit);
                }
                if let Some((_, torque)) = answer.motion() {
                    release |= !torque.is_finite() || torque.abs() >= rating.peak_torque_nm;
                }
            }
            if release {
                self.released_axes[index] = true;
                changed = true;
                self.hold_problem = Some(format!(
                    "{} released after a motor fault, thermal/peak-torque limit or sustained silence; support the arm",
                    self.governor.axes()[index].joint
                ));
            }
        }
        if changed {
            self.establish_hold();
        }
    }

    fn support_stationary(&self) -> bool {
        self.active
            .iter()
            .all(|index| self.dq[*index].is_finite() && self.dq[*index].abs() <= 0.05)
            && self
                .active
                .iter()
                .filter(|index| self.selected[**index])
                .all(|index| {
                    let motion = self.last_motion[*index];
                    motion.received_at.is_some_and(|at| at.elapsed() <= FRESH)
                        && motion.velocity.is_some_and(|v| v.abs() <= 0.05)
                        && motion.torque.is_some_and(f64::is_finite)
                })
    }

    fn accumulate_settled_support(&mut self, receipt: &Receipt<C::Feedback>) {
        if !self.support_stationary() {
            self.settled_support.reset();
            return;
        }
        let window = &mut self.settled_support;
        let changed = self
            .active
            .iter()
            .any(|i| window.target[*i].to_bits() != self.target[*i].to_bits());
        let spread = self.active.iter().any(|i| {
            let q = receipt.observed[*i];
            (window.q_max[*i].max(q) - window.q_min[*i].min(q)) > SUPPORT_SPREAD
        }) || self.active.iter().filter(|i| self.selected[**i]).any(|i| {
            let torque = self.last_motion[*i]
                .torque
                .expect("stationary native torque");
            (window.torque_max[*i].max(torque) - window.torque_min[*i].min(torque))
                > self.config.damping * self.governor.limits()[*i].max_speed
        });
        if changed
            || window.last.is_some_and(|at| at.elapsed() > FRESH)
            || window.count > 0 && spread
        {
            window.reset();
        }
        let now = Instant::now();
        if window.count == 0 {
            window.began = Some(now);
            window.target.copy_from_slice(&self.target);
            for &index in &self.active {
                window.q_min[index] = receipt.observed[index];
                window.q_max[index] = receipt.observed[index];
                if self.selected[index] {
                    let torque = self.last_motion[index]
                        .torque
                        .expect("stationary native torque");
                    window.torque_min[index] = torque;
                    window.torque_max[index] = torque;
                }
            }
        }
        window.last = Some(now);
        window.count += 1;
        for &index in &self.active {
            let q = receipt.observed[index];
            window.q_min[index] = window.q_min[index].min(q);
            window.q_max[index] = window.q_max[index].max(q);
            if !self.selected[index] {
                continue;
            }
            let torque = self.last_motion[index]
                .torque
                .expect("stationary native torque");
            window.torque_min[index] = window.torque_min[index].min(torque);
            window.torque_max[index] = window.torque_max[index].max(torque);
            window.residual[index] += torque - self.gravity[index];
        }
    }

    fn accept_settled_support(&mut self) -> Result<(), String> {
        let mut offset = self.support_offset.clone();
        for &index in &self.active {
            if !self.selected[index] {
                continue;
            }
            offset[index] =
                self.settled_support.residual[index] / f64::from(self.settled_support.count);
            let torque = self.gravity[index] + offset[index];
            if !offset[index].is_finite()
                || !torque.is_finite()
                || torque.abs() > self.torque_caps[index]
            {
                return Err(format!(
                    "{} measured support exceeds continuous cap",
                    self.governor.axes()[index].joint
                ));
            }
        }
        self.support_offset.copy_from_slice(&offset);
        self.apply_support_offset()
    }

    /// Activation uses no arm position spring and no support torque. A fresh read
    /// after watchdog setup/enable supplies the first stiff hold's actual target.
    fn activate(&mut self) -> Result<(), String> {
        self.target.copy_from_slice(&self.q);
        self.fill_compliant();
        self.feedforward.fill(Feedforward::default());
        for command in self.compliant.iter_mut().flatten() {
            if let CompliantCommand::Torque { torque, .. } = command {
                *torque = 0.0;
            }
        }
        let began = Instant::now();
        let receipt = self
            .governor
            .chain_mut()
            .command_compliant(&self.target, &self.feedforward, &self.compliant)
            .map_err(|error| error.to_string())?;
        let took = began.elapsed();
        if took > ACTIVATION_BUDGET {
            return Err(format!(
                "activation took {:.3} ms, exceeding the separate 1000 ms startup budget",
                took.as_secs_f64() * 1000.0
            ));
        }
        self.check_feedback(&receipt, true)?;
        let readings = self
            .governor
            .chain()
            .command_readings()
            .ok_or("activation did not establish fresh joint feedback")?;
        for &index in &self.active {
            if !matches!(readings.get(index), Some(AxisReading::Fresh { received, .. }) if *received >= began && received.elapsed() <= ACTIVATION_BUDGET)
            {
                return Err(format!(
                    "{} has no fresh activation feedback",
                    self.governor.axes()[index].joint
                ));
            }
        }
        // From here, the fresh read, computation and stiff hold retain the ordinary
        // 50 ms cyclic bound. Only watchdog/enable work had the startup budget.
        self.last = Instant::now();
        self.observe(false)?;
        self.compute_gravity()?;
        self.target.copy_from_slice(&self.q);
        self.governor
            .start_at(&self.q)
            .map_err(|error| error.to_string())
    }

    fn scale_engagement_support(&mut self) -> f64 {
        let scale = if self.engagement_complete {
            1.0
        } else {
            self.engagement_started.map_or(0.0, |at| {
                (at.elapsed().as_secs_f64() / ENGAGEMENT_RAMP.as_secs_f64()).min(1.0)
            })
        };
        for ff in &mut self.feedforward {
            ff.torque *= scale;
            ff.gravity *= scale;
        }
        scale
    }

    fn remember_support(&mut self) {
        self.gravity_command_completed_at = Some(Instant::now());
        for (applied, command) in self.applied_gravity.iter_mut().zip(&self.feedforward) {
            *applied = command.gravity;
        }
    }

    fn establish_hold(&mut self) {
        let rests: Vec<_> = self
            .applied_gravity
            .iter()
            .enumerate()
            .map(|(index, gravity)| {
                if self.released_axes[index] {
                    Rest::Release
                } else {
                    Rest::Hold { gravity: *gravity }
                }
            })
            .collect();
        let stop = self.governor.stop_holding(&rests);
        if !stop.unproven.is_empty() || !stop.unreached.is_empty() || !stop.faults.is_empty() {
            self.hold_problem = Some(format!("hold not fully proven: {stop:?}"));
        }
        self.playback_at = None;
        self.approach_at = None;
        self.reached = None;
    }

    fn snapshot(&self) -> Snapshot {
        Snapshot {
            mode: self.mode,
            measured_at: self.read_at,
            position: self.active.iter().map(|i| self.q[*i]).collect(),
            velocity: self.active.iter().map(|i| self.dq[*i]).collect(),
        }
    }
}

fn approach_travel_seconds(distance: f64, limits: AxisLimits) -> Result<f64, String> {
    let acceleration_s = match limits.max_acceleration {
        Some(acceleration) if acceleration.is_finite() && acceleration > 0.0 => {
            limits.max_speed / acceleration
        }
        Some(_) => return Err("invalid replay approach acceleration".into()),
        None => 0.0,
    };
    let seconds = distance / limits.max_speed + acceleration_s;
    if !distance.is_finite()
        || distance < 0.0
        || !limits.max_speed.is_finite()
        || limits.max_speed <= 0.0
        || !seconds.is_finite()
    {
        return Err("invalid replay approach travel duration".into());
    }
    Ok(seconds)
}

fn replay_axis_settled(error: f64, reports_motion: bool, motion: MeasuredMotion) -> bool {
    error.is_finite()
        && error.abs() <= REPLAY_POSITION_TOLERANCE
        && (!reports_motion
            || (motion.received_at.is_some_and(|at| at.elapsed() <= FRESH)
                && motion
                    .velocity
                    .is_some_and(|v| v.is_finite() && v.abs() <= REPLAY_STATIONARY_SPEED)))
}

#[cfg(test)]
mod diagnostic_tests {
    use super::*;

    #[test]
    fn motion_diagnostics_preserve_signed_joint_units_and_real_receipt_time() {
        let received = Instant::now()
            .checked_sub(Duration::from_millis(20))
            .expect("20 ms clock history");
        let motion = MeasuredMotion::from_native(1.5, -3.0, -2.0, Some(received));
        assert!((motion.velocity.unwrap() + 0.75).abs() < 1e-12);
        assert!((motion.torque.unwrap() - 6.0).abs() < 1e-12);
        assert_eq!(motion.received_at, Some(received));
        let absent = MeasuredMotion::from_native(f64::NAN, f64::INFINITY, -2.0, None);
        assert!(
            absent.velocity.is_none() && absent.torque.is_none() && absent.received_at.is_none()
        );
        let invalid_gain = MeasuredMotion::from_native(1.0, 1.0, 0.0, Some(received));
        assert!(invalid_gain.velocity.is_none() && invalid_gain.torque.is_none());
    }

    #[test]
    fn replay_settling_uses_fresh_native_motion_and_position_only_servo_tolerance() {
        let still = MeasuredMotion {
            velocity: Some(0.02),
            received_at: Some(Instant::now()),
            ..MeasuredMotion::default()
        };
        assert!(replay_axis_settled(0.039, true, still));
        assert!(!replay_axis_settled(0.06, true, still));
        assert!(!replay_axis_settled(
            0.0,
            true,
            MeasuredMotion {
                velocity: Some(0.2),
                ..still
            }
        ));
        assert!(!replay_axis_settled(
            0.0,
            true,
            MeasuredMotion {
                received_at: Some(
                    Instant::now()
                        .checked_sub(Duration::from_millis(60))
                        .expect("60 ms clock history")
                ),
                ..still
            }
        ));
        assert!(!replay_axis_settled(0.0, true, MeasuredMotion::default()));
        // Position-only servo encoders can alternate coarse codes while remaining
        // inside tolerance. They do not supply native velocity to certify or reject.
        for error in [-0.04, 0.04, -0.04] {
            assert!(replay_axis_settled(error, false, MeasuredMotion::default()));
        }
        assert!(!replay_axis_settled(0.06, false, MeasuredMotion::default()));
    }

    #[test]
    fn approach_budget_includes_slow_travel_acceleration_and_settling() {
        let limits = AxisLimits {
            max_speed: 0.01,
            max_acceleration: Some(0.02),
            max_tracking_error: 0.2,
            acceleration_rise_s: None,
        };
        let travel = approach_travel_seconds(0.4, limits).expect("slow valid travel");
        assert!((travel - 40.5).abs() < 1e-12);
        let budget = Duration::try_from_secs_f64(travel + APPROACH_SETTLE_ALLOWANCE.as_secs_f64())
            .expect("finite budget");
        assert_eq!(budget, Duration::from_millis(70_500));
        assert!(approach_travel_seconds(f64::INFINITY, limits).is_err());
        assert!(
            approach_travel_seconds(
                0.4,
                AxisLimits {
                    max_speed: 0.0,
                    ..limits
                }
            )
            .is_err()
        );
        assert!(
            approach_travel_seconds(
                0.4,
                AxisLimits {
                    max_acceleration: Some(f64::NAN),
                    ..limits
                }
            )
            .is_err()
        );
    }
}
