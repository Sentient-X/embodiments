//! The per-tick QP: assembled into buffers made once and solved by [`crate::qp::Qp`] in
//! place.

use std::time::{Duration, Instant};

use crate::IkError;
use crate::body::{Body, Target, Task, world};
use crate::geometry::{Obstacle, closest_segments, segment_box};
use crate::math::{
    V3, cross, dot, from_quaternion, log, mat_mul, mat_vec, norm, scale, sub, transpose,
};
use crate::model::Frames;
use crate::qp::Qp;

/// Per-axis speed (unit/s) and acceleration (unit/s^2) caps, in [`crate::Model::axes`] order.
#[derive(Clone, Debug, PartialEq)]
pub struct Caps {
    pub speed: Vec<f64>,
    pub accel: Vec<f64>,
}

impl Caps {
    /// The description's declared speeds, each reached from rest in `ramp_s` seconds (no
    /// description declares an acceleration).
    ///
    /// # Errors
    ///
    /// [`IkError::NoSpeed`] when a solved axis declares no `<limit velocity>`.
    pub fn declared(body: &Body, ramp_s: f64) -> Result<Self, IkError> {
        let axes = body.model.axes();
        let mut speed = Vec::with_capacity(axes.len());
        for (index, axis) in axes.iter().enumerate() {
            match axis.velocity {
                Some(v) => speed.push(v),
                None if !body.solved.contains(&index) => speed.push(0.0),
                None => return Err(IkError::NoSpeed(axis.name.clone())),
            }
        }
        let accel = speed.iter().map(|v| v / ramp_s).collect();
        Ok(Self { speed, accel })
    }

    /// Every speed no higher than `cap` (a governor's bring-up speed), each acceleration
    /// scaled with its speed.
    #[must_use]
    pub fn at_most(mut self, cap: f64) -> Self {
        for (speed, accel) in self.speed.iter_mut().zip(self.accel.iter_mut()) {
            if *speed > cap {
                *accel *= cap / *speed;
                *speed = cap;
            }
        }
        self
    }
}

/// The solver's weights and margins.
#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    /// How many metres of tool position error one radian of orientation error weighs.
    pub orientation_m_per_rad: f64,
    /// Weight on each tick's joint step, per rad^2 (position error is per m^2): the damped
    /// least-squares term. It makes the answer the smallest joint motion that closes the
    /// error, keeps a near-singular arm from spending its free direction, and costs a task
    /// direction with Jacobian gain `s` m/rad only `damping / s^2` of its error per tick.
    pub damping: f64,
    /// Posture weight toward the anchor (the measured posture at engage, [`Ik::reset`]), per
    /// rad^2: far below `damping`, so the free direction returns to where the operator
    /// started at `posture_weight / damping` of its offset per tick, never swings.
    pub posture_weight: f64,
    /// Posture weight for an axis that moves more than one limb (a torso).
    pub shared_posture_weight: f64,
    /// Posture weight toward a preferred posture ([`Ik::prefer_posture`]: a robot's
    /// commissioned Ready pose, elbows down): ten times the engage posture's, so the arms'
    /// redundancy leans toward it along a move, still far below the tool's own error. It is a
    /// weight, not a priority: where the posture and the goal disagree the tool sits off by it
    /// (on the static body under a millimetre and half a degree; settling is 5 mm and 3
    /// degrees), and twice it pushed far poses past their settle.
    pub preferred_posture_weight: f64,
    /// Kept clear of each joint limit.
    pub limit_margin_rad: f64,
    /// Kept between proxies, and between a proxy and an obstacle.
    pub collision_margin_m: f64,
    /// Pairs closer than this are candidates for a collision row.
    pub influence_m: f64,
    /// The share of a pair's full braking capability its approach is planned with.
    pub braking_share: f64,
    /// The most collision rows one tick carries (the closest candidates).
    pub max_rows: usize,
    /// The most any axis moves in one tick, whatever its speed cap: the QP is a
    /// linearisation, and a collision row is only as true as the step is short. Each axis's
    /// step is its cap times the tick ([`Caps`]), so this binds only above 3 rad/s at a 10 ms
    /// tick (a 0.5 m lever then moves 15 mm a tick against the 10 mm collision margin).
    pub max_step_rad: f64,
    /// The most error one tick tries to close (the QP is a linearisation).
    pub max_error_m: f64,
    pub max_error_rad: f64,
    /// Below both, a limb is [`Reach::Tracking`].
    pub tracking_m: f64,
    pub tracking_rad: f64,
    pub max_iterations: u32,
    /// The longest one step may take: past it the QP stops and the step holds
    /// ([`Solve::Held`]). The only clock the solver reads.
    pub deadline: Duration,
    /// The last stretch of a camera's line of sight to a tool that is the tool itself (the
    /// gripper's fingers and base) and never hides it ([`LimbReport::hidden_by`]).
    pub sight_tool_m: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            orientation_m_per_rad: 0.15,
            damping: 1e-4,
            posture_weight: 1e-6,
            shared_posture_weight: 1e-5,
            preferred_posture_weight: 1e-5,
            limit_margin_rad: 0.01,
            collision_margin_m: 0.01,
            influence_m: 0.2,
            braking_share: 0.5,
            max_rows: 32,
            max_step_rad: 0.03,
            max_error_m: 0.1,
            max_error_rad: 0.5,
            tracking_m: 0.005,
            tracking_rad: 0.05,
            max_iterations: 40,
            deadline: Duration::from_millis(4),
            sight_tool_m: 0.1,
        }
    }
}

/// A constraint binding at a step's answer. Axes index [`crate::Model::axes`], links
/// [`crate::Model::links`]; [`Active::describe`] names them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Active {
    Limit {
        axis: usize,
    },
    Speed {
        axis: usize,
    },
    /// At its acceleration cap, or past it where keeping a pair clear needed more.
    Accel {
        axis: usize,
    },
    SelfCollision {
        a: usize,
        b: usize,
    },
    Obstacle {
        link: usize,
        obstacle: usize,
    },
    /// A limb's tool point at the speed its target allows ([`Ik::set_tool_speed`]).
    ToolSpeed {
        limb: usize,
    },
}

impl Active {
    /// A cap that only slows the limb (it will arrive), never one that stops it short.
    #[must_use]
    pub const fn slows(&self) -> bool {
        matches!(
            self,
            Self::Speed { .. } | Self::Accel { .. } | Self::ToolSpeed { .. }
        )
    }

    /// What binds, in words (off the tick: this allocates).
    #[must_use]
    pub fn describe(&self, body: &Body) -> String {
        let axis = |i: usize| body.model.axes()[i].name.as_str();
        let link = |i: usize| body.model.links()[i].as_str();
        match *self {
            Self::Limit { axis: i } => format!("{} at its limit", axis(i)),
            Self::Speed { axis: i } => format!("{} at its speed cap", axis(i)),
            Self::Accel { axis: i } => format!("{} at its acceleration cap", axis(i)),
            Self::SelfCollision { a, b } => format!("{} kept off {}", link(a), link(b)),
            Self::Obstacle { link: l, obstacle } => {
                format!("{} kept off obstacle {obstacle}", link(l))
            }
            Self::ToolSpeed { limb } => format!(
                "{} at its tool speed cap",
                body.limbs.get(limb).map_or("a limb", |l| l.name.as_str())
            ),
        }
    }
}

/// How a limb stands after a step, for the ghost's tint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reach {
    /// No target this tick.
    Idle,
    /// Within [`Settings::tracking_m`] and [`Settings::tracking_rad`].
    Tracking,
    /// Behind only because of speed or acceleration caps: it will arrive.
    CatchingUp,
    /// A limit, a collision or the body's reach stops it short.
    Unreachable,
}

/// One limb after a step ([`Body::limbs`] order).
#[derive(Clone, Debug, PartialEq)]
pub struct LimbReport {
    /// Tool position error at the new positions, metres (zero for a look-at).
    pub residual_m: f64,
    /// Orientation (or pointing) error at the new positions, radians.
    pub residual_rad: f64,
    pub reach: Reach,
    /// Constraints binding on the limb's axes.
    pub active: Vec<Active>,
    /// For a driven tool, the link (index into [`crate::Model::links`]) standing between it
    /// and the camera at the new positions; `None` in clear sight, or with no camera.
    pub hidden_by: Option<usize>,
}

/// How the QP ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Solve {
    Solved,
    /// Solved, with a collision row needing more than the caps allow: the acceleration cap
    /// was exceeded on that row's axes (reported [`Active::Accel`]), or a pair already inside
    /// its margin was held to come no closer.
    Relaxed,
    /// The solver failed: every solved axis slows toward standing still within its caps.
    Held,
}

/// One tick's answer.
#[derive(Clone, Debug, PartialEq)]
pub struct Step {
    /// The next positions, every axis ([`crate::Model::axes`] order); unsolved and
    /// unavailable axes as given.
    pub q: Vec<f64>,
    /// The planned velocity to them, unit/s, every axis (zero where held).
    pub qdot: Vec<f64>,
    pub limbs: Vec<LimbReport>,
    pub solve: Solve,
    pub iterations: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Bound {
    Limit,
    Speed,
    Accel,
}

/// How much faster than its speed cap a joint is taken to move, at most: a follower rings past
/// its command, but no motor jumps the reference gap in a tick. Bounds the measured speed here
/// and the readings [`crate::teleop::Teleop`] takes.
pub const FOLLOWER_SPEED: f64 = 3.0;

/// How far a joint's reading may stray from where it is, radians, either way: an encoder's
/// count and a bus's jitter. A measured pose within this of the command on every axis a pair
/// moves is the command as far as the pair goes; two readings closer than twice this are not
/// progress ([`crate::STALL`]). The clearance tests jitter every reading by this much.
pub const READING_JITTER: f64 = 0.001;

/// How far into its span a row the caps cannot keep is eased: from the step that opens its
/// pair most toward the one that closes it most. Pinned at the opening end, the rows left the
/// QP a sliver to stand on and it failed on 28,688 steps of the clearance table, each failure
/// lifting every cap (the torso 15x; embodiments#206 refute); at a fifth, 1,009, and an
/// act-sim-like PD keeps 5.1 mm instead of touching. A tenth and a quarter both lost the PD.
const SOFTENED: f64 = 0.2;

/// How far a window may pull a step past the acceleration cap, as a multiple of it.
const WINDOW_PULL: f64 = 2.0;
/// Over how long a measured speed is smoothed, seconds.
const MEASURED_SPEED_S: f64 = 0.03;
/// How long a follower carries on at its measured speed, seconds: about the Spark's arm lag
/// (26-63 ms). A row at the measured pose counts the move that carries it, wherever it closes
/// the pair. An act-sim-like PD (zeta 0.2) then keeps 9.3 mm (5.1 without), zeta 0.1 7.7 mm
/// (main: 60 mm in); 30 and 100 ms kept less (embodiments#208).
const MOMENTUM_S: f64 = 0.06;

/// Over how long the lag a joint rests at is learned, seconds ([`Ik::set_measured`]): a
/// position-controlled joint stands a steady few milliradians short of its command under a load
/// (act-sim's arms 2.4-7.5 mrad), and that lag is no motion still to come.
const RESTING_S: f64 = 0.1;
/// A joint is at rest while its measured speed and its commanded velocity are both under this,
/// rad/s: its reading moves less than [`READING_JITTER`] over twice [`RESTING_S`].
const RESTING_SPEED: f64 = READING_JITTER / (2.0 * RESTING_S);

/// The cost of the rows' slack ([`Ik::solve_qp`]), against a cost scaled to a unit diagonal.
const SLACK_WEIGHT: f64 = 1e4;
/// Slack past this (the rows' units) is rows giving way, not the interior-point's sliver.
const SLACK_USED: f64 = 1e-6;
/// How far the slack may go, in the rows' units (unit-norm rows over one-tick speed steps).
const SLACK_ROOM: f64 = 1e3;

#[derive(Clone, Copy, Debug)]
enum Pair {
    /// Two capsules (by index) as measured this tick: a closest point on each and the
    /// direction from the second to the first, on the parts where the pieces refined them.
    Capsules {
        i: usize,
        j: usize,
        x: V3,
        y: V3,
        direction: V3,
    },
    /// A capsule and an obstacle.
    Scene(usize, usize),
}

#[derive(Clone, Copy, Debug)]
struct Candidate {
    distance: f64,
    pair: Pair,
    /// Measured between the links' convex pieces.
    refined: bool,
    /// At the body as measured ([`Ik::set_measured`]), not as commanded.
    measured: bool,
}

/// Every buffer a tick writes, sized once.
struct Workspace {
    frames: Frames,
    after: Frames,
    /// The body as measured ([`Ik::set_measured`]): pairs are kept clear here too.
    seen: Frames,
    /// Per axis, where it was measured, or where the step starts when it was not.
    seen_q: Vec<f64>,
    jv: Vec<V3>,
    jw: Vec<V3>,
    jp: Vec<V3>,
    h: Vec<f64>,
    g: Vec<f64>,
    task: Vec<f64>,
    lower: Vec<f64>,
    upper: Vec<f64>,
    safe_lower: Vec<f64>,
    safe_upper: Vec<f64>,
    lower_by: Vec<Bound>,
    upper_by: Vec<Bound>,
    yielded: Vec<bool>,
    still: Vec<f64>,
    candidates: Vec<Candidate>,
    rows: Vec<f64>,
    rhs: Vec<f64>,
    row_active: Vec<Active>,
    /// Per row, whether main never had it: a pair kept where the body was measured, or a
    /// stuck joint's copy. Such a row never widens a cap ([`Ik::keep_feasible`]).
    row_measured: Vec<bool>,
    row_binds: Vec<bool>,
    used: usize,
    unit: Vec<f64>,
    answer: Vec<f64>,
    column_active: Vec<Option<Active>>,
}

/// The smallest signed distances of one configuration ([`Ik::clearance_into`]), metres: between
/// the body's own checked proxies, and between the body and its scene (its support and the
/// site's obstacles, such as a cell's keep-out boxes). Negative is a penetration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Clearance {
    pub body_m: f64,
    pub scene_m: f64,
}

/// The per-tick solver: a [`Body`], its caps, a scene, the last commanded velocity, and every
/// buffer a tick needs: a steady-state [`Ik::step`] allocates nothing.
pub struct Ik {
    body: Body,
    caps: Caps,
    settings: Settings,
    scene: Vec<Obstacle>,
    rest: Vec<f64>,
    /// The posture the free direction is drawn to: the measured posture at the first step
    /// after [`Ik::reset`].
    anchor: Vec<f64>,
    anchored: bool,
    /// The posture preferred over the one measured at engage, when one is ([`Ik::prefer_posture`]).
    preferred: Option<Vec<f64>>,
    /// (capsule, obstacle) pairs checked: on a moving link, and apart at rest.
    scene_pairs: Vec<(usize, usize)>,
    /// Per solved column: moves more than one limb.
    shared: Vec<bool>,
    /// Per limb, its solved columns.
    limb_columns: Vec<Vec<usize>>,
    /// Per limb, per solved column: whether a look-at task may turn it. A camera turns on its
    /// own neck, never on an axis it shares with other limbs (a torso carrying the arms):
    /// looking around must not swing the hands. A camera with no axis of its own uses all.
    look_columns: Vec<Vec<bool>>,
    /// Per capsule: carried by a camera's own neck, so never between it and what it sees.
    sight_skip: Vec<bool>,
    /// Last commanded velocity per solved column.
    velocity: Vec<f64>,
    /// Per limb, the most its tool point may move, m/s, and from where ([`Ik::set_tool_speed`]).
    tool_speed: Vec<Option<(f64, Option<[f64; 3]>)>>,
    /// Per limb, whether its tool speed bound this tick.
    tool_capped: Vec<bool>,
    /// Per axis, where the next step must leave it ([`Ik::set_window`]).
    window: Vec<[f64; 2]>,
    /// Per axis, where it was last measured ([`Ik::set_measured`]); NaN where it was not.
    measured: Vec<f64>,
    /// Per axis, the measurement before this one, and a smoothed measured velocity (units/s).
    measured_before: Vec<f64>,
    measured_speed: Vec<f64>,
    /// Per axis, whether it does not follow its command ([`Ik::set_measured`]).
    stuck: Vec<bool>,
    /// Per axis, how far short of its command it rests, learned over [`RESTING_S`] while it
    /// and its command stand still (units; 0 until it has).
    resting_lag: Vec<f64>,
    work: Workspace,
    qp: Qp,
    step: Step,
}

impl Ik {
    /// `scene` is what the site places around the body; the body's declared support (the
    /// table its base mount stands on, [`crate::Proxies::support`]) is always added first.
    ///
    /// # Errors
    ///
    /// [`IkError::Caps`] when the caps are not one positive finite speed and acceleration per
    /// solved axis.
    #[allow(clippy::too_many_lines)] // sizing every buffer once, in one place
    pub fn new(
        body: Body,
        caps: Caps,
        settings: Settings,
        mut scene: Vec<Obstacle>,
    ) -> Result<Self, IkError> {
        // The body's own support (its declared base mount) is always in the scene; `scene`
        // adds what the site places around it.
        if let Some(support) = body.proxies.support {
            scene.insert(0, support);
        }
        let axes = body.model.axes();
        if caps.speed.len() != axes.len() || caps.accel.len() != axes.len() {
            return Err(IkError::Caps(format!(
                "{} axes need {} caps each",
                axes.len(),
                axes.len()
            )));
        }
        for &axis in &body.solved {
            let ok = |v: f64| v.is_finite() && v > 0.0;
            if !ok(caps.speed[axis]) || !ok(caps.accel[axis]) {
                return Err(IkError::Caps(format!(
                    "{} needs a positive finite speed and acceleration",
                    axes[axis].name
                )));
            }
        }
        let rest: Vec<f64> = axes
            .iter()
            .map(|axis| {
                axis.limits.map_or(0.0, |[lower, upper]| {
                    let margin = settings.limit_margin_rad.min((upper - lower) / 2.0);
                    0.0_f64.clamp(lower + margin, upper - margin)
                })
            })
            .collect();
        let at_rest = body.model.poses(&rest);
        let mut scene_pairs = Vec::new();
        for (k, obstacle) in scene.iter().enumerate() {
            for (index, capsule) in body.proxies.capsules.iter().enumerate() {
                let (a, b) = world(&at_rest, capsule);
                let moving = !body.model.above(capsule.link).is_empty();
                if moving && segment_box(obstacle, a, b).1 - capsule.radius > 0.0 {
                    scene_pairs.push((index, k));
                }
            }
        }
        let n = body.solved.len();
        let shared: Vec<bool> = body
            .solved
            .iter()
            .map(|axis| body.limbs.iter().filter(|l| l.axes.contains(axis)).count() > 1)
            .collect();
        let limb_columns: Vec<Vec<usize>> = body
            .limbs
            .iter()
            .map(|l| {
                l.axes
                    .iter()
                    .filter_map(|a| body.solved.binary_search(a).ok())
                    .collect()
            })
            .collect();
        let look_columns: Vec<Vec<bool>> = limb_columns
            .iter()
            .map(|columns: &Vec<usize>| {
                let own = columns.iter().any(|&c| !shared[c]);
                (0..n)
                    .map(|c| columns.contains(&c) && !(own && shared[c]))
                    .collect()
            })
            .collect();
        let sight_skip = body
            .proxies
            .capsules
            .iter()
            .map(|capsule| {
                body.model.above(capsule.link).iter().any(|term| {
                    body.solved.binary_search(&term.axis).is_ok_and(|c| {
                        !shared[c]
                            && body.limbs.iter().any(|l| {
                                matches!(l.task, Task::LookAt { .. }) && l.axes.contains(&term.axis)
                            })
                    })
                })
            })
            .collect();
        let k = settings.max_rows;
        // One column more than the solved axes: the rows' slack ([`Ik::solve_qp`]).
        let qp = Qp::new(n + 1, k);
        // Twice: the pairs at the commanded positions and at the measured ones.
        let candidates = Vec::with_capacity(2 * (body.proxies.pairs.len() + scene_pairs.len()));
        let work = Workspace {
            frames: body.model.poses(&rest),
            after: body.model.poses(&rest),
            seen: body.model.poses(&rest),
            seen_q: rest.clone(),
            jv: vec![[0.0; 3]; axes.len()],
            jw: vec![[0.0; 3]; axes.len()],
            jp: vec![[0.0; 3]; axes.len()],
            h: vec![0.0; n * n],
            g: vec![0.0; n],
            task: vec![0.0; n],
            lower: vec![0.0; n],
            upper: vec![0.0; n],
            safe_lower: vec![0.0; n],
            safe_upper: vec![0.0; n],
            lower_by: vec![Bound::Speed; n],
            upper_by: vec![Bound::Speed; n],
            yielded: vec![false; n],
            still: vec![0.0; n],
            candidates,
            rows: vec![0.0; k * n],
            rhs: vec![0.0; k],
            row_active: vec![Active::Limit { axis: 0 }; k],
            row_measured: vec![false; k],
            row_binds: vec![false; k],
            used: 0,
            unit: vec![1.0; n],
            answer: vec![0.0; n],
            column_active: vec![None; n],
        };
        let step = Step {
            q: rest.clone(),
            qdot: vec![0.0; rest.len()],
            limbs: body
                .limbs
                .iter()
                .map(|_| LimbReport {
                    residual_m: 0.0,
                    residual_rad: 0.0,
                    reach: Reach::Idle,
                    // Every column, every row and the tool's speed.
                    active: Vec::with_capacity(n + k + 1),
                    hidden_by: None,
                })
                .collect(),
            solve: Solve::Solved,
            iterations: 0,
        };
        let limbs = body.limbs.len();
        let axes_count = rest.len();
        Ok(Self {
            body,
            caps,
            settings,
            scene,
            anchor: rest.clone(),
            anchored: false,
            preferred: None,
            rest,
            scene_pairs,
            shared,
            limb_columns,
            look_columns,
            sight_skip,
            velocity: vec![0.0; n],
            tool_speed: vec![None; limbs],
            tool_capped: vec![false; limbs],
            window: vec![[f64::NEG_INFINITY, f64::INFINITY]; axes_count],
            measured: vec![f64::NAN; axes_count],
            measured_before: vec![f64::NAN; axes_count],
            measured_speed: vec![0.0; axes_count],
            stuck: vec![false; axes_count],
            resting_lag: vec![0.0; axes_count],
            work,
            qp,
            step,
        })
    }

    #[must_use]
    pub fn body(&self) -> &Body {
        &self.body
    }

    #[must_use]
    pub fn scene(&self) -> &[Obstacle] {
        &self.scene
    }

    #[must_use]
    pub fn caps(&self) -> &Caps {
        &self.caps
    }

    /// The longest one step may take from now on ([`Settings::deadline`]): an evaluation that
    /// scores answers, not a loaded host's clock, gives it room.
    pub fn set_deadline(&mut self, budget: Duration) {
        self.settings.deadline = budget;
    }

    #[must_use]
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// The neutral posture (every axis): zero, held inside each limit. Scene pairs are those
    /// apart here.
    #[must_use]
    pub fn rest(&self) -> &[f64] {
        &self.rest
    }

    /// The posture the free direction is drawn to: the measured posture at engage.
    #[must_use]
    pub fn anchor(&self) -> &[f64] {
        &self.anchor
    }

    /// Engage: forget the last commanded velocity, so the next step starts from standing
    /// still, and anchor posture at the next step's measured positions, or at the preferred
    /// posture when one is set.
    pub fn reset(&mut self) {
        self.velocity.iter_mut().for_each(|v| *v = 0.0);
        self.measured_before.fill(f64::NAN);
        self.measured_speed.fill(0.0);
        self.resting_lag.fill(0.0);
        self.anchored = self.preferred.is_some();
    }

    /// Where the next steps must leave `axis`, positions `[low, high]`, never past its joint
    /// limits; unbounded both ways frees it. Where the caps cannot reach it this tick, the step
    /// goes toward it by up to twice the acceleration cap ([`WINDOW_PULL`]). Its move is part
    /// of the step, so the collision rows weigh it: a caller that moved its positions into the
    /// window itself moved them past every pair unchecked. A window that is not an interval, or an axis out of range, is ignored.
    pub fn set_window(&mut self, axis: usize, low: f64, high: f64) {
        if let Some(slot) = self.window.get_mut(axis)
            && low <= high
        {
            *slot = [low, high];
        }
    }

    /// Where each axis was measured (`None` where it was not: a ghost has no measurement),
    /// and which joints do not follow their commands (pushing on something they cannot
    /// pass), for the next steps. Between ticks each joint is somewhere between where it was
    /// measured and where it was commanded, so pairs are kept clear at both: at the commanded
    /// positions as the step moves them, and at the measured ones for the whole motion still
    /// to come: the command's lead over the reading, opening or closing, less the lag the joint
    /// rests at (learned while it and its command stand still: a load's steady sag moves with
    /// the command), and its momentum where it closes a pair. A stuck joint is never counted
    /// on to open a pair: each pair it would help is kept by the rest alone too (exactly, for
    /// one stuck joint in the pair).
    ///
    /// # Errors
    ///
    /// [`IkError::Positions`] for vectors of the wrong length.
    pub fn set_measured(
        &mut self,
        measured: &[Option<f64>],
        stuck: &[bool],
    ) -> Result<(), IkError> {
        for given in [measured.len(), stuck.len()] {
            if given != self.measured.len() {
                return Err(IkError::Positions {
                    expected: self.measured.len(),
                    given,
                });
            }
        }
        for (slot, reading) in self.measured.iter_mut().zip(measured) {
            *slot = reading.filter(|v| v.is_finite()).unwrap_or(f64::NAN);
        }
        self.stuck.copy_from_slice(stuck);
        Ok(())
    }

    /// How fast `axis` was last measured to move, units a second: its readings differenced,
    /// smoothed over `MEASURED_SPEED_S` and never past [`FOLLOWER_SPEED`] times its speed cap
    /// (one bad reading is no speed). 0 for an axis not measured.
    #[must_use]
    pub fn measured_speed(&self, axis: usize) -> f64 {
        self.measured_speed.get(axis).copied().unwrap_or(0.0)
    }

    /// How far the last step's cost asked `axis` to move this tick, radians (units), every
    /// other solved axis where the step put it: the move that minimises the cost along it
    /// alone, its box and caps aside. An axis held still (unavailable) that its targets press
    /// on asks on that way; one they would move back asks back. 0 for an axis the solve does
    /// not move.
    #[must_use]
    pub fn asked(&self, axis: usize) -> f64 {
        let Some(column) = self.body.solved.iter().position(|&a| a == axis) else {
            return 0.0;
        };
        let w = &self.work;
        let n = self.body.solved.len();
        let row = &w.h[column * n..(column + 1) * n];
        let curvature = row[column];
        if curvature <= 0.0 || !curvature.is_finite() {
            return 0.0;
        }
        let slope: f64 = row
            .iter()
            .zip(&w.still[..n])
            .map(|(h, x)| h * x)
            .sum::<f64>()
            + w.g[column];
        w.still[column] - slope / curvature
    }

    /// The most `limb`'s tool point (a pose limb's frame origin) may move from now on, m/s;
    /// `None`: as fast as its joints' caps allow. A step asks the tool no further than this in
    /// a tick, its turn counted in (weighed by [`Settings::orientation_m_per_rad`]), and an
    /// answer whose tool point would move further by forward kinematics is shortened, the
    /// whole step at once, so a goal at a linear speed lags its target rather than catching up
    /// faster (act-sim 2026-10-31: 0.55 m/s against a 0.288 m/s goal, catching up to a tipped
    /// grasp). It gives way only where holding it would need a joint to slow past its
    /// acceleration cap, which the governor would not do either. `from` is where the tool was
    /// last commanded, when the step's positions are not (a teleoperation reference held back
    /// toward a lagging joint): the bound holds from there. A limb out of range or a speed not
    /// positive and finite clears it.
    pub fn set_tool_speed(&mut self, limb: usize, speed_m_s: Option<f64>, from: Option<[f64; 3]>) {
        if let Some(slot) = self.tool_speed.get_mut(limb) {
            *slot = speed_m_s
                .filter(|v| v.is_finite() && *v > 0.0)
                .map(|speed| (speed, from.filter(|p| p.iter().all(|v| v.is_finite()))));
        }
    }

    /// Draw the free direction toward `posture` (every axis) from now on, at
    /// [`Settings::preferred_posture_weight`], instead of toward the posture at engage: the
    /// arm moves as from its natural pose, whatever posture it engaged in.
    ///
    /// # Errors
    ///
    /// [`IkError::Positions`] for a posture of the wrong length or not finite.
    pub fn prefer_posture(&mut self, posture: &[f64]) -> Result<(), IkError> {
        self.check_positions(posture)?;
        self.anchor.copy_from_slice(posture);
        self.anchored = true;
        self.preferred = Some(posture.to_vec());
        Ok(())
    }

    fn check_positions(&self, q: &[f64]) -> Result<(), IkError> {
        let axes = self.body.model.axes();
        if q.len() != axes.len() {
            return Err(IkError::Positions {
                expected: axes.len(),
                given: q.len(),
            });
        }
        if let Some(index) = q.iter().position(|v| !v.is_finite()) {
            return Err(IkError::NonFinite(axes[index].name.clone()));
        }
        Ok(())
    }

    /// The smallest signed distance between checked proxies (self, and scene pairs apart at
    /// rest) at `q`, metres, with what it is between (link names; an obstacle as
    /// `obstacle:<index>`). For tests and evaluation: it allocates.
    #[must_use]
    pub fn clearance(&self, q: &[f64]) -> (f64, String, String) {
        let body = &self.body;
        let frames = body.model.poses(q);
        let links = body.model.links();
        let mut best = (f64::INFINITY, String::new(), String::new());
        for &(i, j) in &body.proxies.pairs {
            let d = capsule_distance(body, &frames, i, j, 0.0).0;
            if d < best.0 {
                let (p, c) = (&body.proxies.capsules[i], &body.proxies.capsules[j]);
                best = (d, links[p.link].clone(), links[c.link].clone());
            }
        }
        for &(index, k) in &self.scene_pairs {
            let capsule = &body.proxies.capsules[index];
            let (a, b) = world(&frames, capsule);
            let d = segment_box(&self.scene[k], a, b).1 - capsule.radius;
            if d < best.0 {
                best = (d, links[capsule.link].clone(), format!("obstacle:{k}"));
            }
        }
        best
    }

    /// [`Ik::clearance`] without the names, split by what it is between, into `frames` (made by
    /// [`crate::Model::poses`] for this body: the caller's scratch). Never allocates, so a
    /// control thread may check each configuration it commands. `q` holds one finite position
    /// per axis.
    pub fn clearance_into(&self, q: &[f64], frames: &mut Frames) -> Clearance {
        let body = &self.body;
        body.model.poses_into(q, frames);
        let mut clearance = Clearance {
            body_m: f64::INFINITY,
            scene_m: f64::INFINITY,
        };
        for &(i, j) in &body.proxies.pairs {
            clearance.body_m = clearance
                .body_m
                .min(capsule_distance(body, frames, i, j, 0.0).0);
        }
        for &(index, k) in &self.scene_pairs {
            let capsule = &body.proxies.capsules[index];
            let (a, b) = world(frames, capsule);
            clearance.scene_m = clearance
                .scene_m
                .min(segment_box(&self.scene[k], a, b).1 - capsule.radius);
        }
        clearance
    }

    /// One tick: the next positions from `q` toward `targets` (one per limb, in
    /// [`Body::limbs`] order; `None` leaves the limb to posture) over `dt` seconds.
    ///
    /// `available` has one flag per axis, like `q`: an axis marked unavailable (not
    /// answering, faulted, unbound) is held where it was measured, the others solve around it,
    /// and the vectors never shrink.
    ///
    /// # Errors
    ///
    /// [`IkError`] when `q`, `targets` or `dt` is malformed. A target the body cannot reach is
    /// not an error: it is reported in the step.
    #[allow(clippy::too_many_lines)] // one match arm per task and target
    pub fn step(
        &mut self,
        q: &[f64],
        available: &[bool],
        targets: &[Option<Target>],
        dt: f64,
    ) -> Result<&Step, IkError> {
        self.check_positions(q)?;
        if available.len() != q.len() {
            return Err(IkError::Available {
                expected: q.len(),
                given: available.len(),
            });
        }
        if !(dt.is_finite() && dt > 0.0) {
            return Err(IkError::Tick(dt));
        }
        if targets.len() != self.body.limbs.len() {
            return Err(IkError::Targets {
                expected: self.body.limbs.len(),
                given: targets.len(),
            });
        }
        for (limb, target) in self.body.limbs.iter().zip(targets) {
            let fits = match (limb.task, target) {
                (_, None) => true,
                (
                    Task::Pose,
                    Some(Target::Pose {
                        translation_m,
                        rotation_xyzw,
                    }),
                ) => {
                    translation_m
                        .iter()
                        .chain(rotation_xyzw)
                        .all(|v| v.is_finite())
                        && rotation_xyzw.iter().map(|v| v * v).sum::<f64>() > 1e-12
                }
                (Task::LookAt { .. }, Some(Target::LookAt { point_m })) => {
                    point_m.iter().all(|v| v.is_finite())
                }
                (
                    Task::Pose,
                    Some(Target::Axis {
                        translation_m,
                        axis,
                        toward,
                    }),
                ) => {
                    translation_m
                        .iter()
                        .chain(axis)
                        .chain(toward)
                        .all(|v| v.is_finite())
                        && norm(*axis) > 1e-6
                        && norm(*toward) > 1e-6
                }
                _ => false,
            };
            if !fits {
                return Err(IkError::Target(limb.name.clone()));
            }
        }
        let deadline = deadline_after(self.settings.deadline);
        if !self.anchored {
            self.anchor.copy_from_slice(q);
            self.anchored = true;
        }
        self.body.model.poses_into(q, &mut self.work.frames);
        self.cost(q, targets, dt);
        self.bounds(q, dt);
        for (column, &axis) in self.body.solved.iter().enumerate() {
            if !available[axis] {
                let w = &mut self.work;
                (w.lower[column], w.upper[column]) = (0.0, 0.0);
                (w.safe_lower[column], w.safe_upper[column]) = (0.0, 0.0);
                self.velocity[column] = 0.0;
            }
        }
        let w = &mut self.work;
        for ((seen, at), measured) in w.seen_q.iter_mut().zip(q).zip(&self.measured) {
            *seen = if measured.is_finite() { *measured } else { *at };
        }
        // How fast each axis is measured to move, smoothed over MEASURED_SPEED_S (a reading's
        // jitter differenced over a 2 ms tick is a speed of a radian a second).
        let blend = 1.0 - (-dt / MEASURED_SPEED_S).exp();
        for (((speed, before), now), cap) in self
            .measured_speed
            .iter_mut()
            .zip(self.measured_before.iter_mut())
            .zip(&self.measured)
            .zip(&self.caps.speed)
        {
            if now.is_finite() && before.is_finite() {
                // Never faster than the joint can move: a reading that caught up after a stale
                // run, or a glitch the caller let through, is not a speed.
                let most = FOLLOWER_SPEED * cap;
                let seen = ((now - *before) / dt).clamp(-most, most);
                *speed += blend * (seen - *speed);
            } else {
                *speed = 0.0;
            }
            *before = *now;
        }
        // The lag each joint rests at: its command's lead over its reading while both stand
        // still. Held while either moves; a joint not measured rests at its command.
        let blend = 1.0 - (-dt / RESTING_S).exp();
        for (column, &axis) in self.body.solved.iter().enumerate() {
            let lag = &mut self.resting_lag[axis];
            if !self.measured[axis].is_finite() {
                *lag = 0.0;
            } else if self.measured_speed[axis].abs() <= RESTING_SPEED
                && self.velocity[column].abs() <= RESTING_SPEED
            {
                *lag += blend * (q[axis] - w.seen_q[axis] - *lag);
            }
        }
        self.body.model.poses_into(&w.seen_q, &mut w.seen);
        self.collision_rows(q, dt);
        let mut solve = self.keep_feasible(false);
        let mut iterations = self.solve_qp(dt, deadline, false);
        // No step inside the caps keeps every row: the rows give way, as little as they can,
        // before any cap does. Widening every row's axes to their safe range first lurched the
        // torso to 20x its acceleration cap and the left shoulder 155x, for no clearance
        // (embodiments#208).
        if iterations.is_none() && self.work.used > 0 && deadline.is_none_or(|d| Instant::now() < d)
        {
            iterations = self.solve_qp(dt, deadline, true);
            // Rows gave way: say so, as a widened cap does.
            if iterations.is_some() && self.qp.x[self.body.solved.len()] > SLACK_USED {
                solve = Solve::Relaxed;
            }
        }
        if iterations.is_none() && self.work.used > 0 && deadline.is_none_or(|d| Instant::now() < d)
        {
            solve = self.keep_feasible(true);
            iterations = self.solve_qp(dt, deadline, false);
        }
        let w = &mut self.work;
        let n = self.body.solved.len();
        let solved = iterations.is_some();
        // The answer, held to its box; on failure, standing still as nearly as the caps allow.
        for j in 0..n {
            let x = if solved { w.answer[j] } else { w.still[j] };
            w.still[j] = x.clamp(w.lower[j], w.upper[j]);
        }
        if !solved {
            solve = Solve::Held;
        }
        self.bound_tool_speed(q, dt);
        let w = &mut self.work;
        let dq = &w.still;
        self.step.q.copy_from_slice(q);
        self.step.qdot.iter_mut().for_each(|v| *v = 0.0);
        for (column, &axis) in self.body.solved.iter().enumerate() {
            self.step.q[axis] = q[axis] + dq[column];
            self.velocity[column] = dq[column] / dt;
            self.step.qdot[axis] = self.velocity[column];
        }
        self.step.solve = solve;
        self.step.iterations = iterations.unwrap_or(0);
        self.report(targets);
        Ok(&self.step)
    }

    /// `1/2 dq' H dq + g' dq`: tool position, orientation (or pointing), the step itself
    /// (damping), then posture toward the anchor.
    #[allow(clippy::too_many_lines)] // one match arm per task and target
    fn cost(&mut self, q: &[f64], targets: &[Option<Target>], dt: f64) {
        let body = &self.body;
        let s = &self.settings;
        let w = &mut self.work;
        let n = body.solved.len();
        w.h.iter_mut().for_each(|v| *v = 0.0);
        w.g.iter_mut().for_each(|v| *v = 0.0);
        let rho2 = s.orientation_m_per_rad * s.orientation_m_per_rad;
        for (index, (limb, target)) in body.limbs.iter().zip(targets).enumerate() {
            let Some(target) = target else { continue };
            // No more of the error than the tool may move this tick: its point and its turn
            // together, the turn weighed as the solve weighs it (orientation_m_per_rad).
            let most = self.tool_speed[index].map(|(speed, _)| speed * dt);
            let pose = w.frames.links[limb.frame];
            for &axis in &body.solved {
                w.jv[axis] = [0.0; 3];
                w.jw[axis] = [0.0; 3];
            }
            body.model
                .point_jacobian(&w.frames, limb.frame, pose.t, &mut w.jv);
            body.model
                .angular_jacobian(&w.frames, limb.frame, &mut w.jw);
            match (limb.task, target) {
                (
                    Task::Pose,
                    Target::Pose {
                        translation_m,
                        rotation_xyzw,
                    },
                ) => {
                    // Each error is closed no faster than the limb can still stop on it (as a
                    // collision row's approach is): asking for the whole error in one tick
                    // overshoots against the acceleration caps, and the overshoot wanders
                    // the arm's free direction tick after tick.
                    let e_p = sub(*translation_m, pose.t);
                    let goal = from_quaternion(*rotation_xyzw);
                    let e_r = log(&mat_mul(&goal, &transpose(&pose.r)));
                    let share = approach(e_p, e_r, &w.jv, &w.jw, &body.solved, &self.caps, s, dt);
                    let (e_p, e_r) =
                        within_tool_speed(scale(e_p, share), scale(e_r, share), most, s);
                    for k in 0..3 {
                        fill(&mut w.task, &body.solved, |axis| w.jv[axis][k]);
                        least_squares(&mut w.h, &mut w.g, &w.task, e_p[k], 1.0);
                        fill(&mut w.task, &body.solved, |axis| w.jw[axis][k]);
                        least_squares(&mut w.h, &mut w.g, &w.task, e_r[k], rho2);
                    }
                }
                (
                    Task::Pose,
                    Target::Axis {
                        translation_m,
                        axis,
                        toward,
                    },
                ) => {
                    // The axis's pointing error, closed as a pose's is; the roll about the
                    // axis is left out of the rows altogether, so it floats.
                    let a = mat_vec(&pose.r, scale(*axis, 1.0 / norm(*axis)));
                    let u = scale(*toward, 1.0 / norm(*toward));
                    let e_p = sub(*translation_m, pose.t);
                    let e_r = pointing_error(a, u);
                    let share = approach(e_p, e_r, &w.jv, &w.jw, &body.solved, &self.caps, s, dt);
                    let (e_p, e_r) =
                        within_tool_speed(scale(e_p, share), scale(e_r, share), most, s);
                    for k in 0..3 {
                        fill(&mut w.task, &body.solved, |axis| w.jv[axis][k]);
                        least_squares(&mut w.h, &mut w.g, &w.task, e_p[k], 1.0);
                    }
                    for across in across(a) {
                        fill(&mut w.task, &body.solved, |axis| dot(across, w.jw[axis]));
                        least_squares(&mut w.h, &mut w.g, &w.task, dot(across, e_r), rho2);
                    }
                }
                (Task::LookAt { forward }, Target::LookAt { point_m }) => {
                    let mask = &self.look_columns[index];
                    let f = mat_vec(&pose.r, forward);
                    let d = sub(*point_m, pose.t);
                    let distance = norm(d);
                    if distance < 1e-3 {
                        continue;
                    }
                    let u = scale(d, 1.0 / distance);
                    let e = pointing_error(f, u);
                    // Turn f toward u relative to u's own turn as the camera moves: the
                    // direction to the point turns at -(u x v) / distance, so the relative
                    // angular rate is w + (u x v) / distance; ask it for the error's rotation
                    // (its component along f, a roll, for none).
                    for (column, &axis) in body.solved.iter().enumerate() {
                        w.jp[axis] = if mask[column] {
                            let turn = cross(u, w.jv[axis]);
                            [0, 1, 2].map(|k| w.jw[axis][k] + turn[k] / distance)
                        } else {
                            [0.0; 3]
                        };
                    }
                    // Closed no faster than the neck can still stop on it, as a pose error
                    // is: asking for the whole turn at once overshot the acceleration caps
                    // and the camera rang on every big swing (three bounces engaging).
                    let share = pointing_approach(e, &w.jp, &body.solved, &self.caps, s, rho2, dt);
                    let e = clamp_norm(scale(e, share), s.max_error_rad);
                    for k in 0..3 {
                        fill(&mut w.task, &body.solved, |axis| w.jp[axis][k]);
                        least_squares(&mut w.h, &mut w.g, &w.task, e[k], rho2);
                    }
                }
                _ => {}
            }
        }
        for (column, &axis) in body.solved.iter().enumerate() {
            let weight = if self.preferred.is_some() {
                s.preferred_posture_weight
            } else if self.shared[column] {
                s.shared_posture_weight
            } else {
                s.posture_weight
            };
            let error = self.anchor[axis] - q[axis];
            w.h[column * n + column] += 2.0 * (weight + s.damping);
            w.g[column] -= 2.0 * weight * error;
        }
    }

    /// Per column: joint limits with braking, speed, then acceleration where it fits.
    fn bounds(&mut self, q: &[f64], dt: f64) {
        let body = &self.body;
        let axes = body.model.axes();
        let s = &self.settings;
        let w = &mut self.work;
        for (column, &axis) in body.solved.iter().enumerate() {
            let (speed, accel) = (self.caps.speed[axis], self.caps.accel[axis]);
            let step = (speed * dt).min(s.max_step_rad);
            let (mut lo, mut lo_by) = (-step, Bound::Speed);
            let (mut hi, mut hi_by) = (step, Bound::Speed);
            if let Some([low_limit, high_limit]) = axes[axis].limits {
                let margin = s.limit_margin_rad.min((high_limit - low_limit) / 2.0);
                let room_up = (high_limit - margin - q[axis]).max(0.0);
                let room_down = (q[axis] - low_limit - margin).max(0.0);
                let up = braking(room_up, accel, dt);
                let down = braking(room_down, accel, dt);
                if up < hi {
                    (hi, hi_by) = (up, Bound::Limit);
                }
                if -down > lo {
                    (lo, lo_by) = (-down, Bound::Limit);
                }
            }
            // The window, within the joint's limits (with their margin): never past them.
            let [mut low, mut high] = self.window[axis];
            if let Some([low_limit, high_limit]) = axes[axis].limits
                && low.is_finite()
                && high.is_finite()
            {
                let margin = s.limit_margin_rad.min((high_limit - low_limit) / 2.0);
                let (inside_low, inside_high) = (low_limit + margin, high_limit - margin);
                low = low.clamp(inside_low, inside_high);
                high = high.clamp(inside_low, inside_high);
            }
            let window = (low - q[axis], high - q[axis]);
            // Toward a window's edge as toward a limit: braking, so the step comes to rest on
            // it rather than overshoot it and be pulled back (a pushing elbow circled the gap
            // and never stalled, embodiments#208); applied after the acceleration cap, which it
            // never overrules (a jittered edge moved the braking step 3x the cap at 500 Hz).
            let edge_hi = if window.1.is_finite() && window.1 > 0.0 {
                braking(window.1, accel, dt)
            } else {
                f64::INFINITY
            };
            let edge_lo = if window.0.is_finite() && window.0 < 0.0 {
                -braking(-window.0, accel, dt)
            } else {
                f64::NEG_INFINITY
            };
            let (mut safe_lo, mut safe_hi) = (lo, hi);
            if window.1 >= safe_lo && window.0 <= safe_hi {
                within(&mut safe_lo, &mut safe_hi, window);
            }
            w.safe_lower[column] = safe_lo;
            w.safe_upper[column] = safe_hi;
            let previous = self.velocity[column] * dt;
            let swing = accel * dt * dt;
            let (a_lo, a_hi) = (previous - swing, previous + swing);
            if a_lo <= hi && a_hi >= lo {
                if a_lo > lo {
                    (lo, lo_by) = (a_lo, Bound::Accel);
                }
                if a_hi < hi {
                    (hi, hi_by) = (a_hi, Bound::Accel);
                }
            } else if a_hi < lo {
                (hi, hi_by) = (lo, Bound::Accel);
            } else {
                (lo, lo_by) = (hi, Bound::Accel);
            }
            // Where the step must leave it ([`Ik::set_window`]).
            if edge_hi < hi {
                (hi, hi_by) = (edge_hi.max(lo), Bound::Limit);
            }
            if edge_lo > lo {
                (lo, lo_by) = (edge_lo.min(hi), Bound::Limit);
            }
            if window.1 < lo || window.0 > hi {
                // The caps cannot reach the window this tick: toward it by at most twice the
                // acceleration cap, never past the speed and limit box. A window that won
                // outright chattered a stopped elbow's command after its measurement's ring
                // and jitter at 13x its cap (embodiments#208).
                let extra = (WINDOW_PULL - 1.0) * swing;
                if window.1 < lo {
                    lo = (lo - extra).max(window.1).max(safe_lo.min(lo));
                    hi = lo;
                } else {
                    hi = (hi + extra).min(window.0).min(safe_hi.max(hi));
                    lo = hi;
                }
                (lo_by, hi_by) = (Bound::Accel, Bound::Accel);
            } else if within(&mut lo, &mut hi, window) {
                (lo_by, hi_by) = (Bound::Limit, Bound::Limit);
            }
            w.lower[column] = lo;
            w.upper[column] = hi;
            w.lower_by[column] = lo_by;
            w.upper_by[column] = hi_by;
            w.yielded[column] = false;
        }
    }

    /// Every pair within influence this tick at the commanded positions, or at the measured
    /// ones (`measured`), each measured once: capsules, refined between the links' convex
    /// pieces where they touch, and capsules against the scene.
    fn collision_candidates(&mut self, measured: bool) {
        let body = &self.body;
        let s = &self.settings;
        let w = &mut self.work;
        let frames = if measured { &w.seen } else { &w.frames };
        for &(i, j) in &body.proxies.pairs {
            let links = (body.proxies.capsules[i].link, body.proxies.capsules[j].link);
            // Two capsules of a link pair in contact are refined once: one row for the parts.
            let refined_already = w.candidates.iter().any(|c| match c.pair {
                Pair::Capsules { i: a, j: b, .. } => {
                    c.refined
                        && c.measured == measured
                        && (body.proxies.capsules[a].link, body.proxies.capsules[b].link) == links
                }
                Pair::Scene(..) => false,
            });
            let refine_below = if refined_already {
                f64::NEG_INFINITY
            } else {
                0.0
            };
            let (distance, x, y, direction, refined) =
                capsule_distance(body, frames, i, j, refine_below);
            if refined_already && distance < 0.0 {
                continue;
            }
            if distance < s.influence_m {
                w.candidates.push(Candidate {
                    distance,
                    pair: Pair::Capsules {
                        i,
                        j,
                        x,
                        y,
                        direction,
                    },
                    refined,
                    measured,
                });
            }
        }
        for &(capsule, k) in &self.scene_pairs {
            let c = &body.proxies.capsules[capsule];
            let (a, b) = world(frames, c);
            let distance = segment_box(&self.scene[k], a, b).1 - c.radius;
            if distance < s.influence_m {
                w.candidates.push(Candidate {
                    distance,
                    pair: Pair::Scene(capsule, k),
                    refined: false,
                    measured,
                });
            }
        }
    }

    /// The closest pairs within influence, each as `row . dq <= rhs`: the pair closes no
    /// faster this tick than a braking share of its axes' accelerations can still stop
    /// before the margin.
    #[allow(clippy::too_many_lines)] // one pass over the pairs at both poses, in one place
    fn collision_rows(&mut self, q: &[f64], dt: f64) {
        self.work.candidates.clear();
        self.collision_candidates(false);
        // Kept at the body as measured too, where it is not where it was commanded.
        let apart = self
            .work
            .seen_q
            .iter()
            .zip(q)
            .any(|(seen, at)| (seen - at).abs() > 1e-6);
        if apart {
            self.collision_candidates(true);
        }
        let body = &self.body;
        let s = &self.settings;
        let w = &mut self.work;
        let n = body.solved.len();
        w.candidates
            .sort_unstable_by(|x, y| x.distance.total_cmp(&y.distance));
        w.used = 0;
        for index in 0..w.candidates.len() {
            if w.used == s.max_rows {
                break;
            }
            let candidate = w.candidates[index];
            let slot = w.used;
            let row = &mut w.rows[slot * n..(slot + 1) * n];
            row.fill(0.0);
            let frames = if candidate.measured {
                &w.seen
            } else {
                &w.frames
            };
            let (distance, active) = match candidate.pair {
                Pair::Capsules {
                    i,
                    j,
                    x,
                    y,
                    direction,
                } => {
                    let d = candidate.distance;
                    let (p, c) = (&body.proxies.capsules[i], &body.proxies.capsules[j]);
                    point_row(body, frames, &mut w.jp, p.link, x, direction, -1.0, row);
                    point_row(body, frames, &mut w.jp, c.link, y, direction, 1.0, row);
                    (
                        d,
                        Active::SelfCollision {
                            a: p.link,
                            b: c.link,
                        },
                    )
                }
                Pair::Scene(capsule, k) => {
                    let c = &body.proxies.capsules[capsule];
                    let (a, b) = world(frames, c);
                    let (point, d, direction) = segment_box(&self.scene[k], a, b);
                    point_row(body, frames, &mut w.jp, c.link, point, direction, -1.0, row);
                    (
                        d - c.radius,
                        Active::Obstacle {
                            link: c.link,
                            obstacle: k,
                        },
                    )
                }
            };
            // row . dq is how far the pair closes this tick.
            let capability: f64 = body
                .solved
                .iter()
                .zip(row.iter())
                .map(|(&axis, r)| r.abs() * self.caps.accel[axis])
                .sum();
            if capability < 1e-12 {
                continue;
            }
            // Never more than half the room in one tick either: a long step is where the
            // linearisation is least true.
            let room = (distance - s.collision_margin_m).max(0.0);
            let mut rhs = braking(room, s.braking_share * capability, dt).min(0.5 * room);
            if candidate.measured {
                // From where it was measured the body still makes its way to its command, then
                // the step: all of it within half the room, as the commanded row's tick is. One
                // linearisation spans that whole motion: with the whole room, an act-sim-like PD
                // (zeta 0.2) rang 40-60 mm into the other arm; with half, it touches at worst
                // (1.5 mm in; embodiments#206). The pace of closing is the commanded row's to keep.
                //
                // The way to the command is counted as it is, opening or closing, less the lag
                // the joint rests at (no motion still to come: it moves with its command), so a
                // step this row pays for is paid for the next tick too. Counted only where it
                // closed the pair, a lagging joint's opening was never credited while its step
                // was: the solve "opened" the pair by moving joints already behind the arm, which
                // moved nothing that was there, and the row asked again every tick. Near the
                // counter the command swung 30-80 mm at 1-2 Hz for as long as it was held, and a
                // 0.23 m move stalled 0.05 rad short (act-sim, 2026-11-03). Where its measured
                // speed carries it (a follower rings past its command) is counted only where it
                // closes the pair: an underdamped follower otherwise came within 3-7 mm.
                let lead = body
                    .solved
                    .iter()
                    .zip(row.iter())
                    .filter(|&(&axis, _)| !self.stuck[axis])
                    .map(|(&axis, r)| {
                        let ahead = q[axis] - w.seen_q[axis];
                        // Never more of it than the joint lags now, nor the other way.
                        let resting = self.resting_lag[axis].clamp(ahead.min(0.0) - 2.0 * READING_JITTER, ahead.max(0.0) + 2.0 * READING_JITTER);
                        let carried = MOMENTUM_S * self.measured_speed[axis];
                        r * (ahead - resting) + (r * carried).max(0.0)
                    })
                    .sum::<f64>();
                rhs = 0.5 * room - lead;
            }
            if candidate.measured
                && body.solved.iter().zip(row.iter()).all(|(&axis, r)| {
                    *r == 0.0 || (q[axis] - w.seen_q[axis]).abs() <= READING_JITTER
                })
            {
                continue; // measured where it was commanded, as far as this pair goes
            }
            let reach: f64 = (0..n)
                .map(|j| (row[j] * w.lower[j]).max(row[j] * w.upper[j]))
                .sum();
            if reach <= rhs {
                continue; // no step inside the box can close it that far
            }
            w.rhs[slot] = rhs;
            w.row_active[slot] = active;
            w.row_measured[slot] = candidate.measured;
            w.used += 1;
            // The same pair kept without the stuck joints: a step must keep it whether or not
            // they make their move (exactly, for one stuck joint in the pair).
            let untrusted = |j: usize| self.stuck[body.solved[j]];
            if w.used < s.max_rows && (0..n).any(|j| w.rows[slot * n + j] != 0.0 && untrusted(j)) {
                let (kept, spare) = w.rows.split_at_mut((slot + 1) * n);
                let next = &mut spare[..n];
                next.copy_from_slice(&kept[slot * n..]);
                for (j, value) in next.iter_mut().enumerate() {
                    if untrusted(j) {
                        *value = 0.0;
                    }
                }
                w.rhs[slot + 1] = rhs;
                w.row_active[slot + 1] = active;
                w.row_measured[slot + 1] = true;
                w.used += 1;
            }
        }
    }

    /// Ready the rows for the solve. A commanded row no step within the acceleration caps can
    /// keep is left to the elastic retry ([`Ik::solve_qp`]), which keeps the rows as nearly as
    /// the caps allow. With `all` (that retry failed too), every row's axes yield to their safe
    /// range and every row asks only that its pair come no closer, which standing still keeps.
    ///
    /// A row main never had (at the measured pose, or a stuck joint's copy) never widens a cap:
    /// one the box (as the commanded rows left it) cannot keep is eased [`SOFTENED`] of the way
    /// from the step that opens its pair most. A follower ringing past its command,
    /// or sagging under a load, reads a pair closer than the command will bring it; a cap
    /// waived for that lurched the command away and the follower rang back in, 40-60 mm deep
    /// (embodiments#206: a PD at zeta 0.2), or backed a sagging elbow 0.13 rad off its stop.
    fn keep_feasible(&mut self, all: bool) -> Solve {
        let n = self.body.solved.len();
        let w = &mut self.work;
        let mut solve = Solve::Solved;
        let least = |w: &Workspace, slot: usize| -> f64 {
            let row = &w.rows[slot * n..(slot + 1) * n];
            (0..n)
                .map(|j| (row[j] * w.lower[j]).min(row[j] * w.upper[j]))
                .sum()
        };
        for slot in 0..w.used {
            if w.row_measured[slot] {
                continue;
            }
            // A row no step inside the caps keeps widens nothing here: the solve then finds no
            // step and the elastic retry keeps the rows as nearly as the caps allow. Widening its
            // axes to their whole range let one bad reading turn an arm 70-340x its cap
            // (embodiments#210). Only `all`, after the retry too has failed, widens.
            if all {
                for j in 0..n {
                    if w.rows[slot * n + j] != 0.0 && !w.yielded[j] {
                        w.lower[j] = w.safe_lower[j];
                        w.upper[j] = w.safe_upper[j];
                        w.yielded[j] = true;
                    }
                }
                w.rhs[slot] = w.rhs[slot].max(0.0);
                solve = Solve::Relaxed;
            }
        }
        for slot in 0..w.used {
            if w.row_measured[slot] {
                if all {
                    w.rhs[slot] = w.rhs[slot].max(0.0);
                }
                let low = least(w, slot);
                if low > w.rhs[slot] {
                    let row = &w.rows[slot * n..(slot + 1) * n];
                    let most: f64 = (0..n)
                        .map(|j| (row[j] * w.lower[j]).max(row[j] * w.upper[j]))
                        .sum();
                    w.rhs[slot] = low + SOFTENED * (most - low);
                }
            }
        }
        // Standing still as nearly as the caps allow, for a step that fails: from the caps, not
        // from a box `all` widened (a held torso stopped dead at 5.4x its cap, #211 refute).
        if !all {
            for j in 0..n {
                w.still[j] = 0.0_f64.clamp(w.lower[j], w.upper[j]);
            }
        }
        solve
    }

    /// Load the QP and solve; the iteration count, or `None` on failure.
    ///
    /// The QP sees `dq` in units of each axis's one-tick speed step (its box is then about
    /// [-1, 1]), unit-norm collision rows, and a cost scaled to a unit diagonal: the same
    /// problem, conditioned for an interior-point method whose data is otherwise 1e-4 m.
    fn solve_qp(&mut self, dt: f64, deadline: Option<Instant>, elastic: bool) -> Option<u32> {
        let n = self.body.solved.len();
        let m = n + 1;
        let w = &mut self.work;
        let qp = &mut self.qp;
        for (column, &axis) in self.body.solved.iter().enumerate() {
            w.unit[column] = (self.caps.speed[axis] * dt).min(self.settings.max_step_rad);
        }
        let mut largest = 0.0_f64;
        for j in 0..n {
            largest = largest.max(w.h[j * n + j] * w.unit[j] * w.unit[j]);
        }
        let c = if largest > 0.0 { 1.0 / largest } else { 1.0 };
        for i in 0..n {
            for j in 0..n {
                qp.h[i * m + j] = c * w.h[i * n + j] * w.unit[i] * w.unit[j];
            }
            qp.h[i * m + n] = 0.0;
            qp.h[n * m + i] = 0.0;
            qp.g[i] = c * w.g[i] * w.unit[i];
            // A box closed to a point (a held axis, a forced deceleration) keeps a sliver of
            // interior for the interior-point method; the answer is clamped back to it.
            let (lo, hi) = (w.lower[i] / w.unit[i], w.upper[i] / w.unit[i]);
            let open = (1e-7 - (hi - lo)).max(0.0) / 2.0;
            qp.lower[i] = lo - open;
            qp.upper[i] = hi + open;
        }
        // The slack every row may borrow, at a cost no step's tracking outweighs: closed (a
        // sliver) but on the retry after a solve with no step inside the caps that keeps every
        // row, where it finds the step that keeps them as nearly as the caps allow.
        qp.h[n * m + n] = if elastic { SLACK_WEIGHT } else { 1.0 };
        qp.g[n] = 0.0;
        qp.lower[n] = -5e-8;
        qp.upper[n] = if elastic { SLACK_ROOM } else { 5e-8 };
        qp.used = w.used;
        for slot in 0..w.used {
            let mut length = 0.0;
            for j in 0..n {
                let v = w.rows[slot * n + j] * w.unit[j];
                qp.r[slot * m + j] = v;
                length += v * v;
            }
            let scale = if length > 0.0 {
                1.0 / length.sqrt()
            } else {
                0.0
            };
            for j in 0..n {
                qp.r[slot * m + j] *= scale;
            }
            qp.r[slot * m + n] = -1.0;
            qp.rhs[slot] = w.rhs[slot] * scale;
        }
        let iterations = qp.solve(self.settings.max_iterations, deadline).ok()?;
        for j in 0..n {
            w.answer[j] = qp.x[j] * w.unit[j];
        }
        Some(iterations)
    }

    /// Shorten the whole step until every speed-bound tool point moves no further than its
    /// speed allows this tick, by forward kinematics of the answer (not the linearisation),
    /// from where the tool was last commanded; or from where the step starts, when the
    /// reference was held back further than a tick's motion from there (that pull is the
    /// reference's, toward the measured arm, not the step's). The whole step is scaled, every
    /// limb and the torso alike: scaled toward zero it stays inside every speed, limit and
    /// collision bound that standing still satisfies, which one limb's columns scaled alone do
    /// not (a row its other columns hold breaks, and a shared torso rescales the other hand).
    /// Never below the scale at which a joint would slow past its acceleration cap: the
    /// governor would only slew such a step back, unchecked. The cost already asks no tool for
    /// more than its speed, so that floor binds only for a tick of a sharp slow-down.
    fn bound_tool_speed(&mut self, q: &[f64], dt: f64) {
        self.tool_capped.fill(false);
        if self.tool_speed.iter().all(Option::is_none) {
            return;
        }
        let n = self.body.solved.len();
        let w = &mut self.work;
        // The least scale each column's box (speed, limit, acceleration) still holds: any
        // scale down to zero where the box holds standing still, else up to its edge.
        let mut floor = 0.0_f64;
        for column in 0..n {
            let (x, lower, upper) = (w.still[column], w.lower[column], w.upper[column]);
            if x > 0.0 && lower > 0.0 {
                floor = floor.max(lower / x);
            } else if x < 0.0 && upper < 0.0 {
                floor = floor.max(upper / x);
            }
        }
        let floor = floor.min(1.0);
        let mut scale = 1.0_f64;
        self.step.q.copy_from_slice(q);
        // FK is nearly linear over one tick's step: a second shortening lands within a hair of
        // the bound, and a third (rare) takes the rest.
        for _ in 0..3 {
            for (column, &axis) in self.body.solved.iter().enumerate() {
                self.step.q[axis] = scale.mul_add(w.still[column], q[axis]);
            }
            self.body.model.poses_into(&self.step.q, &mut w.after);
            let mut factor = 1.0_f64;
            for (index, limb) in self.body.limbs.iter().enumerate() {
                let Some((speed, from)) = self.tool_speed[index] else {
                    continue;
                };
                if !matches!(limb.task, Task::Pose) {
                    continue;
                }
                let most = speed * dt;
                let start = w.frames.links[limb.frame].t;
                let from = from
                    .filter(|from| norm(sub(*from, start)) <= most)
                    .unwrap_or(start);
                let moved = norm(sub(w.after.links[limb.frame].t, from));
                if moved > most * (1.0 + 1e-6) {
                    factor = factor.min(most / moved);
                    self.tool_capped[index] = true;
                }
            }
            let next = (scale * factor).max(floor);
            if factor >= 1.0 || next >= scale {
                break;
            }
            scale = next;
        }
        if scale < 1.0 {
            for value in &mut w.still[..n] {
                *value *= scale;
            }
        }
    }

    /// Residuals, binding constraints and reach per limb, at the new positions.
    #[allow(clippy::too_many_lines)] // one residual per task and target, then what binds
    fn report(&mut self, targets: &[Option<Target>]) {
        let body = &self.body;
        let s = &self.settings;
        let w = &mut self.work;
        let n = body.solved.len();
        let dq = &w.still;
        body.model.poses_into(&self.step.q, &mut w.after);
        for column in 0..n {
            let width = (w.upper[column] - w.lower[column]).abs().max(1e-12);
            let tolerance = 1e-4 * width + 1e-12;
            let axis = body.solved[column];
            let bound = if w.yielded[column] {
                Some(Bound::Accel)
            } else if dq[column] >= w.upper[column] - tolerance {
                Some(w.upper_by[column])
            } else if dq[column] <= w.lower[column] + tolerance {
                Some(w.lower_by[column])
            } else {
                None
            };
            w.column_active[column] = bound.map(|b| match b {
                Bound::Limit => Active::Limit { axis },
                Bound::Speed => Active::Speed { axis },
                Bound::Accel => Active::Accel { axis },
            });
        }
        for slot in 0..w.used {
            let row = &w.rows[slot * n..(slot + 1) * n];
            let value: f64 = row.iter().zip(dq).map(|(a, b)| a * b).sum();
            w.row_binds[slot] = value >= w.rhs[slot] - 1e-6 * (1e-3 + w.rhs[slot].abs());
        }
        for (index, (limb, target)) in body.limbs.iter().zip(targets).enumerate() {
            let pose = w.after.links[limb.frame];
            let (residual_m, residual_rad) = match (limb.task, target) {
                (
                    _,
                    Some(Target::Pose {
                        translation_m,
                        rotation_xyzw,
                    }),
                ) => (
                    norm(sub(*translation_m, pose.t)),
                    norm(log(&mat_mul(
                        &from_quaternion(*rotation_xyzw),
                        &transpose(&pose.r),
                    ))),
                ),
                (Task::LookAt { forward }, Some(Target::LookAt { point_m })) => {
                    let f = mat_vec(&pose.r, forward);
                    let d = sub(*point_m, pose.t);
                    let cos = dot(f, d) / norm(d).max(1e-12);
                    (0.0, cos.clamp(-1.0, 1.0).acos())
                }
                (
                    _,
                    Some(Target::Axis {
                        translation_m,
                        axis,
                        toward,
                    }),
                ) => {
                    let a = mat_vec(&pose.r, *axis);
                    let cos = dot(a, *toward) / (norm(a) * norm(*toward)).max(1e-12);
                    (
                        norm(sub(*translation_m, pose.t)),
                        cos.clamp(-1.0, 1.0).acos(),
                    )
                }
                _ => (0.0, 0.0),
            };
            let columns = &self.limb_columns[index];
            let report = &mut self.step.limbs[index];
            report.active.clear();
            for &column in columns {
                if let Some(active) = w.column_active[column] {
                    report.active.push(active);
                }
            }
            for slot in 0..w.used {
                let row = &w.rows[slot * n..(slot + 1) * n];
                if w.row_binds[slot] && columns.iter().any(|&c| row[c] != 0.0) {
                    report.active.push(w.row_active[slot]);
                }
            }
            if self.tool_capped[index] {
                report.active.push(Active::ToolSpeed { limb: index });
            }
            let blocked = report.active.iter().any(|a| {
                matches!(
                    a,
                    Active::Limit { .. } | Active::SelfCollision { .. } | Active::Obstacle { .. }
                )
            });
            let capped = report.active.iter().any(Active::slows);
            report.residual_m = residual_m;
            report.residual_rad = residual_rad;
            report.hidden_by = match (limb.task, target) {
                (Task::Pose, Some(_)) => hidden(body, &w.after, &self.sight_skip, pose.t, s),
                _ => None,
            };
            report.reach = if target.is_none() {
                Reach::Idle
            } else if residual_m <= s.tracking_m && residual_rad <= s.tracking_rad {
                Reach::Tracking
            } else if blocked || !capped {
                Reach::Unreachable
            } else {
                Reach::CatchingUp
            };
        }
    }
}

/// `[lo, hi]` cut to `window`, or, where they do not meet, to the window's nearer end: the
/// window wins. Whether it moved either end.
fn within(lo: &mut f64, hi: &mut f64, (low, high): (f64, f64)) -> bool {
    if high < *lo {
        (*lo, *hi) = (high, high);
        true
    } else if low > *hi {
        (*lo, *hi) = (low, low);
        true
    } else {
        let cut = low > *lo || high < *hi;
        *lo = lo.max(low);
        *hi = hi.min(high);
        cut
    }
}

/// The fastest per-tick step from which ticks decelerating by at most `accel` still stop
/// within `room`: `v dt + v^2 / 2a + v dt / 2 <= room`.
fn braking(room: f64, accel: f64, dt: f64) -> f64 {
    dt * accel * ((2.25 * dt * dt + 2.0 * room / accel).sqrt() - 1.5 * dt)
}

/// Signed distance between two capsules at `frames`, their closest axis points, and the unit
/// direction from the second to the first.
/// Two capsules' signed distance, a closest point on each axis, the direction from the second
/// to the first, and whether the pieces measured it. Capsules closer than `refine_below` (in
/// contact, at 0) are measured again between their links' convex pieces ([`crate::hull`]),
/// which follow the parts where a capsule bulges past them: a separation the pieces prove
/// stands for the pair, its closest points on the parts.
fn capsule_distance(
    body: &Body,
    frames: &Frames,
    i: usize,
    j: usize,
    refine_below: f64,
) -> (f64, V3, V3, V3, bool) {
    let (p, c) = (&body.proxies.capsules[i], &body.proxies.capsules[j]);
    let (pa, pb) = world(frames, p);
    let (qa, qb) = world(frames, c);
    let (x, y) = closest_segments(pa, pb, qa, qb);
    let gap = sub(x, y);
    let centres = norm(gap);
    let direction = if centres > 1e-12 {
        scale(gap, 1.0 / centres)
    } else {
        [0.0, 0.0, 1.0]
    };
    let capsules = (centres - p.radius - c.radius, x, y, direction, false);
    if capsules.0 >= refine_below {
        return capsules;
    }
    let proxies = &body.proxies;
    match crate::hull::link_distance(
        &proxies.pieces,
        &proxies.pieces_of[p.link],
        &frames.links[p.link],
        &proxies.pieces_of[c.link],
        &frames.links[c.link],
    ) {
        Some((d, on_p, on_c)) if d > capsules.0 => {
            let gap = sub(on_p, on_c);
            (d, on_p, on_c, scale(gap, 1.0 / norm(gap)), true)
        }
        _ => capsules,
    }
}

/// Add `sign * direction . J(point)` over the solved columns to `row`.
#[allow(clippy::too_many_arguments)]
fn point_row(
    body: &Body,
    frames: &Frames,
    scratch: &mut [V3],
    link: usize,
    point: V3,
    direction: V3,
    sign: f64,
    row: &mut [f64],
) {
    let above = body.model.above(link);
    for term in above {
        scratch[term.axis] = [0.0; 3];
    }
    body.model.point_jacobian(frames, link, point, scratch);
    for term in above {
        let j = scratch[term.axis];
        if j.iter().all(|v| *v == 0.0) {
            continue; // a shared axis already taken
        }
        if let Ok(column) = body.solved.binary_search(&term.axis) {
            row[column] += sign * dot(direction, j);
        }
        scratch[term.axis] = [0.0; 3];
    }
}

/// What hides `tool` from the body's first camera at `frames`: the proxy the line of sight
/// passes through, short of its last [`Settings::sight_tool_m`] (the tool itself). Proxies
/// on the camera's own neck (`skip`) never stand between it and what it sees.
fn hidden(body: &Body, frames: &Frames, skip: &[bool], tool: V3, s: &Settings) -> Option<usize> {
    let camera = body
        .limbs
        .iter()
        .find(|l| matches!(l.task, Task::LookAt { .. }))?;
    let eye = frames.links[camera.frame].t;
    let line = sub(tool, eye);
    let length = norm(line);
    if length <= s.sight_tool_m {
        return None;
    }
    let end = [0, 1, 2].map(|k| eye[k] + line[k] * (1.0 - s.sight_tool_m / length));
    let mut best: Option<(f64, usize)> = None;
    for (capsule, skipped) in body.proxies.capsules.iter().zip(skip) {
        if *skipped {
            continue;
        }
        let (a, b) = world(frames, capsule);
        let (x, y) = closest_segments(eye, end, a, b);
        let gap = norm(sub(x, y)) - capsule.radius;
        if gap < 0.0 && best.is_none_or(|(nearest, _)| gap < nearest) {
            best = Some((gap, capsule.link));
        }
    }
    best.map(|(_, link)| link)
}

fn fill(out: &mut [f64], solved: &[usize], value: impl Fn(usize) -> f64) {
    for (slot, &axis) in out.iter_mut().zip(solved) {
        *slot = value(axis);
    }
}

/// Add `weight * (row . x - target)^2` to `1/2 x'Hx + g'x`.
fn least_squares(h: &mut [f64], g: &mut [f64], row: &[f64], target: f64, weight: f64) {
    let n = g.len();
    for i in 0..n {
        if row[i] == 0.0 {
            continue;
        }
        g[i] -= 2.0 * weight * row[i] * target;
        for j in 0..n {
            h[i * n + j] += 2.0 * weight * row[i] * row[j];
        }
    }
}

/// The rotation turning unit `f` onto unit `u`: about `f x u`, by the whole angle between
/// them. The bare cross product vanishes at 180 degrees too, so a frame facing straight away
/// from its target never turned (a neck folded behind the hands on the bench); there, any
/// axis across `f` turns it.
fn pointing_error(f: V3, u: V3) -> V3 {
    let sine = cross(f, u);
    let (s_norm, cosine) = (norm(sine), dot(f, u));
    let angle = s_norm.atan2(cosine);
    if s_norm > 1e-9 {
        scale(sine, angle / s_norm)
    } else if cosine < 0.0 {
        let [axis, _] = across(f);
        scale(axis, angle)
    } else {
        [0.0; 3]
    }
}

/// Two unit vectors across unit `a` and across each other.
fn across(a: V3) -> [V3; 2] {
    let other = if a[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let first = cross(a, other);
    let first = scale(first, 1.0 / norm(first));
    [first, cross(a, first)]
}

/// The share of a pose error one tick closes, along the error's own straight line: no more
/// than the limb can still stop on, as a collision row's approach is planned.
///
/// The joint motion per unit of the error is its damped least-squares direction
/// `d = A' (A A' + damping I)^-1 e/|e|` over the task rows `A = [Jv; rho Jw]`, the QP's own
/// answer to it, and the error closes at most at the acceleration the tightest axis allows
/// along `d`. Asking for the whole error at once overshoots against the acceleration caps,
/// and the overshoot wanders the arm's free direction tick after tick; near a singularity
/// `d` is large and the approach slows instead.
#[allow(clippy::too_many_arguments)]
fn approach(
    e_p: V3,
    e_r: V3,
    jv: &[V3],
    jw: &[V3],
    solved: &[usize],
    caps: &Caps,
    s: &Settings,
    dt: f64,
) -> f64 {
    let rho = s.orientation_m_per_rad;
    let e = [
        e_p[0],
        e_p[1],
        e_p[2],
        rho * e_r[0],
        rho * e_r[1],
        rho * e_r[2],
    ];
    let length = e.iter().map(|v| v * v).sum::<f64>().sqrt();
    if length < 1e-12 {
        return 1.0;
    }
    let row = |a: usize, axis: usize| {
        if a < 3 {
            jv[axis][a]
        } else {
            rho * jw[axis][a - 3]
        }
    };
    let mut m = [[0.0; 6]; 6];
    for (a, line) in m.iter_mut().enumerate() {
        for (b, cell) in line.iter_mut().enumerate() {
            *cell = solved.iter().map(|&axis| row(a, axis) * row(b, axis)).sum();
        }
        line[a] += s.damping;
    }
    let mut y = e.map(|v| v / length);
    cholesky_solve(&mut m, &mut y);
    let mut accel = f64::INFINITY;
    for &axis in solved {
        let d: f64 = (0..6).map(|a| row(a, axis) * y[a]).sum::<f64>().abs();
        if d > 1e-12 {
            accel = accel.min(caps.accel[axis] / d);
        }
    }
    if !accel.is_finite() {
        return 1.0;
    }
    length.min(braking(length, s.braking_share * accel, dt)) / length
}

/// [`approach`] for a camera's pointing error `e` (radians) over its rows `jl` (per axis, the
/// pointing rate per unit of joint motion; zero on axes it may not turn): the share of `e`
/// one tick closes, no more than the neck can still stop on along its damped least-squares
/// direction.
fn pointing_approach(
    e: V3,
    jl: &[V3],
    solved: &[usize],
    caps: &Caps,
    s: &Settings,
    rho2: f64,
    dt: f64,
) -> f64 {
    let length = norm(e);
    if length < 1e-12 {
        return 1.0;
    }
    // The 6x6 solve with the pointing rows in its first block and identity in the second.
    let mut m = [[0.0; 6]; 6];
    for a in 0..3 {
        for b in 0..3 {
            m[a][b] = solved.iter().map(|&axis| jl[axis][a] * jl[axis][b]).sum();
        }
        m[a][a] += s.damping / rho2;
        m[a + 3][a + 3] = 1.0;
    }
    let mut y = [e[0] / length, e[1] / length, e[2] / length, 0.0, 0.0, 0.0];
    cholesky_solve(&mut m, &mut y);
    let mut accel = f64::INFINITY;
    for &axis in solved {
        let d: f64 = (0..3).map(|a| jl[axis][a] * y[a]).sum::<f64>().abs();
        if d > 1e-12 {
            accel = accel.min(caps.accel[axis] / d);
        }
    }
    if !accel.is_finite() {
        return 1.0;
    }
    length.min(braking(length, s.braking_share * accel, dt)) / length
}

/// One tick's position and orientation error, each held to the most one tick closes
/// (`max_error_m`, `max_error_rad`), and together, weighed as the solve weighs them, to `most`
/// metres when the tool has a speed ([`Ik::set_tool_speed`]): a turn left behind is not caught
/// up faster than the tool may move, nor dragging the tool point with it.
fn within_tool_speed(e_p: V3, e_r: V3, most: Option<f64>, s: &Settings) -> (V3, V3) {
    let (e_p, e_r) = (
        clamp_norm(e_p, s.max_error_m),
        clamp_norm(e_r, s.max_error_rad),
    );
    let Some(most) = most else {
        return (e_p, e_r);
    };
    let weighed = norm(e_p).hypot(s.orientation_m_per_rad * norm(e_r));
    if weighed <= most {
        return (e_p, e_r);
    }
    let share = most / weighed;
    (scale(e_p, share), scale(e_r, share))
}

/// Solve `m x = b` in place for a symmetric positive definite 6x6 `m` (`b` becomes `x`).
fn cholesky_solve(m: &mut [[f64; 6]; 6], b: &mut [f64; 6]) {
    for j in 0..6 {
        let mut diagonal = m[j][j];
        for k in 0..j {
            diagonal -= m[j][k] * m[j][k];
        }
        let diagonal = diagonal.max(1e-18).sqrt();
        m[j][j] = diagonal;
        for i in j + 1..6 {
            let mut v = m[i][j];
            for k in 0..j {
                v -= m[i][k] * m[j][k];
            }
            m[i][j] = v / diagonal;
        }
    }
    for i in 0..6 {
        for k in 0..i {
            b[i] -= m[i][k] * b[k];
        }
        b[i] /= m[i][i];
    }
    for i in (0..6).rev() {
        for k in i + 1..6 {
            b[i] -= m[k][i] * b[k];
        }
        b[i] /= m[i][i];
    }
}

fn clamp_norm(v: V3, most: f64) -> V3 {
    let length = norm(v);
    if length > most {
        scale(v, most / length)
    } else {
        v
    }
}

/// When this tick's QP must stop: `budget` from now. A browser's WASM has no monotonic clock
/// (`Instant::now` panics on `wasm32-unknown-unknown`), so there the iteration bound
/// ([`Settings::max_iterations`]) alone ends the solve; the headset's ghost is not the
/// real-time authority, the station is.
fn deadline_after(budget: Duration) -> Option<Instant> {
    if cfg!(all(target_arch = "wasm32", target_os = "unknown")) {
        None
    } else {
        Some(Instant::now() + budget)
    }
}
