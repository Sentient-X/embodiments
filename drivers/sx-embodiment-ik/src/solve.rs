//! The per-tick QP: assembled into buffers made once and solved by [`crate::qp::Qp`] in
//! place.

use std::time::{Duration, Instant};

use crate::IkError;
use crate::body::{Body, Target, Task, world};
use crate::geometry::{Obstacle, closest_segments, segment_box};
use crate::model::Frames;
use crate::qp::Qp;
use sx_frames::linalg::{Vec3, cross, dot, mat_mul, mat_vec, norm, scale, sub, transpose};
use sx_frames::{Matrix3, QuaternionLayout, Rigid, matrix_rotation_vector};

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

/// The most a limb's tool may move ([`Ik::set_tool_speed`]): its point, m/s, and its turn,
/// rad/s. Each is a cap of the step, as a joint's speed is: rows give way before it does.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToolSpeed {
    pub m_s: f64,
    /// `f64::INFINITY`: the turn is bounded only by the joints' caps.
    pub rad_s: f64,
}

/// The tool's speed rows ([`Ik::set_tool_speed`]): its linearised motion held within the
/// polytope whose faces are normal to these 26 directions (each axis, each pair of axes, all
/// three), each face at the speed. It holds the whole ball of the speed, so no direction is
/// slowed below it; its furthest vertices reach 1.128 of it, which the forward-kinematics check
/// ([`Ik::bound_tool_speed`]) takes back. Fixed in the root frame, so a step that kept it last
/// tick keeps it this tick at the same velocity.
const TOOL_DIRECTIONS: [[i8; 3]; 26] = [
    [1, 0, 0],
    [-1, 0, 0],
    [0, 1, 0],
    [0, -1, 0],
    [0, 0, 1],
    [0, 0, -1],
    [1, 1, 0],
    [1, -1, 0],
    [-1, 1, 0],
    [-1, -1, 0],
    [1, 0, 1],
    [1, 0, -1],
    [-1, 0, 1],
    [-1, 0, -1],
    [0, 1, 1],
    [0, 1, -1],
    [0, -1, 1],
    [0, -1, -1],
    [1, 1, 1],
    [1, 1, -1],
    [1, -1, 1],
    [1, -1, -1],
    [-1, 1, 1],
    [-1, 1, -1],
    [-1, -1, 1],
    [-1, -1, -1],
];

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
    /// Weight on the change of each tick's step from the last, in the motion that moves no
    /// driven tool (the null space of the tools' rows), per rad^2 of a column that turns its
    /// tools about their own points, more for a column that swings them further (a shoulder
    /// about 10x a wrist): `(dq - dq_prev)' N' D N (dq - dq_prev)` ([`Ik::smooth_null_space`]).
    /// A redundant arm's elbow carries on as it was going instead of shivering while the hand
    /// tracks; the driven tools' own motion is not smoothed (a held column, such as a torso held
    /// in one-handed teleop, counts as free: a 0.5 m/s sweep's RMS error 10.96 -> 10.99 mm).
    /// [`Ik::asked`] reads the whole cost, this term included. Tuned on lag against jerk on the
    /// station's solve after the full-braking approach (embodiments#239; `tests/fluid.rs` and
    /// `teleop_shake synth governed`, 2026-11-05): at 1e-4 every jump, sweep and sine kept its
    /// time to the tick (jump to 5 mm 0.452 -> 0.450 s mean, felt solve latency unchanged in
    /// every scenario), the tremor's null-space jerk fell 647 -> 425 rad/s^3 and null-space
    /// reversals 12.6 -> 7.5 /s; 3e-4 added 11 ms of felt solve latency on a clean sweep.
    /// Registry-wide, tracking within 2 % and null-space jerk 8-83 % lower on every redundant
    /// body. Zero: the step before it.
    pub smoothness: f64,
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
    /// A pair closes no faster than its room over this, seconds: the command slows as it nears
    /// the margin, so that a real arm trailing it can still stop within the margin. An arm
    /// stops later than its command by its transport delay and its follower's lag. On the
    /// static upper body that is 26-63 ms of delay plus the shoulders' ringing at zeta 0.1. With
    /// only the braking rows (acceleration caps, no lag), main's arms reached 4.5 mm into the
    /// margin of a box beside the forearm at 90 Hz, and a part 0.49 mm into it at 50 Hz, on those
    /// followers under the caps the station applies (-8.0 mm and a part 5 mm in at 1.25x their
    /// speed). At 0.3 s: 8.77 / 8.76 mm nominal at 90 / 50 Hz, 1.73 mm at worst (main 1.67 in that
    /// case), no part inside a box; 0.2 s left -0.62 mm nominal, and 0.4 s fails `follow_lag`
    /// (embodiments `tests/spark_followers.rs`, 2026-11-03). At 250 and 500 Hz the 1.25x
    /// follower still reaches -20 / -45 mm (main -114 / -119). Zero: no bound.
    pub approach_time_s: f64,
    /// The most any axis moves in one tick, whatever its speed cap: the QP is a
    /// linearisation, and a collision row is only as true as the step is short. Each axis's
    /// step is its cap times the tick ([`Caps`]), so this binds only above 3 rad/s at a 10 ms
    /// tick (a 0.5 m lever then moves 15 mm a tick against the 10 mm collision margin).
    pub max_step_rad: f64,
    /// The horizon beyond the tick ([`Ik::step`]): the two later knots' lengths, seconds. The
    /// step is solved with the motion that follows it: the tool's error at the end of the tick
    /// and of each knot, each joint's velocity held over each knot within its speed cap, its
    /// change between knots within its acceleration cap, and its position at each knot within
    /// its limits with room to stop after the last. The arm is asked to be on the (predicted)
    /// target at the knots, so it brakes only where the plan must, at its full capability,
    /// instead of every tick against the whole error.
    pub horizon_s: [f64; 2],
    /// How much the tool's error at the end of the tick weighs against the knots' (1): the
    /// tick's own error is the least the arm can close, and asking it all overshoots.
    pub tick_weight: f64,
    /// How much matching the target's velocity at the horizon's end weighs, in the units of a
    /// knot's error (the velocity error times the last knot's length): the plan's stop.
    pub terminal_weight: f64,
    /// A knot's error, metres, that a joint spending its whole acceleration cap across a knot
    /// costs as much as: the caps bound the plan softly.
    pub horizon_accel_m: f64,
    /// The most error, metres and radians, the horizon's linearisation is asked to close: the
    /// tools' Jacobian is frozen over the horizon, true only near where they stand, so a target
    /// further off is aimed at along its direction (a random posture a metre away was left
    /// 0.3-1.1 m short without it). A 10 cm jump is within it.
    pub linear_reach_m: f64,
    pub linear_reach_rad: f64,
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
            smoothness: 1e-4,
            posture_weight: 1e-6,
            shared_posture_weight: 1e-5,
            preferred_posture_weight: 1e-5,
            limit_margin_rad: 0.01,
            collision_margin_m: 0.01,
            influence_m: 0.2,
            approach_time_s: 0.3,
            max_step_rad: 0.03,
            horizon_s: [0.03, 0.12],
            tick_weight: 0.5,
            terminal_weight: 1.0,
            horizon_accel_m: 0.01,
            linear_reach_m: 0.15,
            linear_reach_rad: 0.5,
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
    /// was exceeded on that row's axes (reported [`Active::Accel`]), a pair already inside
    /// its margin was held to come no closer, a row gave way (never `Solved` with a row
    /// broken), or a tool's speed gave way to a row ([`Ik::set_tool_speed`]).
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

/// The cost of a knot past what keeps it clear or inside its bounds ([`Ik::keep_knots`]), per
/// metre (radian) squared: against a knot's tracking error at 1, a centimetre over costs as
/// a metre of error would.
const KNOT_KEEP: f64 = 1e6;

/// The null-space projector's damping, as a share of [`Settings::damping`]
/// ([`Ik::smooth_null_space`]). At the step's own damping the projector let the smoothing into
/// a six-joint arm's weak task directions near its wrist singularity, and a `UR10e`'s tool fell
/// behind a tremor by a third (1.81 to 2.45 mm RMS, `tests/fluid.rs`); at a hundredth it holds
/// 1.79 mm, and a hundred times less still left the static body's elbows twice the shiver
/// (348 to 807 rad/s^3), the projector flickering as a weak direction crossed it.
const NULL_DAMPING: f64 = 0.01;

/// How far past its bound a row may close and still be kept, metres: the interior point's sliver.
const ROW_KEPT_M: f64 = 1e-6;

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
        x: Vec3,
        y: Vec3,
        direction: Vec3,
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
    /// The solved axes that may move this step (available, not held): only they close a
    /// limb's error, so only their caps pace its approach.
    moving: Vec<usize>,
    jv: Vec<Vec3>,
    jw: Vec<Vec3>,
    jp: Vec<Vec3>,
    /// Per solved column, how far it swings the tools: [`Settings::smoothness`]'s weight.
    lever: Vec<f64>,
    /// This tick's task rows, each times the square root of its weight (`6 x limbs` by `n`),
    /// and how many are in use: the motion that moves a driven tool.
    task_rows: Vec<f64>,
    task_count: usize,
    /// Their Gram matrix (`6 limbs` square), `gram^-1 rows`, and the damped null-space
    /// projector `I - rows' gram^-1 rows` (`n` square).
    gram: Vec<f64>,
    through: Vec<f64>,
    null: Vec<f64>,
    /// A column of the Gram solve.
    pick: Vec<f64>,
    /// Per task row: its target at the end of the tick and of each knot, and the target's
    /// velocity along it (the horizon's rows are [`Workspace::task_rows`]).
    task_err: Vec<[f64; 3]>,
    /// Per solved column: held this step (unavailable), and where it stands.
    held_column: Vec<bool>,
    here: Vec<f64>,
    task_vel: Vec<f64>,
    /// The horizon's cost over `[dq, v2, v3]` (`3n` square) and its linear part.
    big_h: Vec<f64>,
    big_g: Vec<f64>,
    /// The tasks' Gram matrix over the solved columns, and its right sides per term.
    task_gram: Vec<f64>,
    task_rhs: Vec<[f64; 4]>,
    /// The knots' block's factor (`2n` square), a column of its solve, and its solve against
    /// the step's columns and the knots' linear part (`2n` by `n + 1`).
    knot_factor: Vec<f64>,
    knot_column: Vec<f64>,
    knot_through: Vec<f64>,
    /// The ask of each axis ([`Ik::asked`]): the step's own cost with the tasks' error at the
    /// horizon's end.
    ask_h: Vec<f64>,
    ask_g: Vec<f64>,
    /// A knot's keeping row over `[dq, v2, v3]` ([`Ik::keep_knots`]).
    keep: Vec<f64>,
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
    /// Per row, its pair's room to the margin, metres (a commanded pair's), for its knots'
    /// rows ([`Ik::knot_rows`]).
    row_room: Vec<f64>,
    /// Whether this tick's knots were folded into the step ([`Ik::horizon`]).
    folded: bool,
    row_binds: Vec<bool>,
    used: usize,
    /// The rows `..collisions` keep pairs clear; the rest hold tools to their speed
    /// ([`Ik::set_tool_speed`]), never given way.
    collisions: usize,
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
    /// Per limb, the most its tool may move, and from where ([`Ik::set_tool_speed`]).
    tool_speed: Vec<Option<(ToolSpeed, Option<Rigid>)>>,
    /// Per limb, whether its tool speed bound this tick.
    tool_capped: Vec<bool>,
    /// Per limb, its tool's motion this tick as the cost asks it, and as the last step moved
    /// it (point, metres; turn, a rotation vector): the speed rows' own directions.
    tool_asked: Vec<(Vec3, Vec3)>,
    tool_moved: Vec<(Vec3, Vec3)>,
    /// Per limb, its target's velocity (point m/s, turn rad/s, root frame) for the horizon's
    /// prediction ([`Ik::set_target_velocity`]); `None`: the target stands where it is.
    target_velocity: Vec<Option<(Vec3, Vec3)>>,
    /// Per axis, its follower's lag behind its command, seconds ([`Ik::set_follower_lag`]).
    follower_lag: Vec<f64>,
    /// Per axis, its jerk cap (unit/s^3; infinite: none), and per solved column the last
    /// step's acceleration: the state's third derivative bounded as its second is
    /// ([`Ik::set_jerk_cap`]).
    jerk: Vec<f64>,
    acceleration: Vec<f64>,
    /// Per axis, where the next step must leave it ([`Ik::set_window`]).
    window: Vec<[f64; 2]>,
    /// Per axis, where it was last measured ([`Ik::set_measured`]); NaN where it was not.
    measured: Vec<f64>,
    /// Per axis, the measurement before this one, and a smoothed measured velocity (units/s).
    measured_before: Vec<f64>,
    measured_speed: Vec<f64>,
    /// Per axis, whether it does not follow its command ([`Ik::set_measured`]).
    stuck: Vec<bool>,
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
        // Every pair at both poses, each with its stuck joints' copy: a row is never dropped for
        // want of room. With 32 rows the closest pairs crowded the rest out, arm against arm (up
        // to 86 rows wanted), and the step still said Solved with a dropped row broken by up to
        // 15 mm of its closing (2026-11-03).
        // And per limb, a tool's speed: its point's and its turn's faces.
        let k = 6 * (body.proxies.pairs.len() + scene_pairs.len())
            + 4 * n
            + 2 * (TOOL_DIRECTIONS.len() + 2) * body.limbs.len();
        // One column more than the solved axes: the rows' slack ([`Ik::solve_qp`]).
        let qp = Qp::new(n + 1, k);
        // Twice: the pairs at the commanded positions and at the measured ones.
        let candidates = Vec::with_capacity(2 * (body.proxies.pairs.len() + scene_pairs.len()));
        let work = Workspace {
            frames: body.model.poses(&rest),
            after: body.model.poses(&rest),
            seen: body.model.poses(&rest),
            seen_q: rest.clone(),
            moving: Vec::with_capacity(body.solved.len()),
            jv: vec![[0.0; 3]; axes.len()],
            jw: vec![[0.0; 3]; axes.len()],
            jp: vec![[0.0; 3]; axes.len()],
            lever: vec![0.0; n],
            task_rows: vec![0.0; 6 * body.limbs.len() * n],
            task_count: 0,
            gram: vec![0.0; 36 * body.limbs.len() * body.limbs.len()],
            through: vec![0.0; 6 * body.limbs.len() * n],
            null: vec![0.0; n * n],
            pick: vec![0.0; (6 * body.limbs.len()).max(1)],
            task_err: vec![[0.0; 3]; 6 * body.limbs.len()],
            held_column: vec![false; n],
            here: vec![0.0; n],
            task_vel: vec![0.0; 6 * body.limbs.len()],
            big_h: vec![0.0; 9 * n * n],
            big_g: vec![0.0; 3 * n],
            task_gram: vec![0.0; n * n],
            task_rhs: vec![[0.0; 4]; n],
            knot_factor: vec![0.0; 4 * n * n],
            knot_column: vec![0.0; 2 * n],
            knot_through: vec![0.0; 2 * n * (n + 1)],
            ask_h: vec![0.0; n * n],
            ask_g: vec![0.0; n],
            keep: vec![0.0; 3 * n],
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
            row_room: vec![0.0; k],
            folded: false,
            row_binds: vec![false; k],
            used: 0,
            collisions: 0,
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
            tool_asked: vec![([0.0; 3], [0.0; 3]); limbs],
            tool_moved: vec![([0.0; 3], [0.0; 3]); limbs],
            target_velocity: vec![None; limbs],
            follower_lag: vec![0.0; axes_count],
            jerk: vec![f64::INFINITY; axes_count],
            acceleration: vec![0.0; n],
            window: vec![[f64::NEG_INFINITY, f64::INFINITY]; axes_count],
            measured: vec![f64::NAN; axes_count],
            measured_before: vec![f64::NAN; axes_count],
            measured_speed: vec![0.0; axes_count],
            stuck: vec![false; axes_count],
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
        self.acceleration.fill(0.0);
        self.measured_before.fill(f64::NAN);
        self.measured_speed.fill(0.0);
        self.tool_moved.fill(([0.0; 3], [0.0; 3]));
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
    /// to come, the command's lead counted where it closes a pair. A stuck joint is never
    /// counted on to open a pair: each pair it would help is kept by the rest alone too
    /// (exactly, for one stuck joint in the pair).
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
        let row = &w.ask_h[column * n..(column + 1) * n];
        let curvature = row[column];
        if curvature <= 0.0 || !curvature.is_finite() {
            return 0.0;
        }
        let slope: f64 = row
            .iter()
            .zip(&w.still[..n])
            .map(|(h, x)| h * x)
            .sum::<f64>()
            + w.ask_g[column];
        w.still[column] - slope / curvature
    }

    /// The most `limb`'s tool (a pose limb's frame: its origin, the tool point, and its turn)
    /// may move from now on; `None`: as fast as its joints' caps allow. Each step keeps the
    /// tool's motion within both, as it keeps each joint within its speed: the QP holds the
    /// tool's linearised motion inside a polytope around the speed ([`TOOL_DIRECTIONS`]), and
    /// collision rows give way before it does, so a step that keeps a pair clear finds a way
    /// within the tool's speed or keeps the pair as nearly as it can (act-sim 2026-11-04: an
    /// elastic step opening a measured pair moved the tool point at 1 m/s and turned it at
    /// 1.9 rad/s against 0.288 m/s and 0.96 rad/s). The answer is then checked by forward
    /// kinematics and the whole step shortened where the linearisation was off. The cost asks
    /// no more of the error than both allow, so a goal at a linear speed lags its target rather
    /// than catching up faster (act-sim 2026-10-31: 0.55 m/s against a 0.288 m/s goal).
    /// `from` is where the tool was last commanded, when the step's positions are not (a
    /// teleoperation reference held back toward a lagging joint): the forward-kinematics check
    /// holds from there. A limb out of range or a linear speed not positive and finite clears
    /// it; a turn not positive is the same as none.
    pub fn set_tool_speed(&mut self, limb: usize, speed: Option<ToolSpeed>, from: Option<Rigid>) {
        if let Some(slot) = self.tool_speed.get_mut(limb) {
            *slot = speed.filter(|v| v.m_s.is_finite() && v.m_s > 0.0).map(|v| {
                let rad_s = if v.rad_s > 0.0 {
                    v.rad_s
                } else {
                    f64::INFINITY
                };
                let from = from.filter(|f| {
                    f.t.iter()
                        .chain(f.r.iter().flatten())
                        .all(|v| v.is_finite())
                });
                (ToolSpeed { m_s: v.m_s, rad_s }, from)
            });
        }
    }

    /// Where `limb`'s target is going: its point's velocity (m/s) and its turn rate (rad/s) in
    /// the root frame, for the horizon's prediction of it ([`Settings::horizon_s`]); `None`:
    /// it stands where it is. A limb out of range or a velocity not finite clears it.
    pub fn set_target_velocity(&mut self, limb: usize, velocity: Option<(Vec3, Vec3)>) {
        if let Some(slot) = self.target_velocity.get_mut(limb) {
            *slot = velocity.filter(|(v, w)| v.iter().chain(w).all(|x| x.is_finite()));
        }
    }

    /// How far `axis`'s follower lags its command, seconds (a first-order follower's time
    /// constant, from its journals): the horizon plans where the joint will be, not where it is
    /// sent, and leads the command by it. Zero (the default) or anything not finite and
    /// positive: it is where it is sent.
    pub fn set_follower_lag(&mut self, axis: usize, seconds: f64) {
        if let Some(slot) = self.follower_lag.get_mut(axis) {
            *slot = if seconds.is_finite() && seconds > 0.0 {
                seconds
            } else {
                0.0
            };
        }
    }

    /// The most `axis`'s acceleration may change, unit/s^3 (an embodiment's declared jerk):
    /// each step's acceleration stays within it of the last, as its velocity stays within the
    /// acceleration cap of the last ([`Ik::reset`] starts from rest). A joint limit, a window
    /// or a pair outranks it: where they cannot be kept within it, it gives way. Infinite (the
    /// default) or anything not finite and positive: no cap.
    pub fn set_jerk_cap(&mut self, axis: usize, jerk: f64) {
        if let Some(slot) = self.jerk.get_mut(axis) {
            *slot = if jerk.is_finite() && jerk > 0.0 {
                jerk
            } else {
                f64::INFINITY
            };
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
                (Task::Pose, Some(Target::Pose(target))) => {
                    target
                        .translation_m
                        .iter()
                        .chain(&QuaternionLayout::Xyzw.write(target.rotation))
                        .all(|v| v.is_finite())
                        && target.rotation.norm_squared() > 1e-12
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
        self.work.moving.clear();
        self.work.moving.extend(
            self.body
                .solved
                .iter()
                .copied()
                .filter(|&axis| available[axis]),
        );
        // An unavailable axis carries no motion into this step: not into the smoothness cost's
        // last step either (refute of embodiments#241).
        for (column, &axis) in self.body.solved.iter().enumerate() {
            if !available[axis] {
                self.velocity[column] = 0.0;
            }
            self.work.held_column[column] = !available[axis];
            self.work.here[column] = q[axis];
        }
        self.cost(q, targets, dt);
        self.bounds(q, dt);
        for (column, &axis) in self.body.solved.iter().enumerate() {
            if !available[axis] {
                let w = &mut self.work;
                (w.lower[column], w.upper[column]) = (0.0, 0.0);
                (w.safe_lower[column], w.safe_upper[column]) = (0.0, 0.0);
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
        self.body.model.poses_into(&w.seen_q, &mut w.seen);
        self.collision_rows(q, dt);
        self.horizon(dt);
        self.fold();
        if self.keep_knots(dt) {
            self.fold();
        }
        self.tool_rows(targets, dt);
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
        // The answer, held to its box; on failure, standing still as nearly as the caps allow,
        // every joint braked together: the last step scaled by the one share each joint's box
        // allows, so the tool slows along the line it was moving on. Braked each on its own
        // (until 2026-11-06), the fast-stopping joints stopped first and the tool swerved off
        // its line at up to 0.6 m/s on a missed deadline (refute of #3434).
        let together = if solved {
            None
        } else {
            let (mut low, mut high) = (0.0_f64, 1.0_f64);
            for (j, &v) in self.velocity.iter().enumerate().take(n) {
                let p = v * dt;
                if p.abs() < 1e-12 {
                    if w.lower[j] > 0.0 || w.upper[j] < 0.0 {
                        high = -1.0;
                    }
                    continue;
                }
                let (a, b) = (w.lower[j] / p, w.upper[j] / p);
                low = low.max(a.min(b));
                high = high.min(a.max(b));
            }
            (low <= high).then_some(low)
        };
        for j in 0..n {
            let x = match (solved, together) {
                (true, _) => w.answer[j],
                (false, Some(share)) => self.velocity[j] * dt * share,
                (false, None) => w.still[j],
            };
            w.still[j] = x.clamp(w.lower[j], w.upper[j]);
        }
        if !solved {
            solve = Solve::Held;
        }
        if self.bound_tool_speed(q, dt) && solve == Solve::Solved {
            solve = Solve::Relaxed;
        }
        // Never Solved on a broken row: a step that says its rows hold keeps every one of them
        // as it leaves (the box clamp and the tool's shortening come after the QP), or it says
        // they gave way. It is not swapped for standing still: a row that asks its pair to
        // open is broken worse by standing still than by the step.
        if solve == Solve::Solved && !self.rows_kept() {
            solve = Solve::Relaxed;
        }
        let w = &mut self.work;
        let dq = &w.still;
        self.step.q.copy_from_slice(q);
        self.step.qdot.iter_mut().for_each(|v| *v = 0.0);
        for (column, &axis) in self.body.solved.iter().enumerate() {
            self.step.q[axis] = q[axis] + dq[column];
            let velocity = dq[column] / dt;
            self.acceleration[column] = (velocity - self.velocity[column]) / dt;
            self.velocity[column] = velocity;
            self.step.qdot[axis] = self.velocity[column];
        }
        self.step.solve = solve;
        self.step.iterations = iterations.unwrap_or(0);
        self.report(targets);
        Ok(&self.step)
    }

    /// The step's own cost, `1/2 dq' H dq + g' dq` (the step itself, damping; posture toward the
    /// anchor; smoothness), and every task's rows with their targets over the horizon
    /// ([`Workspace::task_rows`], [`Ik::horizon`]): tool position and orientation (or pointing),
    /// each at the end of the tick and of each knot, the target predicted along its velocity
    /// ([`Ik::set_target_velocity`]). No error is shortened here: the horizon plans how fast it
    /// closes.
    #[allow(clippy::too_many_lines)] // one match arm per task and target
    fn cost(&mut self, q: &[f64], targets: &[Option<Target>], dt: f64) {
        let body = &self.body;
        let s = &self.settings;
        let w = &mut self.work;
        let n = body.solved.len();
        w.h.iter_mut().for_each(|v| *v = 0.0);
        w.g.iter_mut().for_each(|v| *v = 0.0);
        let rho2 = s.orientation_m_per_rad * s.orientation_m_per_rad;
        let [t2, t3] = s.horizon_s;
        let times = [dt, dt + t2, dt + t2 + t3];
        w.lever.fill(0.0);
        w.task_count = 0;
        for (index, (limb, target)) in body.limbs.iter().zip(targets).enumerate() {
            let pose = w.frames.links[limb.frame];
            for &axis in &body.solved {
                w.jv[axis] = [0.0; 3];
                w.jw[axis] = [0.0; 3];
            }
            body.model
                .point_jacobian(&w.frames, limb.frame, pose.t, &mut w.jv);
            body.model
                .angular_jacobian(&w.frames, limb.frame, &mut w.jw);
            // How far each column swings this limb's frame per unit, in the task's own metric
            // (a radian of turn weighs orientation_m_per_rad metres), driven or not.
            for (column, &axis) in body.solved.iter().enumerate() {
                w.lever[column] += dot(w.jv[axis], w.jv[axis]) + rho2 * dot(w.jw[axis], w.jw[axis]);
            }
            let Some(target) = target else { continue };
            let (v, turning) = self.target_velocity[index].unwrap_or(([0.0; 3], [0.0; 3]));
            match (limb.task, target) {
                (Task::Pose, Target::Pose(target)) => {
                    let e_p = sub(target.translation_m, pose.t);
                    let goal = target.rotation.matrix();
                    let e_r = matrix_rotation_vector(&mat_mul(&goal, &transpose(&pose.r)));
                    self.tool_asked[index] = (
                        [0, 1, 2].map(|k| v[k].mul_add(times[1], e_p[k])),
                        [0, 1, 2].map(|k| turning[k].mul_add(times[1], e_r[k])),
                    );
                    match self.tool_speed[index] {
                        // A goal at a speed is planned at it: each knot asks no more of the
                        // error than the tool may close by then, so the plan never builds a
                        // speed the step's own cap would have to take back.
                        Some((speed, _)) => {
                            let at = times.map(|t| within_speed(e_p, e_r, speed, t, s));
                            for k in 0..3 {
                                fill(&mut w.task, &body.solved, |axis| w.jv[axis][k]);
                                horizon_row_at(w, n, at.map(|(p, _)| p[k]), 0.0, 1.0);
                                fill(&mut w.task, &body.solved, |axis| w.jw[axis][k]);
                                horizon_row_at(w, n, at.map(|(_, r)| r[k]), 0.0, rho2);
                            }
                        }
                        None => {
                            let at = times.map(|t| {
                                (
                                    clamp_norm(
                                        [0, 1, 2].map(|k| v[k].mul_add(t, e_p[k])),
                                        s.linear_reach_m,
                                    ),
                                    clamp_norm(
                                        [0, 1, 2].map(|k| turning[k].mul_add(t, e_r[k])),
                                        s.linear_reach_rad,
                                    ),
                                )
                            });
                            for k in 0..3 {
                                fill(&mut w.task, &body.solved, |axis| w.jv[axis][k]);
                                horizon_row_at(w, n, at.map(|(p, _)| p[k]), v[k], 1.0);
                                fill(&mut w.task, &body.solved, |axis| w.jw[axis][k]);
                                horizon_row_at(w, n, at.map(|(_, r)| r[k]), turning[k], rho2);
                            }
                        }
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
                    // The axis's pointing error; the roll about the axis is left out of the
                    // rows altogether, so it floats.
                    let a = mat_vec(&pose.r, scale(*axis, 1.0 / norm(*axis)));
                    let u = scale(*toward, 1.0 / norm(*toward));
                    let e_p = sub(*translation_m, pose.t);
                    let e_r = pointing_error(a, u);
                    self.tool_asked[index] =
                        ([0, 1, 2].map(|k| v[k].mul_add(times[1], e_p[k])), e_r);
                    let at = times.map(|t| {
                        clamp_norm([0, 1, 2].map(|k| v[k].mul_add(t, e_p[k])), s.linear_reach_m)
                    });
                    let e_r = clamp_norm(e_r, s.linear_reach_rad);
                    for k in 0..3 {
                        fill(&mut w.task, &body.solved, |axis| w.jv[axis][k]);
                        horizon_row_at(w, n, at.map(|p| p[k]), v[k], 1.0);
                    }
                    for across in across(a) {
                        fill(&mut w.task, &body.solved, |axis| dot(across, w.jw[axis]));
                        horizon_row(w, n, dot(across, e_r), dot(across, turning), rho2, times);
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
                    let e = clamp_norm(pointing_error(f, u), s.linear_reach_rad);
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
                    for k in 0..3 {
                        fill(&mut w.task, &body.solved, |axis| w.jp[axis][k]);
                        horizon_row(w, n, e[k], 0.0, rho2, times);
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
        // A tool held to a speed ([`Ik::set_tool_speed`]: a program's goal) is a hard cap, and
        // carried null-space motion can keep a joint from slowing as the cap's shortening needs:
        // at 500 Hz a tool went 0.26 m/s against 0.2 (`a_tool_speed_never_shortens_a_step_past_a_row`).
        // Such a step is not smoothed; an operator's hands, which set no speed, are.
        if s.smoothness > 0.0 && self.tool_speed.iter().all(Option::is_none) {
            self.smooth_null_space(dt);
        }
    }

    /// The horizon's cost over `[dq, v2, v3]` ([`Workspace::big_h`]): the step's own cost on
    /// `dq`, and the tasks' error at the end of the tick (weighed [`Settings::tick_weight`]) and
    /// of each knot (1), where the joints stand at `q + dq`, `+ v2 t2`, `+ v3 t3`, each lagging
    /// its command by its follower's lag at its velocity there (`- lag v`, a first-order
    /// follower's steady lag; [`Ik::set_follower_lag`]), and the tools' velocity at the end
    /// against the target's ([`Settings::terminal_weight`]). The later knots carry the step's
    /// damping too, per their own motion.
    fn horizon(&mut self, dt: f64) {
        let s = &self.settings;
        let w = &mut self.work;
        let n = self.body.solved.len();
        let m = w.task_count;
        let [t2, t3] = s.horizon_s;
        w.task_gram.fill(0.0);
        w.task_rhs.fill([0.0; 4]);
        for r in 0..m {
            let row = &w.task_rows[r * n..(r + 1) * n];
            let (e, v) = (w.task_err[r], w.task_vel[r]);
            for i in 0..n {
                if row[i] == 0.0 {
                    continue;
                }
                for j in 0..n {
                    w.task_gram[i * n + j] += row[i] * row[j];
                }
                let rhs = &mut w.task_rhs[i];
                rhs[0] += row[i] * e[0];
                rhs[1] += row[i] * e[1];
                rhs[2] += row[i] * e[2];
                rhs[3] += row[i] * v * t3;
            }
        }
        // What the step asks of each axis on its own ([`Ik::asked`]): the step's own cost and the
        // tasks' error at the horizon's end, as one step would close it. The plan spreads that
        // over its knots; a stall's rule reads the ask, not how the plan spreads it.
        for i in 0..n {
            for j in 0..n {
                w.ask_h[i * n + j] = w.h[i * n + j] + 2.0 * w.task_gram[i * n + j];
            }
            w.ask_g[i] = w.g[i] - 2.0 * w.task_rhs[i][2];
        }
        let big = 3 * n;
        w.big_h.fill(0.0);
        w.big_g.fill(0.0);
        for i in 0..n {
            for j in 0..n {
                w.big_h[i * big + j] = w.h[i * n + j];
            }
            w.big_g[i] = w.g[i];
        }
        // Per term, each block's coefficient on a column: how far that block's motion of the
        // column carries the follower at the term's time.
        let weights = [s.tick_weight, 1.0, 1.0, s.terminal_weight];
        for (term, &weight) in weights.iter().enumerate() {
            if weight == 0.0 {
                continue;
            }
            // Velocity changes linearly across each knot (constant acceleration, as the caps
            // bound it), so a knot's displacement is its length times its mean velocity; the
            // tick's own velocity is dq / dt.
            let carry = 1.0 + t2 / (2.0 * dt);
            let coefficient = |block: usize, column: usize| -> f64 {
                let lag = self.follower_lag[self.body.solved[column]];
                match (term, block) {
                    (0, 0) => 1.0,
                    (1 | 2, 0) => carry,
                    (1, 1) => t2 / 2.0 - lag.min(t2 / 2.0),
                    (2, 1) => (t2 + t3) / 2.0,
                    (2, 2) => t3 / 2.0 - lag.min(t3 / 2.0),
                    (3, 2) => t3,
                    _ => 0.0,
                }
            };
            for bi in 0..3 {
                for i in 0..n {
                    let ci = coefficient(bi, i);
                    if ci == 0.0 {
                        continue;
                    }
                    w.big_g[bi * n + i] -= 2.0 * weight * ci * w.task_rhs[i][term];
                    for bj in 0..3 {
                        for j in 0..n {
                            let cj = coefficient(bj, j);
                            if cj == 0.0 {
                                continue;
                            }
                            w.big_h[(bi * n + i) * big + bj * n + j] +=
                                2.0 * weight * ci * w.task_gram[i * n + j] * cj;
                        }
                    }
                }
            }
        }
        // The later knots carry the step's damping per their own motion, and each change of a
        // joint's velocity across a knot costs as much as a knot's error of `accel_m` when it
        // spends the joint's whole acceleration cap there: the caps bound the plan softly, so
        // the knots fold into the step below.
        let soft = 2.0 * s.horizon_accel_m * s.horizon_accel_m;
        for i in 0..n {
            let axis = self.body.solved[i];
            let a = self.caps.accel[axis];
            let (k2, k3) = (n + i, 2 * n + i);
            w.big_h[k2 * big + k2] += 2.0 * s.damping * t2 * t2;
            w.big_h[k3 * big + k3] += 2.0 * s.damping * t3 * t3;
            // (v2 - dq / dt) / (a t2) and (v3 - v2) / (a t3), squared.
            let (c2, c3) = (soft / (a * t2).powi(2), soft / (a * t3).powi(2));
            let pairs: [(usize, f64, usize, f64, f64); 2] =
                [(k2, 1.0, i, -1.0 / dt, c2), (k3, 1.0, k2, -1.0, c3)];
            for (p, cp, r, cr, weight) in pairs {
                w.big_h[p * big + p] += weight * cp * cp;
                w.big_h[r * big + r] += weight * cr * cr;
                w.big_h[p * big + r] += weight * cp * cr;
                w.big_h[r * big + p] += weight * cp * cr;
            }
            if w.held_column[i] {
                // A held joint plans no motion.
                w.big_h[k2 * big + k2] += 1e6;
                w.big_h[k3 * big + k3] += 1e6;
            }
        }
    }

    /// Fold the knots into the step: minimising over them for each step leaves the step's
    /// own cost H_xx - H_xk H_kk^-1 H_kx, g_x - H_xk H_kk^-1 g_k (a Schur complement, one 2n
    /// square factor), and the knots' answer to any step, `-through [dq; 1]`.
    fn fold(&mut self) {
        let w = &mut self.work;
        let n = self.body.solved.len();
        let big = 3 * n;
        let k = 2 * n;
        for i in 0..k {
            for j in 0..k {
                w.knot_factor[i * k + j] = w.big_h[(n + i) * big + n + j];
            }
        }
        w.folded = false;
        if !crate::qp::cholesky(&mut w.knot_factor[..k * k], k) {
            return; // not finite: the step's own cost alone
        }
        w.folded = true;
        // through = H_kk^-1 [H_kx | g_k], a column at a time.
        for c in 0..=n {
            for i in 0..k {
                w.knot_column[i] = if c < n {
                    w.big_h[(n + i) * big + c]
                } else {
                    w.big_g[n + i]
                };
            }
            crate::qp::cholesky_solve(&w.knot_factor[..k * k], k, &mut w.knot_column[..k]);
            for i in 0..k {
                w.knot_through[i * (n + 1) + c] = w.knot_column[i];
            }
        }
        for r in 0..n {
            for c in 0..n {
                let mut v = w.big_h[r * big + c];
                for i in 0..k {
                    v -= w.big_h[r * big + n + i] * w.knot_through[i * (n + 1) + c];
                }
                w.h[r * n + c] = v;
            }
            let mut v = w.big_g[r];
            for i in 0..k {
                v -= w.big_h[r * big + n + i] * w.knot_through[i * (n + 1) + n];
            }
            w.g[r] = v;
        }
    }

    /// Add [`Settings::smoothness`]: the step's change from the last step, in the motion that
    /// moves no driven tool, `(dq - dq_prev)' N' D N (dq - dq_prev)`. `N = I - A' (A A' +
    /// eps I)^-1 A` over this tick's task rows `A` (`eps` [`NULL_DAMPING`] of the damping) is
    /// the projector onto what the tools leave free, and `D` weighs each column by how far it swings the tools, so a shoulder weighs
    /// more than a wrist. The tools' own motion is never smoothed: their tracking and speed
    /// are the task's.
    fn smooth_null_space(&mut self, dt: f64) {
        let s = &self.settings;
        let w = &mut self.work;
        let n = self.body.solved.len();
        let m = w.task_count;
        let rho2 = s.orientation_m_per_rad * s.orientation_m_per_rad;
        // gram = A A' + NULL_DAMPING damping I, factored.
        for i in 0..m {
            for j in 0..=i {
                let dot: f64 = (0..n)
                    .map(|k| w.task_rows[i * n + k] * w.task_rows[j * n + k])
                    .sum();
                w.gram[i * m + j] = dot;
                w.gram[j * m + i] = dot;
            }
            w.gram[i * m + i] += NULL_DAMPING * s.damping;
        }
        if !(m == 0 || crate::qp::cholesky(&mut w.gram[..m * m], m)) {
            return; // the rows are not finite: no smoothing this tick
        }
        // through = gram^-1 A, a column at a time (through is m x n, row-major).
        for k in 0..n {
            for i in 0..m {
                w.pick[i] = w.task_rows[i * n + k];
            }
            crate::qp::cholesky_solve(&w.gram[..m * m], m, &mut w.pick[..m]);
            for i in 0..m {
                w.through[i * n + k] = w.pick[i];
            }
        }
        // null = I - A' through: symmetric.
        for i in 0..n {
            for j in 0..=i {
                let mut v: f64 = (0..m)
                    .map(|r| w.task_rows[r * n + i] * w.through[r * n + j])
                    .sum();
                v = -v;
                if i == j {
                    v += 1.0;
                }
                w.null[i * n + j] = v;
                w.null[j * n + i] = v;
            }
        }
        // H += 2 N' D N; g -= 2 N' D N p, p the last step carried on.
        for k in 0..n {
            let weight = s.smoothness * (w.lever[k] / rho2).max(1.0);
            // N's row k (= column k), weighed.
            let row = &w.null[k * n..(k + 1) * n];
            let carried: f64 = row
                .iter()
                .zip(&self.velocity)
                .map(|(nv, v)| nv * v * dt)
                .sum();
            for i in 0..n {
                let ri = row[i];
                if ri == 0.0 {
                    continue;
                }
                w.g[i] -= 2.0 * weight * ri * carried;
                for j in 0..n {
                    w.h[i * n + j] += 2.0 * weight * ri * row[j];
                }
            }
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
            // The acceleration within the jerk cap of the last step's, as the velocity is
            // within the acceleration cap of the last ([`Ik::set_jerk_cap`]).
            let jerk = self.jerk[axis];
            if jerk.is_finite() {
                let centre = self.acceleration[column].mul_add(dt * dt, previous);
                let reach = jerk * dt * dt * dt;
                let (j_lo, j_hi) = (centre - reach, centre + reach);
                if j_lo <= hi && j_hi >= lo {
                    if j_lo > lo {
                        (lo, lo_by) = (j_lo, Bound::Accel);
                    }
                    if j_hi < hi {
                        (hi, hi_by) = (j_hi, Bound::Accel);
                    }
                } else if j_hi < lo {
                    (hi, hi_by) = (lo, Bound::Accel);
                } else {
                    (lo, lo_by) = (hi, Bound::Accel);
                }
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
            let mut rhs = braking(room, capability, dt).min(0.5 * room);
            if s.approach_time_s > 0.0 {
                rhs = rhs.min(room * dt / s.approach_time_s);
            }
            if candidate.measured {
                // From where it was measured the body still makes its way to its command (a
                // following joint's lead, wherever it closes the pair), then the step: all of it
                // within half the room, as the commanded row's tick is. One linearisation spans
                // that whole motion, and a follower carries on past its command: with the whole
                // room, an act-sim-like PD (zeta 0.2) rang 40-60 mm into the other arm; with
                // half, it touches at worst (1.5 mm in; embodiments#206). The pace of closing is
                // the commanded row's to keep.
                let lead = body
                    .solved
                    .iter()
                    .zip(row.iter())
                    .filter(|&(&axis, _)| !self.stuck[axis])
                    .map(|(&axis, r)| {
                        let carried = MOMENTUM_S * self.measured_speed[axis];
                        (r * (q[axis] - w.seen_q[axis] + carried)).max(0.0)
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
            w.row_room[slot] = if candidate.measured { f64::NAN } else { room };
            w.used += 1;
            // The same pair kept without the stuck joints: a step must keep it whether or not
            // they make their move (exactly, for one stuck joint in the pair).
            let untrusted = |j: usize| self.stuck[body.solved[j]];
            if (0..n).any(|j| w.rows[slot * n + j] != 0.0 && untrusted(j)) {
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
                w.row_room[slot + 1] = f64::NAN;
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
        for slot in 0..w.collisions {
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
        for slot in 0..w.collisions {
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

    /// Whether the step (`still`) keeps every row it was solved with, to a micrometre.
    fn rows_kept(&self) -> bool {
        let n = self.body.solved.len();
        let w = &self.work;
        (0..w.collisions).all(|slot| {
            let row = &w.rows[slot * n..(slot + 1) * n];
            let closes: f64 = row.iter().zip(&w.still[..n]).map(|(a, b)| a * b).sum();
            closes <= w.rhs[slot] + ROW_KEPT_M
        })
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
            // A tool's speed borrows no slack: rows give way before it does.
            qp.r[slot * m + n] = if slot < w.collisions { -1.0 } else { 0.0 };
            qp.rhs[slot] = w.rhs[slot] * scale;
        }
        let iterations = qp.solve(self.settings.max_iterations, deadline).ok()?;
        for j in 0..n {
            w.answer[j] = qp.x[j] * w.unit[j];
        }
        Some(iterations)
    }

    /// Keep the plan's knots clear of every commanded pair: where the knots' answer to the step the arm is taking (its last velocity carried
    /// a tick) closes a pair by a knot more than the tick's own pacing allows (its room over
    /// [`Settings::approach_time_s`] times the knot's time, never past half the room; half that
    /// again), that knot is drawn back to the bound by a quadratic cost
    /// ([`KNOT_KEEP`]) and the knots are folded again. Soft, so the plan bends and the step never
    /// fails for it; the step's own rows stay hard. Without them the plan ran an arm at full
    /// capability toward a box beyond the tick, and a ringing follower overshot the command into
    /// the margin (7.8 mm against 10). True when it added any.
    fn keep_knots(&mut self, dt: f64) -> bool {
        let s = &self.settings;
        let w = &mut self.work;
        let n = self.body.solved.len();
        if !w.folded {
            return false;
        }
        let [t2, t3] = s.horizon_s;
        let carry = 1.0 + t2 / (2.0 * dt);
        let through = n + 1;
        let big = 3 * n;
        // The knots' answer to the step carried on: v = -through [dq; 1].
        for i in 0..2 * n {
            let mut v = w.knot_through[i * through + n];
            for j in 0..n {
                v += w.knot_through[i * through + j] * self.velocity[j] * dt;
            }
            w.knot_column[i] = -v;
        }
        let knots = [
            (dt + t2, t2 / 2.0, 0.0),
            (dt + t2 + t3, (t2 + t3) / 2.0, t3 / 2.0),
        ];
        let mut added = false;
        for slot in 0..w.used {
            let room = w.row_room[slot];
            if !room.is_finite() {
                continue;
            }
            for (time, alpha, beta) in knots {
                // Half the tick's own pacing: at the full pacing a zeta-0.1 follower ringing
                // past a 500 Hz command still reached 9.6 mm of a 10 mm margin.
                let allowed = room * (time / (2.0 * s.approach_time_s.max(1e-9))).min(0.25);
                let closes: f64 = (0..n)
                    .map(|i| {
                        w.rows[slot * n + i]
                            * (carry * self.velocity[i] * dt
                                + alpha * w.knot_column[i]
                                + beta * w.knot_column[n + i])
                    })
                    .sum();
                if closes > allowed {
                    for (block, factor) in [carry, alpha, beta].into_iter().enumerate() {
                        for i in 0..n {
                            w.keep[block * n + i] = factor * w.rows[slot * n + i];
                        }
                    }
                    keep_knot(w, big, allowed);
                    added = true;
                }
            }
        }
        added
    }

    /// Each speed-bound tool's rows ([`Ik::set_tool_speed`]), after the collision rows: its
    /// point's linearised motion within its speed along each of [`TOOL_DIRECTIONS`], and its
    /// turn the same within its turn. A face the box cannot keep (a joint already moving faster
    /// than its acceleration cap can slow) is set where the box leaves it, as the governor
    /// would: the step is as slow as the joints can make it, never held for a face no step
    /// reaches.
    fn tool_rows(&mut self, targets: &[Option<Target>], dt: f64) {
        let body = &self.body;
        let w = &mut self.work;
        let n = body.solved.len();
        w.collisions = w.used;
        for (index, (limb, target)) in body.limbs.iter().zip(targets).enumerate() {
            let (Some((speed, _)), Some(_), Task::Pose) =
                (self.tool_speed[index], target, limb.task)
            else {
                continue;
            };
            let pose = w.frames.links[limb.frame];
            for &axis in &body.solved {
                w.jv[axis] = [0.0; 3];
                w.jw[axis] = [0.0; 3];
            }
            body.model
                .point_jacobian(&w.frames, limb.frame, pose.t, &mut w.jv);
            body.model
                .angular_jacobian(&w.frames, limb.frame, &mut w.jw);
            let (asked, moved) = (self.tool_asked[index], self.tool_moved[index]);
            for (most, angular) in [(speed.m_s * dt, false), (speed.rad_s * dt, true)] {
                if !most.is_finite() {
                    continue;
                }
                // The fixed faces, then one along the motion asked and one along the last
                // step's: each face at the speed, wherever it points, holds the whole ball, so
                // these take back the fixed faces' corners just where the tool is going.
                let along = if angular {
                    [asked.1, moved.1]
                } else {
                    [asked.0, moved.0]
                };
                let fixed = TOOL_DIRECTIONS.iter().map(|d| d.map(f64::from));
                for u in fixed.chain(along) {
                    let length = norm(u);
                    if length < 1e-12 {
                        continue;
                    }
                    let u = scale(u, 1.0 / length);
                    let slot = w.used;
                    let row = &mut w.rows[slot * n..(slot + 1) * n];
                    for (value, &axis) in row.iter_mut().zip(&body.solved) {
                        *value = dot(u, if angular { w.jw[axis] } else { w.jv[axis] });
                    }
                    let (least, reach) = (0..n).fold((0.0, 0.0), |(least, reach), j| {
                        let (a, b) = (row[j] * w.lower[j], row[j] * w.upper[j]);
                        (least + a.min(b), reach + a.max(b))
                    });
                    if reach <= most {
                        continue; // no step inside the box moves it that far this way
                    }
                    w.rhs[slot] = most.max(least);
                    w.row_active[slot] = Active::ToolSpeed { limb: index };
                    w.row_measured[slot] = false;
                    w.used += 1;
                }
            }
        }
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
    fn bound_tool_speed(&mut self, q: &[f64], dt: f64) -> bool {
        self.tool_capped.fill(false);
        if self.tool_speed.iter().all(Option::is_none) {
            return false;
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
        // Nor below the scale at which a row would break: a row that asks the pair to open
        // (its bound below zero, at the measured pose) is kept by the step and broken by a
        // shorter one. Shortened past it, then held for it, a tool-speed step broke such rows by
        // up to 26 mm and cost 5.6 mm of clearance (refute of embodiments 885f73b).
        // Keeping clear outranks the tool's speed: where a row stops the shortening, the tool
        // moves faster than its speed and the step says so ([`Solve::Relaxed`]).
        let mut row_floor = 0.0_f64;
        for slot in 0..w.collisions {
            // A row at the measured pose (or a stuck joint's copy) is not the command's own
            // clearance: it asks the command back toward a lagging arm, and never outranks a
            // cap ([`Ik::keep_feasible`]). Letting it stop the shortening moved the tool point
            // at 1 m/s while it pulled the command back (act-sim, 2026-11-04).
            if w.row_measured[slot] {
                continue;
            }
            let row = &w.rows[slot * n..(slot + 1) * n];
            let closes: f64 = row.iter().zip(&w.still[..n]).map(|(a, b)| a * b).sum();
            let bound = w.rhs[slot];
            if closes < 0.0 && bound < 0.0 {
                row_floor = row_floor.max(if closes <= bound { bound / closes } else { 1.0 });
            }
        }
        let rows_bind = row_floor > floor;
        let floor = floor.max(row_floor).min(1.0);
        let mut scale = 1.0_f64;
        let mut over = false;
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
                let (most, most_rad) = (speed.m_s * dt, speed.rad_s * dt);
                let start = w.frames.links[limb.frame];
                let from = from
                    .filter(|from| {
                        norm(sub(from.t, start.t)) <= most && turn(&from.r, &start.r) <= most_rad
                    })
                    .map_or((start.t, start.r), |from| (from.t, from.r));
                let after = w.after.links[limb.frame];
                let moved = norm(sub(after.t, from.0));
                if moved > most * (1.0 + 1e-6) {
                    factor = factor.min(most / moved);
                    self.tool_capped[index] = true;
                }
                let turned = turn(&after.r, &from.1);
                if turned > most_rad * (1.0 + 1e-6) {
                    factor = factor.min(most_rad / turned);
                    self.tool_capped[index] = true;
                }
            }
            over = factor < 1.0;
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
        over && rows_bind && scale <= floor
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
            let start = w.frames.links[limb.frame];
            self.tool_moved[index] = (
                sub(pose.t, start.t),
                matrix_rotation_vector(&mat_mul(&pose.r, &transpose(&start.r))),
            );
            let (residual_m, residual_rad) = match (limb.task, target) {
                (_, Some(Target::Pose(target))) => (
                    norm(sub(target.translation_m, pose.t)),
                    norm(matrix_rotation_vector(&mat_mul(
                        &target.rotation.matrix(),
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
) -> (f64, Vec3, Vec3, Vec3, bool) {
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
    scratch: &mut [Vec3],
    link: usize,
    point: Vec3,
    direction: Vec3,
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
fn hidden(body: &Body, frames: &Frames, skip: &[bool], tool: Vec3, s: &Settings) -> Option<usize> {
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

/// The part of a pose error (`e_p` metres, `e_r` a rotation vector) a tool at `speed` closes in
/// `t` seconds: both shortened alike, the turn weighed as the solve weighs it, the turn also
/// within its own cap.
fn within_speed(e_p: Vec3, e_r: Vec3, speed: ToolSpeed, t: f64, s: &Settings) -> (Vec3, Vec3) {
    let weighed = norm(e_p).hypot(s.orientation_m_per_rad * norm(e_r));
    let mut share = 1.0_f64;
    if weighed > speed.m_s * t {
        share = share.min(speed.m_s * t / weighed);
    }
    if norm(e_r) > speed.rad_s * t {
        share = share.min(speed.rad_s * t / norm(e_r));
    }
    (scale(e_p, share), scale(e_r, share))
}

fn clamp_norm(v: Vec3, most: f64) -> Vec3 {
    let length = norm(v);
    if length > most {
        scale(v, most / length)
    } else {
        v
    }
}

/// Add [`KNOT_KEEP`] `(keep . z - bound)^2` to the horizon's cost over `z = [dq, v2, v3]`.
fn keep_knot(w: &mut Workspace, big: usize, bound: f64) {
    for p in 0..big {
        let cp = w.keep[p];
        if cp == 0.0 {
            continue;
        }
        w.big_g[p] -= 2.0 * KNOT_KEEP * cp * bound;
        for pp in 0..big {
            let cpp = w.keep[pp];
            if cpp != 0.0 {
                w.big_h[p * big + pp] += 2.0 * KNOT_KEEP * cp * cpp;
            }
        }
    }
}

/// [`horizon_row`] with its target given at each time.
fn horizon_row_at(w: &mut Workspace, n: usize, targets: [f64; 3], velocity: f64, weight: f64) {
    let count = w.task_count;
    if (count + 1) * n > w.task_rows.len() {
        return;
    }
    let root = weight.sqrt();
    for (slot, value) in w.task_rows[count * n..(count + 1) * n]
        .iter_mut()
        .zip(&w.task)
    {
        *slot = root * value;
    }
    w.task_err[count] = targets.map(|t| root * t);
    w.task_vel[count] = root * velocity;
    w.task_count += 1;
}

/// Keep `w.task` (times the square root of `weight`) as a task row, with its target at each
/// of `times` (the end of the tick and of each knot) moving at `velocity` from `error` now.
fn horizon_row(
    w: &mut Workspace,
    n: usize,
    error: f64,
    velocity: f64,
    weight: f64,
    times: [f64; 3],
) {
    let count = w.task_count;
    if (count + 1) * n > w.task_rows.len() {
        return;
    }
    let root = weight.sqrt();
    for (slot, value) in w.task_rows[count * n..(count + 1) * n]
        .iter_mut()
        .zip(&w.task)
    {
        *slot = root * value;
    }
    w.task_err[count] = times.map(|t| root * velocity.mul_add(t, error));
    w.task_vel[count] = root * velocity;
    w.task_count += 1;
}

/// The rotation turning unit `f` onto unit `u`: about `f x u`, by the whole angle between
/// them. The bare cross product vanishes at 180 degrees too, so a frame facing straight away
/// from its target never turned (a neck folded behind the hands on the bench); there, any
/// axis across `f` turns it.
fn pointing_error(f: Vec3, u: Vec3) -> Vec3 {
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
fn across(a: Vec3) -> [Vec3; 2] {
    let other = if a[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let first = cross(a, other);
    let first = scale(first, 1.0 / norm(first));
    [first, cross(a, first)]
}

/// The angle between two rotations, radians.
fn turn(a: &Matrix3, b: &Matrix3) -> f64 {
    let trace: f64 = (0..3)
        .map(|i| (0..3).map(|k| a[i][k] * b[i][k]).sum::<f64>())
        .sum();
    ((trace - 1.0) / 2.0).clamp(-1.0, 1.0).acos()
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

#[cfg(test)]
mod tests {
    use std::collections::{BTreeMap, VecDeque};

    use crate::teleop::{Targets, Teleop, TeleopConfig};
    use crate::{STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF, Solve};
    use sx_frames::Pose;

    fn tcp(teleop: &Teleop, q: &[f64], hand: usize) -> Pose {
        let body = teleop.ik().body();
        body.frame(&body.model().poses(q), hand).pose()
    }

    /// A step that misses its deadline brakes every joint together: its joint velocity is the
    /// last one scaled by one share, so the tool slows along the line it moved on rather than
    /// swerving as each joint stops on its own.
    #[test]
    fn a_missed_deadline_brakes_every_joint_together() {
        let config =
            TeleopConfig::parse(include_str!("../tests/fixtures/static-teleop-config.json"))
                .expect("the station's solve config");
        let ready: BTreeMap<String, f64> =
            serde_json::from_str(include_str!("../tests/fixtures/static-ready.json"))
                .expect("ready");
        let mut solve = Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config)
            .expect("solve");
        solve.set_deadline(std::time::Duration::from_secs(3600));
        let mut q: Vec<f64> = solve
            .model()
            .axes()
            .iter()
            .map(|a| ready.get(&a.name).copied().unwrap_or(0.0))
            .collect();
        let from = tcp(&solve, &q, 0);
        let targets = Targets {
            tcp: [
                Some(Pose {
                    translation_m: [0, 1, 2].map(|k| from.translation_m[k] + [0.0, 0.12, 0.0][k]),
                    rotation: from.rotation,
                }),
                None,
            ],
            ..Targets::default()
        };
        let measured = vec![None; q.len()];
        let available = vec![true; q.len()];
        let dt = 0.01;
        solve.reset();
        let mut last = vec![0.0; q.len()];
        for _ in 0..12 {
            let step = solve
                .step(&q, &measured, &available, &targets, dt)
                .expect("a step")
                .clone();
            q.copy_from_slice(&step.q);
            last.copy_from_slice(&step.qdot);
        }
        assert!(last.iter().any(|v| v.abs() > 0.2), "moving: {last:?}");
        solve.set_deadline(std::time::Duration::ZERO);
        let held = solve
            .step(&q, &measured, &available, &targets, dt)
            .expect("a step")
            .clone();
        assert_eq!(held.solve, Solve::Held);
        let (i, _) = last
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
            .expect("an axis");
        let share = held.qdot[i] / last[i];
        assert!((0.0..1.0).contains(&share), "braking: {share}");
        for (now, before) in held.qdot.iter().zip(&last) {
            assert!(
                (now - share * before).abs() < 1e-9,
                "each joint braked by the same share {share}: {now} from {before}"
            );
        }
    }

    /// The right hand reaching for the left one, a follower two ticks behind its commands, at
    /// the station's 50 Hz: arm against arm wants more rows than the old cap of 32 (up to 86),
    /// every one is loaded, and no step that says `Solved` breaks one (`Relaxed` says rows gave way). With the cap, the
    /// rows past it were dropped and the step still said `Solved` with a dropped row broken by
    /// up to 15 mm of its closing (2026-11-03).
    #[test]
    fn arm_against_arm_drops_no_row_and_a_kept_step_keeps_every_row() {
        let config =
            TeleopConfig::parse(include_str!("../tests/fixtures/static-teleop-config.json"))
                .expect("the station's solve config");
        let ready: BTreeMap<String, f64> =
            serde_json::from_str(include_str!("../tests/fixtures/static-ready.json"))
                .expect("ready");
        let mut solve = Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config)
            .expect("solve");
        solve.set_deadline(std::time::Duration::from_secs(3600));
        let rest: Vec<f64> = solve
            .model()
            .axes()
            .iter()
            .map(|a| ready.get(&a.name).copied().unwrap_or(0.0))
            .collect();
        let left_tcp = solve
            .model()
            .links()
            .iter()
            .position(|l| l == "left_gripper_tcp")
            .expect("tcp");
        let (left0, right0) = (tcp(&solve, &rest, 0), tcp(&solve, &rest, 1));
        let available = vec![true; rest.len()];
        let dt = 0.02;
        let (mut q, mut real) = (rest.clone(), rest.clone());
        let mut sent = VecDeque::from(vec![rest.clone(); 3]);
        let (mut most, mut held) = (0, 0);
        solve.reset();
        for tick in 0..250 {
            let t = f64::from(tick) * dt;
            let measured: Vec<Option<f64>> = real.iter().copied().map(Some).collect();
            let mut targets = Targets {
                tcp: [
                    Some(Pose {
                        translation_m: [0, 1, 2]
                            .map(|k| left0.translation_m[k] + [0.03, 0.0, -0.04][k]),
                        rotation: left0.rotation,
                    }),
                    None,
                ],
                ..Targets::default()
            };
            if t >= 1.0 {
                targets.tcp[1] = Some(Pose {
                    translation_m: solve.model().poses(&real).links[left_tcp].t,
                    rotation: right0.rotation,
                });
            }
            let step = solve
                .step(&q, &measured, &available, &targets, dt)
                .expect("a step");
            let solved = step.solve;
            q.copy_from_slice(&step.q);
            held += usize::from(solved == Solve::Held);
            let ik = solve.ik();
            most = most.max(ik.work.used);
            assert!(
                solved != Solve::Solved || ik.rows_kept(),
                "tick {tick}: a {solved:?} step breaks a row"
            );
            sent.push_back(q.clone());
            real = sent.pop_front().expect("a command");
        }
        assert!(
            most > 32,
            "the case wants more rows than the old cap: {most}"
        );
        assert_eq!(held, 0, "no step fails for want of a row");
    }
}
