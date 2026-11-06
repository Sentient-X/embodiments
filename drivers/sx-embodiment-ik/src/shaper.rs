//! The tool's target, shaped before the solve: every start and end of a motion jerk-limited.
//!
//! The solve closes a target's error as fast as the joints' acceleration caps allow (full
//! acceleration, then full braking), so a target that appears far away (an engage, a program's
//! first row), stops at once (a release, a plan's last row) or jumps commands the arm's whole
//! acceleration in one tick: 14-74x each joint's jerk cap on the static upper body
//! (`examples/transitions.rs`). Bounding jerk joint by joint inside the one-step QP hunted and
//! stalled (embodiments#258), and an S-curve inside the solve's approach cost 20-30 ms of lag.
//! So the target is shaped instead, before the solve, and the solve follows the shaped target
//! every tick with its collision rows, joint limits and speed caps in force.
//!
//! A [`ToolShaper`] is a third-order online trajectory generator (the Ruckig / Reflexxes idea,
//! reduced to what a tick needs), one per hand. Its state is the shaped pose and its six-vector
//! velocity and acceleration in the solve's own metric: the point in metres and the turn in
//! radians weighed `rho` metres each (`Settings::orientation_m_per_rad`), the turn as a rotation
//! vector re-anchored on the shaped orientation every tick. Each tick it moves along one line,
//! with `sx_embodiment_drivers::jerk`, the one jerk rule:
//!
//! - **follow** a target (where it is, and how it moves): along the line to it in the target's
//!   own moving frame, the fastest approach from which the jerk-bounded stop still ends on it
//!   ([`jerk::velocity_toward`]), so a target that holds still is never passed, and point and
//!   turn arrive together; motion across the line is braked to zero. Once on a target it rides
//!   it exactly while the target moves within the bounds. It starts where the tool stands, at
//!   rest ([`ToolShaper::reset`], then the first [`ToolShaper::step`]).
//! - **stop** when the target is gone (released, let go, a plan's end): along its motion, speed
//!   and acceleration to zero together at the jerk bound ([`jerk::acceleration_window`] into
//!   `[0, 0]`), then it holds and goes idle.
//!
//! The bounds along a line are the joints' own ([`ToolMap`]): the most the tool may move along
//! it with no joint past its cap in the damped least-squares motion the solve answers with.
//! A program's tool speed lowers the speed. Safe-stop never passes through here: it is the
//! chain's own stop.
//!
//! Allocation-free after the map's rows are sized: a shaper is a few fixed arrays.

use sx_embodiment_drivers::jerk;
use sx_frames::linalg::{Vec3, mat_mul, transpose};
use sx_frames::{Pose, Quaternion, matrix_rotation_vector, rotation_vector_matrix};

/// A six-vector in the solve's metric: point (metres) then turn (radians times `rho`).
pub type Six = [f64; 6];

/// How a limb's joints carry its tool's motion at a pose: per joint, how fast it turns per unit
/// of tool motion (the damped least-squares row `c_i` of `A' (A A' + damping I)^-1`, `A` the
/// limb's rows `[Jv; rho Jw]`) and its speed, acceleration and jerk caps. A tool motion `t` is
/// within the joints' caps when every `|c_i . t|` is within its cap.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ToolMap {
    rows: Vec<(Six, [f64; 3])>,
    /// The program's tool speed (metres, radians), when it is slower than the joints.
    speed: Option<(f64, f64)>,
    rho: f64,
}

/// What a bound is of: speed, acceleration or jerk.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Of {
    Speed = 0,
    Acceleration = 1,
    Jerk = 2,
}

impl ToolMap {
    /// A map with room for `axes` joints' rows.
    #[must_use]
    pub fn with_capacity(axes: usize) -> Self {
        Self {
            rows: Vec::with_capacity(axes),
            speed: None,
            rho: 1.0,
        }
    }

    /// The map of a limb at a pose: `jv` and `jw` its Jacobian per axis (every axis,
    /// [`crate::Body::jacobian`]), over the `axes` that `moving` lets move; `caps` per axis its
    /// speed, acceleration and jerk. Never allocates once sized.
    #[allow(clippy::too_many_arguments)]
    pub fn of_limb(
        &mut self,
        jv: &[Vec3],
        jw: &[Vec3],
        axes: &[usize],
        moving: &[bool],
        caps: (&[f64], &[f64], &[f64]),
        rho: f64,
        damping: f64,
    ) {
        self.rows.clear();
        self.speed = None;
        self.rho = rho;
        let row = |a: usize, axis: usize| {
            if a < 3 {
                jv[axis][a]
            } else {
                rho * jw[axis][a - 3]
            }
        };
        let used = |axis: &&usize| moving.get(**axis).copied().unwrap_or(false);
        let mut m = [[0.0; 6]; 6];
        for (a, line) in m.iter_mut().enumerate() {
            for (b, cell) in line.iter_mut().enumerate() {
                *cell = axes
                    .iter()
                    .filter(used)
                    .map(|&axis| row(a, axis) * row(b, axis))
                    .sum();
            }
            line[a] += damping;
        }
        // (A A' + damping I)^-1, a column at a time.
        let mut inverse = [[0.0; 6]; 6];
        for k in 0..6 {
            let mut e = [0.0; 6];
            e[k] = 1.0;
            let mut factor = m;
            crate::solve::cholesky_solve(&mut factor, &mut e);
            for a in 0..6 {
                inverse[a][k] = e[a];
            }
        }
        for &axis in axes.iter().filter(used) {
            let c: Six =
                std::array::from_fn(|k| (0..6).map(|a| row(a, axis) * inverse[a][k]).sum());
            let cap = |of: &[f64]| of.get(axis).copied().unwrap_or(f64::INFINITY);
            let caps = [cap(caps.0), cap(caps.1), cap(caps.2)];
            if self.rows.len() < self.rows.capacity() {
                self.rows.push((c, caps));
            }
        }
    }

    /// No faster than `m_s` and `rad_s` (a program's tool speed).
    pub const fn at_most(&mut self, m_s: f64, rad_s: f64) {
        self.speed = Some((m_s, rad_s));
    }

    /// The most `|t|` may be along unit `t` (unit/s, /s² or /s³).
    fn along(&self, t: &Six, of: Of) -> f64 {
        let mut worst = 0.0_f64;
        for (c, caps) in &self.rows {
            let rate: f64 = (0..6).map(|k| c[k] * t[k]).sum::<f64>().abs();
            worst = worst.max(rate / caps[of as usize]);
        }
        let most = if worst > 1e-12 {
            LINE_SHARE / worst
        } else {
            f64::INFINITY
        };
        if of == Of::Speed {
            most.min(self.program(t))
        } else {
            most
        }
    }

    /// The most `|t|` may be along unit `t` by a program's tool speed alone (infinite with none).
    fn program(&self, t: &Six) -> f64 {
        let Some((m_s, rad_s)) = self.speed else {
            return f64::INFINITY;
        };
        let linear = (t[0] * t[0] + t[1] * t[1] + t[2] * t[2]).sqrt();
        let angular = (t[3] * t[3] + t[4] * t[4] + t[5] * t[5]).sqrt() / self.rho;
        let mut most = f64::INFINITY;
        if linear > 1e-12 {
            most = most.min(m_s / linear);
        }
        if angular > 1e-12 && rad_s > 0.0 {
            most = most.min(rad_s / angular);
        }
        most
    }

    /// `t` no faster than a program's tool speed.
    fn program_bound(&self, t: Six) -> Six {
        let pace = norm6(&t);
        if pace <= 1e-15 {
            return t;
        }
        let most = self.program(&t.map(|x| x / pace));
        if pace > most {
            t.map(|x| x * most / pace)
        } else {
            t
        }
    }

    /// Whether motion `t` is within every joint's `of` cap.
    fn within(&self, t: &Six, of: Of) -> bool {
        let length = norm6(t);
        length <= 1e-15 || length <= self.along(&t.map(|x| x / length), of) * (1.0 + 1e-9)
    }
}

/// The share of each joint's caps the line bounds are taken at: the damped least-squares map is
/// the motion's at this pose, and as the arm moves it changes. At the whole cap a shoulder
/// reached its speed cap mid-move (1.6x its jerk cap) and the 30 cm engage passed 1.0x; at
/// 0.85 every engage, a plan's start and a release stay within 0.97x on the static upper body.
pub const LINE_SHARE: f64 = 0.85;

/// How many times the joints' damped least-squares line bounds a moving target is caught within
/// (its acceleration and jerk), and, read from its own motion, the speed past which a tick's move
/// is a jump. Catching an operator's filtered hand is not a start or an end: the tremor filter
/// turns a 2 Hz sine round almost at rest, far past the line jerk, and at 4x the shaper never
/// caught it (11.4 mm off against main's 6.9). At 8x every tracking case of `table_station`
/// matches main. Starts from rest, landings on a target standing still and every stop keep the
/// line bounds themselves.
pub const RIDE: f64 = 8.0;

/// What a tool is asked to reach this tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aim {
    pub pose: Pose,
    /// How the target moves: its point's velocity (m/s) and turn rate (rad/s), root frame.
    /// `None`: read from how the target moved since the last tick.
    pub motion: Option<(Vec3, Vec3)>,
    /// A unit axis (root frame) whose turn about it is not asked (a free roll): left as the
    /// shaped orientation has it.
    pub free_axis: Option<Vec3>,
}

/// The shaped target: its pose and how it moves.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Shaped {
    pub pose: Pose,
    pub velocity: Vec3,
    pub turning: Vec3,
}

impl Shaped {
    /// Its pose `seconds` on along its motion (back for a negative time).
    #[must_use]
    pub fn at(&self, seconds: f64) -> Pose {
        let mut pose = self.pose;
        for k in 0..3 {
            pose.translation_m[k] = self.velocity[k].mul_add(seconds, pose.translation_m[k]);
        }
        let turn = self.turning.map(|w| w * seconds);
        if turn.iter().any(|w| *w != 0.0) {
            pose.rotation = (Quaternion::from_matrix(&rotation_vector_matrix(turn))
                * pose.rotation)
                .normalized();
        }
        pose
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct State {
    pose: Pose,
    /// Six-vectors in the map's metric.
    velocity: Six,
    acceleration: Six,
    /// The target last tick, for a target that does not say how it moves, and its motion then.
    last: Option<Pose>,
    moved: Option<Six>,
    rho: f64,
    /// Whether it rides a moving target it has caught ([`ToolShaper::step`]).
    riding: bool,
}

/// One hand's target shaper; see the module docs.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ToolShaper {
    state: Option<State>,
}

impl ToolShaper {
    /// Idle: the next target is reached from where the tool then stands, at rest.
    pub fn reset(&mut self) {
        self.state = None;
    }

    /// Whether it shapes a target: following one, or coming to rest after one went.
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.state.is_some()
    }

    /// Whether it is at rest: no target is moving it, and it stands still.
    #[must_use]
    pub fn is_still(&self) -> bool {
        self.state
            .is_some_and(|s| s.velocity.iter().chain(&s.acceleration).all(|x| *x == 0.0))
    }

    /// The shaped target now, if active.
    #[must_use]
    pub fn shaped(&self) -> Option<Shaped> {
        self.state.map(|s| Shaped {
            pose: s.pose,
            velocity: [s.velocity[0], s.velocity[1], s.velocity[2]],
            turning: [3, 4, 5].map(|k| s.velocity[k] / s.rho),
        })
    }

    /// One tick of `dt`: toward `aim`, or to rest without one, where it then stays until
    /// [`ToolShaper::reset`] (the caller lets go once the arm stands there too). `tool` is where
    /// the tool stands (the pose the solve would reach with no target), read only when the
    /// shaper starts. The shaped target; `None` when idle with no aim.
    #[allow(clippy::too_many_lines)] // one tick: the target's motion, riding, catching, landing
    pub fn step(
        &mut self,
        tool: impl FnOnce() -> Pose,
        aim: Option<&Aim>,
        map: &ToolMap,
        dt: f64,
    ) -> Option<Shaped> {
        if !(dt.is_finite() && dt > 0.0) {
            return self.shaped();
        }
        let rho = map.rho;
        let state = match (&mut self.state, aim) {
            (None, None) => return None,
            (Some(state), _) => state,
            (None, Some(_)) => self.state.insert(State {
                pose: tool(),
                velocity: [0.0; 6],
                acceleration: [0.0; 6],
                last: None,
                moved: None,
                rho,
                riding: false,
            }),
        };
        // The metric is the map's; a state carried over a change of weight keeps its motion.
        if (state.rho - rho).abs() > 0.0 {
            for k in 3..6 {
                state.velocity[k] *= rho / state.rho;
                state.acceleration[k] *= rho / state.rho;
            }
            state.rho = rho;
        }
        let Some(aim) = aim else {
            advance(state, None, &[0.0; 6], &[0.0; 6], map, dt);
            state.last = None;
            state.moved = None;
            state.riding = false;
            integrate(state, None, dt);
            return self.shaped();
        };
        let target = aim.pose;
        let mut error: Six = [0.0; 6];
        let turn = matrix_rotation_vector(&mat_mul(
            &target.rotation.matrix(),
            &transpose(&state.pose.rotation.matrix()),
        ));
        for k in 0..3 {
            error[k] = target.translation_m[k] - state.pose.translation_m[k];
            error[k + 3] = rho * turn[k];
        }
        // How the target moves: as it says, or as it moved since the last tick. A move faster
        // than [`RIDE`] times the joints' line speed in one tick is a jump, not a motion to ride.
        let moving: Six = match (aim.motion, state.last) {
            (Some((v, w)), _) => [v[0], v[1], v[2], rho * w[0], rho * w[1], rho * w[2]],
            (None, Some(last)) => {
                let w = matrix_rotation_vector(&mat_mul(
                    &target.rotation.matrix(),
                    &transpose(&last.rotation.matrix()),
                ));
                let m = std::array::from_fn(|k| {
                    if k < 3 {
                        (target.translation_m[k] - last.translation_m[k]) / dt
                    } else {
                        rho * w[k - 3] / dt
                    }
                });
                let pace = norm6(&m);
                if pace <= 1e-15 || pace <= RIDE * map.along(&m.map(|x| x / pace), Of::Speed) {
                    m
                } else {
                    [0.0; 6]
                }
            }
            (None, None) => [0.0; 6],
        };
        state.last = Some(target);
        // Never past a program's tool speed: its feedforward would carry the solve's step past
        // the tool's own cap (0.35 m/s against 0.05 in `a_tool_speed_never_shortens_a_step_past_a_row`).
        // A target moving faster than it is caught, not ridden.
        let mut moving = map.program_bound(moving);
        if let Some(axis) = aim.free_axis {
            let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
            if n > 1e-9 {
                let u = axis.map(|x| x / n);
                for six in [&mut error, &mut moving] {
                    let along: f64 = (0..3).map(|k| six[k + 3] * u[k]).sum();
                    for k in 0..3 {
                        six[k + 3] -= along * u[k];
                    }
                }
            }
        }
        // How the target's motion changes: the planner reads it as holding that acceleration over
        // its stop, so it closes on a turning or oscillating target instead of trailing it (a
        // 2 Hz sine taken as moving steadily was followed 90 degrees behind). A change faster than
        // the ride bounds is a jump in its motion, not an acceleration to ride.
        let turning: Six = match state.moved {
            Some(before) => {
                let at: Six = std::array::from_fn(|k| (moving[k] - before[k]) / dt);
                let size = norm6(&at);
                if size <= 1e-12
                    || size <= RIDE * map.along(&at.map(|x| x / size), Of::Acceleration)
                {
                    at
                } else {
                    [0.0; 6]
                }
            }
            None => [0.0; 6],
        };
        state.moved = Some(moving);
        let still = norm6(&moving) <= STILL_M_S && norm6(&turning) <= STILL_M_S;
        // Riding a moving target it has caught: on it, at its own motion, exactly what the solve
        // was handed before the shaper (an operator's hand is followed as main follows it; held to
        // the joints' line speed, a 1 Hz VR circle was followed 3x further off). Only a jump
        // (further from where the ride would carry it than [`RIDE_JUMP_M`]) is caught again.
        if state.riding && !still {
            let ahead: Six = std::array::from_fn(|k| state.velocity[k].mul_add(-dt, error[k]));
            if norm6(&ahead) <= RIDE_JUMP_M {
                state.velocity = moving;
                state.acceleration = turning;
                state.pose = if aim.free_axis.is_some() {
                    Pose {
                        translation_m: target.translation_m,
                        rotation: turned(state.pose.rotation, error, rho),
                    }
                } else {
                    target
                };
                return self.shaped();
            }
        }
        state.riding = false;
        let landed = advance(state, Some(&error), &moving, &turning, map, dt);
        integrate(
            state,
            landed
                .then_some(&target)
                .filter(|_| aim.free_axis.is_none()),
            dt,
        );
        if landed && aim.free_axis.is_some() {
            state.pose.translation_m = target.translation_m;
        }
        // Caught: within [`CAUGHT_M`] of where the target ends the tick, moving and turning with
        // it closely enough that taking its motion is a step within the line jerk.
        if !still {
            // What is left after this tick's motion, and the jerk of taking the target's motion
            // and its change at once: within the ride jerk.
            let left: Six = std::array::from_fn(|k| state.velocity[k].mul_add(-dt, error[k]));
            let slip: Six =
                std::array::from_fn(|k| (moving[k] - state.velocity[k]) / (RIDE * dt * dt));
            let push: Six =
                std::array::from_fn(|k| (turning[k] - state.acceleration[k]) / (RIDE * dt));
            state.riding = norm6(&left) <= CAUGHT_M
                && map.within(&slip, Of::Jerk)
                && map.within(&push, Of::Jerk);
        }
        self.shaped()
    }
}

/// `rotation` turned by the angular part of `six` (the solve's metric, `rho` metres a radian).
fn turned(rotation: Quaternion, six: Six, rho: f64) -> Quaternion {
    let turn = [3, 4, 5].map(|k| six[k] / rho);
    if turn.iter().all(|w| *w == 0.0) {
        return rotation;
    }
    (Quaternion::from_matrix(&rotation_vector_matrix(turn)) * rotation).normalized()
}

fn norm6(v: &Six) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

fn dot6(a: &Six, b: &Six) -> f64 {
    (0..6).map(|k| a[k] * b[k]).sum()
}

/// The pose a tick on: onto `onto` when the step landed there, else along the velocity.
fn integrate(state: &mut State, onto: Option<&Pose>, dt: f64) {
    if let Some(pose) = onto {
        state.pose = *pose;
        return;
    }
    let v = state.velocity;
    for k in 0..3 {
        state.pose.translation_m[k] = v[k].mul_add(dt, state.pose.translation_m[k]);
    }
    let turn = [3, 4, 5].map(|k| v[k] / state.rho * dt);
    if turn.iter().any(|w| *w != 0.0) {
        state.pose.rotation = (Quaternion::from_matrix(&rotation_vector_matrix(turn))
            * state.pose.rotation)
            .normalized();
    }
}

/// One tick of the state's velocity and acceleration: toward `error` (what is left to the
/// target at the tick's end) with the target moving at `moving`, or to rest without one.
/// Following, whether it lands on the target exactly; stopping, whether it is at rest.
fn advance(
    state: &mut State,
    error: Option<&Six>,
    moving: &Six,
    turning: &Six,
    map: &ToolMap,
    dt: f64,
) -> bool {
    let (v, a) = (state.velocity, state.acceleration);
    let next: Six;
    if let Some(error) = error {
        // A target standing still is landed on exactly once this tick can reach it and the next
        // stand there, within the line bounds (the jerk-bounded approach otherwise closes the
        // last fraction of a millimetre geometrically). A moving one is never snapped onto: the
        // snap's acceleration, left in the state, rang 0.5 mm about a steady 0.5 m/s target.
        let still = norm6(moving) <= STILL_M_S && norm6(turning) <= STILL_M_S;
        if still {
            let land: Six = error.map(|e| e / dt);
            let landing: Six = std::array::from_fn(|k| (land[k] - v[k]) / dt);
            let standing: Six = land.map(|x| -x / dt);
            let change: Six = std::array::from_fn(|k| (landing[k] - a[k]) / dt);
            let settle: Six = std::array::from_fn(|k| (standing[k] - landing[k]) / dt);
            if map.within(&land, Of::Speed)
                && map.within(&landing, Of::Acceleration)
                && map.within(&standing, Of::Acceleration)
                && map.within(&change, Of::Jerk)
                && map.within(&settle, Of::Jerk)
            {
                state.velocity = land;
                state.acceleration = landing;
                return true;
            }
        }
        next = if still {
            line(error, &v, &a, map, dt, false)
        } else {
            track(error, &v, &a, moving, turning, map, dt)
        };
    } else {
        next = line(&v, &v, &a, map, dt, true);
    }
    for k in 0..6 {
        state.acceleration[k] = (next[k] - v[k]) / dt;
    }
    state.velocity = next;
    if error.is_none() {
        let still = norm6(&next) < 1e-9 && norm6(&state.acceleration) < 1e-6;
        if still {
            state.velocity = [0.0; 6];
            state.acceleration = [0.0; 6];
        }
        return still;
    }
    false
}

/// `t` no longer than `factor` times the line bound `of` along it.
fn within(t: Six, of: Of, factor: f64, map: &ToolMap) -> Six {
    let length = norm6(&t);
    if length <= 1e-15 {
        return t;
    }
    let most = factor * map.along(&t.map(|x| x / length), of);
    if length > most {
        t.map(|x| x * most / length)
    } else {
        t
    }
}

/// How far a ridden target may land from where the ride carries it in a tick (the solve's metric,
/// metres) and still be ridden: further is a jump (a regrip, a 10 cm program row), caught again
/// within the bounds. Past the operator's carry (`teleop::LEAD_MAX_M`, 3 cm): a carried hand
/// that stops falls back by up to it in a tick, as main follows it; at 1 cm the shaper caught
/// that fall and carried 39 mm past a dead stop at 0.5 m/s against main's 32.
pub const RIDE_JUMP_M: f64 = 0.04;

/// How near a moving target the shaped one must come (the solve's metric, metres) before it rides
/// it.
pub const CAUGHT_M: f64 = 0.0005;

/// A target moving slower than this (m/s, or m/s² for its change, in the solve's metric) stands
/// still: an operator's still hand carries its filter's leftover rate.
pub const STILL_M_S: f64 = 1e-4;

/// The poles of [`track`], rad/s: three at this, at most a quarter of the tick's rate.
pub const TRACK_RAD_S: f64 = 30.0;

/// The next velocity following a moving target: a critically damped third-order tracker (its
/// three poles at [`TRACK_RAD_S`]) with the target's motion and its change fed forward, each
/// loop held to its bound (the catch-up speed to the line speed, the acceleration and jerk to
/// [`RIDE`] times theirs), so a target that turns or oscillates is followed with a lag of its
/// jerk over the cube of the poles and no wind-up. `error` is what is left to the target at the
/// tick's end.
fn track(
    error: &Six,
    v: &Six,
    a: &Six,
    moving: &Six,
    turning: &Six,
    map: &ToolMap,
    dt: f64,
) -> Six {
    let w = TRACK_RAD_S.min(0.25 / dt);
    // Where the target stands now (its end-of-tick place less this tick's motion).
    let gap: Six = std::array::from_fn(|k| moving[k].mul_add(-dt, error[k]));
    // A moving target is ridden at the ride bounds however far behind: closing its gap at the
    // line bounds fed back (behind a 2 Hz sine by more than a centimetre, it fell 43 mm behind).
    let size = norm6(&gap);
    let ride = RIDE;
    // Caught up no faster than the gap can still be closed and stopped on at the line's
    // acceleration (the time-optimal law): faster, the saturated inner loops overshot and the
    // tracker rang 120 mm about a 3 cm sine.
    let catch_up = if size > 1e-15 {
        let u = gap.map(|x| x / size);
        let most = (ride * map.along(&u, Of::Acceleration) * size)
            .sqrt()
            .min(map.along(&u, Of::Speed))
            .min(w / 3.0 * size);
        u.map(|x| x * most)
    } else {
        [0.0; 6]
    };
    let v_ref = map.program_bound(std::array::from_fn(|k| moving[k] + catch_up[k]));
    let push = within(
        std::array::from_fn(|k| w * (v_ref[k] - v[k])),
        Of::Acceleration,
        ride,
        map,
    );
    let a_ref: Six = std::array::from_fn(|k| turning[k] + push[k]);
    let jerk = within(
        std::array::from_fn(|k| 3.0 * w * (a_ref[k] - a[k])),
        Of::Jerk,
        ride,
        map,
    );
    let mut next_a: Six = std::array::from_fn(|k| jerk[k].mul_add(dt, a[k]));
    // Never past the acceleration bound, wound down at the jerk bound from above it.
    let length = norm6(&next_a);
    if length > 1e-15 {
        let most = (ride * map.along(&next_a.map(|x| x / length), Of::Acceleration))
            .max(norm6(a) - norm6(&jerk) * dt);
        if length > most {
            next_a = next_a.map(|x| x * most / length);
        }
    }
    std::array::from_fn(|k| next_a[k].mul_add(dt, v[k]))
}

/// The next relative velocity: along the unit line `toward` points (`remaining` to come to rest
/// on, or, `stop`, the motion itself), the jerk-bounded approach (or stop) at the joints'
/// bounds along it; across it, the motion braked to zero.
fn line(toward: &Six, v: &Six, a: &Six, map: &ToolMap, dt: f64, stop: bool) -> Six {
    let length = norm6(toward);
    let u: Six = if length > 1e-12 {
        toward.map(|x| x / length)
    } else {
        let speed = norm6(v);
        if speed > 1e-12 {
            v.map(|x| x / speed)
        } else {
            let accel = norm6(a);
            if accel <= 1e-12 {
                return [0.0; 6];
            }
            a.map(|x| x / accel)
        }
    };
    let (speed, accel, jerk) = (
        map.along(&u, Of::Speed),
        map.along(&u, Of::Acceleration),
        map.along(&u, Of::Jerk),
    );
    let v_along = dot6(v, &u);
    let a_along = dot6(a, &u);
    // A ride's acceleration past the line bound winds down at the jerk bound, never cut.
    let accel = accel.max(a_along.abs());
    let along = if stop {
        let (x, _) = jerk::acceleration_window(v_along, a_along, (0.0, 0.0), accel, jerk, dt);
        let next = x.mul_add(dt, v_along);
        if next * v_along < 0.0 { 0.0 } else { next }
    } else {
        jerk::velocity_toward(
            if length > 1e-12 { length } else { 0.0 },
            v_along,
            a_along,
            speed,
            accel,
            jerk,
            dt,
        )
    };
    let mut next: Six = u.map(|x| x * along);
    // Across the line: each component's leftover motion braked to rest at its own bounds.
    for k in 0..6 {
        let (vk, ak) = (v[k] - v_along * u[k], a[k] - a_along * u[k]);
        if vk == 0.0 && ak == 0.0 {
            continue;
        }
        let mut e = [0.0; 6];
        e[k] = 1.0;
        let (x, _) = jerk::acceleration_window(
            vk,
            ak,
            (0.0, 0.0),
            map.along(&e, Of::Acceleration).max(ak.abs()),
            map.along(&e, Of::Jerk),
            dt,
        );
        let across = x.mul_add(dt, vk);
        next[k] += if across * vk < 0.0 { 0.0 } else { across };
    }
    next
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 0.01;
    /// Each component's caps in the test map: speed, acceleration and jerk.
    const CAPS: [f64; 3] = [1.0, 5.0, 50.0];

    fn at(x: f64) -> Pose {
        Pose {
            translation_m: [x, 0.0, 0.0],
            rotation: Quaternion::IDENTITY,
        }
    }

    /// One joint per component, unit rows: each component's line bounds [`LINE_SHARE`] of
    /// [`CAPS`].
    fn map() -> ToolMap {
        let mut map = ToolMap::with_capacity(6);
        map.rho = 0.2;
        for k in 0..6 {
            let mut c = [0.0; 6];
            c[k] = 1.0;
            map.rows.push((c, CAPS));
        }
        map
    }

    fn line_bounds() -> [f64; 3] {
        CAPS.map(|c| c * LINE_SHARE)
    }

    /// Per tick the point's x, its velocity and acceleration.
    fn run(
        shaper: &mut ToolShaper,
        ticks: usize,
        aim: impl Fn(usize) -> Option<Aim>,
    ) -> Vec<[f64; 3]> {
        let m = map();
        let mut out = Vec::new();
        for tick in 0..ticks {
            let stepped = shaper.step(|| at(0.0), aim(tick).as_ref(), &m, DT);
            let s = shaper.state.map_or([0.0; 3], |s| {
                [s.pose.translation_m[0], s.velocity[0], s.acceleration[0]]
            });
            let x = stepped.map_or(f64::NAN, |s| s.pose.translation_m[0]);
            out.push([x, s[1], s[2]]);
        }
        out
    }

    fn peaks(trace: &[[f64; 3]]) -> (f64, f64, f64) {
        let (mut v, mut a, mut j) = (0.0_f64, 0.0_f64, 0.0_f64);
        let mut before = 0.0;
        for s in trace {
            v = v.max(s[1].abs());
            a = a.max(s[2].abs());
            j = j.max((s[2] - before).abs() / DT);
            before = s[2];
        }
        (v, a, j)
    }

    fn moving_at(speed: f64) -> impl Fn(usize) -> Aim {
        move |tick| Aim {
            pose: at(speed * tick as f64 * DT),
            motion: Some(([speed, 0.0, 0.0], [0.0; 3])),
            free_axis: None,
        }
    }

    #[test]
    fn an_engage_reaches_a_far_target_within_the_line_bounds_and_never_past_it() {
        let [speed, accel, jerk] = line_bounds();
        for goal in [0.002, 0.1, 0.3, -0.25] {
            let mut shaper = ToolShaper::default();
            let trace = run(&mut shaper, 300, |_| {
                Some(Aim {
                    pose: at(goal),
                    motion: None,
                    free_axis: None,
                })
            });
            let (v, a, j) = peaks(&trace);
            assert!(v <= speed + 1e-9 && a <= accel + 1e-9, "{goal}: {v} {a}");
            assert!(j <= jerk * (1.0 + 1e-6), "{goal}: jerk {j}");
            let past = trace
                .iter()
                .map(|s| (s[0] - goal) * goal.signum())
                .fold(f64::MIN, f64::max);
            assert!(past <= 1e-12, "{goal}: {past} past");
            let last = trace.last().expect("ticks");
            assert!(
                (last[0] - goal).abs() < 1e-12 && last[1] == 0.0,
                "{goal}: {last:?}"
            );
        }
    }

    #[test]
    fn a_diagonal_move_stays_on_its_line() {
        let goal = Pose {
            translation_m: [0.1, 0.2, -0.05],
            rotation: Quaternion::IDENTITY,
        };
        let mut shaper = ToolShaper::default();
        let m = map();
        for _ in 0..300 {
            let s = shaper
                .step(
                    || at(0.0),
                    Some(&Aim {
                        pose: goal,
                        motion: None,
                        free_axis: None,
                    }),
                    &m,
                    DT,
                )
                .expect("shaped");
            let p = s.pose.translation_m;
            let along = (p[0] * 0.1 + p[1] * 0.2 - p[2] * 0.05) / (0.01 + 0.04 + 0.0025);
            let off = [0, 1, 2].map(|k| p[k] - along * goal.translation_m[k]);
            assert!(off.iter().all(|x| x.abs() < 1e-9), "{p:?}");
        }
        let p = shaper.shaped().expect("active").pose.translation_m;
        assert!(
            (0..3).all(|k| (p[k] - goal.translation_m[k]).abs() < 1e-12),
            "{p:?}"
        );
    }

    #[test]
    fn it_catches_a_moving_target_within_the_ride_bounds_then_rides_it() {
        let mut shaper = ToolShaper::default();
        let speed = 0.5;
        let trace = run(&mut shaper, 200, |tick| Some(moving_at(speed)(tick)));
        for (tick, s) in trace.iter().enumerate().skip(100) {
            assert!(
                (s[0] - speed * tick as f64 * DT).abs() < 1e-6,
                "{tick}: {} behind",
                speed * tick as f64 * DT - s[0]
            );
        }
        let (_, a, j) = peaks(&trace);
        let [_, accel, jerk] = line_bounds();
        assert!(
            a <= RIDE * accel + 1e-9 && j <= RIDE * jerk * (1.0 + 1e-6),
            "{a} {j}"
        );
    }

    #[test]
    fn an_oscillating_target_is_followed_closely_not_rung_about() {
        // A 2 Hz, 3 cm sine (4.7 m/s^2, 59 m/s^3 at its peaks): within the ride bounds after the
        // start. Saturated loops once rang 120 mm about it.
        let mut shaper = ToolShaper::default();
        let w = std::f64::consts::TAU * 2.0;
        let target = |tick: usize| 0.03 * (w * tick as f64 * DT).sin();
        let trace = run(&mut shaper, 400, |tick| {
            let t = tick as f64 * DT;
            Some(Aim {
                pose: at(target(tick)),
                motion: Some(([0.03 * w * (w * t).cos(), 0.0, 0.0], [0.0; 3])),
                free_axis: None,
            })
        });
        let worst = trace
            .iter()
            .enumerate()
            .skip(100)
            .map(|(tick, s)| (s[0] - target(tick)).abs())
            .fold(0.0, f64::max);
        assert!(worst < 0.01, "{worst} m off");
    }

    #[test]
    fn a_release_comes_to_rest_within_the_line_bounds_and_stays_there() {
        let mut shaper = ToolShaper::default();
        let speed = 0.5;
        let trace = run(&mut shaper, 300, |tick| {
            (tick < 100).then(|| moving_at(speed)(tick))
        });
        let stop = &trace[99..];
        let (_, a, j) = peaks(stop);
        let [_, accel, jerk] = line_bounds();
        assert!(a <= accel + 1e-9 && j <= jerk * (1.0 + 1e-6), "{a} {j}");
        assert!(shaper.is_still(), "at rest, held until the caller lets go");
        assert!(
            stop.windows(2).all(|w| w[1][0] >= w[0][0] - 1e-12),
            "never back"
        );
        // The jerk-bounded stop from 0.5 m/s: v^2 / 2a + v a / 2j, and a tick.
        let carried = trace.last().expect("ticks")[0] - speed * 99.0 * DT;
        let most = speed * speed / (2.0 * accel) + speed * accel / (2.0 * jerk) + speed * DT;
        assert!(carried <= most + 1e-9, "{carried} against {most}");
        shaper.reset();
        assert!(!shaper.is_active());
    }

    #[test]
    fn a_turn_is_shaped_as_a_point_is() {
        let mut shaper = ToolShaper::default();
        let m = map();
        let [speed, _, jerk] = line_bounds();
        let goal = Quaternion::from_matrix(&rotation_vector_matrix([0.0, 0.0, 1.2]));
        let mut before = 0.0;
        for _ in 0..400 {
            shaper.step(
                || at(0.0),
                Some(&Aim {
                    pose: Pose {
                        translation_m: [0.0; 3],
                        rotation: goal,
                    },
                    motion: None,
                    free_axis: None,
                }),
                &m,
                DT,
            );
            let s = shaper.state.expect("active");
            assert!(s.velocity[5].abs() <= speed + 1e-9);
            assert!((s.acceleration[5] - before).abs() <= jerk * DT * (1.0 + 1e-6));
            before = s.acceleration[5];
        }
        let s = shaper.shaped().expect("active");
        let left = matrix_rotation_vector(&mat_mul(
            &goal.matrix(),
            &transpose(&s.pose.rotation.matrix()),
        ));
        assert!(left.iter().all(|x| x.abs() < 1e-9), "{left:?}");
    }
}
