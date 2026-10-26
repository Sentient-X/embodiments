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
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            orientation_m_per_rad: 0.15,
            damping: 1e-4,
            posture_weight: 1e-6,
            shared_posture_weight: 1e-5,
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
}

impl Active {
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

#[derive(Clone, Copy, Debug)]
enum Pair {
    Capsules(usize, usize),
    /// A capsule and an obstacle.
    Scene(usize, usize),
}

#[derive(Clone, Copy, Debug)]
struct Candidate {
    distance: f64,
    pair: Pair,
}

/// Every buffer a tick writes, sized once.
struct Workspace {
    frames: Frames,
    after: Frames,
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
    row_binds: Vec<bool>,
    used: usize,
    unit: Vec<f64>,
    answer: Vec<f64>,
    column_active: Vec<Option<Active>>,
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
    /// (capsule, obstacle) pairs checked: on a moving link, and apart at rest.
    scene_pairs: Vec<(usize, usize)>,
    /// Per solved column: moves more than one limb.
    shared: Vec<bool>,
    /// Per limb, its solved columns.
    limb_columns: Vec<Vec<usize>>,
    /// Last commanded velocity per solved column.
    velocity: Vec<f64>,
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
        let shared = body
            .solved
            .iter()
            .map(|axis| body.limbs.iter().filter(|l| l.axes.contains(axis)).count() > 1)
            .collect();
        let limb_columns = body
            .limbs
            .iter()
            .map(|l| {
                l.axes
                    .iter()
                    .filter_map(|a| body.solved.binary_search(a).ok())
                    .collect()
            })
            .collect();
        let k = settings.max_rows;
        let qp = Qp::new(n, k);
        let candidates = Vec::with_capacity(body.proxies.pairs.len() + scene_pairs.len());
        let work = Workspace {
            frames: body.model.poses(&rest),
            after: body.model.poses(&rest),
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
                    active: Vec::with_capacity(n + k),
                })
                .collect(),
            solve: Solve::Solved,
            iterations: 0,
        };
        Ok(Self {
            body,
            caps,
            settings,
            scene,
            anchor: rest.clone(),
            anchored: false,
            rest,
            scene_pairs,
            shared,
            limb_columns,
            velocity: vec![0.0; n],
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
    /// still, and anchor posture at the next step's measured positions.
    pub fn reset(&mut self) {
        self.velocity.iter_mut().for_each(|v| *v = 0.0);
        self.anchored = false;
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
            let d = capsule_distance(body, &frames, i, j).0;
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
        self.collision_rows(dt);
        let mut solve = self.keep_feasible(false);
        let mut iterations = self.solve_qp(dt, deadline);
        if iterations.is_none() && self.work.used > 0 && deadline.is_none_or(|d| Instant::now() < d)
        {
            solve = self.keep_feasible(true);
            iterations = self.solve_qp(dt, deadline);
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
    fn cost(&mut self, q: &[f64], targets: &[Option<Target>], dt: f64) {
        let body = &self.body;
        let s = &self.settings;
        let w = &mut self.work;
        let n = body.solved.len();
        w.h.iter_mut().for_each(|v| *v = 0.0);
        w.g.iter_mut().for_each(|v| *v = 0.0);
        let rho2 = s.orientation_m_per_rad * s.orientation_m_per_rad;
        for (limb, target) in body.limbs.iter().zip(targets) {
            let Some(target) = target else { continue };
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
                    let e_p = clamp_norm(scale(e_p, share), s.max_error_m);
                    let e_r = clamp_norm(scale(e_r, share), s.max_error_rad);
                    for k in 0..3 {
                        fill(&mut w.task, &body.solved, |axis| w.jv[axis][k]);
                        least_squares(&mut w.h, &mut w.g, &w.task, e_p[k], 1.0);
                        fill(&mut w.task, &body.solved, |axis| w.jw[axis][k]);
                        least_squares(&mut w.h, &mut w.g, &w.task, e_r[k], rho2);
                    }
                }
                (Task::LookAt { forward }, Target::LookAt { point_m }) => {
                    let f = mat_vec(&pose.r, forward);
                    let d = sub(*point_m, pose.t);
                    let distance = norm(d);
                    if distance < 1e-3 {
                        continue;
                    }
                    let u = scale(d, 1.0 / distance);
                    // The pointing error as a rotation: about f x u, by the whole angle between
                    // f and u. The bare cross product vanishes at 180 degrees too, so a camera
                    // facing straight away from its target never turned (a neck folded behind
                    // the hands on the bench); there, any axis across f turns it.
                    let sine = cross(f, u);
                    let (s_norm, cosine) = (norm(sine), dot(f, u));
                    let angle = s_norm.atan2(cosine);
                    let e = if s_norm > 1e-9 {
                        scale(sine, angle / s_norm)
                    } else if cosine < 0.0 {
                        let across = if f[0].abs() < 0.9 {
                            [1.0, 0.0, 0.0]
                        } else {
                            [0.0, 1.0, 0.0]
                        };
                        let axis = cross(f, across);
                        scale(axis, angle / norm(axis))
                    } else {
                        [0.0; 3]
                    };
                    let e = clamp_norm(e, s.max_error_rad);
                    // Turn f toward u relative to u's own turn as the camera moves: the
                    // direction to the point turns at -(u x v) / distance, so the relative
                    // angular rate is w + (u x v) / distance; ask it for the error's rotation
                    // (its component along f, a roll, for none).
                    for k in 0..3 {
                        fill(&mut w.task, &body.solved, |axis| {
                            w.jw[axis][k] + cross(u, w.jv[axis])[k] / distance
                        });
                        least_squares(&mut w.h, &mut w.g, &w.task, e[k], rho2);
                    }
                }
                _ => {}
            }
        }
        for (column, &axis) in body.solved.iter().enumerate() {
            let weight = if self.shared[column] {
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
            w.safe_lower[column] = lo;
            w.safe_upper[column] = hi;
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
            w.lower[column] = lo;
            w.upper[column] = hi;
            w.lower_by[column] = lo_by;
            w.upper_by[column] = hi_by;
            w.yielded[column] = false;
        }
    }

    /// The closest pairs within influence, each as `row . dq <= rhs`: the pair closes no
    /// faster this tick than a braking share of its axes' accelerations can still stop
    /// before the margin.
    fn collision_rows(&mut self, dt: f64) {
        let body = &self.body;
        let s = &self.settings;
        let w = &mut self.work;
        let n = body.solved.len();
        w.candidates.clear();
        for &(i, j) in &body.proxies.pairs {
            let distance = capsule_distance(body, &w.frames, i, j).0;
            if distance < s.influence_m {
                w.candidates.push(Candidate {
                    distance,
                    pair: Pair::Capsules(i, j),
                });
            }
        }
        for &(capsule, k) in &self.scene_pairs {
            let c = &body.proxies.capsules[capsule];
            let (a, b) = world(&w.frames, c);
            let distance = segment_box(&self.scene[k], a, b).1 - c.radius;
            if distance < s.influence_m {
                w.candidates.push(Candidate {
                    distance,
                    pair: Pair::Scene(capsule, k),
                });
            }
        }
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
            let (distance, active) = match candidate.pair {
                Pair::Capsules(i, j) => {
                    let (d, x, y, direction) = capsule_distance(body, &w.frames, i, j);
                    let (p, c) = (&body.proxies.capsules[i], &body.proxies.capsules[j]);
                    point_row(body, &w.frames, &mut w.jp, p.link, x, direction, -1.0, row);
                    point_row(body, &w.frames, &mut w.jp, c.link, y, direction, 1.0, row);
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
                    let (a, b) = world(&w.frames, c);
                    let (point, d, direction) = segment_box(&self.scene[k], a, b);
                    point_row(
                        body, &w.frames, &mut w.jp, c.link, point, direction, -1.0, row,
                    );
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
            let rhs = braking(room, s.braking_share * capability, dt).min(0.5 * room);
            let reach: f64 = (0..n)
                .map(|j| (row[j] * w.lower[j]).max(row[j] * w.upper[j]))
                .sum();
            if reach <= rhs {
                continue; // no step inside the box can close it that far
            }
            w.rhs[slot] = rhs;
            w.row_active[slot] = active;
            w.used += 1;
        }
    }

    /// Give every row a step inside the box that keeps it: a row no step within the
    /// acceleration caps can keep widens its axes to their safe range (the acceleration cap
    /// yields to keeping clear). With `all`, every row's axes yield and every row asks only
    /// that its pair come no closer, which standing still keeps.
    fn keep_feasible(&mut self, all: bool) -> Solve {
        let n = self.body.solved.len();
        let w = &mut self.work;
        let mut solve = Solve::Solved;
        for slot in 0..w.used {
            let row = &w.rows[slot * n..(slot + 1) * n];
            let best: f64 = (0..n)
                .map(|j| (row[j] * w.lower[j]).min(row[j] * w.upper[j]))
                .sum();
            if all || best > w.rhs[slot] {
                for j in 0..n {
                    if row[j] != 0.0 && !w.yielded[j] {
                        w.lower[j] = w.safe_lower[j];
                        w.upper[j] = w.safe_upper[j];
                        w.yielded[j] = true;
                    }
                }
                solve = Solve::Relaxed;
            }
            if all {
                w.rhs[slot] = w.rhs[slot].max(0.0);
            }
        }
        for j in 0..n {
            w.still[j] = 0.0_f64.clamp(w.lower[j], w.upper[j]);
        }
        solve
    }

    /// Load the QP and solve; the iteration count, or `None` on failure.
    ///
    /// The QP sees `dq` in units of each axis's one-tick speed step (its box is then about
    /// [-1, 1]), unit-norm collision rows, and a cost scaled to a unit diagonal: the same
    /// problem, conditioned for an interior-point method whose data is otherwise 1e-4 m.
    fn solve_qp(&mut self, dt: f64, deadline: Option<Instant>) -> Option<u32> {
        let n = self.body.solved.len();
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
                qp.h[i * n + j] = c * w.h[i * n + j] * w.unit[i] * w.unit[j];
            }
            qp.g[i] = c * w.g[i] * w.unit[i];
            // A box closed to a point (a held axis, a forced deceleration) keeps a sliver of
            // interior for the interior-point method; the answer is clamped back to it.
            let (lo, hi) = (w.lower[i] / w.unit[i], w.upper[i] / w.unit[i]);
            let open = (1e-7 - (hi - lo)).max(0.0) / 2.0;
            qp.lower[i] = lo - open;
            qp.upper[i] = hi + open;
        }
        qp.used = w.used;
        for slot in 0..w.used {
            let mut length = 0.0;
            for j in 0..n {
                let v = w.rows[slot * n + j] * w.unit[j];
                qp.r[slot * n + j] = v;
                length += v * v;
            }
            let scale = if length > 0.0 {
                1.0 / length.sqrt()
            } else {
                0.0
            };
            for j in 0..n {
                qp.r[slot * n + j] *= scale;
            }
            qp.rhs[slot] = w.rhs[slot] * scale;
        }
        let iterations = qp.solve(self.settings.max_iterations, deadline).ok()?;
        for j in 0..n {
            w.answer[j] = qp.x[j] * w.unit[j];
        }
        Some(iterations)
    }

    /// Residuals, binding constraints and reach per limb, at the new positions.
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
            let blocked = report.active.iter().any(|a| {
                matches!(
                    a,
                    Active::Limit { .. } | Active::SelfCollision { .. } | Active::Obstacle { .. }
                )
            });
            let capped = report
                .active
                .iter()
                .any(|a| matches!(a, Active::Speed { .. } | Active::Accel { .. }));
            report.residual_m = residual_m;
            report.residual_rad = residual_rad;
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

/// The fastest per-tick step from which ticks decelerating by at most `accel` still stop
/// within `room`: `v dt + v^2 / 2a + v dt / 2 <= room`.
fn braking(room: f64, accel: f64, dt: f64) -> f64 {
    dt * accel * ((2.25 * dt * dt + 2.0 * room / accel).sqrt() - 1.5 * dt)
}

/// Signed distance between two capsules at `frames`, their closest axis points, and the unit
/// direction from the second to the first.
fn capsule_distance(body: &Body, frames: &Frames, i: usize, j: usize) -> (f64, V3, V3, V3) {
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
    (centres - p.radius - c.radius, x, y, direction)
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
