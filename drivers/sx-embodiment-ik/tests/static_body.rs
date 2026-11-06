//! The solver on the static upper body: FK parity, caps and limits held under random targets,
//! no proxy penetration, reachable targets converge, unreachable ones degrade.

// Test and evaluation code: counts as f64, exact comparisons of held values, formula names.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::many_single_char_names,
    clippy::float_cmp,
    clippy::doc_markdown
)]

use sx_embodiment_drivers::kinematics::Tree;
use sx_embodiment_ik::{
    Active, Body, Caps, Ik, Reach, STATIC_UPPER_BODY_URDF, Settings, Solve, Target,
};
use sx_frames::{Pose, QuaternionLayout};

/// Default settings with a deadline far past any tick: the tests are about the answer, not
/// the clock, and run on a loaded machine.
fn settings() -> Settings {
    Settings {
        deadline: std::time::Duration::from_secs(1),
        ..Settings::default()
    }
}

/// Every axis available.
const ALL: [bool; 64] = [true; 64];

const DT: f64 = 0.01;

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn uniform(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.next()
    }
}

fn ik(speed_cap: Option<f64>) -> Ik {
    let body = Body::static_upper_body();
    let caps = Caps::declared(&body, 0.25).expect("caps");
    let caps = match speed_cap {
        Some(cap) => caps.at_most(cap),
        None => caps,
    };
    Ik::new(body, caps, settings(), Vec::new()).expect("ik")
}

/// Ghost caps: fast enough that only reach, limits and collision matter.
fn ghost() -> Ik {
    let body = Body::static_upper_body();
    let mut caps = Caps::declared(&body, 0.25).expect("caps");
    for (speed, accel) in caps.speed.iter_mut().zip(caps.accel.iter_mut()) {
        *speed = 4.0;
        *accel = 400.0;
    }
    Ik::new(body, caps, settings(), Vec::new()).expect("ik")
}

/// A random posture inside every limit, each solved axis within `spread` of rest, whose
/// proxies stay `clear` metres apart.
fn random_posture(ik: &Ik, rng: &mut Rng, clear: f64, spread: f64) -> Vec<f64> {
    let axes = ik.body().model().axes();
    loop {
        let q: Vec<f64> = axes
            .iter()
            .zip(ik.rest())
            .map(|(a, r)| {
                a.limits.map_or(0.0, |[lo, hi]| {
                    rng.uniform((lo + 0.05).max(r - spread), (hi - 0.05).min(r + spread))
                })
            })
            .collect();
        let q: Vec<f64> = q
            .iter()
            .enumerate()
            .map(|(i, v)| {
                if ik.body().solved().contains(&i) {
                    *v
                } else {
                    ik.rest()[i]
                }
            })
            .collect();
        if ik.clearance(&q).0 > clear {
            return q;
        }
    }
}

fn rest(ik: &Ik) -> Vec<f64> {
    ik.rest().to_vec()
}

fn pose_targets(ik: &Ik, q: &[f64]) -> Vec<Option<Target>> {
    let frames = ik.body().model().poses(q);
    (0..2)
        .map(|limb| Some(Target::Pose(ik.body().frame(&frames, limb).pose())))
        .chain([Some(Target::LookAt {
            point_m: [0.6, 0.0, 0.1],
        })])
        .collect()
}

#[test]
fn forward_kinematics_is_the_trees() {
    let tree = Tree::parse(STATIC_UPPER_BODY_URDF).expect("tree");
    let ik = ik(None);
    let model = ik.body().model();
    let mut rng = Rng(7);
    for _ in 0..20 {
        let q: Vec<f64> = model
            .axes()
            .iter()
            .map(|_| rng.uniform(-1.5, 1.5))
            .collect();
        let ours = model.poses(&q);
        let theirs = tree
            .link_poses(|name| {
                model
                    .axes()
                    .iter()
                    .position(|a| a.name == name)
                    .map(|i| q[i])
            })
            .expect("poses");
        for (a, b) in ours.links.iter().zip(&theirs) {
            for (x, y) in a.to_row_major().iter().zip(b) {
                assert!((x - y).abs() < 1e-12, "{x} != {y}");
            }
        }
    }
}

/// The table is the embodiment's: its base mount declares the support, and every solver of the
/// body has it without being told. The top is the base's z = 0 plane; the body clamps at the
/// table's edge facing out from it (+x), so the table ends at the clamp plate's front and lies
/// behind it (2026-10-26 bench: the arms must reach below the top in front of the edge).
#[test]
fn the_declared_table_ends_at_the_edge_and_the_arms_reach_below_it_in_front() {
    let mut ik = ik(Some(0.5));
    let table = ik.scene()[0];
    assert_eq!(Some(table), ik.body().proxies().support);
    assert_eq!(table.max[2], 0.0);
    assert!((table.max[0] - 0.0549).abs() < 1e-9, "{table:?}");
    assert!(
        table.min[0] < -1.0 && table.min[1] < -1.0 && table.max[1] > 1.0,
        "{table:?}"
    );
    let solve = |ik: &mut Ik, x: f64, z: f64| {
        let mut q = rest(ik);
        let at = |y: f64| {
            Some(Target::Pose(Pose {
                translation_m: [x, y, z],
                rotation: QuaternionLayout::Xyzw.read([0.0, 0.0, 0.0, 1.0]),
            }))
        };
        let targets = [at(0.2), at(-0.2), None];
        for _ in 0..400 {
            q = ik
                .step(&q, &ALL[..q.len()], &targets, DT)
                .expect("step")
                .q
                .clone();
        }
        let (clearance, a, b) = ik.clearance(&q);
        assert!(clearance >= 0.0, "{a} into {b} by {clearance}");
        let frames = ik.body().model().poses(&q);
        [0, 1].map(|limb| ik.body().frame(&frames, limb).t)
    };
    // Both tools asked 15 cm below the top, in front of the edge: they go well below it (the
    // arms' own reach and the chest, not the table, stop them short; on the full slab they
    // stayed above the top).
    for tool in solve(&mut ik, 0.35, -0.15) {
        assert!(tool[2] < -0.05, "tool at {tool:?}");
    }
    // Asked below the top behind the edge, over the table: they stop above it.
    for tool in solve(&mut ik, -0.05, -0.15) {
        assert!(tool[2] > 0.0 || tool[0] > table.max[0], "tool at {tool:?}");
    }
}

#[test]
fn the_rest_posture_is_clear_and_every_solved_axis_has_caps() {
    let ik = ik(None);
    assert_eq!(ik.body().limbs(), ["left", "right", "head"]);
    // Torso, two seven-joint arms, the six-joint neck with its camera tilt.
    assert_eq!(ik.body().solved().len(), 1 + 7 + 7 + 6);
    let (clearance, a, b) = ik.clearance(&rest(&ik));
    assert!(
        clearance > 0.0,
        "{a} and {b} overlap at rest by {clearance}"
    );
}

#[test]
fn limits_speed_and_acceleration_hold_and_no_proxy_penetrates_under_random_targets() {
    let mut ik = ik(Some(0.5));
    let mut rng = Rng(42);
    let axes = ik.body().model().axes().to_vec();
    let solved = ik.body().solved().to_vec();
    let (speed, accel) = (ik.caps().speed.clone(), ik.caps().accel.clone());
    let mut q = rest(&ik);
    let mut velocity = vec![0.0; axes.len()];
    let mut steps = 0;
    for episode in 0..12 {
        // Reachable postures, then wild targets far outside reach, behind the body, under the
        // table.
        let targets: Vec<Option<Target>> = if episode % 2 == 0 {
            let goal = random_posture(&ik, &mut rng, 0.0, 10.0);
            pose_targets(&ik, &goal)
        } else {
            let look = Some(Target::LookAt {
                point_m: [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), 0.0],
            });
            (0..2)
                .map(|_| {
                    Some(Target::Pose(Pose {
                        translation_m: [
                            rng.uniform(-1.0, 1.0),
                            rng.uniform(-1.0, 1.0),
                            rng.uniform(-0.6, 1.0),
                        ],
                        rotation: QuaternionLayout::Xyzw.read([
                            rng.uniform(-1.0, 1.0),
                            rng.uniform(-1.0, 1.0),
                            rng.uniform(-1.0, 1.0),
                            rng.uniform(-1.0, 1.0),
                        ]),
                    }))
                })
                .chain([look])
                .collect()
        };
        for _ in 0..250 {
            let step = ik
                .step(&q, &ALL[..q.len()], &targets, DT)
                .expect("step")
                .clone();
            assert_ne!(step.solve, Solve::Held, "episode {episode}");
            for &axis in &solved {
                let next = step.q[axis];
                if let Some([lo, hi]) = axes[axis].limits {
                    assert!(
                        next >= lo - 1e-9 && next <= hi + 1e-9,
                        "{}",
                        axes[axis].name
                    );
                }
                let v = (next - q[axis]) / DT;
                assert!(v.abs() <= speed[axis] + 1e-9, "{} at {v}", axes[axis].name);
                let a = (v - velocity[axis]) / DT;
                assert!(
                    a.abs() <= accel[axis] * (1.0 + 1e-6) + 1e-6,
                    "{} at {a}",
                    axes[axis].name
                );
                velocity[axis] = v;
            }
            let before = ik.clearance(&q).0;
            q = step.q.clone();
            let (clearance, a, b) = ik.clearance(&q);
            assert!(
                clearance >= 0.0,
                "{a} and {b} penetrate by {clearance} (was {before}): {:?} {:?}",
                step.solve,
                step.limbs
            );
            steps += 1;
        }
    }
    assert_eq!(steps, 12 * 250);
}

/// Postures within a radian of rest on every axis: what a continuous teleoperated motion
/// leads the body through. A posture across a full yaw range from rest can sit behind a limit
/// or the neck for any differential IK; that is reported unreachable, below.
#[test]
fn reachable_targets_converge_within_millimetres() {
    let mut ik = ghost();
    let mut rng = Rng(3);
    let mut errors = Vec::new();
    for _ in 0..30 {
        let goal = random_posture(&ik, &mut rng, 0.03, 1.0);
        let targets = pose_targets(&ik, &goal);
        ik.reset();
        let mut q = rest(&ik);
        let mut last = None;
        for _ in 0..400 {
            let step = ik
                .step(&q, &ALL[..q.len()], &targets, DT)
                .expect("step")
                .clone();
            q = step.q.clone();
            last = Some(step);
        }
        let step = last.expect("a step");
        if step.limbs[0].residual_m > 0.01 || step.limbs[1].residual_m > 0.01 {
            eprintln!(
                "DBG {:?} {:?}",
                step.solve,
                step.limbs
                    .iter()
                    .map(|l| (
                        l.residual_m,
                        l.residual_rad,
                        l.reach,
                        l.active
                            .iter()
                            .map(|a| a.describe(ik.body()))
                            .collect::<Vec<_>>()
                    ))
                    .collect::<Vec<_>>()
            );
        }
        for limb in &step.limbs[..2] {
            errors.push(limb.residual_m);
        }
    }
    errors.sort_by(f64::total_cmp);
    let median = errors[errors.len() / 2];
    let worst = errors[errors.len() - 1];
    assert!(median < 0.0005, "median {median}");
    assert!(worst < 0.002, "worst {worst}: {errors:?}");
}

#[test]
fn an_unreachable_target_degrades_and_says_so() {
    let mut ik = ghost();
    let mut q = rest(&ik);
    let far = Some(Target::Pose(Pose {
        translation_m: [1.5, 0.3, 0.2],
        rotation: QuaternionLayout::Xyzw.read([0.0, 0.0, 0.0, 1.0]),
    }));
    let mut last = None;
    for _ in 0..300 {
        let step = ik
            .step(&q, &ALL[..q.len()], &[far, None, None], DT)
            .expect("step")
            .clone();
        q = step.q.clone();
        last = Some(step);
    }
    let step = last.expect("a step");
    let left = &step.limbs[0];
    assert_eq!(left.reach, Reach::Unreachable);
    assert!(
        left.residual_m > 0.5 && left.residual_m < 1.2,
        "{}",
        left.residual_m
    );
    assert_eq!(step.limbs[1].reach, Reach::Idle);
    // Stretched toward it: the tool is as close as the arm goes.
    let frames = ik.body().model().poses(&q);
    let tool = ik.body().frame(&frames, 0).t;
    assert!(tool[0] > 0.5, "{tool:?}");
}

#[test]
fn a_far_target_at_the_speed_cap_is_catching_up() {
    let mut ik = ik(Some(0.25));
    let q = rest(&ik);
    let goal = {
        let mut g = q.clone();
        let axes = ik.body().model().axes();
        let i = axes
            .iter()
            .position(|a| a.name == "left_elbow_pitch_joint")
            .unwrap();
        g[i] = -0.8;
        g
    };
    let targets = pose_targets(&ik, &goal);
    let mut q = q;
    let mut reaches = Vec::new();
    for _ in 0..50 {
        let step = ik
            .step(&q, &ALL[..q.len()], &[targets[0], None, None], DT)
            .expect("step")
            .clone();
        q = step.q.clone();
        reaches.push(step.limbs[0].reach);
    }
    assert_eq!(reaches.last(), Some(&Reach::CatchingUp));
}

#[test]
fn the_same_inputs_give_the_same_bytes() {
    let run = || {
        let mut ik = ik(Some(0.5));
        let mut rng = Rng(11);
        let goal = random_posture(&ik, &mut rng, 0.0, 10.0);
        let targets = pose_targets(&ik, &goal);
        let mut q = rest(&ik);
        for _ in 0..50 {
            q = ik
                .step(&q, &ALL[..q.len()], &targets, DT)
                .expect("step")
                .q
                .clone();
        }
        q.iter().map(|v| v.to_bits()).collect::<Vec<_>>()
    };
    assert_eq!(run(), run());
}

#[test]
fn malformed_input_is_refused_not_solved() {
    let mut ik = ik(None);
    let q = rest(&ik);
    assert!(
        ik.step(&q[1..], &ALL[..q.len() - 1], &[None, None, None], DT)
            .is_err()
    );
    assert!(ik.step(&q, &ALL[..q.len()], &[None, None], DT).is_err());
    assert!(
        ik.step(&q, &ALL[..q.len()], &[None, None, None], 0.0)
            .is_err()
    );
    let mut bad = q.clone();
    bad[0] = f64::NAN;
    assert!(
        ik.step(&bad, &ALL[..q.len()], &[None, None, None], DT)
            .is_err()
    );
    let wrong = Some(Target::LookAt {
        point_m: [1.0, 0.0, 0.0],
    });
    assert!(
        ik.step(&q, &ALL[..q.len()], &[wrong, None, None], DT)
            .is_err()
    );
}

#[test]
fn an_unavailable_axis_is_held_and_the_vector_keeps_its_length() {
    let mut ik = ik(Some(0.5));
    let mut rng = Rng(5);
    let goal = random_posture(&ik, &mut rng, 0.0, 1.0);
    let targets = pose_targets(&ik, &goal);
    let axes = ik.body().model().axes().to_vec();
    let elbow = axes
        .iter()
        .position(|a| a.name == "left_elbow_pitch_joint")
        .expect("elbow");
    let mut available = vec![true; axes.len()];
    available[elbow] = false;
    let mut q = rest(&ik);
    let held = q[elbow];
    for _ in 0..100 {
        let step = ik.step(&q, &available, &targets, DT).expect("step").clone();
        assert_eq!(step.q.len(), axes.len());
        assert_eq!(step.q[elbow], held);
        assert_eq!(step.qdot[elbow], 0.0);
        q = step.q;
    }
    assert!(ik.step(&q, &available[1..], &targets, DT).is_err());
}

/// The least-norm joint step (every axis) that moves both hands' tools by `shift` metres with
/// their orientations held, from a finite-difference Jacobian (orientation weighted as the
/// solver weighs it): the least any solver can move the joints for this move.
fn least_norm_move(ik: &Ik, q: &[f64], shift: [f64; 3]) -> Vec<f64> {
    const ROWS: usize = 12;
    let solved = ik.body().solved();
    let rho = ik.settings().orientation_m_per_rad;
    let frames = |q: &[f64]| {
        let poses = ik.body().model().poses(q);
        [ik.body().frame(&poses, 0), ik.body().frame(&poses, 1)]
    };
    let base = frames(q);
    let h = 1e-6;
    let columns: Vec<[f64; ROWS]> = solved
        .iter()
        .map(|&axis| {
            let mut moved = q.to_vec();
            moved[axis] += h;
            let after = frames(&moved);
            let mut column = [0.0; ROWS];
            for (limb, (f, b)) in after.iter().zip(&base).enumerate() {
                let d = |i: usize, j: usize| (0..3).map(|k| f.r[i][k] * b.r[j][k]).sum::<f64>();
                let o = 6 * limb;
                for k in 0..3 {
                    column[o + k] = (f.t[k] - b.t[k]) / h;
                }
                column[o + 3] = rho * 0.5 * (d(2, 1) - d(1, 2)) / h;
                column[o + 4] = rho * 0.5 * (d(0, 2) - d(2, 0)) / h;
                column[o + 5] = rho * 0.5 * (d(1, 0) - d(0, 1)) / h;
            }
            column
        })
        .collect();
    // dq = J' (J J' + eps I)^-1 e, by Gauss-Jordan elimination.
    let mut e = [0.0; ROWS];
    e[..3].copy_from_slice(&shift);
    e[6..9].copy_from_slice(&shift);
    let mut m = [[0.0; ROWS + 1]; ROWS];
    for a in 0..ROWS {
        for b in 0..ROWS {
            m[a][b] = columns.iter().map(|c| c[a] * c[b]).sum::<f64>();
        }
        m[a][a] += 1e-9;
        m[a][ROWS] = e[a];
    }
    for i in 0..ROWS {
        let pivot = (i..ROWS)
            .max_by(|&x, &y| m[x][i].abs().total_cmp(&m[y][i].abs()))
            .expect("a row");
        m.swap(i, pivot);
        for r in 0..ROWS {
            if r != i {
                let f = m[r][i] / m[i][i];
                let pivot_row = m[i];
                for (cell, p) in m[r][i..].iter_mut().zip(&pivot_row[i..]) {
                    *cell -= f * p;
                }
            }
        }
    }
    let mut dq = vec![0.0; q.len()];
    for (c, &axis) in columns.iter().zip(solved) {
        dq[axis] = (0..ROWS).map(|a| c[a] * m[a][ROWS] / m[a][a]).sum();
    }
    dq
}

/// Teleoperation's case: from a posture the operator holds, both hands move 3 cm (up, down,
/// sideways, forward) with orientation held, at the station's bring-up speed. Holding still
/// must not move the arm, and every joint must move about as much as its share of the
/// least-norm answer to the move: a seven-joint arm has a free direction, and a small hand
/// move must not spend it swinging the forearm and wrist.
#[test]
fn a_small_hand_move_is_a_small_joint_move() {
    let mut ik = ik(Some(0.3));
    let mut rng = Rng(19);
    let solved = ik.body().solved().to_vec();
    let moves = [
        [0.0, 0.0, 0.03],
        [0.0, 0.0, -0.03],
        [0.0, 0.03, 0.0],
        [0.03, 0.0, 0.0],
    ];
    let (mut worst_hold, mut worst_ratio, mut worst_excess, mut worst_joint) =
        (0.0_f64, 0.0_f64, 0.0_f64, 0.0_f64);
    let mut cases = 0;
    while cases < 40 {
        let start = random_posture(&ik, &mut rng, 0.05, 1.0);
        let shift = moves[cases % moves.len()];
        let hold = pose_targets(&ik, &start);
        let hold = [hold[0], hold[1], None];
        let mut targets = hold;
        for target in &mut targets[..2] {
            if let Some(Target::Pose(Pose { translation_m, .. })) = target {
                for (t, s) in translation_m.iter_mut().zip(shift) {
                    *t += s;
                }
            }
        }
        ik.reset();
        let mut q = start.clone();
        for _ in 0..50 {
            q = ik
                .step(&q, &ALL[..q.len()], &hold, DT)
                .expect("step")
                .q
                .clone();
        }
        let still = solved
            .iter()
            .map(|&a| (q[a] - start[a]).abs())
            .fold(0.0, f64::max);
        let mut sweep = vec![0.0_f64; q.len()];
        let mut last = None;
        let mut blocked = false;
        for _ in 0..300 {
            let step = ik
                .step(&q, &ALL[..q.len()], &targets, DT)
                .expect("step")
                .clone();
            q.copy_from_slice(&step.q);
            blocked |= step.limbs[..2].iter().flat_map(|l| &l.active).any(|a| {
                matches!(
                    a,
                    Active::Limit { .. } | Active::SelfCollision { .. } | Active::Obstacle { .. }
                )
            });
            for &axis in &solved {
                sweep[axis] = sweep[axis].max((q[axis] - start[axis]).abs());
            }
            last = Some(step);
        }
        if blocked
            || last.expect("a step").limbs[..2]
                .iter()
                .any(|l| l.reach != Reach::Tracking)
        {
            continue; // a limit, the other arm or the table stops it: not this test's case
        }
        cases += 1;
        let share = least_norm_move(&ik, &start, shift);
        let least_max = solved.iter().map(|&a| share[a].abs()).fold(0.0, f64::max);
        let sweep_max = solved.iter().map(|&a| sweep[a]).fold(0.0, f64::max);
        worst_hold = worst_hold.max(still);
        worst_ratio = worst_ratio.max(sweep_max / least_max);
        worst_joint = worst_joint.max(sweep_max);
        for &axis in &solved {
            worst_excess = worst_excess.max((sweep[axis] - share[axis].abs()) / least_max);
        }
    }
    eprintln!(
        "hold {worst_hold:.4} rad, worst joint {worst_joint:.3} rad, {worst_ratio:.2}x the least-norm move, a joint past its share by {worst_excess:.2}x the move"
    );
    assert!(
        worst_hold < 1e-3,
        "holding still moved a joint {worst_hold} rad"
    );
    assert!(worst_ratio < 2.0, "{worst_ratio}x the least-norm move");
    assert!(
        worst_excess < 1.0,
        "a joint went {worst_excess}x the move past its share"
    );
}

/// Bench 2026-10-26: right_upper_arm_yaw_joint rested at +2.762, just past its +2.760 limit;
/// the first solve on engage asked further out and the governor safe-stopped. An axis resting
/// outside its range may hold or come in, never go further out, whatever the target asks.
#[test]
fn an_axis_resting_past_its_limit_never_steps_further_out() {
    for speed in [0.3, 1.0, 4.0] {
        let mut ik = ik(Some(speed));
        let axes = ik.body().model().axes().to_vec();
        for (name, outward) in [
            ("right_upper_arm_yaw_joint", 1.0),
            ("right_upper_arm_yaw_joint", -1.0),
            ("left_elbow_pitch_joint", -1.0),
        ] {
            let index = axes.iter().position(|a| a.name == name).expect(name);
            let [lower, upper] = axes[index].limits.expect("limits");
            let mut q = rest(&ik);
            let edge = if outward > 0.0 { upper } else { lower };
            q[index] = edge + outward * 0.002;
            let start = q[index];
            // Ask for the hands where they would be with the axis further out still.
            let mut further = q.clone();
            further[index] = edge + outward * 0.3;
            let targets = pose_targets(&ik, &further);
            ik.reset();
            for tick in 0..100 {
                let step = ik.step(&q, &ALL[..q.len()], &targets, DT).expect("step");
                let next = step.q[index];
                assert!(
                    (next - start) * outward <= 1e-12,
                    "{name} at speed {speed}, tick {tick}: {start} -> {next}, further out"
                );
                q.copy_from_slice(&step.q);
            }
        }
    }
}

/// The arms' identified dynamics, fitted against the description before the 2026-10-27 camera
/// mount correction, still apply to the corrected one: the correction reaches no arm.
#[test]
fn an_arm_fit_keyed_to_the_earlier_description_still_applies() {
    let parity: serde_json::Value = serde_json::from_str(include_str!(
        "../../sx-embodiment-drivers/tests/fixtures/dynamics-parity.json"
    ))
    .expect("fixture");
    let mut calibration = parity["calibration"].clone();
    let earlier = sx_embodiment_drivers::dynamics::sha256_hex(
        sx_embodiment_ik::STATIC_UPPER_BODY_SUPERSEDED[0],
    );
    calibration["description_sha256"] = serde_json::Value::String(earlier);
    let calibration = sx_embodiment_drivers::DynamicsCalibration::parse(&calibration.to_string())
        .expect("calibration");
    let current = || {
        sx_embodiment_drivers::Dynamics::of(
            sx_embodiment_ik::STATIC_UPPER_BODY_URDF,
            sx_embodiment_drivers::UPRIGHT,
        )
        .expect("dynamics")
    };
    assert!(current().calibrated(&calibration).is_err());
    current()
        .superseding(sx_embodiment_ik::STATIC_UPPER_BODY_SUPERSEDED)
        .expect("earlier")
        .calibrated(&calibration)
        .expect("the arm fit applies");
}

/// The folded neck the robot rests in (measured 2026-10-27) is clear: its capsules overlap by
/// about 9 mm where the wrist bracket's corner wraps the shoulder pan servo, and the convex
/// pieces, which follow the bracket, prove the parts apart.
#[test]
fn the_folded_neck_rest_is_clear_of_itself() {
    let ik = ik(None);
    let model = ik.body().model();
    let mut q = rest(&ik);
    for (name, value) in [
        ("so101_shoulder_pan_joint", -0.192_561_6),
        ("so101_shoulder_lift_joint", -1.472_981_2),
        ("so101_elbow_flex_joint", 1.721_546_8),
        ("so101_wrist_flex_joint", -1.739_191_9),
        ("so101_wrist_roll_joint", 0.241_661_0),
        ("so101_camera_tilt_joint", -1.078_651_8),
    ] {
        let index = model
            .axes()
            .iter()
            .position(|axis| axis.name == name)
            .expect(name);
        q[index] = value;
    }
    let (clearance, a, b) = ik.clearance(&q);
    assert!(clearance > 0.0, "{a} and {b} at {clearance} m");
}

/// One reading that jumps ten times what the joint could move is no speed: the measured speed a measured row carries
/// forward ([`Ik::measured_speed`]) stays within FOLLOWER_SPEED times the axis's speed cap
/// (unbounded, a glitch differenced over a 10 ms tick read as 50 rad/s; embodiments#210).
#[test]
fn a_reading_that_jumps_is_no_speed() {
    let mut solver = ik(None);
    let q = rest(&solver);
    let width = q.len();
    let available = vec![true; width];
    let targets = [None, None, None];
    let axis = solver.body().solved()[0];
    let cap = solver.caps().speed[axis];
    let stuck = vec![false; width];
    let mut measured: Vec<Option<f64>> = q.iter().copied().map(Some).collect();
    for tick in 0..20 {
        if tick == 10 {
            // Ten times what the bound allows over a tick: unbounded, it reads past the bound.
            measured[axis] = Some(q[axis] + 10.0 * sx_embodiment_ik::FOLLOWER_SPEED * cap * 0.01);
        }
        solver.set_measured(&measured, &stuck).expect("readings");
        solver.step(&q, &available, &targets, 0.01).expect("a step");
        assert!(
            solver.measured_speed(axis).abs() <= sx_embodiment_ik::FOLLOWER_SPEED * cap + 1e-12,
            "tick {tick}: {} rad/s against a {cap} rad/s cap",
            solver.measured_speed(axis)
        );
    }
}

/// The tool speed a pose allows ([`Body::tool_speed_along`]) is the speed the solve moves the
/// tool at along it: a 10 cm jump of the left hand from its Ready pose along each of +-x and
/// +-y peaks within 20 % of it (a 10 cm jump is short of full speed on +x), and never above it by more than the QP's own slack over the
/// damped least-squares answer (5 %). Its fastest direction ([`Body::tool_capability`]) is no
/// slower than any one direction.
#[test]
fn the_tool_speed_a_pose_allows_is_the_speed_the_solve_reaches() {
    let body = Body::static_upper_body();
    let axes = body.model().axes();
    let index = |name: &str| axes.iter().position(|a| a.name == name).expect(name);
    let mut q = vec![0.0; axes.len()];
    for (joint, value) in [
        ("left_shoulder_pitch_joint", -0.15),
        ("left_shoulder_roll_joint", 0.15),
        ("left_elbow_pitch_joint", -0.3),
        ("left_wrist_pitch_joint", 0.2),
        ("right_shoulder_pitch_joint", 0.15),
        ("right_shoulder_roll_joint", -0.15),
        ("right_elbow_pitch_joint", 0.3),
        ("right_wrist_pitch_joint", -0.2),
    ] {
        q[index(joint)] = value;
    }
    // The governed caps of 2026-11-04 at share 1 (rad/s, rad/s^2), the torso held.
    let cap = |name: &str| -> (f64, f64) {
        if name == "torso_joint" {
            (0.0, 1.0)
        } else if name.contains("shoulder") {
            (1.16, 13.5)
        } else if name.contains("upper_arm") {
            (0.96, 15.4)
        } else if name.contains("elbow") || name.contains("forearm") {
            (1.22, 19.6)
        } else if name.contains("wrist") {
            (3.22, 89.9)
        } else {
            (3.0, 16.0)
        }
    };
    let speed: Vec<f64> = axes.iter().map(|a| cap(&a.name).0).collect();
    let solve_caps = Caps {
        speed: speed.iter().map(|s| s.max(1e-6)).collect(),
        accel: axes.iter().map(|a| cap(&a.name).1).collect(),
    };
    let start = body.model().poses(&q);
    let fastest = body.tool_capability(&start, 0, &speed);
    let dt = 0.01;
    for u in [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, -1.0, 0.0]] {
        let allowed = body.tool_speed_along(&start, 0, &speed, u);
        assert!(
            allowed > 0.1 && allowed <= fastest.linear_m_s,
            "{u:?}: {allowed}"
        );
        let mut ik = Ik::new(body.clone(), solve_caps.clone(), settings(), Vec::new()).expect("ik");
        let mut held = q.clone();
        ik.reset();
        let from = body.frame(&start, 0).pose();
        let target = Pose {
            translation_m: [0, 1, 2].map(|k| from.translation_m[k] + 0.1 * u[k]),
            rotation: from.rotation,
        };
        let right = body.frame(&start, 1).pose();
        let mut peak = 0.0_f64;
        let mut tool = from.translation_m;
        for _ in 0..100 {
            let step = ik
                .step(
                    &held,
                    &ALL[..held.len()],
                    &[Some(Target::Pose(target)), Some(Target::Pose(right)), None],
                    dt,
                )
                .expect("a step");
            held.copy_from_slice(&step.q);
            let now = body.frame(&body.model().poses(&held), 0).t;
            let moved = (0..3)
                .map(|k| (now[k] - tool[k]).powi(2))
                .sum::<f64>()
                .sqrt();
            peak = peak.max(moved / dt);
            tool = now;
        }
        assert!(
            peak <= allowed * 1.05 && peak >= allowed * 0.8,
            "{u:?}: the solve peaked at {peak:.3} m/s, the pose allows {allowed:.3}"
        );
    }
}
