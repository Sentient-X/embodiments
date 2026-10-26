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
    Body, Caps, Ik, Obstacle, Reach, STATIC_UPPER_BODY_URDF, Settings, Solve, Target,
};

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

/// The table the base is clamped to: its top is the root frame's z = 0, behind the base edge.
fn table() -> Obstacle {
    Obstacle {
        min: [-1.0, -1.0, -0.8],
        max: [0.05, 1.0, 0.0],
    }
}

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
    Ik::new(body, caps, settings(), vec![table()]).expect("ik")
}

/// Ghost caps: fast enough that only reach, limits and collision matter.
fn ghost() -> Ik {
    let body = Body::static_upper_body();
    let mut caps = Caps::declared(&body, 0.25).expect("caps");
    for (speed, accel) in caps.speed.iter_mut().zip(caps.accel.iter_mut()) {
        *speed = 4.0;
        *accel = 400.0;
    }
    Ik::new(body, caps, settings(), vec![table()]).expect("ik")
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
        .map(|limb| {
            let pose = ik.body().frame(&frames, limb).to_transform();
            let p = sx_embodiment_drivers::kinematics::pose_of(&pose);
            Some(Target::Pose {
                translation_m: p.translation_m,
                rotation_xyzw: p.rotation_xyzw,
            })
        })
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
            for (x, y) in a.to_transform().iter().zip(b) {
                assert!((x - y).abs() < 1e-12, "{x} != {y}");
            }
        }
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
                    Some(Target::Pose {
                        translation_m: [
                            rng.uniform(-1.0, 1.0),
                            rng.uniform(-1.0, 1.0),
                            rng.uniform(-0.6, 1.0),
                        ],
                        rotation_xyzw: [
                            rng.uniform(-1.0, 1.0),
                            rng.uniform(-1.0, 1.0),
                            rng.uniform(-1.0, 1.0),
                            rng.uniform(-1.0, 1.0),
                        ],
                    })
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
    let far = Some(Target::Pose {
        translation_m: [1.5, 0.3, 0.2],
        rotation_xyzw: [0.0, 0.0, 0.0, 1.0],
    });
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
