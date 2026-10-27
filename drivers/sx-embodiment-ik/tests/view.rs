//! The camera's view on the static upper body: it frames both hands, turns on its own neck
//! only, holds steady while the hands move, and says which link hides a hand.

#![allow(
    clippy::cast_precision_loss,
    clippy::float_cmp,
    clippy::needless_range_loop
)]

use std::collections::BTreeMap;

use sx_embodiment_ik::teleop::{Pose, TELEOP_SCHEMA, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{
    Body, Caps, Ik, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF, Settings, Target,
};

const DT: f64 = 0.01;

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

fn teleop() -> Teleop {
    let body = Body::static_upper_body();
    let speed: BTreeMap<String, f64> = body
        .model()
        .axes()
        .iter()
        .map(|axis| (axis.name.clone(), 1.0))
        .collect();
    let config = TeleopConfig {
        schema: TELEOP_SCHEMA.into(),
        limbs: TeleopConfig::static_upper_body_limbs(),
        speed,
        accel: BTreeMap::new(),
        ramp_s: 0.25,
        max_step_rad: 0.03,
        held: Vec::new(),
        obstacles: Vec::new(),
    };
    Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).expect("solve")
}

/// Ghost caps: only reach, limits and collision matter.
fn ghost() -> Ik {
    let body = Body::static_upper_body();
    let mut caps = Caps::declared(&body, 0.25).expect("caps");
    for (speed, accel) in caps.speed.iter_mut().zip(caps.accel.iter_mut()) {
        *speed = 4.0;
        *accel = 400.0;
    }
    let settings = Settings {
        deadline: std::time::Duration::from_secs(1),
        ..Settings::default()
    };
    Ik::new(body, caps, settings, Vec::new()).expect("ik")
}

fn zero(teleop: &Teleop) -> Vec<f64> {
    teleop
        .model()
        .axes()
        .iter()
        .map(|a| a.limits.map_or(0.0, |[l, u]| 0.0_f64.clamp(l, u)))
        .collect()
}

fn run(teleop: &mut Teleop, q: &mut [f64], targets: &Targets, ticks: usize) {
    let measured = vec![None; q.len()];
    let available = vec![true; q.len()];
    for _ in 0..ticks {
        let step = teleop
            .step(q, &measured, &available, targets, DT)
            .expect("a step");
        q.copy_from_slice(&step.q);
    }
}

/// The camera's frame columns (forward, across, up) and its position.
fn camera(body: &Body, q: &[f64]) -> ([[f64; 3]; 3], [f64; 3]) {
    let head = body
        .limbs()
        .iter()
        .position(|l| *l == "head")
        .expect("head");
    let frame = body.frame(&body.model().poses(q), head);
    let column = |c: usize| [frame.r[0][c], frame.r[1][c], frame.r[2][c]];
    ([column(0), column(1), column(2)], frame.t)
}

fn tcp(body: &Body, q: &[f64], hand: usize) -> Pose {
    let frame = body.frame(&body.model().poses(q), hand).to_transform();
    let p = sx_embodiment_drivers::kinematics::pose_of(&frame);
    Pose {
        translation_m: p.translation_m,
        rotation_xyzw: p.rotation_xyzw,
    }
}

fn angle(forward: [f64; 3], from: [f64; 3], to: [f64; 3]) -> f64 {
    let d = [0, 1, 2].map(|k| to[k] - from[k]);
    let length = d.iter().map(|v| v * v).sum::<f64>().sqrt();
    let cos: f64 = (0..3).map(|k| forward[k] * d[k] / length).sum();
    cos.clamp(-1.0, 1.0).acos()
}

#[test]
fn both_hands_sit_the_same_angle_from_the_centre_however_far_each_is() {
    let mut solve = teleop();
    let mut q = zero(&solve);
    let body = solve.ik().body().clone();
    let (left, right) = (tcp(&body, &q, 0), tcp(&body, &q, 1));
    // The left hand's target pulled toward the camera: the midpoint would frame it off-centre.
    let (_, eye) = camera(&body, &q);
    let mut near = left;
    for k in 0..3 {
        near.translation_m[k] += 0.35 * (eye[k] - left.translation_m[k]);
    }
    let targets = Targets {
        tcp: [Some(near), Some(right)],
        ..Targets::default()
    };
    solve.reset();
    run(&mut solve, &mut q, &targets, 600);
    let ([forward, ..], eye) = camera(&body, &q);
    let a = angle(forward, eye, near.translation_m);
    let b = angle(forward, eye, right.translation_m);
    assert!(
        (a - b).abs() < 0.03,
        "hands {:.1} and {:.1} degrees off centre",
        a.to_degrees(),
        b.to_degrees()
    );
}

#[test]
fn only_the_neck_turns_the_camera() {
    let mut solve = teleop();
    let mut q = zero(&solve);
    let body = solve.ik().body().clone();
    let begun = q.clone();
    // Both hands held where they are and a gaze well to the side and up: on main the torso
    // (it carries both arms) turned 9 mrad with the gaze, swinging both hands.
    let (left, right) = (tcp(&body, &q, 0), tcp(&body, &q, 1));
    let (yaw, pitch) = (0.6_f64, -0.3_f64);
    let (cy, sy, cp, sp) = (
        (yaw / 2.0).cos(),
        (yaw / 2.0).sin(),
        (pitch / 2.0).cos(),
        (pitch / 2.0).sin(),
    );
    let gaze = Pose {
        translation_m: [0.0, 0.0, 1.0],
        rotation_xyzw: [-sy * sp, cy * sp, sy * cp, cy * cp],
    };
    let targets = Targets {
        tcp: [Some(left), Some(right)],
        head: Some(gaze),
        ..Targets::default()
    };
    solve.reset();
    run(&mut solve, &mut q, &targets, 600);
    let ([forward, ..], _) = camera(&body, &q);
    assert!(
        forward[1].atan2(forward[0]) > 0.3,
        "the camera turned toward the gaze: {forward:?}"
    );
    let torso = solve
        .model()
        .axes()
        .iter()
        .position(|a| a.name == "torso_joint")
        .expect("torso");
    // The camera never asks the torso to turn. What is left is the neck's collision rows
    // against the arms, which may use any axis.
    assert!(
        (q[torso] - begun[torso]).abs() < 2.5e-3,
        "the torso turned {} rad for the camera",
        q[torso] - begun[torso]
    );
}

#[test]
fn a_hand_behind_an_arm_is_reported_with_the_link_hiding_it() {
    let mut ik = ghost();
    let body = ik.body().clone();
    let axes = body.model().axes().to_vec();
    let links = body.model().links().to_vec();
    let mut rng = Rng(0x5eed_cafe);
    let (mut hidden, mut tries) = (Vec::new(), 0);
    while tries < 60 {
        let q: Vec<f64> = axes
            .iter()
            .zip(ik.rest())
            .map(|(a, r)| match a.limits {
                Some([lo, hi]) if !a.name.starts_with("so101_") && a.name != "torso_joint" => {
                    lo + 0.05 + (hi - lo - 0.1) * rng.next()
                }
                _ => *r,
            })
            .collect();
        if ik.clearance(&q).0 < 0.02 {
            continue;
        }
        tries += 1;
        let targets: Vec<Option<Target>> = [0, 1]
            .map(|hand| {
                let p = tcp(&body, &q, hand);
                Some(Target::Pose {
                    translation_m: p.translation_m,
                    rotation_xyzw: p.rotation_xyzw,
                })
            })
            .into_iter()
            .chain([None])
            .collect();
        ik.reset();
        let step = ik
            .step(&q, &vec![true; q.len()], &targets, DT)
            .expect("step");
        hidden.extend(step.limbs[..2].iter().filter_map(|l| l.hidden_by));
        assert_eq!(step.limbs[2].hidden_by, None, "the camera is not a tool");
    }
    // Over 60 random arm postures with the neck at rest, some hand is behind an arm, and what
    // hides it never moves with the camera (the neck's base, fixed on the chest, may).
    assert!(!hidden.is_empty());
    let moving_with_camera = |name: &str| {
        (name.starts_with("so101_") && name != "so101_base_link") || name == "camera_link"
    };
    for link in hidden {
        assert!(!moving_with_camera(&links[link]), "{}", links[link]);
    }
}

#[test]
fn the_neck_holds_steady_while_both_hands_circle() {
    // Both hands trace slow circles (12 cm, 4 s a turn) for 10 s. The looking rows that
    // levelled the picture and stepped around the arms swung this neck through its parallel
    // pitch joints and pinned the camera tilt at its stop; pointing alone must not.
    let mut solve = teleop();
    let mut q = zero(&solve);
    let body = solve.ik().body().clone();
    let start = [tcp(&body, &q, 0), tcp(&body, &q, 1)];
    let neck: Vec<usize> = solve
        .model()
        .axes()
        .iter()
        .enumerate()
        .filter(|(_, a)| a.name.starts_with("so101_"))
        .map(|(i, _)| i)
        .collect();
    let tilt = solve
        .model()
        .axes()
        .iter()
        .position(|a| a.name == "so101_camera_tilt_joint")
        .expect("tilt");
    let [low, high] = solve.model().axes()[tilt].limits.expect("tilt range");
    let measured = vec![None; q.len()];
    let available = vec![true; q.len()];
    let (mut travel, mut pinned, mut ticks) = (0.0, 0, 0);
    solve.reset();
    for tick in 0..1000 {
        let phase = std::f64::consts::TAU * f64::from(tick) * DT / 4.0;
        let mut targets = Targets::default();
        for (hand, pose) in start.iter().enumerate() {
            let mut p = *pose;
            let side = if hand == 0 { 1.0 } else { -1.0 };
            p.translation_m[1] += 0.06 * side * (phase.cos() - 1.0);
            p.translation_m[2] += 0.06 * phase.sin();
            targets.tcp[hand] = Some(p);
        }
        let before = q.clone();
        let step = solve
            .step(&q, &measured, &available, &targets, DT)
            .expect("a step");
        q.copy_from_slice(&step.q);
        if tick >= 100 {
            travel += neck.iter().map(|&i| (q[i] - before[i]).abs()).sum::<f64>();
            pinned += usize::from(q[tilt] - low < 0.02 || high - q[tilt] < 0.02);
            ticks += 1;
        }
    }
    // Measured: 5.2 rad and never at a stop; with the levelling and clear-sight rows, 7.6 rad
    // and the tilt at its stop for 230 of the 900 ticks.
    assert_eq!(
        pinned, 0,
        "the camera tilt sat at its stop {pinned} of {ticks} ticks"
    );
    assert!(travel < 6.5, "the neck travelled {travel:.2} rad");
}

#[test]
fn the_first_look_down_at_the_hands_settles_without_bouncing() {
    // Engaging from the neck's rest, the camera swings well down onto both hands. Pointing
    // once asked for its whole error each tick and overshot the acceleration caps: the pitch
    // joints rang three times before settling (the replay films' first half second).
    let mut solve = teleop();
    let mut q = zero(&solve);
    let body = solve.ik().body().clone();
    let targets = Targets {
        tcp: [Some(tcp(&body, &q, 0)), Some(tcp(&body, &q, 1))],
        ..Targets::default()
    };
    let neck: Vec<usize> = solve
        .model()
        .axes()
        .iter()
        .enumerate()
        .filter(|(_, a)| a.name.starts_with("so101_"))
        .map(|(i, _)| i)
        .collect();
    let measured = vec![None; q.len()];
    let available = vec![true; q.len()];
    let mut last = vec![0.0; neck.len()];
    let mut reversals = vec![0; neck.len()];
    solve.reset();
    for _ in 0..200 {
        let before = q.clone();
        let step = solve
            .step(&q, &measured, &available, &targets, DT)
            .expect("a step");
        q.copy_from_slice(&step.q);
        for (slot, &axis) in neck.iter().enumerate() {
            let v = (q[axis] - before[axis]) / DT;
            if v.abs() > 0.05 {
                if last[slot] * v < 0.0 {
                    reversals[slot] += 1;
                }
                last[slot] = v;
            }
        }
    }
    let ([forward, ..], eye) = camera(&body, &q);
    let mid = [0, 1, 2].map(|k| {
        f64::midpoint(
            targets.tcp[0].expect("left").translation_m[k],
            targets.tcp[1].expect("right").translation_m[k],
        )
    });
    assert!(
        reversals.iter().all(|r| *r == 0),
        "neck joints turned back {reversals:?} times"
    );
    assert!(
        angle(forward, eye, mid) < 0.05,
        "the camera is {:.1} degrees off the hands",
        angle(forward, eye, mid).to_degrees()
    );
}
