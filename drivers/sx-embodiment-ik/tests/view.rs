//! The camera's view on the static upper body: it frames both hands, stays level, turns on
//! its own neck only, and steps around an arm standing between it and a hand.

#![allow(
    clippy::cast_precision_loss,
    clippy::float_cmp,
    clippy::needless_range_loop
)]

use std::collections::BTreeMap;

use sx_embodiment_ik::teleop::{HeadPolicy, Pose, TELEOP_SCHEMA, Targets, Teleop, TeleopConfig};
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

fn teleop(head: HeadPolicy) -> Teleop {
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
        head,
        held: Vec::new(),
        obstacles: Vec::new(),
    };
    Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).expect("solve")
}

/// Ghost caps (only reach, limits and collision matter), sight rows on or off.
fn ghost(sight: bool) -> Ik {
    let body = Body::static_upper_body();
    let mut caps = Caps::declared(&body, 0.25).expect("caps");
    for (speed, accel) in caps.speed.iter_mut().zip(caps.accel.iter_mut()) {
        *speed = 4.0;
        *accel = 400.0;
    }
    let settings = Settings {
        deadline: std::time::Duration::from_secs(1),
        sight_clearance_m: if sight { 0.01 } else { f64::NEG_INFINITY },
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
    let mut solve = teleop(HeadPolicy::Hands);
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
fn the_picture_stays_level_and_only_the_neck_turns_it() {
    let mut solve = teleop(HeadPolicy::Gaze);
    let mut q = zero(&solve);
    let body = solve.ik().body().clone();
    let begun = q.clone();
    // Both hands held where they are and a gaze well to the side, up and rolled: before, the
    // torso (it carries both arms) turned with the gaze and the picture rolled with the neck.
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
    let ([forward, across, _], _) = camera(&body, &q);
    assert!(
        across[2].abs() < 0.02,
        "the picture rolls {:.1} degrees",
        across[2].asin().to_degrees()
    );
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
    // The camera never asks the torso to turn (before: 9 mrad, and the picture rolled 12.6
    // degrees, on this very ask). What is left is the neck's collision rows against the arms,
    // which may use any axis.
    assert!(
        (q[torso] - begun[torso]).abs() < 2.5e-3,
        "the torso turned {} rad for the camera",
        q[torso] - begun[torso]
    );
}

/// The share of random arm postures, each settled with both hands held and the camera on
/// them, at which no part of the body stands between the camera and either hand.
fn both_hands_seen(sight: bool) -> (f64, usize) {
    let mut ik = ghost(sight);
    let body = ik.body().clone();
    let axes = body.model().axes().to_vec();
    let mut rng = Rng(0x5eed_cafe);
    let (mut seen, mut hidden_at_start, mut tries) = (0, 0, 0);
    while tries < 60 {
        let q0: Vec<f64> = axes
            .iter()
            .zip(ik.rest())
            .map(|(a, r)| match a.limits {
                Some([lo, hi]) if !a.name.starts_with("so101_") && a.name != "torso_joint" => {
                    lo + 0.05 + (hi - lo - 0.1) * rng.next()
                }
                _ => *r,
            })
            .collect();
        if ik.clearance(&q0).0 < 0.02 {
            continue;
        }
        tries += 1;
        let hands = [tcp(&body, &q0, 0), tcp(&body, &q0, 1)];
        let mid =
            [0, 1, 2].map(|k| f64::midpoint(hands[0].translation_m[k], hands[1].translation_m[k]));
        let targets: Vec<Option<Target>> = hands
            .iter()
            .map(|p| {
                Some(Target::Pose {
                    translation_m: p.translation_m,
                    rotation_xyzw: p.rotation_xyzw,
                })
            })
            .chain([Some(Target::LookAt { point_m: mid })])
            .collect();
        let available = vec![true; q0.len()];
        let mut q = q0.clone();
        ik.reset();
        let mut first = true;
        for _ in 0..300 {
            let step = ik.step(&q, &available, &targets, DT).expect("step");
            if first {
                hidden_at_start +=
                    usize::from(step.limbs[..2].iter().any(|l| l.hidden_by.is_some()));
                first = false;
            }
            q.copy_from_slice(&step.q);
        }
        let last = ik.step(&q, &available, &targets, DT).expect("step");
        if last.limbs[..2].iter().all(|l| l.hidden_by.is_none()) {
            seen += 1;
        }
    }
    (f64::from(seen) / f64::from(tries), hidden_at_start)
}

#[test]
fn the_camera_steps_around_an_arm_between_it_and_a_hand() {
    let (without, hidden) = both_hands_seen(false);
    let (with, _) = both_hands_seen(true);
    eprintln!(
        "both hands in clear sight: {:.0}% without sight rows, {:.0}% with ({hidden} of 60 hidden at the start)",
        100.0 * without,
        100.0 * with
    );
    assert!(hidden > 0, "the sample has occlusions to clear");
    assert!(with > without, "{with} vs {without}");
}
