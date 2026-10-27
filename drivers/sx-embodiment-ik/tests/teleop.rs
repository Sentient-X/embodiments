//! The shared teleoperation solve on the static upper body: where the camera looks, the holds,
//! the grippers, the reference rule, and one config both sides build from.

#![allow(clippy::float_cmp)]

use std::collections::BTreeMap;

use sx_embodiment_ik::teleop::{
    HANDS, Pose, REFERENCE_GAP, TELEOP_SCHEMA, Targets, Teleop, TeleopConfig,
};
use sx_embodiment_ik::{Body, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};

const DT: f64 = 0.01;

/// Every joint at 1 rad/s, reached in a quarter second.
fn config() -> TeleopConfig {
    let body = Body::static_upper_body();
    let speed: BTreeMap<String, f64> = body
        .model()
        .axes()
        .iter()
        .map(|axis| (axis.name.clone(), 1.0))
        .collect();
    TeleopConfig {
        schema: TELEOP_SCHEMA.into(),
        limbs: TeleopConfig::static_upper_body_limbs(),
        speed,
        accel: BTreeMap::new(),
        ramp_s: 0.25,
        max_step_rad: 0.03,
        held: Vec::new(),
        obstacles: Vec::new(),
    }
}

fn teleop() -> Teleop {
    Teleop::new(
        STATIC_UPPER_BODY_URDF,
        STATIC_UPPER_BODY_COLLISION,
        &config(),
    )
    .expect("solve")
}

/// Every axis inside its range, the mid of zero and its range's middle.
fn start(teleop: &Teleop) -> Vec<f64> {
    teleop
        .model()
        .axes()
        .iter()
        .map(|a| {
            a.limits.map_or(0.0, |[l, u]| {
                0.0_f64.clamp(l, u).midpoint(f64::midpoint(l, u))
            })
        })
        .collect()
}

fn index(teleop: &Teleop, name: &str) -> usize {
    teleop
        .model()
        .axes()
        .iter()
        .position(|a| a.name == name)
        .expect(name)
}

/// Run `ticks` steps as a ghost would (its own reference, nothing measured).
fn run(teleop: &mut Teleop, q: &mut [f64], targets: &Targets, ticks: usize) {
    let width = q.len();
    let measured = vec![None; width];
    let available = vec![true; width];
    for _ in 0..ticks {
        let step = teleop
            .step(q, &measured, &available, targets, DT)
            .expect("a step");
        q.copy_from_slice(&step.q);
    }
}

fn camera_forward(teleop: &Teleop, q: &[f64]) -> ([f64; 3], [f64; 3]) {
    let body = teleop.ik().body();
    let head = body
        .limbs()
        .iter()
        .position(|l| *l == "head")
        .expect("a head");
    let frame = body.frame(&body.model().poses(q), head);
    ([frame.r[0][0], frame.r[1][0], frame.r[2][0]], frame.t)
}

fn tcp(teleop: &Teleop, q: &[f64], hand: usize) -> Pose {
    let body = teleop.ik().body();
    let frame = body.frame(&body.model().poses(q), hand).to_transform();
    let p = sx_embodiment_drivers::kinematics::pose_of(&frame);
    Pose {
        translation_m: p.translation_m,
        rotation_xyzw: p.rotation_xyzw,
    }
}

#[test]
fn the_config_round_trips_and_a_foreign_schema_is_refused() {
    let config = config();
    let json = config.to_json();
    assert_eq!(TeleopConfig::parse(&json).expect("parsed"), config);
    assert!(
        !json.contains("\"head\":"),
        "no head choice in the config: who looks decides"
    );
    let foreign = json.replace(TELEOP_SCHEMA, "sx.teleop-solve/v0");
    assert!(TeleopConfig::parse(&foreign).is_err());
}

#[test]
fn gaze_turns_the_camera_to_the_head_and_holds_the_rest() {
    let mut solve = teleop();
    let mut q = start(&solve);
    let begun = q.clone();
    let (f0, _) = camera_forward(&solve, &q);
    let yaw = f0[1].atan2(f0[0]) + 0.5;
    let gaze = Pose {
        translation_m: [0.0, 0.0, 1.0],
        rotation_xyzw: [0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()],
    };
    let targets = Targets {
        head: Some(gaze),
        ..Targets::default()
    };
    solve.reset();
    run(&mut solve, &mut q, &targets, 400);
    let (f, _) = camera_forward(&solve, &q);
    let dot = f[0] * yaw.cos() + f[1] * yaw.sin();
    assert!(dot > 0.99, "camera forward {f:?}");
    // The arms and the torso carry undriven limbs: held.
    for (axis, (now, was)) in solve.model().axes().iter().zip(q.iter().zip(&begun)) {
        if !axis.name.starts_with("so101_") {
            assert_eq!(now, was, "{} moved", axis.name);
        }
    }
    // No head pose: the neck holds too.
    let turned = q.clone();
    run(&mut solve, &mut q, &Targets::default(), 50);
    assert_eq!(q, turned);
}

#[test]
fn hands_looks_at_the_commanded_hands_even_from_facing_away() {
    let mut solve = teleop();
    let mut q: Vec<f64> = solve
        .model()
        .axes()
        .iter()
        .map(|a| a.limits.map_or(0.0, |[l, u]| 0.0_f64.clamp(l, u)))
        .collect();
    let (left, right) = (tcp(&solve, &q, 0), tcp(&solve, &q, 1));
    let middle: Vec<f64> = (0..3)
        .map(|k| f64::midpoint(left.translation_m[k], right.translation_m[k]))
        .collect();
    let targets = Targets {
        tcp: [Some(left), Some(right)],
        ..Targets::default()
    };
    solve.reset();
    run(&mut solve, &mut q, &targets, 400);
    let (forward, at) = camera_forward(&solve, &q);
    let to: Vec<f64> = (0..3).map(|k| middle[k] - at[k]).collect();
    let distance = to.iter().map(|v| v * v).sum::<f64>().sqrt();
    let dot: f64 = (0..3).map(|k| forward[k] * to[k] / distance).sum();
    assert!(dot > 0.99, "the camera looks {dot} toward the hands");
}

#[test]
fn an_idle_hand_and_the_torso_hold_while_one_hand_drives() {
    let mut solve = teleop();
    let mut q = start(&solve);
    let begun = q.clone();
    let mut target = tcp(&solve, &q, 0);
    target.translation_m[2] += 0.05;
    let targets = Targets {
        tcp: [Some(target), None],
        ..Targets::default()
    };
    solve.reset();
    run(&mut solve, &mut q, &targets, 200);
    let torso = index(&solve, "torso_joint");
    assert_eq!(q[torso], begun[torso], "the torso holds with one hand idle");
    for (axis, (now, was)) in solve.model().axes().iter().zip(q.iter().zip(&begun)) {
        if axis.name.starts_with("right_") {
            assert_eq!(now, was, "{} moved", axis.name);
        }
    }
    assert!(
        q.iter().zip(&begun).any(|(a, b)| a != b),
        "the left arm moved"
    );
}

#[test]
fn each_gripper_maps_the_fingertip_distance_onto_its_fingers() {
    let mut solve = teleop();
    for (hand, gripper) in HANDS.iter().zip(solve.grippers()) {
        let gripper = gripper.as_ref().expect(hand);
        assert_eq!(gripper.fingers.len(), 2);
        assert!(
            gripper.open_width_m > gripper.closed_width_m + 0.01,
            "{gripper:?}"
        );
    }
    let open = solve.grippers()[0].clone().expect("left").open_width_m;
    let mut q = start(&solve);
    let targets = Targets {
        gripper_width_m: [Some(open), None],
        ..Targets::default()
    };
    run(&mut solve, &mut q, &targets, 1);
    for &(axis, _, far) in &solve.grippers()[0].clone().expect("left").fingers {
        assert_eq!(q[axis], far);
        assert!(far.abs() > 0.5);
    }
}

#[test]
fn the_reference_seeds_from_the_measurement_at_engage_and_where_it_strays() {
    let mut solve = teleop();
    let q = start(&solve);
    let width = q.len();
    let elbow = index(&solve, "left_elbow_pitch_joint");
    let available = vec![true; width];
    let mut measured: Vec<Option<f64>> = q.iter().map(|v| Some(*v)).collect();
    let reference = vec![0.0; width];
    // Engage: every answering axis starts where it is measured, whatever the reference.
    solve.reset();
    let step = solve
        .step(&reference, &measured, &available, &Targets::default(), DT)
        .expect("a step");
    assert_eq!(step.q[elbow], q[elbow]);
    // Within the gap the reference wins; past it, the measurement.
    let mut reference = step.q.clone();
    measured[elbow] = Some(q[elbow] + REFERENCE_GAP / 2.0);
    let step = solve
        .step(&reference, &measured, &available, &Targets::default(), DT)
        .expect("a step");
    assert_eq!(step.q[elbow], q[elbow]);
    reference.clone_from(&step.q);
    measured[elbow] = Some(q[elbow] + REFERENCE_GAP * 2.0);
    let step = solve
        .step(&reference, &measured, &available, &Targets::default(), DT)
        .expect("a step");
    assert_eq!(step.q[elbow], q[elbow] + REFERENCE_GAP * 2.0);
}

#[test]
fn following_starts_only_with_every_driven_hand_at_its_gripper() {
    let mut solve = teleop();
    let q = start(&solve);
    let (left, right) = (tcp(&solve, &q, 0), tcp(&solve, &q, 1));
    let at = |left: Option<Pose>, right: Option<Pose>| Targets {
        tcp: [left, right],
        ..Targets::default()
    };
    // No hand driven: nothing to start from.
    assert!(!solve.may_engage(&q, &at(None, None)));
    assert!(solve.may_engage(&q, &at(Some(left), Some(right))));
    assert!(solve.may_engage(&q, &at(Some(left), None)));
    let mut near = left;
    near.translation_m[0] += 0.05;
    assert!(solve.may_engage(&q, &at(Some(near), Some(right))));
    let mut far = left;
    far.translation_m[2] += 0.07;
    assert!(!solve.may_engage(&q, &at(Some(far), Some(right))));
    let gaps = solve.hand_gaps(&q, &at(Some(far), None));
    assert!((gaps[0].expect("left") - 0.07).abs() < 1e-9 && gaps[1].is_none());
}
