//! The arms' redundancy resolves toward a preferred posture (a robot's commissioned Ready pose,
//! elbows down): a hand moved along a line keeps a natural arm instead of reconfiguring.

// Step counts of a few hundred, as f64 and back.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)]

use std::collections::BTreeMap;

use sx_embodiment_ik::Body;
use sx_embodiment_ik::teleop::{Pose, TELEOP_SCHEMA, Targets, Teleop, TeleopConfig};

const DT: f64 = 0.02;
const TOOL_SPEED: f64 = 0.15;

/// The static upper body's declared work-ready configuration (`sx_embodiments`, static upper
/// body `ready`): each arm off its elbow stop, the camera tilted to the cell.
fn ready() -> BTreeMap<String, f64> {
    let left = [-0.1, 0.15, 0.0, -0.3, 0.0, 0.2, 0.0];
    let right = [0.1, -0.15, 0.0, 0.3, 0.0, -0.2, 0.0];
    let arm = [
        "shoulder_pitch_joint",
        "shoulder_roll_joint",
        "upper_arm_yaw_joint",
        "elbow_pitch_joint",
        "forearm_yaw_joint",
        "wrist_pitch_joint",
        "wrist_roll_joint",
    ];
    let mut pose = BTreeMap::new();
    for (joint, (l, r)) in arm.iter().zip(left.iter().zip(&right)) {
        pose.insert(format!("left_{joint}"), *l);
        pose.insert(format!("right_{joint}"), *r);
    }
    pose.insert("so101_camera_tilt_joint".into(), -0.8865);
    pose
}

fn teleop(posture: BTreeMap<String, f64>) -> Teleop {
    let body = Body::static_upper_body();
    let speed = body
        .model()
        .axes()
        .iter()
        .map(|a| (a.name.clone(), 3.77))
        .collect();
    let config = TeleopConfig {
        schema: TELEOP_SCHEMA.into(),
        limbs: TeleopConfig::static_upper_body_limbs(),
        speed,
        accel: BTreeMap::new(),
        ramp_s: 0.1,
        max_step_rad: 0.08,
        held: vec!["torso_joint".into()],
        obstacles: Vec::new(),
        posture,
    };
    Teleop::with_body(Body::static_upper_body(), &config).expect("solve")
}

struct Run {
    /// The largest joint speed over the move per unit tool speed, rad per metre.
    peak_rad_per_m: f64,
    /// The arm's whole joint-space travel per metre of tool travel, rad per metre.
    travel_rad_per_m: f64,
    /// How far the elbow rose above the shoulder at worst, metres (negative: always below).
    elbow_over_shoulder_m: f64,
    /// The tool's residual at the end, metres and radians.
    residual_m: f64,
    residual_rad: f64,
}

/// The left hand from Ready along a straight line to the cup's grasp pose, the target moving at
/// `TOOL_SPEED` and tracked perfectly.
fn run(posture: BTreeMap<String, f64>) -> Run {
    let mut teleop = teleop(posture);
    let names: Vec<String> = teleop
        .model()
        .axes()
        .iter()
        .map(|a| a.name.clone())
        .collect();
    let pose = ready();
    let mut q: Vec<f64> = names
        .iter()
        .zip(teleop.ik().rest())
        .map(|(name, rest)| pose.get(name).copied().unwrap_or(*rest))
        .collect();
    let body = teleop.ik().body();
    let left = body
        .limbs()
        .iter()
        .position(|l| *l == "left")
        .expect("left");
    let frames = body.model().poses(&q);
    let start = body.frame(&frames, left);
    let model = body.model();
    let elbow = model.link("left_elbow_pitch_link").expect("elbow link");
    let shoulder = model
        .link("left_shoulder_pitch_link")
        .expect("shoulder link");
    let rotation = start.r;
    let goal = [0.41, 0.12, 0.155];
    let length = (0..3)
        .map(|k| (goal[k] - start.t[k]).powi(2))
        .sum::<f64>()
        .sqrt();
    let steps = (length / TOOL_SPEED / DT).ceil() as usize + 50;
    let available = vec![true; q.len()];
    let measured: Vec<Option<f64>> = q.iter().copied().map(Some).collect();
    let free = vec![None; q.len()];
    let quaternion = to_quaternion(&rotation);
    let (mut peak, mut over, mut travel) = (0.0_f64, f64::NEG_INFINITY, 0.0_f64);
    let (mut residual_m, mut residual_rad) = (0.0, 0.0);
    teleop.reset();
    for step in 0..steps {
        let fraction = ((step as f64) * DT * TOOL_SPEED / length).min(1.0);
        let mut targets = Targets::default();
        targets.tcp[0] = Some(Pose {
            translation_m: [0, 1, 2].map(|k| start.t[k] + (goal[k] - start.t[k]) * fraction),
            rotation_xyzw: quaternion,
        });
        let given = if step == 0 { &measured } else { &free };
        let answer = teleop
            .step(&q, given, &available, &targets, DT)
            .expect("step");
        for (axis, name) in names.iter().enumerate() {
            if name.starts_with("left_") && !name.contains("gripper") {
                peak = peak.max((answer.q[axis] - q[axis]).abs() / DT);
                travel += (answer.q[axis] - q[axis]).abs();
            }
        }
        residual_m = answer.limbs[left].residual_m;
        residual_rad = answer.limbs[left].residual_rad;
        q.copy_from_slice(&answer.q);
        let frames = teleop.model().poses(&q);
        over = over.max(frames.links[elbow].t[2] - frames.links[shoulder].t[2]);
    }
    Run {
        peak_rad_per_m: peak / TOOL_SPEED,
        travel_rad_per_m: travel / length,
        elbow_over_shoulder_m: over,
        residual_m,
        residual_rad,
    }
}

fn to_quaternion(r: &[[f64; 3]; 3]) -> [f64; 4] {
    let trace = r[0][0] + r[1][1] + r[2][2];
    let s = (trace + 1.0).max(1e-12).sqrt() * 2.0;
    let q = [
        (r[2][1] - r[1][2]) / s,
        (r[0][2] - r[2][0]) / s,
        (r[1][0] - r[0][1]) / s,
        0.25 * s,
    ];
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    q.map(|v| v / n)
}

#[test]
fn the_cup_grasp_keeps_the_elbow_down_and_its_joints_calm_with_the_ready_posture() {
    let anchored_at_engage = run(BTreeMap::new());
    let preferred = run(ready());
    for (name, run) in [
        ("anchored at engage", &anchored_at_engage),
        ("ready posture", &preferred),
    ] {
        eprintln!(
            "{name}: {:.2} rad/m peak, {:.2} rad of joint travel per m, elbow over shoulder {:.3} m, residual {:.4} m {:.4} rad",
            run.peak_rad_per_m,
            run.travel_rad_per_m,
            run.elbow_over_shoulder_m,
            run.residual_m,
            run.residual_rad
        );
    }
    assert!(
        preferred.elbow_over_shoulder_m < 0.0,
        "the elbow stays below the shoulder"
    );
    // The arm travels less in joint space for the same tool travel, and its forearm and wrist
    // counter-spin less: a lower peak joint speed per metre of tool travel (measured where the
    // solve meets its 4 ms deadline every tick: an unoptimised build misses it, holding steps).
    assert!(preferred.travel_rad_per_m < anchored_at_engage.travel_rad_per_m);
    if cfg!(not(debug_assertions)) {
        assert!(preferred.peak_rad_per_m < 0.9 * anchored_at_engage.peak_rad_per_m);
    }
    // ...and the tool still arrives, well inside a goal's 5 mm and 3 degrees.
    assert!(preferred.residual_m < 0.002 && preferred.residual_rad < 0.02);
}
