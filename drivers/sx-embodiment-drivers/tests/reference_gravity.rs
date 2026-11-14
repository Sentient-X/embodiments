//! The left arm's reference model (the November 14 handoff, accepted on the robot) as an
//! identified-dynamics calibration: `fixtures/reference_gravity/left-arm.dynamics.json`,
//! written by `tools/import_reference_dynamics.py` from the fixture bundle and the nominal
//! drive. Ported from the compensation trial's tests (embodiments #279, Kripansh Sharma) onto
//! the one dynamics model every body uses.

use serde_json::Value;
use sx_embodiment_drivers::{Dynamics, DynamicsCalibration, UPRIGHT};

const URDF: &str = include_str!("../../../render_packs/sentient-static-upper-body/robot.urdf");
const CALIBRATION: &str = include_str!("fixtures/reference_gravity/left-arm.dynamics.json");
const ORACLE: &str = include_str!("fixtures/reference_gravity/gravity.json");
const ARM: [&str; 7] = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_upper_arm_yaw_joint",
    "left_elbow_pitch_joint",
    "left_forearm_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
];
/// The share of gravity the accepted run fed forward; the oracle carries it, the calibration
/// (the arm's physics) does not.
const ACCEPTED_GRAVITY_RATIO: f64 = 0.95;

fn calibrated() -> Dynamics {
    let calibration = DynamicsCalibration::parse(CALIBRATION).expect("a dynamics calibration");
    Dynamics::of(URDF, UPRIGHT)
        .expect("description")
        .calibrated(&calibration)
        .expect("inside the plausibility bounds, for this description")
}

/// Where each arm joint sits among the description's independent joints.
fn arm(dynamics: &Dynamics) -> Vec<usize> {
    ARM.iter()
        .map(|name| {
            dynamics
                .tree()
                .joints()
                .iter()
                .position(|axis| axis.name == *name)
                .expect("an arm joint")
        })
        .collect()
}

fn full(dynamics: &Dynamics, rows: &[usize], arm_q: &[f64]) -> Vec<f64> {
    let mut q = vec![0.0; dynamics.tree().joints().len()];
    for (row, value) in rows.iter().zip(arm_q) {
        q[*row] = *value;
    }
    q
}

/// The reference's own gravity oracle, 24 poses: the description with the fitted links
/// reproduces it to 0.041 N·m under the nominal drive. The reference's encoder zeros differ
/// from that drive's (elbow 5.8°, wrist pitch 8.1°, wrist roll 104°), which turns its distal
/// links against the description's; with a drive that agrees, the importer's fit is exact to
/// 2e-5 N·m (`tests/test_import_reference_dynamics.py`).
#[test]
fn the_calibrated_description_reproduces_the_reference_gravity_oracle() {
    let dynamics = calibrated();
    let rows = arm(&dynamics);
    let oracle: Value = serde_json::from_str(ORACLE).expect("oracle");
    let mut scratch = dynamics.scratch();
    let mut torque = vec![0.0; dynamics.tree().joints().len()];
    let mut worst = 0.0_f64;
    let cases = oracle["cases"].as_array().expect("cases");
    for case in cases {
        let q: Vec<f64> = case["q"]
            .as_array()
            .expect("q")
            .iter()
            .map(|v| v.as_f64().expect("number"))
            .collect();
        dynamics
            .gravity_torque_into(&full(&dynamics, &rows, &q), &mut scratch, &mut torque)
            .expect("gravity");
        for (index, want) in case["gravity"].as_array().expect("gravity").iter().enumerate() {
            let got = ACCEPTED_GRAVITY_RATIO * torque[rows[index]];
            worst = worst.max((got - want.as_f64().expect("number")).abs());
        }
    }
    println!("{} reference gravity cases: worst {worst:.6} N·m", cases.len());
    assert!(worst < 0.045, "worst gravity mismatch {worst} N·m");
}

/// A reference velocity jittering about zero inside the elbow's deadband (an operator's hand
/// at rest, an encoder quantum) feeds no friction forward; past it, friction follows the
/// motion's sign, rising from zero.
#[test]
fn friction_inside_the_deadband_is_none_and_past_it_follows_the_motion() {
    let dynamics = calibrated();
    let rows = arm(&dynamics);
    let elbow = rows[3];
    let q = full(&dynamics, &rows, &[0.2, 0.3, 0.0, -0.8, 0.0, 0.2, 0.0]);
    let width = q.len();
    let rest = vec![0.0; width];
    let held = dynamics.inverse(&q, &rest, &rest).expect("held");
    let friction = dynamics.friction()[elbow];
    assert!((friction.velocity_deadband - 0.02).abs() < 1e-12);
    assert!((friction.coulomb - 0.22).abs() < 1e-12);
    for speed in [-0.014_652, -0.004_884, 0.0, 0.004_884, 0.014_652] {
        let mut qd = rest.clone();
        qd[elbow] = speed;
        let moving = dynamics.inverse(&q, &qd, &rest).expect("moving");
        assert!((moving[elbow] - held[elbow]).abs() < 1e-12, "{speed}: {moving:?}");
    }
    for speed in [-0.2_f64, 0.2] {
        let mut qd = rest.clone();
        qd[elbow] = speed;
        let moving = dynamics.inverse(&q, &qd, &rest).expect("moving");
        // Velocity enters the rigid terms too (Coriolis, here negligible at 0.2 rad/s).
        let expected = 0.22 * ((speed.abs() - 0.02) / 0.01).tanh() * speed.signum();
        assert!((moving[elbow] - held[elbow] - expected).abs() < 1e-3);
        assert!((moving[elbow] - held[elbow]) * speed > 0.0);
    }
    // The wrist pitch feeds no friction forward at any speed (the reference's J6).
    let pitch = rows[5];
    let mut qd = rest.clone();
    qd[pitch] = 2.0;
    let moving = dynamics.inverse(&q, &qd, &rest).expect("moving");
    let mut gravity = vec![0.0; width];
    dynamics
        .gravity_torque_into(&q, &mut dynamics.scratch(), &mut gravity)
        .expect("gravity");
    assert!(dynamics.friction()[pitch].coulomb.abs() < 1e-12);
    assert!((moving[pitch] - held[pitch]).abs() < 1e-2);
}

/// Ready, Park, an engagement and a hold all ask for zero velocity: what is fed forward is
/// gravity alone, whatever moved before. The model keeps no history, so no stale friction or
/// slew can follow a motion into a hold.
#[test]
fn at_rest_the_feed_forward_is_gravity_alone_whatever_came_before() {
    let dynamics = calibrated();
    let rows = arm(&dynamics);
    let q = full(&dynamics, &rows, &[0.3, 0.2, 0.1, -1.0, 0.2, -0.3, 0.1]);
    let width = q.len();
    let rest = vec![0.0; width];
    let mut scratch = dynamics.scratch();
    let mut torque = vec![0.0; width];
    let mut fast = rest.clone();
    for row in &rows {
        fast[*row] = 1.5;
    }
    dynamics
        .inverse_into(&q, &fast, &fast, &mut scratch, &mut torque)
        .expect("a fast motion");
    dynamics
        .inverse_into(&q, &rest, &rest, &mut scratch, &mut torque)
        .expect("at rest");
    let mut gravity = vec![0.0; width];
    dynamics
        .gravity_torque_into(&q, &mut scratch, &mut gravity)
        .expect("gravity");
    for row in &rows {
        assert!((torque[*row] - gravity[*row]).abs() < 1e-12);
    }
}

/// A non-finite position or speed is refused, and refuses nothing after it: the next valid
/// state gives its own torque.
#[test]
fn an_invalid_state_is_refused_and_leaves_nothing_behind() {
    let dynamics = calibrated();
    let rows = arm(&dynamics);
    let q = full(&dynamics, &rows, &[0.0, 0.2, 0.0, -0.5, 0.0, 0.0, 0.0]);
    let width = q.len();
    let rest = vec![0.0; width];
    let mut scratch = dynamics.scratch();
    let mut torque = vec![7.0; width];
    let mut bad = q.clone();
    bad[rows[3]] = f64::NAN;
    assert!(
        dynamics
            .inverse_into(&bad, &rest, &rest, &mut scratch, &mut torque)
            .is_err()
    );
    let mut fast = rest.clone();
    fast[rows[3]] = f64::INFINITY;
    assert!(
        dynamics
            .inverse_into(&q, &fast, &rest, &mut scratch, &mut torque)
            .is_err()
    );
    dynamics
        .inverse_into(&q, &rest, &rest, &mut scratch, &mut torque)
        .expect("valid");
    let mut gravity = vec![0.0; width];
    dynamics
        .gravity_torque_into(&q, &mut dynamics.scratch(), &mut gravity)
        .expect("gravity");
    assert_eq!(torque, gravity);
}

/// The Spark carries a whole-body fit (stiction velocity 0.02 rad/s, the right elbow's friction
/// identified) and takes the left arm from the reference (0.01 rad/s). Each joint's friction
/// turns on over the stiction velocity of the calibration that supplied it, so both apply.
#[test]
fn two_calibrations_keep_their_own_stiction_velocity() {
    use sx_embodiment_drivers::JointFriction;
    let digest = serde_json::from_str::<Value>(CALIBRATION).expect("json")["description_sha256"]
        .as_str()
        .expect("digest")
        .to_owned();
    let right = DynamicsCalibration {
        description_sha256: digest,
        stiction_velocity: 0.02,
        links: Vec::new(),
        joints: vec![(
            "right_elbow_pitch_joint".to_owned(),
            JointFriction {
                coulomb: 0.77,
                ..JointFriction::default()
            },
        )],
    };
    let dynamics = calibrated().calibrated(&right).expect("both apply");
    let rows = arm(&dynamics);
    let left = rows[3];
    let right = dynamics
        .tree()
        .joints()
        .iter()
        .position(|axis| axis.name == "right_elbow_pitch_joint")
        .expect("right elbow");
    let q = vec![0.0; dynamics.tree().joints().len()];
    let rest = vec![0.0; q.len()];
    let held = dynamics.inverse(&q, &rest, &rest).expect("held");
    let mut qd = rest.clone();
    qd[left] = 0.03;
    qd[right] = 0.01;
    let moving = dynamics.inverse(&q, &qd, &rest).expect("moving");
    assert!((moving[left] - held[left] - 0.22 * (0.01_f64 / 0.01).tanh()).abs() < 1e-3);
    assert!((moving[right] - held[right] - 0.77 * (0.01_f64 / 0.02).tanh()).abs() < 1e-3);
}
