//! Fast hand goals must slow before exhausting measured-feedback headroom, then follow
//! the latest goal as feedback catches up, without resetting the teleoperation solver.

#![allow(clippy::float_cmp)]

use std::collections::BTreeMap;
use sx_embodiment_ik::teleop::{
    LimbConfig, Pose, REFERENCE_GAP, TELEOP_SCHEMA, Targets, Teleop, TeleopConfig,
};

const DT: f64 = 0.01;
const ACCEL: f64 = 4.0;

fn follower() -> Teleop {
    let urdf = r#"<robot name="wrist"><link name="base"/><link name="tool"/>
    <joint name="wrist" type="revolute"><parent link="base"/><child link="tool"/>
    <axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1" effort="1"/>
    </joint></robot>"#;
    let config = TeleopConfig {
        schema: TELEOP_SCHEMA.into(),
        limbs: vec![LimbConfig {
            name: "left".into(),
            frame: "tool".into(),
            look_forward: None,
            approach: None,
        }],
        speed: BTreeMap::from([("wrist".into(), 1.0)]),
        accel: BTreeMap::from([("wrist".into(), ACCEL)]),
        ramp_s: 0.25,
        max_step_rad: 0.03,
        held: vec![],
        obstacles: vec![],
        posture: BTreeMap::new(),
    };
    let mut teleop = Teleop::new(urdf, r#"{"schema":"sx.collision-proxies/v4","embodiment_id":"wrist","urdf_sha256":"0","capsules":[],"hulls":[],"ignored_pairs":{"adjacent":[],"touching_at_zero":[]},"adjacent_joints":0,"generated_by":"test"}"#, &config).expect("wrist solver");
    teleop.set_deadline(std::time::Duration::from_secs(1));
    teleop
}

fn goal(angle: f64) -> Targets {
    let (z, w) = (angle / 2.0).sin_cos();
    Targets {
        tcp: [
            Some(Pose {
                translation_m: [0.0; 3],
                rotation_xyzw: [0.0, 0.0, z, w],
            }),
            None,
        ],
        ..Targets::default()
    }
}

#[test]
fn fast_hand_goals_brake_before_the_feedback_window_and_follow_the_latest_reversal() {
    for sign in [-1.0, 1.0] {
        let mut teleop = follower();
        let (mut reference, mut measured, mut velocity) = (0.0_f64, 0.0_f64, 0.0_f64);
        let mut limited = false;
        let mut reversed = false;
        for tick in 0..500 {
            if tick == 100 {
                // The station held across a short packet gap: restart planned velocity
                // from rest without forgetting the established reference or contacts.
                teleop.hold();
                velocity = 0.0;
            }
            // The hand outruns a healthy slow motor, then changes its mind. There is no
            // queued trajectory: a later goal must replace the earlier one immediately.
            let angle = if tick < 150 { sign * 0.8 } else { -sign * 0.3 };
            let step = teleop
                .step(&[reference], &[Some(measured)], &[true], &goal(angle), DT)
                .expect("continuous following");
            let next = step.q[0];
            let speed = (next - reference) / DT;
            assert!((next - measured).abs() <= REFERENCE_GAP + 1e-9);
            assert!(speed.abs() <= 1.0 + 1e-9);
            assert!(
                (speed - velocity).abs() <= ACCEL * DT + 1e-6,
                "tick {tick}: velocity jumped from {velocity} to {speed} at lead {}",
                next - measured
            );
            assert!(
                !step.stalled[0],
                "progressing feedback must not latch a contact"
            );
            if tick < 150 && (next - measured).abs() > REFERENCE_GAP * 0.8 {
                limited = true;
            }
            if tick > 150 && speed * sign < -0.01 {
                reversed = true;
            }
            velocity = speed;
            reference = next;
            // A slow but progressing motor; later it catches up completely.
            let travel = if tick < 250 { 0.001 } else { 0.01 };
            measured += (reference - measured).clamp(-travel, travel);
        }
        assert!(limited, "the fast target must exercise the feedback window");
        assert!(
            reversed,
            "the new target must replace the previous direction"
        );
        assert!(
            (measured + sign * 0.3).abs() < 0.01,
            "must recover without reset: {measured}"
        );
    }
}
