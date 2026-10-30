//! Planned velocity and feed-forward torque in the static upper body's MIT frames, on the
//! simulated bus: off by default, mapped through each axis' sign and reduction, bounded per
//! axis, clamped to the motor's `VMAX` and `TMAX`, refused when not finite.
//!
//! The MIT encoder itself is held byte for byte to Damiao's `DM_CAN.py`, nonzero velocity and
//! torque included, by `transcripts.rs` (`control_mit_frames_are_the_vendors`); here each frame
//! is packed with that same encoder from what the chain should have sent.

use serde_json::Value;
use sx_damiao_can::chain::{DamiaoChain, DamiaoChainError, MAX_COAST};
use sx_damiao_can::dm_can::float_to_uint;
use sx_damiao_can::sim::Sim;
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_drivers::can::CanFrame;
use sx_embodiment_drivers::{ActuatorChain, InstanceDrive};
use sx_embodiment_drivers::{Feedforward, FeedforwardLimit};

fn drive() -> InstanceDrive {
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
    let nominal = &drives["sentient-static-upper-body"]["nominal"];
    InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("an instance"),
    )
    .expect("a drive")
}

fn index_of(joint: &str) -> usize {
    STATIC_UPPER_BODY
        .joints
        .iter()
        .position(|gains| gains.joint == joint)
        .expect("a joint of the body")
}

fn opened() -> (Sim, DamiaoChain<Sim>, Vec<f64>) {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let bus = Sim::for_axes(&axes);
    let chain = DamiaoChain::open(bus.clone(), axes).expect("opens");
    let rest = chain.last_observed().expect("rest").to_vec();
    (bus, chain, rest)
}

/// The last MIT frame each axis was sent (an MIT frame is any frame to the motor's id that is
/// not one of the `FF..` mode frames).
fn last_mit(bus: &Sim, id: u16) -> [u8; 8] {
    bus.state()
        .frames
        .iter()
        .rev()
        .find(|frame: &&CanFrame| frame.id == u32::from(id) && frame.data[..7] != [0xFF; 7])
        .expect("an MIT frame")
        .data
}

fn mit_bytes(kp: f64, kd: f64, q: f64, dq: f64, tau: f64, limits: [f64; 3]) -> [u8; 8] {
    let [q_max, dq_max, tau_max] = limits;
    let kp = float_to_uint(kp, 0.0, 500.0, 12).expect("kp");
    let kd = float_to_uint(kd, 0.0, 5.0, 12).expect("kd");
    let q = float_to_uint(q, -q_max, q_max, 16).expect("q");
    let dq = float_to_uint(dq, -dq_max, dq_max, 12).expect("dq");
    let tau = float_to_uint(tau, -tau_max, tau_max, 12).expect("tau");
    let low = |value: u16| value.to_le_bytes()[0];
    [
        low(q >> 8),
        low(q),
        low(dq >> 4),
        low(((dq & 0xf) << 4) | ((kp >> 8) & 0xf)),
        low(kp),
        low(kd >> 4),
        low(((kd & 0xf) << 4) | ((tau >> 8) & 0xf)),
        low(tau),
    ]
}

#[test]
fn feedforward_is_off_until_bounded() {
    let (bus, mut chain, rest) = opened();
    chain.command(&rest).expect("plain");
    let ids: Vec<u16> = chain.axes().iter().map(|a| a.binding.bus_id).collect();
    let plain: Vec<[u8; 8]> = ids.iter().map(|id| last_mit(&bus, *id)).collect();
    let wanted = vec![
        Feedforward {
            velocity: 0.3,
            torque: 4.0
        };
        rest.len()
    ];
    chain.command_with(&rest, &wanted).expect("with");
    let with: Vec<[u8; 8]> = ids.iter().map(|id| last_mit(&bus, *id)).collect();
    assert_eq!(
        plain, with,
        "an axis with no bound sends zero velocity and torque"
    );
    assert!(
        chain
            .applied_feedforward()
            .iter()
            .all(|ff| *ff == Feedforward::default())
    );
}

#[test]
fn feedforward_maps_through_sign_and_clamps_to_the_bound_and_tmax() {
    let (bus, mut chain, rest) = opened();
    let wrist = index_of("left_wrist_roll_joint"); // sign -1, DM4310 (TMAX 10)
    let elbow = index_of("left_elbow_pitch_joint"); // sign +1, DM4340
    let mut limits = vec![FeedforwardLimit::default(); rest.len()];
    limits[wrist] = FeedforwardLimit {
        velocity: Some(0.5),
        torque: Some(100.0),
    };
    limits[elbow] = FeedforwardLimit {
        velocity: Some(0.5),
        torque: Some(3.0),
    };
    chain.set_feedforward_limits(&limits).expect("bounds");
    let mut wanted = vec![Feedforward::default(); rest.len()];
    wanted[wrist] = Feedforward {
        velocity: 0.3,
        torque: 40.0,
    };
    wanted[elbow] = Feedforward {
        velocity: -0.9,
        torque: 2.5,
    };
    // Mid-range targets, so the excursion bound leaves the feed-forward whole.
    let mut targets = rest.clone();
    for index in [wrist, elbow] {
        let range = chain.commandable()[index];
        targets[index] = f64::midpoint(range.lower, range.upper);
    }
    chain.command_with(&targets, &wanted).expect("command");
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    for (index, (dq, tau)) in [(wrist, (-0.3, -10.0)), (elbow, (-0.5, 2.5))] {
        let motor = &axes[index];
        let reported = chain.limits()[index];
        let limit = [
            f64::from(reported.p_max),
            f64::from(reported.v_max),
            f64::from(reported.t_max),
        ];
        let target = motor.axis.actuator_from_joint(targets[index]);
        assert_eq!(
            last_mit(&bus, motor.axis.binding.bus_id),
            mit_bytes(motor.kp, motor.kd, target, dq, tau, limit),
            "{}",
            motor.axis.joint
        );
    }
    let applied = chain.applied_feedforward();
    // The wrist turns against its motor: +0.3 rad/s at the joint is -0.3 at the motor, and
    // 40 N·m is cut to the DM4310's 10 N·m.
    assert_eq!(
        applied[wrist],
        Feedforward {
            velocity: 0.3,
            torque: 10.0
        }
    );
    assert_eq!(
        applied[elbow],
        Feedforward {
            velocity: -0.5,
            torque: 2.5
        }
    );
}

#[test]
fn a_non_finite_feedforward_or_bound_is_refused_before_any_frame() {
    let (bus, mut chain, rest) = opened();
    let sent = bus.sent();
    let mut wanted = vec![Feedforward::default(); rest.len()];
    wanted[3].torque = f64::NAN;
    assert!(matches!(
        chain.command_with(&rest, &wanted),
        Err(DamiaoChainError::Feedforward { .. })
    ));
    assert!(matches!(
        chain.command_with(&rest, &wanted[1..]),
        Err(DamiaoChainError::FeedforwardLength { .. })
    ));
    assert_eq!(bus.sent(), sent);
    let mut limits = vec![FeedforwardLimit::default(); rest.len()];
    limits[0].torque = Some(-1.0);
    assert!(matches!(
        chain.set_feedforward_limits(&limits),
        Err(DamiaoChainError::FeedforwardLimit { .. })
    ));
}

#[test]
fn feedforward_never_pushes_a_joint_past_its_range() {
    let (bus, mut chain, rest) = opened();
    let elbow = index_of("left_elbow_pitch_joint");
    let mut limits = vec![FeedforwardLimit::default(); rest.len()];
    limits[elbow] = FeedforwardLimit {
        velocity: Some(0.5),
        torque: Some(5.0),
    };
    chain.set_feedforward_limits(&limits).expect("bounds");
    let range = chain.commandable()[elbow];
    let mut targets = rest.clone();
    let mut wanted = vec![Feedforward::default(); rest.len()];
    // At the upper edge, a push upwards has only MAX_COAST of room past it (what a stalled
    // host may coast anyway): the unloaded settling point (kd v + tau) / kp stays within it.
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let excursion = |applied: Feedforward| {
        axes[elbow].kd.mul_add(applied.velocity, applied.torque) / axes[elbow].kp
    };
    targets[elbow] = range.upper;
    wanted[elbow] = Feedforward {
        velocity: 0.4,
        torque: 4.0,
    };
    chain.command_with(&targets, &wanted).expect("command");
    let at_edge = excursion(chain.applied_feedforward()[elbow]);
    assert!(at_edge > 0.0 && at_edge <= MAX_COAST + 1e-9, "{at_edge}");
    // 0.05 rad inside it, within 0.05 rad more.
    targets[elbow] = range.upper - 0.05;
    chain.command_with(&targets, &wanted).expect("command");
    let inside = excursion(chain.applied_feedforward()[elbow]);
    assert!(
        inside > 0.0 && inside <= 0.05 + MAX_COAST + 1e-9,
        "{inside}"
    );
    // Pushing away from the edge it is whole.
    wanted[elbow] = Feedforward {
        velocity: -0.4,
        torque: -4.0,
    };
    chain.command_with(&targets, &wanted).expect("command");
    assert_eq!(chain.applied_feedforward()[elbow], wanted[elbow]);
    let _ = bus;
    // A stop clears what the record says was sent.
    let _ = chain.stop();
    assert!(
        chain
            .applied_feedforward()
            .iter()
            .all(|ff| *ff == Feedforward::default())
    );
}

#[test]
fn a_geared_axis_scales_velocity_by_its_reduction_and_torque_by_its_inverse() {
    use sx_damiao_can::b601::b601;
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
    let nominal = &drives["b601-dm"]["nominal"];
    let drive = InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("instance"),
    )
    .expect("drive");
    let axes = b601(&drive).expect("axes");
    let gripper = axes
        .iter()
        .position(|axis| (axis.axis.binding.reduction - 1.0).abs() > 1e-9)
        .expect("a geared axis");
    let gain = axes[gripper]
        .axis
        .actuator_from_joint(axes[gripper].axis.zero + 1.0);
    let bus = Sim::for_axes(&axes);
    let mut chain = DamiaoChain::open(bus.clone(), axes.clone()).expect("opens");
    let mut limits = vec![FeedforwardLimit::default(); axes.len()];
    limits[gripper] = FeedforwardLimit {
        velocity: Some(1.0),
        torque: Some(100.0),
    };
    chain.set_feedforward_limits(&limits).expect("bounds");
    let mut targets = chain.last_observed().expect("rest");
    let range = chain.commandable()[gripper];
    targets[gripper] = f64::midpoint(range.lower, range.upper);
    let mut wanted = vec![Feedforward::default(); axes.len()];
    // Small enough at the joint that nothing clamps: 0.01 rad/s and 0.5 N (or N·m).
    wanted[gripper] = Feedforward {
        velocity: 0.01,
        torque: 0.5,
    };
    chain.command_with(&targets, &wanted).expect("command");
    let reported = chain.limits()[gripper];
    let limit = [
        f64::from(reported.p_max),
        f64::from(reported.v_max),
        f64::from(reported.t_max),
    ];
    let motor = &axes[gripper];
    assert_eq!(
        last_mit(&bus, motor.axis.binding.bus_id),
        mit_bytes(
            motor.kp,
            motor.kd,
            motor.axis.actuator_from_joint(targets[gripper]),
            gain * 0.01,
            0.5 / gain,
            limit
        )
    );
    let applied = chain.applied_feedforward()[gripper];
    assert!((applied.velocity - 0.01).abs() < 1e-12 && (applied.torque - 0.5).abs() < 1e-12);
}
