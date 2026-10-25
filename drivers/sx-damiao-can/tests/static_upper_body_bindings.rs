//! The static upper body's chain is composed from exactly the registry's rendered bindings and
//! the hardware profile's ids and gains, calibrated by the instance drive the registry
//! resolves.

// Equal means bit-for-bit equal: both sides are the same f64 values.
#![allow(clippy::float_cmp)]

use serde_json::Value;
use sx_damiao_can::body::DamiaoBody;
use sx_damiao_can::chain::{DAMIAO_CAN, DAMIAO_DM4310, DAMIAO_DM4340, DAMIAO_DM8009, motor_type};
use sx_damiao_can::static_upper_body::{BINDINGS, MASTER_ID_OFFSET, STATIC_UPPER_BODY};
use sx_embodiment_drivers::{BodyBindings, BodyError, InstanceDrive};

fn drive(body: &str, which: &str) -> InstanceDrive {
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
    InstanceDrive::parse(
        &drives[body][which].to_string(),
        drives[body][which]["instance"]
            .as_str()
            .expect("an instance"),
    )
    .expect("a drive")
}

#[test]
fn the_chain_is_the_registrys_bindings_with_the_hardware_profile() {
    let body = BodyBindings::parse(BINDINGS).expect("bindings");
    assert_eq!(body.embodiment, "sentient-static-upper-body");
    let found = DamiaoBody::of(&body.id)
        .expect("parses")
        .expect("registered");
    assert_eq!(found.bindings, BINDINGS);
    let chain = STATIC_UPPER_BODY
        .axes(&drive("sentient-static-upper-body", "nominal"))
        .expect("the static upper body composes");
    assert_eq!(chain.len(), 15);
    assert_eq!(STATIC_UPPER_BODY.joints.len(), 15);
    let axes: Vec<_> = chain.iter().map(|motor| motor.axis.clone()).collect();
    // The neck and grippers are the same document's Feetech servos, on their own ports.
    assert_eq!(axes, body.clone().on(DAMIAO_CAN).axes);
    assert_eq!(body.axes.len(), 25);
    // The hardware profile's send ids and models, torso then left then right.
    let table: [(&str, u16, _); 15] = [
        ("torso_joint", 0x1F, DAMIAO_DM8009),
        ("left_shoulder_pitch_joint", 0x18, DAMIAO_DM8009),
        ("left_shoulder_roll_joint", 0x19, DAMIAO_DM8009),
        ("left_upper_arm_yaw_joint", 0x1A, DAMIAO_DM4340),
        ("left_elbow_pitch_joint", 0x1B, DAMIAO_DM4340),
        ("left_forearm_yaw_joint", 0x1C, DAMIAO_DM4340),
        ("left_wrist_pitch_joint", 0x1D, DAMIAO_DM4310),
        ("left_wrist_roll_joint", 0x1E, DAMIAO_DM4310),
        ("right_shoulder_pitch_joint", 0x09, DAMIAO_DM8009),
        ("right_shoulder_roll_joint", 0x10, DAMIAO_DM8009),
        ("right_upper_arm_yaw_joint", 0x11, DAMIAO_DM4340),
        ("right_elbow_pitch_joint", 0x12, DAMIAO_DM4340),
        ("right_forearm_yaw_joint", 0x13, DAMIAO_DM4340),
        ("right_wrist_pitch_joint", 0x14, DAMIAO_DM4310),
        ("right_wrist_roll_joint", 0x15, DAMIAO_DM4310),
    ];
    for ((motor, gains), (joint, id, model)) in
        chain.iter().zip(STATIC_UPPER_BODY.joints).zip(table)
    {
        assert_eq!(motor.axis.joint, joint);
        assert_eq!(gains.joint, joint);
        assert_eq!(motor.axis.binding.bus_id, id);
        assert_eq!(motor.axis.binding.model, model);
        assert!(motor_type(model).is_some());
        assert_eq!(MASTER_ID_OFFSET, 0x20);
        assert_eq!(motor.master_id, id + MASTER_ID_OFFSET);
        assert_eq!((motor.kp, motor.kd), (gains.kp, gains.kd));
        // allisa_bot turns both wrist axes against their motors.
        let sign = if joint.contains("_wrist_") { -1 } else { 1 };
        assert_eq!(
            (motor.axis.binding.sign, motor.axis.binding.reduction),
            (sign, 1.0)
        );
        // The forearm yaws span more than one turn: each is commanded within one turn of zero.
        let range = motor.commandable();
        let expected = if joint.ends_with("forearm_yaw_joint") {
            assert!(motor.axis.upper - motor.axis.lower > std::f64::consts::TAU);
            (
                motor.axis.lower.max(-std::f64::consts::PI),
                motor.axis.upper.min(std::f64::consts::PI),
            )
        } else {
            (motor.axis.lower, motor.axis.upper)
        };
        assert_eq!((range.lower, range.upper), expected, "{joint}");
    }
}

#[test]
fn an_instances_measured_joint_reaches_its_motor_and_its_range_follows() {
    let nominal = STATIC_UPPER_BODY
        .axes(&drive("sentient-static-upper-body", "nominal"))
        .expect("composes");
    let measured = STATIC_UPPER_BODY
        .axes(&drive("sentient-static-upper-body", "measured"))
        .expect("composes");
    assert_eq!(measured[0].axis.zero, 0.1);
    assert_eq!(measured[0].axis.direction, -1);
    let (before, after) = (nominal[0].commandable(), measured[0].commandable());
    assert!((before.lower - after.lower).abs() < 1e-12);
    assert!((before.upper - after.upper).abs() < 1e-12);
    assert_eq!(&measured[1..], &nominal[1..]);
}

#[test]
fn another_bodys_drive_opens_nothing() {
    assert!(matches!(
        STATIC_UPPER_BODY.axes(&drive("sentient-humanoid-upper-body", "nominal")),
        Err(BodyError::Instance { .. })
    ));
}
