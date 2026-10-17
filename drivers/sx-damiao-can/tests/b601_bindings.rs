//! The B601 chain is composed from exactly the registry's rendered bindings, and the vendor's
//! gripper transmission is the registry's reduction.

// Equal means bit-for-bit equal: both sides are the same f64 values.
#![allow(clippy::float_cmp)]

use serde_json::Value;
use sx_damiao_can::b601::{
    BINDINGS, GRIPPER_FINGER_OPEN_M, GRIPPER_JOINT, GRIPPER_MOTOR_OPEN_RAD, b601,
};
use sx_embodiment_drivers::{BodyBindings, InstanceDrive};

/// The B601-DM's nominal instance, as the registry renders its drive.
fn nominal() -> InstanceDrive {
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
    InstanceDrive::parse(
        &drives["b601-dm"]["nominal"].to_string(),
        drives["b601-dm"]["nominal"]["instance"]
            .as_str()
            .expect("an instance"),
    )
    .expect("a drive")
}

#[test]
fn the_b601_chain_is_the_registrys_bindings_in_native_state_order() {
    let body = BodyBindings::parse(BINDINGS).expect("bindings");
    assert_eq!(body.embodiment, "b601-dm");
    let chain = b601(&nominal()).expect("the B601-DM composes");
    let axes: Vec<_> = chain.iter().map(|motor| motor.axis.clone()).collect();
    assert_eq!(axes, body.axes);
    for motor in &chain {
        assert_eq!(motor.master_id, motor.axis.binding.bus_id + 0x10);
        let range = motor.commandable();
        assert!(motor.axis.lower <= range.lower && range.upper <= motor.axis.upper);
    }
}

#[test]
fn the_registrys_gripper_reduction_is_the_vendors_transmission() {
    let body = BodyBindings::parse(BINDINGS).expect("bindings");
    let gripper = body.axis(GRIPPER_JOINT).expect("gripper");
    assert_eq!(gripper.binding.sign, -1);
    assert_eq!(
        gripper.binding.reduction,
        -GRIPPER_MOTOR_OPEN_RAD / GRIPPER_FINGER_OPEN_M
    );
}
