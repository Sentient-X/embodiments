//! The humanoid upper body's chain is composed from exactly the registry's rendered bindings,
//! calibrated by the instance drive the registry resolves.

// Equal means bit-for-bit equal: both sides are the same f64 values.
#![allow(clippy::float_cmp)]

use serde_json::Value;
use sx_damiao_can::chain::motor_type;
use sx_damiao_can::humanoid::{BINDINGS, MASTER_ID_OFFSET, humanoid_upper_body};
use sx_embodiment_drivers::{BodyBindings, BodyError, InstanceDrive};

fn drive(body: &str, which: &str) -> InstanceDrive {
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
    InstanceDrive::parse(&drives[body][which].to_string()).expect("a drive")
}

#[test]
fn the_upper_body_chain_is_the_registrys_bindings_in_native_state_order() {
    let body = BodyBindings::parse(BINDINGS).expect("bindings");
    assert_eq!(body.embodiment, "sentient-humanoid-upper-body");
    let chain = humanoid_upper_body(&drive("sentient-humanoid-upper-body", "nominal"))
        .expect("the upper body composes");
    let axes: Vec<_> = chain.iter().map(|motor| motor.axis.clone()).collect();
    assert_eq!(axes, body.axes);
    for motor in &chain {
        assert_eq!(
            motor.master_id,
            motor.axis.binding.bus_id + MASTER_ID_OFFSET
        );
        assert!(motor_type(motor.axis.binding.model).is_some());
        let range = motor.commandable();
        assert_eq!(
            (range.lower, range.upper),
            (motor.axis.lower, motor.axis.upper)
        );
    }
    // The waist linkage is in no chain; its motors are never enabled.
    assert!(
        chain
            .iter()
            .all(|motor| !motor.axis.joint.starts_with("waist_"))
    );
}

#[test]
fn an_instances_measured_joint_reaches_its_motor_and_its_range_follows() {
    let nominal =
        humanoid_upper_body(&drive("sentient-humanoid-upper-body", "nominal")).expect("composes");
    let measured =
        humanoid_upper_body(&drive("sentient-humanoid-upper-body", "measured")).expect("composes");
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
        humanoid_upper_body(&drive("b601-dm", "nominal")),
        Err(BodyError::Instance { .. })
    ));
}
