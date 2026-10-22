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
    InstanceDrive::parse(
        &drives[body][which].to_string(),
        drives[body][which]["instance"]
            .as_str()
            .expect("an instance"),
    )
    .expect("a drive")
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

#[cfg(feature = "sim")]
#[test]
fn measured_upper_body_reads_are_fresh_complete_and_never_enable_or_move() {
    use sx_damiao_can::{chain::DamiaoChain, sim::Sim};
    use sx_embodiment_drivers::ActuatorChain;
    let axes = humanoid_upper_body(&drive("sentient-humanoid-upper-body", "measured")).unwrap();
    let bus = Sim::for_axes(&axes);
    let mut chain = DamiaoChain::open(bus.clone(), axes.clone()).unwrap();
    let first = chain.read_positions().unwrap().unwrap();
    assert_eq!(first.len(), 11);
    let id = axes[0].axis.binding.bus_id;
    bus.0.lock().unwrap().motors.get_mut(&id).unwrap().q = 0.2;
    let second = chain.read_positions().unwrap().unwrap();
    assert!((second[0] - axes[0].axis.joint_from_actuator(0.2)).abs() < 0.001);
    assert!((second[0] - first[0]).abs() > 0.1);
    assert!(
        bus.0
            .lock()
            .unwrap()
            .motors
            .values()
            .all(|motor| motor.state == 0)
    );
    assert!((bus.0.lock().unwrap().motors[&id].q - 0.2).abs() < 1e-12);
    // The successful prior cache cannot satisfy a subsequent missing motor reply.
    bus.0.lock().unwrap().dead.insert(id);
    assert!(chain.read_positions().is_err());
    bus.0.lock().unwrap().dead.remove(&id);
    bus.0.lock().unwrap().motors.get_mut(&id).unwrap().state = 8;
    assert!(chain.read_positions().is_err());
}
