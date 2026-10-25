//! What keeps the static upper body from moving where or how it should not, on the simulated
//! Damiao bus: each motor's communication timeout, the one-turn window, the resting check at
//! open, and the governor's slew from where the body rests.

use std::time::Duration;

use serde_json::Value;
use sx_damiao_can::chain::{DamiaoChain, DamiaoChainError};
use sx_damiao_can::sim::Sim;
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_drivers::can::{CanFrame, FRAME_ALLOWANCE};
use sx_embodiment_drivers::{ActuatorChain, AxisLimits, Governor, InstanceDrive};

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

#[test]
fn open_writes_every_motors_timeout_to_ram_and_reads_it_back() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let bus = Sim::for_axes(&axes);
    DamiaoChain::open(bus.clone(), axes.clone()).expect("opens");
    // 0.75 s in the register's 50 µs counts.
    let counts = 15_000_u32.to_le_bytes();
    let state = bus.state();
    for motor in &axes {
        let id = motor.axis.binding.bus_id.to_le_bytes();
        let write = CanFrame {
            id: 0x7FF,
            data: [
                id[0], id[1], 0x55, 0x09, counts[0], counts[1], counts[2], counts[3],
            ],
        };
        let read = CanFrame {
            id: 0x7FF,
            data: [id[0], id[1], 0x33, 0x09, 0, 0, 0, 0],
        };
        let write_at = state
            .frames
            .iter()
            .position(|frame| *frame == write)
            .expect("the TIMEOUT write");
        assert!(
            state.frames[write_at..].contains(&read),
            "the TIMEOUT read follows the write"
        );
        assert_eq!(
            state.motors[&motor.axis.binding.bus_id].registers[&0x09],
            counts
        );
    }
    // Never saved to flash: no frame carries the save command (0xAA).
    assert!(
        state
            .frames
            .iter()
            .all(|frame| !(frame.id == 0x7FF && frame.data[2] == 0xAA))
    );
}

#[test]
fn a_motor_that_does_not_hold_its_timeout_refuses_the_open() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let first = axes[0].axis.binding.bus_id;
    // A timeout below one count is refused before any frame.
    let bus = Sim::for_axes(&axes);
    let mut zero = axes.clone();
    zero[0].can_timeout = Some(Duration::from_micros(10));
    assert!(matches!(
        DamiaoChain::open(bus.clone(), zero),
        Err(DamiaoChainError::TimeoutRange(id)) if id == first
    ));
    assert_eq!(bus.sent(), 0, "refused before any frame");
    // Firmware that echoes the write but does not hold it fails the read-back.
    bus.state()
        .motors
        .get_mut(&first)
        .expect("first")
        .ignored
        .insert(0x09);
    assert!(matches!(
        DamiaoChain::open(bus.clone(), axes),
        Err(DamiaoChainError::TimeoutNotConfirmed { bus_id, counts: 15_000 }) if bus_id == first
    ));
    assert!(bus.state().motors.values().all(|motor| motor.state == 0));
}

#[test]
fn a_forearm_resting_past_its_one_turn_window_refuses_the_open() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let forearm = index_of("left_forearm_yaw_joint");
    let bus = Sim::for_axes(&axes);
    let id = axes[forearm].axis.binding.bus_id;
    // Inside the declared -6.83..1.88 but more than half a turn from zero.
    bus.state().motors.get_mut(&id).expect("forearm").q = -4.0;
    match DamiaoChain::open(bus.clone(), axes.clone()) {
        Err(DamiaoChainError::RestingOutside { bus_id, .. }) => assert_eq!(bus_id, id),
        other => panic!("expected a refusal, got {:?}", other.map(|_| ())),
    }
    assert!(
        bus.state().motors.values().all(|motor| motor.state == 0),
        "every motor is left disabled"
    );
    // Just past its stop, inside the tolerance, it opens.
    bus.state().motors.get_mut(&id).expect("forearm").q = -std::f64::consts::PI - 0.02;
    DamiaoChain::open(bus, axes).expect("opens at its stop");
}

#[test]
fn an_upper_arm_reading_its_alias_refuses_the_open() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let yaw = index_of("left_upper_arm_yaw_joint");
    let bus = Sim::for_axes(&axes);
    let id = axes[yaw].axis.binding.bus_id;
    // A joint at -3.78 rad reads as 2.5 on a single-turn encoder: outside -5.12..0.81.
    bus.state().motors.get_mut(&id).expect("upper arm").q = 2.5;
    assert!(matches!(
        DamiaoChain::open(bus, axes),
        Err(DamiaoChainError::RestingOutside { bus_id, .. }) if bus_id == id
    ));
}

#[test]
fn the_governor_starts_from_where_the_body_rests_and_slews() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let bus = Sim::for_axes(&axes);
    let elbow = index_of("right_elbow_pitch_joint");
    bus.state()
        .motors
        .get_mut(&axes[elbow].axis.binding.bus_id)
        .expect("elbow")
        .q = 1.0;
    let chain = DamiaoChain::open(bus.clone(), axes).expect("opens");
    let rest = chain.last_observed().expect("observed at open");
    assert!((rest[elbow] - 1.0).abs() < 1e-3);
    let limits = vec![
        AxisLimits {
            max_speed: 0.25,
            max_tracking_error: 0.2,
        };
        15
    ];
    let mut governed = Governor::new(chain, limits).expect("limits");
    let mut target = rest.clone();
    target[elbow] = 2.5;
    let receipt = governed.command(&target).expect("a governed command");
    // The elbow moved at most one 20 ms step at 0.25 rad/s from where it rested.
    assert!((receipt.observed[elbow] - rest[elbow]).abs() <= 0.005 + 1e-3);
    assert!(governed.stop().is_proven());
    // Fifteen motors on a USB bridge: each disable and refresh budgets the default allowance.
    assert!(governed.stop_budget() >= FRAME_ALLOWANCE * 30);
}
