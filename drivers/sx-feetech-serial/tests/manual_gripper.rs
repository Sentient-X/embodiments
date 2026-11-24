//! Hand-guided fingers retain encoder observations without position goals or torque.
use sx_embodiment_drivers::{
    ActuatorChain, BodyBindings, CompliantCommand, CompliantError, Feedforward,
};
use sx_feetech_serial::sim::SimBus;
use sx_feetech_serial::static_upper_body::{BINDINGS, RIGHT_GRIPPER};
use sx_feetech_serial::{FeetechChain, MotorCalibration};

fn open() -> (SimBus, FeetechChain<SimBus>) {
    let calibration = MotorCalibration {
        drive_mode: false,
        homing_offset: 0,
        range_min: 1024,
        range_max: 3072,
    };
    let axes = RIGHT_GRIPPER
        .axes(BodyBindings::parse(BINDINGS).expect("bindings"))
        .expect("axes");
    let servos = RIGHT_GRIPPER
        .servos(axes, &[calibration; 2])
        .expect("servos");
    let bus = SimBus::sts3215(&[(1, calibration), (2, calibration)]);
    let chain = FeetechChain::open(bus.clone(), servos).expect("open");
    (bus, chain)
}

#[test]
fn fingers_release_read_manual_positions_without_writes_and_capture_on_return() {
    let (bus, mut chain) = open();
    let rest = chain.last_observed().expect("rest");
    chain.command(&rest).expect("powered hold");
    assert_eq!(bus.state().torque(1), 1);
    let commands = [Some(CompliantCommand::Released); 2];
    let ff = [Feedforward::default(); 2];
    let before = bus.state().written.len();
    chain
        .validate_compliant(&rest, &ff, &commands)
        .expect("preflight");
    assert_eq!(
        bus.state().written.len(),
        before,
        "preflight must never release"
    );
    chain
        .command_compliant(&rest, &ff, &commands)
        .expect("release");
    assert_eq!(bus.state().torque(1), 0);
    assert_eq!(bus.state().torque(2), 0);
    let before = bus.state().written.len();
    bus.rest_at(1, 2400);
    bus.rest_at(2, 1700);
    let observed = chain
        .command_compliant(&rest, &ff, &commands)
        .expect("manual read")
        .observed;
    assert!((observed[0] - chain.servos()[0].joint_from_raw(2400)).abs() < 1e-9);
    assert!((observed[1] - chain.servos()[1].joint_from_raw(1700)).abs() < 1e-9);
    chain
        .command_compliant(&observed, &ff, &commands)
        .expect("next read");
    assert!(
        chain
            .command_readings()
            .expect("fresh batch")
            .iter()
            .all(sx_embodiment_drivers::AxisReading::is_fresh)
    );
    assert!(
        bus.state().written[before..]
            .iter()
            .all(|packet| packet[4] == 0x82),
        "released ticks must only SYNC READ, not goals/disables/enables"
    );
    let before = bus.state().written.len();
    chain
        .command(&observed)
        .expect("capture current finger positions");
    assert_eq!(bus.state().torque(1), 1);
    assert_eq!(bus.state().torque(2), 1);
    let state = bus.state();
    for (id, raw) in [(1, 2400_u16), (2, 1700_u16)] {
        let written = &state.written[before..];
        let enable = written
            .iter()
            .position(|p| p[2] == id && p[4..] == [3, 40, 1, p[p.len() - 1]])
            .expect("enable");
        assert!(
            written[..enable]
                .iter()
                .any(|p| p[2] == id && p[4] == 3 && p[5] == 42 && p[6..8] == raw.to_le_bytes()),
            "current encoder must be preloaded before torque-on"
        );
    }
}

#[test]
fn partial_or_arm_release_is_refused_before_any_write() {
    let (bus, mut chain) = open();
    let rest = chain.last_observed().expect("rest");
    let ff = [Feedforward::default(); 2];
    let before = bus.state().written.len();
    for control in [
        [Some(CompliantCommand::Released), None],
        [Some(CompliantCommand::Torque {
            torque: 0.0,
            damping: 0.1,
        }); 2],
    ] {
        assert!(chain.command_compliant(&rest, &ff, &control).is_err());
        assert_eq!(bus.state().written.len(), before);
    }
    let calibration = MotorCalibration {
        drive_mode: false,
        homing_offset: 0,
        range_min: 1024,
        range_max: 3072,
    };
    let servos = sx_feetech_serial::so101::so101([calibration; 6]).expect("arm setup");
    let bus = SimBus::sts3215(&(1..=6).map(|id| (id, calibration)).collect::<Vec<_>>());
    let mut arm = FeetechChain::open(bus.clone(), servos).expect("arm");
    let rest = arm.last_observed().expect("rest");
    let before = bus.state().written.len();
    assert!(matches!(
        arm.command_compliant(
            &rest,
            &[Feedforward::default(); 6],
            &[Some(CompliantCommand::Released); 6]
        ),
        Err(CompliantError::Unsupported { .. })
    ));
    assert_eq!(bus.state().written.len(), before);
}

#[test]
fn unproven_disable_blocks_manual_mode_and_unexpected_power_is_not_fresh() {
    let (bus, mut chain) = open();
    let rest = chain.last_observed().expect("rest");
    chain.command(&rest).expect("hold");
    bus.state().ignored.insert((1, 40));
    let ff = [Feedforward::default(); 2];
    let commands = [Some(CompliantCommand::Released); 2];
    assert!(
        chain.command_compliant(&rest, &ff, &commands).is_err(),
        "ignored disable must fail"
    );
    assert!(
        chain.command(&rest).is_err(),
        "unproven release cannot resume motion"
    );
    bus.state().ignored.clear();
    assert!(chain.stop().is_proven());
    chain
        .command_compliant(&rest, &ff, &commands)
        .expect("proven release");
    bus.state().servos.get_mut(&1).expect("servo")[40] = 1;
    let receipt = chain
        .command_compliant(&rest, &ff, &commands)
        .expect("read only");
    assert!(
        !receipt.observed[0].is_finite(),
        "unexpected enabled torque must never look healthy"
    );
    assert!(!chain.command_readings().expect("batch")[0].is_fresh());
}
