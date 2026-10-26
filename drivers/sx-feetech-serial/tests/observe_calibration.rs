//! A read-only observer reads a unit with the calibration measured for it: its range may differ
//! from the EEPROM limits the servo still holds (they bound motion, which a read never makes),
//! while its homing offset, which the servo's reported position comes back through, must agree.

use sx_embodiment_drivers::lerobot::MotorCalibration;
use sx_embodiment_drivers::{AxisReading, Observe};
use sx_feetech_serial::FeetechChainError;
use sx_feetech_serial::body::ServoSetup;
use sx_feetech_serial::observe::FeetechObserver;
use sx_feetech_serial::sim::SimBus;
use sx_feetech_serial::so101::so101;

const HELD: MotorCalibration = MotorCalibration {
    drive_mode: false,
    homing_offset: -12,
    range_min: 800,
    range_max: 3200,
};

fn observe(supplied: MotorCalibration) -> Result<Vec<AxisReading>, FeetechChainError> {
    let bus = SimBus::sts3215(&(1..=6).map(|id| (id, HELD)).collect::<Vec<_>>());
    let servos = so101([HELD; 6]).expect("composes");
    let setups: Vec<ServoSetup> = servos
        .iter()
        .map(|servo| ServoSetup {
            joint: "unused",
            norm_mode: servo.norm_mode,
            configuration: servo.configuration,
        })
        .collect();
    let axes = servos.iter().map(|servo| servo.axis.clone()).collect();
    let mut supplied_all = vec![Some(HELD); 6];
    supplied_all[5] = Some(supplied);
    let mut observer = FeetechObserver::open(bus, axes, &setups, &supplied_all)?;
    Ok(observer.read_positions().to_vec())
}

#[test]
fn a_measured_range_wider_than_the_eeprom_limits_is_read_with_and_a_homing_mismatch_is_refused() {
    let held = observe(HELD).expect("observes");
    // The gripper (servo 6, a percent axis, resting mid-limits) measured wider on one side.
    let measured = observe(MotorCalibration {
        range_min: 800,
        range_max: 4000,
        ..HELD
    })
    .expect("observes with the measured range");
    let (Some(at_held), Some(at_measured)) = (held[5].position(), measured[5].position()) else {
        panic!("both read fresh: {held:?} {measured:?}");
    };
    assert!(
        (at_held - at_measured).abs() > 1e-6,
        "read with the measured range"
    );
    assert!(measured[..5].iter().all(AxisReading::is_fresh));
    // Another homing offset: the reported position means something else, so it is refused.
    assert!(matches!(
        observe(MotorCalibration {
            homing_offset: 40,
            ..HELD
        }),
        Err(FeetechChainError::Uncalibrated { bus_id: 6, .. })
    ));
}
