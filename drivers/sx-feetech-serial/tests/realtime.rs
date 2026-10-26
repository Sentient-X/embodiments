//! A steady Feetech tick allocates nothing: the SO-101's six STS3215 servos commanded (SYNC
//! WRITE, then SYNC READ) and read, by the chain and by the read-only observer, on a simulated
//! bus that itself allocates nothing once warmed.

use sx_embodiment_drivers::lerobot::MotorCalibration;
use sx_embodiment_drivers::{ActuatorChain, Observe};
use sx_feetech_serial::FeetechChain;
use sx_feetech_serial::body::ServoSetup;
use sx_feetech_serial::observe::FeetechObserver;
use sx_feetech_serial::sim::{SimBus, Steady};
use sx_feetech_serial::so101::so101;

const CALIBRATION: MotorCalibration = MotorCalibration {
    drive_mode: false,
    homing_offset: -12,
    range_min: 800,
    range_max: 3200,
};

fn bus() -> Steady {
    Steady::from(&SimBus::sts3215(
        &(1..=6).map(|id| (id, CALIBRATION)).collect::<Vec<_>>(),
    ))
}

#[test]
fn a_steady_command_and_read_tick_allocates_nothing() {
    let servos = so101([CALIBRATION; 6]).expect("composes");
    let mut chain = FeetechChain::open(bus(), servos).expect("opens");
    let rest = chain.last_observed().expect("rest").to_vec();
    // The first command enables every servo: not a steady tick.
    chain.command(&rest).expect("the first command");
    chain.read_positions();
    let ticks = allocation_counter::measure(|| {
        for _ in 0..100 {
            let receipt = chain.command(&rest).expect("a steady command");
            assert_eq!(receipt.observed.len(), 6);
            let readings = chain.read_positions();
            assert!(
                readings
                    .iter()
                    .all(sx_embodiment_drivers::AxisReading::is_fresh)
            );
        }
    });
    assert_eq!(
        ticks.count_total, 0,
        "100 commands and reads allocated {} times ({} bytes)",
        ticks.count_total, ticks.bytes_total
    );
    let seen = allocation_counter::measure(|| {
        std::hint::black_box(vec![0_u8; 16]);
    });
    assert!(seen.count_total > 0, "the counter sees an allocation");
}

#[test]
fn a_steady_read_only_observation_allocates_nothing() {
    let servos = so101([CALIBRATION; 6]).expect("composes");
    let setups: Vec<ServoSetup> = servos
        .iter()
        .map(|servo| ServoSetup {
            joint: "unused",
            norm_mode: servo.norm_mode,
            configuration: servo.configuration,
        })
        .collect();
    let axes = servos.iter().map(|servo| servo.axis.clone()).collect();
    let supplied = vec![Some(CALIBRATION); 6];
    let mut observer = FeetechObserver::open(bus(), axes, &setups, &supplied).expect("observes");
    assert!(
        observer
            .read_positions()
            .iter()
            .all(sx_embodiment_drivers::AxisReading::is_fresh)
    );
    let reads = allocation_counter::measure(|| {
        for _ in 0..100 {
            observer.read_positions();
        }
    });
    assert_eq!(
        reads.count_total, 0,
        "100 observations allocated {} times ({} bytes)",
        reads.count_total, reads.bytes_total
    );
}
