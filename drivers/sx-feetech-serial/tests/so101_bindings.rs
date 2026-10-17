//! The SO-101 chain is composed from exactly the registry's rendered bindings.

use sx_embodiment_drivers::BodyBindings;
use sx_feetech_serial::MotorCalibration;
use sx_feetech_serial::so101::{BINDINGS, so101};

#[test]
fn the_so101_chain_is_the_registrys_bindings_in_native_state_order() {
    let body = BodyBindings::parse(BINDINGS).expect("bindings");
    assert_eq!(body.embodiment, "so101");
    let calibration = MotorCalibration {
        drive_mode: false,
        homing_offset: 0,
        range_min: 0,
        range_max: 4095,
    };
    let chain = so101([calibration; 6]).expect("the SO-101 composes");
    let axes: Vec<_> = chain.iter().map(|servo| servo.axis.clone()).collect();
    assert_eq!(axes, body.axes);
}
