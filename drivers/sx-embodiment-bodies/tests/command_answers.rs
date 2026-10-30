//! Each driver's answer to a command reports exactly what the registry declares for its bus
//! (`COMMAND_ANSWERS`, rendered into the vocabulary), read through `Reports`: a caller asks the
//! bus what its answers carry and never names a vendor type.

use sx_damiao_can::chain::{DAMIAO_CAN, DamiaoChain, MotorFeedback, MotorState};
use sx_dynamixel_serial::DYNAMIXEL_SERIAL;
use sx_dynamixel_serial::chain::DynamixelChain;
use sx_embodiment_bodies::{Port, SerialLink};
use sx_embodiment_drivers::{ActuatorBus, ActuatorChain, CommandAnswer, Reports};
use sx_feetech_serial::FEETECH_SERIAL;
use sx_feetech_serial::chain::FeetechChain;

/// What `answered` reports, as a declaration reads.
fn reported<F: Reports>(answered: &F) -> CommandAnswer {
    CommandAnswer {
        reports_motion: answered.motion().is_some(),
        reports_temperature: answered.temperature().is_some(),
    }
}

/// What chain `C`'s answer reports, for a chain whose answer carries no reading to vary.
fn any_answer<C: ActuatorChain>() -> CommandAnswer {
    reported(&C::Feedback::default())
}

fn declared(bus: ActuatorBus) -> CommandAnswer {
    bus.command_answer()
        .unwrap_or_else(|| panic!("the registry declares {}'s command answer", bus.wire()))
}

/// A Damiao motor that answered a command, every reading finite.
fn answered_damiao() -> <DamiaoChain<Port> as ActuatorChain>::Feedback {
    MotorFeedback {
        bus_id: 3,
        state: MotorState::Enabled,
        position: 0.1,
        velocity: -0.4,
        torque: 1.5,
        mos_celsius: 41,
        rotor_celsius: 47,
    }
}

#[test]
fn every_bus_declares_its_command_answer() {
    for bus in ActuatorBus::ALL {
        declared(bus);
    }
}

#[test]
fn every_drivers_answer_reports_what_its_bus_declares() {
    assert_eq!(reported(&answered_damiao()), declared(DAMIAO_CAN));
    assert_eq!(
        any_answer::<FeetechChain<SerialLink>>(),
        declared(FEETECH_SERIAL)
    );
    assert_eq!(
        any_answer::<DynamixelChain<SerialLink>>(),
        declared(DYNAMIXEL_SERIAL)
    );
}

#[test]
fn a_damiao_answer_reports_its_measurement_in_motor_coordinates_and_nothing_unasked() {
    let answered = answered_damiao();
    assert_eq!(answered.motion(), Some((f64::from(-0.4_f32), 1.5)));
    let (bus_id, temperature) = answered.temperature().expect("temperatures");
    assert_eq!((bus_id, temperature.hottest()), (3, 47));
    let unasked = MotorFeedback {
        state: MotorState::Unavailable,
        ..answered
    };
    assert_eq!((unasked.motion(), unasked.temperature()), (None, None));
    let unmeasured = MotorFeedback {
        velocity: f32::NAN,
        ..answered
    };
    assert_eq!(unmeasured.motion(), None);
}
