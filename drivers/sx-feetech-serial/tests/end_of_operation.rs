//! The static upper body's SO-101 neck at the end of an operation, on a simulated bus of
//! STS3215 servos that move toward their goals at the approach limit the chain sets on them:
//! by default the governor returns the neck to its declared home (the camera tilt's -2.2 rad
//! included) and then disables torque; `Release` disables it in place; a neck nothing
//! drove is not moved; and an axis that stopped answering ends in the plain stop with no motion.

use std::cell::Cell;
use std::rc::Rc;
use std::time::{Duration, Instant};

use sx_embodiment_drivers::{
    ActuatorChain, AtEnd, AxisLimits, BRING_UP_APPROACH, BodyBindings, Clock, Ending, Governor,
    MAX_STEP_INTERVAL, ReturnFailure,
};
use sx_feetech_serial::FeetechChain;
use sx_feetech_serial::MotorCalibration;
use sx_feetech_serial::sim::SimBus;
use sx_feetech_serial::static_upper_body::{BINDINGS, NECK};

/// Every unit calibrated over its whole turn, its zero at mid-range (2047 ticks).
const CALIBRATION: MotorCalibration = MotorCalibration {
    drive_mode: false,
    homing_offset: 0,
    range_min: 0,
    range_max: 4095,
};

/// Ticks per radian.
const TICKS: f64 = 4096.0 / std::f64::consts::TAU;

#[derive(Clone)]
struct Manual(Rc<Cell<Instant>>);

impl Clock for Manual {
    fn now(&self) -> Instant {
        self.0.get()
    }
}

/// The neck opened on `bus`, governed at the station's bring-up speed and tracking bound.
fn neck(bus: &SimBus) -> (Governor<FeetechChain<SimBus>, Manual>, Manual) {
    let axes = NECK
        .axes(BodyBindings::parse(BINDINGS).expect("bindings"))
        .expect("the neck's axes");
    let servos = NECK.servos(axes, &[CALIBRATION; 6]).expect("servos");
    let chain = FeetechChain::open(bus.clone(), servos).expect("the neck opens");
    let clock = Manual(Rc::new(Cell::new(Instant::now())));
    let limits = vec![
        AxisLimits {
            max_speed: BRING_UP_APPROACH.speed,
            max_tracking_error: 0.2,
            max_acceleration: None,
        };
        6
    ];
    let governor = Governor::with_clock(chain, limits, clock.clone()).expect("limits");
    (governor, clock)
}

/// A neck resting away from home: each joint some way off, the camera tilt at -1.0 rad.
fn resting() -> SimBus {
    let bus = SimBus::sts3215(&(1..=6).map(|id| (id, CALIBRATION)).collect::<Vec<_>>());
    for (id, rad) in (1..=6).zip([0.3, -0.45, 0.4, -0.25, 0.15, -1.0]) {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        bus.rest_at(id, (2047.0 + rad * TICKS).round() as u16);
    }
    bus
}

fn end(
    governor: &mut Governor<FeetechChain<SimBus>, Manual>,
    clock: &Manual,
    bus: &SimBus,
    at_end: AtEnd,
) -> Ending {
    let (clock, bus) = (clock.clone(), bus.clone());
    governor.end(
        at_end,
        &[],
        None,
        &mut || {
            clock.0.set(clock.0.get() + MAX_STEP_INTERVAL);
            bus.advance(MAX_STEP_INTERVAL);
        },
        &|| false,
    )
}

fn radians(bus: &SimBus, id: u8) -> f64 {
    (f64::from(bus.present(id)) - 2047.0) / TICKS
}

#[test]
fn the_neck_returns_home_smoothly_then_disables_torque() {
    let bus = resting();
    let (mut governor, clock) = neck(&bus);
    let homes: Vec<f64> = governor
        .axes()
        .iter()
        .map(|axis| axis.home.expect("every neck joint declares a home"))
        .collect();
    assert_eq!(homes, [0.0, 0.0, 0.0, 0.0, 0.0, 0.4135]);
    // The operation drove it: enabled, holding where it rests.
    let rest = governor.last_observed().expect("rest");
    governor.command(&rest).expect("enable where it rests");
    let ending = end(&mut governor, &clock, &bus, AtEnd::default());
    let Ending::Returned { to, took, stop } = &ending else {
        panic!("expected a return home, got {ending}");
    };
    assert_eq!(*to, AtEnd::Home);
    assert!(stop.is_proven(), "{stop:?}");
    // The camera tilt's 1.41 rad at half the 0.25 rad/s cap.
    assert!(*took >= Duration::from_secs_f64(11.3), "{took:?}");
    for (id, home) in (1..=6).zip(&homes) {
        assert!(
            (radians(&bus, id) - home).abs() < 0.01,
            "servo {id} at {} rad, home {home}",
            radians(&bus, id)
        );
        assert_eq!(bus.state().torque(id), 0, "servo {id} disabled at home");
    }
}

#[test]
fn release_disables_in_place() {
    let bus = resting();
    let (mut governor, clock) = neck(&bus);
    let rest = governor.last_observed().expect("rest");
    governor.command(&rest).expect("enable where it rests");
    let before: Vec<u16> = (1..=6).map(|id| bus.present(id)).collect();
    let ending = end(&mut governor, &clock, &bus, AtEnd::Release);
    assert!(matches!(ending, Ending::Released { .. }), "{ending}");
    let after: Vec<u16> = (1..=6).map(|id| bus.present(id)).collect();
    assert_eq!(before, after, "nothing moved");
    assert!((1..=6).all(|id| bus.state().torque(id) == 0));
}

#[test]
fn a_servo_that_stopped_answering_is_a_plain_stop_with_no_motion() {
    let bus = resting();
    let (mut governor, clock) = neck(&bus);
    let rest = governor.last_observed().expect("rest");
    governor.command(&rest).expect("enable where it rests");
    let goals: Vec<u16> = (1..=6).map(|id| bus.state().goal(id)).collect();
    let writes = bus.state().written.len();
    bus.kill(3);
    let ending = end(&mut governor, &clock, &bus, AtEnd::Home);
    let Ending::FellBack { failure, .. } = &ending else {
        panic!("expected the plain stop, got {ending}");
    };
    assert_eq!(
        failure,
        &ReturnFailure::Unobserved {
            joint: "so101_elbow_flex_joint".into()
        }
    );
    let state = bus.state();
    assert_eq!(
        (1..=6).map(|id| state.goal(id)).collect::<Vec<_>>(),
        goals,
        "no goal was written"
    );
    // Only reads and the stop's own frames were written: no SYNC WRITE of a goal.
    assert!(
        state.written[writes..]
            .iter()
            .all(|packet| packet.get(4) != Some(&0x83)),
        "a goal was sent after the axis stopped answering"
    );
    for id in [1, 2, 4, 5, 6] {
        assert_eq!(state.torque(id), 0, "servo {id} disabled where it was");
    }
}

#[test]
fn a_neck_nothing_drove_is_not_moved_at_the_end() {
    let bus = resting();
    let (mut governor, clock) = neck(&bus);
    let before: Vec<u16> = (1..=6).map(|id| bus.present(id)).collect();
    let ending = end(&mut governor, &clock, &bus, AtEnd::Home);
    assert!(matches!(ending, Ending::Rested { .. }), "{ending}");
    assert_eq!(
        before,
        (1..=6).map(|id| bus.present(id)).collect::<Vec<_>>()
    );
    assert!((1..=6).all(|id| bus.state().torque(id) == 0));
}
