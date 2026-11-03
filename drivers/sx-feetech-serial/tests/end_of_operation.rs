//! The static upper body's SO-101 neck at the end of an operation, on a simulated bus of
//! STS3215 servos that move toward their goals at the approach limit the chain sets on them:
//! by default the governor returns the neck to its declared home (the camera tilt's -2.2 rad
//! included) and holds it there until a person releases it; `Release` disables it in place; a
//! neck nothing drove is not moved; and an axis that stopped answering ends held where each
//! servo stands, with no motion.

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
fn the_neck_returns_home_smoothly_holds_there_and_releases_on_a_persons_call() {
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
        assert_eq!(bus.state().torque(id), 1, "servo {id} held at home");
    }
    // Torque comes off only on a person's call: a second Ctrl-C, as sx-jog waits for it.
    let released = governor.hold_until_released(&|| true, &mut || {});
    assert!(
        released.is_proven() && released.held.is_empty(),
        "{released:?}"
    );
    assert!((1..=6).all(|id| bus.state().torque(id) == 0));
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
fn a_servo_that_stopped_answering_holds_the_rest_where_they_stand() {
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
    let _ = goals;
    let presents: Vec<u16> = [1, 2, 4, 5, 6].iter().map(|id| bus.present(*id)).collect();
    let state = bus.state();
    // Held where each servo stands (2026-11-04): its goal is its present position, no motion.
    assert_eq!(
        [1, 2, 4, 5, 6]
            .iter()
            .map(|id| state.goal(*id))
            .collect::<Vec<_>>(),
        presents,
        "held where it stands"
    );
    // Only reads and the stop's own frames were written: no SYNC WRITE of a goal.
    assert!(
        state.written[writes..]
            .iter()
            .all(|packet| packet.get(4) != Some(&0x83)),
        "a goal was sent after the axis stopped answering"
    );
    for id in [1, 2, 4, 5, 6] {
        assert_eq!(state.torque(id), 1, "servo {id} held where it was");
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

/// A stop that holds: the servos it holds keep torque on with their goal where they stand, so
/// nothing moves and nothing drops; the ones it releases are disabled as before; a neck
/// nothing drove is held by nothing.
#[test]
fn a_held_stop_keeps_torque_where_each_servo_stands_and_releases_the_rest() {
    use sx_embodiment_drivers::Rest;
    let bus = resting();
    let (mut governor, _clock) = neck(&bus);
    let mut rests = vec![Rest::Hold { gravity: 0.0 }; 6];
    rests[0] = Rest::Release;
    let untouched = governor.stop_holding(&rests);
    assert!(
        untouched.held.is_empty() && untouched.is_proven(),
        "{untouched:?}"
    );
    let rest = governor.last_observed().expect("rest");
    governor.command(&rest).expect("enable where it rests");
    let before: Vec<u16> = (1..=6).map(|id| bus.present(id)).collect();
    let report = governor.stop_holding(&rests);
    assert!(report.is_proven(), "{report:?}");
    assert_eq!(ids(&report.held), vec![2, 3, 4, 5, 6], "{report:?}");
    bus.advance(Duration::from_secs(2));
    let receipt = governor.keep_holding().expect("read");
    assert_eq!(receipt.observed.len(), 6);
    let after: Vec<u16> = (1..=6).map(|id| bus.present(id)).collect();
    assert_eq!(before, after, "nothing moved");
    assert_eq!(bus.state().torque(1), 0, "the released servo is disabled");
    for id in 2..=6 {
        assert_eq!(bus.state().torque(id), 1, "servo {id} holds");
        let present = bus.present(id);
        assert_eq!(
            bus.state().goal(id),
            present,
            "servo {id} holds where it is"
        );
    }
    // A Park's end releases every one.
    let parked = governor.stop();
    assert!(parked.held.is_empty() && parked.is_proven(), "{parked:?}");
    assert!((1..=6).all(|id| bus.state().torque(id) == 0));
}

/// The held motors' bus addresses, in order.
fn ids(held: &[sx_embodiment_drivers::Motor]) -> Vec<u16> {
    held.iter().map(|motor| motor.bus_id).collect()
}
