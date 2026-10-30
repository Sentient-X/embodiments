//! The static upper body at its rated limits on a simulated bus whose motors run the MIT law on
//! a loaded joint (`Sim::with_mit_dynamics`), not jumping to each target: what the speed, the
//! planned velocity, the gains' ceilings and the tracking stop do to a real lag, a stalled host
//! and a blocked joint (the refute of 2026-10-30). Simulated time: the governor's clock and the
//! motors' dynamics move together by each tick, with no sleeps.

use std::cell::Cell;
use std::rc::Rc;
use std::time::{Duration, Instant};

use serde_json::Value;
use sx_damiao_can::chain::{DamiaoChain, MAX_COAST};
use sx_damiao_can::sim::Sim;
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_drivers::{
    ActuatorChain, AxisLimits, Clock, Governor, GovernorError, InstanceDrive, MAX_STEP_INTERVAL,
};

const ELBOW: &str = "left_elbow_pitch_joint";
const SHOULDER: &str = "left_shoulder_pitch_joint";
/// How long a stalled host leaves the motors: past their 0.75 s watchdog.
const STALL: Duration = Duration::from_millis(900);

fn drive() -> InstanceDrive {
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
    let nominal = &drives["sentient-static-upper-body"]["nominal"];
    InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("an instance"),
    )
    .expect("a drive")
}

/// The governor's clock, moved by the test.
#[derive(Clone)]
struct Manual(Rc<Cell<Instant>>);

impl Clock for Manual {
    fn now(&self) -> Instant {
        self.0.get()
    }
}

struct Rig {
    governor: Governor<DamiaoChain<Sim>, Manual>,
    bus: Sim,
    clock: Manual,
}

impl Rig {
    /// The static upper body on a loaded MIT bus on manual time, governed at its full rated
    /// limits (each held to its gains' ceilings).
    fn new() -> Self {
        Self::new_at(1.0)
    }

    /// The same at `share` of the rated limits.
    fn new_at(share: f64) -> Self {
        let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
        let bus = Sim::for_axes(&axes).with_mit_dynamics().with_manual_time();
        let chain = DamiaoChain::open(bus.clone(), axes).expect("opens");
        let limits = AxisLimits::rated_all(chain.axes(), share).expect("rated");
        let clock = Manual(Rc::new(Cell::new(Instant::now())));
        let governor = Governor::with_clock(chain, limits, clock.clone()).expect("governed");
        Self {
            governor,
            bus,
            clock,
        }
    }

    fn index(&self, joint: &str) -> usize {
        self.governor
            .axes()
            .iter()
            .position(|axis| axis.joint == joint)
            .expect("a joint of the body")
    }

    /// One control period passes, for the governor and the motors alike.
    fn pass(&self, by: Duration) {
        self.clock.0.set(self.clock.0.get() + by);
        self.bus.advance_time(by);
    }

    fn q(&self, index: usize) -> f64 {
        let id = self.governor.axes()[index].binding.bus_id;
        self.bus.state().motors[&id].q
    }
}

/// The elbow's speed is its gains' ceiling, `(MAX_COAST + MAX_LAG) kp / kd` (1.31 rad/s for kp
/// 30.492, kd 4.655), below its motor's rated 3.77 rad/s; a move at it tracks inside the
/// bound and settles.
#[test]
fn a_full_speed_move_on_the_mit_law_tracks_and_settles() {
    let mut rig = Rig::new();
    let elbow = rig.index(ELBOW);
    let ceiling = (MAX_COAST + sx_damiao_can::chain::MAX_LAG) * 30.492 / 4.655;
    assert!((rig.governor.limits()[elbow].max_speed - ceiling).abs() < 1e-9);
    let mut target = rig.governor.last_observed().expect("rest").to_vec();
    target[elbow] -= 1.2;
    for _ in 0..150 {
        rig.governor
            .command(&target)
            .expect("no tracking stop at full speed on the MIT law");
        rig.pass(MAX_STEP_INTERVAL);
    }
    let q = rig.q(elbow);
    // Inside the friction band: 1.4 N m of Coulomb friction against kp 30.5 holds it up to
    // 0.046 rad short.
    assert!(
        (q - target[elbow]).abs() < 0.06,
        "settled at {q}, not {}",
        target[elbow]
    );
    assert!(rig.governor.stop().is_proven());
}

/// Drive `joint` toward the far end of its range until it moves at its governed top speed,
/// then stall the host past the watchdog: how far it went past the last position sent.
fn coast_at_top_speed(joint: &str) -> (f64, f64) {
    let mut rig = Rig::new();
    let index = rig.index(joint);
    let top = rig.governor.limits()[index].max_speed;
    let mut target = rig.governor.last_observed().expect("rest").to_vec();
    let range = rig.governor.commandable()[index];
    let rest = target[index];
    target[index] = if range.upper - rest >= rest - range.lower {
        range.upper
    } else {
        range.lower
    };
    let sign = (target[index] - rest).signum();
    let mut previous = rest;
    // Stall at the top speed, or at the fastest the range lets it reach (when it starts to
    // brake), whichever comes first.
    let mut speed: f64 = 0.0;
    for _ in 0..400 {
        rig.governor.command(&target).expect("moving");
        let sent = rig.governor.sent().expect("sent")[index];
        let now = sign * (sent - previous) / MAX_STEP_INTERVAL.as_secs_f64();
        previous = sent;
        rig.pass(MAX_STEP_INTERVAL);
        if now >= top - 1e-6 || now < speed - 1e-6 {
            break;
        }
        speed = now;
    }
    assert!(
        speed > 0.5 * top,
        "{joint} reached only {speed} of {top} rad/s"
    );
    // The host stalls: nothing more is sent until the watchdog has disabled the motors.
    let last = rig.governor.sent().expect("sent")[index];
    let mut farthest = f64::NEG_INFINITY;
    for _ in 0..(STALL.as_millis() / 5) {
        rig.pass(Duration::from_millis(5));
        farthest = farthest.max(sign * (rig.q(index) - last));
    }
    (top, farthest)
}

/// A host that stops commanding at a joint's top governed speed: each motor holds its last
/// frame until its 0.75 s watchdog. The planned velocity is capped so the velocity term
/// coasts at most `MAX_COAST`, and each axis is no faster than its momentum lets the held
/// position stop it within `MAX_COAST` more (`MAX_COAST sqrt(kp / J)`). Uncapped, the elbow
/// coasted 0.20 rad at its ceiling; without the momentum ceiling, the shoulder at its rated
/// 10.47 rad/s went far past.
#[test]
fn a_stalled_host_leaves_the_elbow_within_max_coast_of_its_last_position() {
    // Overdamped (kd/kp 0.15 s), its momentum adds next to nothing: the velocity term is the
    // coast, and the cap holds it to MAX_COAST.
    let (top, coast) = coast_at_top_speed(ELBOW);
    assert!(
        coast <= MAX_COAST + 0.03,
        "the elbow at {top} rad/s coasted {coast} rad past its last position"
    );
}

#[test]
fn a_stalled_host_leaves_a_joint_at_top_speed_within_two_coasts_of_its_last_position() {
    for joint in [
        SHOULDER,
        "right_shoulder_roll_joint",
        "left_wrist_pitch_joint",
    ] {
        let (top, coast) = coast_at_top_speed(joint);
        assert!(
            coast <= 2.0 * MAX_COAST + 0.02,
            "{joint} at {top} rad/s coasted {coast} rad past its last position"
        );
    }
}

/// A joint blocked while moving slowly (a hand, the coffee machine) stops by about the at-rest
/// 0.2 rad, so it pushes with no more than `kp` times that (6.4 N m on the elbow), whatever it
/// could do at speed.
#[test]
fn a_slow_joint_that_is_blocked_stops_at_the_at_rest_bound() {
    let mut rig = Rig::new();
    let elbow = rig.index(ELBOW);
    let id = rig.governor.axes()[elbow].binding.bus_id;
    let mut target = rig.governor.last_observed().expect("rest").to_vec();
    rig.governor.command(&target).expect("held");
    rig.bus
        .state()
        .motors
        .get_mut(&id)
        .and_then(|motor| motor.dynamics.as_mut())
        .expect("a loaded motor")
        .inertia = 1e9;
    let limit = (0..250)
        .find_map(|_| {
            rig.pass(MAX_STEP_INTERVAL);
            target[elbow] -= 0.1 * MAX_STEP_INTERVAL.as_secs_f64();
            match rig.governor.command(&target) {
                Ok(_) => None,
                Err(GovernorError::Tracking { joint, limit, .. }) => {
                    assert_eq!(joint, ELBOW);
                    Some(limit)
                }
                Err(other) => panic!("{other}"),
            }
        })
        .expect("a blocked joint is stopped");
    assert!(
        limit < 0.21,
        "a blocked joint creeping at 0.1 rad/s stopped only at {limit} rad"
    );
}

/// A stop of a body moving fast (a tracking fault, a latched failure, an operator's stop) at
/// the share a station opens at: the chain brakes, holding each motor where it was, before it
/// disables, so the shoulder stops within `MAX_COAST` instead of coasting limp on friction
/// (without the brake, from 1.24 rad/s on 1.2 N m of friction, 0.2 rad).
#[test]
fn a_stop_at_speed_brakes_before_it_disables() {
    let mut rig = Rig::new_at(0.25);
    let shoulder = rig.index(SHOULDER);
    let mut target = rig.governor.last_observed().expect("rest").to_vec();
    let rest = target[shoulder];
    let range = rig.governor.commandable()[shoulder];
    target[shoulder] = range.upper;
    let top = rig.governor.limits()[shoulder].max_speed;
    let mut speed = 0.0;
    let mut previous = rest;
    for _ in 0..200 {
        rig.governor.command(&target).expect("moving");
        let sent = rig.governor.sent().expect("sent")[shoulder];
        speed = (sent - previous) / MAX_STEP_INTERVAL.as_secs_f64();
        previous = sent;
        rig.pass(MAX_STEP_INTERVAL);
        if speed >= top - 1e-6 {
            break;
        }
    }
    assert!(
        speed >= top - 1e-6,
        "the shoulder reached only {speed} of {top} rad/s"
    );
    let at_stop = rig.q(shoulder);
    assert!(rig.governor.stop().is_proven(), "a proven stop");
    let mut farthest: f64 = 0.0;
    for _ in 0..300 {
        rig.pass(Duration::from_millis(5));
        farthest = farthest.max(rig.q(shoulder) - at_stop);
    }
    assert!(
        farthest <= MAX_COAST + 0.02,
        "stopped at {top} rad/s, the shoulder went {farthest} rad on"
    );
}

/// A stop with one motor silent (the Spark's most common fault, and what an `Unanswered`
/// governor fault stops for) still brakes every motor that answers: the moving shoulder is held
/// over several phases and stops within `MAX_COAST`; the silent motor is reported unproven.
#[test]
fn a_stop_with_a_silent_motor_still_brakes_the_others() {
    let mut rig = Rig::new_at(0.25);
    let shoulder = rig.index(SHOULDER);
    let wrist = rig.index("right_wrist_roll_joint");
    let mut target = rig.governor.last_observed().expect("rest").to_vec();
    target[shoulder] = rig.governor.commandable()[shoulder].upper;
    for _ in 0..40 {
        rig.governor.command(&target).expect("moving");
        rig.pass(MAX_STEP_INTERVAL);
    }
    let silent = rig.governor.axes()[wrist].binding.bus_id;
    let held_id = rig.governor.axes()[shoulder].binding.bus_id;
    rig.bus.kill(silent);
    let before = rig.bus.state().frames.len();
    let at_stop = rig.q(shoulder);
    let report = rig.governor.stop();
    assert!(report.unproven.contains(&silent), "{report:?}");
    // MIT frames go to the motor's own id; the disable frames end in 0xFD.
    let holds = rig.bus.state().frames[before..]
        .iter()
        .filter(|frame| frame.id == u32::from(held_id) && frame.data[7] != 0xFD)
        .count();
    assert!(holds > 1, "the shoulder was held for {holds} phase(s)");
    let mut farthest: f64 = 0.0;
    for _ in 0..300 {
        rig.pass(Duration::from_millis(5));
        farthest = farthest.max(rig.q(shoulder) - at_stop);
    }
    assert!(
        farthest <= MAX_COAST + 0.02,
        "the shoulder went {farthest} rad on"
    );
}

/// A stop whose link fails does not brake: it disables at once.
#[test]
fn a_stop_on_a_failing_link_does_not_brake() {
    let mut rig = Rig::new_at(0.25);
    let shoulder = rig.index(SHOULDER);
    let mut target = rig.governor.last_observed().expect("rest").to_vec();
    target[shoulder] = rig.governor.commandable()[shoulder].upper;
    for _ in 0..40 {
        rig.governor.command(&target).expect("moving");
        rig.pass(MAX_STEP_INTERVAL);
    }
    let held_id = rig.governor.axes()[shoulder].binding.bus_id;
    rig.bus.fail_next(&[std::io::ErrorKind::TimedOut]);
    let before = rig.bus.state().frames.len();
    let _ = rig.governor.stop();
    let holds = rig.bus.state().frames[before..]
        .iter()
        .filter(|frame| frame.id == u32::from(held_id) && frame.data[7] != 0xFD)
        .count();
    assert!(holds <= 1, "{holds} hold frames on a failing link");
}

/// The brake keeps the gravity part of the feed-forward torque each joint was last sent: a
/// loaded joint is held up while it brakes, not dropped onto its position term.
#[test]
fn the_brake_keeps_each_joints_gravity_torque_only() {
    use sx_embodiment_drivers::{Feedforward, FeedforwardLimit};
    let mut rig = Rig::new_at(0.25);
    let shoulder = rig.index(SHOULDER);
    let width = rig.governor.axes().len();
    rig.governor
        .set_feedforward_limits(&vec![
            FeedforwardLimit {
                velocity: None,
                torque: Some(10.0),
            };
            width
        ])
        .expect("bounds");
    let mut target = rig.governor.last_observed().expect("rest").to_vec();
    target[shoulder] = rig.governor.commandable()[shoulder].upper;
    let mut ff = vec![Feedforward::default(); width];
    // 5 N m sent, 3 of it gravity: the brake keeps the 3 and drops the rest (inertia,
    // friction in the direction of motion), which would push a stopping joint on.
    ff[shoulder].torque = 5.0;
    ff[shoulder].gravity = 3.0;
    for _ in 0..40 {
        rig.governor.command_with(&target, &ff).expect("moving");
        rig.pass(MAX_STEP_INTERVAL);
    }
    let held_id = rig.governor.axes()[shoulder].binding.bus_id;
    let before = rig.bus.state().frames.len();
    let _ = rig.governor.stop();
    let state = rig.bus.state();
    let tmax = state.motors[&held_id].limits[2];
    let hold = state.frames[before..]
        .iter()
        .find(|frame| frame.id == u32::from(held_id) && frame.data[7] != 0xFD)
        .expect("a hold frame");
    let tau = (u16::from(hold.data[6] & 0x0f) << 8) | u16::from(hold.data[7]);
    let tau = f64::from(tau) / 4095.0 * 2.0 * tmax - tmax;
    assert!(
        (tau - 3.0).abs() < 0.05,
        "the hold sent {tau} N m, not the 3 N m of gravity it carried"
    );
}
