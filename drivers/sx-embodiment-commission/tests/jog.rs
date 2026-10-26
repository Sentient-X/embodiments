//! `sx-jog`'s run on the simulated static upper body, opened through the embodiment layer:
//! the governor's slewed command is what the log records beside the target, the out-of-service
//! torso keeps its place and is never addressed, the log is written off the tick path, and a
//! finished run returns the body home before it stops it.

use std::cell::Cell;
use std::io::{self, Write};
use std::rc::Rc;
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::Value;
use sx_damiao_can::sim::Sim;
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_bodies::{Body, Commissioning, OpenError, Port, PortSource, open};
use sx_embodiment_commission::jog::{self, Step, TickLog};
use sx_embodiment_drivers::can::CanFrame;
use sx_embodiment_drivers::{
    ActuatorChain, AtEnd, AxisLimits, Clock, Ending, Governor, InstanceDrive, Progress, Return,
    ReturnFailure,
};

const DRIVES: &str = include_str!("../../sx-damiao-can/tests/fixtures/drives.json");
const PERIOD: Duration = Duration::from_millis(20);
const SPEED: f64 = 0.25;

struct Can(Sim);

impl PortSource for Can {
    fn open(&mut self, _serial: &str) -> Result<Port, OpenError> {
        Ok(Box::new(self.0.clone()))
    }
}

#[derive(Clone)]
struct Manual(Rc<Cell<Instant>>);

impl Clock for Manual {
    fn now(&self) -> Instant {
        self.0.get()
    }
}

/// A sink the test reads after the writer thread is done.
#[derive(Clone, Default)]
struct Sink(Arc<Mutex<Vec<u8>>>);

impl Write for Sink {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.lock().expect("sink").extend_from_slice(buf);
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

fn drive() -> InstanceDrive {
    let drives: Value = serde_json::from_str(DRIVES).expect("drives");
    let nominal = &drives["sentient-static-upper-body"]["nominal"];
    InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("instance"),
    )
    .expect("drive")
}

/// The static upper body on `bus`, its torso out of service, governed at [`SPEED`] on a
/// clock the test moves, with its joint names.
fn governed(drive: &InstanceDrive, bus: &Sim) -> (Governor<Body, Manual>, Manual, Vec<String>) {
    let document = serde_json::json!({
        "schema": "sx.station-commissioning/v1",
        "instance": drive.instance,
        "pod_id": "bench",
        "pod_revision": 1,
        "ports": {"can": "CAN"},
        "out_of_service": ["torso_joint"],
    });
    let commissioning =
        Commissioning::parse(&document.to_string(), &drive.instance).expect("commissioning");
    let body = open(drive, &commissioning, &mut Can(bus.clone())).expect("opens");
    let joints: Vec<String> = body.axes().iter().map(|axis| axis.joint.clone()).collect();
    let clock = Manual(Rc::new(Cell::new(Instant::now())));
    let governor = Governor::with_clock(
        body,
        vec![
            AxisLimits {
                max_speed: SPEED,
                max_tracking_error: 0.2,
                max_acceleration: None,
            };
            joints.len()
        ],
        clock.clone(),
    )
    .expect("limits");
    (governor, clock, joints)
}

#[test]
fn a_jog_logs_the_slewed_command_and_keeps_the_out_of_service_torso_in_its_place() {
    let drive = drive();
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    let torso_id = 0x1F;
    let bus = Sim::for_axes(&axes);
    bus.kill(torso_id);
    let (mut governor, clock, joints) = governed(&drive, &bus);
    let torso = joints
        .iter()
        .position(|joint| joint == "torso_joint")
        .expect("the torso keeps its place");
    let steps =
        jog::parse_sequence("right_elbow_pitch_joint:10,pause:0.1", &joints).expect("a sequence");
    let elbow = joints
        .iter()
        .position(|joint| joint == "right_elbow_pitch_joint")
        .expect("the elbow");
    assert!(matches!(steps[0], Step::Move { axis, .. } if axis == elbow));
    let sink = Sink::default();
    let log = TickLog::spawn(Box::new(sink.clone()), joints.clone(), 1000);
    let stop = AtomicBool::new(false);
    let sent = bus.sent();
    let ended = jog::run(
        &mut governor,
        &steps,
        PERIOD,
        &stop,
        Some(&log),
        AtEnd::Release,
        &mut || clock.0.set(clock.0.get() + PERIOD),
    )
    .expect("runs");
    assert!(ended.failure.is_none(), "{:?}", ended.failure);
    assert!(matches!(ended.ending, Ending::Released { .. }));
    assert!(ended.ending.stop().is_proven());
    let (written, dropped) = log.finish().expect("log");
    assert_eq!(dropped, 0);
    // 10 degrees at 0.25 rad/s is 0.7 s, plus 0.4 s settling and the 0.1 s pause: 60 ticks.
    assert_eq!(written, 60);
    let addressed = |frame: &CanFrame| {
        frame.id == u32::from(torso_id)
            || (frame.id == 0x7FF && u16::from_le_bytes([frame.data[0], frame.data[1]]) == torso_id)
    };
    assert!(!bus.state().frames[sent..].iter().any(addressed));
    let text = String::from_utf8(sink.0.lock().expect("sink").clone()).expect("utf-8");
    let lines: Vec<Value> = text
        .lines()
        .map(|line| serde_json::from_str(line).expect("a JSON line"))
        .collect();
    assert_eq!(lines.len(), 60);
    let elbow_name = "right_elbow_pitch_joint";
    let first = &lines[0];
    let requested = first["requested"][elbow_name].as_f64().expect("requested");
    let commanded = first["commanded"][elbow_name].as_f64().expect("commanded");
    let rest = lines[0]["observed"][elbow_name].as_f64().expect("observed");
    // The target is the whole 10 degrees away; the command went one 20 ms step of it.
    assert!(
        (requested - commanded).abs() > 0.1,
        "{requested} {commanded}"
    );
    assert!((commanded - (requested - 10_f64.to_radians())).abs() <= SPEED * 0.02 + 1e-9);
    assert!((rest - commanded).abs() < 1e-2);
    // Every step of the command is within the speed bound; it reaches the target.
    let commands: Vec<f64> = lines
        .iter()
        .map(|line| line["commanded"][elbow_name].as_f64().expect("commanded"))
        .collect();
    assert!(
        commands
            .windows(2)
            .all(|pair| (pair[1] - pair[0]).abs() <= SPEED * 0.02 + 1e-9)
    );
    assert!((commands[59] - requested).abs() < 1e-9);
    // The torso is in every record, with nothing commanded, observed or answered for it.
    for line in &lines {
        assert!(line["observed"]["torso_joint"].is_null());
        assert_eq!(line["motors"].as_array().expect("motors").len(), 15);
        assert_eq!(line["motors"][torso]["state"], "unavailable");
        assert_eq!(line["motors"][torso]["bus_id"], torso_id);
        assert!(line["motors"][torso]["q"].is_null());
        assert_eq!(line["outcome"], "commanded");
    }
    assert!(lines[0]["motors"][elbow]["tau"].is_number());
}

#[test]
fn an_ampersand_step_moves_with_the_next() {
    let joints = vec!["a".to_owned(), "b".to_owned()];
    let steps = jog::parse_sequence("&a:10,b:-20,pause:1", &joints).expect("parses");
    assert!(
        matches!(steps[0], Step::With { axis: 0, delta } if (delta - 10f64.to_radians()).abs() < 1e-12)
    );
    assert!(matches!(steps[1], Step::Move { axis: 1, .. }));
    assert!(matches!(steps[2], Step::Pause(_)));
    assert!(jog::parse_sequence("&pause:1", &joints).is_err());
}

#[test]
fn a_finished_jog_returns_home_then_stops_and_never_addresses_the_torso() {
    let drive = drive();
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    let torso_id = 0x1F;
    let bus = Sim::for_axes(&axes);
    bus.kill(torso_id);
    let (mut governor, clock, joints) = governed(&drive, &bus);
    let elbow = joints
        .iter()
        .position(|joint| joint == "right_elbow_pitch_joint")
        .expect("the elbow");
    let steps = jog::parse_sequence("right_elbow_pitch_joint:10", &joints).expect("a sequence");
    let stop = AtomicBool::new(false);
    let sent = bus.sent();
    let ended = jog::run(
        &mut governor,
        &steps,
        PERIOD,
        &stop,
        None,
        AtEnd::default(),
        &mut || clock.0.set(clock.0.get() + PERIOD),
    )
    .expect("runs");
    assert!(ended.failure.is_none(), "{:?}", ended.failure);
    let Ending::Returned { to, stop, .. } = &ended.ending else {
        panic!("expected a return home, got {}", ended.ending);
    };
    assert_eq!(*to, AtEnd::Home);
    assert!(stop.is_proven(), "{stop:?}");
    let home = governor.axes()[elbow]
        .home
        .expect("the elbow declares a home");
    let readings = governor.read_positions();
    let at = readings[elbow].position().expect("the elbow answers");
    assert!((at - home).abs() < 0.05, "elbow at {at}, home {home}");
    let addressed = |frame: &CanFrame| {
        frame.id == u32::from(torso_id)
            || (frame.id == 0x7FF && u16::from_le_bytes([frame.data[0], frame.data[1]]) == torso_id)
    };
    assert!(!bus.state().frames[sent..].iter().any(addressed));
}

#[test]
fn a_jog_whose_step_is_refused_stops_where_it_is_and_never_returns() {
    let drive = drive();
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    let bus = Sim::for_axes(&axes);
    bus.kill(0x1F);
    let (mut governor, clock, joints) = governed(&drive, &bus);
    // The second move asks for the elbow far outside its range: refused before the bus.
    let steps = jog::parse_sequence(
        "right_elbow_pitch_joint:10,right_elbow_pitch_joint:-60",
        &joints,
    )
    .expect("a sequence");
    let stop = AtomicBool::new(false);
    let ended = jog::run(
        &mut governor,
        &steps,
        PERIOD,
        &stop,
        None,
        AtEnd::Start,
        &mut || clock.0.set(clock.0.get() + PERIOD),
    )
    .expect("runs");
    assert!(ended.failure.is_some(), "the step was refused");
    assert!(
        matches!(ended.ending, Ending::Released { .. }),
        "a refused run must not return: {}",
        ended.ending
    );
}

/// The static upper body jogged away from home, as an operation leaves it.
fn jogged(bus: &Sim, drive: &InstanceDrive) -> (Governor<Body, Manual>, Manual, usize) {
    let (mut governor, clock, joints) = governed(drive, bus);
    let elbow = joints
        .iter()
        .position(|joint| joint == "right_elbow_pitch_joint")
        .expect("the elbow");
    let steps = jog::parse_sequence("right_elbow_pitch_joint:10", &joints).expect("a sequence");
    let stop = AtomicBool::new(false);
    // Release here only to get a driven body: re-drive it where it rests below.
    let _ = jog::run(
        &mut governor,
        &steps,
        PERIOD,
        &stop,
        None,
        AtEnd::Release,
        &mut || clock.0.set(clock.0.get() + PERIOD),
    )
    .expect("runs");
    let rest = governor.last_observed().expect("rest");
    governor.command(&rest).expect("driven where it rests");
    (governor, clock, elbow)
}

#[test]
fn a_disengaged_body_returns_home_and_is_held_there_enabled() {
    let drive = drive();
    let bus = Sim::for_axes(&STATIC_UPPER_BODY.axes(&drive).expect("axes"));
    bus.kill(0x1F);
    let (mut governor, clock, elbow) = jogged(&bus, &drive);
    let mut ret = Return::begin(&mut governor, AtEnd::Home, &[]).expect("begins");
    let mut gravity = |_: &[f64], out: &mut [sx_embodiment_drivers::Feedforward]| {
        for out in out.iter_mut() {
            out.torque = 0.5;
        }
    };
    let mut ticks = 0;
    while ret
        .tick(&mut governor, Some(&mut gravity))
        .expect("returns")
        == Progress::Moving
    {
        clock.0.set(clock.0.get() + PERIOD);
        ticks += 1;
        assert!(ticks < 10_000, "never settled");
    }
    for _ in 0..500 {
        clock.0.set(clock.0.get() + PERIOD);
        ret.hold(&mut governor, Some(&mut gravity)).expect("held");
    }
    // Still enabled (a stop clears what was sent), at home.
    assert!(governor.sent().is_some(), "never stopped while held");
    let home = governor.axes()[elbow].home.expect("a home");
    let at = governor.last_observed().expect("observed")[elbow];
    assert!((at - home).abs() < 0.05, "elbow at {at}, home {home}");
}

#[test]
fn a_motor_lost_during_the_return_is_a_governor_fault_for_the_plain_stop() {
    let drive = drive();
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    let bus = Sim::for_axes(&axes);
    bus.kill(0x1F);
    let (mut governor, clock, elbow) = jogged(&bus, &drive);
    let mut ret = Return::begin(&mut governor, AtEnd::Home, &[]).expect("begins");
    for _ in 0..5 {
        assert_eq!(ret.tick(&mut governor, None), Ok(Progress::Moving));
        clock.0.set(clock.0.get() + PERIOD);
    }
    bus.kill(governor.axes()[elbow].binding.bus_id);
    let failure = loop {
        match ret.tick(&mut governor, None) {
            Ok(_) => clock.0.set(clock.0.get() + PERIOD),
            Err(failure) => break failure,
        }
    };
    assert!(
        matches!(failure, ReturnFailure::Governor { .. }),
        "{failure}"
    );
    // The governor stopped the body and latched: nothing moves it again.
    assert!(governor.is_latched());
    assert!(governor.sent().is_none());
    assert!(matches!(
        Return::begin(&mut governor, AtEnd::Home, &[]),
        Err(ReturnFailure::Latched)
    ));
}
