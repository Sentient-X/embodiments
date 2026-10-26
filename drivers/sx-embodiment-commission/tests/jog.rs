//! `sx-jog`'s run on the simulated static upper body, opened through the embodiment layer:
//! the governor's slewed command is what the log records beside the target, the out-of-service
//! torso keeps its place and is never addressed, and the log is written off the tick path.

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
use sx_embodiment_drivers::{ActuatorChain, AxisLimits, Clock, Governor, InstanceDrive};

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
        SPEED,
        PERIOD,
        &stop,
        Some(&log),
        &mut || clock.0.set(clock.0.get() + PERIOD),
    )
    .expect("runs");
    assert!(ended.failure.is_none(), "{:?}", ended.failure);
    assert!(ended.stop.is_proven());
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
