//! `sx-record`'s recording on the simulated static upper body, observed through the embodiment
//! layer: every poll is a JSON line in joint coordinates, the out-of-service torso is in each
//! as unavailable, and nothing but a read ever reaches the bus.

use std::io::{self, Write};
use std::sync::atomic::AtomicBool;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde_json::Value;
use sx_damiao_can::sim::Sim;
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_bodies::{
    Commissioning, OpenError, OpenOptions, Port, PortSource, ServoCalibrations, observe,
};
use sx_embodiment_commission::record::record;
use sx_embodiment_drivers::InstanceDrive;

struct Can(Sim);

impl PortSource for Can {
    fn open(&mut self, _serial: &str) -> Result<Port, OpenError> {
        Ok(Box::new(self.0.clone()))
    }
}

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

#[test]
fn a_recording_reads_every_joint_through_the_layer_and_writes_nothing_to_the_bus() {
    let drives: Value = serde_json::from_str(include_str!(
        "../../sx-damiao-can/tests/fixtures/drives.json"
    ))
    .expect("drives");
    let nominal = &drives["sentient-static-upper-body"]["nominal"];
    let drive = InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("instance"),
    )
    .expect("drive");
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    let bus = Sim::for_axes(&axes);
    bus.kill(0x1F);
    let elbow = axes
        .iter()
        .find(|axis| axis.axis.joint == "right_elbow_pitch_joint")
        .expect("the elbow");
    bus.state()
        .motors
        .get_mut(&elbow.axis.binding.bus_id)
        .expect("elbow")
        .q = elbow.axis.actuator_from_joint(1.25);
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
    let observer = observe(
        &drive,
        &commissioning,
        &ServoCalibrations::new(),
        OpenOptions::default(),
        &mut Can(bus.clone()),
    )
    .expect("observes");
    let sink = Sink::default();
    let recorded = record(
        observer.into_buses(),
        Box::new(sink.clone()),
        Duration::from_millis(10),
        Duration::from_millis(200),
        &AtomicBool::new(false),
    )
    .expect("records");
    assert_eq!(recorded.dropped, 0);
    assert!(recorded.lines >= 5, "{recorded:?}");
    let text = String::from_utf8(sink.0.lock().expect("sink").clone()).expect("utf-8");
    let lines: Vec<Value> = text
        .lines()
        .map(|line| serde_json::from_str(line).expect("a JSON line"))
        .collect();
    assert_eq!(u64::try_from(lines.len()).expect("count"), recorded.lines);
    for line in &lines {
        assert_eq!(line["bus"], "can");
        let positions = line["positions"].as_object().expect("positions");
        assert_eq!(positions.len(), 15, "every axis, the torso in its place");
        assert!(positions["torso_joint"].is_null());
        assert_eq!(line["missing"]["torso_joint"], "out_of_service");
        let at = positions["right_elbow_pitch_joint"]
            .as_f64()
            .expect("the elbow");
        assert!((at - 1.25).abs() < 1e-3, "{at}");
        assert_eq!(line["missing"].as_object().expect("missing").len(), 1);
    }
    assert!(
        bus.state()
            .frames
            .iter()
            .all(sx_damiao_can::observe::only_asks),
        "only register reads and status refreshes reached the bus"
    );
}
