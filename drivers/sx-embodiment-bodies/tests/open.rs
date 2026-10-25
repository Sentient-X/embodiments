//! A body opens from its drive and its commissioning, on simulated Damiao and Feetech buses,
//! and every mismatch is refused before a byte reaches a bus.

use std::collections::BTreeMap;

use std::time::Duration;
use sx_damiao_can::b601::b601;
use sx_damiao_can::humanoid::humanoid_upper_body;
use sx_damiao_can::sim::Sim;

use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_bodies::{
    Body, BusError, Commissioning, OpenError, Port, PortSource, SerialLink, ServoCalibrations,
    open, open_calibrated, ports,
};
use sx_embodiment_drivers::lerobot::MotorCalibration;
use sx_embodiment_drivers::{ActuatorChain, BodyBindings, Evidence, InstanceDrive};
use sx_feetech_serial::FeetechChainError;
use sx_feetech_serial::sim::SimBus;

const DRIVES: &str = include_str!("../../sx-damiao-can/tests/fixtures/drives.json");

fn drive(body: &str) -> InstanceDrive {
    let drives: serde_json::Value = serde_json::from_str(DRIVES).expect("drives");
    let nominal = &drives[body]["nominal"];
    InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("instance"),
    )
    .expect("drive")
}

fn commissioning(drive: &InstanceDrive, ports: &[(&str, &str)]) -> Commissioning {
    let document = serde_json::json!({
        "schema": "sx.station-commissioning/v1",
        "instance": drive.instance,
        "pod_id": "pod-a",
        "pod_revision": 3,
        "ports": ports.iter().map(|(name, serial)| ((*name).to_owned(), (*serial).to_owned()))
            .collect::<BTreeMap<_, _>>(),
    });
    Commissioning::parse(&document.to_string(), &drive.instance).expect("commissioning")
}

/// Every neck servo's calibration on the simulated neck: mid-range 2000.
const NECK_CALIBRATION: MotorCalibration = MotorCalibration {
    drive_mode: false,
    homing_offset: -40,
    range_min: 1000,
    range_max: 3000,
};

/// `LeRobot`'s calibration file for servos `ids`, each holding [`NECK_CALIBRATION`].
fn lerobot_calibration(ids: std::ops::RangeInclusive<u16>) -> String {
    let motors: serde_json::Map<String, serde_json::Value> = ids
        .map(|id| {
            (
                format!("motor_{id}"),
                serde_json::json!({
                    "id": id, "drive_mode": 0, "homing_offset": NECK_CALIBRATION.homing_offset,
                    "range_min": NECK_CALIBRATION.range_min, "range_max": NECK_CALIBRATION.range_max,
                }),
            )
        })
        .collect();
    serde_json::Value::Object(motors).to_string()
}

fn neck_calibrations() -> ServoCalibrations {
    ServoCalibrations::new()
        .with_lerobot("neck", &lerobot_calibration(1..=6))
        .expect("parses")
}

/// The SO-101 follower's nominal drive: every joint uncalibrated.
fn so101_drive() -> InstanceDrive {
    let body = BodyBindings::parse(sx_feetech_serial::so101::BINDINGS).expect("bindings");
    let document = serde_json::json!({
        "schema": "sx.instance-drive/v1",
        "embodiment": body.id,
        "instance": "so101-unit",
        "joints": body.axes.iter().map(|axis| serde_json::json!({
            "joint": axis.joint, "zero": 0.0, "direction": 1, "calibration": null,
        })).collect::<Vec<_>>(),
    });
    InstanceDrive::parse(&document.to_string(), "so101-unit").expect("drive")
}

/// Bridges by serial, each a simulated bus; records every serial opened, a serial bus with
/// its rate.
#[derive(Default)]
struct Bridges {
    buses: BTreeMap<String, Sim>,
    serial: BTreeMap<String, SimBus>,
    opened: Vec<String>,
}

impl PortSource for Bridges {
    fn open(&mut self, serial: &str) -> Result<Port, OpenError> {
        self.opened.push(serial.to_owned());
        self.buses
            .get(serial)
            .map(|sim| Box::new(sim.clone()) as Port)
            .ok_or_else(|| OpenError::NotAttached(serial.to_owned()))
    }

    fn open_serial(&mut self, serial: &str, baud: u32) -> Result<SerialLink, OpenError> {
        self.opened.push(format!("{serial}@{baud}"));
        self.serial
            .get(serial)
            .map(|bus| Box::new(bus.clone()) as SerialLink)
            .ok_or_else(|| OpenError::NotAttached(serial.to_owned()))
    }
}

/// A source carrying only CAN bridges, every one the same simulated bus.
struct CanOnly(Sim);

impl PortSource for CanOnly {
    fn open(&mut self, _serial: &str) -> Result<Port, OpenError> {
        Ok(Box::new(self.0.clone()))
    }
}

#[test]
fn a_b601_opens_on_two_bridges_or_one_shared_bridge() {
    let drive = drive("b601-dm");
    assert_eq!(ports(&drive).expect("ports"), &["arm", "gripper"]);
    let mut two = Bridges {
        buses: BTreeMap::from([
            ("ARM".to_owned(), Sim::b601(&[1, 2, 3, 4, 5, 6])),
            ("GRIP".to_owned(), Sim::b601(&[7])),
        ]),
        ..Bridges::default()
    };
    let body = open(
        &drive,
        &commissioning(&drive, &[("arm", "ARM"), ("gripper", "GRIP")]),
        &mut two,
    )
    .expect("two bridges");
    assert!(matches!(body, Body::B601(_)));
    assert_eq!(body.axes().len(), b601(&drive).expect("axes").len());
    let mut one = Bridges {
        buses: BTreeMap::from([("BOTH".to_owned(), Sim::b601(&[1, 2, 3, 4, 5, 6, 7]))]),
        ..Bridges::default()
    };
    let mut body = open(
        &drive,
        &commissioning(&drive, &[("arm", "BOTH"), ("gripper", "BOTH")]),
        &mut one,
    )
    .expect("one shared bridge");
    // One bridge carrying both chains is opened once.
    assert_eq!(one.opened, ["BOTH"]);
    let receipt = body
        .command(&[0.0; 7])
        .expect("a command on the shared bridge");
    assert_eq!(receipt.observed.len(), 7);
    assert!(body.stop().is_proven());
}

#[test]
fn the_humanoid_upper_body_opens_on_its_one_bridge() {
    let drive = drive("sentient-humanoid-upper-body");
    assert_eq!(ports(&drive).expect("ports"), &["can"]);
    let axes = humanoid_upper_body(&drive).expect("axes");
    let mut bridges = Bridges {
        buses: BTreeMap::from([("CAN".to_owned(), Sim::for_axes(&axes))]),
        ..Bridges::default()
    };
    let mut body = open(
        &drive,
        &commissioning(&drive, &[("can", "CAN")]),
        &mut bridges,
    )
    .expect("humanoid");
    assert!(matches!(body, Body::Buses(_)));
    assert_eq!(body.axes().len(), axes.len());
    assert!(body.stop().is_proven());
}

#[test]
fn the_static_upper_body_opens_across_its_can_bus_and_feetech_neck_and_stops_both() {
    let drive = drive("sentient-static-upper-body");
    assert_eq!(ports(&drive).expect("ports"), &["can", "neck"]);
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    assert_eq!(axes.len(), 15);
    let can = Sim::for_axes(&axes);
    let neck = SimBus::sts3215(&(1..=6).map(|id| (id, NECK_CALIBRATION)).collect::<Vec<_>>());
    let mut bridges = Bridges {
        buses: BTreeMap::from([("CAN".to_owned(), can.clone())]),
        serial: BTreeMap::from([("NECK".to_owned(), neck.clone())]),
        opened: Vec::new(),
    };
    let mut body = open_calibrated(
        &drive,
        &commissioning(&drive, &[("can", "CAN"), ("neck", "NECK")]),
        &neck_calibrations(),
        &mut bridges,
    )
    .expect("static upper body");
    assert!(matches!(body, Body::Buses(_)));
    assert_eq!(bridges.opened, ["CAN", "NECK@1000000"]);
    // The torso and arms, then the neck, as the native state order has them.
    let joints: Vec<&str> = body.axes().iter().map(|axis| axis.joint.as_str()).collect();
    assert_eq!(joints[0], "torso_joint");
    assert_eq!(
        joints[15..],
        [
            "so101_shoulder_pan_joint",
            "so101_shoulder_lift_joint",
            "so101_elbow_flex_joint",
            "so101_wrist_flex_joint",
            "so101_wrist_roll_joint",
            "so101_camera_tilt_joint",
        ]
    );
    // The Damiao bus's 248 ms, then three 10 ms exchanges for each of six neck servos.
    assert_eq!(body.stop_budget(), Duration::from_millis(248 + 180));
    let receipt = body
        .command(&[0.0; 21])
        .expect("a command at the zero pose");
    // The neck's SYNC WRITE has no answer, so no partial joint vector is reported.
    assert_eq!(receipt.evidence, Evidence::DispatchAttempted);
    assert!(receipt.observed.is_empty());
    assert!(can.state().motors.values().any(|motor| motor.state == 1));
    assert!((1..=6).all(|id| neck.state().torque(id) == 1));
    // Mid-range of 1000..3000 at every neck joint's zero.
    assert!((1..=6).all(|id| neck.state().goal(id) == 2000));
    let stop = body.stop();
    assert!(stop.is_proven(), "{stop:?}");
    assert!(can.state().motors.values().all(|motor| motor.state == 0));
    assert!((1..=6).all(|id| neck.state().torque(id) == 0));
    // A neck servo that stops answering leaves the whole stop unproven, and the CAN bus is
    // still stopped.
    body.command(&[0.0; 21]).expect("re-enabled");
    neck.kill(4);
    let stop = body.stop();
    assert_eq!(stop.unproven, [4]);
    assert!(can.state().motors.values().all(|motor| motor.state == 0));
    // A target outside a neck joint's bounds reaches neither bus.
    let sent = can.sent();
    let written = neck.state().written.len();
    let mut far = [0.0; 21];
    far[20] = 3.0;
    assert!(body.command(&far).is_err());
    assert_eq!((can.sent(), neck.state().written.len()), (sent, written));
}

#[test]
fn the_neck_is_refused_without_its_calibration_or_on_a_shared_bridge() {
    let drive = drive("sentient-static-upper-body");
    let mut bridges = Bridges::default();
    let both = commissioning(&drive, &[("can", "CAN"), ("neck", "NECK")]);
    // No calibration: refused before any bridge is opened.
    assert!(matches!(
        open(&drive, &both, &mut bridges),
        Err(OpenError::Uncalibrated { port, bus_id: 1 }) if port == "neck"
    ));
    // A calibration file missing one servo.
    let five = ServoCalibrations::new()
        .with_lerobot("neck", &lerobot_calibration(1..=5))
        .expect("parses");
    assert!(matches!(
        open_calibrated(&drive, &both, &five, &mut bridges),
        Err(OpenError::Uncalibrated { bus_id: 6, .. })
    ));
    // The neck commissioned on the CAN bridge, and the neck not commissioned at all.
    assert!(matches!(
        open_calibrated(
            &drive,
            &commissioning(&drive, &[("can", "CAN"), ("neck", "CAN")]),
            &neck_calibrations(),
            &mut bridges
        ),
        Err(OpenError::SharedBridge(serial)) if serial == "CAN"
    ));
    assert!(matches!(
        open_calibrated(
            &drive,
            &commissioning(&drive, &[("can", "CAN")]),
            &neck_calibrations(),
            &mut bridges
        ),
        Err(OpenError::MissingPort(port)) if port == "neck"
    ));
    assert!(
        bridges.opened.is_empty(),
        "nothing opened before the refusals"
    );
    // A source that carries no serial bus refuses the neck after opening the CAN bridge.
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    assert!(matches!(
        open_calibrated(&drive, &both, &neck_calibrations(), &mut CanOnly(Sim::for_axes(&axes))),
        Err(OpenError::Port { serial, .. }) if serial == "NECK"
    ));
    // A neck servo whose registers disagree with the calibration supplied.
    let neck = SimBus::sts3215(
        &(1..=6)
            .map(|id| {
                let mut calibration = NECK_CALIBRATION;
                calibration.range_max -= u16::from(id == 3);
                (id, calibration)
            })
            .collect::<Vec<_>>(),
    );
    let mut bridges = Bridges {
        buses: BTreeMap::from([("CAN".to_owned(), Sim::for_axes(&axes))]),
        serial: BTreeMap::from([("NECK".to_owned(), neck)]),
        opened: Vec::new(),
    };
    assert!(matches!(
        open_calibrated(&drive, &both, &neck_calibrations(), &mut bridges),
        Err(OpenError::Chain(BusError::Feetech(
            FeetechChainError::Uncalibrated { bus_id: 3, .. }
        )))
    ));
    // A malformed LeRobot calibration.
    for document in [
        r#"{"a": {"id": 1, "drive_mode": 2, "homing_offset": 0, "range_min": 1, "range_max": 2}}"#,
        r#"{"a": {"id": 1, "drive_mode": 0, "homing_offset": 0, "range_min": 1, "range_max": 2},
            "b": {"id": 1, "drive_mode": 0, "homing_offset": 0, "range_min": 1, "range_max": 2}}"#,
        r#"{"a": {"id": 1, "drive_mode": 0, "homing_offset": 0, "range_min": 1}}"#,
    ] {
        assert!(matches!(
            ServoCalibrations::new().with_lerobot("neck", document),
            Err(OpenError::Document(_))
        ));
    }
}

#[test]
fn the_so101_follower_opens_on_its_one_serial_bus() {
    let drive = so101_drive();
    assert_eq!(ports(&drive).expect("ports"), &["serial"]);
    let calibration = ServoCalibrations::new()
        .with_lerobot("serial", &lerobot_calibration(1..=6))
        .expect("parses");
    let bus = SimBus::sts3215(&(1..=6).map(|id| (id, NECK_CALIBRATION)).collect::<Vec<_>>());
    let mut bridges = Bridges {
        serial: BTreeMap::from([("ARM".to_owned(), bus.clone())]),
        ..Bridges::default()
    };
    let mut body = open_calibrated(
        &drive,
        &commissioning(&drive, &[("serial", "ARM")]),
        &calibration,
        &mut bridges,
    )
    .expect("so101");
    assert_eq!(body.axes().len(), 6);
    body.command(&[0.0; 6]).expect("commanded");
    assert!(body.stop().is_proven());
    assert!((1..=6).all(|id| bus.state().torque(id) == 0));
}

#[test]
fn every_mismatch_is_refused_before_a_bus_is_opened() {
    let drive = drive("b601-dm");
    let mut bridges = Bridges::default();
    // Commissioning for another instance.
    let other = serde_json::json!({
        "schema": "sx.station-commissioning/v1", "instance": "another-instance",
        "pod_id": "pod-a", "pod_revision": 1, "ports": {"arm": "A", "gripper": "G"},
    })
    .to_string();
    assert!(matches!(
        Commissioning::parse(&other, &drive.instance),
        Err(OpenError::Instance { .. })
    ));
    let mut foreign = commissioning(&drive, &[("arm", "A"), ("gripper", "G")]);
    foreign.instance = "another-instance".to_owned();
    assert!(matches!(
        open(&drive, &foreign, &mut bridges),
        Err(OpenError::Instance { .. })
    ));
    // A port the opener does not declare, and a declared port commissioning does not name.
    assert!(matches!(
        open(
            &drive,
            &commissioning(&drive, &[("arm", "A"), ("gripper", "G"), ("neck", "N")]),
            &mut bridges
        ),
        Err(OpenError::UndeclaredPort(port)) if port == "neck"
    ));
    assert!(matches!(
        open(&drive, &commissioning(&drive, &[("arm", "A")]), &mut bridges),
        Err(OpenError::MissingPort(port)) if port == "gripper"
    ));
    assert!(
        bridges.opened.is_empty(),
        "nothing opened before the refusals"
    );
    // A bridge that is not attached.
    assert!(matches!(
        open(
            &drive,
            &commissioning(&drive, &[("arm", "A"), ("gripper", "G")]),
            &mut bridges
        ),
        Err(OpenError::NotAttached(serial)) if serial == "A"
    ));
    // A drive of this body whose joints are not the ones its opener composes.
    let mut missing = drive.clone();
    missing.joints.pop();
    assert!(matches!(
        open(
            &missing,
            &commissioning(&drive, &[("arm", "A"), ("gripper", "G")]),
            &mut bridges
        ),
        Err(OpenError::Bindings(_))
    ));
    assert_eq!(
        bridges.opened,
        ["A"],
        "only the absent bridge above was asked for"
    );
    // A drive for a body no opener composes.
    let mut unknown = drive.clone();
    unknown.embodiment = "0".repeat(64);
    assert!(matches!(ports(&unknown), Err(OpenError::Body(_))));
    // A malformed document.
    for document in [
        r#"{"schema": "sx.station-commissioning/v2", "instance": "x", "pod_id": "p", "pod_revision": 1, "ports": {}}"#,
        r#"{"schema": "sx.station-commissioning/v1", "instance": "x", "pod_id": "", "pod_revision": 1, "ports": {}}"#,
        r#"{"schema": "sx.station-commissioning/v1", "instance": "x", "pod_id": "p", "pod_revision": 0, "ports": {}}"#,
        r#"{"schema": "sx.station-commissioning/v1", "instance": "x", "pod_id": "p", "pod_revision": 1, "ports": {"arm": "a b"}}"#,
        r#"{"schema": "sx.station-commissioning/v1", "instance": "x", "pod_id": "p", "pod_revision": 1, "ports": {}, "extra": 1}"#,
    ] {
        assert!(matches!(
            Commissioning::parse(document, "x"),
            Err(OpenError::Document(_))
        ));
    }
}

#[test]
fn an_open_that_fails_part_way_leaves_every_motor_it_reached_disabled() {
    let drive = drive("sentient-humanoid-upper-body");
    let axes = humanoid_upper_body(&drive).expect("axes");
    let bus = Sim::for_axes(&axes);
    // The first motor was left enabled by a crashed run; the second does not answer.
    let first = axes[0].axis.binding.bus_id;
    bus.state().motors.get_mut(&first).expect("first").state = 1;
    bus.kill(axes[1].axis.binding.bus_id);
    let mut bridges = Bridges {
        buses: BTreeMap::from([("CAN".to_owned(), bus.clone())]),
        ..Bridges::default()
    };
    assert!(matches!(
        open(
            &drive,
            &commissioning(&drive, &[("can", "CAN")]),
            &mut bridges
        ),
        Err(OpenError::Chain(_))
    ));
    assert_eq!(
        bus.state().motors[&first].state,
        0,
        "the motor it reached is disabled"
    );
}
