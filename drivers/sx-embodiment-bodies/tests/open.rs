//! A body opens from its drive and its commissioning, on simulated Damiao and Feetech buses,
//! and every mismatch is refused before a byte reaches a bus.

use std::collections::BTreeMap;

use std::time::Duration;
use sx_damiao_can::b601::b601;
use sx_damiao_can::humanoid::humanoid_upper_body;
use sx_damiao_can::sim::Sim;

use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_bodies::{
    Body, BusError, Commissioning, OpenError, OpenOptions, Port, PortSource, SerialLink,
    ServoCalibrations, closed_ports, open, open_calibrated, open_with, ports,
};
use sx_embodiment_drivers::lerobot::MotorCalibration;
use sx_embodiment_drivers::{
    ActuatorChain, AxisLimits, BodyBindings, Evidence, Governor, GovernorError, InstanceDrive,
};
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

/// Every SO-101 servo's calibration on its simulated bus: mid-range 2000.
const SO101_CALIBRATION: MotorCalibration = MotorCalibration {
    drive_mode: false,
    homing_offset: -40,
    range_min: 1000,
    range_max: 3000,
};

/// `LeRobot`'s calibration file for servos `ids`, each holding [`SO101_CALIBRATION`].
fn lerobot_calibration(ids: std::ops::RangeInclusive<u16>) -> String {
    let motors: serde_json::Map<String, serde_json::Value> = ids
        .map(|id| {
            (
                format!("motor_{id}"),
                serde_json::json!({
                    "id": id, "drive_mode": 0, "homing_offset": SO101_CALIBRATION.homing_offset,
                    "range_min": SO101_CALIBRATION.range_min, "range_max": SO101_CALIBRATION.range_max,
                }),
            )
        })
        .collect();
    serde_json::Value::Object(motors).to_string()
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

/// The static upper body's provisional ports, opened for a supervised bring-up.
const BRING_UP: OpenOptions = OpenOptions {
    allow_provisional: &["left_gripper", "right_gripper", "neck"],
};

/// Open with every provisional port opted in, as a supervised bring-up does.
fn bring_up(
    drive: &InstanceDrive,
    commissioning: &Commissioning,
    calibrations: &ServoCalibrations,
    source: &mut dyn PortSource,
) -> Result<Body, OpenError> {
    open_with(drive, commissioning, calibrations, BRING_UP, source)
}

/// The static upper body's Feetech ports, by the bridge serial commissioning names each.
const FEETECH_PORTS: [(&str, &str); 3] = [
    ("left_gripper", "5AB0158173"),
    ("right_gripper", "5B91028463"),
    ("neck", "5A7C118709"),
];

/// What the read-only probe of the robot found on its Feetech buses.
fn probe() -> serde_json::Value {
    serde_json::from_str(include_str!("fixtures/static-upper-body-probe.json")).expect("probe")
}

/// The probe's calibration registers, as `LeRobot` calibration files per port.
fn probed_calibrations() -> ServoCalibrations {
    let probe = probe();
    FEETECH_PORTS
        .iter()
        .fold(ServoCalibrations::new(), |calibrations, (port, _)| {
            calibrations
                .with_lerobot(port, &probe["calibration"][port].to_string())
                .expect("a LeRobot calibration")
        })
}

/// A simulated bus per Feetech port, holding the probe's registers and resting where the
/// probe found each servo.
fn probed_buses() -> BTreeMap<String, SimBus> {
    let probe = probe();
    FEETECH_PORTS
        .iter()
        .map(|(port, serial)| {
            let servos: Vec<(u8, MotorCalibration)> = probe["calibration"][port]
                .as_object()
                .expect("motors")
                .values()
                .map(|motor| {
                    let field = |name: &str| motor[name].as_i64().expect(name);
                    (
                        u8::try_from(field("id")).expect("id"),
                        MotorCalibration {
                            drive_mode: field("drive_mode") == 1,
                            homing_offset: i16::try_from(field("homing_offset")).expect("offset"),
                            range_min: u16::try_from(field("range_min")).expect("min"),
                            range_max: u16::try_from(field("range_max")).expect("max"),
                        },
                    )
                })
                .collect();
            let bus = SimBus::sts3215(&servos);
            for (id, raw) in probe["present_position"][port].as_object().expect("rest") {
                bus.rest_at(
                    id.parse().expect("an id"),
                    u16::try_from(raw.as_u64().expect("ticks")).expect("ticks"),
                );
            }
            ((*serial).to_owned(), bus)
        })
        .collect()
}

fn static_commissioning(drive: &InstanceDrive) -> Commissioning {
    let mut ports = vec![("can", "CAN")];
    ports.extend(FEETECH_PORTS);
    commissioning(drive, &ports)
}

#[test]
fn the_static_upper_body_opens_across_its_can_bus_and_three_feetech_buses_and_stops_all() {
    let drive = drive("sentient-static-upper-body");
    assert_eq!(
        ports(&drive).expect("ports"),
        &["can", "left_gripper", "right_gripper", "neck"]
    );
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    assert_eq!(axes.len(), 15);
    let can = Sim::for_axes(&axes);
    let serial = probed_buses();
    let mut bridges = Bridges {
        buses: BTreeMap::from([("CAN".to_owned(), can.clone())]),
        serial: serial.clone(),
        opened: Vec::new(),
    };
    let mut body = bring_up(
        &drive,
        &static_commissioning(&drive),
        &probed_calibrations(),
        &mut bridges,
    )
    .expect("static upper body");
    assert!(matches!(body, Body::Buses(_)));
    assert_eq!(
        bridges.opened,
        [
            "CAN",
            "5AB0158173@1000000",
            "5B91028463@1000000",
            "5A7C118709@1000000"
        ]
    );
    // The torso and arms, then each gripper's two fingers, then the neck, as the native state
    // order has them.
    let joints: Vec<&str> = body.axes().iter().map(|axis| axis.joint.as_str()).collect();
    assert_eq!(joints.len(), 25);
    assert_eq!(joints[0], "torso_joint");
    assert_eq!(
        joints[15..21],
        [
            "left_gripper_left_finger_joint",
            "left_gripper_right_finger_joint",
            "right_gripper_left_finger_joint",
            "right_gripper_right_finger_joint",
            "so101_shoulder_pan_joint",
            "so101_shoulder_lift_joint",
        ]
    );
    // The Damiao bus's own budget, then four 10 ms exchanges for each of ten servos.
    let damiao = sx_damiao_can::static_upper_body::STATIC_UPPER_BODY
        .open(Sim::for_axes(&axes), &drive)
        .expect("a lone Damiao chain")
        .stop_budget();
    assert_eq!(body.stop_budget(), damiao + Duration::from_millis(400));
    // Where every axis rests, in state order: the neck's camera tilt (id 6), found ~1000 ticks
    // below its calibrated minimum, is reported there, far outside what its unit reaches.
    let rest = body.last_observed().expect("every bus observed at open");
    assert_eq!(rest.len(), 25);
    let tilt = rest[24];
    assert!(tilt < body.commandable()[24].lower - 1.0, "{tilt}");
    let receipt = body.command(&rest_inside(&body)).expect("a command");
    // The Feetech SYNC WRITEs have no answer, so the evidence is a dispatch; every bus still
    // observes every axis, the Feetech ones through their SYNC READs.
    assert_eq!(receipt.evidence, Evidence::DispatchAttempted);
    assert_eq!(receipt.observed.len(), 25);
    assert!(receipt.observed.iter().all(|joint| joint.is_finite()));
    assert!(
        (receipt.observed[24] - tilt).abs() < 1e-9,
        "the tilt has not moved yet"
    );
    assert_eq!(
        body.last_observed(),
        None,
        "a command may have moved the servos"
    );
    assert!(can.state().motors.values().any(|motor| motor.state == 1));
    assert!(torques(&serial, 1));
    let stop = body.stop();
    assert!(stop.is_proven(), "{stop:?}");
    assert!(can.state().motors.values().all(|motor| motor.state == 0));
    assert!(torques(&serial, 0));
    assert!(body.last_observed().is_some(), "read again after the stop");
    // A neck servo that stops answering leaves the whole stop unproven; every bus still stops.
    body.command(&rest_inside(&body)).expect("re-enabled");
    serial["5A7C118709"].kill(4);
    let stop = body.stop();
    assert_eq!(stop.unproven, [4]);
    assert!(can.state().motors.values().all(|motor| motor.state == 0));
    assert_eq!(body.last_observed(), None, "the neck could not be read");
    // A target outside a neck joint's bounds reaches no bus.
    let sent = can.sent();
    let written: Vec<usize> = serial
        .values()
        .map(|bus| bus.state().written.len())
        .collect();
    let mut far = rest_inside(&body);
    far[24] = 3.0;
    assert!(body.command(&far).is_err());
    assert_eq!(can.sent(), sent);
    assert_eq!(
        serial
            .values()
            .map(|bus| bus.state().written.len())
            .collect::<Vec<_>>(),
        written
    );
}

/// Whether every servo on every simulated bus has `Torque_Enable` at `value`.
fn torques(serial: &BTreeMap<String, SimBus>, value: u8) -> bool {
    serial.values().all(|bus| {
        let state = bus.state();
        state.servos.keys().all(|id| state.torque(*id) == value)
    })
}

/// Each axis's commandable midpoint: a target every bus admits.
fn rest_inside(body: &Body) -> Vec<f64> {
    body.commandable()
        .iter()
        .map(|range| range.lower.midpoint(range.upper))
        .collect()
}

#[test]
fn the_feetech_buses_are_refused_without_their_calibration_or_on_a_shared_bridge() {
    let drive = drive("sentient-static-upper-body");
    let mut bridges = Bridges::default();
    let every = static_commissioning(&drive);
    // Opted in with no calibration: refused before any bridge is opened, naming the first
    // Feetech port.
    assert!(matches!(
        bring_up(&drive, &every, &ServoCalibrations::new(), &mut bridges),
        Err(OpenError::Uncalibrated { port, bus_id: 3 }) if port == "left_gripper"
    ));
    // The neck's calibration file missing one servo.
    let probe = probe();
    let mut neck = probe["calibration"]["neck"].clone();
    neck.as_object_mut()
        .expect("motors")
        .remove("so101_camera_tilt_joint");
    let partial = ServoCalibrations::new()
        .with_lerobot(
            "left_gripper",
            &probe["calibration"]["left_gripper"].to_string(),
        )
        .and_then(|c| {
            c.with_lerobot(
                "right_gripper",
                &probe["calibration"]["right_gripper"].to_string(),
            )
        })
        .and_then(|c| c.with_lerobot("neck", &neck.to_string()))
        .expect("parses");
    assert!(matches!(
        bring_up(&drive, &every, &partial, &mut bridges),
        Err(OpenError::Uncalibrated { port, bus_id: 6 }) if port == "neck"
    ));
    // The neck commissioned on a gripper's bridge, and the neck not commissioned at all.
    let mut shared: Vec<(&str, &str)> = vec![("can", "CAN")];
    shared.extend(FEETECH_PORTS[..2].iter().copied());
    shared.push(("neck", FEETECH_PORTS[0].1));
    assert!(matches!(
        bring_up(&drive, &commissioning(&drive, &shared), &probed_calibrations(), &mut bridges),
        Err(OpenError::SharedBridge(serial)) if serial == FEETECH_PORTS[0].1
    ));
    shared.pop();
    assert!(matches!(
        bring_up(&drive, &commissioning(&drive, &shared), &probed_calibrations(), &mut bridges),
        Err(OpenError::MissingPort(port)) if port == "neck"
    ));
    assert!(
        bridges.opened.is_empty(),
        "nothing opened before the refusals"
    );
    // A source that carries no serial bus refuses the first Feetech port after the CAN bridge.
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    assert!(matches!(
        bring_up(&drive, &every, &probed_calibrations(), &mut CanOnly(Sim::for_axes(&axes))),
        Err(OpenError::Port { serial, .. }) if serial == FEETECH_PORTS[0].1
    ));
    // A neck servo whose registers disagree with the calibration supplied.
    let serial = probed_buses();
    {
        let mut state = serial["5A7C118709"].state();
        // Servo 3's maximum position limit (address 11) one tick lower.
        let registers = state.servos.get_mut(&3).expect("servo 3");
        registers[11] -= 1;
    }
    let mut bridges = Bridges {
        buses: BTreeMap::from([("CAN".to_owned(), Sim::for_axes(&axes))]),
        serial,
        opened: Vec::new(),
    };
    assert!(matches!(
        bring_up(&drive, &every, &probed_calibrations(), &mut bridges),
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
    let bus = SimBus::sts3215(
        &(1..=6)
            .map(|id| (id, SO101_CALIBRATION))
            .collect::<Vec<_>>(),
    );
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
    // Governed, the first command starts from where the stop read the arm resting (mid-range,
    // tick 2000) and moves it one step: 0.25 rad/s for 20 ms is under four ticks.
    let limits = vec![
        AxisLimits {
            max_speed: 0.25,
            max_tracking_error: 0.2,
        };
        6
    ];
    let mut governed = Governor::new(body, limits).expect("limits");
    governed
        .command(&[1.0; 6])
        .expect("a first, slewed command");
    let goal = bus.state().goal(1);
    assert!((2000..=2004).contains(&goal), "{goal}");
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

#[test]
fn a_neck_servo_resting_outside_its_range_is_held_then_brought_in_at_the_servos_own_speed() {
    let drive = drive("sentient-static-upper-body");
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    let serial = probed_buses();
    let neck = serial["5A7C118709"].clone();
    // The camera tilt (id 6) rests at tick 753, about 1000 below its calibrated 1760.
    assert_eq!(neck.present(6), 753);
    let mut bridges = Bridges {
        buses: BTreeMap::from([("CAN".to_owned(), Sim::for_axes(&axes))]),
        serial,
        opened: Vec::new(),
    };
    let mut body = bring_up(
        &drive,
        &static_commissioning(&drive),
        &probed_calibrations(),
        &mut bridges,
    )
    .expect("open does not refuse a servo resting outside its range");
    // The Feetech axes bound their own approach; the Damiao ones do not.
    assert!((0..15).all(|axis| !body.approach_speed_limited(axis)));
    assert!((15..25).all(|axis| body.approach_speed_limited(axis)));
    // Every Feetech servo holds Goal_Speed 3 and Acceleration 3 (0.25 rad/s, 0.5 rad/s²).
    {
        let state = neck.state();
        let registers = &state.servos[&6];
        assert_eq!((registers[46], registers[47], registers[41]), (3, 0, 3));
    }
    // The first command asks for the tilt's nearest commandable joint, its lower bound.
    let mut target = rest_inside(&body);
    target[24] = body.commandable()[24].lower;
    let before = neck.state().written.len();
    body.command(&target).expect("commanded");
    let written: Vec<Vec<u8>> = neck.state().written[before..].to_vec();
    // Before torque came on, servo 6's goal was written to where it rests clamped into its
    // calibrated limits, 1760: the firmware would have driven it there anyway, and this enable
    // had just confirmed its approach limit.
    let enable = written
        .iter()
        .position(|packet| packet[2] == 6 && packet[4..7] == [0x03, 40, 1])
        .expect("servo 6 enabled");
    let [low, high] = TILT_MIN.to_le_bytes();
    assert!(
        written[..enable]
            .iter()
            .any(|packet| packet[2] == 6 && packet[4..8] == [0x03, 42, low, high]),
        "the goal was held at the nearest limit before enabling"
    );
    assert!(
        !written[..enable]
            .iter()
            .any(|packet| packet[2] == 6 && packet[4..8] == [0x03, 42, 0xF1, 0x02]),
        "never at the rest tick 753, outside the limits"
    );
    // Its declared range reaches below what its unit does, so its goal is that reach's edge,
    // tick 1760 to within the radian round trip.
    let goal = neck.state().goal(6);
    assert!(
        goal + 1 >= TILT_MIN,
        "the goal is at the calibrated range's edge: {goal}"
    );
    // The servo itself moves at most 3 steps of 0.732 rpm: about 150 ticks a second.
    neck.advance(Duration::from_millis(100));
    let moved = neck.present(6) - 753;
    assert!((1..=15).contains(&moved), "{moved} ticks in 100 ms");
    neck.advance(Duration::from_secs(1));
    assert!(neck.present(6) - 753 <= 15 + 150);
    assert!(body.stop().is_proven());
}

/// The static upper body opened on simulated buses resting where the probe found every servo,
/// with its neck's bus.
fn probed_body() -> (Body, SimBus, BTreeMap<String, SimBus>) {
    let drive = drive("sentient-static-upper-body");
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    let serial = probed_buses();
    let mut bridges = Bridges {
        buses: BTreeMap::from([("CAN".to_owned(), Sim::for_axes(&axes))]),
        serial: serial.clone(),
        opened: Vec::new(),
    };
    let body = bring_up(
        &drive,
        &static_commissioning(&drive),
        &probed_calibrations(),
        &mut bridges,
    )
    .expect("open");
    (body, serial["5A7C118709"].clone(), serial)
}

/// Where each axis rests, held inside its declared bounds.
fn rest_held(body: &Body) -> Vec<f64> {
    body.last_observed()
        .expect("rest")
        .iter()
        .zip(body.axes())
        .map(|(rest, axis)| rest.clamp(axis.lower, axis.upper))
        .collect()
}

/// Every axis 0.2 rad/s and 0.2 rad: slower than the servos' own 0.23 rad/s, so a following
/// servo never falls behind.
fn bring_up_limits() -> Vec<AxisLimits> {
    vec![
        AxisLimits {
            max_speed: 0.2,
            max_tracking_error: 0.2,
        };
        25
    ]
}

/// The tilt's lowest calibrated tick.
const TILT_MIN: u16 = 1760;

/// A clock the test moves: one 20 ms control period per command.
#[derive(Clone)]
struct Stepped(std::sync::Arc<std::sync::Mutex<std::time::Instant>>);

impl Stepped {
    fn new() -> Self {
        Self(std::sync::Arc::new(std::sync::Mutex::new(
            std::time::Instant::now(),
        )))
    }

    fn tick(&self) {
        *self.0.lock().expect("clock") += Duration::from_millis(20);
    }
}

impl sx_embodiment_drivers::Clock for Stepped {
    fn now(&self) -> std::time::Instant {
        *self.0.lock().expect("clock")
    }
}

#[test]
fn a_governed_static_upper_body_brings_the_neck_in_from_where_the_probe_found_it() {
    let (body, neck, serial) = probed_body();
    let target = rest_held(&body);
    let clock = Stepped::new();
    let mut governed =
        Governor::with_clock(body, bring_up_limits(), clock.clone()).expect("limits");
    // Hold every axis where it rests, the out-of-range neck axes at their nearest in-range
    // position, which the servos themselves approach at their own speed.
    let mut tilt = Vec::new();
    for _ in 0..900 {
        clock.tick();
        governed
            .command(&target)
            .unwrap_or_else(|error| panic!("{error}"));
        for bus in serial.values() {
            bus.advance(Duration::from_millis(20));
        }
        tilt.push(neck.present(6));
    }
    // The camera tilt left tick 753 at the servo's pace, about 150 ticks a second and no
    // faster (the simulated servo moves whole ticks per 20 ms step, so it lags that pace: 2 a
    // step), until it reached its range.
    assert!(tilt[0] > 753 && tilt[0] <= 753 + 3);
    let moved = tilt[99] - 753;
    assert!((150..=300).contains(&moved), "{moved}");
    // It came to rest at the in-range position the governor held: its declared range reaches
    // below its unit's, so the edge its unit reaches, tick 1760 to within the radian round trip.
    let held = neck.state().goal(6);
    assert!(held + 1 >= TILT_MIN && held <= TILT_MIN + 1, "{held}");
    // The servo stops at its own calibrated minimum.
    assert_eq!(*tilt.last().expect("ticks"), TILT_MIN);
    // Inside, the tilt is tracked again like any axis: commanded away while its servo stalls,
    // the governor stops the body and latches a tracking fault.
    let mut away = target.clone();
    away[24] = -0.5;
    let mut failure = None;
    for _ in 0..200 {
        clock.tick();
        if let Err(error) = governed.command(&away) {
            failure = Some(error);
            break;
        }
    }
    assert!(
        matches!(failure, Some(GovernorError::Tracking { ref joint, .. }) if joint == "so101_camera_tilt_joint"),
        "{failure:?}"
    );
}

#[test]
fn a_neck_axis_resting_past_its_calibrated_reach_is_brought_in_and_held_at_what_it_reaches() {
    let drive = drive("sentient-static-upper-body");
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    let serial = probed_buses();
    let neck = serial["5A7C118709"].clone();
    // The shoulder lift (id 2) rests at +1.7 rad: inside its declared +/-1.745 rad, past its
    // unit's calibrated 932..3039 ticks, 1985.5 +/- 1053.5 (1.616 rad) about the drive's zero.
    let resting = 3093;
    neck.rest_at(2, resting);
    let mut bridges = Bridges {
        buses: BTreeMap::from([("CAN".to_owned(), Sim::for_axes(&axes))]),
        serial: serial.clone(),
        opened: Vec::new(),
    };
    let body = bring_up(
        &drive,
        &static_commissioning(&drive),
        &probed_calibrations(),
        &mut bridges,
    )
    .expect("open");
    let lift = body
        .axes()
        .iter()
        .position(|axis| axis.joint == "so101_shoulder_lift_joint")
        .expect("the shoulder lift");
    let declared = body.axes()[lift].upper;
    let range = body.commandable()[lift];
    let reach = 1053.5 * std::f64::consts::TAU / 4095.0;
    assert!((range.upper - reach).abs() < 1e-9, "{range:?}");
    assert!(range.upper < 1.7 && 1.7 < declared);
    let rest = body.last_observed().expect("rest");
    assert!((rest[lift] - 1.7).abs() < 1e-3, "{}", rest[lift]);
    // Asked for 1.74 rad, inside the declared bound and past what the unit reaches.
    let mut target = rest_held(&body);
    target[lift] = 1.74;
    let clock = Stepped::new();
    let mut governed =
        Governor::with_clock(body, bring_up_limits(), clock.clone()).expect("limits");
    let before = neck.state().written.len();
    let mut lifts = Vec::new();
    for _ in 0..900 {
        clock.tick();
        governed
            .command(&target)
            .unwrap_or_else(|error| panic!("{error}"));
        for bus in serial.values() {
            bus.advance(Duration::from_millis(20));
        }
        lifts.push(neck.present(2));
    }
    // Before torque came on, its goal was its rest clamped into its limits: 3039.
    let written = neck.state().written[before..].to_vec();
    let enable = written
        .iter()
        .position(|packet| packet[2] == 2 && packet[4..7] == [0x03, 40, 1])
        .expect("servo 2 enabled");
    let [low, high] = 3039_u16.to_le_bytes();
    assert!(
        written[..enable]
            .iter()
            .any(|packet| packet[2] == 2 && packet[4..8] == [0x03, 42, low, high])
    );
    // It came in at the servo's own pace, then held at the edge it reaches: the governor
    // never tracked it toward 1.74 rad, which the firmware would clamp.
    assert!(
        lifts[0] < resting && resting - lifts[0] <= 3,
        "{}",
        lifts[0]
    );
    let last = *lifts.last().expect("ticks");
    assert!((3030..=3039).contains(&last), "{last}");
    assert!(neck.state().goal(2) <= 3039);
}

/// A body that does not claim its devices bound their own approach.
struct Unlimited(Body);

impl ActuatorChain for Unlimited {
    type Error = <Body as ActuatorChain>::Error;
    type Feedback = <Body as ActuatorChain>::Feedback;

    fn axes(&self) -> &[sx_embodiment_drivers::BoundAxis] {
        self.0.axes()
    }

    fn commandable(&self) -> &[sx_embodiment_drivers::CommandRange] {
        self.0.commandable()
    }

    fn command(
        &mut self,
        joints: &[f64],
    ) -> Result<sx_embodiment_drivers::Receipt<Self::Feedback>, Self::Error> {
        self.0.command(joints)
    }

    fn stop(&mut self) -> sx_embodiment_drivers::StopReport {
        self.0.stop()
    }

    fn stop_budget(&self) -> Duration {
        self.0.stop_budget()
    }

    fn last_observed(&self) -> Option<Vec<f64>> {
        self.0.last_observed()
    }
}

#[test]
fn without_a_confirmed_approach_limit_the_governor_refuses_a_neck_resting_outside() {
    let (body, neck, _) = probed_body();
    let target = rest_held(&body);
    let mut governed = Governor::new(Unlimited(body), bring_up_limits()).expect("limits");
    let written = neck.state().written.len();
    let refused = governed.command(&target).expect_err("refused");
    assert!(
        matches!(refused, GovernorError::RestingOutside { .. }),
        "{refused}"
    );
    assert_eq!(neck.state().written.len(), written, "nothing was sent");
    assert_eq!(neck.state().torque(6), 0);
}

#[test]
fn by_default_the_provisional_neck_and_grippers_stay_off_and_the_damiao_bus_drives() {
    let drive = drive("sentient-static-upper-body");
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    assert_eq!(
        closed_ports(&drive, OpenOptions::default()).expect("closed"),
        ["left_gripper", "right_gripper", "neck"]
    );
    let can = Sim::for_axes(&axes);
    let mut bridges = Bridges {
        buses: BTreeMap::from([("CAN".to_owned(), can.clone())]),
        serial: probed_buses(),
        opened: Vec::new(),
    };
    // Commissioning may name the provisional ports; nothing opens them, and no calibration is
    // needed.
    let mut body = open(&drive, &static_commissioning(&drive), &mut bridges).expect("open");
    assert_eq!(bridges.opened, ["CAN"]);
    assert_eq!(body.axes().len(), 15);
    assert!(body.axes().iter().all(|axis| axis.port == "can"));
    body.command(&[0.0; 15]).expect("the Damiao axes drive");
    assert!(body.stop().is_proven());
    // An opt-in for a port that is not provisional is refused.
    for wrong in ["can", "elbow"] {
        let options = OpenOptions {
            allow_provisional: &[wrong],
        };
        assert!(matches!(
            open_with(
                &drive,
                &static_commissioning(&drive),
                &probed_calibrations(),
                options,
                &mut bridges
            ),
            Err(OpenError::NotProvisional(port)) if port == wrong
        ));
    }
    // One port opted in opens that one alone beside the verified bus.
    let neck_only = OpenOptions {
        allow_provisional: &["neck"],
    };
    let mut bridges = Bridges {
        buses: BTreeMap::from([("CAN".to_owned(), Sim::for_axes(&axes))]),
        serial: probed_buses(),
        opened: Vec::new(),
    };
    let body = open_with(
        &drive,
        &static_commissioning(&drive),
        &probed_calibrations(),
        neck_only,
        &mut bridges,
    )
    .expect("open with the neck");
    assert_eq!(bridges.opened, ["CAN", "5A7C118709@1000000"]);
    assert_eq!(body.axes().len(), 21);
}
