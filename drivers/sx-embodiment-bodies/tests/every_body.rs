//! Every driven body in the registry opens on simulated buses: a body a driver crate composes
//! through that crate, and any other (a composition that repeats a part) bus by bus as the
//! twin each bus repeats. No body is listed by hand: the bodies are the rendered registry's.

use std::collections::BTreeMap;

use sx_damiao_can::sim::Sim;
use sx_embodiment_bodies::{
    Commissioning, OpenError, OpenOptions, Port, PortSource, SerialLink, ServoCalibrations,
    damiao_motors, open_with, port_buses,
};
use sx_embodiment_drivers::lerobot::MotorCalibration;
use sx_embodiment_drivers::{ActuatorChain, BodyBindings, InstanceDrive};
use sx_feetech_serial::FEETECH_SERIAL;
use sx_feetech_serial::sim::SimBus;

const REGISTRY: &str = include_str!("../bindings/registry.json");

/// Bodies whose twin no driver crate here composes as a body of its own: each B601-DM arm is
/// opened by the B601's own opener, whose arm and gripper bridges the registry does not carry.
const UNOPENED: [&str; 2] = ["b601-dm-station", "bimanual-b601-dm"];

/// Every servo mid-range, as the SO-101 open test calibrates them.
const CALIBRATION: MotorCalibration = MotorCalibration {
    drive_mode: false,
    homing_offset: -40,
    range_min: 1000,
    range_max: 3000,
};

fn documents() -> Vec<String> {
    serde_json::from_str::<Vec<serde_json::Value>>(REGISTRY)
        .expect("registry")
        .iter()
        .map(ToString::to_string)
        .collect()
}

/// The body's nominal drive: every joint uncalibrated.
fn nominal(body: &BodyBindings) -> InstanceDrive {
    let instance = format!("{}-unit", body.embodiment);
    let document = serde_json::json!({
        "schema": "sx.instance-drive/v1",
        "embodiment": body.id,
        "instance": instance,
        "joints": body.axes.iter().map(|axis| serde_json::json!({
            "joint": axis.joint, "zero": 0.0, "direction": 1, "calibration": null,
        })).collect::<Vec<_>>(),
    });
    InstanceDrive::parse(&document.to_string(), &instance).expect("drive")
}

/// Each port's bridge, simulated, by the serial commissioning names it under (the port).
#[derive(Default)]
struct Simulated {
    can: BTreeMap<String, Sim>,
    serial: BTreeMap<String, SimBus>,
}

impl PortSource for Simulated {
    fn open(&mut self, serial: &str) -> Result<Port, OpenError> {
        self.can
            .get(serial)
            .map(|sim| Box::new(sim.clone()) as Port)
            .ok_or_else(|| OpenError::NotAttached(serial.to_owned()))
    }

    fn open_serial(&mut self, serial: &str, _baud: u32) -> Result<SerialLink, OpenError> {
        self.serial
            .get(serial)
            .map(|bus| Box::new(bus.clone()) as SerialLink)
            .ok_or_else(|| OpenError::NotAttached(serial.to_owned()))
    }
}

fn lerobot(ids: &[u16]) -> String {
    let motors: serde_json::Map<String, serde_json::Value> = ids
        .iter()
        .map(|id| {
            (
                format!("motor_{id}"),
                serde_json::json!({
                    "id": id, "drive_mode": 0, "homing_offset": CALIBRATION.homing_offset,
                    "range_min": CALIBRATION.range_min, "range_max": CALIBRATION.range_max,
                }),
            )
        })
        .collect();
    serde_json::Value::Object(motors).to_string()
}

/// Open `document`'s body with a simulated bridge on every port it declares, every provisional
/// port opted in, and stop it: the number of axes it opened with.
fn open_simulated(document: &str) -> Result<usize, OpenError> {
    let body = BodyBindings::parse(document).expect("bindings");
    let drive = nominal(&body);
    let mut bridges = Simulated::default();
    let mut calibrations = ServoCalibrations::new();
    let mut ports = BTreeMap::new();
    let mut provisional: Vec<String> = Vec::new();
    for (port, kind, marked) in port_buses(&drive)? {
        if kind == FEETECH_SERIAL {
            let ids: Vec<u16> = body
                .axes
                .iter()
                .filter(|axis| axis.port == port)
                .map(|axis| axis.binding.bus_id)
                .collect();
            let servos: Vec<(u8, MotorCalibration)> = ids
                .iter()
                .map(|id| (u8::try_from(*id).expect("servo id"), CALIBRATION))
                .collect();
            bridges
                .serial
                .insert(port.clone(), SimBus::sts3215(&servos));
            calibrations = calibrations
                .with_lerobot(&port, &lerobot(&ids))
                .expect("calibration");
        } else {
            let motors = damiao_motors(&drive, &port)?;
            bridges.can.insert(port.clone(), Sim::for_axes(&motors));
        }
        if marked {
            provisional.push(port.clone());
        }
        ports.insert(port.clone(), port);
    }
    let commissioning = Commissioning::parse(
        &serde_json::json!({
            "schema": "sx.station-commissioning/v1",
            "instance": drive.instance,
            "pod_id": "pod-a",
            "pod_revision": 1,
            "ports": ports,
        })
        .to_string(),
        &drive.instance,
    )
    .expect("commissioning");
    let provisional: Vec<&str> = provisional.iter().map(String::as_str).collect();
    let options = OpenOptions {
        allow_provisional: &provisional,
    };
    let mut opened = open_with(&drive, &commissioning, &calibrations, options, &mut bridges)?;
    let axes = opened.axes().len();
    assert!(opened.stop().is_proven(), "{} stops", body.embodiment);
    Ok(axes)
}

#[test]
fn every_driven_body_opens_on_simulated_buses() {
    let mut opened = Vec::new();
    for document in documents() {
        let body = BodyBindings::parse(&document).expect("bindings");
        let result = open_simulated(&document);
        if UNOPENED.contains(&body.embodiment.as_str()) {
            assert!(
                matches!(result, Err(OpenError::Bus { .. })),
                "{} was expected to be refused for want of a driver: {result:?}",
                body.embodiment
            );
            continue;
        }
        let axes = result.unwrap_or_else(|error| panic!("{}: {error}", body.embodiment));
        assert_eq!(axes, body.axes.len(), "{}", body.embodiment);
        opened.push(body.embodiment);
    }
    for name in [
        "bimanual-so101",
        "so101",
        "sentient-humanoid",
        "sentient-static-upper-body",
    ] {
        assert!(opened.iter().any(|body| body == name), "{name} opened");
    }
}

#[test]
fn the_bimanual_so101_opens_each_arm_on_its_own_serial_bus() {
    let document = documents()
        .into_iter()
        .find(|document| {
            BodyBindings::parse(document).expect("bindings").embodiment == "bimanual-so101"
        })
        .expect("bimanual-so101");
    let body = BodyBindings::parse(&document).expect("bindings");
    let drive = nominal(&body);
    let ports: Vec<String> = port_buses(&drive)
        .expect("ports")
        .into_iter()
        .map(|(port, _, _)| port)
        .collect();
    assert_eq!(ports, ["left_arm_serial", "right_arm_serial"]);
    assert_eq!(open_simulated(&document).expect("opens"), 12);
}
