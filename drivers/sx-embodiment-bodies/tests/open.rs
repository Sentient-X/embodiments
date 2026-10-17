//! A body opens from its drive and its commissioning, on the simulated Damiao bus, and every
//! mismatch is refused before a byte reaches a bus.

use std::collections::BTreeMap;

use sx_damiao_can::b601::b601;
use sx_damiao_can::humanoid::humanoid_upper_body;
use sx_damiao_can::sim::Sim;
use sx_embodiment_bodies::{Body, Commissioning, OpenError, Port, PortSource, open, ports};
use sx_embodiment_drivers::{ActuatorChain, InstanceDrive};

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

/// Bridges by serial, each a simulated bus; records every serial opened.
struct Bridges {
    buses: BTreeMap<String, Sim>,
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
        opened: Vec::new(),
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
        opened: Vec::new(),
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
        opened: Vec::new(),
    };
    let mut body = open(
        &drive,
        &commissioning(&drive, &[("can", "CAN")]),
        &mut bridges,
    )
    .expect("humanoid");
    assert!(matches!(body, Body::HumanoidUpperBody(_)));
    assert_eq!(body.axes().len(), axes.len());
    assert!(body.stop().is_proven());
}

#[test]
fn every_mismatch_is_refused_before_a_bus_is_opened() {
    let drive = drive("b601-dm");
    let mut bridges = Bridges {
        buses: BTreeMap::new(),
        opened: Vec::new(),
    };
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
        opened: Vec::new(),
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
