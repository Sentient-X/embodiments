//! Actual body opener and simulated vendor buses: hand-moved fingers are measured while
//! torque is off, retained on disk, then followed under power during explicit replay.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use sx_damiao_can::sim::Sim;
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_bodies::{
    Body, Commissioning, OpenError, OpenOptions, Port, PortSource, SerialLink, ServoCalibrations,
    open_with,
};
use sx_embodiment_commission::teach::{Action, Config, Controller, Mode, Snapshot};
use sx_embodiment_commission::teach_episode::{Episode, EpisodeWriter, Header};
use sx_embodiment_drivers::lerobot::MotorCalibration;
use sx_embodiment_drivers::{ActuatorChain, Dynamics, InstanceDrive};
use sx_feetech_serial::sim::{RealTime, SimBus};

const ARM: &str = "left_shoulder_pitch_joint";
const PERIOD: Duration = Duration::from_millis(10);
static NEXT: AtomicU64 = AtomicU64::new(0);

struct Bridges {
    can: Sim,
    serial: BTreeMap<String, SimBus>,
}

impl PortSource for Bridges {
    fn open(&mut self, _serial: &str) -> Result<Port, OpenError> {
        Ok(Box::new(self.can.clone()))
    }

    fn open_serial(&mut self, serial: &str, _baud: u32) -> Result<SerialLink, OpenError> {
        self.serial
            .get(serial)
            .map(|bus| Box::new(RealTime::new(bus.clone())) as SerialLink)
            .ok_or_else(|| OpenError::NotAttached(serial.to_owned()))
    }
}

struct Rig {
    controller: Controller<Body>,
    can: Sim,
    selected_id: u16,
    fingers: Vec<(SimBus, u8, u16, usize)>,
}

impl Rig {
    #[allow(clippy::too_many_lines)] // The fixture uses the same complete bus setup as a real body.
    fn new() -> Self {
        let drives: serde_json::Value = serde_json::from_str(include_str!(
            "../../sx-damiao-can/tests/fixtures/drives.json"
        ))
        .unwrap();
        let raw = &drives["sentient-static-upper-body"]["nominal"];
        let drive =
            InstanceDrive::parse(&raw.to_string(), raw["instance"].as_str().unwrap()).unwrap();
        let axes = STATIC_UPPER_BODY.axes(&drive).unwrap();
        let can = Sim::for_axes(&axes);
        let selected_id = axes
            .iter()
            .find(|axis| axis.axis.joint == ARM)
            .unwrap()
            .axis
            .binding
            .bus_id;
        for axis in &axes {
            let joint = if axis.axis.joint == ARM {
                0.2
            } else {
                f64::midpoint(axis.axis.lower, axis.axis.upper)
            };
            can.state()
                .motors
                .get_mut(&axis.axis.binding.bus_id)
                .unwrap()
                .q = axis.axis.actuator_from_joint(joint);
        }
        let probe: serde_json::Value = serde_json::from_str(include_str!(
            "../../sx-embodiment-bodies/tests/fixtures/static-upper-body-probe.json"
        ))
        .unwrap();
        let mut serial = BTreeMap::new();
        let mut calibrations = ServoCalibrations::new();
        for port in ["left_gripper", "right_gripper", "neck"] {
            let entries = probe["calibration"][port].as_object().unwrap();
            let servos: Vec<_> = entries
                .values()
                .map(|motor| {
                    let field = |name: &str| motor[name].as_i64().unwrap();
                    (
                        u8::try_from(field("id")).unwrap(),
                        MotorCalibration {
                            drive_mode: field("drive_mode") == 1,
                            homing_offset: i16::try_from(field("homing_offset")).unwrap(),
                            range_min: u16::try_from(field("range_min")).unwrap(),
                            range_max: u16::try_from(field("range_max")).unwrap(),
                        },
                    )
                })
                .collect();
            let bus = SimBus::sts3215(&servos);
            for (id, raw) in probe["present_position"][port].as_object().unwrap() {
                bus.rest_at(
                    id.parse().unwrap(),
                    u16::try_from(raw.as_u64().unwrap()).unwrap(),
                );
            }
            serial.insert(port.to_owned(), bus);
            calibrations = calibrations
                .with_lerobot(port, &probe["calibration"][port].to_string())
                .unwrap();
        }
        let commissioning = Commissioning::parse(&serde_json::json!({
            "schema":"sx.station-commissioning/v1", "instance":drive.instance,
            "pod_id":"manual-gripper-workflow", "pod_revision":1,
            "out_of_service":drive.joints.iter().filter(|joint| joint.joint.starts_with("so101_")).map(|joint| joint.joint.clone()).collect::<Vec<_>>(),
            "ports":{"can":"CAN", "left_gripper":"left_gripper", "right_gripper":"right_gripper", "neck":"neck"}
        }).to_string(), &drive.instance).unwrap();
        let mut bridges = Bridges {
            can: can.clone(),
            serial,
        };
        let body = open_with(
            &drive,
            &commissioning,
            &calibrations,
            OpenOptions {
                allow_provisional: &["left_gripper", "right_gripper", "neck"],
                ..OpenOptions::default()
            },
            &mut bridges,
        )
        .unwrap();
        let recorded_joints: Vec<_> = body
            .axes()
            .iter()
            .filter(|axis| axis.port != "neck")
            .map(|axis| axis.joint.clone())
            .collect();
        let fingers = body
            .axes()
            .iter()
            .enumerate()
            .filter(|(_, axis)| axis.gripper)
            .map(|(index, axis)| {
                let bus = bridges.serial[&axis.port].clone();
                let id = u8::try_from(axis.binding.bus_id).unwrap();
                let initial = bus.present(id);
                let recorded = recorded_joints
                    .iter()
                    .position(|name| name == &body.axes()[index].joint)
                    .unwrap();
                (bus, id, initial, recorded)
            })
            .collect();
        let mut urdf = String::from("<robot name='manual-gripper'><link name='base'/>");
        for (index, axis) in body.axes().iter().enumerate() {
            let mass = if axis.joint == ARM { 0.2 } else { 0.0 };
            write!(urdf,"<link name='link{index}'><inertial><origin xyz='0.1 0 0'/><mass value='{mass}'/><inertia ixx='0.001' ixy='0' ixz='0' iyy='0.001' iyz='0' izz='0.001'/></inertial></link><joint name='{}' type='revolute'><parent link='base'/><child link='link{index}'/><axis xyz='0 1 0'/><limit lower='{}' upper='{}' effort='100' velocity='1'/></joint>",axis.joint,axis.lower,axis.upper).unwrap();
        }
        urdf.push_str("</robot>");
        let controller = Controller::new(
            body,
            Dynamics::of(&urdf, [0.0, 0.0, -9.81]).unwrap(),
            Config {
                joints: vec![ARM.to_owned()],
                grippers: Vec::new(),
                held: BTreeMap::new(),
                speed: 0.2,
                share: 0.25,
                damping: 0.5,
                period: PERIOD,
                max_seconds: 20.0,
            },
        )
        .unwrap();
        Self {
            controller,
            can,
            selected_id,
            fingers,
        }
    }

    fn tick(&mut self, action: Option<Action>) -> Snapshot {
        std::thread::sleep(PERIOD);
        self.controller.tick(action).expect("gripper control tick")
    }

    fn assert_gripper_torque(&self, enabled: bool) {
        assert_eq!(self.fingers.len(), 4, "exercise both complete grippers");
        for (bus, id, _, _) in &self.fingers {
            assert_eq!(bus.state().torque(*id), u8::from(enabled));
        }
    }
}

#[test]
#[allow(clippy::too_many_lines)] // Keep the complete hand-guidance/save/replay contract in one journey.
fn manually_moved_both_grippers_are_recorded_and_replayed_without_disabling_arm_support() {
    let mut rig = Rig::new();
    assert_eq!(rig.tick(Some(Action::Engage)).mode, Mode::Hold);
    rig.assert_gripper_torque(true);
    assert_eq!(rig.tick(Some(Action::Teach)).mode, Mode::Teach);
    rig.assert_gripper_torque(false);
    let path = std::env::temp_dir().join(format!(
        "sx-manual-gripper-{}-{}.jsonl",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let header = Header::new(
        "a".repeat(64),
        rig.controller.joints(),
        PERIOD.as_secs_f64(),
    )
    .unwrap();
    let mut writer = EpisodeWriter::create(&path, header.clone(), 128).unwrap();
    let mut start = None;
    let mut before = Vec::new();
    let mut after = Vec::new();
    for step in 0..=40u16 {
        // Move the encoder by hand while torque is released; no servo target is supplied.
        for (bus, id, initial, _) in &rig.fingers {
            bus.rest_at(*id, *initial + step);
        }
        let snapshot = rig.tick(None);
        rig.assert_gripper_torque(false);
        assert_eq!(snapshot.mode, Mode::Teach);
        let state = rig.can.state();
        assert_eq!(state.motors[&rig.selected_id].state, 1, "arm stays enabled");
        let frame = state
            .frames
            .iter()
            .rev()
            .find(|frame| frame.id == u32::from(rig.selected_id) && frame.data[..7] != [0xff; 7])
            .unwrap();
        let kp = (u16::from(frame.data[3] & 15) << 8) | u16::from(frame.data[4]);
        let torque = (u16::from(frame.data[6] & 15) << 8) | u16::from(frame.data[7]);
        assert_eq!(kp, 0, "arm remains compliant");
        assert_ne!(torque, 2047, "gravity support remains nonzero");
        drop(state);
        if step == 0 {
            before = snapshot.position.clone();
        }
        after = snapshot.position.clone();
        let began = *start.get_or_insert(snapshot.measured_at);
        writer
            .record(
                snapshot.measured_at.duration_since(began).as_secs_f64(),
                &snapshot.position,
                &snapshot.velocity,
            )
            .unwrap();
    }
    // Stop touching fingers before Hold. It must capture their current pose, not the pose
    // at Teach entry, before the normal position command can enable their torque again.
    rig.tick(None);
    assert_eq!(rig.tick(Some(Action::Hold)).mode, Mode::Hold);
    rig.assert_gripper_torque(true);
    for (bus, id, initial, index) in &rig.fingers {
        assert!(
            (before[*index] - after[*index]).abs() > 0.02,
            "measured finger moved"
        );
        assert!(
            bus.state().goal(*id).abs_diff(*initial + 40) <= 2,
            "Hold must not snap to the old finger pose"
        );
    }
    assert_eq!(writer.finish().unwrap(), 41);
    let episode = Episode::load(&path, &header).unwrap();
    for (_, _, _, index) in &rig.fingers {
        assert_eq!(
            episode.samples.first().unwrap().position[*index].to_bits(),
            before[*index].to_bits()
        );
        assert_eq!(
            episode.samples.last().unwrap().position[*index].to_bits(),
            after[*index].to_bits()
        );
    }
    rig.controller.set_episode(episode).unwrap();
    let deadline = Instant::now() + Duration::from_secs(4);
    while !rig.controller.replay_ready() {
        rig.tick(None);
        assert!(Instant::now() < deadline);
    }
    assert_eq!(rig.tick(Some(Action::Replay)).mode, Mode::Approach);
    while rig.controller.mode() != Mode::ReadyReplay {
        rig.tick(None);
        assert!(Instant::now() < deadline, "approach must complete");
    }
    for (bus, id, initial, _) in &rig.fingers {
        assert!(
            bus.present(*id).abs_diff(*initial) < 30,
            "powered approach moves toward recorded start"
        );
    }
    assert_eq!(rig.tick(Some(Action::Replay)).mode, Mode::Replay);
    while rig.controller.mode() != Mode::Hold {
        rig.tick(None);
        assert!(Instant::now() < deadline, "playback must finish");
    }
    rig.assert_gripper_torque(true);
    for (bus, id, initial, _) in &rig.fingers {
        assert!(
            bus.state().goal(*id).abs_diff(*initial + 40) <= 2,
            "replay follows recorded finger endpoint"
        );
    }
    std::fs::remove_file(path).unwrap();
}

#[test]
fn missing_passive_finger_reading_faults_the_teach_session() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    rig.tick(Some(Action::Teach));
    rig.assert_gripper_torque(false);
    let (bus, id, _, _) = &rig.fingers[0];
    bus.kill(*id);
    let error = rig.controller.tick(None).unwrap_err();
    assert!(error.contains("fresh") || error.contains("read"), "{error}");
    assert_eq!(rig.controller.mode(), Mode::Fault);
}

#[test]
fn passive_finger_still_respects_the_existing_command_range() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    rig.tick(Some(Action::Teach));
    let (bus, id, _, _) = &rig.fingers[0];
    bus.rest_at(*id, 0);
    let error = rig.controller.tick(None).unwrap_err();
    assert!(error.contains("range"), "{error}");
    assert_eq!(rig.controller.mode(), Mode::Fault);
}
