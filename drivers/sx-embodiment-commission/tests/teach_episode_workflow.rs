//! The local demonstration pipeline through the real body/driver and simulated CAN:
//! engage, hand-guided measurements, durable save, reload, explicit approach and playback.
//! This verifies software behavior, not physical backdrive stability or camera capture.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use sx_damiao_can::sim::Sim;
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_bodies::{Body, Commissioning, OpenError, Port, PortSource, open};
use sx_embodiment_commission::teach::{Action, Config, Controller, Mode, Snapshot};
use sx_embodiment_commission::teach_episode::{Episode, EpisodeWriter, Header};
use sx_embodiment_drivers::can::{CanFrame, CanPort};
use sx_embodiment_drivers::dynamics::sha256_hex;
use sx_embodiment_drivers::{ActuatorChain, Dynamics, InstanceDrive};

const SELECTED: &str = "left_shoulder_pitch_joint";
const PERIOD: Duration = Duration::from_millis(10);
static NEXT: AtomicU64 = AtomicU64::new(0);

/// Inject only a slow first enable, reproducing work which the cyclic budget must not
/// include. All frames and acknowledgements otherwise use the actual simulated vendor bus.
#[derive(Clone)]
struct LocalCan {
    bus: Sim,
    enable_pending: Arc<AtomicBool>,
}

impl CanPort for LocalCan {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        if frame.data == [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc]
            && self.enable_pending.swap(false, Ordering::Relaxed)
        {
            std::thread::sleep(Duration::from_millis(70));
        }
        self.bus.send(frame)
    }
    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        self.bus.receive()
    }
    fn receive_until(&mut self, deadline: Instant) -> io::Result<Option<CanFrame>> {
        self.bus.receive_until(deadline)
    }
    fn sleep(&mut self, duration: Duration) {
        self.bus.sleep(duration);
    }
}

impl PortSource for LocalCan {
    fn open(&mut self, _serial: &str) -> Result<Port, OpenError> {
        Ok(Box::new(self.clone()))
    }
}

fn tick(controller: &mut Controller<Body>, action: Option<Action>) -> Snapshot {
    std::thread::sleep(PERIOD);
    controller
        .tick(action)
        .expect("local workflow control tick")
}

#[test]
#[allow(clippy::too_many_lines)] // Keep one complete physical-interface-to-file-to-replay journey visible.
fn measured_teaching_survives_durable_save_and_reload_then_replays_only_explicitly() {
    let drives: serde_json::Value = serde_json::from_str(include_str!(
        "../../sx-damiao-can/tests/fixtures/drives.json"
    ))
    .unwrap();
    let raw = &drives["sentient-static-upper-body"]["nominal"];
    let drive = InstanceDrive::parse(&raw.to_string(), raw["instance"].as_str().unwrap()).unwrap();
    let axes = STATIC_UPPER_BODY.axes(&drive).unwrap();
    let selected = axes
        .iter()
        .position(|axis| axis.axis.joint == SELECTED)
        .unwrap();
    let selected_id = axes[selected].axis.binding.bus_id;
    let bus = Sim::for_axes(&axes);
    for axis in &axes {
        let joint = if axis.axis.joint == SELECTED {
            0.2
        } else {
            f64::midpoint(axis.axis.lower, axis.axis.upper)
        };
        bus.state()
            .motors
            .get_mut(&axis.axis.binding.bus_id)
            .unwrap()
            .q = axis.axis.actuator_from_joint(joint);
    }
    let commissioning = Commissioning::parse(
        &serde_json::json!({
            "schema":"sx.station-commissioning/v1", "instance":drive.instance,
            "pod_id":"local-episode-workflow", "pod_revision":1, "ports":{"can":"SIM"}
        })
        .to_string(),
        &drive.instance,
    )
    .unwrap();
    let delayed = Arc::new(AtomicBool::new(true));
    let body = open(
        &drive,
        &commissioning,
        &mut LocalCan {
            bus: bus.clone(),
            enable_pending: Arc::clone(&delayed),
        },
    )
    .unwrap();
    assert!(
        delayed.load(Ordering::Relaxed),
        "opening a body must not enable it"
    );
    // Independent simple links keep this fixture's gravity separate from registry guesses.
    let mut urdf = String::from("<robot name='local-episode-workflow'><link name='base'/>");
    for (index, axis) in body.axes().iter().enumerate() {
        let mass = if axis.joint == SELECTED { 0.2 } else { 0.0 };
        write!(urdf,"<link name='link{index}'><inertial><origin xyz='0.1 0 0'/><mass value='{mass}'/><inertia ixx='0.001' ixy='0' ixz='0' iyy='0.001' iyz='0' izz='0.001'/></inertial></link><joint name='{}' type='revolute'><parent link='base'/><child link='link{index}'/><axis xyz='0 1 0'/><limit lower='{}' upper='{}' effort='100' velocity='1'/></joint>",axis.joint,axis.lower,axis.upper).unwrap();
    }
    urdf.push_str("</robot>");
    let mut controller = Controller::new(
        body,
        Dynamics::of(&urdf, [0.0, 0.0, -9.81]).unwrap(),
        Config {
            joints: vec![SELECTED.to_owned()],
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
    assert_eq!(
        controller.tick(Some(Action::Engage)).unwrap().mode,
        Mode::Hold
    );
    assert!(
        !delayed.load(Ordering::Relaxed),
        "the startup delay must actually be exercised"
    );
    // The normal terminal sequence is e -> t -> r, without a settled-readings gate.
    assert!(!controller.teaching_ready());
    assert_eq!(tick(&mut controller, Some(Action::Teach)).mode, Mode::Teach);
    let path = std::env::temp_dir().join(format!(
        "sx-teach-workflow-{}-{}.jsonl",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    let header = Header::new(sha256_hex(&urdf), controller.joints(), PERIOD.as_secs_f64()).unwrap();
    let (created_tx, created_rx) = mpsc::sync_channel(1);
    let (created_path, created_header) = (path.clone(), header.clone());
    std::thread::spawn(move || {
        let _ = created_tx.send(EpisodeWriter::create(&created_path, created_header, 128));
    });
    let create_deadline = Instant::now() + Duration::from_secs(3);
    let mut writer = loop {
        tick(&mut controller, None);
        if let Ok(created) = created_rx.try_recv() {
            break created.unwrap();
        }
        assert!(
            Instant::now() < create_deadline,
            "local recording could not be created"
        );
    };
    let mut began = None;
    for sample in 0..=120 {
        // The bus's encoder observation is varied by hand, never a position command supplied
        // as the demonstration. Returning to the start avoids an unrelated large approach.
        let phase = f64::from(sample) / 120.0 * std::f64::consts::TAU;
        // A smooth feasible demonstration exceeds the slow positioning cap and
        // starts/stops at rest. Playback must preserve its recorded timing.
        let joint = 0.2 + 0.09 * (1.0 - phase.cos());
        bus.state().motors.get_mut(&selected_id).unwrap().q =
            axes[selected].axis.actuator_from_joint(joint);
        let snapshot = tick(&mut controller, None);
        assert_eq!(snapshot.mode, Mode::Teach);
        let start = *began.get_or_insert(snapshot.measured_at);
        writer
            .record(
                snapshot.measured_at.duration_since(start).as_secs_f64(),
                &snapshot.position,
                &snapshot.velocity,
            )
            .unwrap();
    }
    assert_eq!(tick(&mut controller, Some(Action::Hold)).mode, Mode::Hold);
    // The real CLI drains/syncs and reloads away from the motor thread. Continue refreshing
    // the hold while that work runs, rather than hiding a disk wait inside a control tick.
    let (saved_tx, saved_rx) = mpsc::sync_channel(1);
    let (saved_path, saved_header) = (path.clone(), header.clone());
    std::thread::spawn(move || {
        let saved = writer.finish().and_then(|count| {
            Episode::load(&saved_path, &saved_header).map(|episode| (count, episode))
        });
        saved_tx.send(saved).unwrap();
    });
    let save_deadline = Instant::now() + Duration::from_secs(3);
    let (count, episode) = loop {
        tick(&mut controller, None);
        if let Ok(saved) = saved_rx.try_recv() {
            break saved.unwrap();
        }
        assert!(
            Instant::now() < save_deadline,
            "local recording did not complete"
        );
    };
    assert_eq!(count, 121);
    let recorded_peak_speed = episode
        .samples
        .windows(2)
        .map(|pair| {
            (pair[1].position[selected] - pair[0].position[selected]).abs()
                / (pair[1].t_s - pair[0].t_s)
        })
        .fold(0.0_f64, f64::max);
    assert!(
        recorded_peak_speed > 0.2,
        "the demonstration must exceed the positioning cap"
    );
    let (low, high) = episode
        .samples
        .iter()
        .map(|sample| sample.position[selected])
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), value| {
            (low.min(value), high.max(value))
        });
    assert!(
        high - low > 0.006,
        "the saved take must contain actual measured movement"
    );
    assert_eq!(episode.samples[0].t_s.to_bits(), 0.0_f64.to_bits());
    let recorded_duration = episode.samples.last().unwrap().t_s;
    let final_pose = episode.samples.last().unwrap().position.clone();
    controller.set_episode(episode).unwrap();
    let validation_deadline = Instant::now() + Duration::from_secs(1);
    while !controller.replay_ready() {
        assert!(
            Instant::now() < validation_deadline,
            "saved episode was not validated"
        );
        tick(&mut controller, None);
    }
    assert!((controller.playback_rate() - 1.0).abs() < f64::EPSILON);
    assert_eq!(
        tick(&mut controller, Some(Action::Replay)).mode,
        Mode::Approach
    );
    let approach_deadline = Instant::now() + Duration::from_secs(2);
    while controller.mode() != Mode::ReadyReplay {
        assert!(
            Instant::now() < approach_deadline,
            "replay approach never settled"
        );
        tick(&mut controller, None);
    }
    assert_eq!(
        tick(&mut controller, None).mode,
        Mode::ReadyReplay,
        "arrival must not start playback"
    );
    let replay_frame_start = bus.state().frames.len();
    let replay_began = Instant::now();
    assert_eq!(
        tick(&mut controller, Some(Action::Replay)).mode,
        Mode::Replay
    );
    let replay_deadline = Instant::now() + Duration::from_secs(6);
    let final_read = loop {
        let snapshot = tick(&mut controller, None);
        if snapshot.mode == Mode::Hold {
            break snapshot;
        }
        assert!(
            Instant::now() < replay_deadline,
            "recorded motion never finished in hold"
        );
    };
    let replay_duration = replay_began.elapsed().as_secs_f64();
    assert!(replay_duration >= recorded_duration);
    assert!(
        replay_duration < recorded_duration + 1.5,
        "playback must use recorded timing plus settling, not the positioning cap"
    );
    let q_max = bus.state().motors[&selected_id].limits[0];
    let commanded: Vec<_> = bus.state().frames[replay_frame_start..]
        .iter()
        .filter(|frame| frame.id == u32::from(selected_id) && frame.data[..7] != [0xff; 7])
        .map(|frame| {
            let code = (u16::from(frame.data[0]) << 8) | u16::from(frame.data[1]);
            let actuator = f64::from(code) / 65_535.0 * 2.0 * q_max - q_max;
            axes[selected].axis.joint_from_actuator(actuator)
        })
        .collect();
    let (command_low, command_high) = commanded
        .iter()
        .copied()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(low, high), value| {
            (low.min(value), high.max(value))
        });
    let quantum = (axes[selected]
        .axis
        .joint_from_actuator(2.0 * q_max / 65_535.0)
        - axes[selected].axis.joint_from_actuator(0.0))
    .abs();
    assert!(
        command_high - command_low > 0.004,
        "a stationary replay must not pass just because its final pose equals its start"
    );
    assert!(
        (command_low - low).abs() <= 2.0 * quantum && (command_high - high).abs() <= 2.0 * quantum,
        "replay MIT target extrema {command_low}..{command_high} must traverse the measured extrema {low}..{high} within encoder quantization"
    );
    for (observed, expected) in final_read.position.iter().zip(final_pose) {
        assert!(
            (observed - expected).abs() < 0.03,
            "replay did not reach recorded final pose"
        );
    }
    tick(&mut controller, Some(Action::Release));
    assert_eq!(controller.mode(), Mode::Released);
    std::fs::remove_file(path).unwrap();
}
