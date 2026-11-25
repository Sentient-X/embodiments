//! Exercise the local controller through the real body opener, Damiao driver and simulated
//! vendor bus. These tests establish command/transition behavior, not physical float stability.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use serde_json::Value;
use sx_damiao_can::dm_can::float_to_uint;
use sx_damiao_can::sim::Sim;
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_bodies::{Body, Commissioning, OpenError, Port, PortSource, open};
use sx_embodiment_commission::teach::{Action, Config, Controller, Mode};
use sx_embodiment_commission::teach_episode::{Episode, Header, Sample};
use sx_embodiment_drivers::can::{CanFrame, CanPort};
use sx_embodiment_drivers::guidance::GuidanceProfile;
use sx_embodiment_drivers::{ActuatorChain, Dynamics, InstanceDrive, MAX_STEP_INTERVAL};

const SELECTED: &str = "left_shoulder_pitch_joint";
const DRIVES: &str = include_str!("../../sx-damiao-can/tests/fixtures/drives.json");

/// A real simulated vendor bus with selected temperature bytes or a one-shot hold
/// transport delay. The production driver still constructs and decodes every exchange.
#[derive(Default)]
struct ActivationProbe {
    enable_delay_ms: AtomicU64,
    encoder_shift_bits: AtomicU64,
    lose_following_read: AtomicBool,
    reply_delay_ms: AtomicU64,
    collecting: AtomicBool,
    selected_seen: AtomicBool,
    torque_override_bits: AtomicU64,
    velocity_override_bits: AtomicU64,
    position_offset_bits: AtomicU64,
}

#[derive(Clone)]
struct Can {
    sim: Sim,
    temperature: Arc<AtomicU8>,
    selected_master: u32,
    selected_id: u32,
    next_hold_delay_ms: Arc<AtomicU64>,
    activation: Arc<ActivationProbe>,
}

impl Can {
    fn measured(&self, frame: Option<CanFrame>) -> Option<CanFrame> {
        frame.map(|mut frame| {
            if self.activation.collecting.load(Ordering::Relaxed) {
                if frame.id == self.selected_master {
                    self.activation.selected_seen.store(true, Ordering::Relaxed);
                } else if self.activation.selected_seen.load(Ordering::Relaxed) {
                    let delay = self.activation.reply_delay_ms.swap(0, Ordering::Relaxed);
                    if delay > 0 {
                        std::thread::sleep(Duration::from_millis(delay));
                        self.activation.collecting.store(false, Ordering::Relaxed);
                    }
                }
            }
            let register_reply = u32::from(u16::from_le_bytes([frame.data[0], frame.data[1]]))
                == self.selected_id
                && matches!(frame.data[2], 0x33 | 0x55);
            if frame.id == self.selected_master && !register_reply {
                let offset =
                    f64::from_bits(self.activation.position_offset_bits.load(Ordering::Relaxed));
                if offset != 0.0 {
                    let qmax = self.sim.state().motors
                        [&u16::try_from(self.selected_id).expect("bus id")]
                        .limits[0];
                    let code = u16::from_be_bytes([frame.data[1], frame.data[2]]);
                    let q = f64::from(code) / 65535.0 * 2.0 * qmax - qmax;
                    let shifted =
                        float_to_uint(q + offset, -qmax, qmax, 16).expect("encoder offset");
                    [frame.data[1], frame.data[2]] = shifted.to_be_bytes();
                }
                let velocity_bits = self
                    .activation
                    .velocity_override_bits
                    .load(Ordering::Relaxed);
                if velocity_bits != 0 {
                    let vmax = self.sim.state().motors
                        [&u16::try_from(self.selected_id).expect("bus id")]
                        .limits[1];
                    let code = float_to_uint(f64::from_bits(velocity_bits), -vmax, vmax, 12)
                        .expect("injected measured velocity");
                    frame.data[3] = u8::try_from(code >> 4).expect("high velocity");
                    frame.data[4] = (frame.data[4] & 15) | (code.to_le_bytes()[0] << 4);
                }
                let state = self.sim.state();
                let motor = &state.motors[&u16::try_from(self.selected_id).expect("bus id")];
                let override_bits = self.activation.torque_override_bits.load(Ordering::Relaxed);
                let torque_code = if override_bits != 0 {
                    float_to_uint(
                        f64::from_bits(override_bits),
                        -motor.limits[2],
                        motor.limits[2],
                        12,
                    )
                    .expect("injected torque")
                } else if let Some(dynamics) = motor.dynamics {
                    let torque = if motor.state == 1 {
                        (dynamics.kp * (dynamics.p_des - motor.q)
                            + dynamics.kd * (dynamics.v_des - dynamics.velocity)
                            + dynamics.tau_ff)
                            .clamp(-dynamics.peak, dynamics.peak)
                    } else {
                        0.0
                    };
                    float_to_uint(torque, -motor.limits[2], motor.limits[2], 12)
                        .expect("plant torque")
                } else {
                    state
                        .frames
                        .iter()
                        .rev()
                        .find(|f| f.id == self.selected_id && f.data[..7] != [0xff; 7])
                        .map_or_else(
                            || {
                                float_to_uint(0.0, -motor.limits[2], motor.limits[2], 12)
                                    .expect("zero torque")
                            },
                            |f| (u16::from(f.data[6] & 15) << 8) | u16::from(f.data[7]),
                        )
                };
                frame.data[4] =
                    (frame.data[4] & 0xf0) | u8::try_from(torque_code >> 8).expect("high torque");
                frame.data[5] = torque_code.to_le_bytes()[0];
            }
            let temperature = self.temperature.load(Ordering::Relaxed);
            if temperature != 0 && frame.id == self.selected_master {
                frame.data[6] = temperature;
                frame.data[7] = temperature;
            }
            frame
        })
    }
}

impl CanPort for Can {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        if frame.id == self.selected_id
            && frame.data == [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc]
        {
            let delay = self.activation.enable_delay_ms.swap(0, Ordering::Relaxed);
            if delay > 0 {
                std::thread::sleep(Duration::from_millis(delay));
            }
            let shift = f64::from_bits(
                self.activation
                    .encoder_shift_bits
                    .swap(0, Ordering::Relaxed),
            );
            if shift.abs() > 0.0 {
                self.sim
                    .state()
                    .motors
                    .get_mut(&u16::try_from(self.selected_id).expect("bus id"))
                    .expect("motor")
                    .q += shift;
            }
        }
        let passive_activation =
            frame.id == self.selected_id && frame.data[..7] != [0xff; 7] && kp(frame.data) == 0;
        if passive_activation {
            self.activation.collecting.store(true, Ordering::Relaxed);
            self.activation
                .selected_seen
                .store(false, Ordering::Relaxed);
        }
        if frame.id == self.selected_id && frame.data[..7] != [0xff; 7] && kp(frame.data) > 0 {
            let delay = self.next_hold_delay_ms.swap(0, Ordering::Relaxed);
            if delay > 0 {
                std::thread::sleep(Duration::from_millis(delay));
            }
        }
        self.sim.send(frame)?;
        if passive_activation
            && self
                .activation
                .lose_following_read
                .swap(false, Ordering::Relaxed)
        {
            self.sim
                .kill(u16::try_from(self.selected_id).expect("bus id"));
        }
        Ok(())
    }
    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        let frame = self.sim.receive()?;
        Ok(self.measured(frame))
    }
    fn receive_until(&mut self, deadline: Instant) -> io::Result<Option<CanFrame>> {
        let frame = self.sim.receive_until(deadline)?;
        Ok(self.measured(frame))
    }
    fn sleep(&mut self, duration: Duration) {
        self.sim.sleep(duration);
    }
}

impl PortSource for Can {
    fn open(&mut self, _serial: &str) -> Result<Port, OpenError> {
        Ok(Box::new(self.clone()))
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum GuidanceCase {
    Distinct,
    TightCap,
    Coupled,
}

struct Rig {
    controller: Controller<Body>,
    /// The instant just before the last `Controller::tick`: no clock reading inside that
    /// tick, the governor's included, can be earlier.
    dispatched: Instant,
    bus: Sim,
    selected: usize,
    id: u16,
    gain: f64,
    kp: f64,
    tmax: f64,
    temperature: Arc<AtomicU8>,
    torso_id: u16,
    isolated_ids: Vec<u16>,
    next_hold_delay_ms: Arc<AtomicU64>,
    activation: Arc<ActivationProbe>,
}

impl Rig {
    fn new() -> Self {
        Self::configured(false, false, Vec::new(), None)
    }

    fn configured(
        fixed_torso: bool,
        near_limit: bool,
        grippers: Vec<sx_embodiment_ik::teleop::Gripper>,
        held_torso: Option<f64>,
    ) -> Self {
        Self::configured_with_torso_sensor(
            fixed_torso,
            near_limit,
            grippers,
            held_torso,
            [0.0, 1.0, 0.0],
            false,
        )
    }

    fn configured_with_torso_sensor(
        fixed_torso: bool,
        near_limit: bool,
        grippers: Vec<sx_embodiment_ik::teleop::Gripper>,
        held_torso: Option<f64>,
        torso_axis: [f64; 3],
        silent_unavailable: bool,
    ) -> Self {
        Self::configured_isolated(
            fixed_torso,
            near_limit,
            grippers,
            held_torso,
            torso_axis,
            silent_unavailable,
            &[],
        )
    }

    fn configured_isolated(
        fixed_torso: bool,
        near_limit: bool,
        grippers: Vec<sx_embodiment_ik::teleop::Gripper>,
        held_torso: Option<f64>,
        torso_axis: [f64; 3],
        silent_unavailable: bool,
        extra_unavailable: &[&str],
    ) -> Self {
        Self::configured_profile(
            fixed_torso,
            near_limit,
            grippers,
            held_torso,
            torso_axis,
            silent_unavailable,
            extra_unavailable,
            None,
        )
    }

    #[allow(
        clippy::too_many_lines,
        clippy::too_many_arguments,
        clippy::fn_params_excessive_bools
    )] // Complete simulated hardware fixture; booleans select independent failure conditions.
    fn configured_profile(
        fixed_torso: bool,
        near_limit: bool,
        grippers: Vec<sx_embodiment_ik::teleop::Gripper>,
        held_torso: Option<f64>,
        torso_axis: [f64; 3],
        silent_unavailable: bool,
        extra_unavailable: &[&str],
        guidance: Option<GuidanceCase>,
    ) -> Self {
        let drives: Value = serde_json::from_str(DRIVES).expect("drives");
        let nominal = &drives["sentient-static-upper-body"]["nominal"];
        let drive = InstanceDrive::parse(
            &nominal.to_string(),
            nominal["instance"].as_str().expect("instance"),
        )
        .expect("drive");
        let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
        let selected = axes
            .iter()
            .position(|a| a.axis.joint == SELECTED)
            .expect("selected");
        let bus = Sim::for_axes(&axes);
        for axis in &axes {
            let q = if axis.axis.joint == SELECTED {
                if near_limit {
                    axis.axis.upper - 0.01
                } else {
                    0.2
                }
            } else if extra_unavailable.contains(&axis.axis.joint.as_str()) {
                axis.axis.upper + 0.5
            } else if fixed_torso && axis.axis.joint == "torso_joint" {
                0.0
            } else {
                f64::midpoint(axis.axis.lower, axis.axis.upper)
            };
            bus.state()
                .motors
                .get_mut(&axis.axis.binding.bus_id)
                .expect("motor")
                .q = axis.axis.actuator_from_joint(q);
        }
        let mut unavailable: Vec<&str> = extra_unavailable.to_vec();
        if fixed_torso {
            unavailable.push("torso_joint");
        }
        let commissioning = Commissioning::parse(
            &serde_json::json!({
                "schema": "sx.station-commissioning/v1", "instance": drive.instance,
                "pod_id": "local-teach-test", "pod_revision": 1, "ports": {"can": "CAN"},
                "out_of_service": unavailable
            })
            .to_string(),
            &drive.instance,
        )
        .expect("commissioning");
        let temperature = Arc::new(AtomicU8::new(0));
        let next_hold_delay_ms = Arc::new(AtomicU64::new(0));
        let activation = Arc::new(ActivationProbe::default());
        let mut can = Can {
            sim: bus.clone(),
            temperature: Arc::clone(&temperature),
            selected_master: u32::from(
                bus.state().motors[&axes[selected].axis.binding.bus_id].master,
            ),
            selected_id: u32::from(axes[selected].axis.binding.bus_id),
            next_hold_delay_ms: Arc::clone(&next_hold_delay_ms),
            activation: Arc::clone(&activation),
        };
        let body = open(&drive, &commissioning, &mut can).expect("body");
        // Independent pendulums: only the selected joint carries a 0.2 kg mass at x=0.1 m.
        // Its analytic gravity support is -0.1962*cos(q), independent of registry dynamics.
        let mut urdf = String::from("<robot name='teach-test'><link name='base'/>");
        for (i, axis) in body.axes().iter().enumerate() {
            let coupled_elbow =
                guidance == Some(GuidanceCase::Coupled) && axis.joint == "left_elbow_pitch_joint";
            let mass = if coupled_elbow {
                0.4
            } else if axis.joint == SELECTED {
                0.2
            } else {
                0.0
            };
            let centre = if coupled_elbow { 0.2 } else { 0.1 };
            let parent = if coupled_elbow {
                format!("link{selected}")
            } else if fixed_torso && axis.joint == SELECTED {
                "link0".to_owned()
            } else {
                "base".to_owned()
            };
            let [ax, ay, az] = if axis.joint == "torso_joint" {
                torso_axis
            } else {
                [0.0, 1.0, 0.0]
            };
            write!(urdf, "<link name='link{i}'><inertial><origin xyz='{centre} 0 0'/><mass value='{mass}'/><inertia ixx='0.001' ixy='0' ixz='0' iyy='0.001' iyz='0' izz='0.001'/></inertial></link><joint name='{}' type='revolute'><parent link='{parent}'/><child link='link{i}'/><axis xyz='{ax} {ay} {az}'/><limit lower='{}' upper='{}' effort='100' velocity='1'/></joint>", axis.joint, axis.lower, axis.upper).expect("URDF formatting");
        }
        urdf.push_str("</robot>");
        let model = Dynamics::of(&urdf, [0.0, 0.0, -9.81]).expect("model");
        if silent_unavailable {
            for axis in &axes {
                if unavailable.contains(&axis.axis.joint.as_str()) {
                    bus.kill(axis.axis.binding.bus_id);
                }
            }
        }
        let profile = guidance.map(test_guidance_profile);
        let controller = Controller::with_guidance(
            body,
            model,
            Config {
                joints: match guidance {
                    Some(GuidanceCase::Coupled) => vec!["left_elbow_pitch_joint".to_owned()],
                    Some(_) => vec![SELECTED.to_owned(), "left_upper_arm_yaw_joint".to_owned()],
                    None => vec![SELECTED.to_owned()],
                },
                grippers,
                held: held_torso.map_or_else(BTreeMap::new, |q| {
                    BTreeMap::from([("torso_joint".into(), q)])
                }),
                speed: 0.2,
                share: 0.25,
                damping: 0.5,
                period: Duration::from_millis(10),
                max_seconds: 60.0,
            },
            profile,
        )
        .expect("controller preflight");
        let selected_motor = &axes[selected];
        Self {
            controller,
            dispatched: Instant::now(),
            bus: bus.clone(),
            selected,
            id: selected_motor.axis.binding.bus_id,
            gain: selected_motor
                .axis
                .actuator_from_joint(selected_motor.axis.zero + 1.0),
            kp: selected_motor.kp,
            tmax: bus.state().motors[&selected_motor.axis.binding.bus_id].limits[2],
            temperature,
            next_hold_delay_ms,
            activation,
            isolated_ids: axes
                .iter()
                .filter(|a| extra_unavailable.contains(&a.axis.joint.as_str()))
                .map(|a| a.axis.binding.bus_id)
                .collect(),
            torso_id: axes
                .iter()
                .find(|a| a.axis.joint == "torso_joint")
                .expect("torso")
                .axis
                .binding
                .bus_id,
        }
    }

    fn tick(&mut self, action: Option<Action>) -> sx_embodiment_commission::teach::Snapshot {
        if self.controller.mode() == Mode::Hold && action == Some(Action::Replay) {
            self.wait_engagement();
        }
        std::thread::sleep(Duration::from_millis(10));
        self.dispatched = Instant::now();
        self.controller.tick(action).expect("controller tick")
    }

    /// The farthest one commanded step may move at `speed`: the governor bounds each step by
    /// its own interval since the previous one, capped at [`MAX_STEP_INTERVAL`]. That interval
    /// is at most the time from just before the previous tick (`since`) until now, so a slow
    /// tick on a loaded host widens the allowance exactly as it widened the governor's step.
    fn step_allowance(speed: f64, since: Instant) -> f64 {
        speed * since.elapsed().min(MAX_STEP_INTERVAL).as_secs_f64()
    }

    fn wait_engagement(&mut self) {
        let began = Instant::now();
        while !self.controller.engagement_ready() {
            assert!(
                began.elapsed() < Duration::from_secs(2),
                "engagement did not complete"
            );
            std::thread::sleep(Duration::from_millis(10));
            self.controller.tick(None).expect("engagement ramp tick");
        }
        std::thread::sleep(Duration::from_millis(10));
        self.controller.tick(None).expect("ready engagement tick");
    }

    fn wait_teaching_ready(&mut self) {
        let began = Instant::now();
        while !self.controller.teaching_ready() {
            assert!(
                began.elapsed() < Duration::from_secs(4),
                "support did not settle"
            );
            std::thread::sleep(Duration::from_millis(10));
            self.controller.tick(None).expect("settled support tick");
        }
    }

    fn support_torque(&self) -> f64 {
        let bytes = self.frame();
        let code = (u16::from(bytes[6] & 15) << 8) | u16::from(bytes[7]);
        (f64::from(code) / 4095.0 * 2.0 * self.tmax - self.tmax) * self.gain
    }

    fn frame(&self) -> [u8; 8] {
        self.bus
            .state()
            .frames
            .iter()
            .rev()
            .find(|frame| frame.id == u32::from(self.id) && frame.data[..7] != [0xff; 7])
            .expect("MIT frame")
            .data
    }

    fn episode(&self, position: Vec<f64>) -> Episode {
        let width = position.len();
        Episode {
            header: Header::new("a".repeat(64), self.controller.joints(), 0.01).expect("header"),
            samples: vec![
                Sample {
                    t_s: 0.0,
                    position: position.clone(),
                    velocity: vec![0.0; width],
                },
                Sample {
                    t_s: 1.0,
                    position,
                    velocity: vec![0.0; width],
                },
            ],
        }
    }
}

fn kp(bytes: [u8; 8]) -> u16 {
    (u16::from(bytes[3] & 15) << 8) | u16::from(bytes[4])
}

fn test_guidance_profile(case: GuidanceCase) -> GuidanceProfile {
    let source = "<robot name='guidance'><link name='base'/><link name='payload'><inertial><origin xyz='0.2 0 0'/><mass value='0.4'/><inertia ixx='0.001' ixy='0' ixz='0' iyy='0.001' iyz='0' izz='0.001'/></inertial></link><link name='empty'/><joint name='left_shoulder_pitch_joint' type='revolute'><parent link='base'/><child link='payload'/><axis xyz='0 1 0'/><limit lower='-10' upper='10' effort='100' velocity='5'/></joint><joint name='left_upper_arm_yaw_joint' type='revolute'><parent link='base'/><child link='empty'/><axis xyz='0 1 0'/><limit lower='-10' upper='10' effort='100' velocity='5'/></joint></robot>";
    let source = if case == GuidanceCase::Coupled {
        source.replace(SELECTED, "left_elbow_pitch_joint")
    } else {
        source.to_owned()
    };
    let joint = if case == GuidanceCase::Coupled {
        "left_elbow_pitch_joint"
    } else {
        SELECTED
    };
    let entries: Vec<_> = [
        (joint, -1.0, 0.3, 0.45),
        ("left_upper_arm_yaw_joint", 1.0, 0.0, 0.01),
    ]
    .into_iter()
    .map(|(joint, gain, offset, damping)| {
        serde_json::json!({
            "joint":joint, "coordinate_gain":gain, "coordinate_offset":offset,
            "gravity_scale":0.95, "damping":damping, "friction_coulomb":0.8125,
            "friction_scale":0.0,
            "torque_limit":if case == GuidanceCase::TightCap { Some(0.1) } else { None },
            "torque_slew_rate":20.0
        })
    })
    .collect();
    let document = serde_json::json!({"schema":"sx.guidance/v1", "description_sha256":"a".repeat(64),
        "drive_sha256":"b".repeat(64), "gravity":[0.0,0.0,-9.81], "model_urdf":source,"joints":entries});
    GuidanceProfile::parse(&document.to_string(), &"a".repeat(64), &"b".repeat(64))
        .expect("guidance")
}

#[test]
fn guidance_profile_uses_joint_damping_and_slews_separate_gravity_without_changing_hold() {
    let mut rig = Rig::configured_profile(
        false,
        false,
        Vec::new(),
        None,
        [0.0, 1.0, 0.0],
        false,
        &[],
        Some(GuidanceCase::Distinct),
    );
    rig.tick(Some(Action::Engage));
    rig.wait_engagement();
    let before = rig.support_torque();
    rig.tick(Some(Action::Teach));
    assert_eq!(rig.controller.mode(), Mode::Teach);
    assert!(
        (rig.support_torque() - before).abs() < 0.03,
        "Teach entry must retain completed Hold support"
    );
    let kd_code = |bytes: [u8; 8]| (u16::from(bytes[5]) << 4) | u16::from(bytes[6] >> 4);
    assert!(
        kd_code(rig.frame())
            .abs_diff(float_to_uint(0.45 / (rig.gain * rig.gain), 0.0, 5.0, 12).unwrap())
            <= 1
    );
    let second = rig
        .bus
        .state()
        .frames
        .iter()
        .rev()
        .find(|frame| frame.id == 0x1a && frame.data[..7] != [0xff; 7])
        .expect("second selected joint")
        .data;
    assert_eq!(kp(second), 0);
    assert!(kd_code(second).abs_diff(float_to_uint(0.01, 0.0, 5.0, 12).unwrap()) <= 1);
    let until = Instant::now() + Duration::from_millis(2100);
    let mut snapshot = rig.tick(None);
    while Instant::now() < until {
        snapshot = rig.tick(None);
    }
    let desired = 0.95 * 0.7848 * (-snapshot.position[rig.selected] + 0.3).cos();
    assert!(
        (rig.support_torque() - desired).abs() < 0.03,
        "affine guidance gravity must replace the old model"
    );
    let guided = rig.support_torque();
    rig.tick(Some(Action::Hold));
    assert!(kp(rig.frame()) > 0, "normal stiffness must return");
    assert!(
        (rig.support_torque() - guided).abs() < 0.03,
        "Hold entry must retain completed guidance support"
    );
    let until = Instant::now() + Duration::from_millis(2100);
    while Instant::now() < until {
        snapshot = rig.tick(None);
    }
    let legacy = -0.1962 * snapshot.position[rig.selected].cos();
    assert!(
        (rig.support_torque() - legacy).abs() < 0.03,
        "Hold must return to the unchanged legacy model"
    );
    let expected_kd = float_to_uint(1.091, 0.0, 5.0, 12).unwrap();
    assert!(
        kd_code(rig.frame()).abs_diff(expected_kd) <= 1,
        "normal position damping must return"
    );
}

#[test]
fn guiding_elbow_refreshes_held_shoulder_gravity_at_each_measured_pose() {
    let mut rig = Rig::configured_profile(
        false,
        false,
        Vec::new(),
        None,
        [0.0, 1.0, 0.0],
        false,
        &[],
        Some(GuidanceCase::Coupled),
    );
    rig.tick(Some(Action::Engage));
    rig.wait_engagement();
    let snapshot = rig.tick(Some(Action::Teach));
    assert!(
        kp(rig.frame()) > 0,
        "the nonselected shoulder remains position-held"
    );
    let before = rig.controller.diagnostics()[rig.selected]
        .completed_gravity_command_nm
        .unwrap();
    let elbow = rig
        .controller
        .joints()
        .iter()
        .position(|joint| joint == "left_elbow_pitch_joint")
        .unwrap();
    let mut last = snapshot;
    for _ in 0..100 {
        rig.bus.state().motors.get_mut(&0x1b).unwrap().q += 0.004;
        last = rig.tick(None);
    }
    let shoulder_q = last.position[rig.selected];
    let elbow_q = last.position[elbow];
    let expected = -0.1962 * shoulder_q.cos() - 0.7848 * (shoulder_q + elbow_q).cos();
    let completed = rig.controller.diagnostics()[rig.selected]
        .completed_gravity_command_nm
        .unwrap();
    assert!(
        (completed - expected).abs() < 1e-10,
        "held shoulder uses current coupled model: {completed} vs {expected}"
    );
    assert!(
        (completed - before).abs() > 0.1,
        "elbow movement must change upstream support"
    );
    assert!((rig.support_torque() - expected).abs() <= 2.0 * rig.tmax / 4095.0);
    assert!(kp(rig.frame()) > 0);
}

#[test]
fn guidance_hard_cap_bounds_entry_support_before_slew_and_does_not_limit_hold() {
    let mut rig = Rig::configured_profile(
        false,
        false,
        Vec::new(),
        None,
        [0.0, 1.0, 0.0],
        false,
        &[],
        Some(GuidanceCase::TightCap),
    );
    rig.tick(Some(Action::Engage));
    rig.wait_engagement();
    assert!(
        rig.support_torque() < -0.15,
        "normal Hold support exceeds the tighter Teach cap"
    );
    rig.tick(Some(Action::Teach));
    let first = rig.controller.diagnostics()[rig.selected]
        .completed_gravity_command_nm
        .unwrap();
    assert!(
        first.abs() <= 0.1,
        "hard cap overrides inherited out-of-cap Hold torque"
    );
    let quantum = 2.0 * rig.tmax / 4095.0;
    assert!(rig.support_torque().abs() <= 0.1 + quantum);
    let mut previous = first;
    let mut previous_at = rig.controller.diagnostics()[rig.selected]
        .gravity_command_completed_at
        .unwrap();
    for _ in 0..30 {
        rig.tick(None);
        let diagnostic = rig.controller.diagnostics()[rig.selected];
        let torque = diagnostic.completed_gravity_command_nm.unwrap();
        let at = diagnostic.gravity_command_completed_at.unwrap();
        assert!(torque.abs() <= 0.1);
        assert!(
            (torque - previous).abs() <= 20.0 * at.duration_since(previous_at).as_secs_f64() + 0.01,
            "after entry, ordinary slew applies inside the cap"
        );
        previous = torque;
        previous_at = at;
    }
    rig.tick(Some(Action::Hold));
    let until = Instant::now() + Duration::from_millis(2100);
    while Instant::now() < until {
        rig.tick(None);
    }
    assert!(
        rig.support_torque() < -0.15,
        "Teach cap must not change normal Hold bounds"
    );
    assert!(kp(rig.frame()) > 0);
}

#[test]
fn hand_guidance_uses_measured_gravity_and_hold_restores_position_stiffness() {
    let mut rig = Rig::new();
    assert_eq!(rig.controller.mode(), Mode::Disarmed);
    rig.tick(Some(Action::Engage));
    rig.wait_engagement();
    let snapshot = rig.tick(Some(Action::Teach));
    assert_eq!(snapshot.mode, Mode::Teach);
    let frame = rig.frame();
    assert_eq!(kp(frame), 0);
    let torque_code = (u16::from(frame[6] & 15) << 8) | u16::from(frame[7]);
    let gravity = -0.1962 * snapshot.position[rig.selected].cos();
    let expected =
        float_to_uint(gravity / rig.gain, -rig.tmax, rig.tmax, 12).expect("gravity code");
    assert!(
        torque_code.abs_diff(expected) <= 1,
        "gravity support differs beyond one MIT code"
    );
    let held = rig.tick(Some(Action::Hold));
    assert_eq!(held.mode, Mode::Hold);
    assert_eq!(
        kp(rig.frame()),
        float_to_uint(rig.kp, 0.0, 500.0, 12).expect("configured stiffness")
    );
    assert_eq!(rig.tick(None).mode, Mode::Hold);
}

#[test]
fn stop_is_terminal_and_never_reengages_from_a_keyboard_engage() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    rig.tick(Some(Action::Teach));
    assert!(rig.controller.tick(Some(Action::Stop)).is_err());
    assert_eq!(rig.controller.mode(), Mode::Fault);
    let held = rig.frame();
    assert!(kp(held) > 0);
    assert_eq!(rig.tick(Some(Action::Engage)).mode, Mode::Fault);
    assert_eq!(rig.tick(Some(Action::Teach)).mode, Mode::Fault);
    assert!(kp(rig.frame()) > 0);
}

#[test]
fn explicit_operator_exit_disables_the_real_driver_from_every_operating_mode() {
    for initial in [
        Mode::Disarmed,
        Mode::Hold,
        Mode::Teach,
        Mode::Fault,
        Mode::Replay,
    ] {
        let mut rig = Rig::new();
        if initial != Mode::Disarmed {
            let held = rig.tick(Some(Action::Engage));
            match initial {
                Mode::Teach => {
                    rig.tick(Some(Action::Teach));
                }
                Mode::Fault => rig.controller.fault("test fault before explicit exit"),
                Mode::Replay => {
                    rig.controller
                        .set_episode(rig.episode(held.position))
                        .unwrap();
                    rig.tick(None);
                    rig.tick(Some(Action::Replay));
                    for _ in 0..40 {
                        if rig.tick(None).mode == Mode::ReadyReplay {
                            break;
                        }
                    }
                    assert_eq!(rig.tick(Some(Action::Replay)).mode, Mode::Replay);
                }
                _ => {}
            }
        }
        assert_eq!(rig.controller.mode(), initial);
        let before = rig.bus.state().frames.len();
        assert_eq!(rig.tick(Some(Action::Exit)).mode, Mode::Released);
        let state = rig.bus.state();
        assert!(state.motors.values().all(|motor| motor.state == 0));
        assert!(
            state.frames[before..]
                .iter()
                .any(|frame| frame.data == [255, 255, 255, 255, 255, 255, 255, 253])
        );
    }
}

#[test]
fn explicit_exit_does_not_claim_release_when_a_motor_cannot_acknowledge_disable() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    rig.tick(Some(Action::Teach));
    rig.bus.kill(rig.id);
    let error = rig.controller.tick(Some(Action::Exit)).unwrap_err();
    assert!(error.contains("release unproven"), "{error}");
    assert_eq!(rig.controller.mode(), Mode::Fault);
    assert_eq!(rig.bus.state().motors[&rig.id].state, 1);
    rig.bus.state().dead.remove(&rig.id);
    assert_eq!(rig.tick(Some(Action::Exit)).mode, Mode::Released);
    assert!(
        rig.bus
            .state()
            .motors
            .values()
            .all(|motor| motor.state == 0)
    );
}

#[test]
fn incompatible_optional_gripper_maps_do_not_block_arm_teaching() {
    use sx_embodiment_ik::teleop::Gripper;

    for fingers in [
        vec![(usize::MAX, 0.0, 1.0)],
        vec![(0, 0.0, 1.0)],
        Vec::new(),
    ] {
        let mut rig = Rig::configured(
            false,
            false,
            vec![Gripper {
                fingers,
                closed_width_m: 0.0,
                open_width_m: 0.1,
            }],
            None,
        );
        assert_eq!(rig.controller.mode(), Mode::Disarmed);
        rig.tick(Some(Action::Engage));
        assert_eq!(rig.tick(Some(Action::Teach)).mode, Mode::Teach);
        assert_eq!(kp(rig.frame()), 0);
        rig.tick(Some(Action::Hold));
        let before = rig.bus.state().motors[&rig.id].q;
        // Optional gripper controls still refuse an incompatible map before it can
        // index an absent finger or send a gripper target to an arm joint.
        assert!(
            rig.controller
                .tick(Some(Action::GripperStep(0.05)))
                .is_err()
        );
        assert_eq!(rig.controller.mode(), Mode::Fault);
        assert!((rig.bus.state().motors[&rig.id].q - before).abs() < 1e-3);
        assert!(kp(rig.frame()) > 0);
    }
}

#[test]
fn missed_motor_reply_faults_without_releasing_the_answering_joints() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    rig.tick(Some(Action::Teach));
    let other = rig
        .bus
        .state()
        .motors
        .keys()
        .copied()
        .find(|id| *id != rig.id)
        .expect("other joint");
    rig.bus.kill(other);
    std::thread::sleep(Duration::from_millis(10));
    assert!(rig.controller.tick(None).is_err());
    assert_eq!(rig.controller.mode(), Mode::Fault);
    assert!(kp(rig.frame()) > 0, "the answering arm remains held");
    assert_eq!(rig.bus.state().motors[&rig.id].state, 1);
}

#[test]
fn a_stalled_control_loop_faults_into_hold_instead_of_resuming_teach() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    rig.tick(Some(Action::Teach));
    std::thread::sleep(Duration::from_millis(60));
    assert!(
        rig.controller
            .tick(None)
            .expect_err("deadline")
            .contains("50 ms")
    );
    assert_eq!(rig.controller.mode(), Mode::Fault);
    assert!(kp(rig.frame()) > 0);
    assert_eq!(rig.tick(Some(Action::Teach)).mode, Mode::Fault);
}

#[test]
fn replay_approaches_then_waits_for_a_second_explicit_play_command() {
    let mut rig = Rig::new();
    let held = rig.tick(Some(Action::Engage));
    rig.controller
        .set_episode(rig.episode(held.position))
        .expect("episode");
    rig.tick(None);
    assert!(rig.controller.replay_ready());
    assert_eq!(rig.tick(Some(Action::Replay)).mode, Mode::Approach);
    for _ in 0..40 {
        if rig.tick(None).mode == Mode::ReadyReplay {
            break;
        }
    }
    assert_eq!(rig.controller.mode(), Mode::ReadyReplay);
    assert_eq!(
        rig.tick(None).mode,
        Mode::ReadyReplay,
        "arrival must not auto-play"
    );
    assert_eq!(rig.tick(Some(Action::Replay)).mode, Mode::Replay);
    rig.tick(Some(Action::Hold));
}

#[test]
fn replay_rejects_wrong_joint_order_and_unsafe_positions_before_enable() {
    let mut rig = Rig::new();
    let mut position = rig.tick(None).position;
    let mut wrong = rig.episode(position.clone());
    wrong.header.joints.swap(0, 1);
    assert!(rig.controller.set_episode(wrong).is_err());
    position[rig.selected] = 100.0;
    rig.controller
        .set_episode(rig.episode(position))
        .expect("queued validation");
    assert!(
        rig.controller
            .tick(None)
            .expect_err("unsafe replay")
            .contains("out of range")
    );
    assert_eq!(rig.controller.mode(), Mode::Fault);
    assert!(rig.bus.state().motors.values().all(|m| m.state == 0));
}

#[test]
fn teaching_uses_full_platform_range_and_still_stops_outside_bounds() {
    let bindings = STATIC_UPPER_BODY.axes(&drive_for_test()).unwrap();
    let axis = &bindings
        .iter()
        .find(|axis| axis.axis.joint == SELECTED)
        .unwrap()
        .axis;
    for upper in [false, true] {
        let bound = if upper { axis.upper } else { axis.lower };
        let inward = if upper { -1.0 } else { 1.0 };
        let mut rig = Rig::new();
        rig.bus.state().motors.get_mut(&rig.id).unwrap().q =
            axis.actuator_from_joint(bound + inward * 0.01);
        rig.tick(Some(Action::Engage));
        assert_eq!(rig.tick(Some(Action::Teach)).mode, Mode::Teach);
        assert_eq!(kp(rig.frame()), 0);
        rig.bus.state().motors.get_mut(&rig.id).unwrap().q =
            axis.actuator_from_joint(bound + inward * 0.002);
        assert_eq!(rig.tick(None).mode, Mode::Teach);
        assert_eq!(kp(rig.frame()), 0);
        rig.bus.state().motors.get_mut(&rig.id).unwrap().q =
            axis.actuator_from_joint(bound - inward * 0.002);
        let reason = rig
            .controller
            .tick(None)
            .expect_err("outside platform range");
        assert!(reason.contains("outside its command range"), "{reason}");
        assert_eq!(rig.controller.mode(), Mode::Fault);
    }
}

#[test]
fn hand_motion_over_the_observed_speed_limit_stops_before_another_compliant_frame() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    rig.tick(Some(Action::Teach));
    std::thread::sleep(Duration::from_millis(10));
    // Change the encoder, not v_des: the default simulated motor reports zero planned
    // velocity, so the independent measured-position derivative must catch this.
    rig.bus.state().motors.get_mut(&rig.id).expect("motor").q += rig.gain * 0.6;
    let reason = rig
        .controller
        .tick(None)
        .expect_err("too fast by measured encoder motion");
    assert!(reason.contains("observed speed limit"), "{reason}");
    assert_eq!(rig.controller.mode(), Mode::Fault);
    assert!(kp(rig.frame()) > 0);
}

#[test]
fn isolated_left_teaches_and_replays_without_enabling_the_unavailable_right_arm() {
    let right = [
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_upper_arm_yaw_joint",
        "right_elbow_pitch_joint",
        "right_forearm_yaw_joint",
        "right_wrist_pitch_joint",
        "right_wrist_roll_joint",
    ];
    for silent in [false, true] {
        let mut rig = Rig::configured_isolated(
            true,
            false,
            Vec::new(),
            None,
            [0.0, 0.0, 1.0],
            silent,
            &right,
        );
        let names = rig.controller.joints();
        assert_eq!(names.len(), 7);
        assert!(names.iter().all(|n| n.starts_with("left_")));
        rig.tick(Some(Action::Engage));
        assert_eq!(rig.tick(Some(Action::Teach)).mode, Mode::Teach);
        assert_eq!(kp(rig.frame()), 0);
        let held = rig.tick(Some(Action::Hold));
        let episode = rig.episode(held.position);
        assert!(episode.header.joints.iter().all(|n| n.starts_with("left_")));
        rig.controller
            .set_episode(episode)
            .expect("isolated episode");
        rig.tick(None);
        rig.tick(Some(Action::Replay));
        for _ in 0..40 {
            if rig.tick(None).mode == Mode::ReadyReplay {
                break;
            }
        }
        assert_eq!(rig.controller.mode(), Mode::ReadyReplay);
        assert_eq!(rig.tick(Some(Action::Replay)).mode, Mode::Replay);
        assert!(rig.controller.tick(Some(Action::Stop)).is_err());
        let state = rig.bus.state();
        for id in &rig.isolated_ids {
            assert_eq!(state.motors[id].state, 0, "isolated motor stays disabled");
            assert!(
                state
                    .frames
                    .iter()
                    .filter(|f| f.id == u32::from(*id))
                    .all(|f| f.data == [255, 255, 255, 255, 255, 255, 255, 253]),
                "isolated motor receives no enable or MIT command"
            );
        }
    }
}

#[test]
fn missing_vertical_yaw_feedback_does_not_block_arm_teaching() {
    for silent_at_start in [false, true] {
        let mut rig = Rig::configured_with_torso_sensor(
            true,
            false,
            Vec::new(),
            None,
            [0.0, 0.0, 1.0],
            silent_at_start,
        );
        rig.tick(Some(Action::Engage));
        rig.tick(Some(Action::Teach));
        rig.bus.kill(rig.torso_id);
        for _ in 0..10 {
            std::thread::sleep(Duration::from_millis(10));
            assert_eq!(rig.tick(None).mode, Mode::Teach);
            assert_eq!(kp(rig.frame()), 0);
        }
        assert_eq!(rig.bus.state().motors[&rig.torso_id].state, 0);
        assert!(!rig.controller.joints().iter().any(|n| n == "torso_joint"));
        // The exception is only for passive gravity-independent axes: arm loss faults.
        rig.bus.kill(rig.id);
        assert!(rig.controller.tick(None).is_err());
        assert_eq!(rig.controller.mode(), Mode::Fault);
    }
}

#[test]
fn a_passive_torso_uses_live_feedback_without_a_historical_fixed_reference() {
    let mut rig = Rig::configured(true, false, Vec::new(), None);
    rig.tick(Some(Action::Engage));
    rig.tick(Some(Action::Teach));
    assert!(!rig.controller.joints().iter().any(|n| n == "torso_joint"));
    for _ in 0..30 {
        std::thread::sleep(Duration::from_millis(10));
        rig.bus
            .state()
            .motors
            .get_mut(&rig.torso_id)
            .expect("torso")
            .q += 0.001;
        assert_eq!(rig.tick(None).mode, Mode::Teach);
    }
    assert_eq!(
        rig.bus.state().motors[&rig.torso_id].state,
        0,
        "passive torso stays disabled"
    );
    rig.bus.kill(rig.torso_id);
    std::thread::sleep(Duration::from_millis(10));
    let reason = rig
        .controller
        .tick(None)
        .expect_err("live torso feedback is required");
    assert!(
        reason.contains("torso_joint has no fresh passive-joint reading"),
        "{reason}"
    );
    assert_eq!(rig.controller.mode(), Mode::Fault);
    assert!(kp(rig.frame()) > 0);
}

#[test]
fn a_mechanically_fixed_out_of_service_axis_cannot_drift_during_teaching() {
    let mut rig = Rig::configured(true, false, Vec::new(), Some(0.0));
    rig.tick(Some(Action::Engage));
    rig.tick(Some(Action::Teach));
    assert!(
        !rig.controller
            .joints()
            .iter()
            .any(|name| name == "torso_joint")
    );
    let mut stopped = None;
    for _ in 0..30 {
        std::thread::sleep(Duration::from_millis(10));
        rig.bus
            .state()
            .motors
            .get_mut(&rig.torso_id)
            .expect("torso")
            .q += 0.001;
        if let Err(reason) = rig.controller.tick(None) {
            stopped = Some(reason);
            break;
        }
    }
    let reason = stopped.expect("fixed torso drift must stop teaching");
    assert!(reason.contains("fixed torso_joint moved"), "{reason}");
    assert_eq!(rig.controller.mode(), Mode::Fault);
    assert!(kp(rig.frame()) > 0);
}

#[test]
fn thermal_margin_holds_then_hard_thermal_limit_releases_only_the_hot_motor() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    rig.tick(Some(Action::Teach));
    rig.temperature.store(90, Ordering::Relaxed);
    let reason = rig.controller.tick(None).expect_err("thermal margin");
    assert!(reason.contains("temperature margin"), "{reason}");
    assert_eq!(rig.controller.mode(), Mode::Fault);
    assert!(kp(rig.frame()) > 0);
    assert_eq!(rig.bus.state().motors[&rig.id].state, 1);
    rig.temperature.store(100, Ordering::Relaxed);
    let _ = rig.controller.tick(None);
    assert_eq!(
        rig.bus.state().motors[&rig.id].state,
        0,
        "thermal limit releases the hot motor"
    );
    assert!(
        rig.bus
            .state()
            .motors
            .iter()
            .filter(|(id, _)| **id != rig.id)
            .all(|(_, motor)| motor.state == 1)
    );
    assert_eq!(rig.controller.mode(), Mode::Fault);
}

#[test]
fn a_silent_held_motor_is_not_released_on_one_miss_but_is_released_after_300_ms() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    rig.tick(Some(Action::Teach));
    rig.bus.kill(rig.id);
    let _ = rig.controller.tick(None);
    assert_eq!(rig.controller.mode(), Mode::Fault);
    // First fault-mode tick starts the continuous-silence timer.
    let _ = rig.controller.tick(None);
    let before = rig.bus.state().frames.len();
    std::thread::sleep(Duration::from_millis(40));
    let _ = rig.controller.tick(None);
    let still_holds = rig.bus.state().frames[before..]
        .iter()
        .any(|f| f.id == u32::from(rig.id) && f.data[..7] != [0xff; 7]);
    assert!(still_holds, "one brief silence must not drop the arm");
    let before = rig.bus.state().frames.len();
    std::thread::sleep(Duration::from_millis(310));
    let _ = rig.controller.tick(None);
    let disabled = rig.bus.state().frames[before..].iter().any(|f| {
        f.id == u32::from(rig.id) && f.data == [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd]
    });
    assert!(disabled, "sustained silence sends the motor's disable");
    assert!(
        rig.bus
            .state()
            .motors
            .iter()
            .filter(|(id, _)| **id != rig.id)
            .all(|(_, motor)| motor.state == 1)
    );
    assert_eq!(rig.controller.mode(), Mode::Fault);
}

#[test]
fn engagement_starts_without_gravity_step_and_ramps_while_held() {
    let mut rig = Rig::new();
    let held = rig.tick(Some(Action::Engage));
    assert_eq!(held.mode, Mode::Hold);
    assert!(!rig.controller.engagement_ready());
    let quantum = 2.0 * rig.tmax * rig.gain.abs() / 4095.0;
    assert!(
        rig.support_torque().abs() <= quantum,
        "first hold must send zero gravity support"
    );
    assert!(kp(rig.frame()) > 0, "engagement begins in stiff hold");
    let began = Instant::now();
    while began.elapsed() < Duration::from_millis(400) {
        rig.tick(None);
    }
    let intermediate = rig.support_torque();
    let full = -0.1962 * held.position[rig.selected].cos();
    assert!(
        intermediate.abs() > quantum && intermediate.abs() < full.abs() - quantum,
        "support must pass through an intermediate torque: {intermediate} vs {full}"
    );
    assert_eq!(rig.controller.mode(), Mode::Hold);
    rig.wait_engagement();
    assert_eq!(rig.controller.mode(), Mode::Hold);
    rig.tick(Some(Action::Teach));
    assert_eq!(rig.controller.mode(), Mode::Teach);
    assert_eq!(kp(rig.frame()), 0);
    assert!((rig.support_torque() - full).abs() <= quantum);
}

#[test]
fn teaching_immediately_after_engagement_does_not_require_a_settled_support_window() {
    let mut rig = Rig::new();
    assert_eq!(
        rig.controller
            .tick(Some(Action::Engage))
            .expect("engage")
            .mode,
        Mode::Hold
    );
    assert!(!rig.controller.engagement_ready());
    assert!(!rig.controller.teaching_ready());
    let teaching = rig
        .controller
        .tick(Some(Action::Teach))
        .expect("explicit teach");
    assert_eq!(teaching.mode, Mode::Teach);
    assert_eq!(kp(rig.frame()), 0);
    assert!(rig.controller.engagement_ready());
    let support = rig.support_torque();
    assert_eq!(rig.tick(Some(Action::Hold)).mode, Mode::Hold);
    assert!((rig.support_torque() - support).abs() <= 2.0 * rig.tmax / 4095.0);
    assert!(
        rig.controller.diagnostics()[rig.selected]
            .support_offset_nm
            .expect("offset")
            .abs()
            < 1e-12
    );
}

#[test]
fn premature_replay_never_starts_motion_automatically() {
    let mut rig = Rig::new();
    let held = rig.tick(Some(Action::Engage));
    rig.controller
        .set_episode(rig.episode(held.position))
        .expect("episode");
    std::thread::sleep(Duration::from_millis(10));
    assert_eq!(
        rig.controller
            .tick(Some(Action::Replay))
            .expect("wait before replay")
            .mode,
        Mode::Hold
    );
    std::thread::sleep(Duration::from_millis(10));
    rig.controller
        .tick(Some(Action::Hold))
        .expect("explicit hold");
    rig.wait_engagement();
    assert_eq!(
        rig.tick(None).mode,
        Mode::Hold,
        "premature replay request must not start replay"
    );
    assert!(kp(rig.frame()) > 0);
    assert_eq!(rig.tick(Some(Action::Replay)).mode, Mode::Approach);
}

#[test]
fn fault_during_engagement_keeps_last_scaled_support_instead_of_applying_full_gravity() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    let began = Instant::now();
    while began.elapsed() < Duration::from_millis(350) {
        rig.tick(None);
    }
    let before = rig.support_torque();
    let quantum = 2.0 * rig.tmax * rig.gain.abs() / 4095.0;
    assert!(before.abs() > quantum && before.abs() < 0.15);
    std::thread::sleep(Duration::from_millis(10));
    assert!(rig.controller.tick(Some(Action::Stop)).is_err());
    assert_eq!(rig.controller.mode(), Mode::Fault);
    assert!(!rig.controller.engagement_ready());
    assert!(
        (rig.support_torque() - before).abs() <= quantum,
        "fault must retain scaled support, not jump to full gravity"
    );
    assert_eq!(rig.tick(Some(Action::Teach)).mode, Mode::Fault);
    assert!((rig.support_torque() - before).abs() <= quantum);
}

#[test]
fn blocking_hold_pauses_engagement_ramp_without_a_support_jump() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    let began = Instant::now();
    while began.elapsed() < Duration::from_millis(200) {
        rig.tick(None);
    }
    let applied = |rig: &Rig| {
        rig.controller.diagnostics()[rig.selected]
            .completed_gravity_command_nm
            .expect("applied support")
    };
    let before = applied(&rig);
    let previous = rig.dispatched;
    rig.next_hold_delay_ms.store(300, Ordering::Relaxed);
    let began = Instant::now();
    let held = rig.tick(Some(Action::Hold));
    assert!(began.elapsed() >= Duration::from_millis(300));
    assert_eq!(held.mode, Mode::Hold);
    assert!(!rig.controller.engagement_ready());
    // The first command after the block is where a catch-up would land. It is checked in the
    // controller's own unquantised support: the full support is only about seven 12-bit torque
    // codes, so encoded frames flicker by a code from tick to tick and cannot resolve the ramp.
    // The ramp still advances by the unblocked part of the interval (two ordinary ticks,
    // longer on a loaded host) over its 1 s length; catching up the 300 ms block adds 30 %.
    rig.tick(None);
    let unblocked = previous
        .elapsed()
        .saturating_sub(Duration::from_millis(300));
    let full = rig.controller.diagnostics()[rig.selected]
        .calculated_gravity_nm
        .expect("full model support");
    let after = applied(&rig);
    assert!(
        (after - before).abs() <= full.abs() * unblocked.as_secs_f64() + 1e-3,
        "blocking hold must not catch up the support ramp in one frame: {before} -> {after}, unblocked {unblocked:?}, full {full}"
    );
    let resumed = Instant::now();
    while resumed.elapsed() < Duration::from_millis(550) {
        assert_eq!(rig.tick(None).mode, Mode::Hold);
    }
    assert!(
        !rig.controller.engagement_ready(),
        "blocking time must not count toward support ramp completion"
    );
    rig.wait_engagement();
    assert_eq!(
        rig.controller.mode(),
        Mode::Hold,
        "H must not start teaching"
    );
}

#[test]
fn slow_activation_is_separate_from_cycles_and_first_stiff_hold_uses_fresh_position() {
    let mut rig = Rig::new();
    let initial = rig.bus.state().motors[&rig.id].q;
    {
        let mut bus = rig.bus.state();
        let motor = bus.motors.get_mut(&rig.id).expect("motor");
        let mut dynamics = sx_damiao_can::sim::MitDynamics::loaded(motor.model);
        dynamics.load = 0.0;
        dynamics.coulomb = 0.0;
        dynamics.damping = 0.0;
        dynamics.inertia = 1.0;
        motor.dynamics = Some(dynamics);
    }
    rig.activation.enable_delay_ms.store(70, Ordering::Relaxed);
    rig.activation
        .encoder_shift_bits
        .store(0.002_f64.to_bits(), Ordering::Relaxed);
    let before = rig.bus.state().frames.len();
    let began = Instant::now();
    let held = rig.tick(Some(Action::Engage));
    assert!(began.elapsed() >= Duration::from_millis(70));
    assert_eq!(held.mode, Mode::Hold);
    assert!(!rig.controller.engagement_ready());
    let frames: Vec<_> = rig.bus.state().frames[before..]
        .iter()
        .filter(|f| f.id == u32::from(rig.id) && f.data[..7] != [0xff; 7])
        .copied()
        .collect();
    assert!(frames.len() >= 2);
    assert_eq!(
        kp(frames[0].data),
        0,
        "activation must not apply a stale position spring"
    );
    assert!(kp(frames[1].data) > 0);
    let code = u16::from_be_bytes([frames[1].data[0], frames[1].data[1]]);
    let pmax = rig.bus.state().motors[&rig.id].limits[0];
    let target = f64::from(code) / 65_535.0 * 2.0 * pmax - pmax;
    assert!(
        (target - (initial + 0.002)).abs() < 2.0 * pmax / 65_535.0,
        "first stiff hold must use the post-enable encoder, not the old pose"
    );
    let quantum = 2.0 * rig.tmax * rig.gain.abs() / 4095.0;
    assert!(rig.support_torque().abs() <= quantum);
    rig.next_hold_delay_ms.store(70, Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(10));
    let reason = rig
        .controller
        .tick(None)
        .expect_err("normal cyclic work must still be bounded");
    assert!(
        reason.contains("Hold control read/compute/write took") && reason.contains("50 ms"),
        "{reason}"
    );
    let dispatch_ms: f64 = reason
        .split("dispatch_ms=")
        .nth(1)
        .expect("dispatch timing")
        .split(',')
        .next()
        .unwrap()
        .parse()
        .unwrap();
    assert!(
        dispatch_ms >= 70.0,
        "injected dispatch delay must be located: {reason}"
    );
    assert!(
        reason.contains("read_ms=") && reason.contains("retries=0"),
        "{reason}"
    );
    assert_eq!(rig.controller.mode(), Mode::Fault);
}

#[test]
fn initial_activation_answers_can_age_within_startup_but_stiff_hold_gets_new_readings() {
    let mut rig = Rig::new();
    rig.activation.reply_delay_ms.store(70, Ordering::Relaxed);
    assert_eq!(rig.tick(Some(Action::Engage)).mode, Mode::Hold);
    assert!(!rig.controller.engagement_ready());
    assert_eq!(rig.tick(None).mode, Mode::Hold);
}

#[test]
fn activation_has_its_own_one_second_bound() {
    let mut rig = Rig::new();
    rig.activation
        .enable_delay_ms
        .store(1050, Ordering::Relaxed);
    let reason = rig
        .controller
        .tick(Some(Action::Engage))
        .expect_err("activation deadline");
    assert!(
        reason.contains("activation took") && reason.contains("1000 ms startup budget"),
        "{reason}"
    );
    assert_eq!(rig.controller.mode(), Mode::Fault);
}

#[test]
fn missing_post_activation_reading_never_enters_the_normal_hold_cycle() {
    let mut rig = Rig::new();
    rig.activation
        .lose_following_read
        .store(true, Ordering::Relaxed);
    let reason = rig
        .controller
        .tick(Some(Action::Engage))
        .expect_err("fresh post-enable reading required");
    assert!(reason.contains("no fresh fault-free reading"), "{reason}");
    assert_eq!(rig.controller.mode(), Mode::Fault);
    assert!(!rig.controller.engagement_ready());
}

#[test]
fn settled_native_support_replaces_pd_cancellation_without_a_teach_torque_step_or_drift() {
    // Joint N m the simulated plant needs to stay still. The learned support is checked against
    // it, not against one native torque sample: a sample read after a slow tick on a loaded host
    // can be 0.03 N m off while the controller's settled window is not.
    const PLANT_SUPPORT: f64 = 0.1;
    let mut rig = Rig::new();
    {
        let mut state = rig.bus.state();
        let motor = state.motors.get_mut(&rig.id).expect("motor");
        let mut dynamics = sx_damiao_can::sim::MitDynamics::loaded(motor.model);
        // The plant requires PLANT_SUPPORT while the deliberately biased model
        // requests about -0.19 N m. Hold must cancel the mismatch through its spring.
        dynamics.load = -PLANT_SUPPORT / rig.gain;
        dynamics.coulomb = 0.0;
        dynamics.damping = 3.0;
        dynamics.inertia = 0.2;
        motor.dynamics = Some(dynamics);
    }
    rig.tick(Some(Action::Engage));
    rig.wait_teaching_ready();
    let diagnostic = rig.controller.diagnostics()[rig.selected];
    let before = diagnostic.feedback_torque_nm.expect("native support");
    let raw = diagnostic.calculated_gravity_nm.expect("raw model");
    assert!(
        (raw - before).abs() > 0.05,
        "the fixture must exercise a genuine model bias"
    );
    let held = rig.tick(None).position[rig.selected];
    rig.tick(Some(Action::Teach));
    let diagnostic = rig.controller.diagnostics()[rig.selected];
    let corrected = diagnostic
        .completed_gravity_command_nm
        .expect("corrected support");
    let offset = diagnostic.support_offset_nm.expect("session offset");
    let quantum = 2.0 * rig.tmax / 4095.0;
    assert!(
        (corrected - PLANT_SUPPORT).abs() <= quantum + 0.005,
        "removing the spring must retain measured support: {corrected} (last sample {before})"
    );
    assert!((offset - (PLANT_SUPPORT - raw)).abs() <= quantum + 0.005);
    let began = Instant::now();
    while began.elapsed() < Duration::from_millis(500) {
        assert_eq!(rig.tick(None).mode, Mode::Teach);
    }
    let after = rig.tick(None).position[rig.selected];
    assert!(
        (after - held).abs() < 0.003,
        "biased model must not restart autonomous drift: {held} -> {after}"
    );
    let frozen = rig.controller.diagnostics()[rig.selected]
        .support_offset_nm
        .expect("frozen offset");
    rig.tick(Some(Action::Hold));
    rig.wait_teaching_ready();
    rig.tick(Some(Action::Teach));
    let again = rig.controller.diagnostics()[rig.selected]
        .support_offset_nm
        .expect("new absolute residual");
    assert!(
        (again - frozen).abs() <= quantum + 0.01,
        "re-teaching must not double the correction"
    );
}

#[test]
fn slowly_moving_hold_does_not_learn_an_offset_but_can_enter_teach() {
    let mut rig = Rig::new();
    {
        let mut state = rig.bus.state();
        let motor = state.motors.get_mut(&rig.id).expect("motor");
        let mut dynamics = sx_damiao_can::sim::MitDynamics::loaded(motor.model);
        dynamics.inertia = 1_000_000.0;
        dynamics.load = 0.0;
        dynamics.coulomb = 0.0;
        motor.dynamics = Some(dynamics);
    }
    rig.tick(Some(Action::Engage));
    rig.wait_engagement();
    for _ in 0..70 {
        std::thread::sleep(Duration::from_millis(10));
        rig.bus.state().motors.get_mut(&rig.id).expect("motor").q += rig.gain * 0.0002;
        rig.controller
            .tick(None)
            .expect("moving hold remains valid");
        assert_eq!(rig.controller.mode(), Mode::Hold);
    }
    assert!(!rig.controller.teaching_ready());
    assert_eq!(
        rig.controller
            .tick(Some(Action::Teach))
            .expect("unsettled support does not gate teaching")
            .mode,
        Mode::Teach
    );
    assert!(
        rig.controller.diagnostics()[rig.selected]
            .support_offset_nm
            .expect("offset")
            .abs()
            < 1e-12
    );
}

#[test]
fn missing_native_feedback_cannot_qualify_a_support_measurement() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    rig.wait_engagement();
    rig.bus.kill(rig.id);
    assert!(rig.controller.tick(Some(Action::Teach)).is_err());
    assert_eq!(rig.controller.mode(), Mode::Fault);
    assert!(!rig.controller.teaching_ready());
    assert!(
        rig.controller.diagnostics()[rig.selected]
            .support_offset_nm
            .expect("offset")
            .abs()
            < 1e-12
    );
}

#[test]
fn a_frozen_support_offset_still_obeys_effort_bounds_at_each_new_pose() {
    let mut rig = Rig::new();
    // A bounded sensor support near the continuous cap can be accepted at one pose;
    // it must never authorize an out-of-bound corrected support at a later pose.
    let cap = 0.8
        * STATIC_UPPER_BODY.axes(&drive_for_test()).expect("axes")[rig.selected]
            .axis
            .binding
            .model
            .rating()
            .rated_torque_nm
            .expect("continuous rating");
    rig.activation
        .torque_override_bits
        .store((rig.gain * (cap - 0.03)).to_bits(), Ordering::Relaxed);
    rig.tick(Some(Action::Engage));
    rig.wait_teaching_ready();
    rig.tick(Some(Action::Teach));
    let mut stopped = None;
    for _ in 0..600 {
        std::thread::sleep(Duration::from_millis(10));
        rig.bus.state().motors.get_mut(&rig.id).expect("motor").q += rig.gain * 0.001;
        if let Err(reason) = rig.controller.tick(None) {
            stopped = Some(reason);
            break;
        }
    }
    let reason = stopped.expect("changed gravity plus the frozen correction must reach the cap");
    assert!(
        reason.contains("corrected gravity exceeds continuous cap"),
        "{reason}"
    );
    assert_eq!(rig.controller.mode(), Mode::Fault);
}

fn drive_for_test() -> InstanceDrive {
    let drives: Value = serde_json::from_str(DRIVES).expect("drives");
    let nominal = &drives["sentient-static-upper-body"]["nominal"];
    InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("instance"),
    )
    .expect("drive")
}

#[test]
fn manual_teaching_uses_physical_speed_not_replay_speed_or_command_share() {
    let mut rig = Rig::new(); // --speed .2 and --share .25 do not cap hand-guided motion.
    rig.tick(Some(Action::Engage));
    rig.tick(Some(Action::Teach));
    rig.activation
        .velocity_override_bits
        .store((1.01 * rig.gain).to_bits(), Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(10));
    rig.bus.state().motors.get_mut(&rig.id).expect("motor").q += rig.gain * 0.012;
    assert_eq!(
        rig.controller
            .tick(None)
            .expect("manual guidance above 1 rad/s is below this joint physical rating")
            .mode,
        Mode::Teach
    );
    let physical = STATIC_UPPER_BODY.axes(&drive_for_test()).expect("axes")[rig.selected]
        .axis
        .motion
        .speed;
    rig.activation
        .velocity_override_bits
        .store((physical * 1.1 * rig.gain).to_bits(), Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(10));
    let reason = rig
        .controller
        .tick(None)
        .expect_err("native manual speed still has its own bound");
    assert!(
        reason.contains("feedback speed") && reason.contains(&format!("{physical:.6} rad/s")),
        "{reason}"
    );
    assert_eq!(rig.controller.mode(), Mode::Fault);
}

#[test]
fn normal_hold_observes_physical_speed_independently_of_the_command_limit() {
    let mut rig = Rig::new();
    rig.tick(Some(Action::Engage));
    rig.activation
        .velocity_override_bits
        .store((0.4 * rig.gain).to_bits(), Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(10));
    assert_eq!(
        rig.controller
            .tick(None)
            .expect("command cap .2 is not an instantaneous measured-motion cap")
            .mode,
        Mode::Hold
    );
    let physical = STATIC_UPPER_BODY.axes(&drive_for_test()).expect("axes")[rig.selected]
        .axis
        .motion
        .speed;
    rig.activation
        .velocity_override_bits
        .store((physical * 1.1 * rig.gain).to_bits(), Ordering::Relaxed);
    std::thread::sleep(Duration::from_millis(10));
    let reason = rig
        .controller
        .tick(None)
        .expect_err("physical feedback speed boundary remains");
    assert!(
        reason.contains("feedback speed") && reason.contains(&format!("{physical:.6} rad/s")),
        "{reason}"
    );
    assert_eq!(rig.controller.mode(), Mode::Fault);
}

#[test]
fn fast_valid_recordings_keep_their_original_timestamps() {
    let mut rig = Rig::new();
    let position = rig.tick(None).position;
    let mut fast = rig.episode(position.clone());
    fast.samples[1].t_s = 0.01;
    fast.samples[1].position[rig.selected] += 0.02; // 2 rad/s exceeds the .2 approach cap.
    rig.controller
        .set_episode(fast)
        .expect("queued fast recording");
    rig.tick(None);
    assert!(rig.controller.replay_ready());
    assert!((rig.controller.playback_rate() - 1.0).abs() < f64::EPSILON);
    rig.controller
        .set_episode(rig.episode(position))
        .expect("replacement");
    rig.tick(None);
    assert!(rig.controller.replay_ready());
    assert!((rig.controller.playback_rate() - 1.0).abs() < f64::EPSILON);
}

fn commanded_joint_position(rig: &Rig) -> f64 {
    let bytes = rig.frame();
    let code = u16::from_be_bytes([bytes[0], bytes[1]]);
    let qmax = rig.bus.state().motors[&rig.id].limits[0];
    (f64::from(code) / 65535.0 * 2.0 * qmax - qmax) / rig.gain
}

#[test]
fn replay_uses_recorded_time_and_restores_slow_positioning_after_completion() {
    let mut rig = Rig::new();
    let held = rig.tick(Some(Action::Engage));
    let mut episode = rig.episode(held.position.clone());
    episode.samples = (0..=120)
        .map(|i| {
            let t_s = f64::from(i) * 0.01;
            let mut position = held.position.clone();
            position[rig.selected] +=
                0.08 + 0.07 * (1.0 - (std::f64::consts::TAU * t_s / 1.2).cos());
            Sample {
                t_s,
                velocity: vec![0.0; position.len()],
                position,
            }
        })
        .collect();
    rig.controller
        .set_episode(episode)
        .expect("smooth feasible episode");
    for _ in 0..3 {
        rig.tick(None);
    }
    assert_eq!(rig.tick(Some(Action::Replay)).mode, Mode::Approach);
    let quantum = 2.0 * rig.bus.state().motors[&rig.id].limits[0] / 65535.0 / rig.gain.abs();
    let mut previous = commanded_joint_position(&rig);
    let mut previous_at = rig.dispatched;
    for _ in 0..160 {
        let mode = rig.tick(None).mode;
        let position = commanded_joint_position(&rig);
        assert!(
            (position - previous).abs() <= Rig::step_allowance(0.2, previous_at) + 3.0 * quantum,
            "approach must retain configured .2 rad/s cap"
        );
        previous = position;
        previous_at = rig.dispatched;
        if mode == Mode::ReadyReplay {
            break;
        }
    }
    assert_eq!(rig.controller.mode(), Mode::ReadyReplay);
    assert_eq!(rig.tick(Some(Action::Replay)).mode, Mode::Replay);
    let began = Instant::now();
    let mut peak_speed: f64 = 0.0;
    let mut previous = commanded_joint_position(&rig);
    let mut previous_at = Instant::now();
    for _ in 0..220 {
        let mode = rig.tick(None).mode;
        let now = Instant::now();
        let position = commanded_joint_position(&rig);
        peak_speed = peak_speed
            .max((position - previous).abs() / now.duration_since(previous_at).as_secs_f64());
        previous = position;
        previous_at = now;
        if mode == Mode::Hold {
            break;
        }
    }
    assert_eq!(rig.controller.mode(), Mode::Hold);
    assert!(began.elapsed() >= Duration::from_millis(1450));
    assert!(
        began.elapsed() < Duration::from_millis(2100),
        "replay must use the recorded 1.2 s plus settle time"
    );
    assert!(
        peak_speed > 0.25,
        "replay must restore validated speed above the approach cap: {peak_speed}"
    );
    let position = rig.tick(None).position;
    let mut next = rig.episode(position);
    for sample in &mut next.samples {
        sample.position[rig.selected] += 0.08;
    }
    rig.controller.set_episode(next).expect("new approach");
    rig.tick(None);
    assert_eq!(rig.tick(Some(Action::Replay)).mode, Mode::Approach);
    let mut previous = commanded_joint_position(&rig);
    let mut previous_at = rig.dispatched;
    for _ in 0..25 {
        rig.tick(None);
        let position = commanded_joint_position(&rig);
        assert!(
            (position - previous).abs() <= Rig::step_allowance(0.2, previous_at) + 3.0 * quantum,
            "completed replay must restore the positioning cap"
        );
        previous = position;
        previous_at = rig.dispatched;
    }
    assert_eq!(rig.tick(Some(Action::Hold)).mode, Mode::Hold);
}

// A persistent encoder offset exercises a real driver's ordinary tracking residual;
// it does not change the controller target or bypass feedback decoding.
fn replay_approach_with_residual(error: f64, native_velocity: f64) -> Rig {
    let mut rig = Rig::new();
    let held = rig.tick(Some(Action::Engage));
    let mut episode = rig.episode(held.position);
    episode.samples[1].t_s = 0.01;
    rig.controller.set_episode(episode).expect("episode");
    rig.tick(None);
    assert_eq!(rig.tick(Some(Action::Replay)).mode, Mode::Approach);
    rig.activation
        .position_offset_bits
        .store((error * rig.gain).to_bits(), Ordering::Relaxed);
    if native_velocity != 0.0 {
        rig.activation
            .velocity_override_bits
            .store((native_velocity * rig.gain).to_bits(), Ordering::Relaxed);
    }
    rig
}

#[test]
fn replay_approach_and_final_settle_accept_a_stationary_039_radian_tracking_residual() {
    let mut rig = replay_approach_with_residual(0.039, 0.0);
    for _ in 0..50 {
        if rig.tick(None).mode == Mode::ReadyReplay {
            break;
        }
    }
    assert_eq!(rig.controller.mode(), Mode::ReadyReplay);
    assert_eq!(
        rig.tick(None).mode,
        Mode::ReadyReplay,
        "arrival still requires p"
    );
    assert_eq!(rig.tick(Some(Action::Replay)).mode, Mode::Replay);
    for _ in 0..60 {
        if rig.tick(None).mode == Mode::Hold {
            break;
        }
    }
    assert_eq!(rig.controller.mode(), Mode::Hold);
}

#[test]
fn replay_approach_waits_for_position_tolerance_and_native_velocity() {
    let mut outside = replay_approach_with_residual(0.06, 0.0);
    for _ in 0..45 {
        assert_eq!(outside.tick(None).mode, Mode::Approach);
    }
    let mut moving = replay_approach_with_residual(0.0, 0.2);
    for _ in 0..45 {
        assert_eq!(
            moving.tick(None).mode,
            Mode::Approach,
            "native motion must prevent readiness even at the target"
        );
    }
    moving
        .activation
        .velocity_override_bits
        .store(0, Ordering::Relaxed);
    for _ in 0..50 {
        if moving.tick(None).mode == Mode::ReadyReplay {
            break;
        }
    }
    assert_eq!(moving.controller.mode(), Mode::ReadyReplay);
}
