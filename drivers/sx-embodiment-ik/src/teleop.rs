//! The teleoperation solve: one operator's task-space ask to the next joint positions, the same
//! code on the station (the authority, on measured joints) and in the headset (its ghost).
//!
//! [`Teleop`] wraps one [`Ik`] built from a [`TeleopConfig`]: the body's limbs, each joint's
//! speed and acceleration caps (the governor's, derate included, on the station), the
//! per-tick step bound, the joints held this run and the scene. The station
//! builds the config from its open body and serves it (`sx.teleop-solve/v2`, JSON); the
//! headset builds its solver from those same bytes, so both answer the same step to the same
//! inputs. On top of [`Ik::step`] it owns every teleoperation rule:
//!
//! - **reference**: the solve integrates from the positions it last commanded, not from the
//!   measured ones: a position-controlled joint lags its target, and re-seeding from where it
//!   is measured every tick asks it to move one tick's step at a time, which its PD never
//!   lifts against friction and gravity (the first bench run: commands dithered by 3 mrad and
//!   the arm stayed put). It seeds from the measured joints at the first step after
//!   [`Teleop::reset`] (engage), and holds each axis's reference within [`REFERENCE_GAP`] of
//!   its measurement: one that strays further is brought back to the gap, never snapped onto
//!   the measurement (a snap is a one-tick jump of the whole gap: act-sim's yaw joints, lagging
//!   0.1 rad at speed, jumped 0.08 rad back in a tick, 1.9 m/s at the tool, 2026-10-31). A
//!   ghost with no measurement integrates freely.
//! - **holds**: an axis is held where it is while any limb it moves has no target: an idle
//!   arm stays put (left free, the posture task swung it by up to 1 rad on the first bench
//!   A/B), and so does the torso, which carries both arms and the neck, unless both hands and
//!   the head are driven. Joints the config holds are never solved.
//! - **head**: the limb named `head` (a camera, [`Task::LookAt`]) looks along the operator's
//!   gaze when the ask carries their head (they look through the robot: the head pose's local
//!   +x, as the point [`GAZE_DISTANCE_M`] ahead of the camera along it), and otherwise between
//!   the commanded hands. There is no choice to make: who is looking decides. The neck alone
//!   turns it ([`Ik`] owns that).
//! - **grippers**: each hand's gripper width moves its fingers, linear between the fingertips'
//!   closed and open distance ([`Gripper`]).
//!
//! Nothing here allocates per step after [`Teleop::new`], and nothing reads a clock.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::IkError;
use crate::body::{Body, Limb, Target, Task};
use crate::geometry::Obstacle;
use crate::model::{Frames, Model};
use crate::solve::{Caps, Ik, LimbReport, Reach, Settings, Solve};

/// The config's schema, written in every config and refused when another.
pub const TELEOP_SCHEMA: &str = "sx.teleop-solve/v2";

/// The hands an operator's intent names, in the order [`Targets`] carries them.
pub const HANDS: [&str; 2] = ["left", "right"];

/// How far a joint's reference may lead its measurement: one that strays further is held at
/// this far; well inside the governor's tracking stop.
pub const REFERENCE_GAP: f64 = 0.1;

/// How far ahead of the camera the gaze is turned into a look-at point: far enough that the
/// camera's offset from the operator's eyes does not bend the gaze, near enough to stay a
/// well-conditioned pointing task.
pub const GAZE_DISTANCE_M: f64 = 1.0;

/// The stall rule, the one a gripper closing on something uses too: a joint whose reference
/// stands at [`REFERENCE_GAP`] from its measurement for this long, with its measurement come no
/// nearer the reference than [`STALL_STILL`], is pushing on something that does not give.
pub const STALL: std::time::Duration = std::time::Duration::from_millis(500);

/// Movement under this (metres of a gripper's opening, radians of a joint) is no progress.
pub const STALL_STILL: f64 = 0.0005;

/// How close each driven hand's target must be to its gripper's tool point, as the robot
/// stands, before following starts: control starts from the robot's own pose, never by jumping
/// to the operator's hands. The station and the headset both ask [`Teleop::may_engage`].
pub const ENGAGE_RADIUS_M: f64 = 0.06;

/// One limb of the config: a frame of the description and its task.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LimbConfig {
    pub name: String,
    pub frame: String,
    /// `None`: reach a pose (a gripper's tool frame); a unit vector in the frame: point it at
    /// a target (a camera).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub look_forward: Option<[f64; 3]>,
    /// A pose limb's approach axis, a unit vector in its frame (out between a gripper's
    /// fingers): a target may hold only it and let the roll about it float
    /// ([`Targets::free_roll`]). `None`: the limb is only ever asked for whole poses.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub approach: Option<[f64; 3]>,
}

/// Everything the solve is built from besides the description, so a station and a headset
/// that read the same config answer the same step: [`TELEOP_SCHEMA`] as JSON.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TeleopConfig {
    pub schema: String,
    pub limbs: Vec<LimbConfig>,
    /// Per joint (by name), its speed cap in rad/s (unit/s): every joint the limbs move.
    pub speed: BTreeMap<String, f64>,
    /// Per joint, its acceleration cap in rad/s²; absent: its speed reached in `ramp_s`.
    #[serde(default)]
    pub accel: BTreeMap<String, f64>,
    /// The time a joint takes to reach its speed cap from rest where `accel` names none.
    pub ramp_s: f64,
    /// The most any joint moves in one tick ([`Settings::max_step_rad`]).
    pub max_step_rad: f64,
    /// Joints never solved this run: held where they are.
    #[serde(default)]
    pub held: Vec<String>,
    /// Boxes beside the body's own support, `[min x, y, z, max x, y, z]` in the root frame.
    #[serde(default)]
    pub obstacles: Vec<[f64; 6]>,
    /// The posture the arms' redundancy resolves toward, by joint (a robot's commissioned Ready
    /// pose, elbows down; [`crate::Ik::prefer_posture`]); empty: the posture at engage. A
    /// joint it does not name stays at its own value at engage.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub posture: BTreeMap<String, f64>,
}

impl TeleopConfig {
    /// The static upper body's limbs: both grippers' tool frames, approaching along their +x
    /// (the tool point sits 15 cm out along the gripper base's +x), and the ZED looking out of
    /// its lenses (`ZED_camera` +x) from the SO-101 neck.
    #[must_use]
    pub fn static_upper_body_limbs() -> Vec<LimbConfig> {
        vec![
            LimbConfig {
                name: "left".into(),
                frame: "left_gripper_tcp".into(),
                look_forward: None,
                approach: Some([1.0, 0.0, 0.0]),
            },
            LimbConfig {
                name: "right".into(),
                frame: "right_gripper_tcp".into(),
                look_forward: None,
                approach: Some([1.0, 0.0, 0.0]),
            },
            LimbConfig {
                name: "head".into(),
                frame: "ZED_camera".into(),
                look_forward: Some([1.0, 0.0, 0.0]),
                approach: None,
            },
        ]
    }

    /// Parse a config.
    ///
    /// # Errors
    ///
    /// [`IkError::Caps`] for a document that is not [`TELEOP_SCHEMA`].
    pub fn parse(json: &str) -> Result<Self, IkError> {
        let config: Self =
            serde_json::from_str(json).map_err(|e| IkError::Caps(format!("teleop config: {e}")))?;
        if config.schema != TELEOP_SCHEMA {
            return Err(IkError::Caps(format!(
                "teleop config is {:?}, not {TELEOP_SCHEMA}",
                config.schema
            )));
        }
        Ok(config)
    }

    /// The config as its JSON document.
    ///
    /// # Panics
    ///
    /// Never: every field serialises.
    #[must_use]
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).expect("a teleop config serialises")
    }
}

/// A rigid pose in the robot's root frame: metres and a unit quaternion (x, y, z, w).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub translation_m: [f64; 3],
    pub rotation_xyzw: [f64; 4],
}

/// One tick's ask, the operator's intent as the solve takes it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Targets {
    /// Per hand in [`HANDS`] order: the tool point, absent when that hand is not driven.
    pub tcp: [Option<Pose>; 2],
    /// Per hand: the gripper opening in metres.
    pub gripper_width_m: [Option<f64>; 2],
    /// Per hand: hold only `tcp`'s approach axis (the limb's [`LimbConfig::approach`] turned
    /// by `tcp`'s rotation) and let the roll about it float: a grasp or a press whose roll
    /// does not matter.
    pub free_roll: [bool; 2],
    /// Per hand: the most its tool point may move, m/s ([`crate::Ik::set_tool_speed`]): a
    /// program's goal at its linear speed. `None`: as fast as the joints' caps allow (an
    /// operator's hands, whose own motion is the speed).
    pub tool_speed_m_s: [Option<f64>; 2],
    /// The operator's head when they look through the robot (remote): its local +x is the
    /// gaze, and the camera follows it. Absent (beside the robot, or no operator): the camera
    /// looks between the commanded hands.
    pub head: Option<Pose>,
}

/// How one gripper's opening in metres becomes its finger angles: linear between the fingers'
/// closed angles at `closed_width_m` and their open angles at `open_width_m`. The width is the
/// distance between the two fingertips (each finger link's collision capsule end furthest from
/// its hinge; the finger links' origins sit on their hinges and never part).
#[derive(Clone, Debug, PartialEq)]
pub struct Gripper {
    /// Per finger: (axis, angle closed, angle open).
    pub fingers: Vec<(usize, f64, f64)>,
    pub closed_width_m: f64,
    pub open_width_m: f64,
}

impl Gripper {
    /// The finger angles for `width_m`, written into `q`.
    pub fn write(&self, width_m: f64, q: &mut [f64]) {
        let span = self.open_width_m - self.closed_width_m;
        let fraction = if span > 0.0 {
            ((width_m - self.closed_width_m) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        for &(axis, closed, open) in &self.fingers {
            q[axis] = closed + (open - closed) * fraction;
        }
    }

    /// `side`'s gripper of `body` (axes `<side>_gripper_*`), each finger from the end of its
    /// range nearest zero to the far end; the end at which the fingertips stand further apart
    /// is open. `None` when the body has no such gripper.
    #[must_use]
    pub fn of(body: &Body, side: &str) -> Option<Self> {
        let model = body.model();
        let prefix = format!("{side}_gripper_");
        let axes: Vec<(usize, [f64; 2])> = model
            .axes()
            .iter()
            .enumerate()
            .filter(|(_, axis)| axis.name.starts_with(&prefix))
            .filter_map(|(index, axis)| axis.limits.map(|limits| (index, limits)))
            .collect();
        let norm = |v: [f64; 3]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
        let tips: Vec<(usize, [f64; 3])> = model
            .links()
            .iter()
            .enumerate()
            .filter(|(_, link)| link.starts_with(&prefix) && link.contains("finger"))
            .filter_map(|(index, _)| {
                body.proxies()
                    .capsules
                    .iter()
                    .filter(|capsule| capsule.link == index)
                    .flat_map(|capsule| [capsule.a, capsule.b])
                    .max_by(|a, b| norm(*a).total_cmp(&norm(*b)))
                    .map(|tip| (index, tip))
            })
            .collect();
        if axes.is_empty() || tips.len() < 2 {
            return None;
        }
        let near = |[lower, upper]: [f64; 2]| 0.0_f64.clamp(lower, upper);
        let far = |[lower, upper]: [f64; 2]| {
            if lower.abs() >= upper.abs() {
                lower
            } else {
                upper
            }
        };
        let rest: Vec<f64> = model
            .axes()
            .iter()
            .map(|axis| axis.limits.map_or(0.0, near))
            .collect();
        let width = |at_far: bool| {
            let mut q = rest.clone();
            for (index, limits) in &axes {
                q[*index] = if at_far { far(*limits) } else { near(*limits) };
            }
            let frames = model.poses(&q);
            let (a, b) = (
                frames.links[tips[0].0].apply(tips[0].1),
                frames.links[tips[1].0].apply(tips[1].1),
            );
            norm([0, 1, 2].map(|k| a[k] - b[k]))
        };
        let (near_width, far_width) = (width(false), width(true));
        let far_opens = far_width >= near_width;
        let fingers = axes
            .iter()
            .map(|(index, limits)| {
                let (closed, open) = if far_opens {
                    (near(*limits), far(*limits))
                } else {
                    (far(*limits), near(*limits))
                };
                (*index, closed, open)
            })
            .collect();
        let (closed_width_m, open_width_m) = if far_opens {
            (near_width, far_width)
        } else {
            (far_width, near_width)
        };
        Some(Self {
            fingers,
            closed_width_m,
            open_width_m,
        })
    }
}

/// One teleoperation step: every axis in [`Model::axes`] order.
#[derive(Clone, Debug, PartialEq)]
pub struct TeleopStep {
    /// The next positions: the new reference.
    pub q: Vec<f64>,
    /// The planned velocity to them (zero where held, and for the fingers).
    pub qdot: Vec<f64>,
    /// Per limb in the config's order.
    pub limbs: Vec<LimbReport>,
    pub solve: Solve,
    /// Per axis: stalled ([`STALL`]): its reference is at its measurement and it stands
    /// rather than push on, free to move back. The caller may end what it was doing.
    pub stalled: Vec<bool>,
}

/// The shared teleoperation solve; see the module docs.
pub struct Teleop {
    ik: Ik,
    config: TeleopConfig,
    /// Per axis, the limbs it moves, one bit per limb.
    limbs_of: Vec<u64>,
    /// Per limb, its index in [`HANDS`] (`None` for the head).
    hand_of: Vec<Option<usize>>,
    /// Per limb, its approach axis in its frame ([`LimbConfig::approach`]).
    approach: Vec<Option<[f64; 3]>>,
    head: Option<usize>,
    /// Axes the config holds.
    held: Vec<bool>,
    grippers: [Option<Gripper>; 2],
    /// Whether the next step seeds from the measured joints (after [`Teleop::reset`]).
    seed: bool,
    q: Vec<f64>,
    available: Vec<bool>,
    wanted: Vec<Option<Target>>,
    frames: Frames,
    step: TeleopStep,
    /// Per axis: where it was measured when its reference first stood at the gap, and the
    /// seconds since, while it comes no nearer ([`STALL`]).
    leashed: Vec<Option<(f64, f64)>>,
    /// Per axis: the way it pushed when it stalled (+1, -1), until a step moves it back.
    stall: Vec<f64>,
}

impl Teleop {
    /// The solve for the description `urdf` with its `collision` proxies, as `config` says.
    ///
    /// # Errors
    ///
    /// [`IkError`] for a description, limb or cap the solver refuses, and a config whose
    /// caps name a joint the description lacks or leave a solved joint without a speed.
    pub fn new(urdf: &str, collision: &str, config: &TeleopConfig) -> Result<Self, IkError> {
        let limbs: Vec<Limb> = config
            .limbs
            .iter()
            .map(|limb| Limb {
                name: limb.name.clone(),
                frame: limb.frame.clone(),
                task: limb
                    .look_forward
                    .map_or(Task::Pose, |forward| Task::LookAt { forward }),
            })
            .collect();
        let body = Body::new(urdf, collision, &limbs)?;
        Self::with_body(body, config)
    }

    /// [`Teleop::new`] over a body already parsed.
    ///
    /// # Errors
    ///
    /// As [`Teleop::new`].
    #[allow(clippy::too_many_lines)] // One build: caps, holds, limbs, grippers, buffers.
    pub fn with_body(body: Body, config: &TeleopConfig) -> Result<Self, IkError> {
        let names: Vec<String> = body.model().axes().iter().map(|a| a.name.clone()).collect();
        for joint in config
            .speed
            .keys()
            .chain(config.accel.keys())
            .chain(&config.held)
            .chain(config.posture.keys())
        {
            if !names.contains(joint) {
                return Err(IkError::Caps(format!(
                    "no joint {joint:?} in the description"
                )));
            }
        }
        let positive = |value: f64| value.is_finite() && value > 0.0;
        if !positive(config.ramp_s) || !positive(config.max_step_rad) {
            return Err(IkError::Caps("ramp and step must be positive".into()));
        }
        let mut speed = Vec::with_capacity(names.len());
        let mut accel = Vec::with_capacity(names.len());
        for (index, name) in names.iter().enumerate() {
            let cap = match config.speed.get(name) {
                Some(cap) => *cap,
                None if !body.solved().contains(&index) => 0.0,
                None => return Err(IkError::NoSpeed(name.clone())),
            };
            speed.push(cap);
            accel.push(
                config
                    .accel
                    .get(name)
                    .copied()
                    .unwrap_or(cap / config.ramp_s),
            );
        }
        let settings = Settings {
            max_step_rad: config.max_step_rad,
            ..Settings::default()
        };
        let scene = config
            .obstacles
            .iter()
            .map(|o| Obstacle {
                min: [o[0], o[1], o[2]],
                max: [o[3], o[4], o[5]],
            })
            .collect();
        let width = names.len();
        let limb_names: Vec<&str> = body.limbs();
        let mut limbs_of = vec![0_u64; width];
        for index in 0..limb_names.len().min(64) {
            for &axis in body.limb_axes(index) {
                limbs_of[axis] |= 1 << index;
            }
        }
        let hand_of = limb_names
            .iter()
            .map(|name| HANDS.iter().position(|hand| hand == name))
            .collect();
        let head = limb_names.iter().position(|name| *name == "head");
        let mut approach = Vec::with_capacity(limb_names.len());
        for name in &limb_names {
            let axis = config
                .limbs
                .iter()
                .find(|limb| limb.name == *name)
                .and_then(|limb| limb.approach);
            if let Some(axis) = axis {
                let length = axis.iter().map(|v| v * v).sum::<f64>().sqrt();
                if !(length.is_finite() && (length - 1.0).abs() < 1e-6) {
                    return Err(IkError::Caps(format!(
                        "limb {name:?}'s approach axis is not a unit vector"
                    )));
                }
            }
            approach.push(axis);
        }
        let held_axes = names
            .iter()
            .map(|name| config.held.contains(name))
            .collect();
        let grippers = HANDS.map(|side| Gripper::of(&body, side));
        let mut q = vec![0.0; width];
        for (slot, axis) in q.iter_mut().zip(body.model().axes()) {
            if let Some([lower, upper]) = axis.limits {
                *slot = 0.0_f64.clamp(lower, upper);
            }
        }
        let frames = body.model().poses(&q);
        let limb_count = limb_names.len();
        let mut ik = Ik::new(body, Caps { speed, accel }, settings, scene)?;
        if !config.posture.is_empty() {
            let posture: Vec<f64> = names
                .iter()
                .zip(ik.rest())
                .map(|(name, rest)| config.posture.get(name).copied().unwrap_or(*rest))
                .collect();
            ik.prefer_posture(&posture)?;
        }
        let step = TeleopStep {
            q: q.clone(),
            qdot: vec![0.0; width],
            limbs: (0..limb_count)
                .map(|_| LimbReport {
                    residual_m: 0.0,
                    residual_rad: 0.0,
                    reach: Reach::Idle,
                    active: Vec::new(),
                    hidden_by: None,
                })
                .collect(),
            solve: Solve::Solved,
            stalled: vec![false; width],
        };
        Ok(Self {
            ik,
            config: config.clone(),
            limbs_of,
            hand_of,
            approach,
            head,
            held: held_axes,
            grippers,
            seed: true,
            q,
            available: vec![false; width],
            wanted: vec![None; limb_count],
            frames,
            step,
            leashed: vec![None; width],
            stall: vec![0.0; width],
        })
    }

    /// The config this solve was built from.
    #[must_use]
    pub const fn config(&self) -> &TeleopConfig {
        &self.config
    }

    /// The solver underneath.
    #[must_use]
    pub const fn ik(&self) -> &Ik {
        &self.ik
    }

    /// The description's joints, the order of every vector here.
    #[must_use]
    pub fn model(&self) -> &Model {
        self.ik.body().model()
    }

    /// Each hand's gripper, in [`HANDS`] order.
    #[must_use]
    pub const fn grippers(&self) -> &[Option<Gripper>; 2] {
        &self.grippers
    }

    /// Each driven hand's distance, in metres, from its target to its gripper's tool point at
    /// `q` (every axis, [`Teleop::model`] order); `None` for a hand with no target.
    pub fn hand_gaps(&mut self, q: &[f64], targets: &Targets) -> [Option<f64>; 2] {
        let mut gaps = [None; 2];
        if q.len() != self.q.len() {
            return gaps;
        }
        self.ik.body().model().poses_into(q, &mut self.frames);
        for (limb, hand) in self.hand_of.iter().enumerate() {
            let Some(hand) = *hand else { continue };
            if let Some(target) = targets.tcp[hand] {
                let at = self.ik.body().frame(&self.frames, limb).t;
                let gap = (0..3)
                    .map(|k| (target.translation_m[k] - at[k]).powi(2))
                    .sum::<f64>()
                    .sqrt();
                gaps[hand] = Some(gap);
            }
        }
        gaps
    }

    /// Whether following may start from `q`, where the robot stands: at least one hand is
    /// driven and every driven hand's target is within [`ENGAGE_RADIUS_M`] of its gripper.
    pub fn may_engage(&mut self, q: &[f64], targets: &Targets) -> bool {
        let gaps = self.hand_gaps(q, targets);
        gaps.iter().any(Option::is_some) && gaps.iter().flatten().all(|gap| *gap <= ENGAGE_RADIUS_M)
    }

    /// Engage: the next step seeds from the measured joints, starts from standing still and
    /// anchors posture there.
    pub fn reset(&mut self) {
        self.ik.reset();
        self.seed = true;
    }

    /// Each hand's target as the solve takes it: its whole pose, or with its roll free only
    /// its approach axis.
    fn want_hands(&mut self, targets: &Targets) -> Result<(), IkError> {
        for (limb, (slot, hand)) in self.wanted.iter_mut().zip(&self.hand_of).enumerate() {
            let Some(hand) = *hand else {
                *slot = None;
                continue;
            };
            *slot = match (targets.tcp[hand], targets.free_roll[hand]) {
                (None, _) => None,
                (Some(p), false) => Some(Target::Pose {
                    translation_m: p.translation_m,
                    rotation_xyzw: p.rotation_xyzw,
                }),
                (Some(p), true) => {
                    let axis = self.approach[limb]
                        .ok_or_else(|| IkError::Target(HANDS[hand].to_owned()))?;
                    Some(Target::Axis {
                        translation_m: p.translation_m,
                        axis,
                        toward: rotate(p.rotation_xyzw, axis),
                    })
                }
            };
        }
        Ok(())
    }

    /// Each hand's tool speed for this step, held from where `reference` (the last command)
    /// puts the tool, not from where the reference was held back to; at engage (`seeded`)
    /// there is no last command, only the measurement.
    fn bound_tools(&mut self, reference: &[f64], targets: &Targets, seeded: bool) {
        let from_last = !seeded && targets.tool_speed_m_s.iter().any(Option::is_some);
        if from_last {
            self.ik
                .body()
                .model()
                .poses_into(reference, &mut self.frames);
        }
        for (limb, hand) in self.hand_of.iter().enumerate() {
            let speed = hand.and_then(|hand| targets.tool_speed_m_s[hand]);
            let from = speed
                .filter(|_| from_last)
                .map(|_| self.ik.body().frame(&self.frames, limb).t);
            self.ik.set_tool_speed(limb, speed, from);
        }
    }

    /// The reference the step integrates from: seeded from the measurement at engage, held
    /// within [`REFERENCE_GAP`] of it, and at the measurement for a joint that stalled there
    /// ([`STALL`]).
    fn take_reference(
        &mut self,
        reference: &[f64],
        measured: &[Option<f64>],
        seeded: bool,
        dt: f64,
    ) {
        if seeded {
            self.leashed.fill(None);
            self.stall.fill(0.0);
        }
        let stall_s = STALL.as_secs_f64();
        for (axis, (slot, (sent, reading))) in self
            .q
            .iter_mut()
            .zip(reference.iter().zip(measured))
            .enumerate()
        {
            *slot = match reading {
                Some(at) if seeded => *at,
                // A reading that is not a number leaves the reference (as the old rule did,
                // and f64::clamp would panic on it).
                Some(at) if at.is_finite() => {
                    let lead = sent - at;
                    if self.stall[axis] == 0.0 && lead.abs() >= REFERENCE_GAP - 1e-9 {
                        // At the gap: pushing on something, or only lagging? It stalls when
                        // the measurement comes no nearer for the gripper's stall time.
                        let push = lead.signum();
                        self.leashed[axis] = match self.leashed[axis] {
                            Some((from, since)) if (at - from) * push <= STALL_STILL => {
                                let since = since + dt;
                                if since >= stall_s {
                                    self.stall[axis] = push;
                                    None
                                } else {
                                    Some((from, since))
                                }
                            }
                            _ => Some((*at, 0.0)),
                        };
                    } else {
                        self.leashed[axis] = None;
                    }
                    if self.stall[axis] == 0.0 {
                        sent.clamp(at - REFERENCE_GAP, at + REFERENCE_GAP)
                    } else {
                        // Stalled: no push left, the reference where the joint stands.
                        *at
                    }
                }
                Some(_) | None => *sent,
            };
        }
    }

    /// One tick. `reference` is what the solve last commanded (the page's ghost: its own last
    /// answer); `measured` each axis's fresh reading, `None` where it did not answer or there
    /// is none (a ghost); `available[i]` false for an axis that must stay where it is.
    ///
    /// # Errors
    ///
    /// [`IkError`] for vectors of the wrong length, a non-finite position or `dt`, and a free
    /// roll asked of a limb that declares no approach axis.
    pub fn step(
        &mut self,
        reference: &[f64],
        measured: &[Option<f64>],
        available: &[bool],
        targets: &Targets,
        dt: f64,
    ) -> Result<&TeleopStep, IkError> {
        let width = self.q.len();
        for (given, expected) in [
            (reference.len(), width),
            (measured.len(), width),
            (available.len(), width),
        ] {
            if given != expected {
                return Err(IkError::Positions { expected, given });
            }
        }
        // The reference, seeded from the measurement at engage and held within the gap of it.
        let seeded = self.seed;
        self.take_reference(reference, measured, seeded, dt);
        self.seed = false;
        for (slot, (available, held)) in self
            .available
            .iter_mut()
            .zip(available.iter().zip(&self.held))
        {
            *slot = *available && !held;
        }
        self.want_hands(targets)?;
        if let Some(head) = self.head {
            self.wanted[head] = if let Some(gaze) = targets.head {
                self.ik.body().model().poses_into(&self.q, &mut self.frames);
                let camera = self.ik.body().frame(&self.frames, head);
                let forward = rotate(gaze.rotation_xyzw, [1.0, 0.0, 0.0]);
                Some(Target::LookAt {
                    point_m: [0, 1, 2].map(|k| GAZE_DISTANCE_M.mul_add(forward[k], camera.t[k])),
                })
            } else {
                self.ik.body().model().poses_into(&self.q, &mut self.frames);
                let camera = self.ik.body().frame(&self.frames, head).t;
                let mut sum = [0.0; 3];
                let mut bisector = [0.0; 3];
                let mut distance = 0.0;
                let mut count = 0.0;
                for pose in targets.tcp.iter().flatten() {
                    let to = [0, 1, 2].map(|k| pose.translation_m[k] - camera[k]);
                    let length = to.iter().map(|v| v * v).sum::<f64>().sqrt().max(1e-9);
                    for k in 0..3 {
                        sum[k] += pose.translation_m[k];
                        bisector[k] += to[k] / length;
                    }
                    distance += length;
                    count += 1.0;
                }
                let across = bisector.iter().map(|v| v * v).sum::<f64>().sqrt();
                (count > 0.0).then(|| Target::LookAt {
                    // Hands on opposite sides of the camera have no bisector: their midpoint.
                    point_m: if across > 1e-6 {
                        let reach = distance / count / across;
                        [0, 1, 2].map(|k| reach.mul_add(bisector[k], camera[k]))
                    } else {
                        sum.map(|total| total / count)
                    },
                })
            };
        }
        let mut idle = 0_u64;
        for (index, wanted) in self.wanted.iter().enumerate().take(64) {
            if wanted.is_none() {
                idle |= 1 << index;
            }
        }
        for (avail, limbs) in self.available.iter_mut().zip(&self.limbs_of) {
            if limbs & idle != 0 {
                *avail = false;
            }
        }
        self.bound_tools(reference, targets, seeded);
        for (axis, push) in self.stall.iter().enumerate() {
            self.ik.set_blocked(axis, *push);
        }
        let solved = self.ik.step(&self.q, &self.available, &self.wanted, dt)?;
        // A stalled joint stays held until a step moves it back, away from what stopped it.
        for (axis, push) in self.stall.iter_mut().enumerate() {
            if *push != 0.0 && (solved.q[axis] - self.q[axis]) * *push < -1e-9 {
                *push = 0.0;
            }
            self.step.stalled[axis] = *push != 0.0;
        }
        self.step.q.copy_from_slice(&solved.q);
        self.step.qdot.copy_from_slice(&solved.qdot);
        self.step.limbs.clone_from(&solved.limbs);
        self.step.solve = solved.solve;
        for (gripper, width) in self.grippers.iter().zip(targets.gripper_width_m) {
            if let (Some(gripper), Some(width)) = (gripper, width) {
                gripper.write(width, &mut self.step.q);
                for &(axis, _, _) in &gripper.fingers {
                    self.step.qdot[axis] = 0.0;
                }
            }
        }
        Ok(&self.step)
    }
}

/// `v` rotated by the unit quaternion `xyzw`.
fn rotate([x, y, z, w]: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    // v + 2w (u x v) + 2 u x (u x v), u = (x, y, z).
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let u = [x, y, z];
    let uv = cross(u, v);
    let uuv = cross(u, uv);
    [0, 1, 2].map(|k| 2.0f64.mul_add(w.mul_add(uv[k], uuv[k]), v[k]))
}
