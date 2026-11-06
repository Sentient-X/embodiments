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
//!   joint the solve moves is brought back by the step itself ([`crate::Ik::set_window`]), so
//!   the collision rows see the move; moving the reference there first skipped every pair
//!   (embodiments#206). A ghost with no measurement integrates freely.
//! - **pairs where the robot is**: between ticks a joint is somewhere between where it was
//!   measured and where it was commanded, so pairs are kept clear at both
//!   ([`crate::Ik::set_measured`]); a joint that does not follow (stalled, at the gap and
//!   coming no nearer, or led past a stop it stalled on) is kept where it stands and never
//!   counted on to open a pair.
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
//! - **tremor**: an operator's hand (a tool target with no [`Targets::tool_speed`]) passes a
//!   One-Euro filter ([`crate::smoothing`], [`Teleop::set_input_filter`]) on its whole pose
//!   before anything reads it, so the station and the headset's ghost filter it identically and
//!   it is the only tremor filter between the operator and the robot. A program's goal (a tool
//!   speed) is smooth by construction and passes as given. Each hand's filter starts afresh at
//!   [`Teleop::reset`] and whenever the hand stops being driven.
//! - **starts and ends**: each hand's target, filtered, then passes its [`ToolShaper`]
//!   ([`crate::shaper`]) before the solve reads it: from where the tool stands at rest, the
//!   shaped target reaches the asked one jerk-limited and never past it, rides it while it
//!   moves within the tool's bounds (the joints' caps carried to the tool at the pose), and
//!   when the hand's target goes (released, let go, a program's end) comes to rest
//!   jerk-limited before the hand is held ([`Teleop::settling`]). The solve follows the
//!   shaped target with every row in force. A program's goal is shaped too, under its tool
//!   speed. Each joint's jerk cap is [`TeleopConfig::jerk`].
//!
//! Nothing here allocates per step after [`Teleop::new`], and nothing reads a clock.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use sx_frames::{Pose, Quaternion};

use crate::IkError;
use crate::body::{Body, Limb, Target, Task};
use crate::geometry::Obstacle;
use crate::model::{Frames, Model};
use crate::shaper::{Aim, Shaped, ToolMap, ToolShaper};
use crate::smoothing::{OneEuro, PoseFilter};
use crate::solve::{
    Caps, FOLLOWER_SPEED, Ik, LimbReport, READING_JITTER, Reach, Settings, Solve, ToolSpeed,
};

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
/// nearer the reference than [`STALL_STILL`], is pushing on something that does not give. Only
/// a driven arm's solved joints stall (not a finger written from a width, a held joint or an
/// idle arm), and a stalled one is held stiff where it stopped. Something soft that keeps
/// giving faster than [`STALL_STILL`] a stall time is not a stall: the joint goes on pushing at
/// most the gap, as a gripper closing on something soft goes on closing.
pub const STALL: std::time::Duration = std::time::Duration::from_millis(500);

/// Movement under this (metres of a gripper's opening, radians of a joint) is no progress.
pub const STALL_STILL: f64 = 0.0005;

/// The least jump a reading is doubted for, radians: well past any reading's noise (the
/// clearance tests jitter readings by 1-3 mrad, and doubting noise held a reading back a tick
/// and lurched an arm 150x its cap); a glitch worth catching is 0.05 rad and more.
pub const GLITCH_FLOOR: f64 = 0.01;

/// Over how long readings are smoothed where a stall is judged (whether the joint pushes at the
/// gap, and whether it comes nearer): long enough to quiet a few mrad of noise at 500 Hz. The
/// gap itself is kept from the reading: a window from the smoothed one lagged the joint and
/// cost clearance.
pub const STEADY_S: std::time::Duration = std::time::Duration::from_millis(20);

/// How near the reference gap, from the smoothed reading, a joint whose stall clock runs still
/// pushes at it: at 500 Hz the acceleration cap holds the step near the noisy edge's lowest
/// jitter, 2-3 mrad inside.
pub const AT_THE_GAP: f64 = 0.005;

/// How long a reading that jumps further than the joint could move is set aside before it is
/// taken as real (someone moved the arm, a joint slipped).
pub const GLITCH_HOLD: std::time::Duration = std::time::Duration::from_millis(20);

/// A stalled joint lets go when its targets have asked it back off what stopped it by more
/// than this, radians: what each tick's solve asks of it ([`crate::Ik::asked`]), summed over
/// the ticks since they last asked it on, so a distance rather than a rate. On the static
/// body's held left elbow (2026-11-02) a hand pushing on asks it 2e-4 to 5e-4 rad a tick on; a
/// hand turned back 8 cm asks 1.2e-2 rad back in its first 10 ms at 100, 250 and 500 Hz, a
/// release at once. Once the rest of the arm takes up a slow turn back, a tick asks little:
/// the release waits (0.2 to 1 s, the tool within 5 mm of a slow hand) while it stands still.
/// A jump whose free path first draws the joint back by more than this releases it, as any
/// genuine ask back does ([0.06, 0, 0.06] from Ready at 250 and 500 Hz, not at 100 Hz); the
/// push that follows is a fresh contact's, at most the reference gap, and lasts while the
/// hand stays there, as a fresh contact under the gap does.
pub const STALL_RELEASE: f64 = 0.01;

/// The operator's hands as offsets from where they took hold: a clutch. At engage (after
/// [`Clutch::release`]) each hand's first target is taken as its gripper's tool point where
/// the robot stands, and from then on the gripper moves as the hand moves from there: the hand's
/// translation since, added in the robot's frame, and its turn since, applied in the robot's
/// frame. So control starts from the robot's own pose wherever the operator's hand is, with no
/// lining up, and a regrip anywhere re-takes it (the 6 cm match it replaces refused every
/// resume whose hand had drifted, 2026-11-03). A hand that loses its target lets go, and takes
/// hold afresh when it is seen again. Only an operator's hands are clutched; a program's goals
/// are where they are. The station's follower and the headset's ghost hold one each, so both
/// answer the same targets.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Clutch {
    /// Per hand in [`HANDS`] order: the translation added to the hand, and the rotation applied
    /// after the hand's own (its rotation at hold, inverted, then the gripper's).
    held: [Option<([f64; 3], Quaternion)>; 2],
}

impl Clutch {
    /// Let go of both hands: the next targets take hold afresh. Every engage.
    pub fn release(&mut self) {
        self.held = [None; 2];
    }

    /// Whether `hand` holds on.
    #[must_use]
    pub fn holds(&self, hand: usize) -> bool {
        self.held.get(hand).is_some_and(Option::is_some)
    }

    /// `targets`' hands moved onto the grippers they hold, taking hold with `tools` (each
    /// gripper's tool point where the robot stands, [`Teleop::tool_poses`]) for a hand seen
    /// first; a hand whose gripper has no pose is left as given.
    pub fn apply(&mut self, tools: &[Option<Pose>; 2], targets: &mut Targets) {
        for hand in 0..HANDS.len() {
            let Some(target) = targets.tcp[hand] else {
                self.held[hand] = None;
                continue;
            };
            if self.held[hand].is_none() {
                let Some(tool) = tools[hand] else { continue };
                let offset = [0, 1, 2].map(|k| tool.translation_m[k] - target.translation_m[k]);
                let local = target.rotation.conjugate() * tool.rotation;
                self.held[hand] = Some((offset, local));
            }
            if let Some((offset, local)) = self.held[hand] {
                targets.tcp[hand] = Some(Pose {
                    translation_m: [0, 1, 2].map(|k| target.translation_m[k] + offset[k]),
                    rotation: target.rotation * local,
                });
            }
        }
    }
}

/// One tick of a limb whose target went (`still`: it can be held without a jolt, [`holds`]): a
/// shaper that ran is reset and the limb comes to rest (`resting`, its joints braking each
/// within its caps, `Target::Rest`) until it is still, or [`COAST_MAX_S`] has passed whatever it
/// does; then it is held.
fn come_to_rest(
    resting: &mut bool,
    gone_s: &mut f64,
    shaper: &mut ToolShaper,
    still: bool,
    dt: f64,
) {
    if shaper.is_active() {
        shaper.reset();
        *resting = true;
        *gone_s = 0.0;
    }
    if !*resting {
        return;
    }
    if *gone_s >= COAST_MAX_S || still {
        *resting = false;
        *gone_s = 0.0;
    } else {
        *gone_s += dt;
    }
}

/// Whether every joint of `limb` can be held where it is next tick within its acceleration cap
/// and half its jerk cap, from the velocity `qdot` the last step commanded and `before` the one
/// before it, both on the tick it stops (to `-v / dt`) and the tick after (back to none): a limb
/// whose target went is let go then, so it is held without a jolt. Testing the first alone, a
/// limb let go at 0.05 rad/s jerked a shoulder 3x its cap the tick after.
fn holds(ik: &Ik, limb: usize, qdot: &[f64], before: &[f64], jerk: &[f64], dt: f64) -> bool {
    let caps = ik.caps();
    ik.body().limb_axes(limb).iter().all(|&axis| {
        let v = qdot[axis];
        let a = (v - before[axis]) / dt;
        let stop = -v / dt;
        let most = 0.5 * jerk[axis] * dt;
        stop.abs() <= caps.accel[axis] && (stop - a).abs() <= most && stop.abs() <= most
    })
}

/// The rotation vector turning unit `from` onto unit `to` the least.
fn least_turn(from: [f64; 3], to: [f64; 3]) -> [f64; 3] {
    let axis = [
        from[1] * to[2] - from[2] * to[1],
        from[2] * to[0] - from[0] * to[2],
        from[0] * to[1] - from[1] * to[0],
    ];
    let sin = axis.iter().map(|x| x * x).sum::<f64>().sqrt();
    let cos: f64 = (0..3).map(|k| from[k] * to[k]).sum();
    if sin < 1e-12 {
        return [0.0; 3];
    }
    let angle = sin.atan2(cos);
    axis.map(|x| x / sin * angle)
}

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
    /// Per joint, its jerk cap in rad/s³, the most its acceleration changes: its acceleration
    /// cap over its follower's rise time (`sx_embodiment_drivers::governor::AxisLimits::max_jerk`).
    /// Absent: its acceleration reached in `ramp_s`. The tool's target is shaped within it
    /// ([`crate::shaper`]).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub jerk: BTreeMap<String, f64>,
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
    /// Per hand: the most its tool may move and turn ([`crate::Ik::set_tool_speed`]): a
    /// program's goal or timed plan at the robot's caps. `None`: as fast as the joints' caps
    /// allow (an operator's hands, whose own motion is the speed).
    pub tool_speed: [Option<ToolSpeed>; 2],
    /// The operator's head when they look through the robot (remote): its local +x is the
    /// gaze, and the camera follows it. Absent (beside the robot, or no operator): the camera
    /// looks between the commanded hands.
    pub head: Option<Pose>,
    /// Per hand: how it is moving, its point's velocity (m/s) and turn rate (rad/s) in the root
    /// frame, for the solve to follow it by ([`crate::Ik::set_target_velocity`]): a person's
    /// hand as the station measured it (`Intent::motion`), a goal's path speed. `None`: for a
    /// person's hand the tremor filter's own estimate, for a goal none.
    pub velocity: [Option<([f64; 3], [f64; 3])>; 2],
    /// How old the hands' sample is, seconds (the link's latency on arrival plus the time held
    /// since): a person's hand is carried forward by it after its tremor is filtered out, along
    /// the filter's own tremor-free motion ([`LEAD_MAX_S`], [`LEAD_MAX_M`], [`LEAD_MAX_RAD`]),
    /// and the filter
    /// reads it on the sender's clock (now less its age). `None`: as read, filtered on the
    /// solve's clock.
    pub age_s: Option<f64>,
}

/// The longest a shaped target may still run after its target went, seconds: then the limb is
/// held whatever it does. A stop from 2 m/s ends within 0.2 s; at 250 and 500 Hz the joints'
/// creep after a release never met [`holds`]' one-tick test, and the station never held (refute).
pub const COAST_MAX_S: f64 = 0.6;

/// The longest a person's hand is carried forward by its sample's age ([`Targets::age_s`]). The
/// station gives the age bounded by its own horizon (the sample's latency on arrival plus one
/// send period), so a stall does not carry the hand on; this is the backstop. At 0.3 s a hand
/// that stopped in a stall was carried 3 cm past where it stopped (refute, 2026-11-06).
pub const LEAD_MAX_S: f64 = 0.15;
/// The furthest it is carried, metres.
pub const LEAD_MAX_M: f64 = 0.03;
/// The most it is turned, radians: a twist moves the tool point nowhere, so the distance cap
/// alone let a 4 rad/s twist carry 0.52 rad past the stopped hand (refute, 2026-11-06).
pub const LEAD_MAX_RAD: f64 = 0.25;

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
    /// rather than push on, until the hand asks it back ([`STALL_RELEASE`]). The caller may
    /// end what it was doing.
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
    /// Per axis: the way it pushed when it stalled (+1, -1), until the targets ask it back.
    stall: Vec<f64>,
    /// Per axis: where it stands when it does not follow its reference (stalled, or at the
    /// gap and coming no nearer), for the solve's pairs ([`Ik::set_measured`]).
    stuck: Vec<bool>,
    /// Per axis, a stop it stalled on (where, and the way it pushed), until it is measured
    /// past it.
    stop: Vec<Option<(f64, f64)>>,
    /// Per axis, the window its last step was given ([`Ik::set_window`]).
    window: Vec<Option<[f64; 2]>>,
    /// Per stalled axis: how far its targets have asked it back since they last asked it on,
    /// radians ([`STALL_RELEASE`]).
    asked_back: Vec<f64>,
    /// The operator's tremor filter ([`Teleop::set_input_filter`]), each hand's state, and the
    /// solve's own clock for it (the sum of its ticks).
    input_filter: Option<OneEuro>,
    filters: [PoseFilter; 2],
    clock_s: f64,
    /// The last step's targets as the solve took them, the operator's hands filtered.
    taken: Targets,
    /// Per axis: where it stood when it stalled, the reference it is held at: stiff there, so
    /// it neither pushes on what stopped it nor sags away from it under a load.
    hold: Vec<f64>,
    /// Per axis: the last reading taken as where the joint is, the ticks since it was read
    /// (a missing reading leaves the joint free to move), and for how many in a row a reading
    /// has been refused as a jump it could not make ([`Teleop::gate_readings`]).
    trusted: Vec<f64>,
    /// Per axis, the last reading refused: a next one that agrees with it is a real move.
    doubted: Vec<f64>,
    /// Per axis, the reference the last tick was given: how fast the command moves.
    reference_before: Vec<f64>,
    since_read: Vec<u32>,
    refused: Vec<u32>,
    /// This tick's readings as the solve takes them.
    gated: Vec<Option<f64>>,
    /// Per axis, the readings smoothed over [`STEADY_S`]: where the gap and a stall are judged.
    steady: Vec<f64>,
    /// Per hand, how its filtered target moves (the filter's estimate, or the caller's).
    rates: [Option<([f64; 3], [f64; 3])>; 2],
    /// Per axis, its jerk cap ([`TeleopConfig::jerk`]).
    jerk: Vec<f64>,
    /// Per hand, its target's shaper ([`crate::shaper`]); the camera's look-at point's; the map
    /// their bounds are read from.
    shapers: [ToolShaper; 2],
    gaze: ToolShaper,
    map: ToolMap,
    /// Per limb, the axes its tool's bounds are carried from; per axis, whether it moves this
    /// tick; the limb's Jacobian.
    shape_axes: Vec<Vec<usize>>,
    shape_moving: Vec<bool>,
    jv: Vec<[f64; 3]>,
    jw: Vec<[f64; 3]>,
    /// Per hand, whether its target went while its arm still moved: its joints come to rest
    /// ([`Target::Rest`]) until they can be held without a jolt ([`holds`]).
    resting: [bool; 2],
    /// Per axis, the velocity the step before the last commanded.
    qdot_before: Vec<f64>,
    /// Whether targets pass their shapers ([`Teleop::set_target_shaping`]).
    shaping: bool,
    /// Per hand, and the camera's look: seconds since its target went while it still comes to
    /// rest ([`COAST_MAX_S`]).
    gone_s: [f64; 2],
    gaze_gone_s: f64,
    /// Whether the camera's look went while the neck still moved: it comes to rest as a hand does.
    gaze_resting: bool,
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
            .chain(config.jerk.keys())
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
        let mut jerk = Vec::with_capacity(names.len());
        for (index, name) in names.iter().enumerate() {
            let cap = match config.speed.get(name) {
                Some(cap) => *cap,
                None if !body.solved().contains(&index) => 0.0,
                None => return Err(IkError::NoSpeed(name.clone())),
            };
            speed.push(cap);
            let a = config
                .accel
                .get(name)
                .copied()
                .unwrap_or(cap / config.ramp_s);
            accel.push(a);
            jerk.push(config.jerk.get(name).copied().unwrap_or(a / config.ramp_s));
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
        // The axes a hand's tool bounds are carried from: its own, not one it shares (a torso,
        // held unless every limb is driven, which the damped least-squares motion would hand a
        // share its low caps never take: the bounds fell to a third and it still saturated).
        let shape_axes = (0..limb_names.len())
            .map(|index| {
                body.limb_axes(index)
                    .iter()
                    .copied()
                    .filter(|&axis| limbs_of[axis].is_power_of_two())
                    .collect()
            })
            .collect();
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
        for (axis, &j) in jerk.iter().enumerate() {
            ik.set_jerk_rule(axis, j);
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
            asked_back: vec![0.0; width],
            stuck: vec![false; width],
            stop: vec![None; width],
            window: vec![None; width],
            hold: vec![0.0; width],
            trusted: vec![f64::NAN; width],
            doubted: vec![f64::NAN; width],
            reference_before: vec![f64::NAN; width],
            since_read: vec![0; width],
            refused: vec![0; width],
            gated: vec![None; width],
            steady: vec![f64::NAN; width],
            input_filter: Some(OneEuro::TOOL),
            filters: [PoseFilter::default(); 2],
            clock_s: 0.0,
            taken: Targets::default(),
            rates: [None; 2],
            jerk,
            shapers: [ToolShaper::default(); 2],
            gaze: ToolShaper::default(),
            map: ToolMap::with_capacity(3 * width),
            shape_axes,
            shape_moving: vec![false; width],
            jv: vec![[0.0; 3]; width],
            jw: vec![[0.0; 3]; width],
            resting: [false; 2],
            qdot_before: vec![0.0; width],
            shaping: true,
            gone_s: [0.0; 2],
            gaze_gone_s: 0.0,
            gaze_resting: false,
        })
    }

    /// Whether each target passes its shaper ([`crate::shaper`], the default); `false` hands the
    /// solve the targets as given, with the filter's estimate of a hand's motion, as before the
    /// shaper: a measurement of the solve's own rules on scripted targets (a hand turning back in
    /// a tick, which the shaper takes jerk-limited). Every shaper starts afresh.
    pub fn set_target_shaping(&mut self, shaping: bool) {
        self.shaping = shaping;
        self.shapers = [ToolShaper::default(); 2];
        self.gaze = ToolShaper::default();
    }

    /// The filter an operator's tool targets pass (default [`OneEuro::TOOL`]); `None` passes
    /// them as given (a measurement of the filter itself). Each hand's filter starts afresh.
    pub fn set_input_filter(&mut self, filter: Option<OneEuro>) {
        self.input_filter = filter;
        self.filters = [PoseFilter::default(); 2];
    }

    /// The last step's targets as the solve took them: the operator's hands with their tremor
    /// filtered out. A ghost draws these to show what the robot follows.
    #[must_use]
    pub const fn taken(&self) -> &Targets {
        &self.taken
    }

    /// `targets` with each operator hand's tremor filtered out (see the module docs).
    fn filtered(&mut self, targets: &Targets, dt: f64) -> Targets {
        if dt.is_finite() && dt > 0.0 {
            self.clock_s += dt;
        }
        let sample_age = targets.age_s.filter(|a| a.is_finite() && *a >= 0.0);
        let lead = sample_age.map_or(0.0, |a| a.min(LEAD_MAX_S));
        let mut out = *targets;
        for (hand, filter) in self.filters.iter_mut().enumerate() {
            let rates = match (
                targets.tcp[hand],
                targets.tool_speed[hand],
                self.input_filter,
            ) {
                (Some(pose), None, Some(params)) => {
                    // Read on the sender's clock when the age is known: a sample held between
                    // arrivals, or bunched after a stall, is the same reading, not a hand that
                    // stopped and leapt (the filter's velocity, carried by the age, turned that
                    // into shake: 9 shiver windows to 4, `examples/trace.rs --network --age`). Past
                    // the station's horizon the age stops growing, so a stalled hand reads as
                    // one that stopped and its carry winds down.
                    let read_s = sample_age.map_or(self.clock_s, |a| self.clock_s - a);
                    let filtered = filter.filter(&params, read_s, pose);
                    let (v, turning) = filter.rates();
                    // Carried forward by the sample's age along the filtered motion: a raw
                    // two-sample velocity carries the tremor too, and over a 150 ms lead turned
                    // a still hand's 1.6 mm shake into 2.8-5.7 mm (`examples/trace.rs --network`).
                    let step = v.map(|x| x * lead);
                    let length = step.iter().map(|x| x * x).sum::<f64>().sqrt();
                    let shorten = if length > LEAD_MAX_M {
                        LEAD_MAX_M / length
                    } else {
                        1.0
                    };
                    let mut carried = filtered;
                    for k in 0..3 {
                        carried.translation_m[k] += step[k] * shorten;
                    }
                    let turn = turning.map(|w| w * lead);
                    let angle = turn.iter().map(|x| x * x).sum::<f64>().sqrt();
                    let turn = if angle > LEAD_MAX_RAD {
                        turn.map(|x| x * LEAD_MAX_RAD / angle)
                    } else {
                        turn
                    };
                    carried.rotation = sx_frames::Quaternion::from_matrix(
                        &sx_frames::rotation_vector_matrix(turn),
                    ) * carried.rotation;
                    out.tcp[hand] = Some(carried);
                    Some(targets.velocity[hand].unwrap_or((v, turning)))
                }
                // A person's hand unfiltered, or a program's goal: followed by the velocity the
                // caller gives (a goal's own path speed), if any.
                (Some(_), _, _) => {
                    filter.reset();
                    targets.velocity[hand]
                }
                (None, _, _) => {
                    filter.reset();
                    None
                }
            };
            // Where the hand is going, for its shaper to follow it.
            self.rates[hand] = rates;
        }
        self.taken = out;
        out
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

    /// Each hand's gripper tool point at `q` (every axis, [`Teleop::model`] order), in
    /// [`HANDS`] order; `None` for a hand this body has no gripper limb for, or a `q` of the
    /// wrong width. What a [`Clutch`] takes hold with.
    pub fn tool_poses(&mut self, q: &[f64]) -> [Option<Pose>; 2] {
        let mut poses = [None; 2];
        if q.len() != self.q.len() {
            return poses;
        }
        self.ik.body().model().poses_into(q, &mut self.frames);
        for (limb, hand) in self.hand_of.iter().enumerate() {
            let Some(hand) = *hand else { continue };
            poses[hand] = Some(self.ik.body().frame(&self.frames, limb).pose());
        }
        poses
    }

    /// The longest one step may take ([`crate::Ik::set_deadline`]): for an evaluation that
    /// scores answers, not the clock of a loaded host.
    pub fn set_deadline(&mut self, budget: std::time::Duration) {
        self.ik.set_deadline(budget);
    }

    /// How far `joint`'s follower lags its command, seconds ([`Ik::set_follower_lag`]): a moving
    /// hand is followed to where it will be that much later. An unknown joint is ignored.
    pub fn set_follower_lag(&mut self, joint: &str, seconds: f64) {
        if let Some(axis) = self
            .ik
            .body()
            .model()
            .axes()
            .iter()
            .position(|a| a.name == joint)
        {
            self.ik.set_follower_lag(axis, seconds);
        }
    }

    /// The most `joint`'s acceleration may change, unit/s^3 ([`Ik::set_jerk_cap`]). An unknown
    /// joint is ignored.
    pub fn set_jerk_cap(&mut self, joint: &str, jerk: f64) {
        if let Some(axis) = self
            .ik
            .body()
            .model()
            .axes()
            .iter()
            .position(|a| a.name == joint)
        {
            self.ik.set_jerk_cap(axis, jerk);
        }
    }

    /// Engage: the next step seeds from the measured joints, starts from standing still and
    /// anchors posture there.
    pub fn reset(&mut self) {
        self.ik.reset();
        self.seed = true;
        self.filters = [PoseFilter::default(); 2];
        self.shapers = [ToolShaper::default(); 2];
        self.gaze = ToolShaper::default();
        self.gone_s = [0.0; 2];
        self.resting = [false; 2];
        self.gaze_gone_s = 0.0;
        self.gaze_resting = false;
    }

    /// Whether a shaped target still has motion to bring to rest: a caller whose targets went
    /// (the operator let go, the intent went stale, a plan ended) keeps stepping with none until
    /// this is false, then holds; each limb is held as soon as it can be without a jolt. A stop at
    /// the jerk bound from 0.5 m/s takes about a quarter of a second.
    #[must_use]
    pub fn settling(&self) -> bool {
        self.shaping
            && (self.shapers.iter().any(ToolShaper::is_active)
                || self.resting.iter().any(|r| *r)
                || self.gaze.is_active()
                || self.gaze_resting)
    }

    /// Each hand's target as the solve last followed it, shaped ([`crate::shaper`]).
    #[must_use]
    pub fn shaped(&self) -> [Option<Pose>; 2] {
        self.shapers.map(|s| s.shaped().map(|s| s.pose))
    }

    /// Each hand's target shaped from where its tool stands ([`crate::shaper`]): `asked` holds
    /// the targets the solve then takes. A hand whose target went comes to rest.
    #[allow(clippy::too_many_lines)] // each hand: its hold, its aim, its bounds, its step
    fn shape(&mut self, targets: &Targets, asked: &mut Targets, available: &[bool], dt: f64) {
        if !self.shaping {
            *asked = *targets;
            for (limb, hand) in self.hand_of.iter().enumerate() {
                if let Some(hand) = *hand {
                    self.ik.set_target_velocity(limb, self.rates[hand]);
                }
            }
            return;
        }
        self.ik.body().model().poses_into(&self.q, &mut self.frames);
        // The limbs driven this tick, as the step will hold them: an axis moves only while every
        // limb it carries is (a torso carries both arms and the neck).
        let mut driven = 0_u64;
        let hands = (0..HANDS.len()).any(|hand| {
            targets.tcp[hand].is_some() || self.shapers[hand].is_active() || self.resting[hand]
        });
        for (index, hand) in self.hand_of.iter().enumerate().take(64) {
            let on = match hand {
                Some(hand) => {
                    targets.tcp[*hand].is_some()
                        || self.shapers[*hand].is_active()
                        || self.resting[*hand]
                }
                None => {
                    Some(index) == self.head
                        && (targets.head.is_some()
                            || hands
                            || self.gaze.is_active()
                            || self.gaze_resting)
                }
            };
            if on {
                driven |= 1 << index;
            }
        }
        for (axis, moving) in self.shape_moving.iter_mut().enumerate() {
            *moving = available[axis]
                && !self.held[axis]
                && self.limbs_of[axis] != 0
                && self.limbs_of[axis] & !driven == 0;
        }
        let rho = self.ik.settings().orientation_m_per_rad;
        let damping = self.ik.settings().damping;
        for (limb, hand) in self.hand_of.iter().enumerate() {
            let Some(hand) = *hand else { continue };
            // Its target went: its joints come to rest each within its own caps (`Target::Rest`).
            if targets.tcp[hand].is_none() {
                let still = holds(
                    &self.ik,
                    limb,
                    &self.step.qdot,
                    &self.qdot_before,
                    &self.jerk,
                    dt,
                );
                come_to_rest(
                    &mut self.resting[hand],
                    &mut self.gone_s[hand],
                    &mut self.shapers[hand],
                    still,
                    dt,
                );
                asked.tcp[hand] = None;
                self.ik.set_target_velocity(limb, None);
                continue;
            }
            self.resting[hand] = false;
            self.gone_s[hand] = 0.0;
            let shaper = &mut self.shapers[hand];
            let Some(pose) = targets.tcp[hand] else {
                continue;
            };
            // Aimed where the target will be at the tick's end, along its own motion: the solve
            // once led a moving hand by a tick this way itself.
            let aim = Aim {
                pose: self.rates[hand].map_or(pose, |(v, w)| {
                    Shaped {
                        pose,
                        velocity: v,
                        turning: w,
                    }
                    .at(dt)
                }),
                motion: self.rates[hand],
                free_axis: targets.free_roll[hand]
                    .then_some(self.approach[limb])
                    .flatten()
                    .map(|axis| pose.rotation.rotate(axis)),
            };
            let body = self.ik.body();
            body.jacobian(&self.frames, limb, &mut self.jv, &mut self.jw);
            let caps = self.ik.caps();
            self.map.of_limb(
                &self.jv,
                &self.jw,
                &self.shape_axes[limb],
                &self.shape_moving,
                (&caps.speed, &caps.accel, &self.jerk),
                rho,
                damping,
            );
            if let Some(speed) = asked.tool_speed[hand] {
                self.map.at_most(speed.m_s, speed.rad_s);
            }
            let frames = &self.frames;
            let next = shaper.step(|| body.frame(frames, limb).pose(), &aim, &self.map, dt);
            // The solve carries its target over the tick by the target's velocity: it is given
            // where the shaped target stands at the tick's start and arrives where it ends, so
            // the tool moves exactly as the shaped target does. Given the end, the tool landed a
            // tick on along the velocity, and the shaped target's jerk steps came through as
            // three times its jerk.
            asked.tcp[hand] = next.map(|s| s.at(-dt));
            self.ik
                .set_target_velocity(limb, next.map(|s| (s.velocity, s.turning)));
        }
    }

    /// Where the camera looks, shaped as a hand's target is: its look direction turned
    /// jerk-limited from where it looks now (a first look, a turn to the operator's gaze, a
    /// hand's engage). The direction is a rotation whose roll about the view is free, bounded
    /// through the camera's turn per joint with that roll taken out (the pointing the neck
    /// does). A look-at point bounded by each neck joint's own caps missed a program's 3 s
    /// settle on a slow neck (the six-joint SO-101 points with several at once); the solve's
    /// look-at carries no target velocity, so the neck still passes a wrist's jerk cap about 2x
    /// engaging (main: 13-20x, the look jumping), and feeding the turn forward made it worse.
    fn shape_gaze(&mut self, head: usize, dt: f64) {
        if !self.shaping {
            return;
        }
        let asked = match self.wanted[head] {
            Some(Target::LookAt { point_m }) => Some(point_m),
            _ => None,
        };
        // Its look went: the neck comes to rest as a hand does.
        if asked.is_none() {
            let still = holds(
                &self.ik,
                head,
                &self.step.qdot,
                &self.qdot_before,
                &self.jerk,
                dt,
            );
            come_to_rest(
                &mut self.gaze_resting,
                &mut self.gaze_gone_s,
                &mut self.gaze,
                still,
                dt,
            );
            self.ik.set_target_velocity(head, None);
            if self.gaze_resting {
                self.wanted[head] = Some(Target::Rest);
            }
            return;
        }
        self.gaze_resting = false;
        self.gaze_gone_s = 0.0;
        let body = self.ik.body();
        let Task::LookAt { forward } = body.limbs[head].task else {
            return;
        };
        let camera = body.frame(&self.frames, head);
        let unit = |v: [f64; 3]| {
            let n = v.iter().map(|x| x * x).sum::<f64>().sqrt();
            (n > 1e-12).then(|| v.map(|x| x / n))
        };
        let Some(local) = unit(forward) else { return };
        let ahead = [0, 1, 2].map(|k| (0..3).map(|j| camera.r[k][j] * local[j]).sum::<f64>());
        // The look's turn per joint as the solve's look-at rows take it: the camera's turn plus
        // the turn its own motion gives the line to the point it looks at, its roll about the
        // view held (the solve asks none). With the roll taken out and the camera's motion left
        // out, the solve moved the neck past the bounds the map gave (1.04x engaging again).
        body.jacobian(&self.frames, head, &mut self.jv, &mut self.jw);
        for (v, w) in self.jv.iter_mut().zip(self.jw.iter_mut()) {
            let line = [
                ahead[1] * v[2] - ahead[2] * v[1],
                ahead[2] * v[0] - ahead[0] * v[2],
                ahead[0] * v[1] - ahead[1] * v[0],
            ];
            for k in 0..3 {
                w[k] += line[k] / GAZE_DISTANCE_M;
            }
            *v = [0.0; 3];
        }
        let caps = self.ik.caps();
        self.map.of_limb(
            &self.jv,
            &self.jw,
            &self.shape_axes[head],
            &self.shape_moving,
            (&caps.speed, &caps.accel, &self.jerk),
            1.0,
            self.ik.settings().damping / self.ik.settings().orientation_m_per_rad.powi(2),
        );
        let start = Pose {
            translation_m: camera.t,
            rotation: Quaternion::from_matrix(&camera.r),
        };
        // The asked direction as the shaped orientation turned the least that lays its view
        // along it.
        let now = self.gaze.shaped().map_or(start, |s| s.pose);
        let aim = asked.and_then(|p| {
            let toward = unit([0, 1, 2].map(|k| p[k] - camera.t[k]))?;
            let looking = now.rotation.rotate(local);
            Some(Aim {
                pose: Pose {
                    translation_m: camera.t,
                    rotation: Quaternion::from_matrix(&sx_frames::rotation_vector_matrix(
                        least_turn(looking, toward),
                    )) * now.rotation,
                },
                motion: None,
                free_axis: Some(toward),
            })
        });
        let Some(aim) = aim else { return };
        let shaped = self.gaze.step(|| start, &aim, &self.map, dt);
        self.wanted[head] = shaped.map(|s| {
            let look = s.pose.rotation.rotate(local);
            Target::LookAt {
                point_m: [0, 1, 2].map(|k| GAZE_DISTANCE_M.mul_add(look[k], camera.t[k])),
            }
        });
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
                (None, _) if self.resting[hand] => Some(Target::Rest),
                (None, _) => None,
                (Some(p), false) => Some(Target::Pose(p)),
                (Some(p), true) => {
                    let axis = self.approach[limb]
                        .ok_or_else(|| IkError::Target(HANDS[hand].to_owned()))?;
                    Some(Target::Axis {
                        translation_m: p.translation_m,
                        axis,
                        toward: p.rotation.rotate(axis),
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
        let from_last = !seeded && targets.tool_speed.iter().any(Option::is_some);
        if from_last {
            self.ik
                .body()
                .model()
                .poses_into(reference, &mut self.frames);
        }
        for (limb, hand) in self.hand_of.iter().enumerate() {
            let speed = hand.and_then(|hand| targets.tool_speed[hand]);
            let from = speed
                .filter(|_| from_last)
                .map(|_| self.ik.body().frame(&self.frames, limb));
            self.ik.set_tool_speed(limb, speed, from);
        }
    }

    /// The reference the step integrates from: seeded from the measurement at engage, at the
    /// measurement for a joint that stalled there ([`STALL`]), and the last command otherwise,
    /// with the step's window within [`REFERENCE_GAP`] of the measurement for a joint the solve
    /// moves (clamped there for one it does not); and which joints do not follow.
    #[allow(clippy::too_many_lines)] // every axis's gap and stall rule, in one pass
    fn take_reference(
        &mut self,
        reference: &[f64],
        measured: &[Option<f64>],
        available: &[bool],
        targets: &Targets,
        seeded: bool,
        dt: f64,
    ) {
        // The limbs driven this tick: only their solved joints can stall (a finger is written
        // from a width, a held or unavailable joint is not moved, an idle arm is held).
        let mut driven = 0_u64;
        for (index, hand) in self.hand_of.iter().enumerate().take(64) {
            let asked = match hand {
                Some(hand) => targets.tcp[*hand].is_some(),
                // The camera turns while a hand is driven or the operator's head is given.
                None => {
                    Some(index) == self.head
                        && (targets.head.is_some() || targets.tcp.iter().any(Option::is_some))
                }
            };
            if asked {
                driven |= 1 << index;
            }
        }
        if seeded {
            self.stop.fill(None);
            self.leashed.fill(None);
            self.stall.fill(0.0);
        }
        let stall_s = STALL.as_secs_f64();
        // The gap and a stall are judged from the readings smoothed over STEADY_S: at 250 Hz and
        // faster a reading's noise of 1.5 mrad moved the gap's edge past the stall's tolerance
        // every few ticks, and a joint pushing a stop never stalled (embodiments#210).
        let blend = 1.0 - (-dt / STEADY_S.as_secs_f64()).exp();
        for (steady, reading) in self.steady.iter_mut().zip(measured) {
            *steady = match reading {
                Some(at) if at.is_finite() && !seeded && steady.is_finite() => {
                    *steady + blend * (at - *steady)
                }
                Some(at) if at.is_finite() => *at,
                _ => f64::NAN,
            };
        }
        for (axis, (slot, (sent, reading))) in self
            .q
            .iter_mut()
            .zip(reference.iter().zip(measured))
            .enumerate()
        {
            self.ik.set_window(axis, f64::NEG_INFINITY, f64::INFINITY);
            let window = self.window[axis].take();
            self.stuck[axis] = false;
            *slot = match reading {
                Some(at) if seeded => *at,
                // A reading that is not a number leaves the reference (as the old rule did,
                // and f64::clamp would panic on it).
                Some(raw) if raw.is_finite() && self.steady[axis].is_finite() => {
                    // The gap is kept from the reading; whether the joint pushes at it, and
                    // whether it comes nearer, from the smoothed one.
                    let raw = *raw;
                    let at = &self.steady[axis];
                    let lead = sent - at;
                    // Whether the last step ended on its window's edge. It starts the stall's
                    // clock; a noisy tick off the edge does not stop it while the smoothed
                    // reading stays at the gap (at 250 Hz and faster the acceleration cap keeps
                    // the step from following the edge's jitter, and a pushing joint never
                    // stalled). A joint is counted on to open no pair only on the edge.
                    let on_edge = window.is_some_and(|[low, high]| {
                        let near = STALL_STILL + 2.0 * READING_JITTER;
                        if *sent - raw > 0.0 {
                            *sent >= high - near
                        } else {
                            *sent <= low + near
                        }
                    });
                    let solved = available[axis]
                        && !self.held[axis]
                        && self.limbs_of[axis] != 0
                        && self.limbs_of[axis] & !driven == 0;
                    if !solved {
                        self.stall[axis] = 0.0;
                        self.stop[axis] = None;
                        self.leashed[axis] = None;
                    } else if self.stall[axis] == 0.0
                        && (on_edge
                            || (self.leashed[axis].is_some()
                                && lead.abs() >= REFERENCE_GAP - AT_THE_GAP))
                    {
                        // At the gap (the step ended on its window's edge, whatever the reading
                        // now: noise in it moves the gap, not the push): pushing on something,
                        // or only lagging? It stalls when the measurement comes no nearer for
                        // the gripper's stall time.
                        let push = lead.signum();
                        self.leashed[axis] = match self.leashed[axis] {
                            Some((from, since))
                                if (at - from) * push <= STALL_STILL + 2.0 * READING_JITTER =>
                            {
                                let since = since + dt;
                                // Never on a reading in doubt: it may be a glitch, and the hold
                                // is where the joint is taken to stand (a glitch on the stall's
                                // tick held the elbow 0.09 rad off its stop, #211 refute).
                                if since >= stall_s && self.refused[axis] == 0 {
                                    self.stall[axis] = push;
                                    self.asked_back[axis] = 0.0;
                                    self.hold[axis] = raw;
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
                    // A stop it stalled on stays where it was until the joint is measured past
                    // it (never by two readings' jitter): released, it pushes the same stop
                    // again.
                    if self.stall[axis] != 0.0 {
                        self.stop[axis] = Some((self.hold[axis], self.stall[axis]));
                    } else if let Some((stop, push)) = self.stop[axis]
                        && (raw - stop) * push > STALL_STILL + 2.0 * READING_JITTER
                    {
                        self.stop[axis] = None;
                    }
                    // Not following: stalled, at the gap and come no nearer for a tick, or led
                    // past a stop it stalled on. Planned where it stands, never counted on to
                    // open a pair.
                    let past_stop = self.stop[axis]
                        .is_some_and(|(stop, push)| (sent - stop) * push > STALL_STILL);
                    if self.stall[axis] != 0.0
                        || past_stop
                        || (on_edge
                            && matches!(self.leashed[axis], Some((_, since)) if since > 0.0))
                    {
                        self.stuck[axis] = true;
                    }
                    if self.stall[axis] != 0.0 {
                        // Stalled: held stiff where it stood, pushing no further.
                        self.hold[axis]
                    } else if solved {
                        // Within the gap of its measurement by the step, not by moving the
                        // reference: a reference moved there skipped every pair the solve
                        // keeps clear (a lagging right arm's commands touched the left gripper,
                        // 2026-11-02).
                        self.ik
                            .set_window(axis, raw - REFERENCE_GAP, raw + REFERENCE_GAP);
                        self.window[axis] = Some([raw - REFERENCE_GAP, raw + REFERENCE_GAP]);
                        *sent
                    } else {
                        // Not moved by the solve this tick (held, idle): within the gap of
                        // where it is, as someone moving it takes it.
                        sent.clamp(raw - REFERENCE_GAP, raw + REFERENCE_GAP)
                    }
                }
                Some(_) | None => *sent,
            };
        }
    }

    /// The readings the solve takes: one that jumps further from the last trusted reading than
    /// the joint could move since ([`FOLLOWER_SPEED`] times its speed cap, and two readings'
    /// jitter) is taken as that reading again, for up to [`GLITCH_HOLD`]; a jump that lasts
    /// longer is real and taken. One bad reading moved the window by the gap and the measured
    /// speed by radians a second: an arm turned 70-140x its acceleration cap and a hand
    /// strayed 12-31 mm (embodiments#210).
    fn gate_readings(
        &mut self,
        reference: &[f64],
        measured: &[Option<f64>],
        gated: &mut [Option<f64>],
        seeded: bool,
        dt: f64,
    ) {
        let hold_ticks = (GLITCH_HOLD.as_secs_f64() / dt).ceil().max(1.0);
        for (axis, (out, reading)) in gated.iter_mut().zip(measured).enumerate() {
            let speed = self
                .ik
                .caps()
                .speed
                .get(axis)
                .copied()
                .unwrap_or(f64::INFINITY);
            let sent = reference[axis];
            let before = std::mem::replace(&mut self.reference_before[axis], sent);
            let trusted = &mut self.trusted[axis];
            let since_read = &mut self.since_read[axis];
            let refused = &mut self.refused[axis];
            let doubted = &mut self.doubted[axis];
            // How far the joint could have moved since it was last read: after a dropout, the
            // reading that catches up is news, not a glitch. (A station sends a stale reading
            // as missing; a joint at rest may read the same count every tick.)
            *since_read = since_read.saturating_add(1);
            *out = match reading {
                Some(at) if at.is_finite() => {
                    // Where a follower can be: toward its command by up to all of its lag, past
                    // it by a few of the command's own steps, or by the speed cap over a
                    // dropout; anything further waits for the next reading to confirm it.
                    let commanded = if before.is_finite() {
                        (sent - before).abs()
                    } else {
                        0.0
                    };
                    // Its lag only toward the command: a jump away from it (off a stop it
                    // pushes, by up to the gap) is no move a follower makes, and taken on the
                    // tick a stall fired it became the hold (embodiments#211 refute).
                    let toward = if trusted.is_finite() && (at - *trusted) * (sent - *trusted) > 0.0
                    {
                        (sent - *trusted).abs()
                    } else {
                        0.0
                    };
                    let missed = f64::from(since_read.saturating_sub(1) + *refused);
                    let reach = toward
                        + FOLLOWER_SPEED * commanded
                        + FOLLOWER_SPEED * speed * dt * missed
                        + GLITCH_FLOOR;
                    let step =
                        FOLLOWER_SPEED * speed * dt * f64::from(*since_read) + 2.0 * READING_JITTER;
                    *since_read = 0;
                    let jump = (at - *trusted).abs();
                    // A jump the next reading confirms is a move: it agrees with the doubted one,
                    // or goes on past it the same way (a joint shoved faster than its cap; held
                    // back until GLITCH_HOLD, it landed in one jump, 10,000x a cap). A glitch
                    // comes back.
                    let further = (at - *trusted) * (*doubted - *trusted) > 0.0
                        && (at - *trusted).abs() >= (*doubted - *trusted).abs() - step;
                    let confirmed = (at - *doubted).abs() <= step || further;
                    if seeded
                        || !trusted.is_finite()
                        || jump <= reach
                        || confirmed
                        || f64::from(*refused) >= hold_ticks
                    {
                        *trusted = *at;
                        *refused = 0;
                        *doubted = f64::NAN;
                        Some(*at)
                    } else {
                        *refused += 1;
                        *doubted = *at;
                        Some(*trusted)
                    }
                }
                Some(_) | None => *reading,
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
    #[allow(clippy::too_many_lines)] // one tick, in its order
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
        if !(dt.is_finite() && dt > 0.0) {
            return Err(IkError::Tick(dt));
        }
        // A target that is not a number is refused before it reaches the filter or the shaper,
        // as the solve refused it before them: one infinite target once refused every later step.
        let pose_finite = |p: &Pose| {
            p.translation_m.iter().all(|x| x.is_finite())
                && [p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w]
                    .iter()
                    .all(|x| x.is_finite())
        };
        for (hand, name) in HANDS.iter().enumerate() {
            if targets.tcp[hand].is_some_and(|p| !pose_finite(&p))
                || targets.gripper_width_m[hand].is_some_and(|w| !w.is_finite())
            {
                return Err(IkError::Target((*name).to_owned()));
            }
        }
        if targets.head.is_some_and(|p| !pose_finite(&p)) {
            return Err(IkError::Target("head".to_owned()));
        }
        // A motion that is not a number is no motion (as the solve took it before the shaper).
        let mut cleaned = *targets;
        for velocity in &mut cleaned.velocity {
            if velocity.is_some_and(|(v, w)| !v.iter().chain(&w).all(|x| x.is_finite())) {
                *velocity = None;
            }
        }
        let targets = &cleaned;
        // Filtered once the step is taken: a refused step leaves the filter and `taken` as
        // they were.
        let targets = &self.filtered(targets, dt);
        let mut asked = *targets;
        if self.seed {
            self.shapers = [ToolShaper::default(); 2];
            self.resting = [false; 2];
            self.gaze = ToolShaper::default();
            self.gaze_resting = false;
        }
        // The reference, seeded from the measurement at engage and held within the gap of it.
        let seeded = self.seed;
        let mut gated = std::mem::take(&mut self.gated);
        self.gate_readings(reference, measured, &mut gated, seeded, dt);
        let measured = &gated[..];
        self.take_reference(reference, measured, available, &asked, seeded, dt);
        self.seed = false;
        self.shape(targets, &mut asked, available, dt);
        let targets = &asked;
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
                let forward = gaze.rotation.rotate([1.0, 0.0, 0.0]);
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
        if let Some(head) = self.head {
            self.shape_gaze(head, dt);
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
        // A stalled joint stands where it stopped (its reference is there): the solve moves the
        // rest of the arm around it, and its own velocity is zero, so a release starts it from
        // rest within its caps.
        for (avail, push) in self.available.iter_mut().zip(&self.stall) {
            if *push != 0.0 {
                *avail = false;
            }
        }
        // Pairs are kept clear with a joint that does not follow where it stands, not where
        // its reference says: planned from the reference, a pair beside an elbow pushing a
        // stop was kept clear on a move the stop never let it make, and the right gripper
        // reaching for the left one touched it (2026-11-02).
        let set = self.ik.set_measured(measured, &self.stuck);
        self.gated = gated;
        set?;
        let solved = self.ik.step(&self.q, &self.available, &self.wanted, dt)?;
        self.qdot_before.copy_from_slice(&self.step.qdot);
        self.step.q.copy_from_slice(&solved.q);
        self.step.qdot.copy_from_slice(&solved.qdot);
        self.step.limbs.clone_from(&solved.limbs);
        self.step.solve = solved.solve;
        // It lets go once the targets have asked it back by STALL_RELEASE since they last asked
        // it on. A step's own move is no sign: one-way blocked, the solve left it a sliver
        // inside its block (1e-9 rad) whichever way the targets pressed, and releasing on that
        // pushed it into what stopped it again while the hand still asked that way
        // (embodiments#198).
        for (axis, (push, back)) in self.stall.iter_mut().zip(&mut self.asked_back).enumerate() {
            if *push != 0.0 {
                let on = self.ik.asked(axis) * *push;
                *back = if on >= 0.0 { 0.0 } else { *back - on };
                if *back > STALL_RELEASE {
                    *push = 0.0;
                }
            }
            self.step.stalled[axis] = *push != 0.0;
        }
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
