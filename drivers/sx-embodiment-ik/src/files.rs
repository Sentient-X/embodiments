//! A body as the embodiment layer describes it, and every body this crate compiles in.
//!
//! [`BodyFiles`] are the three documents a render pack carries (`render_packs/<body>/`,
//! `sx_embodiments.render_pack`): the composed description, its collision proxies and
//! `pack.json`. From them alone the solve's limbs are derived, the same rule for every body,
//! so no consumer lists a body's hands or camera by hand:
//!
//! - **one hand per tool** (`pack.json` `tools`, in its order) that a joint carries: named by
//!   the side of the body it is on where the body places one (an operator's `left` and `right`
//!   hand), else by its component (a single arm). Its frame is its tool point
//!   (`tcp_candidates`, the link at its jaws' grasp point) where one rides with it, else the
//!   tool's own frame. Its approach axis is its jaws' (`Body::tool`), restated in that frame; on
//!   a tool point, the point's own axis nearest it (within [`APPROACH_SNAP_RAD`]): a tool point
//!   link is placed on the jaws by the description's author, along the way they close in, where
//!   the derived approach runs from the mount to the grasp point (4 degrees off on the static
//!   upper body, whose grasp point sits 1 cm above its mount's axis). A tool no joint carries
//!   (a handheld rig) is no hand. Its **arm** ([`ArmLinks`], what an operator's shoulder,
//!   elbow and wrist are matched to) is read from the joints that move only it, root first
//!   (a torso both arms ride is left out): with `n` of them, at least five, the shoulder is the
//!   link the second moves, the wrist the link the second-last moves and the elbow the link the
//!   one midway between moves (`(n - 1) / 2`): a 7-joint arm's 2, 4 and 6 (the static upper
//!   body's shoulder roll, elbow pitch and wrist pitch), a 6-joint arm's shoulder lift, elbow and
//!   second wrist joint, a 5-joint arm's shoulder lift, elbow flex and wrist flex. An arm of
//!   fewer joints has no elbow to place;
//! - **the head's camera as its look** ([`HEAD`]): the camera frames (`frames` role `view`)
//!   riding rigidly with the head chain's last link (role `head`), looked through from the
//!   deepest link they all ride rigidly with, along their view (+z of an optical frame). A body
//!   with no head chain, or no camera on it, has no look;
//! - its **dynamics** from the description, accepting a fit keyed to any earlier description
//!   the asset tree keeps (`superseded_descriptions`).
//!
//! [`registered`] answers the bodies whose render packs are committed (what a station or a
//! headset build compiles in, `tools/render_packs.py`); a test builds any registry body from
//! `tests/fixtures/registry.json` through the same [`BodyFiles`].

use serde::Deserialize;
use std::collections::BTreeMap;

use crate::IkError;
use crate::body::{Body, Limb, Task};
use crate::teleop::{ArmLinks, LimbConfig, MAX_HANDS};
use crate::tool::Grip;

/// The look limb's name: the head's camera (`sx_embodiments.frames.FrameRole.HEAD`).
pub const HEAD: &str = "head";

/// How far a tool point's own axis may lie from its jaws' derived approach and still be taken
/// as the approach, radians.
pub const APPROACH_SNAP_RAD: f64 = 0.2;

include!(concat!(env!("OUT_DIR"), "/registered.rs"));

/// One body's documents: its description, collision proxies and `pack.json`, and the
/// earlier descriptions its dynamics fits may still be keyed to.
#[derive(Clone, Copy, Debug)]
pub struct BodyFiles<'a> {
    /// The registry's name for the body (`sentient-static-upper-body`).
    pub embodiment: &'a str,
    pub urdf: &'a str,
    pub collision: &'a str,
    pub pack: &'a str,
    pub superseded: &'a [&'a str],
}

/// The body named `embodiment` whose render pack this crate compiles in; `None` for a body
/// with none (its solve is not described here).
#[must_use]
pub fn registered(embodiment: &str) -> Option<BodyFiles<'static>> {
    REGISTERED
        .iter()
        .find(|files| files.embodiment == embodiment)
        .copied()
}

/// Every body whose render pack this crate compiles in, by name.
#[must_use]
pub fn every_registered() -> &'static [BodyFiles<'static>] {
    REGISTERED
}

#[derive(Deserialize)]
struct FrameDocument {
    name: String,
    #[serde(default)]
    roles: Vec<String>,
}

#[derive(Deserialize)]
struct PackDocument {
    #[serde(default)]
    tcp_candidates: BTreeMap<String, String>,
    #[serde(default)]
    frames: Vec<FrameDocument>,
    #[serde(default)]
    ready: BTreeMap<String, f64>,
}

impl BodyFiles<'_> {
    /// The body ready to solve, with its tools and the limbs [`BodyFiles::limbs`] derives.
    ///
    /// # Errors
    ///
    /// [`IkError`] for documents that do not parse or do not fit each other, a body with more
    /// hands than [`MAX_HANDS`], or two hands of one name.
    pub fn body(&self) -> Result<Body, IkError> {
        let (body, limbs) = self.solved()?;
        let limbs: Vec<Limb> = limbs
            .iter()
            .map(|limb| Limb {
                name: limb.name.clone(),
                frame: limb.frame.clone(),
                task: limb
                    .look_forward
                    .map_or(Task::Pose, |forward| Task::LookAt { forward }),
            })
            .collect();
        body.with_limbs(&limbs)
    }

    /// What the solve drives on this body: one pose limb per hand, then the head's look (see
    /// the module docs).
    ///
    /// # Errors
    ///
    /// As [`BodyFiles::body`].
    pub fn limbs(&self) -> Result<Vec<LimbConfig>, IkError> {
        self.solved().map(|(_, limbs)| limbs)
    }

    /// The body's own home: each chain joint's declared work-ready value
    /// (`sx_embodiments.parts.ChainSpec.ready`, elbows down on an arm), by joint name. It is the
    /// posture a teleop solve resolves the arms' redundancy toward ([`TeleopConfig::posture`])
    /// where the robot commissioned no Ready pose of its own, so a solve always has a home and
    /// never anchors to wherever the arm stood at engage. Empty for a body whose chains declare
    /// none.
    ///
    /// [`TeleopConfig::posture`]: crate::teleop::TeleopConfig::posture
    ///
    /// # Errors
    ///
    /// A `pack.json` that does not parse, or a value that is not finite.
    pub fn ready(&self) -> Result<BTreeMap<String, f64>, IkError> {
        let pack: PackDocument =
            serde_json::from_str(self.pack).map_err(|e| IkError::Tools(e.to_string()))?;
        if let Some((joint, _)) = pack.ready.iter().find(|(_, value)| !value.is_finite()) {
            return Err(IkError::Tools(format!("ready {joint:?} is not finite")));
        }
        Ok(pack.ready)
    }

    /// The description's dynamics in the world's `gravity`, accepting a fit keyed to any of
    /// its [`BodyFiles::superseded`] descriptions.
    ///
    /// # Errors
    ///
    /// A description that does not parse.
    pub fn dynamics(
        &self,
        gravity: [f64; 3],
    ) -> Result<sx_embodiment_drivers::Dynamics, sx_embodiment_drivers::kinematics::TreeError> {
        sx_embodiment_drivers::Dynamics::of(self.urdf, gravity)?.superseding(self.superseded)
    }

    fn solved(&self) -> Result<(Body, Vec<LimbConfig>), IkError> {
        let body = Body::new(self.urdf, self.collision, &[])?.with_tools(self.pack)?;
        let pack: PackDocument =
            serde_json::from_str(self.pack).map_err(|e| IkError::Tools(e.to_string()))?;
        let model = body.model();
        let mut limbs: Vec<LimbConfig> = Vec::new();
        for tool in body.tools() {
            let own = model.link(&tool.frame).ok_or_else(|| {
                IkError::Tools(format!("{} names no link {:?}", tool.component, tool.frame))
            })?;
            if model.above(own).is_empty() {
                continue; // no joint carries it: a handheld rig
            }
            let name = tool.side.clone().unwrap_or_else(|| tool.component.clone());
            if limbs.iter().any(|limb| limb.name == name) || name == HEAD {
                return Err(IkError::Tools(format!("two hands are named {name:?}")));
            }
            let point = tool
                .side
                .as_ref()
                .and_then(|side| pack.tcp_candidates.get(side))
                .filter(|point| {
                    model
                        .link(point)
                        .is_some_and(|at| model.rigidly_below(own, at))
                });
            let frame = point.cloned().unwrap_or_else(|| tool.frame.clone());
            let approach = body
                .tool(&frame)
                .and_then(|tool| match tool.grip {
                    Grip::Jaws(jaws) => unit(jaws.approach_axis),
                    Grip::None { .. } => None,
                })
                .map(|derived| {
                    if point.is_none() {
                        return derived;
                    }
                    let nearest = (0..3)
                        .flat_map(|k| [1.0, -1.0].map(|sign| (k, sign)))
                        .max_by(|a, b| (a.1 * derived[a.0]).total_cmp(&(b.1 * derived[b.0])))
                        .map_or(derived, |(k, sign)| {
                            let mut axis = [0.0; 3];
                            axis[k] = sign;
                            axis
                        });
                    let cos: f64 = (0..3).map(|k| nearest[k] * derived[k]).sum();
                    if cos >= APPROACH_SNAP_RAD.cos() {
                        nearest
                    } else {
                        derived
                    }
                });
            limbs.push(LimbConfig {
                name,
                frame,
                look_forward: None,
                approach,
                arm: None,
            });
        }
        name_arms(model, &mut limbs);
        if limbs.len() > MAX_HANDS {
            return Err(IkError::Tools(format!(
                "{} hands; the solve carries at most {MAX_HANDS}",
                limbs.len()
            )));
        }
        if let Some((frame, forward)) = look(&body, &pack.frames) {
            limbs.push(LimbConfig {
                name: HEAD.to_owned(),
                frame,
                look_forward: Some(forward),
                approach: None,
                arm: None,
            });
        }
        Ok((body, limbs))
    }
}

/// Each hand's arm, from the joints only it moves (see the module docs).
fn name_arms(model: &crate::Model, limbs: &mut [LimbConfig]) {
    let carried: Vec<Vec<usize>> = limbs
        .iter()
        .map(|limb| {
            model.link(&limb.frame).map_or_else(Vec::new, |link| {
                model.above(link).iter().map(|term| term.joint).collect()
            })
        })
        .collect();
    for (index, limb) in limbs.iter_mut().enumerate() {
        let own: Vec<usize> = carried[index]
            .iter()
            .copied()
            .filter(|joint| {
                carried
                    .iter()
                    .enumerate()
                    .all(|(other, joints)| other == index || !joints.contains(joint))
            })
            .collect();
        let n = own.len();
        if n >= 5 {
            let name = |at: usize| model.links()[model.child(own[at])].clone();
            limb.arm = Some(ArmLinks {
                shoulder: name(1),
                elbow: name((n - 1) / 2),
                wrist: name(n - 2),
            });
        }
    }
}

/// The head's look: the link its cameras all ride rigidly with, deepest first, and their view
/// in it. `None` without a head chain or a camera on it.
fn look(body: &Body, frames: &[FrameDocument]) -> Option<(String, [f64; 3])> {
    let model = body.model();
    let with_role = |role: &str| {
        frames
            .iter()
            .filter(|frame| frame.roles.iter().any(|r| r == role))
            .filter_map(|frame| model.link(&frame.name))
            .collect::<Vec<_>>()
    };
    let heads = with_role("head");
    let views: Vec<usize> = with_role("view")
        .into_iter()
        .filter(|&view| heads.iter().any(|&head| model.rigidly_below(head, view)))
        .collect();
    let first = *views.first()?;
    let mut at = first;
    while !views.iter().all(|&view| model.rigidly_below(at, view)) {
        match model.parent(at) {
            Some((parent, true)) => at = parent,
            _ => return None,
        }
    }
    let q: Vec<f64> = model
        .axes()
        .iter()
        .map(|axis| axis.limits.map_or(0.0, |[lo, hi]| 0.0_f64.clamp(lo, hi)))
        .collect();
    let poses = model.poses(&q);
    let (frame, view) = (poses.links[at], poses.links[first]);
    // The view's +z (optical) in the root frame, then in the look's frame.
    let along = [0, 1, 2].map(|k| view.r[k][2]);
    let forward = [0, 1, 2].map(|j| (0..3).map(|k| frame.r[k][j] * along[k]).sum::<f64>());
    // Rounded to a picometre's worth, so a quarter turn's last bits do not reach the config (and
    // no negative zero does).
    let forward = unit(forward.map(|x| (x * 1e12).round() / 1e12))?.map(|x| x + 0.0);
    Some((model.links()[at].clone(), forward))
}

fn unit(v: [f64; 3]) -> Option<[f64; 3]> {
    let length = v.iter().map(|x| x * x).sum::<f64>().sqrt();
    (length.is_finite() && length > 1e-9).then(|| v.map(|x| x / length))
}

#[cfg(test)]
mod tests {
    use super::*;

    const STATIC: &str = "sentient-static-upper-body";

    /// The static upper body's limbs, derived from its pack alone, are the ones its solve was
    /// written for by hand until 2026-11-07: both grippers' tool points by side, the ZED looking
    /// out of its lenses from the SO-101 neck, each hand approaching along its tool point's +x.
    /// The body's own home is its registry chains' declared Ready: on the static upper body
    /// every arm and torso joint, elbows bent down (0.3 rad off the stop) and the shoulders
    /// rolled out, so a solve with no commissioned Ready still has one; a body whose chains
    /// declare none has an empty one.
    #[test]
    fn the_static_upper_bodys_home_is_its_registry_ready() {
        let ready = registered(STATIC)
            .expect("committed")
            .ready()
            .expect("ready");
        for side in ["left", "right"] {
            for joint in [
                "shoulder_pitch",
                "shoulder_roll",
                "upper_arm_yaw",
                "elbow_pitch",
                "forearm_yaw",
                "wrist_pitch",
                "wrist_roll",
            ] {
                assert!(
                    ready.contains_key(&format!("{side}_{joint}_joint")),
                    "{side} {joint}"
                );
            }
        }
        for (joint, value) in [
            ("left_elbow_pitch_joint", -0.3),
            ("right_elbow_pitch_joint", 0.3),
            ("torso_joint", 0.0),
        ] {
            assert!(
                (ready[joint] - value).abs() < 1e-12,
                "{joint}: {}",
                ready[joint]
            );
        }
        for files in every_registered() {
            let ready = files.ready().expect("parses");
            let model = files.body().expect("body");
            let axes: Vec<&str> = model
                .model()
                .axes()
                .iter()
                .map(|a| a.name.as_str())
                .collect();
            assert!(
                ready.keys().all(|joint| axes.contains(&joint.as_str())),
                "{}: a ready joint the description lacks",
                files.embodiment
            );
        }
    }

    #[test]
    fn the_static_upper_bodys_limbs_come_from_its_pack() {
        let limbs = registered(STATIC)
            .expect("committed")
            .limbs()
            .expect("limbs");
        let named: Vec<(&str, &str)> = limbs
            .iter()
            .map(|l| (l.name.as_str(), l.frame.as_str()))
            .collect();
        assert_eq!(
            named,
            [
                ("left", "left_gripper_tcp"),
                ("right", "right_gripper_tcp"),
                (HEAD, "ZED_camera"),
            ]
        );
        assert_eq!(limbs[2].look_forward, Some([1.0, 0.0, 0.0]));
        for hand in &limbs[..2] {
            assert_eq!(hand.approach, Some([1.0, 0.0, 0.0]));
            assert!(hand.look_forward.is_none());
            // The arm an operator's is matched to: the links robot-bench's elbow-swivel
            // measure reads (shoulder roll, elbow pitch, wrist pitch).
            let side = &hand.name;
            assert_eq!(
                hand.arm,
                Some(ArmLinks {
                    shoulder: format!("{side}_shoulder_roll_link"),
                    elbow: format!("{side}_elbow_pitch_link"),
                    wrist: format!("{side}_wrist_pitch_link"),
                })
            );
        }
        assert!(limbs[2].arm.is_none());
        let body = registered(STATIC).expect("committed").body().expect("body");
        assert_eq!(body.limbs(), ["left", "right", HEAD]);
        assert!(
            registered(STATIC)
                .expect("committed")
                .dynamics(sx_embodiment_drivers::UPRIGHT)
                .is_ok()
        );
    }

    /// Every registry body with a tool a joint carries gets one hand per such tool, named by its
    /// side or component, each a link of its description, at most [`MAX_HANDS`]; a body whose
    /// tools no joint carries gets none; a look only where a camera rides a head chain.
    #[test]
    fn every_registry_body_derives_its_hands_and_look() {
        let documents: Vec<serde_json::Value> =
            serde_json::from_str(include_str!("../tests/fixtures/registry.json")).expect("fixture");
        let mut with_hands = 0;
        for document in &documents {
            let name = document["embodiment"].as_str().expect("name");
            let (collision, pack) = (
                document["collision"].to_string(),
                document["pack"].to_string(),
            );
            let files = BodyFiles {
                embodiment: name,
                urdf: document["urdf"].as_str().expect("urdf"),
                collision: &collision,
                pack: &pack,
                superseded: &[],
            };
            let limbs = files.limbs().unwrap_or_else(|e| panic!("{name}: {e}"));
            let body = files.body().unwrap_or_else(|e| panic!("{name}: {e}"));
            let hands: Vec<&LimbConfig> =
                limbs.iter().filter(|l| l.look_forward.is_none()).collect();
            assert!(hands.len() <= MAX_HANDS, "{name}");
            for (index, limb) in limbs.iter().enumerate() {
                assert!(
                    body.model().link(&limb.frame).is_some(),
                    "{name}: {}",
                    limb.frame
                );
                assert!(
                    !body.limb_axes(index).is_empty(),
                    "{name}: {} moves",
                    limb.name
                );
            }
            let sided = document["pack"]["tools"]
                .as_array()
                .expect("tools")
                .iter()
                .filter(|tool| tool.get("side").is_some())
                .count();
            if sided == 2 && hands.len() == 2 {
                let mut names: Vec<&str> = hands.iter().map(|h| h.name.as_str()).collect();
                names.sort_unstable();
                assert_eq!(names, ["left", "right"], "{name}");
            }
            // An arm of five joints or more names its shoulder, elbow and wrist, three distinct
            // links in order down to the tool, which a solve builds with.
            for hand in &hands {
                let link = body.model().link(&hand.frame).expect("frame");
                let carried = body.model().above(link).len();
                if let Some(arm) = &hand.arm {
                    let at = |name: &str| body.model().link(name).expect("arm link");
                    let (s, e, w) = (at(&arm.shoulder), at(&arm.elbow), at(&arm.wrist));
                    assert!(
                        s != e
                            && e != w
                            && body.model().descends(e, s)
                            && body.model().descends(w, e)
                            && body.model().descends(link, w),
                        "{name}: {arm:?}"
                    );
                    println!(
                        "{name} {}: {} / {} / {}",
                        hand.name, arm.shoulder, arm.elbow, arm.wrist
                    );
                } else {
                    println!("{name} {}: no arm ({carried} joints carry it)", hand.name);
                }
            }
            with_hands += usize::from(!hands.is_empty());
            let looks = limbs.len() - hands.len();
            assert!(looks <= 1, "{name}");
            if looks == 1 {
                assert_eq!(
                    name, STATIC,
                    "only the static upper body turns a camera on a head"
                );
            }
        }
        assert!(with_hands >= 20, "{with_hands} bodies have hands");
    }
}
