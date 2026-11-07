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
//!   (a handheld rig) is no hand;
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
use crate::teleop::{LimbConfig, MAX_HANDS};
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
                        .map(|(k, sign)| {
                            let mut axis = [0.0; 3];
                            axis[k] = sign;
                            axis
                        })
                        .unwrap_or(derived);
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
            });
        }
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
            });
        }
        Ok((body, limbs))
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
    // Rounded to a picometre's worth, so a quarter turn's last bits do not reach the config.
    let forward = unit(forward.map(|x| (x * 1e12).round() / 1e12))?;
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
        }
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
