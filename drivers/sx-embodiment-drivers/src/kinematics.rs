//! A mount's pose at a joint state: the chain a media plan renders, evaluated.
//!
//! `sx_embodiments.media` renders, for each camera view, every joint from the description's
//! root down to the camera's mount link: its type, its `parent_from_child` origin at zero, its
//! unit axis, and, for a mimic joint, the independent joint it follows. [`Chain::pose`] composes
//! that chain at given joint positions exactly as `sx_embodiments.kinematics.Kinematics.pose`
//! does (row-major 4x4 in metres, the same rotation about an axis), so a station places a camera
//! that rides moving joints without reading a description; the crate's `media-poses.json`
//! fixture holds the two equal.

use serde::Deserialize;
use thiserror::Error;

use crate::media::Pose;

/// A row-major 4x4 rigid transform in metres.
pub type Transform = [f64; 16];

const IDENTITY: Transform = [
    1.0, 0.0, 0.0, 0.0, //
    0.0, 1.0, 0.0, 0.0, //
    0.0, 0.0, 1.0, 0.0, //
    0.0, 0.0, 0.0, 1.0,
];

/// How one joint moves its child.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum JointType {
    Fixed,
    Revolute,
    Continuous,
    Prismatic,
}

/// The independent joint a mimic joint follows: `position = multiplier * leader + offset`.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Follows {
    pub joint: String,
    pub multiplier: f64,
    pub offset: f64,
}

/// One joint on the chain from the description's root down to a mount link.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChainStep {
    pub joint: String,
    #[serde(rename = "type")]
    pub kind: JointType,
    /// `parent_from_child` at zero.
    pub origin: Vec<f64>,
    /// Unit length.
    pub axis: [f64; 3],
    pub follows: Option<Follows>,
}

/// A chain cannot be placed at the positions given.
#[derive(Debug, Error, PartialEq)]
pub enum PlacementError {
    #[error("no position for joint {0}")]
    Missing(String),
    #[error("joint {0} has a non-finite position")]
    NonFinite(String),
}

/// Every joint from the description's root down to one mount link, root first.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(transparent)]
pub struct Chain(pub Vec<ChainStep>);

impl Chain {
    /// Whether every step is a rigid transform, an axis and (for a mimic) a finite rule.
    #[must_use]
    pub fn is_well_formed(&self) -> bool {
        self.0.iter().all(|step| {
            let norm = step.axis.iter().map(|v| v * v).sum::<f64>().sqrt();
            step.origin.len() == 16
                && step.origin.iter().all(|v| v.is_finite())
                && step.origin[12..] == [0.0, 0.0, 0.0, 1.0]
                && is_rotation(&step.origin)
                && (step.kind == JointType::Fixed || (norm - 1.0).abs() < 1e-9)
                && step
                    .follows
                    .as_ref()
                    .is_none_or(|f| f.multiplier.is_finite() && f.offset.is_finite())
        })
    }

    /// `root_from_mount` at the positions `position` answers for each independent joint.
    ///
    /// # Errors
    ///
    /// Returns [`PlacementError`] when a moving joint (or the leader a mimic follows) has no
    /// finite position.
    pub fn transform(
        &self,
        position: impl Fn(&str) -> Option<f64>,
    ) -> Result<Transform, PlacementError> {
        let mut result = IDENTITY;
        for step in &self.0 {
            let origin: Transform = step.origin.as_slice().try_into().unwrap_or(IDENTITY);
            let local = match step.kind {
                JointType::Fixed => origin,
                JointType::Revolute | JointType::Continuous => {
                    compose(&origin, &about(step.axis, value(step, &position)?))
                }
                JointType::Prismatic => {
                    compose(&origin, &along(step.axis, value(step, &position)?))
                }
            };
            result = compose(&result, &local);
        }
        Ok(result)
    }

    /// The mount's pose in the root frame at `position`, as the plan's [`Pose`] carries one.
    ///
    /// # Errors
    ///
    /// As [`Chain::transform`].
    pub fn pose(&self, position: impl Fn(&str) -> Option<f64>) -> Result<Pose, PlacementError> {
        Ok(pose_of(&self.transform(position)?))
    }

    /// The independent joints whose positions place the mount, chain order, root first.
    #[must_use]
    pub fn joints(&self) -> Vec<&str> {
        let mut joints: Vec<&str> = Vec::new();
        for step in &self.0 {
            if step.kind == JointType::Fixed {
                continue;
            }
            let needed = step
                .follows
                .as_ref()
                .map_or(step.joint.as_str(), |f| f.joint.as_str());
            if !joints.contains(&needed) {
                joints.push(needed);
            }
        }
        joints
    }
}

/// Whether the upper 3x3 of a row-major transform is a rotation: orthonormal, determinant one.
fn is_rotation(m: &[f64]) -> bool {
    let r = |row: usize, column: usize| m[row * 4 + column];
    let orthonormal = (0..3).all(|a| {
        (0..3).all(|b| {
            let dot: f64 = (0..3).map(|k| r(k, a) * r(k, b)).sum();
            (dot - if a == b { 1.0 } else { 0.0 }).abs() < 1e-9
        })
    });
    let determinant = r(0, 0) * r(1, 1).mul_add(r(2, 2), -(r(1, 2) * r(2, 1)))
        - r(0, 1) * r(1, 0).mul_add(r(2, 2), -(r(1, 2) * r(2, 0)))
        + r(0, 2) * r(1, 0).mul_add(r(2, 1), -(r(1, 1) * r(2, 0)));
    orthonormal && (determinant - 1.0).abs() < 1e-9
}

fn value(step: &ChainStep, position: &impl Fn(&str) -> Option<f64>) -> Result<f64, PlacementError> {
    let (joint, multiplier, offset) = step
        .follows
        .as_ref()
        .map_or((step.joint.as_str(), 1.0, 0.0), |f| {
            (f.joint.as_str(), f.multiplier, f.offset)
        });
    let x = position(joint).ok_or_else(|| PlacementError::Missing(joint.to_owned()))?;
    if !x.is_finite() {
        return Err(PlacementError::NonFinite(joint.to_owned()));
    }
    Ok(multiplier.mul_add(x, offset))
}

/// The product of two row-major transforms, `a` then `b`.
#[must_use]
pub fn compose(a: &Transform, b: &Transform) -> Transform {
    let mut out = [0.0; 16];
    for row in 0..4 {
        for column in 0..4 {
            out[4 * row + column] = (0..4).map(|k| a[4 * row + k] * b[4 * k + column]).sum();
        }
    }
    out
}

#[allow(clippy::many_single_char_names)] // the axis-angle formula's own names
fn about([x, y, z]: [f64; 3], angle: f64) -> Transform {
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    [
        t * x * x + c,
        t * x * y - s * z,
        t * x * z + s * y,
        0.0, //
        t * x * y + s * z,
        t * y * y + c,
        t * y * z - s * x,
        0.0, //
        t * x * z - s * y,
        t * y * z + s * x,
        t * z * z + c,
        0.0, //
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

fn along([x, y, z]: [f64; 3], distance: f64) -> Transform {
    [
        1.0,
        0.0,
        0.0,
        x * distance, //
        0.0,
        1.0,
        0.0,
        y * distance, //
        0.0,
        0.0,
        1.0,
        z * distance, //
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

/// A rigid transform as translation and unit quaternion (x, y, z, w) with `w >= 0`, the
/// conversion `sx_embodiments.media` renders a fixed mount's pose with.
#[must_use]
#[allow(clippy::many_single_char_names)] // the matrix-to-quaternion formula's own names
pub fn pose_of(m: &Transform) -> Pose {
    let at = |row: usize, column: usize| m[row * 4 + column];
    let trace = at(0, 0) + at(1, 1) + at(2, 2);
    let (mut w, mut x, mut y, mut z);
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        w = 0.25 * s;
        x = (at(2, 1) - at(1, 2)) / s;
        y = (at(0, 2) - at(2, 0)) / s;
        z = (at(1, 0) - at(0, 1)) / s;
    } else if at(0, 0) > at(1, 1) && at(0, 0) > at(2, 2) {
        let s = (1.0 + at(0, 0) - at(1, 1) - at(2, 2)).sqrt() * 2.0;
        w = (at(2, 1) - at(1, 2)) / s;
        x = 0.25 * s;
        y = (at(0, 1) + at(1, 0)) / s;
        z = (at(0, 2) + at(2, 0)) / s;
    } else if at(1, 1) > at(2, 2) {
        let s = (1.0 + at(1, 1) - at(0, 0) - at(2, 2)).sqrt() * 2.0;
        w = (at(0, 2) - at(2, 0)) / s;
        x = (at(0, 1) + at(1, 0)) / s;
        y = 0.25 * s;
        z = (at(1, 2) + at(2, 1)) / s;
    } else {
        let s = (1.0 + at(2, 2) - at(0, 0) - at(1, 1)).sqrt() * 2.0;
        w = (at(1, 0) - at(0, 1)) / s;
        x = (at(0, 2) + at(2, 0)) / s;
        y = (at(1, 2) + at(2, 1)) / s;
        z = 0.25 * s;
    }
    if w < 0.0 {
        (w, x, y, z) = (-w, -x, -y, -z);
    }
    Pose {
        translation_m: [at(0, 3), at(1, 3), at(2, 3)],
        rotation_xyzw: [x, y, z, w],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::media::MediaPlan;
    use std::collections::HashMap;

    const PLANS: &str = include_str!("../tests/fixtures/media-plans.json");
    const POSES: &str = include_str!("../tests/fixtures/media-poses.json");

    fn plan(key: &str) -> MediaPlan {
        let fixtures: serde_json::Value = serde_json::from_str(PLANS).expect("plans");
        let document = fixtures[key].to_string();
        let instance = fixtures[key]["instance"]
            .as_str()
            .expect("instance")
            .to_owned();
        MediaPlan::parse(&document, &instance).expect("plan")
    }

    #[derive(Deserialize)]
    struct Sample {
        track: String,
        camera: String,
        positions: HashMap<String, f64>,
        translation_m: [f64; 3],
        rotation_xyzw: [f64; 4],
    }

    #[test]
    fn the_chain_places_a_moving_camera_where_the_description_does() {
        let samples: HashMap<String, Vec<Sample>> = serde_json::from_str(POSES).expect("poses");
        for (key, samples) in samples {
            let plan = plan(&key);
            assert!(!samples.is_empty());
            for sample in samples {
                let view = plan
                    .tracks
                    .iter()
                    .find(|t| t.track == sample.track)
                    .and_then(|t| t.views.iter().find(|v| v.camera == sample.camera))
                    .expect("view");
                let pose = view
                    .chain
                    .pose(|joint| sample.positions.get(joint).copied())
                    .expect("pose");
                for (got, want) in pose.translation_m.iter().zip(sample.translation_m) {
                    assert!((got - want).abs() < 1e-9, "{key}: {got} != {want}");
                }
                for (got, want) in pose.rotation_xyzw.iter().zip(sample.rotation_xyzw) {
                    assert!((got - want).abs() < 1e-9, "{key}: {got} != {want}");
                }
            }
        }
    }

    #[test]
    fn a_fixed_mount_chain_is_its_rendered_pose() {
        let plan = plan("quest-ego-mono");
        for view in plan.tracks.iter().flat_map(|t| &t.views) {
            let fixed = view.root_from_mount.expect("fixed");
            let pose = view.chain.pose(|_| None).expect("no joint is needed");
            for (got, want) in pose.translation_m.iter().zip(fixed.translation_m) {
                assert!((got - want).abs() < 1e-12);
            }
            for (got, want) in pose.rotation_xyzw.iter().zip(fixed.rotation_xyzw) {
                assert!((got - want).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn a_missing_or_non_finite_position_places_nothing() {
        let plan = plan("sentient-humanoid-upper-body");
        let view = &plan.tracks[0].views[0];
        let first = view.chain.joints()[0].to_owned();
        assert_eq!(
            view.chain.pose(|_| None),
            Err(PlacementError::Missing(first.clone()))
        );
        assert_eq!(
            view.chain.pose(|_| Some(f64::NAN)),
            Err(PlacementError::NonFinite(first))
        );
        assert_eq!(view.chain.joints(), view.moves_with);
    }

    #[test]
    fn a_scaled_sheared_or_mirrored_origin_is_not_a_chain() {
        let step = |origin: Vec<f64>| {
            Chain(vec![ChainStep {
                joint: "j".into(),
                kind: JointType::Fixed,
                origin,
                axis: [1.0, 0.0, 0.0],
                follows: None,
            }])
        };
        assert!(step(IDENTITY.to_vec()).is_well_formed());
        let mut scaled = IDENTITY.to_vec();
        scaled[0] = 2.0;
        assert!(!step(scaled).is_well_formed());
        let mut sheared = IDENTITY.to_vec();
        sheared[1] = 0.3;
        assert!(!step(sheared).is_well_formed());
        let mut mirrored = IDENTITY.to_vec();
        mirrored[10] = -1.0;
        assert!(!step(mirrored).is_well_formed(), "determinant -1");
    }

    #[test]
    fn a_mimic_follows_its_leader() {
        let chain = Chain(vec![ChainStep {
            joint: "finger".into(),
            kind: JointType::Prismatic,
            origin: IDENTITY.to_vec(),
            axis: [1.0, 0.0, 0.0],
            follows: Some(Follows {
                joint: "gripper".into(),
                multiplier: -2.0,
                offset: 0.5,
            }),
        }]);
        let pose = chain
            .pose(|j| (j == "gripper").then_some(0.25))
            .expect("pose");
        assert!((pose.translation_m[0] - 0.0).abs() < 1e-12);
        assert_eq!(chain.joints(), ["gripper"]);
    }
}
