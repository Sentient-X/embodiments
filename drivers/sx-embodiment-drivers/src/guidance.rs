//! A local hand-guidance profile, separate from the platform's identified dynamics.
//!
//! Its source gravity model uses affine source coordinates. Virtual work gives the matching
//! target-joint torque: `q_source = gain*q_target + offset`, `tau_target = gain*tau_source`.
//! Per-joint gravity scales and optional target-unit torque caps apply after that mapping.
//! Damping and slew limits are settings for the guidance controller, not terms added here.

use std::collections::{BTreeMap, BTreeSet};

use serde::Deserialize;
use thiserror::Error;

use crate::dynamics::{Dynamics, DynamicsError, Scratch};
use crate::kinematics::{Tree, TreeError};

pub const GUIDANCE_SCHEMA: &str = "sx.guidance/v1";

#[derive(Debug, Error)]
pub enum GuidanceError {
    #[error("invalid guidance profile: {0}")]
    Document(String),
    #[error("guidance {field} is {found}, expected {expected}")]
    Identity {
        field: &'static str,
        expected: String,
        found: String,
    },
    #[error("guidance {field} needs {expected} values, got {found}")]
    Width {
        field: &'static str,
        expected: usize,
        found: usize,
    },
    #[error("guidance joint {0} produced a non-finite coordinate or torque")]
    NonFinite(String),
    #[error(transparent)]
    Model(#[from] TreeError),
    #[error(transparent)]
    Dynamics(#[from] DynamicsError),
}

/// Settings are expressed in target joint coordinates (rad, N·m, seconds).
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointSettings {
    pub joint: String,
    pub coordinate_gain: f64,
    pub coordinate_offset: f64,
    pub gravity_scale: f64,
    pub damping: f64,
    pub friction_coulomb: f64,
    /// Reserved: v1 requires zero because friction assistance is not implemented.
    pub friction_scale: f64,
    /// Optional symmetric bound in target-joint N·m, applied after coordinate conversion.
    pub torque_limit: Option<f64>,
    /// Positive target-joint N·m/s. The guidance controller applies the slew bound.
    pub torque_slew_rate: f64,
}

impl JointSettings {
    fn valid(&self) -> bool {
        !self.joint.trim().is_empty()
            && self.coordinate_gain.is_finite()
            && self.coordinate_gain != 0.0
            && self.coordinate_offset.is_finite()
            && self.gravity_scale.is_finite()
            && self.gravity_scale >= 0.0
            && self.damping.is_finite()
            && self.damping > 0.0
            && self.friction_coulomb.is_finite()
            && self.friction_coulomb >= 0.0
            && self.friction_scale.is_finite()
            && self.friction_scale == 0.0
            && self
                .torque_limit
                .is_none_or(|limit| limit.is_finite() && limit > 0.0)
            && self.torque_slew_rate.is_finite()
            && self.torque_slew_rate > 0.0
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    schema: String,
    description_sha256: String,
    drive_sha256: String,
    model_urdf: String,
    gravity: [f64; 3],
    joints: Vec<JointSettings>,
    #[serde(default)]
    source_sha256: BTreeMap<String, String>,
}

/// An immutable guidance-only model tied to exact target description and drive bytes.
/// Matching this model's moving joint names to the target body is the consumer's boundary;
/// the two target digests alone cannot establish that correspondence.
#[derive(Clone, Debug)]
pub struct GuidanceProfile {
    model: Dynamics,
    settings: Vec<JointSettings>,
    description_sha256: String,
    drive_sha256: String,
    source_sha256: BTreeMap<String, String>,
}

/// Caller-owned buffers allocated once with [`GuidanceProfile::scratch`].
pub struct GuidanceScratch {
    model: Scratch,
    position: Vec<f64>,
    torque: Vec<f64>,
}

fn digest_valid(value: &str) -> bool {
    value.len() == 64 && value.bytes().all(|byte| byte.is_ascii_hexdigit())
}

impl GuidanceProfile {
    /// Parse a strict guidance profile bound to the target's exact content identities.
    /// Uses the explicit source gravity vector. Source joint-origin rotations belong in
    /// `model_urdf`, rather than being reapplied as coordinate offsets.
    ///
    /// # Errors
    /// Refuses unknown fields, duplicate/missing/extra joint settings, mismatched identities,
    /// malformed models/scalars, and nonzero unimplemented friction assistance.
    pub fn parse(
        document: &str,
        expected_description_sha: &str,
        expected_drive_sha: &str,
    ) -> Result<Self, GuidanceError> {
        let document: Document = serde_json::from_str(document)
            .map_err(|error| GuidanceError::Document(error.to_string()))?;
        if document.schema != GUIDANCE_SCHEMA {
            return Err(GuidanceError::Document(
                "schema must be sx.guidance/v1".to_owned(),
            ));
        }
        for (field, found, expected) in [
            (
                "description_sha256",
                document.description_sha256.as_str(),
                expected_description_sha,
            ),
            (
                "drive_sha256",
                document.drive_sha256.as_str(),
                expected_drive_sha,
            ),
        ] {
            if !digest_valid(found) || !digest_valid(expected) {
                return Err(GuidanceError::Document(format!(
                    "{field} must be a 64-digit hexadecimal digest"
                )));
            }
            if found != expected {
                return Err(GuidanceError::Identity {
                    field,
                    expected: expected.to_owned(),
                    found: found.to_owned(),
                });
            }
        }
        if document
            .source_sha256
            .iter()
            .any(|(name, digest)| name.trim().is_empty() || !digest_valid(digest))
        {
            return Err(GuidanceError::Document(
                "source_sha256 names must be nonempty and values valid digests".to_owned(),
            ));
        }
        if !document.gravity.iter().all(|value| value.is_finite()) {
            return Err(GuidanceError::Document(
                "gravity must contain three finite components".to_owned(),
            ));
        }
        let model = Dynamics::of(&document.model_urdf, document.gravity)?;
        if model.tree().joints().is_empty()
            || model
                .tree()
                .inertials
                .iter()
                .any(|inertia| !inertia.is_physical())
        {
            return Err(GuidanceError::Document(
                "source model needs moving coordinates and physical link inertials".to_owned(),
            ));
        }
        let mut by_joint = BTreeMap::new();
        for settings in document.joints {
            if !settings.valid() {
                return Err(GuidanceError::Document(format!(
                    "invalid settings or unimplemented friction for {}",
                    settings.joint
                )));
            }
            let name = settings.joint.clone();
            if by_joint.insert(name.clone(), settings).is_some() {
                return Err(GuidanceError::Document(format!(
                    "duplicate guidance joint {name}"
                )));
            }
        }
        let mut settings = Vec::with_capacity(model.tree().joints().len());
        for axis in model.tree().joints() {
            settings.push(by_joint.remove(&axis.name).ok_or_else(|| {
                GuidanceError::Document(format!("missing guidance joint {}", axis.name))
            })?);
        }
        if !by_joint.is_empty() {
            return Err(GuidanceError::Document(format!(
                "guidance joints absent from source model: {:?}",
                by_joint.keys().collect::<BTreeSet<_>>()
            )));
        }
        Ok(Self {
            model,
            settings,
            description_sha256: document.description_sha256,
            drive_sha256: document.drive_sha256,
            source_sha256: document.source_sha256,
        })
    }

    #[must_use]
    pub fn tree(&self) -> &Tree {
        self.model.tree()
    }
    /// Settings in [`Tree::joints`] independent moving-coordinate order.
    #[must_use]
    pub fn settings(&self) -> &[JointSettings] {
        &self.settings
    }
    #[must_use]
    pub fn joint_settings(&self, joint: &str) -> Option<&JointSettings> {
        self.settings
            .iter()
            .find(|settings| settings.joint == joint)
    }
    #[must_use]
    pub fn description_sha256(&self) -> &str {
        &self.description_sha256
    }
    #[must_use]
    pub fn drive_sha256(&self) -> &str {
        &self.drive_sha256
    }
    #[must_use]
    pub fn source_sha256(&self) -> &BTreeMap<String, String> {
        &self.source_sha256
    }
    /// Forward the dynamics owner's conservative dependency proof; unknown names fail closed.
    #[must_use]
    pub fn gravity_depends_on(&self, target: &str, coordinate: &str) -> bool {
        self.model.gravity_depends_on(target, coordinate)
    }
    #[must_use]
    pub fn scratch(&self) -> GuidanceScratch {
        GuidanceScratch {
            model: self.model.scratch(),
            position: vec![0.0; self.settings.len()],
            torque: vec![0.0; self.settings.len()],
        }
    }

    /// Compute gravity in target joint units, without friction, damping, slew or allocation.
    /// Both vectors follow [`Tree::joints`] order. Unrelated/unobserved coordinates may only
    /// be supplied by a consumer after proving them irrelevant with `gravity_depends_on`.
    /// Output is replaced only after the complete calculation succeeds.
    ///
    /// # Errors
    /// Wrong vector/scratch dimensions, nonfinite positions/transforms/results, or model errors.
    pub fn gravity_into(
        &self,
        platform_q: &[f64],
        scratch: &mut GuidanceScratch,
        platform_tau: &mut [f64],
    ) -> Result<(), GuidanceError> {
        let expected = self.settings.len();
        for (field, found) in [
            ("position", platform_q.len()),
            ("torque", platform_tau.len()),
            ("scratch-position", scratch.position.len()),
            ("scratch-torque", scratch.torque.len()),
        ] {
            if found != expected {
                return Err(GuidanceError::Width {
                    field,
                    expected,
                    found,
                });
            }
        }
        for ((target, source), setting) in platform_q
            .iter()
            .zip(&mut scratch.position)
            .zip(&self.settings)
        {
            *source = setting
                .coordinate_gain
                .mul_add(*target, setting.coordinate_offset);
            if !target.is_finite() || !source.is_finite() {
                return Err(GuidanceError::NonFinite(setting.joint.clone()));
            }
        }
        self.model.gravity_torque_into(
            &scratch.position,
            &mut scratch.model,
            &mut scratch.torque,
        )?;
        for (torque, setting) in scratch.torque.iter_mut().zip(&self.settings) {
            *torque *= setting.coordinate_gain * setting.gravity_scale;
            if !torque.is_finite() {
                return Err(GuidanceError::NonFinite(setting.joint.clone()));
            }
            if let Some(limit) = setting.torque_limit {
                *torque = torque.clamp(-limit, limit);
            }
        }
        platform_tau.copy_from_slice(&scratch.torque);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};

    const DESCRIPTION: &str = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const DRIVE: &str = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const SINGLE: &str = "<robot name='guidance'><link name='base'/><link name='pendulum'><inertial><origin xyz='0.2 0 0'/><mass value='2'/><inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01'/></inertial></link><joint name='elbow' type='revolute'><parent link='base'/><child link='pendulum'/><origin rpy='0 1.5707963267948966 0'/><axis xyz='0 1 0'/><limit lower='-6' upper='6' effort='20' velocity='2'/></joint></robot>";
    fn setting(name: &str) -> Value {
        json!({"joint":name,"coordinate_gain":1.0,"coordinate_offset":0.0,"gravity_scale":1.0,"damping":0.5,"friction_coulomb":0.2,"friction_scale":0.0,"torque_slew_rate":20.0})
    }
    fn document(urdf: &str, joints: &[Value]) -> Value {
        json!({"schema":GUIDANCE_SCHEMA,"description_sha256":DESCRIPTION,"drive_sha256":DRIVE,"model_urdf":urdf,"gravity":[0.0,0.0,-9.81],"joints":joints})
    }
    fn parse(document: &Value) -> Result<GuidanceProfile, GuidanceError> {
        GuidanceProfile::parse(&document.to_string(), DESCRIPTION, DRIVE)
    }

    #[test]
    fn affine_gravity_preserves_signed_virtual_work_and_caps_in_target_units() {
        let mut joint = setting("elbow");
        joint["coordinate_gain"] = json!(-2.0);
        joint["coordinate_offset"] = json!(0.3);
        joint["gravity_scale"] = json!(0.7);
        let mut doc = document(SINGLE, &[joint]);
        let profile = parse(&doc).unwrap();
        let mut scratch = profile.scratch();
        let mut out = [0.0];
        let q = 0.1;
        profile.gravity_into(&[q], &mut scratch, &mut out).unwrap();
        // The source origin already contains pi/2. A second elbow coordinate offset would
        // give the wrong torque. Gravity is -m*g*l*cos(pi/2 + gain*q + offset).
        let expected =
            -2.0 * 0.7 * (-2.0 * 9.81 * 0.2 * (std::f64::consts::FRAC_PI_2 - 2.0 * q + 0.3).cos());
        assert!((out[0] - expected).abs() < 1e-10);
        doc["joints"][0]["torque_limit"] = json!(0.1);
        let capped = parse(&doc).unwrap();
        capped
            .gravity_into(&[q], &mut capped.scratch(), &mut out)
            .unwrap();
        assert!((out[0] - expected.signum() * 0.1).abs() < 1e-12);
        let unchanged = out;
        assert!(
            capped
                .gravity_into(&[f64::NAN], &mut scratch, &mut out)
                .is_err()
        );
        assert!((out[0] - unchanged[0]).abs() < 1e-12);
    }

    #[test]
    fn two_link_model_matches_independent_planar_gravity_and_has_allocation_free_ticks() {
        let urdf = "<robot name='two'><link name='base'/><link name='one'><inertial><origin xyz='0.2 0 0'/><mass value='2'/><inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01'/></inertial></link><link name='two'><inertial><origin xyz='0.1 0 0'/><mass value='1'/><inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' iyz='0' izz='0.01'/></inertial></link><joint name='shoulder' type='revolute'><parent link='base'/><child link='one'/><axis xyz='0 1 0'/><limit lower='-6' upper='6' effort='20' velocity='2'/></joint><joint name='elbow' type='revolute'><parent link='one'/><child link='two'/><origin xyz='0.4 0 0'/><axis xyz='0 1 0'/><limit lower='-6' upper='6' effort='20' velocity='2'/></joint></robot>";
        // Deliberately unordered JSON entries must bind by joint name, not input row order.
        let profile = parse(&document(urdf, &[setting("elbow"), setting("shoulder")])).unwrap();
        assert_eq!(profile.settings()[0].joint, "shoulder");
        let q = [0.4, -0.2];
        let mut scratch = profile.scratch();
        let mut out = [0.0; 2];
        profile.gravity_into(&q, &mut scratch, &mut out).unwrap();
        let elbow = -9.81 * 0.1 * (q[0] + q[1]).cos();
        let shoulder = -9.81 * (2.0 * 0.2 + 1.0 * 0.4) * q[0].cos() + elbow;
        assert!((out[0] - shoulder).abs() < 1e-10 && (out[1] - elbow).abs() < 1e-10);
        let allocations = allocation_counter::measure(|| {
            for _ in 0..100 {
                profile.gravity_into(&q, &mut scratch, &mut out).unwrap();
            }
        });
        assert_eq!(allocations.count_total, 0);
    }

    #[test]
    fn profiles_refuse_identity_coverage_scalars_models_and_unimplemented_friction() {
        let valid = document(SINGLE, &[setting("elbow")]);
        for (field, value) in [
            ("coordinate_gain", 0.0),
            ("damping", 0.0),
            ("gravity_scale", -1.0),
            ("friction_coulomb", -0.1),
            ("friction_scale", 0.1),
            ("torque_slew_rate", 0.0),
            ("torque_limit", -1.0),
        ] {
            let mut bad = valid.clone();
            bad["joints"][0][field] = json!(value);
            assert!(parse(&bad).is_err(), "{field}");
        }
        for bad in [
            {
                let mut bad = valid.clone();
                bad["description_sha256"] = json!(DRIVE);
                bad
            },
            {
                let mut bad = valid.clone();
                bad["drive_sha256"] = json!("not-a-digest");
                bad
            },
            {
                let mut bad = valid.clone();
                bad["joints"] = json!([]);
                bad
            },
            {
                let mut bad = valid.clone();
                bad["joints"].as_array_mut().unwrap().push(setting("elbow"));
                bad
            },
            {
                let mut bad = valid.clone();
                bad["joints"]
                    .as_array_mut()
                    .unwrap()
                    .push(setting("unknown"));
                bad
            },
            {
                let mut bad = valid.clone();
                bad["joints"][0]["unknown"] = json!(1);
                bad
            },
            {
                let mut bad = valid.clone();
                bad["unknown"] = json!(1);
                bad
            },
            {
                let mut bad = valid.clone();
                bad["model_urdf"] = json!("<robot/>");
                bad
            },
            {
                let mut bad = valid.clone();
                bad["source_sha256"] = json!({"source":"bad"});
                bad
            },
        ] {
            assert!(parse(&bad).is_err());
        }
        let profile = parse(&valid).unwrap();
        assert!(
            profile
                .gravity_into(&[], &mut profile.scratch(), &mut [0.0])
                .is_err()
        );
        assert!(profile.gravity_depends_on("missing", "elbow"));
    }
}
