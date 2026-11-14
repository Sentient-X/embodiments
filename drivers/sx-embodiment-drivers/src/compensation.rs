//! A unit's quasi-static tracking compensation, evaluated by the existing dynamics kernel.
//!
//! A source description may use different joint coordinates from the canonical description
//! used for IK. Each selected joint maps `source = scale * canonical + offset`; virtual work
//! maps its gravity torque back by the same scale. The artifact binds both canonical URDF
//! bytes, exact drive bytes and physical bus identities, so changing encoder calibration or
//! moving the profile to another unit cannot silently reuse it. The station checks `units`
//! against commissioning before opening hardware; this pure parser cannot inspect hardware.
//!
//! This is a tracking adaptation, not the hand-guidance FREE/HOLD state machine: friction is
//! a smooth function of supplied tracking velocity, absent inside its deadband. Nothing here
//! opens hardware, changes motor gains or bypasses the governor and driver's effort limits.

use std::collections::BTreeSet;

use serde::Deserialize;
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::{Dynamics, DynamicsError, Scratch, Tree, TreeError, UPRIGHT};

pub const COMPENSATION_SCHEMA: &str = "sx.quasi-static-compensation/v1";

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    schema: String,
    description_sha256: String,
    drive_sha256: String,
    units: Vec<String>,
    source_urdf: String,
    gravity_scale: f64,
    torque_slew_nm_s: f64,
    joints: Vec<JointDocument>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct JointDocument {
    joint: String,
    source_joint: String,
    scale: f64,
    offset: f64,
    coulomb: f64,
    velocity_deadband: f64,
    transition_velocity: f64,
    max_torque: f64,
}

#[derive(Debug, Error)]
pub enum CompensationError {
    #[error("invalid quasi-static compensation document: {0}")]
    Document(#[from] serde_json::Error),
    #[error("unsupported quasi-static compensation schema")]
    Schema,
    #[error("compensation does not name these canonical description bytes")]
    Description,
    #[error("compensation does not name these drive bytes")]
    Drive,
    #[error("invalid compensation parameters for {0}")]
    Parameters(String),
    #[error("invalid or duplicate compensation joint mapping: {0}")]
    Mapping(String),
    #[error("compensation must map every independent source joint exactly once")]
    SourceCoverage,
    #[error("invalid compensation description: {0}")]
    Tree(#[from] TreeError),
    #[error("compensation vector has length {found}, expected {expected}")]
    Length { expected: usize, found: usize },
    #[error("compensation requires finite positions, velocities and positive finite dt")]
    NonFinite,
    #[error("compensation dynamics failed: {0}")]
    Dynamics(#[from] DynamicsError),
}

#[derive(Clone, Debug)]
struct Joint {
    chain: usize,
    source: usize,
    scale: f64,
    offset: f64,
    coulomb: f64,
    velocity_deadband: f64,
    transition_velocity: f64,
    max_torque: f64,
    previous: Option<f64>,
}

/// Parsed, identity-bound tracking compensation. A steady-state step allocates nothing.
#[derive(Clone, Debug)]
pub struct QuasiStaticCompensation {
    dynamics: Dynamics,
    scratch: Scratch,
    chain_width: usize,
    units: Vec<String>,
    gravity_scale: f64,
    torque_slew_nm_s: f64,
    joints: Vec<Joint>,
    source_q: Vec<f64>,
    source_gravity: Vec<f64>,
    next_effort: Vec<f64>,
    next_gravity: Vec<f64>,
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

impl QuasiStaticCompensation {
    /// Parse a calibration against the exact description, drive and ordered chain it serves.
    /// A partial canonical chain may be selected, but every moving source joint must be mapped.
    ///
    /// # Errors
    /// Refuses unknown fields, stale identities, invalid parameters and incomplete mappings.
    pub fn parse(
        text: &str,
        canonical_urdf: &str,
        drive_bytes: &[u8],
        chain_joints: &[&str],
    ) -> Result<Self, CompensationError> {
        let document: Document = serde_json::from_str(text)?;
        if document.schema != COMPENSATION_SCHEMA {
            return Err(CompensationError::Schema);
        }
        if document.description_sha256 != sha256(canonical_urdf.as_bytes()) {
            return Err(CompensationError::Description);
        }
        if document.drive_sha256 != sha256(drive_bytes) {
            return Err(CompensationError::Drive);
        }
        if document.units.is_empty()
            || document
                .units
                .iter()
                .any(|unit| unit.is_empty() || unit.chars().any(char::is_whitespace))
            || document.units.windows(2).any(|pair| pair[0] >= pair[1])
        {
            return Err(CompensationError::Parameters("unit identities".to_owned()));
        }
        if !(document.gravity_scale.is_finite()
            && document.gravity_scale > 0.0
            && document.gravity_scale <= 1.0
            && document.torque_slew_nm_s.is_finite()
            && document.torque_slew_nm_s > 0.0)
        {
            return Err(CompensationError::Parameters("gravity or slew".to_owned()));
        }
        let canonical = Tree::parse(canonical_urdf)?;
        let source = Tree::parse(&document.source_urdf)?;
        let mut chain_names = BTreeSet::new();
        for name in chain_joints {
            if !chain_names.insert(*name)
                || !canonical.joints().iter().any(|axis| axis.name == *name)
            {
                return Err(CompensationError::Mapping((*name).to_owned()));
            }
        }
        let mut chain_seen = BTreeSet::new();
        let mut source_seen = BTreeSet::new();
        let mut joints = Vec::with_capacity(document.joints.len());
        for entry in document.joints {
            let chain = chain_joints
                .iter()
                .position(|joint| *joint == entry.joint)
                .ok_or_else(|| CompensationError::Mapping(entry.joint.clone()))?;
            let source_index = source
                .joints()
                .iter()
                .position(|joint| joint.name == entry.source_joint)
                .ok_or_else(|| CompensationError::Mapping(entry.source_joint.clone()))?;
            if !chain_seen.insert(chain) || !source_seen.insert(source_index) {
                return Err(CompensationError::Mapping(entry.joint));
            }
            if !(entry.scale.is_finite()
                && entry.scale != 0.0
                && entry.offset.is_finite()
                && entry.coulomb.is_finite()
                && entry.coulomb >= 0.0
                && entry.velocity_deadband.is_finite()
                && entry.velocity_deadband >= 0.0
                && entry.transition_velocity.is_finite()
                && entry.transition_velocity > 0.0
                && entry.max_torque.is_finite()
                && entry.max_torque > 0.0)
            {
                return Err(CompensationError::Parameters(entry.joint));
            }
            joints.push(Joint {
                chain,
                source: source_index,
                scale: entry.scale,
                offset: entry.offset,
                coulomb: entry.coulomb,
                velocity_deadband: entry.velocity_deadband,
                transition_velocity: entry.transition_velocity,
                max_torque: entry.max_torque,
                previous: None,
            });
        }
        if joints.is_empty() || source_seen.len() != source.joints().len() {
            return Err(CompensationError::SourceCoverage);
        }
        let width = source.joints().len();
        let dynamics = Dynamics::new(source, UPRIGHT);
        let scratch = dynamics.scratch();
        Ok(Self {
            dynamics,
            scratch,
            chain_width: chain_joints.len(),
            units: document.units,
            gravity_scale: document.gravity_scale,
            torque_slew_nm_s: document.torque_slew_nm_s,
            next_effort: vec![0.0; joints.len()],
            next_gravity: vec![0.0; joints.len()],
            joints,
            source_q: vec![0.0; width],
            source_gravity: vec![0.0; width],
        })
    }

    /// Start the next sequence from bounded gravity rather than a stale sequence's effort.
    pub fn reset(&mut self) {
        for joint in &mut self.joints {
            joint.previous = None;
        }
    }

    /// Indices of the canonical chain this calibration owns, in artifact order.
    pub fn affected_indices(&self) -> impl Iterator<Item = usize> + '_ {
        self.joints.iter().map(|joint| joint.chain)
    }

    /// Physical bus identities to match against commissioning before opening hardware.
    #[must_use]
    pub fn units(&self) -> &[String] {
        &self.units
    }

    fn evaluate_gravity(&mut self, q: &[f64]) -> Result<(), CompensationError> {
        if q.len() != self.chain_width {
            return Err(CompensationError::Length {
                expected: self.chain_width,
                found: q.len(),
            });
        }
        if !q.iter().all(|value| value.is_finite()) {
            return Err(CompensationError::NonFinite);
        }
        for joint in &self.joints {
            let position = joint.scale.mul_add(q[joint.chain], joint.offset);
            if !position.is_finite() {
                return Err(CompensationError::NonFinite);
            }
            self.source_q[joint.source] = position;
        }
        self.dynamics.gravity_torque_into(
            &self.source_q,
            &mut self.scratch,
            &mut self.source_gravity,
        )?;
        for (index, joint) in self.joints.iter().enumerate() {
            let raw = joint.scale * self.source_gravity[joint.source] * self.gravity_scale;
            if !raw.is_finite() {
                return Err(CompensationError::NonFinite);
            }
            self.next_gravity[index] = raw.clamp(-joint.max_torque, joint.max_torque);
        }
        Ok(())
    }

    /// Scaled, bounded physical gravity on selected axes, with no friction or slew history.
    /// This is suitable for the existing stop/fall-rule path. Other axes remain untouched.
    ///
    /// # Errors
    /// A bad length, non-finite position or dynamics failure leaves `out` and slew history
    /// unchanged. Only reusable computation scratch changes.
    pub fn gravity_into(&mut self, q: &[f64], out: &mut [f64]) -> Result<(), CompensationError> {
        if out.len() != self.chain_width {
            return Err(CompensationError::Length {
                expected: self.chain_width,
                found: out.len(),
            });
        }
        self.evaluate_gravity(q)?;
        for (index, joint) in self.joints.iter().enumerate() {
            out[joint.chain] = self.next_gravity[index];
        }
        Ok(())
    }

    /// Overwrite only selected axes. Other axes retain their caller-supplied compensation.
    ///
    /// First use starts at bounded physical gravity and slews toward gravity plus friction.
    /// `gravity` reports only the physical-gravity portion of applied effort: at most its
    /// magnitude and zero if the applied effort opposes gravity. Friction is never called
    /// gravity. The driver may tighten both outputs further through its normal protections.
    ///
    /// # Errors
    /// Length, non-finite input, arithmetic overflow or a dynamics failure leaves outputs and
    /// the committed slew state untouched. Internal scratch buffers are not control state.
    pub fn step(
        &mut self,
        q: &[f64],
        velocity: &[f64],
        dt: f64,
        effort: &mut [f64],
        gravity: &mut [f64],
    ) -> Result<(), CompensationError> {
        for found in [q.len(), velocity.len(), effort.len(), gravity.len()] {
            if found != self.chain_width {
                return Err(CompensationError::Length {
                    expected: self.chain_width,
                    found,
                });
            }
        }
        let delta = self.torque_slew_nm_s * dt;
        if !(dt.is_finite()
            && dt > 0.0
            && delta.is_finite()
            && q.iter().chain(velocity).all(|x| x.is_finite()))
        {
            return Err(CompensationError::NonFinite);
        }
        self.evaluate_gravity(q)?;
        for (index, joint) in self.joints.iter().enumerate() {
            let raw = joint.scale * self.source_gravity[joint.source] * self.gravity_scale;
            if !raw.is_finite() {
                return Err(CompensationError::NonFinite);
            }
            let held = raw.clamp(-joint.max_torque, joint.max_torque);
            let speed = velocity[joint.chain];
            let sliding = (speed.abs() - joint.velocity_deadband).max(0.0);
            let friction =
                joint.coulomb * (sliding / joint.transition_velocity).tanh() * speed.signum();
            let combined = raw + friction;
            if !combined.is_finite() {
                return Err(CompensationError::NonFinite);
            }
            let wanted = combined.clamp(-joint.max_torque, joint.max_torque);
            let previous = joint.previous.unwrap_or(held);
            // Difference and addition can overflow at extreme finite parameters; refuse the
            // whole step rather than partially committing another joint's output.
            let change = wanted - previous;
            let applied = previous + change.clamp(-delta, delta);
            if !(change.is_finite() && applied.is_finite()) {
                return Err(CompensationError::NonFinite);
            }
            self.next_effort[index] = applied;
            self.next_gravity[index] =
                if held != 0.0 && applied != 0.0 && held.signum() == applied.signum() {
                    held.abs().min(applied.abs()).copysign(held)
                } else {
                    0.0
                };
        }
        for (index, joint) in self.joints.iter_mut().enumerate() {
            effort[joint.chain] = self.next_effort[index];
            gravity[joint.chain] = self.next_gravity[index];
            joint.previous = Some(self.next_effort[index]);
        }
        Ok(())
    }
}
