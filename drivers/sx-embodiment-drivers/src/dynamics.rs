//! The torque a body's joints need to follow a motion: inverse dynamics over the [`Tree`].
//!
//! [`Dynamics::inverse`] answers `tau = M(q) q̈ + C(q, q̇) q̇ + g(q) + friction(q̇)` for every
//! independent joint of a description, by the recursive Newton-Euler algorithm over the same
//! joint tree [`Tree::link_poses`] places (fixed joints carry their child's load, a mimic joint
//! moves at `multiplier` times its leader and loads it by the same factor). Each link's mass
//! properties are [`LinkInertia`], the ten numbers torque is linear in: mass, first moment
//! `m c` and the inertia tensor about the link's own origin, on its own axes. Each independent
//! joint adds [`JointFriction`]: rotor inertia reflected through the gearbox (armature), viscous
//! friction, and Coulomb friction with a smooth sign `tanh(q̇ / stiction_velocity)`, so the
//! feed-forward never chatters about zero speed.
//!
//! The description's inertials are a design estimate (hull-estimated masses on the static upper
//! body). A [`DynamicsCalibration`], the `dynamics` calibration kind
//! `sx_embodiments.calibration.DynamicsCalibration` seals after identification on one built
//! robot, replaces them link by link and supplies each joint's friction; it is refused for
//! any description but the one it was measured against. Gravity is a vector in the root link's
//! frame and comes from how the body is mounted: [`UPRIGHT`] for a body standing with its root
//! z up, as the table-clamped static upper body does.
//!
//! `sx_embodiments.calibrate.dynamics` holds the same model as a linear regressor; the crate's
//! `dynamics-parity.json` fixture holds this, that and `MuJoCo`'s `mj_inverse` equal.

use serde::Deserialize;
use thiserror::Error;

use crate::kinematics::{JointType, PlacementError, Transform, Tree, about, along, compose};

/// Standard gravity, m/s².
pub const STANDARD_GRAVITY: f64 = 9.806_65;

/// Gravity in the root frame of a body standing upright, its root link's z axis up.
pub const UPRIGHT: [f64; 3] = [0.0, 0.0, -STANDARD_GRAVITY];

/// The `kind` of a dynamics calibration document (`sx.calibration/v1`).
pub const DYNAMICS_KIND: &str = "dynamics";

/// One link's mass properties, in its own frame: what joint torque is linear in.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LinkInertia {
    /// kg.
    pub mass: f64,
    /// Mass times the centre of mass, kg·m.
    pub first_moment: [f64; 3],
    /// The inertia tensor about the link's origin, kg·m²: `xx, xy, xz, yy, yz, zz`.
    pub inertia: [f64; 6],
}

impl LinkInertia {
    pub const ZERO: Self = Self {
        mass: 0.0,
        first_moment: [0.0; 3],
        inertia: [0.0; 6],
    };

    /// A URDF `<inertial>`: `mass` at the origin of `frame` (the link-from-inertial transform),
    /// `about_centre` the tensor there on `frame`'s axes.
    #[must_use]
    pub fn from_centre(mass: f64, frame: &Transform, about_centre: [f64; 6]) -> Self {
        let rotation = rotation(frame);
        let centre = [frame[3], frame[7], frame[11]];
        let local = symmetric(about_centre);
        // R I Rᵀ, then the parallel-axis shift to the link origin.
        let mut turned = [[0.0; 3]; 3];
        for (i, row) in turned.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = (0..3)
                    .map(|k| {
                        (0..3)
                            .map(|l| rotation[i][k] * local[k][l] * rotation[j][l])
                            .sum::<f64>()
                    })
                    .sum();
            }
        }
        let squared = dot(centre, centre);
        let shifted = |i: usize, j: usize| {
            let identity = if i == j { squared } else { 0.0 };
            mass.mul_add(identity - centre[i] * centre[j], turned[i][j])
        };
        Self {
            mass,
            first_moment: scale(centre, mass),
            inertia: [
                shifted(0, 0),
                shifted(0, 1),
                shifted(0, 2),
                shifted(1, 1),
                shifted(1, 2),
                shifted(2, 2),
            ],
        }
    }

    /// Whether some distribution of non-negative mass has these properties: a non-negative
    /// mass (zero only with no first moment or inertia), and about the centre of mass a tensor
    /// whose principal moments are non-negative and obey the triangle inequality, which is the
    /// second moment `tr(I)/2 · 1 - I` being positive semidefinite.
    #[must_use]
    pub fn is_physical(&self) -> bool {
        let finite = std::iter::once(self.mass)
            .chain(self.first_moment)
            .chain(self.inertia)
            .all(f64::is_finite);
        if !finite || self.mass < 0.0 {
            return false;
        }
        if self.mass == 0.0 {
            return self
                .first_moment
                .iter()
                .chain(&self.inertia)
                .all(|v| v.abs() <= 1e-12);
        }
        let centre = scale(self.first_moment, 1.0 / self.mass);
        let squared = dot(centre, centre);
        let about = symmetric(self.inertia);
        let mut at_centre = [[0.0; 3]; 3];
        for (i, row) in at_centre.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                let identity = if i == j { squared } else { 0.0 };
                *value = self
                    .mass
                    .mul_add(-(identity - centre[i] * centre[j]), about[i][j]);
            }
        }
        let half_trace = (at_centre[0][0] + at_centre[1][1] + at_centre[2][2]) / 2.0;
        let mut second = at_centre;
        for (i, row) in second.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = if i == j { half_trace } else { 0.0 } - *value;
            }
        }
        let tolerance = 1e-9 * half_trace.abs().max(1e-12);
        is_semidefinite(&second, tolerance)
    }
}

/// What one independent joint's drive (motor, gearbox, bearings) adds to the rigid-body torque.
///
/// The friction family is BAM's ("Better Actuator Models", Duclusaud and Passault, Rhoban,
/// <https://github.com/Rhoban/bam>, Apache-2.0; `bam/model.py` at e9a619d, models M1 to M5):
/// a friction *budget* that resistive friction may reach,
///
/// ```text
/// budget = coulomb + |e τ_ext − m τ_motor|
///        + s(q̇) (stribeck + |e_s τ_ext − m_s τ_motor|),   s(q̇) = exp(−|q̇ / v_s|^α)
/// ```
///
/// with `τ_motor` the torque the drive delivers and `τ_ext` the load's torque on the joint
/// (gravity's, `−g(q)`). A non-directional load term is `e = m`. In motion the friction is
/// `sign(q̇) · budget + viscous · q̇`; at rest it is any torque inside the budget, which a
/// simulator applies by clipping the stopping torque to it (BAM's `frictionloss`). Feed-forward
/// uses the moving form with the smooth sign `tanh(q̇ / stiction_velocity)`, so it never
/// chatters about zero speed and adds nothing at rest, where the direction is unknown.
/// Every term is zero by default; M1 is `coulomb` and `viscous` alone.
#[derive(Clone, Copy, Debug, Default, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointFriction {
    /// Coulomb friction, N·m (N for a prismatic joint).
    pub coulomb: f64,
    /// Viscous friction, N·m·s/rad.
    pub viscous: f64,
    /// Rotor inertia reflected to the joint, kg·m².
    pub armature: f64,
    /// Extra friction near standstill, N·m, fading as `s(q̇)`.
    pub stribeck: f64,
    /// `v_s`, rad/s; zero switches every Stribeck term off.
    pub stribeck_velocity: f64,
    /// `α`, the Stribeck curve's exponent, at least 1 where Stribeck terms are used.
    pub stribeck_exponent: f64,
    /// `m` and `e`: friction per N·m of drive and load torque through the gearbox.
    pub load_motor: f64,
    pub load_external: f64,
    /// `m_s` and `e_s`: the same near standstill.
    pub load_motor_stribeck: f64,
    pub load_external_stribeck: f64,
}

impl JointFriction {
    fn is_physical(&self) -> bool {
        let values = [
            self.coulomb,
            self.viscous,
            self.armature,
            self.stribeck,
            self.stribeck_velocity,
            self.stribeck_exponent,
            self.load_motor,
            self.load_external,
            self.load_motor_stribeck,
            self.load_external_stribeck,
        ];
        let stribeck_used = self.stribeck > 0.0
            || self.load_motor_stribeck > 0.0
            || self.load_external_stribeck > 0.0;
        values.iter().all(|v| v.is_finite() && *v >= 0.0)
            && (!stribeck_used || (self.stribeck_velocity > 0.0 && self.stribeck_exponent >= 1.0))
    }

    /// Whether any term depends on the drive or load torque.
    #[must_use]
    pub fn is_loaded(&self) -> bool {
        self.load_motor > 0.0
            || self.load_external > 0.0
            || self.load_motor_stribeck > 0.0
            || self.load_external_stribeck > 0.0
    }

    /// The friction budget at `speed` with the drive delivering `motor` against a load
    /// `external`, N·m: the most friction can resist.
    #[must_use]
    pub fn budget(&self, speed: f64, motor: f64, external: f64) -> f64 {
        let moving = self.coulomb + (self.load_external * external - self.load_motor * motor).abs();
        if self.stribeck_velocity <= 0.0 {
            return moving;
        }
        let fade = (-(speed / self.stribeck_velocity)
            .abs()
            .powf(self.stribeck_exponent))
        .exp();
        moving
            + fade
                * (self.stribeck
                    + (self.load_external_stribeck * external - self.load_motor_stribeck * motor)
                        .abs())
    }

    /// What the drive adds to move at `speed` and `acceleration`: armature, viscous, and the
    /// budget with the smooth sign `tanh(speed / stiction_velocity)`.
    #[must_use]
    pub fn torque(
        &self,
        speed: f64,
        acceleration: f64,
        motor: f64,
        external: f64,
        stiction_velocity: f64,
    ) -> f64 {
        self.armature * acceleration
            + self.viscous * speed
            + (speed / stiction_velocity).tanh() * self.budget(speed, motor, external)
    }
}

/// A dynamics calibration cannot be read or applied.
#[derive(Debug, Error, PartialEq)]
pub enum DynamicsError {
    #[error("not a dynamics calibration: {0}")]
    Document(String),
    #[error("the calibration was measured against description {found}, not {expected}")]
    Description { expected: String, found: String },
    #[error("the calibration names {0:?}, which the description lacks")]
    Absent(String),
    #[error("{0:?} is listed twice")]
    Repeated(String),
    #[error("{0:?} has mass properties no body has, or friction that is negative")]
    Unphysical(String),
    #[error("expected {expected} values, one per independent joint, got {found}")]
    Length { expected: usize, found: usize },
    #[error("joint {0:?} has a non-finite velocity or acceleration")]
    NonFinite(String),
    #[error(transparent)]
    Placement(#[from] PlacementError),
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct LinkEntry {
    link: String,
    mass: f64,
    first_moment: [f64; 3],
    inertia: [f64; 6],
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct JointEntry {
    joint: String,
    coulomb: f64,
    viscous: f64,
    armature: f64,
    stribeck: f64,
    stribeck_velocity: f64,
    stribeck_exponent: f64,
    load_motor: f64,
    load_external: f64,
    load_motor_stribeck: f64,
    load_external_stribeck: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    schema: String,
    kind: String,
    description_sha256: String,
    stiction_velocity: f64,
    links: Vec<LinkEntry>,
    joints: Vec<JointEntry>,
    units: Vec<String>,
    measurement: serde::de::IgnoredAny,
}

/// One built robot's identified dynamics, as `sx_embodiments.calibration.DynamicsCalibration`
/// documents it.
#[derive(Clone, Debug, PartialEq)]
pub struct DynamicsCalibration {
    /// The description the parameters were identified against.
    pub description_sha256: String,
    /// The speed, rad/s, at which Coulomb friction reaches `tanh(1)` of its magnitude.
    pub stiction_velocity: f64,
    pub links: Vec<(String, LinkInertia)>,
    pub joints: Vec<(String, JointFriction)>,
}

impl DynamicsCalibration {
    /// Read an `sx.calibration/v1` document of kind `dynamics`.
    ///
    /// # Errors
    ///
    /// [`DynamicsError::Document`] for any other document, and [`DynamicsError::Unphysical`]
    /// for parameters no body has.
    pub fn parse(document: &str) -> Result<Self, DynamicsError> {
        let parsed: Document = serde_json::from_str(document)
            .map_err(|error| DynamicsError::Document(error.to_string()))?;
        if parsed.schema != "sx.calibration/v1" || parsed.kind != DYNAMICS_KIND {
            return Err(DynamicsError::Document(format!(
                "schema {:?} kind {:?}",
                parsed.schema, parsed.kind
            )));
        }
        let _ = (&parsed.units, parsed.measurement);
        if !(parsed.stiction_velocity.is_finite() && parsed.stiction_velocity > 0.0) {
            return Err(DynamicsError::Document(
                "stiction_velocity is a positive speed".into(),
            ));
        }
        let links: Vec<(String, LinkInertia)> = parsed
            .links
            .into_iter()
            .map(|entry| {
                (
                    entry.link,
                    LinkInertia {
                        mass: entry.mass,
                        first_moment: entry.first_moment,
                        inertia: entry.inertia,
                    },
                )
            })
            .collect();
        let joints: Vec<(String, JointFriction)> = parsed
            .joints
            .into_iter()
            .map(|entry| {
                (
                    entry.joint,
                    JointFriction {
                        coulomb: entry.coulomb,
                        viscous: entry.viscous,
                        armature: entry.armature,
                        stribeck: entry.stribeck,
                        stribeck_velocity: entry.stribeck_velocity,
                        stribeck_exponent: entry.stribeck_exponent,
                        load_motor: entry.load_motor,
                        load_external: entry.load_external,
                        load_motor_stribeck: entry.load_motor_stribeck,
                        load_external_stribeck: entry.load_external_stribeck,
                    },
                )
            })
            .collect();
        if let Some((name, _)) = links.iter().find(|(_, inertia)| !inertia.is_physical()) {
            return Err(DynamicsError::Unphysical(name.clone()));
        }
        if let Some((name, _)) = joints.iter().find(|(_, friction)| !friction.is_physical()) {
            return Err(DynamicsError::Unphysical(name.clone()));
        }
        Ok(Self {
            description_sha256: parsed.description_sha256,
            stiction_velocity: parsed.stiction_velocity,
            links,
            joints,
        })
    }
}

/// A description's rigid-body dynamics, with its drives' friction, under one gravity.
#[derive(Clone, Debug, PartialEq)]
pub struct Dynamics {
    tree: Tree,
    gravity: [f64; 3],
    /// In [`Tree::links`] order.
    inertia: Vec<LinkInertia>,
    /// In [`Tree::joints`] order.
    friction: Vec<JointFriction>,
    stiction_velocity: f64,
    /// Whether any joint's friction depends on load, so a tick also needs gravity alone.
    loaded: bool,
    /// The digest of the description bytes, when read from them.
    description_sha256: Option<String>,
    /// For each tree joint, the index of the independent joint it moves with.
    leader: Vec<Option<usize>>,
}

/// Where the default stiction speed sits when no calibration chose one; it only matters once
/// a calibration supplies Coulomb friction, which then also supplies its own.
const DEFAULT_STICTION_VELOCITY: f64 = 0.02;

impl Dynamics {
    /// A description's dynamics, remembering its digest so only a calibration identified
    /// against exactly these bytes applies ([`Dynamics::calibrated`]).
    ///
    /// # Errors
    ///
    /// [`TreeError`] when the description is not a tree.
    pub fn of(urdf: &str, gravity: [f64; 3]) -> Result<Self, crate::kinematics::TreeError> {
        let mut dynamics = Self::new(Tree::parse(urdf)?, gravity);
        dynamics.description_sha256 = Some(sha256_hex(urdf));
        Ok(dynamics)
    }

    /// The description's own inertials, no friction, under `gravity` (root frame, m/s²). No
    /// calibration applies to dynamics built this way; read the description with
    /// [`Dynamics::of`] to calibrate.
    #[must_use]
    pub fn new(tree: Tree, gravity: [f64; 3]) -> Self {
        let leader = tree
            .joints
            .iter()
            .map(|joint| {
                (joint.kind != JointType::Fixed)
                    .then(|| {
                        tree.joints()
                            .iter()
                            .position(|axis| axis.name == joint.leader)
                    })
                    .flatten()
            })
            .collect();
        Self {
            inertia: tree.inertials.clone(),
            friction: vec![JointFriction::default(); tree.joints().len()],
            stiction_velocity: DEFAULT_STICTION_VELOCITY,
            loaded: false,
            description_sha256: None,
            gravity,
            leader,
            tree,
        }
    }

    /// These dynamics with one built robot's identified parameters in place of the
    /// description's: each listed link's mass properties, each listed joint's friction.
    ///
    /// # Errors
    ///
    /// [`DynamicsError::Description`] when the calibration was identified against another
    /// description than the one these dynamics were read from ([`Dynamics::of`] hashes it;
    /// dynamics built from a bare [`Tree`] accept none), and [`DynamicsError::Absent`] or
    /// [`DynamicsError::Repeated`] for a link or joint the tree lacks or the calibration lists
    /// twice. Which robot's units were fitted is the robot instance's to check
    /// (`sx_embodiments.instance.resolve`), which selects the calibration this is given.
    pub fn calibrated(mut self, calibration: &DynamicsCalibration) -> Result<Self, DynamicsError> {
        let Some(digest) = self.description_sha256.as_deref() else {
            return Err(DynamicsError::Description {
                expected: "a description read by Dynamics::of".into(),
                found: calibration.description_sha256.clone(),
            });
        };
        if calibration.description_sha256 != digest {
            return Err(DynamicsError::Description {
                expected: digest.to_owned(),
                found: calibration.description_sha256.clone(),
            });
        }
        let mut seen = std::collections::HashSet::new();
        for (link, inertia) in &calibration.links {
            if !seen.insert(link.as_str()) {
                return Err(DynamicsError::Repeated(link.clone()));
            }
            let index = self
                .tree
                .links()
                .iter()
                .position(|name| name == link)
                .ok_or_else(|| DynamicsError::Absent(link.clone()))?;
            self.inertia[index] = *inertia;
        }
        let mut seen = std::collections::HashSet::new();
        for (joint, friction) in &calibration.joints {
            if !seen.insert(joint.as_str()) {
                return Err(DynamicsError::Repeated(joint.clone()));
            }
            let index = self
                .tree
                .joints()
                .iter()
                .position(|axis| &axis.name == joint)
                .ok_or_else(|| DynamicsError::Absent(joint.clone()))?;
            self.friction[index] = *friction;
        }
        self.stiction_velocity = calibration.stiction_velocity;
        self.loaded = self.friction.iter().any(JointFriction::is_loaded);
        Ok(self)
    }

    /// The tree the dynamics run over; its [`Tree::joints`] order indexes every vector here.
    #[must_use]
    pub const fn tree(&self) -> &Tree {
        &self.tree
    }

    #[must_use]
    pub const fn gravity(&self) -> [f64; 3] {
        self.gravity
    }

    /// Buffers sized for this tree, for [`Dynamics::inverse_into`] to reuse every tick.
    #[must_use]
    pub fn scratch(&self) -> Scratch {
        let links = self.tree.links().len();
        Scratch {
            motion: vec![Motion::REST; links],
            placed: vec![[0.0; 16]; self.tree.joints.len()],
            force: vec![[0.0; 3]; links],
            moment: vec![[0.0; 3]; links],
            rest: vec![0.0; self.tree.joints().len()],
            held: vec![0.0; self.tree.joints().len()],
        }
    }

    /// The torque (force, for a prismatic joint) each independent joint needs to move at
    /// `velocity` and `acceleration` through `position`: rigid body, gravity and friction.
    /// Allocates its answer; a control tick calls [`Dynamics::inverse_into`].
    ///
    /// # Errors
    ///
    /// As [`Dynamics::inverse_into`].
    pub fn inverse(
        &self,
        position: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
    ) -> Result<Vec<f64>, DynamicsError> {
        let mut torque = vec![0.0; self.tree.joints().len()];
        self.inverse_into(
            position,
            velocity,
            acceleration,
            &mut self.scratch(),
            &mut torque,
        )?;
        Ok(torque)
    }

    /// [`Dynamics::inverse`] into `torque`, through `scratch` (from [`Dynamics::scratch`]):
    /// no allocation unless it fails.
    ///
    /// # Errors
    ///
    /// [`DynamicsError::Length`] unless each vector (and `torque`) has one value per
    /// [`Tree::joints`] and `scratch` was sized for this tree, and [`DynamicsError::NonFinite`]
    /// or [`DynamicsError::Placement`] for a non-finite value.
    pub fn inverse_into(
        &self,
        position: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        scratch: &mut Scratch,
        torque: &mut [f64],
    ) -> Result<(), DynamicsError> {
        self.rigid(position, velocity, acceleration, scratch, torque)?;
        if self.loaded {
            // The load is gravity's torque on each joint; the drive delivers the rigid torque.
            let mut held = std::mem::take(&mut scratch.held);
            let outcome = self.gravity_torque_into(position, scratch, &mut held);
            scratch.held = held;
            outcome?;
        }
        for (index, friction) in self.friction.iter().enumerate() {
            let external = if self.loaded {
                -scratch.held[index]
            } else {
                0.0
            };
            torque[index] += friction.torque(
                velocity[index],
                acceleration[index],
                torque[index],
                external,
                self.stiction_velocity,
            );
        }
        Ok(())
    }

    /// The torque that holds `position` still against gravity: [`Dynamics::inverse`] at rest.
    ///
    /// # Errors
    ///
    /// As [`Dynamics::inverse_into`].
    pub fn gravity_torque_into(
        &self,
        position: &[f64],
        scratch: &mut Scratch,
        torque: &mut [f64],
    ) -> Result<(), DynamicsError> {
        let rest = std::mem::take(&mut scratch.rest);
        let held = self.rigid(position, &rest, &rest, scratch, torque);
        scratch.rest = rest;
        held
    }

    fn check(
        &self,
        position: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        scratch: &Scratch,
        torque: &[f64],
    ) -> Result<(), DynamicsError> {
        let axes = self.tree.joints();
        for found in [
            position.len(),
            velocity.len(),
            acceleration.len(),
            torque.len(),
        ] {
            if found != axes.len() {
                return Err(DynamicsError::Length {
                    expected: axes.len(),
                    found,
                });
            }
        }
        if scratch.motion.len() != self.tree.links().len()
            || scratch.placed.len() != self.tree.joints.len()
        {
            return Err(DynamicsError::Length {
                expected: self.tree.links().len(),
                found: scratch.motion.len(),
            });
        }
        for (index, axis) in axes.iter().enumerate() {
            if !position[index].is_finite() {
                return Err(PlacementError::NonFinite(axis.name.clone()).into());
            }
            if !(velocity[index].is_finite() && acceleration[index].is_finite()) {
                return Err(DynamicsError::NonFinite(axis.name.clone()));
            }
        }
        Ok(())
    }

    /// Recursive Newton-Euler: velocities and accelerations root to leaf ([`Dynamics::motion`]),
    /// each link's wrench about its origin, then wrenches leaf to root, each joint taking its
    /// axis' share.
    fn rigid(
        &self,
        position: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        scratch: &mut Scratch,
        torque: &mut [f64],
    ) -> Result<(), DynamicsError> {
        self.check(position, velocity, acceleration, scratch, torque)?;
        self.motion(position, velocity, acceleration, scratch);
        let Scratch {
            motion,
            placed,
            force,
            moment,
            ..
        } = scratch;
        for (link, (inertia, link_motion)) in self.inertia.iter().zip(motion.iter()).enumerate() {
            let Motion {
                spin,
                spin_rate,
                acceleration,
            } = *link_motion;
            let h = inertia.first_moment;
            let tensor = symmetric(inertia.inertia);
            force[link] = add(
                add(scale(acceleration, inertia.mass), cross(spin_rate, h)),
                cross(spin, cross(spin, h)),
            );
            moment[link] = add(
                add(times(&tensor, spin_rate), cross(spin, times(&tensor, spin))),
                cross(h, acceleration),
            );
        }
        torque.fill(0.0);
        for ((joint, leader), local) in self
            .tree
            .joints
            .iter()
            .zip(&self.leader)
            .zip(placed.iter())
            .rev()
        {
            let (f, n) = (force[joint.child], moment[joint.child]);
            if let Some(index) = leader {
                let along_axis = if joint.kind == JointType::Prismatic {
                    dot(joint.axis, f)
                } else {
                    dot(joint.axis, n)
                };
                torque[*index] += joint.multiplier * along_axis;
            }
            let turn = rotation(local);
            let offset = [local[3], local[7], local[11]];
            let f_parent = times(&turn, f);
            let n_parent = add(times(&turn, n), cross(offset, f_parent));
            force[joint.parent] = add(force[joint.parent], f_parent);
            moment[joint.parent] = add(moment[joint.parent], n_parent);
        }
        Ok(())
    }

    /// Each link's motion at its origin, on its own axes, in [`Tree::links`] order, and each
    /// tree joint's `parent_from_child` at the positions given, into `scratch`. The root
    /// accelerates against gravity, which loads every link with its weight.
    fn motion(
        &self,
        position: &[f64],
        velocity: &[f64],
        acceleration: &[f64],
        scratch: &mut Scratch,
    ) {
        let motion = &mut scratch.motion;
        motion.fill(Motion::REST);
        motion[self.tree.root_index()].acceleration = scale(self.gravity, -1.0);
        for ((joint, leader), placed) in self
            .tree
            .joints
            .iter()
            .zip(&self.leader)
            .zip(scratch.placed.iter_mut())
        {
            let (q, qd, qdd) = leader.map_or((0.0, 0.0, 0.0), |index| {
                (
                    joint.multiplier.mul_add(position[index], joint.offset),
                    joint.multiplier * velocity[index],
                    joint.multiplier * acceleration[index],
                )
            });
            let local = match joint.kind {
                JointType::Fixed => joint.origin,
                JointType::Prismatic => compose(&joint.origin, &along(joint.axis, q)),
                JointType::Revolute | JointType::Continuous => {
                    compose(&joint.origin, &about(joint.axis, q))
                }
            };
            let turn = rotation(&local);
            let offset = [local[3], local[7], local[11]];
            let parent = motion[joint.parent];
            let carried = add(
                add(parent.acceleration, cross(parent.spin_rate, offset)),
                cross(parent.spin, cross(parent.spin, offset)),
            );
            let spin = transpose_times(&turn, parent.spin);
            let spin_rate = transpose_times(&turn, parent.spin_rate);
            let carried = transpose_times(&turn, carried);
            let z = joint.axis;
            motion[joint.child] = match joint.kind {
                JointType::Revolute | JointType::Continuous => Motion {
                    spin: add(spin, scale(z, qd)),
                    spin_rate: add(add(spin_rate, scale(z, qdd)), cross(spin, scale(z, qd))),
                    acceleration: carried,
                },
                JointType::Prismatic => Motion {
                    spin,
                    spin_rate,
                    acceleration: add(add(carried, scale(cross(spin, z), 2.0 * qd)), scale(z, qdd)),
                },
                JointType::Fixed => Motion {
                    spin,
                    spin_rate,
                    acceleration: carried,
                },
            };
            *placed = local;
        }
    }
}

/// The buffers one evaluation of [`Dynamics`] works in, sized once for its tree
/// ([`Dynamics::scratch`]) so a control tick reusing it allocates nothing.
#[derive(Clone, Debug)]
pub struct Scratch {
    motion: Vec<Motion>,
    placed: Vec<Transform>,
    force: Vec<[f64; 3]>,
    moment: Vec<[f64; 3]>,
    rest: Vec<f64>,
    /// Gravity's holding torque, for load-dependent friction.
    held: Vec<f64>,
}

/// One link's motion at its origin, on its own axes.
#[derive(Clone, Copy, Debug)]
struct Motion {
    spin: [f64; 3],
    spin_rate: [f64; 3],
    /// The origin's linear acceleration.
    acceleration: [f64; 3],
}

impl Motion {
    const REST: Self = Self {
        spin: [0.0; 3],
        spin_rate: [0.0; 3],
        acceleration: [0.0; 3],
    };
}

/// The lowercase hex SHA-256 of a description's bytes, as `sx_embodiments` names it.
#[must_use]
pub fn sha256_hex(text: &str) -> String {
    use sha2::{Digest, Sha256};
    use std::fmt::Write;
    Sha256::digest(text.as_bytes())
        .iter()
        .fold(String::with_capacity(64), |mut out, byte| {
            let _ = write!(out, "{byte:02x}");
            out
        })
}

type Matrix = [[f64; 3]; 3];

fn rotation(m: &Transform) -> Matrix {
    [[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]]
}

const fn symmetric([xx, xy, xz, yy, yz, zz]: [f64; 6]) -> Matrix {
    [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]
}

fn times(m: &Matrix, v: [f64; 3]) -> [f64; 3] {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

fn transpose_times(m: &Matrix, v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
        m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
        m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2],
    ]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// Every principal minor of a symmetric 3x3 is at least `-tolerance`.
fn is_semidefinite(m: &Matrix, tolerance: f64) -> bool {
    let diagonal = (0..3).all(|i| m[i][i] >= -tolerance);
    let pairs = [(0, 1), (0, 2), (1, 2)].iter().all(|&(i, j)| {
        m[i][i].mul_add(m[j][j], -(m[i][j] * m[i][j])) >= -tolerance * tolerance.max(1.0)
    });
    let determinant = m[0][0] * m[1][1].mul_add(m[2][2], -(m[1][2] * m[2][1]))
        - m[0][1] * m[1][0].mul_add(m[2][2], -(m[1][2] * m[2][0]))
        + m[0][2] * m[1][0].mul_add(m[2][1], -(m[1][1] * m[2][0]));
    diagonal && pairs && determinant >= -tolerance * tolerance.max(1.0) * tolerance.max(1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(text: &str) -> String {
        sha256_hex(text)
    }

    const PENDULUM: &str = r#"<robot name="pendulum">
      <link name="base"/>
      <link name="arm">
        <inertial><origin xyz="0 0 -0.5"/><mass value="2"/>
          <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
      </link>
      <joint name="hinge" type="revolute">
        <parent link="base"/><child link="arm"/><axis xyz="0 1 0"/>
        <limit lower="-3" upper="3"/>
      </joint>
    </robot>"#;

    #[test]
    fn a_pendulum_needs_m_g_l_sin_q_plus_its_inertia() {
        let dynamics = Dynamics::new(Tree::parse(PENDULUM).expect("tree"), UPRIGHT);
        let q = 0.3_f64;
        let qdd = 1.7;
        let tau = dynamics.inverse(&[q], &[0.9], &[qdd]).expect("torque");
        // Hanging along -z, turning about +y: gravity pulls back with -m g l sin q.
        let expected = (0.01 + 2.0 * 0.25) * qdd + 2.0 * STANDARD_GRAVITY * 0.5 * q.sin();
        assert!(
            (tau[0] - expected).abs() < 1e-12,
            "{} != {expected}",
            tau[0]
        );
    }

    #[test]
    fn friction_adds_armature_viscous_and_a_smooth_coulomb() {
        let tree = Tree::parse(PENDULUM).expect("tree");
        let calibration = DynamicsCalibration {
            description_sha256: digest(PENDULUM),
            stiction_velocity: 0.05,
            links: vec![],
            joints: vec![(
                "hinge".into(),
                JointFriction {
                    coulomb: 0.4,
                    viscous: 0.2,
                    armature: 0.03,
                    ..JointFriction::default()
                },
            )],
        };
        let plain = Dynamics::new(tree, UPRIGHT);
        let rubbed = Dynamics::of(PENDULUM, UPRIGHT)
            .expect("dynamics")
            .calibrated(&calibration)
            .expect("calibrated");
        let (q, qd, qdd) = (0.2, 0.1, 2.0);
        let added = rubbed.inverse(&[q], &[qd], &[qdd]).expect("torque")[0]
            - plain.inverse(&[q], &[qd], &[qdd]).expect("torque")[0];
        let expected = 0.03 * qdd + 0.2 * qd + 0.4 * (qd / 0.05_f64).tanh();
        assert!((added - expected).abs() < 1e-12);
        assert_eq!(
            rubbed.inverse(&[q], &[0.0], &[0.0]),
            plain.inverse(&[q], &[0.0], &[0.0])
        );
    }

    #[test]
    fn a_calibration_of_another_description_or_absent_link_is_refused() {
        let tree = Tree::parse(PENDULUM).expect("tree");
        let mut calibration = DynamicsCalibration {
            description_sha256: digest(PENDULUM),
            stiction_velocity: 0.02,
            links: vec![("elsewhere".into(), LinkInertia::ZERO)],
            joints: vec![],
        };
        assert!(matches!(
            Dynamics::new(tree, UPRIGHT).calibrated(&calibration),
            Err(DynamicsError::Description { .. })
        ));
        let dynamics = Dynamics::of(PENDULUM, UPRIGHT).expect("dynamics");
        let mut elsewhere = calibration.clone();
        elsewhere.description_sha256 = "0".repeat(64);
        assert!(matches!(
            dynamics.clone().calibrated(&elsewhere),
            Err(DynamicsError::Description { .. })
        ));
        assert_eq!(
            dynamics.clone().calibrated(&calibration),
            Err(DynamicsError::Absent("elsewhere".into()))
        );
        calibration.links = vec![
            ("arm".into(), LinkInertia::ZERO),
            ("arm".into(), LinkInertia::ZERO),
        ];
        assert_eq!(
            dynamics.calibrated(&calibration),
            Err(DynamicsError::Repeated("arm".into()))
        );
    }

    #[test]
    fn physical_inertia_obeys_the_triangle_inequality() {
        let rod = LinkInertia::from_centre(
            1.0,
            &crate::kinematics::from_xyz_rpy([0.1, 0.0, 0.0], [0.0; 3]),
            [0.0, 0.0, 0.0, 0.1, 0.0, 0.1],
        );
        assert!(rod.is_physical());
        // Principal moments 0.1, 0.1, 0.3: 0.3 > 0.1 + 0.1.
        let impossible = LinkInertia::from_centre(
            1.0,
            &crate::kinematics::from_xyz_rpy([0.0; 3], [0.0; 3]),
            [0.1, 0.0, 0.0, 0.1, 0.0, 0.3],
        );
        assert!(!impossible.is_physical());
        let negative = LinkInertia { mass: -1.0, ..rod };
        assert!(!negative.is_physical());
    }
}
