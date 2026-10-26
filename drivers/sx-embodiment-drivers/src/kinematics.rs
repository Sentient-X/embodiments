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

use crate::dynamics::LinkInertia;
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
pub(crate) fn about([x, y, z]: [f64; 3], angle: f64) -> Transform {
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

pub(crate) fn along([x, y, z]: [f64; 3], distance: f64) -> Transform {
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

/// A description cannot be read as a joint tree.
#[derive(Debug, Error, PartialEq)]
pub enum TreeError {
    #[error("the description is not XML: {0}")]
    Xml(String),
    #[error("{0}")]
    Malformed(String),
}

/// One joint of a [`Tree`], placed: `root_from_child = root_from_parent · origin · motion`,
/// where the motion turns about (or slides along) `axis` by `multiplier * leader + offset`.
#[derive(Clone, Debug, PartialEq)]
pub struct TreeJoint {
    /// Fixed for a fixed, floating or planar joint (which holds its origin).
    pub kind: JointType,
    /// Index into [`Tree::links`].
    pub parent: usize,
    pub child: usize,
    /// `parent_from_child` at zero.
    pub origin: Transform,
    /// Unit length, in the child's frame (which is the joint's frame).
    pub axis: [f64; 3],
    /// The independent joint this one's position comes from, as `multiplier * leader +
    /// offset` with mimic chains folded; itself with `(1, 0)` when it is independent.
    pub leader: String,
    pub multiplier: f64,
    pub offset: f64,
}

/// A description's whole joint tree, parsed once: every link's pose in the root link's frame.
///
/// The same reading as `sx_embodiments.kinematics.Kinematics`: a URDF origin is translation
/// then fixed-axis roll, pitch, yaw; a revolute or continuous joint turns about its unit axis,
/// a prismatic one slides along it; a `<mimic>` joint takes `multiplier * leader + offset`
/// (chains of mimics fold to the one independent joint), so the YUBI's fingers, nails and
/// connectors all follow the one encoder joint. A floating or planar joint has no one-number
/// position: it holds its origin, and the subtree below it is placed by its consumer (each
/// YUBI hand by the operator's controller). Links are in description order, the render pack's
/// link order.
#[derive(Clone, Debug, PartialEq)]
pub struct Tree {
    links: Vec<String>,
    root: usize,
    /// Parent before child.
    pub(crate) joints: Vec<TreeJoint>,
    independent: Vec<Axis>,
    /// Each link's `<inertial>`, in [`Tree::links`] order; zero where the description has none.
    pub(crate) inertials: Vec<LinkInertia>,
}

/// One joint whose position places the tree: moving, and mimicking none.
#[derive(Clone, Debug, PartialEq)]
pub struct Axis {
    pub name: String,
    /// Revolute, continuous or prismatic.
    pub kind: JointType,
    /// The description's `<limit lower upper>` (radians or metres); `None` for continuous.
    pub limits: Option<[f64; 2]>,
    /// The description's `<limit velocity>` (unit/s), when it declares one above zero.
    pub velocity: Option<f64>,
}

fn attribute_floats<const N: usize>(
    node: Option<roxmltree::Node>,
    key: &str,
    default: [f64; N],
) -> Result<[f64; N], TreeError> {
    let Some(text) = node.and_then(|n| n.attribute(key)) else {
        return Ok(default);
    };
    let values: Vec<f64> = text
        .split_whitespace()
        .map(str::parse)
        .collect::<Result<_, _>>()
        .map_err(|_| TreeError::Malformed(format!("{key}={text:?} is not {N} numbers")))?;
    let values: [f64; N] = values
        .try_into()
        .map_err(|_| TreeError::Malformed(format!("{key}={text:?} is not {N} numbers")))?;
    if values.iter().all(|v| v.is_finite()) {
        Ok(values)
    } else {
        Err(TreeError::Malformed(format!(
            "{key}={text:?} is not finite"
        )))
    }
}

fn child_of<'a, 'input>(
    node: roxmltree::Node<'a, 'input>,
    tag: &str,
) -> Option<roxmltree::Node<'a, 'input>> {
    node.children()
        .find(|n| n.is_element() && n.tag_name().name() == tag)
}

fn number(node: roxmltree::Node, key: &str, default: f64) -> Result<f64, TreeError> {
    node.attribute(key).map_or(Ok(default), |text| {
        text.trim()
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite())
            .ok_or_else(|| TreeError::Malformed(format!("{key}={text:?} is not a finite number")))
    })
}

/// A URDF origin: translation, then fixed-axis roll, pitch, yaw.
#[must_use]
pub fn from_xyz_rpy([x, y, z]: [f64; 3], [roll, pitch, yaw]: [f64; 3]) -> Transform {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [
        cy * cp,
        cy * sp * sr - sy * cr,
        cy * sp * cr + sy * sr,
        x, //
        sy * cp,
        sy * sp * sr + cy * cr,
        sy * sp * cr - cy * sr,
        y, //
        -sp,
        cp * sr,
        cp * cr,
        z, //
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

/// A link's `<inertial>` as [`LinkInertia`]: the mass at its origin's centre, the inertia
/// tensor given there on the origin's axes, carried to the link's own origin and axes.
fn read_inertial(inertial: Option<roxmltree::Node>, link: &str) -> Result<LinkInertia, TreeError> {
    let Some(inertial) = inertial else {
        return Ok(LinkInertia::ZERO);
    };
    let origin = child_of(inertial, "origin");
    let frame = from_xyz_rpy(
        attribute_floats(origin, "xyz", [0.0; 3])?,
        attribute_floats(origin, "rpy", [0.0; 3])?,
    );
    let mass = child_of(inertial, "mass").map_or(Ok(0.0), |m| number(m, "value", 0.0))?;
    let tensor = child_of(inertial, "inertia");
    let read = |key: &str| tensor.map_or(Ok(0.0), |t| number(t, key, 0.0));
    let about_centre = [
        read("ixx")?,
        read("ixy")?,
        read("ixz")?,
        read("iyy")?,
        read("iyz")?,
        read("izz")?,
    ];
    if mass < 0.0 {
        return Err(TreeError::Malformed(format!(
            "link {link:?} has a negative mass"
        )));
    }
    Ok(LinkInertia::from_centre(mass, &frame, about_centre))
}

struct RawJoint {
    name: String,
    kind: Option<JointType>,
    parent: usize,
    child: usize,
    origin: Transform,
    axis: [f64; 3],
    mimic: Option<(String, f64, f64)>,
    limits: Option<[f64; 2]>,
    velocity: Option<f64>,
}

impl RawJoint {
    fn read(element: roxmltree::Node, links: &[String]) -> Result<Self, TreeError> {
        let index = |name: Option<&str>, joint: &str| {
            name.and_then(|n| links.iter().position(|l| l == n))
                .ok_or_else(|| {
                    TreeError::Malformed(format!(
                        "joint {joint:?} joins links the description lacks"
                    ))
                })
        };
        let name = element
            .attribute("name")
            .ok_or_else(|| TreeError::Malformed("a joint has no name".into()))?;
        let parent = index(
            child_of(element, "parent").and_then(|n| n.attribute("link")),
            name,
        )?;
        let child = index(
            child_of(element, "child").and_then(|n| n.attribute("link")),
            name,
        )?;
        let kind = match element.attribute("type").unwrap_or_default() {
            "fixed" => Some(JointType::Fixed),
            "revolute" => Some(JointType::Revolute),
            "continuous" => Some(JointType::Continuous),
            "prismatic" => Some(JointType::Prismatic),
            "floating" | "planar" => None,
            other => {
                return Err(TreeError::Malformed(format!(
                    "joint {name:?} has type {other:?}, which URDF does not define"
                )));
            }
        };
        let origin = child_of(element, "origin");
        let axis = attribute_floats(child_of(element, "axis"), "xyz", [1.0, 0.0, 0.0])?;
        let norm = axis.iter().map(|v| v * v).sum::<f64>().sqrt();
        let moves = matches!(
            kind,
            Some(JointType::Revolute | JointType::Continuous | JointType::Prismatic)
        );
        if moves && norm < 1e-12 {
            return Err(TreeError::Malformed(format!(
                "joint {name:?} moves about a zero axis"
            )));
        }
        let mimic = match child_of(element, "mimic") {
            None => None,
            Some(m) => Some((
                m.attribute("joint")
                    .ok_or_else(|| TreeError::Malformed(format!("joint {name:?} mimics no joint")))?
                    .to_owned(),
                number(m, "multiplier", 1.0)?,
                number(m, "offset", 0.0)?,
            )),
        };
        Ok(Self {
            name: name.to_owned(),
            kind,
            parent,
            child,
            origin: from_xyz_rpy(
                attribute_floats(origin, "xyz", [0.0; 3])?,
                attribute_floats(origin, "rpy", [0.0; 3])?,
            ),
            axis: if norm >= 1e-12 {
                [axis[0] / norm, axis[1] / norm, axis[2] / norm]
            } else {
                [1.0, 0.0, 0.0]
            },
            mimic,
            limits: match (kind, child_of(element, "limit")) {
                (Some(JointType::Revolute | JointType::Prismatic), limit) => Some([
                    limit.map_or(Ok(0.0), |l| number(l, "lower", 0.0))?,
                    limit.map_or(Ok(0.0), |l| number(l, "upper", 0.0))?,
                ]),
                _ => None,
            },
            velocity: match child_of(element, "limit") {
                Some(limit) => Some(number(limit, "velocity", 0.0)?).filter(|v| *v > 0.0),
                None => None,
            },
        })
    }

    /// This joint with its mimic chain folded onto the one independent joint it follows.
    fn fold(raw: &[Self], j: usize) -> Result<TreeJoint, TreeError> {
        let joint = &raw[j];
        let (mut leader, mut multiplier, mut offset) = (j, 1.0_f64, 0.0_f64);
        let mut seen = vec![j];
        while let Some((name, m, o)) = &raw[leader].mimic {
            let next = raw.iter().position(|r| &r.name == name).ok_or_else(|| {
                TreeError::Malformed(format!(
                    "joint {:?} mimics an absent joint",
                    raw[leader].name
                ))
            })?;
            if seen.contains(&next) {
                return Err(TreeError::Malformed(format!(
                    "joint {:?} mimics itself through a cycle",
                    joint.name
                )));
            }
            seen.push(next);
            // x_self = multiplier * x_leader + offset, x_leader = m * x_next + o.
            offset = multiplier.mul_add(*o, offset);
            multiplier *= m;
            leader = next;
        }
        Ok(TreeJoint {
            kind: joint.kind.unwrap_or(JointType::Fixed),
            parent: joint.parent,
            child: joint.child,
            origin: joint.origin,
            axis: joint.axis,
            leader: raw[leader].name.clone(),
            multiplier,
            offset,
        })
    }
}

impl Tree {
    /// Read a description's links and joints.
    ///
    /// # Errors
    ///
    /// [`TreeError`] when the bytes are not XML, a joint names an absent link, a link has two
    /// parents or none reaches the one root, a type is not URDF's, a moving axis is zero, or a
    /// mimic names an absent joint or a cycle.
    pub fn parse(urdf: &str) -> Result<Self, TreeError> {
        let document =
            roxmltree::Document::parse(urdf).map_err(|e| TreeError::Xml(e.to_string()))?;
        let description = document.root_element();
        let elements = |tag: &'static str| {
            description
                .children()
                .filter(move |n| n.is_element() && n.tag_name().name() == tag)
        };
        let mut links: Vec<String> = Vec::new();
        let mut inertials: Vec<LinkInertia> = Vec::new();
        for link in elements("link") {
            let name = link
                .attribute("name")
                .ok_or_else(|| TreeError::Malformed("a link has no name".into()))?;
            links.push(name.to_owned());
            inertials.push(read_inertial(child_of(link, "inertial"), name)?);
        }
        let mut raw: Vec<RawJoint> = Vec::new();
        let mut parent_joint: Vec<Option<usize>> = vec![None; links.len()];
        for element in elements("joint") {
            let joint = RawJoint::read(element, &links)?;
            if parent_joint[joint.child].is_some() {
                return Err(TreeError::Malformed(format!(
                    "link {:?} has two parent joints",
                    links[joint.child]
                )));
            }
            parent_joint[joint.child] = Some(raw.len());
            raw.push(joint);
        }
        let roots: Vec<usize> = (0..links.len())
            .filter(|&l| parent_joint[l].is_none())
            .collect();
        let [root] = roots[..] else {
            return Err(TreeError::Malformed(format!(
                "a tree has one root link; this description has {}",
                roots.len()
            )));
        };
        // Parent before child: breadth first from the root. A link it never reaches sits on a cycle.
        let mut order: Vec<usize> = Vec::with_capacity(raw.len());
        let mut frontier = vec![root];
        while let Some(link) = frontier.pop() {
            for (j, joint) in raw.iter().enumerate() {
                if joint.parent == link {
                    order.push(j);
                    frontier.push(joint.child);
                }
            }
        }
        if order.len() != raw.len() {
            return Err(TreeError::Malformed(
                "the description's links form a cycle".into(),
            ));
        }
        let independent = raw
            .iter()
            .filter(|j| j.kind.is_some_and(|k| k != JointType::Fixed) && j.mimic.is_none())
            .map(|j| Axis {
                name: j.name.clone(),
                kind: j.kind.unwrap_or(JointType::Fixed),
                limits: j.limits,
                velocity: j.velocity,
            })
            .collect();
        let joints = order
            .iter()
            .map(|&j| RawJoint::fold(&raw, j))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            links,
            root,
            joints,
            independent,
            inertials,
        })
    }

    /// Every link, in description order: the order [`Tree::link_poses`] answers in.
    #[must_use]
    pub fn links(&self) -> &[String] {
        &self.links
    }

    pub(crate) const fn root_index(&self) -> usize {
        self.root
    }

    /// The root link, whose frame every pose is in.
    #[must_use]
    pub fn root(&self) -> &str {
        &self.links[self.root]
    }

    /// The joints whose positions place the tree: every moving joint that mimics none, in
    /// description order, with its kind and limits. [`Tree::link_poses`] asks for exactly
    /// these, and refuses a missing one.
    #[must_use]
    pub fn joints(&self) -> &[Axis] {
        &self.independent
    }

    /// Every joint, parent before child: what [`Tree::link_poses`] composes, for a consumer
    /// that differentiates the tree (an inverse-kinematics solver) or evaluates it many times.
    #[must_use]
    pub fn placed_joints(&self) -> &[TreeJoint] {
        &self.joints
    }

    /// Every link's `root_from_link` at the positions `position` answers for each independent
    /// joint, in [`Tree::links`] order.
    ///
    /// # Errors
    ///
    /// [`PlacementError`] when an independent joint the tree moves has no finite position.
    pub fn link_poses(
        &self,
        position: impl Fn(&str) -> Option<f64>,
    ) -> Result<Vec<Transform>, PlacementError> {
        let mut poses = vec![IDENTITY; self.links.len()];
        for joint in &self.joints {
            let local = if joint.kind == JointType::Fixed {
                joint.origin
            } else {
                let x = position(&joint.leader)
                    .ok_or_else(|| PlacementError::Missing(joint.leader.clone()))?;
                if !x.is_finite() {
                    return Err(PlacementError::NonFinite(joint.leader.clone()));
                }
                let q = joint.multiplier.mul_add(x, joint.offset);
                let motion = if joint.kind == JointType::Prismatic {
                    along(joint.axis, q)
                } else {
                    about(joint.axis, q)
                };
                compose(&joint.origin, &motion)
            };
            poses[joint.child] = compose(&poses[joint.parent], &local);
        }
        Ok(poses)
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

    const TREE_POSES: &str = include_str!("../tests/fixtures/tree-poses.json");
    const STATIC_UPPER_BODY: &str =
        include_str!("../../../render_packs/sentient-static-upper-body/robot.urdf");
    const YUBI: &str = include_str!("../../../render_packs/yubi/robot.urdf");

    #[derive(Deserialize)]
    struct TreeSample {
        positions: HashMap<String, f64>,
        poses: HashMap<String, Vec<f64>>,
    }

    #[test]
    fn the_tree_places_every_link_where_the_description_does() {
        let samples: HashMap<String, Vec<TreeSample>> =
            serde_json::from_str(TREE_POSES).expect("tree poses");
        for (key, urdf) in [
            ("sentient-static-upper-body", STATIC_UPPER_BODY),
            ("yubi", YUBI),
        ] {
            let tree = Tree::parse(urdf).expect("tree");
            assert_eq!(samples[key].len(), 4);
            for sample in &samples[key] {
                let mut joints: Vec<&String> = sample.positions.keys().collect();
                joints.sort();
                let mut independent: Vec<&String> = tree.joints().iter().map(|a| &a.name).collect();
                independent.sort();
                assert_eq!(joints, independent, "{key}");
                let poses = tree
                    .link_poses(|j| sample.positions.get(j).copied())
                    .expect("poses");
                assert_eq!(poses.len(), sample.poses.len(), "{key}: every link");
                for (link, pose) in tree.links().iter().zip(&poses) {
                    for (got, want) in pose.iter().zip(&sample.poses[link]) {
                        assert!((got - want).abs() <= 1e-9, "{key} {link}: {got} != {want}");
                    }
                }
            }
        }
    }

    #[test]
    fn yubi_fingers_follow_the_one_encoder_joint() {
        let tree = Tree::parse(YUBI).expect("tree");
        assert_eq!(
            tree.joints()
                .iter()
                .map(|a| a.name.as_str())
                .collect::<Vec<_>>(),
            ["left_left_finger", "right_left_finger"]
        );
        let jaw = &tree.joints()[0];
        assert_eq!(jaw.kind, JointType::Revolute);
        let [lower, upper] = jaw.limits.expect("revolute limits");
        assert!(lower < upper);

        let at = |q: f64| tree.link_poses(|_| Some(q)).expect("poses");
        let (open, shut) = (at(0.3), at(0.0));
        let moved = |link: &str| {
            let i = tree.links().iter().position(|l| l == link).expect("link");
            open[i]
                .iter()
                .zip(shut[i])
                .any(|(a, b)| (a - b).abs() > 1e-9)
        };
        for link in [
            "left_link_right_finger",
            "left_right_nail",
            "left_link_connector_left",
        ] {
            assert!(moved(link), "{link} follows left_left_finger");
        }
        assert!(!moved("left_base_link"));
    }

    #[test]
    fn a_mimic_of_a_mimic_folds_to_the_leader() {
        let urdf = r#"<robot name="r">
            <link name="a"/><link name="b"/><link name="c"/><link name="d"/>
            <joint name="lead" type="prismatic"><parent link="a"/><child link="b"/><axis xyz="1 0 0"/></joint>
            <joint name="m1" type="prismatic"><parent link="a"/><child link="c"/><axis xyz="0 2 0"/>
              <mimic joint="lead" multiplier="2" offset="0.5"/></joint>
            <joint name="m2" type="prismatic"><parent link="c"/><child link="d"/><axis xyz="0 0 1"/>
              <mimic joint="m1" multiplier="-1" offset="0.25"/></joint>
        </robot>"#;
        let tree = Tree::parse(urdf).expect("tree");
        assert_eq!(tree.root(), "a");
        assert_eq!(tree.joints()[0].name, "lead");
        assert_eq!(tree.joints()[0].kind, JointType::Prismatic);
        assert_eq!(tree.joints()[0].limits, Some([0.0, 0.0]));
        let poses = tree
            .link_poses(|j| (j == "lead").then_some(1.0))
            .expect("poses");
        // m1 = 2.5 along y; m2 = -(2.5) + 0.25 = -2.25 along z.
        assert!((poses[2][7] - 2.5).abs() < 1e-12);
        assert!((poses[3][7] - 2.5).abs() < 1e-12 && (poses[3][11] + 2.25).abs() < 1e-12);
        assert_eq!(
            tree.link_poses(|_| None),
            Err(PlacementError::Missing("lead".into()))
        );
        assert_eq!(
            tree.link_poses(|_| Some(f64::INFINITY)),
            Err(PlacementError::NonFinite("lead".into()))
        );
    }

    #[test]
    fn a_description_that_is_not_one_tree_is_refused() {
        let two_roots = r#"<robot name="r"><link name="a"/><link name="b"/></robot>"#;
        assert!(matches!(
            Tree::parse(two_roots),
            Err(TreeError::Malformed(_))
        ));
        let cycle = r#"<robot name="r"><link name="a"/><link name="b"/><link name="c"/>
            <joint name="x" type="fixed"><parent link="b"/><child link="c"/></joint>
            <joint name="y" type="fixed"><parent link="c"/><child link="b"/></joint></robot>"#;
        assert!(matches!(Tree::parse(cycle), Err(TreeError::Malformed(_))));
        let self_mimic = r#"<robot name="r"><link name="a"/><link name="b"/>
            <joint name="x" type="revolute"><parent link="a"/><child link="b"/>
              <mimic joint="x"/></joint></robot>"#;
        assert!(matches!(
            Tree::parse(self_mimic),
            Err(TreeError::Malformed(_))
        ));
        assert!(matches!(Tree::parse("<robot"), Err(TreeError::Xml(_))));
    }
}
