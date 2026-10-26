//! The tree compiled for many evaluations a second: links and joints by index, every link's
//! pose, and the Jacobian of any point riding a link.

use sx_embodiment_drivers::kinematics::{Axis, JointType, Tree, TreeJoint};

use crate::math::{Iso, V3, about, add, cross, mat_vec, scale, sub};

#[derive(Clone, Debug)]
struct Joint {
    kind: JointType,
    parent: usize,
    child: usize,
    origin: Iso,
    axis: V3,
    /// Index into [`Model::axes`], `None` for a fixed joint.
    leader: Option<usize>,
    multiplier: f64,
    offset: f64,
}

/// A moving joint above a link, as one Jacobian term: `multiplier` times the joint's motion
/// moves the link, and the joint's position is axis `axis`.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Term {
    pub joint: usize,
    pub axis: usize,
    pub multiplier: f64,
}

/// [`Tree`] by index: [`Model::poses`] is [`Tree::link_poses`] without names.
#[derive(Clone, Debug)]
pub struct Model {
    links: Vec<String>,
    root: usize,
    joints: Vec<Joint>,
    axes: Vec<Axis>,
    /// For each link, the moving joints above it, root first.
    above: Vec<Vec<Term>>,
}

/// Every link's pose in the root frame, and each joint's axis there.
#[derive(Clone, Debug)]
pub struct Frames {
    pub links: Vec<Iso>,
    /// Per joint, its axis in the root frame (the child link's rotation applied).
    axes: Vec<V3>,
}

impl Model {
    #[must_use]
    pub fn new(tree: &Tree) -> Self {
        let links = tree.links().to_vec();
        let root = links.iter().position(|l| l == tree.root()).unwrap_or(0);
        let axes = tree.joints().to_vec();
        let joints: Vec<Joint> = tree
            .placed_joints()
            .iter()
            .map(|j: &TreeJoint| Joint {
                kind: j.kind,
                parent: j.parent,
                child: j.child,
                origin: Iso::from_transform(&j.origin),
                axis: j.axis,
                leader: if j.kind == JointType::Fixed {
                    None
                } else {
                    axes.iter().position(|a| a.name == j.leader)
                },
                multiplier: j.multiplier,
                offset: j.offset,
            })
            .collect();
        let mut above: Vec<Vec<Term>> = vec![Vec::new(); links.len()];
        // Parent before child, so a parent's list is complete when its child is reached.
        for (index, joint) in joints.iter().enumerate() {
            let mut terms = above[joint.parent].clone();
            if let Some(axis) = joint.leader {
                terms.push(Term {
                    joint: index,
                    axis,
                    multiplier: joint.multiplier,
                });
            }
            above[joint.child] = terms;
        }
        Self {
            links,
            root,
            joints,
            axes,
            above,
        }
    }

    #[must_use]
    pub fn links(&self) -> &[String] {
        &self.links
    }

    #[must_use]
    pub fn link(&self, name: &str) -> Option<usize> {
        self.links.iter().position(|l| l == name)
    }

    /// The independent joints, in [`Tree::joints`] order: the order of every position vector.
    #[must_use]
    pub fn axes(&self) -> &[Axis] {
        &self.axes
    }

    pub(crate) fn above(&self, link: usize) -> &[Term] {
        &self.above[link]
    }

    /// Every link's pose at `q` (one position per [`Model::axes`]).
    #[must_use]
    pub fn poses(&self, q: &[f64]) -> Frames {
        let mut frames = Frames {
            links: vec![Iso::IDENTITY; self.links.len()],
            axes: vec![[0.0; 3]; self.joints.len()],
        };
        self.poses_into(q, &mut frames);
        frames
    }

    /// [`Model::poses`] into frames made by it, without allocating.
    pub fn poses_into(&self, q: &[f64], frames: &mut Frames) {
        let Frames { links, axes } = frames;
        links[self.root] = Iso::IDENTITY;
        for (index, joint) in self.joints.iter().enumerate() {
            let local = match joint.leader {
                None => joint.origin,
                Some(axis) => {
                    let x = joint.multiplier.mul_add(q[axis], joint.offset);
                    let motion = if joint.kind == JointType::Prismatic {
                        Iso {
                            r: crate::math::I3,
                            t: scale(joint.axis, x),
                        }
                    } else {
                        Iso {
                            r: about(joint.axis, x),
                            t: [0.0; 3],
                        }
                    };
                    joint.origin.then(&motion)
                }
            };
            let pose = links[joint.parent].then(&local);
            axes[index] = mat_vec(&pose.r, joint.axis);
            links[joint.child] = pose;
        }
    }

    /// The linear velocity of `point` (root frame), riding `link`, per unit speed of each
    /// axis: `out[axis]`, accumulated into (so mimics and shared axes add).
    pub(crate) fn point_jacobian(&self, frames: &Frames, link: usize, point: V3, out: &mut [V3]) {
        for term in &self.above[link] {
            let joint = &self.joints[term.joint];
            let z = frames.axes[term.joint];
            let v = if joint.kind == JointType::Prismatic {
                z
            } else {
                cross(z, sub(point, frames.links[joint.child].t))
            };
            out[term.axis] = add(out[term.axis], scale(v, term.multiplier));
        }
    }

    /// The angular velocity of `link` per unit speed of each axis, accumulated into `out`.
    pub(crate) fn angular_jacobian(&self, frames: &Frames, link: usize, out: &mut [V3]) {
        for term in &self.above[link] {
            if self.joints[term.joint].kind != JointType::Prismatic {
                out[term.axis] = add(
                    out[term.axis],
                    scale(frames.axes[term.joint], term.multiplier),
                );
            }
        }
    }
}
