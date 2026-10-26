//! A body ready to solve: its tree, collision proxies and the limbs targets are given for.

use sx_embodiment_drivers::kinematics::Tree;

use crate::geometry::{Capsule, Proxies};
use crate::math::{Iso, V3};
use crate::model::{Frames, Model};
use crate::{IkError, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};

/// What a limb's frame is asked to do.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Task {
    /// Reach a pose: a gripper's tool frame.
    Pose,
    /// Point the frame's `forward` axis (unit, in the frame) at a point: a camera.
    LookAt { forward: [f64; 3] },
}

/// One task frame of the body and what it is asked to do.
#[derive(Clone, Debug, PartialEq)]
pub struct Limb {
    pub name: String,
    pub frame: String,
    pub task: Task,
}

/// One tick's ask of a limb, in the root frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Target {
    Pose {
        translation_m: [f64; 3],
        rotation_xyzw: [f64; 4],
    },
    LookAt {
        point_m: [f64; 3],
    },
}

#[derive(Clone, Debug)]
pub(crate) struct LimbFrame {
    pub(crate) name: String,
    pub(crate) frame: usize,
    pub(crate) task: Task,
    /// Axes that move the frame.
    pub(crate) axes: Vec<usize>,
}

/// A body ready to solve: its tree, collision proxies and limbs.
#[derive(Clone, Debug)]
pub struct Body {
    pub(crate) model: Model,
    pub(crate) proxies: Proxies,
    pub(crate) limbs: Vec<LimbFrame>,
    /// The axes solved: every axis that moves a limb's frame, in axis order.
    pub(crate) solved: Vec<usize>,
}

impl Body {
    /// A body from its render pack's `robot.urdf` and `collision.json`.
    ///
    /// # Errors
    ///
    /// [`IkError`] when the description is not a tree, the proxies do not fit it, or a limb's
    /// frame is not a link.
    pub fn new(urdf: &str, collision: &str, limbs: &[Limb]) -> Result<Self, IkError> {
        let model = Model::new(&Tree::parse(urdf)?);
        let proxies = Proxies::parse(collision, &model)?;
        let limbs = limbs
            .iter()
            .map(|limb| {
                let frame = model
                    .link(&limb.frame)
                    .ok_or_else(|| IkError::UnknownFrame {
                        limb: limb.name.clone(),
                        frame: limb.frame.clone(),
                    })?;
                let mut axes: Vec<usize> = model.above(frame).iter().map(|t| t.axis).collect();
                axes.sort_unstable();
                axes.dedup();
                Ok(LimbFrame {
                    name: limb.name.clone(),
                    frame,
                    task: limb.task,
                    axes,
                })
            })
            .collect::<Result<Vec<_>, IkError>>()?;
        let mut solved: Vec<usize> = limbs.iter().flat_map(|l| l.axes.clone()).collect();
        solved.sort_unstable();
        solved.dedup();
        Ok(Self {
            model,
            proxies,
            limbs,
            solved,
        })
    }

    /// The Sentient static upper body: both grippers' tool frames, and the ZED looking out
    /// of its lenses (`ZED_camera` +x) from the SO-101 neck.
    ///
    /// # Panics
    ///
    /// Never: the committed pack is held to parse by this crate's tests.
    #[must_use]
    pub fn static_upper_body() -> Self {
        let limbs = [
            Limb {
                name: "left".into(),
                frame: "left_gripper_tcp".into(),
                task: Task::Pose,
            },
            Limb {
                name: "right".into(),
                frame: "right_gripper_tcp".into(),
                task: Task::Pose,
            },
            Limb {
                name: "head".into(),
                frame: "ZED_camera".into(),
                task: Task::LookAt {
                    forward: [1.0, 0.0, 0.0],
                },
            },
        ];
        Self::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &limbs)
            .expect("the committed static upper body pack solves")
    }

    #[must_use]
    pub fn model(&self) -> &Model {
        &self.model
    }

    #[must_use]
    pub fn proxies(&self) -> &Proxies {
        &self.proxies
    }

    /// The limbs' names, in the order [`crate::Ik::step`] takes targets.
    #[must_use]
    pub fn limbs(&self) -> Vec<&str> {
        self.limbs.iter().map(|l| l.name.as_str()).collect()
    }

    /// The axes the solver moves (indices into [`Model::axes`]); every other axis is held.
    #[must_use]
    pub fn solved(&self) -> &[usize] {
        &self.solved
    }

    /// The frame of limb `index` at `q`.
    #[must_use]
    pub fn frame(&self, frames: &Frames, index: usize) -> Iso {
        frames.links[self.limbs[index].frame]
    }
}

pub(crate) fn world(frames: &Frames, capsule: &Capsule) -> (V3, V3) {
    let pose = &frames.links[capsule.link];
    (pose.apply(capsule.a), pose.apply(capsule.b))
}
