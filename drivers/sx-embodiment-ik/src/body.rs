//! A body ready to solve: its tree, collision proxies and the limbs targets are given for.

use sx_embodiment_drivers::kinematics::Tree;

use sx_frames::linalg::Vec3;
use sx_frames::{Pose, Rigid};

use crate::geometry::{Capsule, Proxies};
use crate::model::{Frames, Model};
use crate::tool::{ToolFrame, parse_tools};
use crate::{IkError, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_PACK, STATIC_UPPER_BODY_URDF};

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
    /// Reach a whole pose: the tool frame in the root frame.
    Pose(Pose),
    LookAt {
        point_m: [f64; 3],
    },
    /// Reach a point with the tool's `axis` (in the limb's frame, its approach axis) along
    /// `toward` (in the root frame); the roll about it floats. A grasp or a press whose roll
    /// does not matter, so the arm never spends its wrist turning to a roll nobody needs.
    Axis {
        translation_m: [f64; 3],
        axis: [f64; 3],
        toward: [f64; 3],
    },
    /// No target: come to rest. Each of the limb's joints, a shared one (a torso) included,
    /// brakes from its commanded motion to standing still within its acceleration cap and its
    /// follower's jerk (`sx_embodiment_drivers::jerk`), as the rows allow: an operator letting
    /// go mid-sweep. Brought to rest along the tool's line instead,
    /// a sweep that held joints at their speed caps (0.5 m/s on the static upper body's
    /// governed caps) left them with a kink at 6x a shoulder's jerk cap, and a stop short
    /// enough to bound the coast raised its bounds past the caps.
    Rest,
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
    /// Its tools, as its pack carries them ([`Body::with_tools`]).
    tools: Vec<ToolFrame>,
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
            tools: Vec::new(),
        })
    }

    /// The body with its tools, from its render pack's `pack.json`.
    ///
    /// # Errors
    ///
    /// [`IkError::Tools`] when the pack carries no readable tools, or one names a frame the
    /// tree lacks.
    pub fn with_tools(mut self, pack_json: &str) -> Result<Self, IkError> {
        let tools = parse_tools(pack_json)?;
        if let Some(tool) = tools.iter().find(|t| self.model.link(&t.frame).is_none()) {
            return Err(IkError::Tools(format!(
                "{} is stated in {:?}, which the tree lacks",
                tool.component, tool.frame
            )));
        }
        self.tools = tools;
        Ok(self)
    }

    /// Every tool the pack carries, each in its own frame.
    #[must_use]
    pub fn tools(&self) -> &[ToolFrame] {
        &self.tools
    }

    /// The tool whose frame link `frame` hangs rigidly below (a gripper's tool point on its
    /// hand), its grip stated in `frame`. `None` when no tool does, or `frame` is not a link.
    #[must_use]
    pub fn tool(&self, frame: &str) -> Option<ToolFrame> {
        let at = self.model.link(frame)?;
        let (tool, own) = self.tools.iter().find_map(|tool| {
            let own = self.model.link(&tool.frame)?;
            self.model.rigidly_below(own, at).then_some((tool, own))
        })?;
        let q: Vec<f64> = self
            .model
            .axes()
            .iter()
            .map(|axis| axis.limits.map_or(0.0, |[lo, hi]| 0.0_f64.clamp(lo, hi)))
            .collect();
        let frames = self.model.poses(&q);
        let (mine, theirs) = (frames.links[own], frames.links[at]);
        Some(tool.restated(frame, (&mine.r, mine.t), (&theirs.r, theirs.t)))
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
            .and_then(|body| body.with_tools(STATIC_UPPER_BODY_PACK))
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

    /// The axes that move limb `index`'s frame (indices into [`Model::axes`], ascending): an
    /// axis in more than one limb's list (a torso) is shared.
    #[must_use]
    pub fn limb_axes(&self, index: usize) -> &[usize] {
        &self.limbs[index].axes
    }

    /// The axes the solver moves (indices into [`Model::axes`]); every other axis is held.
    #[must_use]
    pub fn solved(&self) -> &[usize] {
        &self.solved
    }

    /// The frame of limb `index` at `q`.
    #[must_use]
    pub fn frame(&self, frames: &Frames, index: usize) -> Rigid {
        frames.links[self.limbs[index].frame]
    }

    /// Limb `index`'s Jacobian at `frames`: per axis ([`Model::axes`] order), the linear
    /// velocity of its frame's origin and its angular velocity, both in the root frame, per
    /// unit speed of that axis (zero for an axis that does not move it). The solver's own
    /// rows, for evaluation: how much of a joint motion moves the tool, and how much is the
    /// limb's null space.
    ///
    /// # Panics
    ///
    /// When `linear` or `angular` is shorter than [`Model::axes`].
    pub fn jacobian(
        &self,
        frames: &Frames,
        index: usize,
        linear: &mut [Vec3],
        angular: &mut [Vec3],
    ) {
        let width = self.model.axes().len();
        linear[..width].iter_mut().for_each(|v| *v = [0.0; 3]);
        angular[..width].iter_mut().for_each(|v| *v = [0.0; 3]);
        let frame = self.limbs[index].frame;
        self.model
            .point_jacobian(frames, frame, frames.links[frame].t, linear);
        self.model.angular_jacobian(frames, frame, angular);
    }

    /// How fast limb `index`'s frame can move along unit `direction` (root frame) at `frames`,
    /// m/s, with each axis at most `speed[axis]` rad/s (indices into [`Model::axes`]; 0 for an
    /// axis held still): the tool speed the joints deliver at this pose, not a constant.
    ///
    /// The joints move as the solve's damped least-squares answer `d = J' (J J' + damping
    /// I)^-1 u` for the frame's origin's rows `J`, so the frame moves along `u` at
    /// `min_i speed_i / |d_i|`. Near a singularity the weak direction is slow, and so is this.
    #[must_use]
    pub fn tool_speed_along(
        &self,
        frames: &Frames,
        index: usize,
        speed: &[f64],
        direction: Vec3,
    ) -> f64 {
        let rows = self.response(frames, index, speed, false);
        along(&rows, speed, direction)
    }

    /// How fast limb `index`'s frame can move (m/s) and turn (rad/s) at `frames` in its fastest
    /// direction, each axis at most `speed[axis]` rad/s ([`Body::tool_speed_along`] over the 26
    /// directions of a cube's faces, edges and corners). A program's tool asked to go faster
    /// than this is asked for what no joint can give; slower directions are held by the joints'
    /// own caps in the solve.
    #[must_use]
    pub fn tool_capability(&self, frames: &Frames, index: usize, speed: &[f64]) -> ToolCapability {
        let fastest = |angular: bool| {
            let rows = self.response(frames, index, speed, angular);
            let mut best = 0.0_f64;
            for x in -1..=1_i32 {
                for y in -1..=1_i32 {
                    for z in -1..=1_i32 {
                        if x == 0 && y == 0 && z == 0 {
                            continue;
                        }
                        let u = [f64::from(x), f64::from(y), f64::from(z)];
                        let length = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt();
                        best = best.max(along(&rows, speed, u.map(|v| v / length)));
                    }
                }
            }
            if best.is_finite() { best } else { 0.0 }
        };
        ToolCapability {
            linear_m_s: fastest(false),
            angular_rad_s: fastest(true),
        }
    }

    /// Per axis that moves limb `index` and may (`speed` above 0): the row `b_i` of
    /// `J' (J J' + damping I)^-1` for its linear (or `angular`) rows `J`.
    fn response(
        &self,
        frames: &Frames,
        index: usize,
        speed: &[f64],
        angular: bool,
    ) -> Vec<(usize, Vec3)> {
        let limb = &self.limbs[index];
        let mut j = vec![[0.0; 3]; self.model.axes().len()];
        if angular {
            self.model.angular_jacobian(frames, limb.frame, &mut j);
        } else {
            let origin = frames.links[limb.frame].t;
            self.model
                .point_jacobian(frames, limb.frame, origin, &mut j);
        }
        let moving: Vec<usize> = limb
            .axes
            .iter()
            .copied()
            .filter(|&axis| speed.get(axis).is_some_and(|s| *s > 0.0))
            .collect();
        let mut m = [[0.0; 3]; 3];
        for (a, line) in m.iter_mut().enumerate() {
            for (b, cell) in line.iter_mut().enumerate() {
                *cell = moving.iter().map(|&axis| j[axis][a] * j[axis][b]).sum();
            }
            line[a] += CAPABILITY_DAMPING;
        }
        let Some(inverse) = inverse3(&m) else {
            return Vec::new();
        };
        moving
            .iter()
            .map(|&axis| {
                let b = [0, 1, 2].map(|k| (0..3).map(|a| j[axis][a] * inverse[a][k]).sum::<f64>());
                (axis, b)
            })
            .collect()
    }
}

/// The speed along unit `u` at which the first axis of `rows` ([`Body::response`]) reaches its
/// cap; 0 where nothing moves.
fn along(rows: &[(usize, Vec3)], speed: &[f64], u: Vec3) -> f64 {
    let most = rows
        .iter()
        .map(|(axis, b)| (b[0] * u[0] + b[1] * u[1] + b[2] * u[2]).abs() / speed[*axis])
        .fold(0.0, f64::max);
    if most > 1e-12 { 1.0 / most } else { 0.0 }
}

/// The damped least-squares damping [`Body::tool_speed_along`] answers with: the solve's own
/// (`Settings::damping`).
const CAPABILITY_DAMPING: f64 = 1e-4;

/// How fast a limb's frame can move and turn in its fastest direction ([`Body::tool_capability`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToolCapability {
    pub linear_m_s: f64,
    pub angular_rad_s: f64,
}

fn inverse3(m: &[[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let c =
        |r0: usize, c0: usize, r1: usize, c1: usize| m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
    let det = m[0][0] * c(1, 1, 2, 2) - m[0][1] * c(1, 0, 2, 2) + m[0][2] * c(1, 0, 2, 1);
    if det.abs() < 1e-18 {
        return None;
    }
    let inv = 1.0 / det;
    Some([
        [
            c(1, 1, 2, 2) * inv,
            -c(0, 1, 2, 2) * inv,
            c(0, 1, 1, 2) * inv,
        ],
        [
            -c(1, 0, 2, 2) * inv,
            c(0, 0, 2, 2) * inv,
            -c(0, 0, 1, 2) * inv,
        ],
        [
            c(1, 0, 2, 1) * inv,
            -c(0, 0, 2, 1) * inv,
            c(0, 0, 1, 1) * inv,
        ],
    ])
}

pub(crate) fn world(frames: &Frames, capsule: &Capsule) -> (Vec3, Vec3) {
    let pose = &frames.links[capsule.link];
    (pose.apply(capsule.a), pose.apply(capsule.b))
}
