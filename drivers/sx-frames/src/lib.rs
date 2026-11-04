//! Frames: one model for every pose (`docs/plans/frames-2026-11.md` in sx). The Rust twin of
//! the Python kernel `sx_geometry.frames`, tested against its golden file
//! (`tests/golden/frames.json`, a byte-identical copy of `packages/sx-geometry/golden/frames.json`).
//!
//! Spatial data is one graph: nodes are **frames**, edges are rigid **transforms**. A
//! convention (FLU, RDF, RUB, Unity's left-handed RUF) is the [`Axes`] of a frame, and two frames
//! on one thing differ by a rotation computed from their axes ([`basis`]), never typed in.
//!
//! - [`Direction`], [`Axes`] (named values [`FLU`] [`RDF`] [`RUB`] [`RFU`] [`LUF`] and the
//!   left-handed [`UNITY`]), [`Handedness`] derived, never stored.
//! - [`Frame`]: a small id from a [`Frames`] registry and right-handed axes;
//!   [`Transform`] `parent_from_child` with checked [`Transform::compose`].
//! - [`recorded`]: the one ingest; a left-handed source is reflected exactly once, there.
//! - Codecs: [`QuaternionLayout`] (xyzw/wxyz), [`EulerSequence`] ([`URDF_RPY`],
//!   [`MUJOCO_EULER`]), rotation vectors ([`rotation_vector_matrix`],
//!   [`matrix_rotation_vector`]) and [`Quaternion::slerp`].
//! - [`Pose`] (quaternion form) and [`Rigid`] (matrix form, what a kinematic chain composes).
//!
//! Every value is `Copy` and fixed-size. Nothing but [`Frames::declare`] allocates, so the
//! control tick composes, rotates and interpolates without touching the heap.

mod axes;
mod frame;
pub mod linalg;
mod pose;
mod rotation;

pub use axes::{Axes, Direction, FLU, Handedness, LUF, NAMED_AXES, RDF, RFU, RUB, UNITY, basis};
pub use frame::{Authority, Frame, FrameId, Frames, LengthUnit, Source, Transform, recorded};
pub use linalg::{Matrix3, Vec3};
pub use pose::{Pose, Rigid};
pub use rotation::{
    Axis, EulerSequence, MUJOCO_EULER, Quaternion, QuaternionLayout, URDF_RPY, about,
    matrix_rotation_vector, rotation_vector_matrix,
};

/// A frame declaration, edge or query that cannot mean anything.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum FrameError {
    #[error("{0}")]
    Axes(String),
    #[error("axes repeat a direction's line; each axis needs its own")]
    AxesRepeatLine,
    #[error("{0} is left-handed; left-handed numbers enter only by recorded()")]
    LeftHanded(Axes),
    #[error("recorded space {space} and local axes {local} differ in handedness")]
    MixedHandedness { space: Axes, local: Axes },
    #[error("frame name {0:?} must be non-empty with no whitespace")]
    Name(String),
    #[error("frame {name} is declared {declared}, not {asked}")]
    Redeclared {
        name: String,
        declared: Axes,
        asked: Axes,
    },
    #[error("too many frames")]
    Full,
    #[error("a transform from frame {0:?} to itself carries nothing")]
    SelfEdge(FrameId),
    #[error(
        "cannot compose an edge ending at {left_child:?} with one starting at {right_parent:?}"
    )]
    Mismatch {
        left_child: FrameId,
        right_parent: FrameId,
    },
    #[error("{0}")]
    Euler(String),
}
