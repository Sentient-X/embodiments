//! One constrained QP differential IK per tick, over an embodiment's own tree.
//!
//! The station runs it on measured joints and is the authority; the headset runs the same
//! crate on the same render pack to draw the instant ghost. Each [`Ik::step`] takes the current
//! positions `q`, one optional [`Target`] per [`Limb`] (a tool pose in the robot's root frame,
//! or a point a camera should look at) and the tick `dt`, and answers the next positions.
//!
//! The step `dq` minimises, in one dense quadratic program solved by this crate's own
//! interior-point method (`qp`, no dependencies, so it builds wherever the station and the
//! headset do):
//!
//! - **tool position** error, in metres, and **tool orientation** error times
//!   [`Settings::orientation_m_per_rad`] (a look-at target's pointing error weighs the same),
//!   each closed per tick no faster than the limb can still stop on it;
//! - **the step itself** ([`Settings::damping`]): damped least squares, the smallest joint
//!   motion that closes the error;
//! - **posture**: each joint toward the anchor, its measured position at engage, weighted so
//!   little ([`Settings::posture_weight`]) that it decides only what the tasks leave free, the
//!   7-DoF arms' null space, and returns it there slowly instead of swinging it.
//!
//! subject to hard constraints, never clamps after the fact:
//!
//! - **joint limits** (with [`Settings::limit_margin_rad`]), with a braking bound so the
//!   acceleration cap can always stop the joint before its limit;
//! - **speed** `|dq| <= speed * dt` and **acceleration** `|dq/dt - v_prev| <= accel * dt` per
//!   joint ([`Caps`]; speed is the description's `<limit velocity>`, acceleration the speed
//!   reached in a ramp the caller names, since no description declares one);
//! - **self-collision** between proxy capsules on links the pack's `collision.json` does not
//!   ignore, and **scene collision** against boxes ([`Obstacle`]: the table), as linearised
//!   distance rows for the [`Settings::max_rows`] closest pairs within
//!   [`Settings::influence_m`]: each pair closes this tick no faster than a share of its axes'
//!   acceleration can still stop it before [`Settings::collision_margin_m`].
//!
//! Where a row cannot be kept within the acceleration caps, the cap yields on that row's axes
//! (keeping clear outranks smoothness) and the step says [`Solve::Relaxed`]; if the QP then
//! fails, each axis slows toward standing still ([`Solve::Held`]).
//!
//! An unreachable or colliding target degrades smoothly: the step goes as far as the
//! constraints allow, and each limb's [`LimbReport`] carries its residual (metres and radians
//! at the new positions), the constraints binding on it and a [`Reach`] the headset tints the
//! ghost by. The headset's ghost runs the same solver with caps high enough that only reach,
//! limits and collision bind.
//!
//! The step is real-time shaped: every buffer and the report are made in [`Ik::new`] and
//! overwritten in place, so a steady-state tick allocates nothing (`tests/allocation.rs`
//! counts); nothing is formatted or logged on the tick; the QP runs at most
//! [`Settings::max_iterations`] iterations and stops at [`Settings::deadline`]. Below the
//! deadline the answer is deterministic.

// A numeric kernel: index loops over matched dense buffers, the formulas' own one-letter
// names, and counts (at most a few hundred) as f64.
#![allow(
    clippy::needless_range_loop,
    clippy::many_single_char_names,
    clippy::cast_precision_loss
)]

mod body;
mod geometry;
mod math;
mod model;
pub mod placement;
mod qp;
mod solve;
pub mod teleop;

use sx_embodiment_drivers::kinematics::TreeError;
use thiserror::Error;

pub use body::{Body, Limb, Target, Task};
pub use geometry::{
    Capsule, Obstacle, PROXIES_SCHEMA, Proxies, box_distance, closest_segments, segment_box,
};
pub use math::Iso;
pub use model::{Frames, Model};
pub use solve::{Active, Caps, Ik, LimbReport, Reach, Settings, Solve, Step};

/// The static upper body's render pack, as committed.
pub const STATIC_UPPER_BODY_URDF: &str =
    include_str!("../../../render_packs/sentient-static-upper-body/robot.urdf");
/// The static upper body's earlier descriptions a dynamics calibration may still be keyed to
/// (`sx_embodiment_drivers::Dynamics::superseding`), as the package keeps each one: a
/// re-render that does not reach a fit (the neck's camera mount, 2026-10-27) needs no re-key.
pub const STATIC_UPPER_BODY_SUPERSEDED: &[&str] = &[include_str!(
    "../../../assets/sentient_static_upper_body/_by_digest/a15896916502f5f9a0a5bdbd0c58d6f943a294f569b48dc83cb355a1b62ebab7/sentient_static_upper_body.urdf"
)];
pub const STATIC_UPPER_BODY_COLLISION: &str =
    include_str!("../../../render_packs/sentient-static-upper-body/collision.json");

/// A body or a step cannot be solved as asked.
#[derive(Debug, Error, PartialEq)]
pub enum IkError {
    #[error("the description is not a tree: {0}")]
    Tree(#[from] TreeError),
    #[error("collision proxies: {0}")]
    Proxies(String),
    #[error("limb {limb:?} names frame {frame:?}, which the tree lacks")]
    UnknownFrame { limb: String, frame: String },
    #[error("joint {0:?} declares no speed limit; give its caps")]
    NoSpeed(String),
    #[error("caps: {0}")]
    Caps(String),
    #[error("{expected} positions are needed, {given} were given")]
    Positions { expected: usize, given: usize },
    #[error("joint {0:?} has a non-finite position")]
    NonFinite(String),
    #[error("{expected} availability flags are needed (one per axis), {given} were given")]
    Available { expected: usize, given: usize },
    #[error("{expected} targets are needed (one per limb), {given} were given")]
    Targets { expected: usize, given: usize },
    #[error("limb {0:?} was given a non-finite target or one of the wrong kind")]
    Target(String),
    #[error("a tick is a positive finite time in seconds, not {0}")]
    Tick(f64),
}
