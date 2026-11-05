//! Frames, the checked edges between them, and the one ingest.
//!
//! A frame is a small id into a [`Frames`] registry plus its axes, so a [`Transform`] is a
//! `Copy` value of fixed size and checking a composition is two integer compares. Names are
//! runtime data (URDF links, customer frames, new bodies) and live in the registry, declared
//! once at setup; nothing on a control tick touches a string or the heap.

use std::fmt;

use crate::FrameError;
use crate::axes::{Axes, Handedness, basis_unchecked};
use crate::linalg::{Matrix3, Vec3, mat_mul, mat_vec, negate, scale, transpose};
use crate::pose::Pose;
use crate::rotation::Quaternion;

/// A frame's index in the [`Frames`] that declared it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FrameId(u32);

impl FrameId {
    #[must_use]
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// A declared frame: its id and its axes. Inside the system every frame is right-handed, so a
/// `Frame` cannot hold left-handed axes. An id means something only within the [`Frames`] that
/// declared it: one robot, episode or delivery keeps one registry, and frames from two are never
/// composed (Python compares names; here the registry makes the name an id).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Frame {
    id: FrameId,
    axes: Axes,
}

impl Frame {
    #[must_use]
    #[inline]
    pub const fn id(self) -> FrameId {
        self.id
    }

    #[must_use]
    #[inline]
    pub const fn axes(self) -> Axes {
        self.axes
    }
}

/// The frames one robot, episode or delivery declares: names and axes, interned once.
#[derive(Clone, Debug, Default)]
pub struct Frames {
    declared: Vec<(String, Axes)>,
}

impl Frames {
    #[must_use]
    #[inline]
    pub const fn new() -> Self {
        Self {
            declared: Vec::new(),
        }
    }

    /// Declare `name` with `axes`, or find it if it is already declared with the same axes.
    ///
    /// # Errors
    ///
    /// [`FrameError::Name`] for an empty name or one with whitespace, [`FrameError::LeftHanded`]
    /// for left-handed axes (a left-handed source is ingested by [`recorded`]), and
    /// [`FrameError::Redeclared`] for a name already declared with other axes.
    pub fn declare(&mut self, name: &str, axes: Axes) -> Result<Frame, FrameError> {
        if name.is_empty() || name.chars().any(char::is_whitespace) {
            return Err(FrameError::Name(name.to_owned()));
        }
        if axes.handedness() == Handedness::Left {
            return Err(FrameError::LeftHanded(axes));
        }
        if let Some(index) = self.declared.iter().position(|(known, _)| known == name) {
            let declared = self.declared[index].1;
            if declared != axes {
                return Err(FrameError::Redeclared {
                    name: name.to_owned(),
                    declared,
                    asked: axes,
                });
            }
            return Ok(Frame {
                id: FrameId(u32::try_from(index).map_err(|_| FrameError::Full)?),
                axes,
            });
        }
        let id = FrameId(u32::try_from(self.declared.len()).map_err(|_| FrameError::Full)?);
        self.declared.push((name.to_owned(), axes));
        Ok(Frame { id, axes })
    }

    /// The frame declared as `name`.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<Frame> {
        self.declared
            .iter()
            .position(|(known, _)| known == name)
            .and_then(|index| {
                u32::try_from(index)
                    .ok()
                    .map(|i| (i, self.declared[index].1))
            })
            .map(|(index, axes)| Frame {
                id: FrameId(index),
                axes,
            })
    }

    /// The name `frame` was declared with.
    #[must_use]
    pub fn name(&self, frame: Frame) -> &str {
        self.declared
            .get(frame.id.index())
            .map_or("?", |(name, _)| name.as_str())
    }
}

/// Where a static edge came from. A correction stays its own edge, applied once.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Authority {
    Design,
    Factory,
    Measured,
    Correction,
    Recorded,
    Computed,
}

impl Authority {
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Design => "design",
            Self::Factory => "factory",
            Self::Measured => "measured",
            Self::Correction => "correction",
            Self::Recorded => "recorded",
            Self::Computed => "computed",
        }
    }
}

/// `parent_from_child`: where `child` is, seen from `parent`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    parent: Frame,
    child: Frame,
    pose: Pose,
    authority: Authority,
}

impl Transform {
    /// # Errors
    ///
    /// [`FrameError::SelfEdge`] for an edge from a frame to itself.
    #[inline]
    pub fn new(
        parent: Frame,
        child: Frame,
        pose: Pose,
        authority: Authority,
    ) -> Result<Self, FrameError> {
        if parent.id == child.id {
            return Err(FrameError::SelfEdge(parent.id));
        }
        Ok(Self {
            parent,
            child,
            pose,
            authority,
        })
    }

    /// The computed rotation between two frames on one thing at one origin.
    ///
    /// # Errors
    ///
    /// [`FrameError::SelfEdge`] when both are one frame.
    #[inline]
    pub fn between(parent: Frame, child: Frame) -> Result<Self, FrameError> {
        let rotation = Quaternion::from_matrix(&basis_unchecked(parent.axes, child.axes));
        Self::new(
            parent,
            child,
            Pose {
                translation_m: [0.0; 3],
                rotation,
            },
            Authority::Computed,
        )
    }

    #[must_use]
    #[inline]
    pub const fn parent(&self) -> Frame {
        self.parent
    }

    #[must_use]
    #[inline]
    pub const fn child(&self) -> Frame {
        self.child
    }

    #[must_use]
    #[inline]
    pub const fn pose(&self) -> Pose {
        self.pose
    }

    #[must_use]
    #[inline]
    pub const fn authority(&self) -> Authority {
        self.authority
    }

    /// `self ∘ other`: `a_from_b ∘ b_from_c = a_from_c`.
    ///
    /// # Errors
    ///
    /// [`FrameError::Mismatch`] when `self`'s child is not `other`'s parent.
    #[inline]
    pub fn compose(&self, other: &Self) -> Result<Self, FrameError> {
        if self.child != other.parent {
            return Err(FrameError::Mismatch {
                left_child: self.child.id,
                right_parent: other.parent.id,
            });
        }
        Self::new(
            self.parent,
            other.child,
            self.pose.compose(&other.pose),
            Authority::Computed,
        )
    }

    /// The reversed edge.
    #[must_use]
    #[inline]
    pub fn inverse(&self) -> Self {
        Self {
            parent: self.child,
            child: self.parent,
            pose: self.pose.inverse(),
            authority: self.authority,
        }
    }

    /// A point given in `child` coordinates, in `parent` coordinates.
    #[must_use]
    #[inline]
    pub fn apply(&self, point: Vec3) -> Vec3 {
        self.pose.apply(point)
    }

    /// This edge between sibling frames of its ends: the convention change, computed. Keeps
    /// its authority.
    ///
    /// # Errors
    ///
    /// [`FrameError::SelfEdge`] when a sibling is the other end.
    pub fn expressed(
        &self,
        parent: Option<Frame>,
        child: Option<Frame>,
    ) -> Result<Self, FrameError> {
        let mut result = *self;
        if let Some(parent) = parent.filter(|p| *p != self.parent) {
            result = Self::between(parent, self.parent)?.compose(&result)?;
        }
        if let Some(child) = child.filter(|c| *c != self.child) {
            result = result.compose(&Self::between(self.child, child)?)?;
        }
        result.authority = self.authority;
        Ok(result)
    }
}

impl fmt::Display for Frame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "#{}[{}]", self.id.0, self.axes)
    }
}

/// A length unit a source writes positions in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LengthUnit {
    Metre,
    Millimetre,
    Centimetre,
}

impl LengthUnit {
    #[must_use]
    #[inline]
    pub const fn metres(self) -> f64 {
        match self {
            Self::Metre => 1.0,
            Self::Millimetre => 1e-3,
            Self::Centimetre => 1e-2,
        }
    }
}

/// How a source writes its numbers: the axes positions and orientations are in (`space`),
/// the axes of the device's own frame in that source (`local`, usually `space`), the length
/// unit and the authority the edge carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Source {
    pub space: Axes,
    pub local: Axes,
    pub unit: LengthUnit,
    pub authority: Authority,
}

impl Source {
    /// A source writing metres in `space`, its devices' local axes the same.
    #[must_use]
    #[inline]
    pub const fn new(space: Axes) -> Self {
        Self {
            space,
            local: space,
            unit: LengthUnit::Metre,
            authority: Authority::Recorded,
        }
    }

    /// `(A, B)` with `parent_from_child = A · recorded · B`, both proper rotations, and the
    /// sign positions take.
    fn proper_pair(
        self,
        parent: Frame,
        child: Frame,
    ) -> Result<(Matrix3, Matrix3, f64), FrameError> {
        let (space, local) = (self.space, self.local);
        if space.handedness() != local.handedness() {
            return Err(FrameError::MixedHandedness { space, local });
        }
        let a = mat_mul(
            &transpose(&parent.axes.facing_from_axes()),
            &space.facing_from_axes(),
        );
        let b = mat_mul(
            &transpose(&local.facing_from_axes()),
            &child.axes.facing_from_axes(),
        );
        Ok(if space.handedness() == Handedness::Left {
            // Both outer factors are reflections; their negations are the proper pair.
            (negate(&a), negate(&b), -1.0)
        } else {
            (a, b, 1.0)
        })
    }
}

/// The one ingest: a pose recorded in a source's axes, as an edge between declared frames.
///
/// `rotation` is the recorded orientation as a matrix in the source's own coordinates (read a
/// quaternion through [`crate::QuaternionLayout`] and take [`Quaternion::matrix`]); a
/// left-handed source (Unity's `RUF`) is reflected exactly once, here.
///
/// # Errors
///
/// [`FrameError::MixedHandedness`] when `space` and `local` differ in handedness;
/// [`FrameError::SelfEdge`] when both frames are one.
pub fn recorded(
    parent: Frame,
    child: Frame,
    translation: Vec3,
    rotation: &Matrix3,
    source: Source,
) -> Result<Transform, FrameError> {
    let (a, b, sign) = source.proper_pair(parent, child)?;
    let position = scale(translation, source.unit.metres());
    let moved = scale(mat_vec(&a, position), sign);
    let turned = mat_mul(&mat_mul(&a, rotation), &b);
    Transform::new(
        parent,
        child,
        Pose {
            translation_m: moved,
            rotation: Quaternion::from_matrix(&turned),
        },
        source.authority,
    )
}

/// The numbers a source writes for an edge: [`recorded`]'s inverse, for a consumer that takes
/// poses in a source's own axes (a Unity scene the headset draws). Returns the translation in
/// the source's unit and the orientation as a matrix in the source's coordinates, so
/// `recorded(edge.parent(), edge.child(), t, &r, source)` is `edge` again; a left-handed source
/// is reflected exactly once, here as there.
///
/// # Errors
///
/// [`FrameError::MixedHandedness`] when `space` and `local` differ in handedness.
pub fn unrecorded(edge: &Transform, source: Source) -> Result<(Vec3, Matrix3), FrameError> {
    let (a, b, sign) = source.proper_pair(edge.parent, edge.child)?;
    let a_inverse = transpose(&a);
    let position = mat_vec(&a_inverse, scale(edge.pose.translation_m, sign));
    let rotation = mat_mul(
        &mat_mul(&a_inverse, &edge.pose.rotation.matrix()),
        &transpose(&b),
    );
    Ok((scale(position, 1.0 / source.unit.metres()), rotation))
}
