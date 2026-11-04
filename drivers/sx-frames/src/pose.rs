//! The rigid motion an edge carries, in its two working forms: [`Pose`] (translation and a
//! quaternion — what crosses a boundary and what interpolates) and [`Rigid`] (translation and a
//! matrix — what a kinematic chain composes per tick). Both are `Copy` and fixed-size.

use serde::{Deserialize, Serialize};

use crate::linalg::{I3, Matrix3, Vec3, add, mat_mul, mat_vec, transpose};
use crate::rotation::{Quaternion, QuaternionLayout};

/// A rigid motion: `p_parent = rotation · p_child + translation_m`. Which two frames it joins
/// is a [`crate::Transform`]'s business; a bare `Pose` is a value inside one edge.
///
/// As a document it is `{"translation_m": [x, y, z], "rotation_xyzw": [x, y, z, w]}`: the
/// layout is the codec's, named in the key.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(from = "PoseDocument", into = "PoseDocument")]
pub struct Pose {
    pub translation_m: Vec3,
    pub rotation: Quaternion,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PoseDocument {
    translation_m: Vec3,
    rotation_xyzw: [f64; 4],
}

impl From<PoseDocument> for Pose {
    fn from(document: PoseDocument) -> Self {
        Self {
            translation_m: document.translation_m,
            rotation: QuaternionLayout::Xyzw.read(document.rotation_xyzw),
        }
    }
}

impl From<Pose> for PoseDocument {
    fn from(pose: Pose) -> Self {
        Self {
            translation_m: pose.translation_m,
            rotation_xyzw: QuaternionLayout::Xyzw.write(pose.rotation),
        }
    }
}

impl Pose {
    pub const IDENTITY: Self = Self {
        translation_m: [0.0; 3],
        rotation: Quaternion::IDENTITY,
    };

    /// `self ∘ other`: `other` applied in this pose's frame (`self` is the parent).
    #[must_use]
    #[inline]
    pub fn compose(&self, other: &Self) -> Self {
        Self {
            translation_m: add(
                self.translation_m,
                self.rotation.rotate(other.translation_m),
            ),
            rotation: (self.rotation * other.rotation).normalized(),
        }
    }

    /// The inverse motion: `p.compose(&p.inverse())` is the identity.
    #[must_use]
    #[inline]
    pub fn inverse(&self) -> Self {
        let inverse = self.rotation.normalized().conjugate();
        let back = inverse.rotate(self.translation_m);
        Self {
            translation_m: [-back[0], -back[1], -back[2]],
            rotation: inverse,
        }
    }

    /// A point given in the child's coordinates, in the parent's.
    #[must_use]
    #[inline]
    pub fn apply(&self, point: Vec3) -> Vec3 {
        add(self.translation_m, self.rotation.rotate(point))
    }

    /// Whether every number is finite and the quaternion is unit to within `tolerance`.
    #[must_use]
    #[inline]
    pub fn is_rigid(&self, tolerance: f64) -> bool {
        let q = QuaternionLayout::Xyzw.write(self.rotation);
        self.translation_m.iter().chain(&q).all(|v| v.is_finite())
            && (self.rotation.norm_squared().sqrt() - 1.0).abs() <= tolerance
    }

    /// The matrix form.
    #[must_use]
    #[inline]
    pub fn rigid(&self) -> Rigid {
        Rigid {
            r: self.rotation.matrix(),
            t: self.translation_m,
        }
    }
}

/// A rigid motion in matrix form: `p_parent = r · p_child + t`. What a kinematic chain
/// composes once per link per tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rigid {
    pub r: Matrix3,
    pub t: Vec3,
}

impl Rigid {
    pub const IDENTITY: Self = Self { r: I3, t: [0.0; 3] };

    /// From a row-major 4x4 homogeneous matrix (a URDF chain's `Transform`).
    #[must_use]
    #[inline]
    pub const fn from_row_major(m: &[f64; 16]) -> Self {
        Self {
            r: [[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]],
            t: [m[3], m[7], m[11]],
        }
    }

    /// The row-major 4x4 homogeneous matrix.
    #[must_use]
    #[inline]
    pub const fn to_row_major(&self) -> [f64; 16] {
        let (r, t) = (&self.r, &self.t);
        [
            r[0][0], r[0][1], r[0][2], t[0], //
            r[1][0], r[1][1], r[1][2], t[1], //
            r[2][0], r[2][1], r[2][2], t[2], //
            0.0, 0.0, 0.0, 1.0,
        ]
    }

    /// `self` then `other`: `a_from_b.then(b_from_c) = a_from_c`.
    #[must_use]
    #[inline]
    pub fn then(&self, other: &Self) -> Self {
        Self {
            r: mat_mul(&self.r, &other.r),
            t: add(mat_vec(&self.r, other.t), self.t),
        }
    }

    /// The inverse motion.
    #[must_use]
    #[inline]
    pub fn inverse(&self) -> Self {
        let r = transpose(&self.r);
        let back = mat_vec(&r, self.t);
        Self {
            r,
            t: [-back[0], -back[1], -back[2]],
        }
    }

    #[must_use]
    #[inline]
    pub fn apply(&self, p: Vec3) -> Vec3 {
        add(mat_vec(&self.r, p), self.t)
    }

    /// The quaternion form (`w >= 0`).
    #[must_use]
    #[inline]
    pub fn pose(&self) -> Pose {
        Pose {
            translation_m: self.t,
            rotation: Quaternion::from_matrix(&self.r),
        }
    }
}
