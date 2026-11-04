//! Rotations: the quaternion, its layouts at a boundary, Euler sequences, rotation vectors and
//! interpolation. Every function is a fixed-size `Copy` computation: nothing allocates.

use crate::FrameError;
use crate::linalg::{I3, Matrix3, Vec3, dot, mat_mul, norm, scale};

/// A quaternion with named parts, so its order can never be misread. A unit quaternion is a
/// rotation; [`Quaternion::matrix`] normalises first, as the Python kernel's `quat_to_mat` does.
///
/// Numbers enter and leave through [`QuaternionLayout`]; nothing indexes `[3, 0, 1, 2]` by
/// hand.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Quaternion {
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    /// The squared norm.
    #[must_use]
    #[inline]
    pub fn norm_squared(self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    }

    /// Scaled to unit norm; the identity for a (numerically) zero quaternion.
    #[must_use]
    #[inline]
    pub fn normalized(self) -> Self {
        let n = self.norm_squared().sqrt();
        if n < 1e-12 {
            return Self::IDENTITY;
        }
        Self {
            x: self.x / n,
            y: self.y / n,
            z: self.z / n,
            w: self.w / n,
        }
    }

    /// The conjugate: the inverse of a unit quaternion.
    #[must_use]
    #[inline]
    pub const fn conjugate(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    /// `v` rotated by this unit quaternion: `v + 2w (u × v) + 2 u × (u × v)`, `u` the vector
    /// part.
    #[must_use]
    #[inline]
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let cross = |a: Vec3, b: Vec3| {
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        };
        let u = [self.x, self.y, self.z];
        let uv = cross(u, v);
        let uuv = cross(u, uv);
        [0, 1, 2].map(|k| 2.0f64.mul_add(self.w.mul_add(uv[k], uuv[k]), v[k]))
    }

    /// The rotation matrix; the quaternion is normalised first.
    #[must_use]
    #[allow(clippy::many_single_char_names)] // the quaternion's own names
    #[inline]
    pub fn matrix(self) -> Matrix3 {
        let Self { x, y, z, w } = self;
        let n = (x * x + y * y + z * z + w * w).sqrt();
        let (x, y, z, w) = (x / n, y / n, z / n, w / n);
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    }

    /// The unit quaternion of a rotation matrix (Shepperd's method, branching on the largest
    /// diagonal term), with `w >= 0`.
    #[must_use]
    #[allow(clippy::many_single_char_names)]
    #[inline]
    pub fn from_matrix(m: &Matrix3) -> Self {
        let at = |row: usize, column: usize| m[row][column];
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
        Self { x, y, z, w }
    }

    /// The one sign the Python kernel's `Pose` keeps: the first non-zero of `(w, x, y, z)` is
    /// positive.
    #[must_use]
    #[inline]
    pub fn canonical(self) -> Self {
        let first = [self.w, self.x, self.y, self.z]
            .into_iter()
            .find(|value| value.abs() > 1e-12)
            .unwrap_or(1.0);
        if first < 0.0 {
            Self {
                x: -self.x,
                y: -self.y,
                z: -self.z,
                w: -self.w,
            }
        } else {
            self
        }
    }

    /// The geodesic angle in radians between two orientations, in `[0, π]`.
    #[must_use]
    #[inline]
    pub fn angle_to(self, other: Self) -> f64 {
        let (a, b) = (self.normalized(), other.normalized());
        let d = (a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w).abs();
        2.0 * d.min(1.0).acos()
    }

    /// Shortest-path spherical interpolation from `self` (`u = 0`) to `other` (`u = 1`).
    #[must_use]
    #[allow(clippy::many_single_char_names)] // the formula's own names
    #[inline]
    pub fn slerp(self, other: Self, u: f64) -> Self {
        let a = self.normalized();
        let mut b = other.normalized();
        let mut d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
        if d < 0.0 {
            b = Self {
                x: -b.x,
                y: -b.y,
                z: -b.z,
                w: -b.w,
            };
            d = -d;
        }
        let blend = |wa: f64, wb: f64| Self {
            x: wa * a.x + wb * b.x,
            y: wa * a.y + wb * b.y,
            z: wa * a.z + wb * b.z,
            w: wa * a.w + wb * b.w,
        };
        if d > 1.0 - 1e-12 {
            return blend(1.0 - u, u).normalized();
        }
        let theta = d.acos();
        let s = theta.sin();
        blend(((1.0 - u) * theta).sin() / s, (u * theta).sin() / s)
    }
}

/// The Hamilton product `self ⊗ other`: apply `other`, then `self`.
impl std::ops::Mul for Quaternion {
    type Output = Self;

    #[inline]
    fn mul(self, other: Self) -> Self {
        let (ax, ay, az, aw) = (self.x, self.y, self.z, self.w);
        let (bx, by, bz, bw) = (other.x, other.y, other.z, other.w);
        Self {
            x: aw * bx + ax * bw + ay * bz - az * by,
            y: aw * by - ax * bz + ay * bw + az * bx,
            z: aw * bz + ax * by - ay * bx + az * bw,
            w: aw * bw - ax * bx - ay * by - az * bz,
        }
    }
}

/// How four numbers spell a quaternion at one boundary (a file, a tensor, a wire field).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum QuaternionLayout {
    Xyzw,
    Wxyz,
}

impl QuaternionLayout {
    /// These numbers as a quaternion; nothing is normalised.
    #[must_use]
    #[inline]
    pub const fn read(self, [a, b, c, d]: [f64; 4]) -> Quaternion {
        match self {
            Self::Xyzw => Quaternion {
                x: a,
                y: b,
                z: c,
                w: d,
            },
            Self::Wxyz => Quaternion {
                w: a,
                x: b,
                y: c,
                z: d,
            },
        }
    }

    /// The quaternion in this layout.
    #[must_use]
    #[inline]
    pub const fn write(self, q: Quaternion) -> [f64; 4] {
        match self {
            Self::Xyzw => [q.x, q.y, q.z, q.w],
            Self::Wxyz => [q.w, q.x, q.y, q.z],
        }
    }

    #[must_use]
    #[inline]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Xyzw => "xyzw",
            Self::Wxyz => "wxyz",
        }
    }
}

/// Rotation by `angle` (radians) about the unit `axis`.
#[must_use]
#[allow(clippy::many_single_char_names)] // the axis-angle formula's own names
#[inline]
pub fn about([x, y, z]: Vec3, angle: f64) -> Matrix3 {
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
}

/// A rotation vector (axis times angle, radians) as a matrix.
#[must_use]
#[inline]
pub fn rotation_vector_matrix(v: Vec3) -> Matrix3 {
    let angle = norm(v);
    if angle < 1e-15 {
        return I3;
    }
    about(scale(v, 1.0 / angle), angle)
}

/// The rotation vector `w` (axis times angle, angle in `[0, π]`) with
/// `rotation_vector_matrix(w) = r`.
#[must_use]
#[inline]
pub fn matrix_rotation_vector(r: &Matrix3) -> Vec3 {
    let cos = ((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0).clamp(-1.0, 1.0);
    let angle = cos.acos();
    let skew = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]];
    if angle < 1e-6 {
        return scale(skew, 0.5);
    }
    if angle > std::f64::consts::PI - 1e-4 {
        // Near a half turn the skew part vanishes: read the axis from the symmetric part.
        let diagonal = [r[0][0], r[1][1], r[2][2]];
        let k = (0..3)
            .max_by(|&a, &b| diagonal[a].total_cmp(&diagonal[b]))
            .unwrap_or(0);
        let mut axis = [0.0; 3];
        axis[k] = ((diagonal[k] - cos) / (1.0 - cos)).max(0.0).sqrt();
        for j in (0..3).filter(|&j| j != k) {
            axis[j] = (r[k][j] + r[j][k]) / (2.0 * (1.0 - cos) * axis[k]);
        }
        let length = norm(axis);
        let sign = if dot(axis, skew) < 0.0 { -1.0 } else { 1.0 };
        return scale(axis, sign * angle / length);
    }
    scale(skew, angle / (2.0 * angle.sin()))
}

/// One coordinate axis of an Euler sequence.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    const fn index(self) -> usize {
        match self {
            Self::X => 0,
            Self::Y => 1,
            Self::Z => 2,
        }
    }

    fn elementary(self, angle: f64) -> Matrix3 {
        let (s, c) = angle.sin_cos();
        match self {
            Self::X => [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]],
            Self::Y => [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
            Self::Z => [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        }
    }
}

/// Three angles about named axes, and whether each turn is about the moving axis.
///
/// URDF `rpy` is [`URDF_RPY`] (extrinsic xyz, `R = Rz·Ry·Rx`); `MuJoCo`'s default `euler` is
/// [`MUJOCO_EULER`] (intrinsic xyz, `R = Rx·Ry·Rz`). They are different rotations for the same
/// numbers whenever more than one angle is non-zero.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct EulerSequence {
    axes: [Axis; 3],
    intrinsic: bool,
}

/// URDF `rpy`: extrinsic x, y, z.
pub const URDF_RPY: EulerSequence = EulerSequence {
    axes: [Axis::X, Axis::Y, Axis::Z],
    intrinsic: false,
};
/// `MuJoCo`'s default `euler`: intrinsic x, y, z.
pub const MUJOCO_EULER: EulerSequence = EulerSequence {
    axes: [Axis::X, Axis::Y, Axis::Z],
    intrinsic: true,
};

impl EulerSequence {
    /// A sequence spelled like `"xyz"`.
    ///
    /// # Errors
    ///
    /// [`FrameError::Euler`] for anything but three of x, y, z with no axis twice in a row.
    #[inline]
    pub fn new(axes: &str, intrinsic: bool) -> Result<Self, FrameError> {
        let refuse = || FrameError::Euler(format!("Euler axes {axes:?} must be three of x, y, z"));
        let mut parsed = [Axis::X; 3];
        let mut letters = axes.chars();
        for slot in &mut parsed {
            *slot = match letters.next() {
                Some('x') => Axis::X,
                Some('y') => Axis::Y,
                Some('z') => Axis::Z,
                _ => return Err(refuse()),
            };
        }
        if letters.next().is_some() {
            return Err(refuse());
        }
        if parsed[0] == parsed[1] || parsed[1] == parsed[2] {
            return Err(FrameError::Euler(format!(
                "Euler axes {axes:?} repeat an axis in a row"
            )));
        }
        Ok(Self {
            axes: parsed,
            intrinsic,
        })
    }

    #[must_use]
    #[inline]
    pub const fn axes(self) -> [Axis; 3] {
        self.axes
    }

    #[must_use]
    #[inline]
    pub const fn intrinsic(self) -> bool {
        self.intrinsic
    }

    /// The rotation these angles (radians) spell.
    #[must_use]
    #[inline]
    pub fn matrix(self, angles: [f64; 3]) -> Matrix3 {
        let [a, b, c] = [0, 1, 2].map(|k| self.axes[k].elementary(angles[k]));
        if self.intrinsic {
            mat_mul(&mat_mul(&a, &b), &c)
        } else {
            mat_mul(&mat_mul(&c, &b), &a)
        }
    }

    /// The inverse of [`EulerSequence::matrix`] (one branch; a gimbal lock folds into the last
    /// angle).
    ///
    /// # Errors
    ///
    /// [`FrameError::Euler`] for a proper Euler sequence (x-y-x and its kin), which nothing in
    /// the tree uses.
    #[allow(clippy::many_single_char_names)] // the Tait-Bryan solver's own names
    #[inline]
    pub fn angles(self, rotation: &Matrix3) -> Result<[f64; 3], FrameError> {
        if !self.intrinsic {
            let [x, y, z] = self.axes;
            let reverse = Self {
                axes: [z, y, x],
                intrinsic: true,
            };
            let [c, b, a] = reverse.angles(rotation)?;
            return Ok([a, b, c]);
        }
        let [i, j, k] = self.axes.map(Axis::index);
        if i == k {
            return Err(FrameError::Euler(
                "proper Euler sequences are not supported".into(),
            ));
        }
        let parity = if (j + 3 - i) % 3 == 1 { 1.0 } else { -1.0 };
        // R = R_i(a) R_j(b) R_k(c): R[i][k] = parity · sin(b).
        let sb = (parity * rotation[i][k]).clamp(-1.0, 1.0);
        let b = sb.asin();
        if sb.abs() < 1.0 - 1e-9 {
            let a = (-parity * rotation[j][k]).atan2(rotation[k][k]);
            let c = (-parity * rotation[i][j]).atan2(rotation[i][i]);
            Ok([a, b, c])
        } else {
            let c = (parity * rotation[j][i]).atan2(rotation[j][j]);
            Ok([0.0, b, c])
        }
    }
}
