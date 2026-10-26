//! Three-vectors, rotations and rigid transforms: just what one tick of the solver needs.

use sx_embodiment_drivers::kinematics::Transform;

pub type V3 = [f64; 3];
pub type M3 = [[f64; 3]; 3];

pub const I3: M3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

#[must_use]
pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[must_use]
pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[must_use]
pub fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

#[must_use]
pub fn dot(a: V3, b: V3) -> f64 {
    a[2].mul_add(b[2], a[0].mul_add(b[0], a[1] * b[1]))
}

#[must_use]
pub fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[must_use]
pub fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

#[must_use]
pub fn mat_vec(m: &M3, v: V3) -> V3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

#[must_use]
pub fn mat_mul(a: &M3, b: &M3) -> M3 {
    let mut out = [[0.0; 3]; 3];
    for (row, out_row) in out.iter_mut().enumerate() {
        for (column, value) in out_row.iter_mut().enumerate() {
            *value = (0..3).map(|k| a[row][k] * b[k][column]).sum();
        }
    }
    out
}

#[must_use]
pub fn transpose(m: &M3) -> M3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

/// Rotation by `angle` about the unit `axis`.
#[must_use]
#[allow(clippy::many_single_char_names)] // the axis-angle formula's own names
pub fn about([x, y, z]: V3, angle: f64) -> M3 {
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
}

/// The rotation vector `w` (axis times angle, angle in `[0, pi]`) with `about(w/|w|, |w|) = r`.
#[must_use]
pub fn log(r: &M3) -> V3 {
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

/// A rotation from a unit quaternion `(x, y, z, w)`; it is normalised first.
#[must_use]
#[allow(clippy::many_single_char_names)]
pub fn from_quaternion([x, y, z, w]: [f64; 4]) -> M3 {
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

/// A rigid transform: `p_parent = r * p_child + t`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Iso {
    pub r: M3,
    pub t: V3,
}

impl Iso {
    pub const IDENTITY: Self = Self { r: I3, t: [0.0; 3] };

    #[must_use]
    pub fn from_transform(m: &Transform) -> Self {
        Self {
            r: [[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]],
            t: [m[3], m[7], m[11]],
        }
    }

    #[must_use]
    pub fn to_transform(&self) -> Transform {
        let (r, t) = (&self.r, &self.t);
        [
            r[0][0], r[0][1], r[0][2], t[0], //
            r[1][0], r[1][1], r[1][2], t[1], //
            r[2][0], r[2][1], r[2][2], t[2], //
            0.0, 0.0, 0.0, 1.0,
        ]
    }

    /// `self` then `other`.
    #[must_use]
    pub fn then(&self, other: &Self) -> Self {
        Self {
            r: mat_mul(&self.r, &other.r),
            t: add(mat_vec(&self.r, other.t), self.t),
        }
    }

    #[must_use]
    pub fn apply(&self, p: V3) -> V3 {
        add(mat_vec(&self.r, p), self.t)
    }
}
