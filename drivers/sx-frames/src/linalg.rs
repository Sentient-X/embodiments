//! Three-vectors and 3x3 matrices (row-major): just what one control tick needs, on the stack.
//!
//! These are the solver's own arithmetic, moved here unchanged from `sx-embodiment-ik`'s
//! `math.rs` so every crate turns vectors the same way, to the bit.

/// A vector in metres or a direction; which frame it is in is the caller's edge.
pub type Vec3 = [f64; 3];
/// A row-major 3x3 matrix: `m[row][column]`.
pub type Matrix3 = [[f64; 3]; 3];

/// The identity matrix.
pub const I3: Matrix3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

#[must_use]
#[inline]
pub fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[must_use]
#[inline]
pub fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[must_use]
#[inline]
pub fn scale(a: Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

#[must_use]
#[inline]
pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a[2].mul_add(b[2], a[0].mul_add(b[0], a[1] * b[1]))
}

#[must_use]
#[inline]
pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[must_use]
#[inline]
pub fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}

#[must_use]
#[inline]
pub fn mat_vec(m: &Matrix3, v: Vec3) -> Vec3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

#[must_use]
#[inline]
pub fn mat_mul(a: &Matrix3, b: &Matrix3) -> Matrix3 {
    let mut out = [[0.0; 3]; 3];
    for (row, out_row) in out.iter_mut().enumerate() {
        for (column, value) in out_row.iter_mut().enumerate() {
            *value = (0..3).map(|k| a[row][k] * b[k][column]).sum();
        }
    }
    out
}

#[must_use]
#[inline]
pub fn transpose(m: &Matrix3) -> Matrix3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}

/// `-m`: a reflection's proper partner.
#[must_use]
#[inline]
pub fn negate(m: &Matrix3) -> Matrix3 {
    m.map(|row| row.map(|value| -value))
}

/// The determinant.
#[must_use]
#[inline]
pub fn determinant(m: &Matrix3) -> f64 {
    dot(m[0], cross(m[1], m[2]))
}
