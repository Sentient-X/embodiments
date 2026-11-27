//! The small fixed-size linear algebra the solver needs: 3-vectors, 3x3 matrices, rotation
//! vectors (OpenCV's `Rodrigues`, both ways), the principal axis of a set of 3-vectors, and the
//! numpy reductions the solver was first written in (`interp`, `percentile`, `median`,
//! `corrcoef`, a two-parameter least squares), each with numpy's exact semantics.

pub type Vec3 = [f64; 3];
/// Row-major.
pub type Mat3 = [[f64; 3]; 3];

pub const IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}

pub fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn scale(a: Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub fn neg(a: Vec3) -> Vec3 {
    [-a[0], -a[1], -a[2]]
}

pub fn transpose(m: &Mat3) -> Mat3 {
    let mut t = [[0.0; 3]; 3];
    for (i, row) in m.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            t[j][i] = *value;
        }
    }
    t
}

pub fn matmul(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut out = [[0.0; 3]; 3];
    for (i, row) in out.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
        }
    }
    out
}

pub fn apply(m: &Mat3, v: Vec3) -> Vec3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

/// `a^T b`, the rotation from `a` to `b` in `a`'s frame.
pub fn relative(a: &Mat3, b: &Mat3) -> Mat3 {
    matmul(&transpose(a), b)
}

pub fn column(m: &Mat3, j: usize) -> Vec3 {
    [m[0][j], m[1][j], m[2][j]]
}

/// OpenCV's `Rodrigues` from a rotation matrix to its rotation vector (without its SVD
/// re-orthonormalisation: every matrix here is a product of exact rotations).
pub fn log(r: &Mat3) -> Vec3 {
    let rx = r[2][1] - r[1][2];
    let ry = r[0][2] - r[2][0];
    let rz = r[1][0] - r[0][1];
    let s = ((rx * rx + ry * ry + rz * rz) * 0.25).sqrt();
    let c = ((r[0][0] + r[1][1] + r[2][2] - 1.0) * 0.5).clamp(-1.0, 1.0);
    let theta = c.acos();
    if s < 1e-5 {
        if c > 0.0 {
            return [0.0, 0.0, 0.0];
        }
        let mut out = [
            ((r[0][0] + 1.0) * 0.5).max(0.0).sqrt(),
            ((r[1][1] + 1.0) * 0.5).max(0.0).sqrt() * if r[0][1] < 0.0 { -1.0 } else { 1.0 },
            ((r[2][2] + 1.0) * 0.5).max(0.0).sqrt() * if r[0][2] < 0.0 { -1.0 } else { 1.0 },
        ];
        if out[0].abs() < out[1].abs()
            && out[0].abs() < out[2].abs()
            && (r[1][2] > 0.0) != (out[1] * out[2] > 0.0)
        {
            out[2] = -out[2];
        }
        let length = norm(out);
        return scale(out, std::f64::consts::PI / length);
    }
    scale([rx, ry, rz], theta / (2.0 * s))
}

/// OpenCV's `Rodrigues` from a rotation vector to its matrix.
pub fn exp(v: Vec3) -> Mat3 {
    let theta = norm(v);
    if theta < f64::EPSILON {
        return IDENTITY;
    }
    let (s, c) = theta.sin_cos();
    let c1 = 1.0 - c;
    let [x, y, z] = scale(v, 1.0 / theta);
    [
        [c + c1 * x * x, c1 * x * y - s * z, c1 * x * z + s * y],
        [c1 * x * y + s * z, c + c1 * y * y, c1 * y * z - s * x],
        [c1 * x * z - s * y, c1 * y * z + s * x, c + c1 * z * z],
    ]
}

/// The angle between two rotations (radians).
pub fn geodesic(a: &Mat3, b: &Mat3) -> f64 {
    norm(log(&relative(a, b)))
}

/// The principal axis of `rows`: the first right singular vector of the `n x 3` matrix they
/// form (numpy's `svd(rows)[2][0]`), up to sign, which no caller depends on.
pub fn principal_axis(rows: impl IntoIterator<Item = Vec3>) -> Vec3 {
    let mut gram = [[0.0; 3]; 3];
    for row in rows {
        for i in 0..3 {
            for j in 0..3 {
                gram[i][j] += row[i] * row[j];
            }
        }
    }
    let (values, vectors) = symmetric_eigen(gram);
    let mut best = 0;
    for k in 1..3 {
        if values[k] > values[best] {
            best = k;
        }
    }
    column(&vectors, best)
}

/// Eigenvalues and eigenvectors (columns) of a symmetric 3x3 matrix by cyclic Jacobi.
fn symmetric_eigen(mut a: Mat3) -> (Vec3, Mat3) {
    let mut v = IDENTITY;
    for _ in 0..64 {
        let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
        if off <= 1e-300 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if a[p][q].abs() <= 1e-300 {
                continue;
            }
            let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let akp = a[k][p];
                let akq = a[k][q];
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for k in 0..3 {
                let apk = a[p][k];
                let aqk = a[q][k];
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            for row in &mut v {
                let vkp = row[p];
                let vkq = row[q];
                row[p] = c * vkp - s * vkq;
                row[q] = s * vkp + c * vkq;
            }
        }
    }
    ([a[0][0], a[1][1], a[2][2]], v)
}

pub fn inverse(m: &Mat3) -> Option<Mat3> {
    let c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    let c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    let c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    let det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    if det == 0.0 || !det.is_finite() {
        return None;
    }
    let inv = 1.0 / det;
    Some([
        [
            c00 * inv,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv,
        ],
        [
            c01 * inv,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv,
        ],
        [
            c02 * inv,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv,
        ],
    ])
}

// -- numpy's reductions -------------------------------------------------------------------------

/// numpy's `percentile` (linear), NaN for an empty set.
pub fn percentile(values: &[f64], q: f64) -> f64 {
    if values.is_empty() {
        return f64::NAN;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(f64::total_cmp);
    percentile_sorted(&sorted, q)
}

pub fn percentile_sorted(sorted: &[f64], q: f64) -> f64 {
    let index = q / 100.0 * (sorted.len() - 1) as f64;
    let low = index.floor() as usize;
    let high = (low + 1).min(sorted.len() - 1);
    let fraction = index - low as f64;
    if fraction == 0.0 {
        return sorted[low];
    }
    sorted[low] + (sorted[high] - sorted[low]) * fraction
}

pub fn median(values: &[f64]) -> f64 {
    percentile(values, 50.0)
}

/// numpy's `ptp`.
pub fn ptp(values: &[f64]) -> f64 {
    let (low, high) = values
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), v| {
            (l.min(*v), h.max(*v))
        });
    high - low
}

/// Pearson correlation (numpy's `corrcoef(a, b)[0, 1]`); NaN when either is constant.
pub fn correlation(a: &[f64], b: &[f64]) -> f64 {
    let n = a.len() as f64;
    let ma = a.iter().sum::<f64>() / n;
    let mb = b.iter().sum::<f64>() / n;
    let (mut sab, mut saa, mut sbb) = (0.0, 0.0, 0.0);
    for (x, y) in a.iter().zip(b) {
        sab += (x - ma) * (y - mb);
        saa += (x - ma) * (x - ma);
        sbb += (y - mb) * (y - mb);
    }
    sab / (saa * sbb).sqrt()
}

/// numpy's `searchsorted` (side left) over increasing `stamps`.
pub fn search_sorted(stamps: &[i64], at: i64) -> usize {
    stamps.partition_point(|s| *s < at)
}

/// numpy's `interp(x, xp, fp)`: clamped at both ends; an `x` on a stamp takes the last value
/// stamped there.
pub fn interp(x: f64, xp: &[f64], fp: &[f64]) -> f64 {
    let n = xp.len();
    if x >= xp[n - 1] {
        return fp[n - 1];
    }
    if x < xp[0] {
        return fp[0];
    }
    let j = xp.partition_point(|v| *v <= x) - 1;
    if xp[j] == x {
        return fp[j];
    }
    let slope = (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j]);
    slope * (x - xp[j]) + fp[j]
}

/// numpy's `interp` over integer stamps.
pub fn interp_ns(x: i64, xp: &[i64], fp: &[f64]) -> f64 {
    let n = xp.len();
    if x >= xp[n - 1] {
        return fp[n - 1];
    }
    if x < xp[0] {
        return fp[0];
    }
    let j = xp.partition_point(|v| *v <= x) - 1;
    if xp[j] == x {
        return fp[j];
    }
    let slope = (fp[j + 1] - fp[j]) / (xp[j + 1] - xp[j]) as f64;
    slope * (x - xp[j]) as f64 + fp[j]
}

/// numpy's `lstsq` of `y = slope * x + offset`: the least-squares line, and with every `x`
/// equal the minimum-norm one.
pub fn least_squares_line(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len() as f64;
    let mx = x.iter().sum::<f64>() / n;
    let my = y.iter().sum::<f64>() / n;
    let (mut sxx, mut sxy) = (0.0, 0.0);
    for (a, b) in x.iter().zip(y) {
        sxx += (a - mx) * (a - mx);
        sxy += (a - mx) * (b - my);
    }
    if sxx == 0.0 {
        let factor = my / (mx * mx + 1.0);
        return (factor * mx, factor);
    }
    let slope = sxy / sxx;
    (slope, my - slope * mx)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotation_vectors_round_trip_including_near_half_turns() {
        for v in [
            [0.1, -0.2, 0.3],
            [0.0, 0.0, 1e-9],
            [3.1, 0.05, -0.02],
            [0.0, std::f64::consts::PI, 0.0],
        ] {
            let back = log(&exp(v));
            let error = norm(sub(back, v));
            assert!(error < 1e-6, "{v:?} -> {back:?}");
        }
    }

    #[test]
    fn interp_is_numpys() {
        let xp = [0.0, 1.0, 1.0, 2.0];
        let fp = [0.0, 1.0, 5.0, 6.0];
        assert_eq!(interp(-1.0, &xp, &fp), 0.0);
        assert_eq!(interp(1.0, &xp, &fp), 5.0);
        assert_eq!(interp(1.5, &xp, &fp), 5.5);
        assert_eq!(interp(3.0, &xp, &fp), 6.0);
    }

    #[test]
    fn percentile_is_linear() {
        assert_eq!(percentile(&[1.0, 2.0, 3.0, 4.0], 50.0), 2.5);
        assert!((percentile(&[0.0, 10.0], 95.0) - 9.5).abs() < 1e-12);
    }

    #[test]
    fn the_principal_axis_is_the_dominant_direction() {
        let axis = principal_axis([[1.0, 2.0, 0.0], [2.0, 4.1, 0.0], [-1.0, -2.0, 0.01]]);
        let expected = scale([1.0, 2.0, 0.0], 1.0 / 5f64.sqrt());
        assert!(dot(axis, expected).abs() > 0.9999);
    }
}
