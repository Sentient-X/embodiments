//! A small dense convex QP, solved by a Mehrotra predictor-corrector interior-point method in
//! buffers sized once: no allocation, no logging, a fixed iteration bound and an optional
//! deadline.
//!
//! ```text
//! minimise 1/2 x'Hx + g'x  subject to  lower <= x <= upper,  R x <= r
//! ```
//!
//! `H` is dense `n x n` (positive definite), the box is per variable, and `R` holds up to
//! `rows` dense rows. Each iteration factors the `n x n` normal matrix `H + A' (Z/S) A` by
//! Cholesky; the box's rows are the identity, so they add only to its diagonal.

use std::time::Instant;

/// Why a solve stopped short.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QpFailure {
    /// The iteration bound was reached first.
    Iterations,
    /// The deadline passed first.
    Deadline,
    /// The normal matrix was not positive definite (bad data).
    Numerical,
}

/// The problem's data and every buffer a solve writes.
pub struct Qp {
    n: usize,
    /// Rows in use this solve.
    pub used: usize,
    pub h: Vec<f64>,
    pub g: Vec<f64>,
    pub lower: Vec<f64>,
    pub upper: Vec<f64>,
    /// Row-major `rows x n`.
    pub r: Vec<f64>,
    pub rhs: Vec<f64>,
    /// The answer.
    pub x: Vec<f64>,
    // Constraint order: n upper (x <= u), n lower (-x <= -l), then the used rows.
    s: Vec<f64>,
    z: Vec<f64>,
    ds: Vec<f64>,
    dz: Vec<f64>,
    ds_aff: Vec<f64>,
    dz_aff: Vec<f64>,
    rc: Vec<f64>,
    rp: Vec<f64>,
    rd: Vec<f64>,
    dx: Vec<f64>,
    m: Vec<f64>,
    rhs_x: Vec<f64>,
    ax: Vec<f64>,
}

impl Qp {
    #[must_use]
    pub fn new(n: usize, rows: usize) -> Self {
        let m = 2 * n + rows;
        Self {
            n,
            used: 0,
            h: vec![0.0; n * n],
            g: vec![0.0; n],
            lower: vec![0.0; n],
            upper: vec![0.0; n],
            r: vec![0.0; rows * n],
            rhs: vec![0.0; rows],
            x: vec![0.0; n],
            s: vec![0.0; m],
            z: vec![0.0; m],
            ds: vec![0.0; m],
            dz: vec![0.0; m],
            ds_aff: vec![0.0; m],
            dz_aff: vec![0.0; m],
            rc: vec![0.0; m],
            rp: vec![0.0; m],
            rd: vec![0.0; n],
            dx: vec![0.0; n],
            m: vec![0.0; n * n],
            rhs_x: vec![0.0; n],
            ax: vec![0.0; m],
        }
    }

    fn constraints(&self) -> usize {
        2 * self.n + self.used
    }

    /// `A v` into `out` (A: `[I; -I; R]`).
    fn a_times(&self, v: &[f64], out: &mut [f64]) {
        let n = self.n;
        for j in 0..n {
            out[j] = v[j];
            out[n + j] = -v[j];
        }
        for k in 0..self.used {
            let row = &self.r[k * n..(k + 1) * n];
            out[2 * n + k] = row.iter().zip(v).map(|(a, b)| a * b).sum();
        }
    }

    fn b(&self, i: usize) -> f64 {
        let n = self.n;
        if i < n {
            self.upper[i]
        } else if i < 2 * n {
            -self.lower[i - n]
        } else {
            self.rhs[i - 2 * n]
        }
    }

    /// Solve; the iterations taken.
    ///
    /// # Errors
    ///
    /// [`QpFailure`] when the bound or deadline passes first, or the data is not convex.
    #[allow(clippy::too_many_lines, clippy::many_single_char_names)]
    pub fn solve(
        &mut self,
        max_iterations: u32,
        deadline: Option<Instant>,
    ) -> Result<u32, QpFailure> {
        let n = self.n;
        let m = self.constraints();
        // Start inside the box, slacks and duals at one.
        for j in 0..n {
            self.x[j] = 0.5 * (self.lower[j] + self.upper[j]);
        }
        let mut ax = std::mem::take(&mut self.ax);
        self.a_times(&self.x, &mut ax);
        for i in 0..m {
            self.s[i] = (self.b(i) - ax[i]).max(1.0);
            self.z[i] = 1.0;
        }
        self.ax = ax;
        let scale_b = (0..m).map(|i| self.b(i).abs()).fold(1.0, f64::max);
        let scale_g = self.g.iter().map(|v| v.abs()).fold(1.0, f64::max);
        for iteration in 0..max_iterations {
            if deadline.is_some_and(|d| Instant::now() >= d) {
                return Err(QpFailure::Deadline);
            }
            // Residuals: rd = Hx + g + A'z, rp = Ax + s - b.
            let mut ax = std::mem::take(&mut self.ax);
            self.a_times(&self.x, &mut ax);
            for i in 0..m {
                self.rp[i] = ax[i] + self.s[i] - self.b(i);
            }
            self.ax = ax;
            for j in 0..n {
                let hx: f64 = (0..n).map(|k| self.h[j * n + k] * self.x[k]).sum();
                let mut atz = self.z[j] - self.z[n + j];
                for k in 0..self.used {
                    atz += self.r[k * n + j] * self.z[2 * n + k];
                }
                self.rd[j] = hx + self.g[j] + atz;
            }
            let mu: f64 = (0..m).map(|i| self.s[i] * self.z[i]).sum::<f64>() / m as f64;
            let primal = self.rp[..m].iter().fold(0.0_f64, |a, v| a.max(v.abs()));
            let dual = self.rd.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
            if primal <= 1e-9 * scale_b && dual <= 1e-9 * scale_g && mu <= 1e-10 {
                return Ok(iteration);
            }
            // Normal matrix M = H + A' (Z/S) A, factored once for both directions.
            self.m.copy_from_slice(&self.h);
            for j in 0..n {
                self.m[j * n + j] += self.z[j] / self.s[j] + self.z[n + j] / self.s[n + j] + 1e-12;
            }
            for k in 0..self.used {
                let w = self.z[2 * n + k] / self.s[2 * n + k];
                let row = &self.r[k * n..(k + 1) * n];
                for i in 0..n {
                    if row[i] == 0.0 {
                        continue;
                    }
                    for j in 0..n {
                        self.m[i * n + j] += w * row[i] * row[j];
                    }
                }
            }
            if !cholesky(&mut self.m, n) {
                return Err(QpFailure::Numerical);
            }
            // Predictor: rc = -s z.
            for i in 0..m {
                self.rc[i] = -self.s[i] * self.z[i];
            }
            self.direction();
            let alpha = self.step_length();
            let mut mu_aff = 0.0;
            for i in 0..m {
                mu_aff += (self.s[i] + alpha * self.ds[i]) * (self.z[i] + alpha * self.dz[i]);
                self.ds_aff[i] = self.ds[i];
                self.dz_aff[i] = self.dz[i];
            }
            mu_aff /= m as f64;
            let sigma = (mu_aff / mu).powi(3).clamp(0.0, 1.0);
            // Corrector.
            for i in 0..m {
                self.rc[i] = -self.s[i] * self.z[i] - self.ds_aff[i] * self.dz_aff[i] + sigma * mu;
            }
            self.direction();
            let alpha = 0.99 * self.step_length();
            for j in 0..n {
                self.x[j] += alpha * self.dx[j];
            }
            for i in 0..m {
                self.s[i] = (self.s[i] + alpha * self.ds[i]).max(1e-13);
                self.z[i] = (self.z[i] + alpha * self.dz[i]).max(1e-13);
            }
        }
        Err(QpFailure::Iterations)
    }

    /// The Newton direction for the current `rc`, with `self.m` holding the factor:
    /// `M dx = -rd - A' S^-1 (rc + Z rp)`, `ds = -rp - A dx`, `dz = (rc - Z ds) / S`.
    fn direction(&mut self) {
        let n = self.n;
        let m = self.constraints();
        for i in 0..m {
            self.ds[i] = (self.rc[i] + self.z[i] * self.rp[i]) / self.s[i]; // scratch
        }
        for j in 0..n {
            let mut v = -self.rd[j] - (self.ds[j] - self.ds[n + j]);
            for k in 0..self.used {
                v -= self.r[k * n + j] * self.ds[2 * n + k];
            }
            self.rhs_x[j] = v;
        }
        cholesky_solve(&self.m, n, &mut self.rhs_x);
        self.dx.copy_from_slice(&self.rhs_x);
        let mut ax = std::mem::take(&mut self.ax);
        self.a_times(&self.dx, &mut ax);
        for i in 0..m {
            self.ds[i] = -self.rp[i] - ax[i];
            self.dz[i] = (self.rc[i] - self.z[i] * self.ds[i]) / self.s[i];
        }
        self.ax = ax;
    }

    /// The largest step in (0, 1] keeping slacks and duals positive.
    fn step_length(&self) -> f64 {
        let mut alpha = 1.0_f64;
        for i in 0..self.constraints() {
            if self.ds[i] < 0.0 {
                alpha = alpha.min(-self.s[i] / self.ds[i]);
            }
            if self.dz[i] < 0.0 {
                alpha = alpha.min(-self.z[i] / self.dz[i]);
            }
        }
        alpha
    }
}

/// In-place lower Cholesky of a symmetric positive-definite row-major `n x n` matrix.
fn cholesky(a: &mut [f64], n: usize) -> bool {
    for j in 0..n {
        let mut d = a[j * n + j];
        for k in 0..j {
            d -= a[j * n + k] * a[j * n + k];
        }
        if !(d > 0.0 && d.is_finite()) {
            return false;
        }
        let d = d.sqrt();
        a[j * n + j] = d;
        for i in j + 1..n {
            let mut v = a[i * n + j];
            for k in 0..j {
                v -= a[i * n + k] * a[j * n + k];
            }
            a[i * n + j] = v / d;
        }
    }
    true
}

/// Solve `L L' x = b` in place with the factor from [`cholesky`].
fn cholesky_solve(l: &[f64], n: usize, b: &mut [f64]) {
    for i in 0..n {
        let mut v = b[i];
        for k in 0..i {
            v -= l[i * n + k] * b[k];
        }
        b[i] = v / l[i * n + i];
    }
    for i in (0..n).rev() {
        let mut v = b[i];
        for k in i + 1..n {
            v -= l[k * n + i] * b[k];
        }
        b[i] = v / l[i * n + i];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_box_and_a_row_bind_where_they_should() {
        // min (x0 - 2)^2 + (x1 - 2)^2, x in [-1, 1]^2, x0 + x1 <= 1.
        let mut qp = Qp::new(2, 1);
        qp.h.copy_from_slice(&[2.0, 0.0, 0.0, 2.0]);
        qp.g.copy_from_slice(&[-4.0, -4.0]);
        qp.lower.copy_from_slice(&[-1.0, -1.0]);
        qp.upper.copy_from_slice(&[1.0, 1.0]);
        qp.used = 1;
        qp.r.copy_from_slice(&[1.0, 1.0]);
        qp.rhs[0] = 1.0;
        qp.solve(50, None).expect("solved");
        assert!(
            (qp.x[0] - 0.5).abs() < 1e-7 && (qp.x[1] - 0.5).abs() < 1e-7,
            "{:?}",
            qp.x
        );
        qp.used = 0;
        qp.solve(50, None).expect("solved");
        assert!(
            (qp.x[0] - 1.0).abs() < 1e-7 && (qp.x[1] - 1.0).abs() < 1e-7,
            "{:?}",
            qp.x
        );
    }
}
