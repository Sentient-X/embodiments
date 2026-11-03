//! A joint's step response, fitted: the natural frequency, damping ratio and transport delay
//! of the second-order follower its controller makes of it, from what it was commanded and
//! where it was measured (`sx-identify --part step:<joint>`).
//!
//! The model is the MIT law: `x'' = omega^2 (u(t - delay) - offset - x) + 2 zeta omega (w(t -
//! delay) - x')`, `u` each tick's target and `w` its planned velocity (zero where none is sent),
//! held until the next, less Coulomb friction: a band of `band` radians of the position term the
//! joint sticks within and is braked by while it moves. Unmodelled, friction reads as damping:
//! a 5 mrad band made a zeta 0.10 joint read 0.41 (refute of 572d868). `offset` is where the
//! joint rests short of its target (gravity), as it rested at the segment's start. The fit is a
//! grid search, coarse then finer, over omega, zeta, delay and band, minimising the squared
//! position error; [`Fit::explains`] says whether it explains the step at all. It is
//! deterministic and allocates only the grids.

/// One measured sample: seconds since the segment began, the target and planned velocity
/// commanded then (held until the next sample), and the position measured.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub t: f64,
    pub target: f64,
    pub velocity: f64,
    pub position: f64,
}

/// What a step's response says the follower is.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Fit {
    pub omega: f64,
    pub zeta: f64,
    pub delay_s: f64,
    /// Where the joint rests short of its target, radians.
    pub offset: f64,
    /// The Coulomb friction band, radians of the position term.
    pub band: f64,
    /// The fit's rms position error, radians.
    pub rms: f64,
    /// The largest overshoot past the step's final target, as a share of the step.
    pub overshoot: f64,
}

/// The integration step, seconds.
const H: f64 = 1e-3;
/// The grids' ends: a fit on one is not a fit.
const OMEGA: (f64, f64) = (1.0, 60.0);
const ZETA: (f64, f64) = (0.02, 2.0);

impl Fit {
    /// Whether the fit explains a step of `size` radians: its rms within a tenth of the step,
    /// and omega and zeta inside their grids, not pinned to an end (a joint friction holds
    /// still fits to an end, refute of 572d868: omega 0.78, zeta 0.012 or 3.04).
    #[must_use]
    pub fn explains(&self, size: f64) -> bool {
        let inside = |v: f64, (lo, hi): (f64, f64)| v > lo * 1.05 && v < hi / 1.05;
        self.rms <= 0.1 * size.abs() && inside(self.omega, OMEGA) && inside(self.zeta, ZETA)
    }
}

#[allow(clippy::many_single_char_names)] // the law's own names
fn simulate(samples: &[Sample], omega: f64, zeta: f64, delay: f64, offset: f64, band: f64) -> f64 {
    let friction = omega * omega * band;
    let (mut x, mut v) = (samples[0].position, 0.0);
    let mut t = samples[0].t;
    let mut error = 0.0;
    let command_at = |time: f64| -> Sample {
        // The command in force at `time`: the last sample's at or before it.
        let i = samples.partition_point(|s| s.t <= time);
        samples[i.saturating_sub(1)]
    };
    for s in samples {
        while t < s.t {
            let command = command_at(t - delay);
            let u = command.target - offset;
            let drive = omega * omega * (u - x) + 2.0 * zeta * omega * (command.velocity - v);
            if v == 0.0 && drive.abs() <= friction {
                // Stuck: the drive does not pass the friction.
            } else {
                let a = drive - friction * if v == 0.0 { drive.signum() } else { v.signum() };
                let next = v + a * H;
                // Friction stops the joint; it never turns it.
                v = if friction > 0.0 && v != 0.0 && next.signum() != v.signum() {
                    0.0
                } else {
                    next
                };
                x += v * H;
            }
            t += H;
        }
        error += (x - s.position).powi(2);
    }
    error
}

/// Fit `samples` (at least ten, times increasing, the first at rest): `None` when they are too
/// few or the target never moves.
#[must_use]
#[allow(clippy::cast_precision_loss)] // sample counts
pub fn fit(samples: &[Sample]) -> Option<Fit> {
    type Best = (f64, f64, f64, f64, f64);
    if samples.len() < 10 || samples.windows(2).any(|w| w[1].t <= w[0].t) {
        return None;
    }
    let first = samples[0].target;
    let last = samples[samples.len() - 1].target;
    let size = last - first;
    if size.abs() < 1e-6 {
        return None;
    }
    // At rest before the step: what gravity and friction hold it short by, kept over a step
    // this small.
    let offset = samples[0].target - samples[0].position;
    let search = |best: &mut Best, omegas: &[f64], zetas: &[f64], delays: &[f64], bands: &[f64]| {
        for &w in omegas {
            for &z in zetas {
                for &d in delays {
                    for &b in bands {
                        let e = simulate(samples, w, z, d, offset, b);
                        if e < best.0 {
                            *best = (e, w, z, d, b);
                        }
                    }
                }
            }
        }
    };
    let grid = |lo: f64, hi: f64, n: usize, log: bool| -> Vec<f64> {
        (0..n)
            .map(|i| {
                let s = i as f64 / (n - 1) as f64;
                if log {
                    lo * (hi / lo).powf(s)
                } else {
                    lo + (hi - lo) * s
                }
            })
            .collect()
    };
    let step = size.abs();
    // Friction and damping both shrink a ring: each is searched from its own start, one with
    // no friction, one with it, and the better kept (from one start, a frictionless zeta 0.063
    // torso read 0.02 with a 1 mrad band).
    let mut best = (f64::INFINITY, 0.0, 0.0, 0.0, 0.0);
    for bands in [
        &[0.0][..],
        &[0.05 * step, 0.1 * step, 0.2 * step, 0.4 * step],
    ] {
        let mut start = (f64::INFINITY, 0.0, 0.0, 0.0, 0.0);
        search(
            &mut start,
            // A ringing joint's error is rugged in omega (each swing's phase): a 4 % step.
            &grid(OMEGA.0, OMEGA.1, 100, true),
            &grid(ZETA.0, ZETA.1, 16, true),
            &grid(0.0, 0.1, 5, false),
            bands,
        );
        for span in [1.0, 0.4, 0.15] {
            let (_, w, z, d, b) = start;
            search(
                &mut start,
                &grid(w * (1.0 - 0.15 * span), w * (1.0 + 0.15 * span), 7, false),
                &grid(z * (1.0 - 0.3 * span), z * (1.0 + 0.3 * span), 7, false),
                &grid((d - 0.025 * span).max(0.0), d + 0.025 * span, 7, false),
                &grid(
                    (b - 0.05 * step * span).max(0.0),
                    b + 0.05 * step * span,
                    5,
                    false,
                ),
            );
        }
        if start.0 < best.0 {
            best = start;
        }
    }
    let (error, omega, zeta, delay_s, band) = best;
    let overshoot = samples
        .iter()
        .map(|s| (s.position - (last - offset)) / size)
        .fold(0.0, f64::max);
    Some(Fit {
        omega,
        zeta,
        delay_s,
        offset,
        band,
        rms: (error / samples.len() as f64).sqrt(),
        overshoot,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A step of 0.02 rad at 50 Hz from a follower with `band` radians of Coulomb friction.
    #[allow(clippy::many_single_char_names)] // the law's own names
    fn stepped(omega: f64, zeta: f64, delay: f64, band: f64) -> Vec<Sample> {
        let friction = omega * omega * band;
        let mut samples = Vec::new();
        let (mut x, mut v, mut t) = (0.0_f64, 0.0_f64, 0.0_f64);
        for tick in 0..150 {
            let at = f64::from(tick) * 0.02;
            let target = if at >= 0.1 { 0.02 } else { 0.0 };
            while t < at {
                let u = if t - delay >= 0.1 { 0.02 } else { 0.0 };
                let drive = omega * omega * (u - x) - 2.0 * zeta * omega * v;
                if !(v == 0.0 && drive.abs() <= friction) {
                    let a = drive - friction * if v == 0.0 { drive.signum() } else { v.signum() };
                    let next = v + a * 1e-5;
                    v = if friction > 0.0 && v != 0.0 && next.signum() != v.signum() {
                        0.0
                    } else {
                        next
                    };
                    x += v * 1e-5;
                }
                t += 1e-5;
            }
            samples.push(Sample {
                t: at,
                target,
                velocity: 0.0,
                position: x,
            });
        }
        samples
    }

    /// A follower stepped at 50 Hz is fitted back to itself, its delay included.
    #[test]
    fn a_simulated_step_is_fitted_to_its_follower() {
        for (omega, zeta, delay) in [(12.0, 0.11, 0.04), (12.0, 0.5, 0.02), (5.0, 0.063, 0.0)] {
            let f = fit(&stepped(omega, zeta, delay, 0.0)).expect("a fit");
            assert!(f.explains(0.02), "{omega} {zeta}: {f:?}");
            assert!(
                (f.omega - omega).abs() < 0.05 * omega,
                "{omega} {zeta}: {f:?}"
            );
            assert!(
                (f.zeta - zeta).abs() < 0.1 * zeta + 0.01,
                "{omega} {zeta}: {f:?}"
            );
            assert!((f.delay_s - delay).abs() < 0.006, "{omega} {zeta}: {f:?}");
        }
    }

    /// Friction is not damping: a ringing joint (zeta 0.10) with a 2 or 5 mrad friction band
    /// never reads as damped (the refute of 572d868 read 0.41 with friction unmodelled), and a
    /// damped one (0.5) still does.
    #[test]
    fn friction_is_not_read_as_damping() {
        for band in [0.002, 0.005] {
            let ringing = fit(&stepped(12.0, 0.10, 0.03, band)).expect("a fit");
            assert!(
                !(ringing.explains(0.02) && ringing.zeta >= 0.4),
                "band {band}: {ringing:?}"
            );
            let damped = fit(&stepped(12.0, 0.5, 0.03, band)).expect("a fit");
            assert!(
                damped.explains(0.02) && damped.zeta >= 0.4,
                "band {band}: {damped:?}"
            );
        }
    }
}
