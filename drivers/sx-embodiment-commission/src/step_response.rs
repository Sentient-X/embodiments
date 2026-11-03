//! A joint's step response, fitted: the natural frequency, damping ratio and transport delay
//! of the second-order follower its controller makes of it, from what it was commanded and
//! where it was measured (`sx-identify --part step:<joint>`).
//!
//! The model is the MIT law: `x'' = omega^2 (u(t - delay) - offset - x) + 2 zeta omega (w(t -
//! delay) - x')`, `u` each tick's target and `w` its planned velocity (zero where none is sent),
//! held until the next. `offset` is where the joint rests short of its target (gravity, friction),
//! as it rested at the segment's start. The fit is a grid search, coarse then finer, over omega,
//! zeta and delay, minimising the squared position error. It is deterministic and allocates
//! only the samples.

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
    /// The fit's rms position error, radians.
    pub rms: f64,
    /// The largest overshoot past the step's final target, as a share of the step.
    pub overshoot: f64,
}

/// The integration step, seconds.
const H: f64 = 1e-3;

#[allow(clippy::many_single_char_names)] // the law's own names
fn simulate(samples: &[Sample], omega: f64, zeta: f64, delay: f64, offset: f64) -> f64 {
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
            let a = omega * omega * (u - x) + 2.0 * zeta * omega * (command.velocity - v);
            v += a * H;
            x += v * H;
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
    let search =
        |best: &mut (f64, f64, f64, f64), omegas: &[f64], zetas: &[f64], delays: &[f64]| {
            for &w in omegas {
                for &z in zetas {
                    for &d in delays {
                        let e = simulate(samples, w, z, d, offset);
                        if e < best.0 {
                            *best = (e, w, z, d);
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
    let mut best = (f64::INFINITY, 0.0, 0.0, 0.0);
    search(
        &mut best,
        // A ringing joint's error is rugged in omega (each swing's phase): a 4 % step.
        &grid(1.0, 60.0, 100, true),
        &grid(0.02, 2.0, 16, true),
        &grid(0.0, 0.1, 6, false),
    );
    for span in [1.0, 0.4, 0.15] {
        let (_, w, z, d) = best;
        search(
            &mut best,
            &grid(w * (1.0 - 0.15 * span), w * (1.0 + 0.15 * span), 9, false),
            &grid(z * (1.0 - 0.3 * span), z * (1.0 + 0.3 * span), 9, false),
            &grid((d - 0.02 * span).max(0.0), d + 0.02 * span, 9, false),
        );
    }
    let (error, omega, zeta, delay_s) = best;
    let overshoot = samples
        .iter()
        .map(|s| (s.position - (last - offset)) / size)
        .fold(0.0, f64::max);
    Some(Fit {
        omega,
        zeta,
        delay_s,
        offset,
        rms: (error / samples.len() as f64).sqrt(),
        overshoot,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A follower stepped at 50 Hz is fitted back to itself, its delay included.
    #[test]
    #[allow(clippy::many_single_char_names)] // the law's own names
    fn a_simulated_step_is_fitted_to_its_follower() {
        for (omega, zeta, delay) in [(12.0, 0.11, 0.04), (12.0, 0.5, 0.02), (5.0, 0.063, 0.0)] {
            let mut samples = Vec::new();
            let (mut x, mut v, mut t) = (0.0_f64, 0.0_f64, 0.0_f64);
            for tick in 0..150 {
                let at = f64::from(tick) * 0.02;
                let target = if at >= 0.1 { 0.02 } else { 0.0 };
                while t < at {
                    let u = if t - delay >= 0.1 { 0.02 } else { 0.0 };
                    let a = omega * omega * (u - x) - 2.0 * zeta * omega * v;
                    v += a * 1e-5;
                    x += v * 1e-5;
                    t += 1e-5;
                }
                samples.push(Sample {
                    t: at,
                    target,
                    velocity: 0.0,
                    position: x,
                });
            }
            let f = fit(&samples).expect("a fit");
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
}
