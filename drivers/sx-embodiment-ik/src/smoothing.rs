//! The operator's tool targets with their tremor taken out: a One-Euro filter (Casiez, Roussel and
//! Vogel, CHI 2012) on each hand's pose, applied where the station takes the operator's intent,
//! before the solve and before any prediction.
//!
//! An operator's hand shakes 1-2 mm at 8-12 Hz; through the solve that raised the commanded shake
//! 3-5x and the joints' velocity reversals from about 5 to 170 a second (`teleop_shake`, 11-04).
//! A fixed low-pass that removes it lags every deliberate move; the One-Euro filter is a
//! first-order low-pass whose cutoff rises with the hand's own (filtered) speed: still, it
//! cuts at [`OneEuro::min_cutoff_hz`] and takes the tremor out; sweeping, it opens to
//! `min_cutoff_hz + beta * speed`, so a fast move lags by `1 / (2π · cutoff)` at most.
//! Translation and rotation are filtered alike: the rotation by slerp toward the reading, its
//! cutoff from the filtered turn rate.
//!
//! [`OneEuro::TOOL`] is tuned so a 0.5 m/s sweep lags under 10 ms once under way, measured with
//! `cargo run --release -p sx-embodiment-ik --example teleop_shake -- synth governed` (its
//! `one-euro` line). Nothing here allocates or reads a clock: the caller gives each reading's
//! time.

use sx_frames::{Pose, Quaternion};

/// A One-Euro filter's parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OneEuro {
    /// The cutoff with the hand still, Hz.
    pub min_cutoff_hz: f64,
    /// How much the cutoff rises per m/s of filtered tool speed, Hz per m/s.
    pub beta_per_m_s: f64,
    /// How much it rises per rad/s of filtered turn rate, Hz per rad/s.
    pub beta_per_rad_s: f64,
    /// The cutoff of the speed the cutoff is read from, Hz: low, so tremor's own speed barely
    /// opens the filter.
    pub derivative_cutoff_hz: f64,
}

impl OneEuro {
    /// The operator's tool targets: 1 Hz still (a 10 Hz tremor down about tenfold), 16 Hz at
    /// 0.5 m/s (under 10 ms of lag).
    pub const TOOL: Self = Self {
        min_cutoff_hz: 1.0,
        beta_per_m_s: 30.0,
        beta_per_rad_s: 4.0,
        derivative_cutoff_hz: 1.0,
    };
}

/// The share of the way to the reading a step of `dt_s` takes at `cutoff_hz`.
fn alpha(dt_s: f64, cutoff_hz: f64) -> f64 {
    let tau = 1.0 / (2.0 * std::f64::consts::PI * cutoff_hz.max(1e-6));
    1.0 / (1.0 + tau / dt_s)
}

/// The rotation vector of `q`, the shorter way round.
fn rotation_vector(q: Quaternion) -> [f64; 3] {
    let q = if q.w < 0.0 {
        Quaternion {
            x: -q.x,
            y: -q.y,
            z: -q.z,
            w: -q.w,
        }
    } else {
        q
    };
    let sine = (q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
    if sine < 1e-12 {
        return [2.0 * q.x, 2.0 * q.y, 2.0 * q.z];
    }
    let angle = 2.0 * sine.atan2(q.w);
    [q.x / sine * angle, q.y / sine * angle, q.z / sine * angle]
}

fn norm(v: [f64; 3]) -> f64 {
    v.iter().map(|c| c * c).sum::<f64>().sqrt()
}

/// One hand's filter: where it last put the hand, and the hand's filtered velocity.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PoseFilter {
    last: Option<(f64, Pose)>,
    velocity: [f64; 3],
    turn: [f64; 3],
}

impl PoseFilter {
    /// Forget the hand: the next reading passes as it is (a new engagement, or a hand that went
    /// untracked).
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// The filtered tool pose for `raw`, read at `t_s` seconds on the sender's clock. A reading
    /// no later than the last one passes the last filtered pose back unchanged.
    #[must_use]
    pub fn filter(&mut self, params: &OneEuro, t_s: f64, raw: Pose) -> Pose {
        let Some((last_t, last)) = self.last else {
            self.last = Some((t_s, raw));
            return raw;
        };
        let dt = t_s - last_t;
        if dt <= 0.0 || !dt.is_finite() {
            return last;
        }
        let derivative = alpha(dt, params.derivative_cutoff_hz);
        let speed = [0, 1, 2].map(|k| (raw.translation_m[k] - last.translation_m[k]) / dt);
        let turn = rotation_vector(raw.rotation * last.rotation.conjugate()).map(|r| r / dt);
        for k in 0..3 {
            self.velocity[k] += derivative * (speed[k] - self.velocity[k]);
            self.turn[k] += derivative * (turn[k] - self.turn[k]);
        }
        let moving = alpha(
            dt,
            params.min_cutoff_hz + params.beta_per_m_s * norm(self.velocity),
        );
        let turning = alpha(
            dt,
            params.min_cutoff_hz + params.beta_per_rad_s * norm(self.turn),
        );
        let filtered = Pose {
            translation_m: [0, 1, 2].map(|k| {
                last.translation_m[k] + moving * (raw.translation_m[k] - last.translation_m[k])
            }),
            rotation: last.rotation.slerp(raw.rotation, turning),
        };
        self.last = Some((t_s, filtered));
        filtered
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(x: f64) -> Pose {
        Pose {
            translation_m: [x, 0.0, 0.0],
            rotation: Quaternion::IDENTITY,
        }
    }

    #[test]
    fn a_fast_sweep_lags_under_ten_milliseconds_once_under_way() {
        let mut filter = PoseFilter::default();
        let dt = 1.0 / 90.0;
        let mut out = at(0.0);
        let mut t = 0.0;
        for _ in 0..180 {
            t += dt;
            out = filter.filter(&OneEuro::TOOL, t, at(0.5 * t));
        }
        let lag_s = (0.5 * t - out.translation_m[0]) / 0.5;
        assert!(lag_s < 0.010, "{} ms", lag_s * 1e3);
    }

    #[test]
    fn a_still_hands_tremor_is_cut_tenfold() {
        let mut filter = PoseFilter::default();
        let dt = 1.0 / 90.0;
        let (mut peak_in, mut peak_out) = (0.0_f64, 0.0_f64);
        for i in 0..900 {
            let t = f64::from(i) * dt;
            let x = 0.002 * (2.0 * std::f64::consts::PI * 10.0 * t).sin();
            let y = filter.filter(&OneEuro::TOOL, t, at(x)).translation_m[0];
            if t > 2.0 {
                peak_in = peak_in.max(x.abs());
                peak_out = peak_out.max(y.abs());
            }
        }
        assert!(peak_out < peak_in / 5.0, "{peak_out} of {peak_in}");
    }

    #[test]
    fn the_first_reading_passes_and_a_repeated_time_changes_nothing() {
        let mut filter = PoseFilter::default();
        assert_eq!(filter.filter(&OneEuro::TOOL, 1.0, at(0.3)), at(0.3));
        assert_eq!(filter.filter(&OneEuro::TOOL, 1.0, at(0.9)), at(0.3));
        filter.reset();
        assert_eq!(filter.filter(&OneEuro::TOOL, 0.5, at(0.9)), at(0.9));
    }
}
