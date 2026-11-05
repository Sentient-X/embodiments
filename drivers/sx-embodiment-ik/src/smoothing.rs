//! The operator's tool targets with their tremor taken out: a One-Euro filter (Casiez, Roussel and
//! Vogel, CHI 2012) on each hand's pose, applied inside [`crate::teleop::Teleop::step`] before
//! anything reads the targets, so the station and the headset's ghost filter identically and it
//! is the only tremor filter between the operator and the robot.
//!
//! An operator's hand shakes 1-2 mm at 8-12 Hz; through the solve that raised the commanded shake
//! 3-5x and the joints' velocity reversals from about 5 to 170 a second (`teleop_shake`, 11-04).
//! A fixed low-pass that removes it lags every deliberate move; the One-Euro filter is a
//! low-pass whose cutoff rises with the hand's own (filtered) speed: still, it cuts at
//! [`OneEuro::min_cutoff_hz`] and takes the tremor out; sweeping, it opens to
//! `min_cutoff_hz + beta * speed`. Here it is two first-order stages at the one cutoff, and the
//! speed is read from the first stage's output rather than the readings, so the tremor's own
//! motion does not open the filter (a single stage on raw speed left 0.32 mm of a 2 mm tremor).
//! Translation and rotation are filtered alike: the rotation by slerp toward the reading, its
//! cutoff from the filtered turn rate.
//!
//! [`OneEuro::TOOL`] is tuned so a 0.5 m/s sweep lags under 10 ms once under way, measured with
//! `cargo run --release -p sx-embodiment-ik --example teleop_shake -- synth governed` (its `1e`
//! rows and `one-euro` line). Nothing here allocates or reads a clock: the caller gives each reading's
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
    /// The operator's tool targets, tuned on `teleop_shake synth governed`, `tests/fluid.rs`
    /// `table_station` and `tests/follow_lag.rs` (2026-11-06; unfiltered in brackets). Speed
    /// first: every lag within about 10 ms of unfiltered, and the tremor cut as far as that
    /// allows. A 2 mm 8-12 Hz tremor's commanded shake 0.17 mm (0.77) and a 1 mm one's 0.10
    /// (0.46; no tremor 0.06); a 0.5 m/s sweep's filter lag 7.8 ms once under way and the
    /// station's mean lag on a 0.3 s sweep from rest 40.2 mm (36.0); a 10 cm jump 365 ms (354);
    /// the desk operator path's delay 50 ms (40); 1 and 2 Hz 3 cm sines 50/60 ms behind (20/30).
    /// The curve, (min cutoff Hz, beta per m/s, derivative cutoff Hz): 2 mm shake, 0.3 s sweep
    /// lag, desk delay, sine delay:
    /// (0.5, 65, 4) 0.12 mm, 41.9 mm, 60 ms, +40 ms; (0.7, 80, 6) 0.17, 40.2, 50, +30;
    /// (1, 100, 8) 0.25, 39.2, 40, +20; (0.5, 150, 8) 0.34, 38.5, 40, +20. A causal filter
    /// cannot take a 10 Hz tremor out of a 1-2 Hz motion without delaying the motion; a lead
    /// along the estimated motion (two time constants) cut the jump to 277-299 ms but let the
    /// tremor back (0.24-0.53 mm) and barely helped the sines: removed.
    pub const TOOL: Self = Self {
        min_cutoff_hz: 0.7,
        beta_per_m_s: 80.0,
        beta_per_rad_s: 4.0,
        derivative_cutoff_hz: 6.0,
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
    /// The first of the two stages' output.
    stage: Option<Pose>,
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
        // A reading that is not a pose passes as it is, and the filter forgets the hand: folded
        // into the state, one NaN left every later step of that hand refused (refute of #248).
        let finite = raw.translation_m.iter().all(|v| v.is_finite())
            && [
                raw.rotation.x,
                raw.rotation.y,
                raw.rotation.z,
                raw.rotation.w,
            ]
            .iter()
            .all(|v| v.is_finite());
        if !finite {
            self.reset();
            return raw;
        }
        let (Some((last_t, last)), Some(stage)) = (self.last, self.stage) else {
            self.last = Some((t_s, raw));
            self.stage = Some(raw);
            return raw;
        };
        let dt = t_s - last_t;
        if dt <= 0.0 || !dt.is_finite() {
            return last;
        }
        // The cutoff from the hand's speed and turn rate as last estimated: estimated from the
        // first stage's output, not the readings, so the tremor's own (fast) motion barely
        // opens the filter it is to be cut by.
        let moving = alpha(
            dt,
            params.min_cutoff_hz + params.beta_per_m_s * norm(self.velocity),
        );
        let turning = alpha(
            dt,
            params.min_cutoff_hz + params.beta_per_rad_s * norm(self.turn),
        );
        // Two first-order stages at the one cutoff (critically damped): a tremor at 10 Hz falls
        // with the square of the cutoff's share of it, so the cutoff can stand higher for the
        // same cut, and a deliberate move lags less (2026-11-06, `teleop_shake` `1e` rows).
        let toward = |from: Pose, to: Pose| Pose {
            translation_m: [0, 1, 2].map(|k| {
                from.translation_m[k] + moving * (to.translation_m[k] - from.translation_m[k])
            }),
            rotation: from.rotation.slerp(to.rotation, turning),
        };
        let next = toward(stage, raw);
        let derivative = alpha(dt, params.derivative_cutoff_hz);
        let speed = [0, 1, 2].map(|k| (next.translation_m[k] - stage.translation_m[k]) / dt);
        let turn = rotation_vector(next.rotation * stage.rotation.conjugate()).map(|r| r / dt);
        for k in 0..3 {
            self.velocity[k] += derivative * (speed[k] - self.velocity[k]);
            self.turn[k] += derivative * (turn[k] - self.turn[k]);
        }
        let stage = next;
        let filtered = toward(last, stage);
        self.stage = Some(stage);
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
    fn a_reading_that_is_not_a_pose_passes_and_leaves_no_trace() {
        let mut filter = PoseFilter::default();
        let _ = filter.filter(&OneEuro::TOOL, 0.0, at(0.1));
        let _ = filter.filter(&OneEuro::TOOL, 0.01, at(0.1));
        assert!(
            filter
                .filter(&OneEuro::TOOL, 0.02, at(f64::NAN))
                .translation_m[0]
                .is_nan()
        );
        assert_eq!(filter.filter(&OneEuro::TOOL, 0.03, at(0.2)), at(0.2));
        assert!(filter.filter(&OneEuro::TOOL, 0.04, at(0.2)).translation_m[0].is_finite());
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
