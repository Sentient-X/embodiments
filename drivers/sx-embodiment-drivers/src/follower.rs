//! A joint's commissioned follower: how a position-controlled joint follows its command.
//!
//! The MIT law on a joint, `J x'' = kp (u - x) + kd (u' - x')`, is a second-order follower of
//! natural frequency `sqrt(kp / J)` and damping ratio `kd / (2 sqrt(kp J))`, with `J` the
//! joint's effective inertia (its links' and its rotor's through the gearbox). The solve
//! (`sx-embodiment-ik`) predicts each joint through it.

use serde::{Deserialize, Serialize};

/// A joint's closed-loop response to its command: its natural frequency (rad/s) and damping
/// ratio, commissioned per joint (`sx_embodiment_ik::teleop::TeleopConfig::followers`).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FollowerModel {
    pub omega: f64,
    pub zeta: f64,
    /// Whether the drive is sent each step's planned velocity too (the MIT law's
    /// `kd (v_des - v)`): the joint is pushed along its step for the tick it is sent.
    #[serde(default)]
    pub feedforward: bool,
}

/// The longest a prediction is checked, seconds.
pub const MAX_HORIZON_S: f64 = 2.0;

impl FollowerModel {
    /// The follower an MIT drive makes of its joint: position gain `kp` and damping `kd`
    /// over `inertia`, all at the motor (N m/rad, N m s/rad, kg m^2). `None` where any is
    /// not positive and finite: a joint whose inertia the registry does not know.
    #[must_use]
    pub fn of_mit(kp: f64, kd: f64, inertia: f64) -> Option<Self> {
        let ok = |v: f64| v.is_finite() && v > 0.0;
        if !(ok(kp) && ok(kd) && ok(inertia)) {
            return None;
        }
        let model = Self {
            omega: (kp / inertia).sqrt(),
            zeta: kd / (2.0 * (kp * inertia).sqrt()),
            feedforward: true,
        };
        model.is_valid().then_some(model)
    }

    /// Positive and finite.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.omega.is_finite() && self.zeta.is_finite() && self.omega > 0.0 && self.zeta > 0.0
    }

    /// From rest at an offset of 1 from its command (`e`), and at its command moving at 1 a
    /// second (`f`), where it is after `t` and how fast it moves: `[e, f, e', f']`.
    #[must_use]
    #[allow(clippy::many_single_char_names)] // the closed forms' own names
    pub fn response(&self, t: f64) -> [f64; 4] {
        let (w, z) = (self.omega, self.zeta);
        if (z - 1.0).abs() < 1e-3 {
            let decay = (-w * t).exp();
            return [
                (1.0 + w * t) * decay,
                t * decay,
                -w * w * t * decay,
                (1.0 - w * t) * decay,
            ];
        }
        if z < 1.0 {
            let (sigma, wd) = (z * w, w * (1.0 - z * z).sqrt());
            let (s, c) = (wd * t).sin_cos();
            let decay = (-sigma * t).exp();
            return [
                decay * (c + sigma / wd * s),
                decay * s / wd,
                -decay * w * w / wd * s,
                decay * (c - sigma / wd * s),
            ];
        }
        let root = (z * z - 1.0).sqrt();
        let (r1, r2) = (-w * (z - root), -w * (z + root));
        let (p, q) = ((r1 * t).exp(), (r2 * t).exp());
        let d = r1 - r2;
        [
            (r1 * q - r2 * p) / d,
            (p - q) / d,
            r1 * r2 * (q - p) / d,
            (r1 * p - r2 * q) / d,
        ]
    }

    /// How long its prediction is checked: its first two overshoots when it rings, else four of
    /// its slow time constants, never past [`MAX_HORIZON_S`] (a follower slower than that has
    /// made its first, largest overshoot inside it).
    #[must_use]
    pub fn horizon(&self) -> f64 {
        let (w, z) = (self.omega, self.zeta);
        let settle = if z < 1.0 {
            let wd = w * (1.0 - z * z).sqrt();
            (3.0 * std::f64::consts::PI / wd).min(4.0 / (z * w))
        } else {
            4.0 / (w * (z - (z * z - 1.0).sqrt()))
        };
        settle.min(MAX_HORIZON_S)
    }
}

#[cfg(test)]
#[allow(clippy::many_single_char_names)]
mod tests {
    use super::*;

    /// The closed forms answer a simulated PD: ringing, critical and overdamped.
    #[test]
    fn the_response_is_the_pd_it_models() {
        for (omega, zeta) in [
            (9.5, 0.2),
            (6.0, 0.15),
            (30.0, 0.7),
            (20.0, 1.0),
            (35.0, 3.6),
        ] {
            let model = FollowerModel {
                omega,
                zeta,
                feedforward: false,
            };
            for (x0, v0) in [(1.0, 0.0), (0.0, 1.0)] {
                let (mut x, mut v) = (x0, v0);
                let h = 1e-5;
                for step in 1..=50_000 {
                    let a = -omega * omega * x - 2.0 * zeta * omega * v;
                    v += a * h;
                    x += v * h;
                    if step % 10_000 == 0 {
                        let t = f64::from(step) * h;
                        let [e, f, de, df] = model.response(t);
                        let (want_x, want_v) = (e * x0 + f * v0, de * x0 + df * v0);
                        assert!((x - want_x).abs() < 1e-3, "{omega} {zeta} x at {t}");
                        assert!((v - want_v).abs() < 1e-2 * omega, "{omega} {zeta} v at {t}");
                    }
                }
            }
        }
    }

    /// The static upper body's left shoulder pitch at its gains and the registry's inertia:
    /// a ringing follower.
    #[test]
    fn an_mit_drive_is_its_follower() {
        let model = FollowerModel::of_mit(60.984, 1.091, 0.43).expect("a follower");
        assert!((model.omega - (60.984_f64 / 0.43).sqrt()).abs() < 1e-12);
        assert!(model.zeta < 0.2);
        assert!(FollowerModel::of_mit(60.0, 1.0, 0.0).is_none());
    }
}
