//! A joint's commissioned follower, and what the solve predicts from it.
//!
//! A position-controlled joint follows its command `u` as a second-order system,
//! `x'' = omega^2 (u - b - x) - 2 zeta omega x'`: its drive's stiffness and damping (MIT `kp`,
//! `kd`) over the joint's effective inertia (the links' plus the rotor's), and `b` the few
//! milliradians it rests short of its command under gravity or a load. A [`Tracker`] keeps the
//! joint's state `(x, x', b)` from its readings and its commands (a steady-state Kalman filter
//! on that model), says whether the readings bear the model out, and predicts where the joint
//! will be over the time it takes to settle, if its command were held from this step on:
//! `x(t) - seen = A(t) dq + c(t)`, linear in the step `dq`.

use sx_embodiment_drivers::FollowerModel;

/// How many times over the horizon a prediction is checked: the first [`EARLY`] ticks one by
/// one, the rest evenly to the horizon (a ring's peak between two samples is missed by under
/// 1% of its amplitude at 6 rad/s).
pub const SAMPLES: usize = 40;
const EARLY: usize = 4;

/// A reading off the model by more than this many of its expected spreads is the model
/// refused: the joint is not doing what its follower says (blocked, pushed, driven by something
/// else), and is not planned on until it has agreed again for [`DOUBT_S`].
const REFUSED: f64 = 5.0;
const DOUBT_S: f64 = 0.15;
/// A reading never refuses the model by less than this, radians: three readings' jitter.
const REFUSED_AT_LEAST: f64 = 0.003;

/// The filter's noise: a reading's (uniform within [`crate::READING_JITTER`]), the model's
/// (an unmodelled acceleration, rad/s^2, each tick), and the rest offset's drift (rad/s^0.5).
/// Trusting the model this much keeps a reading's jitter out of the prediction (at 3 rad/s^2
/// and 0.01, a held command dithered 1-3 mm under 1 mrad of jitter, its velocity estimate
/// 0.05 rad/s and rest offset 3 mrad noisy; at these, 0.07 mm), and still learns a 45 mrad
/// sag in about a second (at 1e-4 the arm sagged 0.4 mm inside the margin meanwhile).
const ACCEL_NOISE: f64 = 0.2;
const REST_DRIFT: f64 = 3e-4;

/// One joint's state as its follower model and readings have it, and its prediction tables.
#[derive(Clone, Debug)]
pub struct Tracker {
    pub model: FollowerModel,
    /// The tick the transition and gains are for; NaN before the first.
    dt: f64,
    phi: [[f64; 3]; 3],
    gamma: [f64; 3],
    gain: [f64; 3],
    /// The expected spread of a reading against the model's one-tick prediction.
    spread: f64,
    /// Position, velocity and rest offset (`b`, its command less where it rests).
    pub x: f64,
    pub v: f64,
    pub b: f64,
    pub ready: bool,
    /// Ticks left before a refused model is planned on again.
    doubt: u32,
    /// Per sample time: `e` and `f` ([`FollowerModel::response`]).
    pub e: Vec<f64>,
    pub f: Vec<f64>,
    /// Per sample time: how much of a step sent with its planned velocity
    /// ([`FollowerModel::feedforward`]) the joint has made by then.
    pub a: Vec<f64>,
}

impl Tracker {
    #[must_use]
    pub fn new(model: FollowerModel) -> Self {
        Self {
            model,
            dt: f64::NAN,
            phi: [[0.0; 3]; 3],
            gamma: [0.0; 3],
            gain: [0.0; 3],
            spread: 0.0,
            x: 0.0,
            v: 0.0,
            b: 0.0,
            ready: false,
            doubt: 0,
            e: vec![0.0; SAMPLES],
            f: vec![0.0; SAMPLES],
            a: vec![0.0; SAMPLES],
        }
    }

    /// Whether its readings bear the model out: planned on.
    #[must_use]
    pub const fn trusted(&self) -> bool {
        self.ready && self.doubt == 0
    }

    /// Forget the state: the next reading starts it, at rest.
    pub fn reset(&mut self) {
        self.ready = false;
        self.doubt = 0;
    }

    /// Ready the transition, the filter's gains and the prediction tables for tick `dt` and the
    /// sample `times` (no allocation: the tables are sized once).
    pub fn prepare(&mut self, dt: f64, times: &[f64], jitter: f64) {
        // The tables for exactly this tick are ready: the bits, not a margin.
        if self.dt.to_bits() == dt.to_bits() {
            return;
        }
        self.dt = dt;
        let [e, f, de, df] = self.model.response(dt);
        // State (x, v, b), input u: x - (u - b) decays by (e, f); v by (e', f').
        self.phi = [[e, f, -(1.0 - e)], [de, df, de], [0.0, 0.0, 1.0]];
        self.gamma = [1.0 - e, -de, 0.0];
        let r = (jitter * jitter / 3.0).max(1e-12);
        let g = [0.5 * dt * dt, dt, 0.0];

        let mut q = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                q[i][j] = ACCEL_NOISE * ACCEL_NOISE * g[i] * g[j];
            }
        }
        q[2][2] = REST_DRIFT * REST_DRIFT * dt;
        // The steady-state Riccati solution, iterated.
        let mut p = q;
        p[0][0] += r;
        p[2][2] += 1e-4;
        let mut s = r;
        for _ in 0..2000 {
            s = p[0][0] + r;
            let k = [p[0][0] / s, p[1][0] / s, p[2][0] / s];
            let mut post = p;
            for i in 0..3 {
                for j in 0..3 {
                    post[i][j] -= k[i] * p[0][j];
                }
            }
            let mut next = q;
            for i in 0..3 {
                for j in 0..3 {
                    let mut sum = 0.0;
                    for a in 0..3 {
                        for c in 0..3 {
                            sum += self.phi[i][a] * post[a][c] * self.phi[j][c];
                        }
                    }
                    next[i][j] += sum;
                }
            }
            p = next;
        }
        self.gain = [p[0][0] / s, p[1][0] / s, p[2][0] / s];
        self.spread = s.sqrt();
        // A planned velocity sent with a step pushes as a command `2 zeta / omega` times it
        // ahead, for the one tick it is sent.
        let push = self.push(dt);
        for (k, &t) in times.iter().enumerate() {
            let [e, f, _, _] = self.model.response(t);
            let before = if t > dt {
                self.model.response(t - dt)[0]
            } else {
                1.0
            };
            self.e[k] = e;
            self.f[k] = f;
            self.a[k] = push.mul_add(before - e, 1.0 - e);
        }
    }

    /// Whether a reading `y` after a tick of command `u` is one the model expects: a joint
    /// planned on, read within what refuses the model of where it says the joint is.
    #[must_use]
    pub fn expects(&self, y: f64, u: f64) -> bool {
        if !self.trusted() {
            return false;
        }
        let s = [self.x, self.v, self.b];
        let ahead = self.gamma[0] * u + (0..3).map(|j| self.phi[0][j] * s[j]).sum::<f64>();
        (y - ahead).abs() <= (REFUSED * self.spread).max(REFUSED_AT_LEAST)
    }

    /// How far ahead of its command a step's planned velocity pushes the joint, per unit of
    /// step, over the tick it is sent: none without [`FollowerModel::feedforward`].
    fn push(&self, dt: f64) -> f64 {
        if self.model.feedforward {
            2.0 * self.model.zeta / (self.model.omega * dt)
        } else {
            0.0
        }
    }

    /// The command the joint followed over the last tick, `u`, sent with planned velocity
    /// `velocity`: with [`FollowerModel::feedforward`], the command that velocity pushed it
    /// like, `2 zeta / omega` times it ahead.
    #[must_use]
    pub fn driven(&self, u: f64, velocity: f64) -> f64 {
        if self.model.feedforward {
            (2.0 * self.model.zeta / self.model.omega).mul_add(velocity, u)
        } else {
            u
        }
    }

    /// Take a reading `y` (`None`: unread) after a tick of command `u`.
    pub fn read(&mut self, y: Option<f64>, u: f64) {
        let Some(y) = y else {
            self.ready = false;
            return;
        };
        if !self.ready {
            (self.x, self.v, self.b, self.ready) = (y, 0.0, 0.0, true);
            self.doubt = self.doubt_ticks();
            return;
        }
        let s = [self.x, self.v, self.b];
        let mut ahead = [0.0; 3];
        for i in 0..3 {
            ahead[i] = self.gamma[i] * u + (0..3).map(|j| self.phi[i][j] * s[j]).sum::<f64>();
        }
        let surprise = y - ahead[0];
        self.x = self.gain[0].mul_add(surprise, ahead[0]);
        self.v = self.gain[1].mul_add(surprise, ahead[1]);
        self.b = self.gain[2].mul_add(surprise, ahead[2]);
        if surprise.abs() > (REFUSED * self.spread).max(REFUSED_AT_LEAST) {
            self.doubt = self.doubt_ticks();
        } else {
            self.doubt = self.doubt.saturating_sub(1);
        }
    }

    fn doubt_ticks(&self) -> u32 {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let ticks = (DOUBT_S / self.dt).ceil() as u32;
        ticks.max(1)
    }
}

/// The sample times for tick `dt` over `horizon`: the first [`EARLY`] ticks, then evenly.
pub fn sample_times(dt: f64, horizon: f64, times: &mut [f64]) {
    let early = EARLY.min(times.len());
    for (k, t) in times.iter_mut().enumerate().take(early) {
        *t = dt * (k + 1) as f64;
    }
    let start = dt * early as f64;
    let rest = times.len() - early;
    let span = (horizon - start).max(0.0);
    for k in 0..rest {
        times[early + k] = start + span * (k + 1) as f64 / rest as f64;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The filter follows a PD resting short of its command: its rest offset and its position.
    #[test]
    fn the_filter_finds_the_rest_offset() {
        let model = FollowerModel {
            omega: 30.0,
            zeta: 0.7,
            feedforward: false,
        };
        let dt = 0.01;
        let mut tracker = Tracker::new(model);
        let mut times = vec![0.0; SAMPLES];
        sample_times(dt, model.horizon(), &mut times);
        tracker.prepare(dt, &times, 0.001);
        let (mut x, mut v, sag, u) = (0.0, 0.0, 0.004, 0.1);
        for _ in 0..400 {
            tracker.read(Some(x), u);
            let h = dt / 100.0;
            for _ in 0..100 {
                let a = omega_sq(model) * (u - sag - x) - 2.0 * model.zeta * model.omega * v;
                v += a * h;
                x += v * h;
            }
        }
        assert!((tracker.b - sag).abs() < 2e-4, "rest offset {}", tracker.b);
        assert!(tracker.trusted());
    }

    fn omega_sq(model: FollowerModel) -> f64 {
        model.omega * model.omega
    }
}
