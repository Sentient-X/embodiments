//! How far behind an operator's hand the teleoperation solve keeps the static upper body's tool
//! point: a desk-speed and a VR-speed hand path, followed through [`Teleop`] at the station's
//! own solve config, the joints a second-order motor (PD with the command's velocity fed
//! forward) that lags and rings as act-sim's do. Prints, per path and motor, the delay that best
//! lines the measured tool point up with the hand, the error left at that delay, the RMS and peak
//! error, how many ticks some joint's command stood at the reference gap, and how many some
//! joint was stalled (none should be: nothing stops these arms).
//!
//! ```text
//! cargo run --release -p sx-embodiment-ik --example follow_lag -- config.json ready.json
//! ```
//!
//! `config.json` is the station's composed solve config (`sx.teleop-solve/v2`, what it serves the
//! page); `ready.json` the commissioned Ready pose by joint. An operator's hands carry no tool
//! speed, so this measures what an operator feels.

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::too_many_lines,
    clippy::many_single_char_names,
    clippy::items_after_statements
)]

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use sx_embodiment_ik::teleop::{Pose, REFERENCE_GAP, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{Body, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};

const DT: f64 = 0.01;
const SECONDS: f64 = 8.0;
const SETTLE_S: f64 = 1.0;
const SUBSTEPS: usize = 10;

/// `turn` (about x, radians) applied after `q`.
fn turned_about_x(angle: f64, q: [f64; 4]) -> [f64; 4] {
    let (tx, tw) = ((angle / 2.0).sin(), (angle / 2.0).cos());
    let [rx, ry, rz, rw] = q;
    [
        tw * rx + tx * rw,
        tw * ry - tx * rz,
        tw * rz + tx * ry,
        tw * rw - tx * rx,
    ]
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let config =
        TeleopConfig::parse(&std::fs::read_to_string(&args[0]).expect("config")).expect("parse");
    let ready: BTreeMap<String, f64> =
        serde_json::from_str(&std::fs::read_to_string(&args[1]).expect("ready")).expect("ready");
    let body = Body::static_upper_body();
    let names: Vec<String> = body.model().axes().iter().map(|a| a.name.clone()).collect();
    let rest: Vec<f64> = names
        .iter()
        .map(|n| ready.get(n).copied().unwrap_or(0.0))
        .collect();
    let left = body
        .limbs()
        .iter()
        .position(|l| *l == "left")
        .expect("left");
    let tool = |q: &[f64]| body.frame(&body.model().poses(q), left);
    let start = tool(&rest);
    let start_q = {
        let p = sx_embodiment_drivers::kinematics::pose_of(&start.to_transform());
        p.rotation_xyzw
    };
    type Path = fn(f64) -> ([f64; 3], f64);
    let paths: [(&str, Path); 2] = [
        // A desk operator: 5 cm side to side at 0.5 Hz, 3 cm up and down at 0.25 Hz.
        ("desk", |t| {
            (
                [
                    0.0,
                    0.05 * (TAU * 0.5 * t).sin(),
                    0.03 * (TAU * 0.25 * t).sin(),
                ],
                0.0,
            )
        }),
        // A VR operator: an 8 cm circle at 1 Hz (0.5 m/s), the wrist rolling 15 degrees at 0.5 Hz.
        ("vr", |t| {
            (
                [0.0, 0.08 * (TAU * t).sin(), 0.08 * ((TAU * t).cos() - 1.0)],
                15_f64.to_radians() * (TAU * 0.5 * t).sin(),
            )
        }),
    ];
    // (name, natural frequency rad/s, damping ratio): act-sim's ringing (about 0.65 s a swing,
    // dying over a second), and a stiffer, well-damped motor.
    let motors = [("act-sim-like", 9.5, 0.2), ("stiff", 30.0, 0.7)];
    for (path_name, path) in paths {
        for (motor_name, omega, zeta) in motors {
            let mut teleop =
                Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config)
                    .expect("solve");
            let width = names.len();
            let mut reference = rest.clone();
            let (mut q, mut v) = (rest.clone(), vec![0.0; width]);
            let available = vec![true; width];
            teleop.reset();
            let ticks = (SECONDS / DT) as usize;
            let mut wanted = Vec::with_capacity(ticks);
            let mut got = Vec::with_capacity(ticks);
            let mut at_gap = 0;
            let mut stalled = 0;
            for tick in 0..ticks {
                let t = tick as f64 * DT;
                let (offset, roll) = path(t);
                let target = Pose {
                    translation_m: [0, 1, 2].map(|k| start.t[k] + offset[k]),
                    rotation_xyzw: turned_about_x(roll, start_q),
                };
                let measured: Vec<Option<f64>> = q.iter().copied().map(Some).collect();
                let targets = Targets {
                    tcp: [Some(target), None],
                    ..Targets::default()
                };
                let step = teleop
                    .step(&reference, &measured, &available, &targets, DT)
                    .expect("a step");
                reference.copy_from_slice(&step.q);
                let qdot = step.qdot.clone();
                stalled += usize::from(step.stalled.iter().any(|s| *s));
                if reference
                    .iter()
                    .zip(&q)
                    .any(|(r, m)| (r - m).abs() >= REFERENCE_GAP * 0.999)
                {
                    at_gap += 1;
                }
                // The motors over the tick: PD on the command, its velocity fed forward.
                let h = DT / SUBSTEPS as f64;
                for _ in 0..SUBSTEPS {
                    for j in 0..width {
                        let a = omega * omega * (reference[j] - q[j])
                            + 2.0 * zeta * omega * (qdot[j] - v[j]);
                        v[j] += a * h;
                        q[j] += v[j] * h;
                    }
                }
                wanted.push(target.translation_m);
                got.push(tool(&q).t);
            }
            let first = (SETTLE_S / DT) as usize;
            let apart = |a: [f64; 3], b: [f64; 3]| {
                (0..3).map(|k| (a[k] - b[k]).powi(2)).sum::<f64>().sqrt()
            };
            let mean_at = |delay: usize| {
                let n = ticks - first;
                (first..ticks)
                    .map(|i| apart(wanted[i - delay.min(i)], got[i]))
                    .sum::<f64>()
                    / n as f64
            };
            let best = (0..=60)
                .min_by(|a, b| mean_at(*a).total_cmp(&mean_at(*b)))
                .unwrap_or(0);
            let errors: Vec<f64> = (first..ticks).map(|i| apart(wanted[i], got[i])).collect();
            let rms = (errors.iter().map(|e| e * e).sum::<f64>() / errors.len() as f64).sqrt();
            let peak = errors.iter().copied().fold(0.0, f64::max);
            println!(
                "{path_name:5} {motor_name:13} delay {:3.0} ms (left {:4.1} mm), rms {:5.1} mm, \
                 peak {:5.1} mm, ticks at the gap {at_gap}, stalled {stalled}",
                best as f64 * DT * 1e3,
                mean_at(best) * 1e3,
                rms * 1e3,
                peak * 1e3
            );
        }
    }
}
