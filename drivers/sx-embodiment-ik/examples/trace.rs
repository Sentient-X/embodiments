//! One scripted operator trace through the station's own solve, for comparing solves: the
//! static upper body under the station's composed config (`tests/fixtures/static-teleop-config.json`)
//! from its commissioned Ready (`tests/fixtures/static-ready.json`), stepped at 100 Hz with an
//! ideal follower (each step measured where it was commanded), every tick written out.
//!
//! The left hand, from where it stands at Ready, with a physiological tremor (1 mm at 7-11 Hz,
//! 0.5 degrees at 8 Hz) on top throughout: holds 1.5 s; jumps 10 cm up at once and holds;
//! sweeps 15 cm left at 0.5 m/s and back; reaches 12 cm forward and 8 cm down over 1.2 s,
//! closes its gripper, lifts 8 cm; returns to Ready. The right hand holds where it stands, with
//! its own tremor.
//!
//! ```text
//! cargo run --release -p sx-embodiment-ik --example trace -- out.json
//! ```
//!
//! `out.json`: `{"axes": [names], "dt", "ticks": [{"t", "q": [per axis], "target": {"left",
//! "right"}: [x, y, z, qx, qy, qz, qw], "width": [left, right], "solve", "took_us"}]}`.

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::many_single_char_names
)]

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use serde_json::json;
use sx_embodiment_ik::teleop::{Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};
use sx_frames::{Pose, QuaternionLayout};

const DT: f64 = 0.01;
const END_S: f64 = 11.0;
const OPEN_M: f64 = 0.07;
const CLOSED_M: f64 = 0.03;

/// Minimum-jerk progress from 0 to 1 over `[start, start + length]`.
fn min_jerk(t: f64, start: f64, length: f64) -> f64 {
    let s = ((t - start) / length).clamp(0.0, 1.0);
    s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)
}

/// The left hand's offset from Ready at `t`, metres, and its gripper opening.
fn script(t: f64) -> ([f64; 3], f64) {
    let mut at = [0.0; 3];
    // A 10 cm jump up at 1.5 s.
    if t >= 1.5 {
        at[2] += 0.10;
    }
    // A 15 cm sweep left at 0.5 m/s (3.0-3.3 s), and back (4.5-4.8 s).
    at[1] += 0.5 * (t - 3.0).clamp(0.0, 0.3) - 0.5 * (t - 4.5).clamp(0.0, 0.3);
    // A pick-like reach: forward 12 cm and down 8 cm over 1.2 s from 5.5 s (15 cm put the
    // wrist on the coffee cell's counter), the gripper
    // closing over 0.4 s at 7.0 s, a lift of 8 cm over 0.8 s from 7.6 s.
    let reach = min_jerk(t, 5.5, 1.2);
    at[0] += 0.12 * reach;
    at[2] -= 0.08 * reach;
    at[2] += 0.08 * min_jerk(t, 7.6, 0.8);
    // Back to Ready over 1.5 s from 9.0 s.
    let back = min_jerk(t, 9.0, 1.5);
    for v in &mut at {
        *v *= 1.0 - back;
    }
    let width = OPEN_M - (OPEN_M - CLOSED_M) * (min_jerk(t, 7.0, 0.4) - min_jerk(t, 9.4, 0.4));
    (at, width)
}

/// A physiological tremor: per axis, three sines and none of the noise a tracker adds (the
/// trace is the same bytes every run).
fn tremor(t: f64, seed: f64) -> ([f64; 3], [f64; 3]) {
    let mut shake = [0.0; 3];
    for (k, v) in shake.iter_mut().enumerate() {
        let phase = seed + k as f64 * 1.7;
        *v = 0.001 * (TAU * 7.0 * t + phase).sin()
            + 0.0007 * (TAU * 9.5 * t + 2.0 * phase).sin()
            + 0.0005 * (TAU * 11.0 * t + 3.0 * phase).sin();
    }
    let angle = 0.5_f64.to_radians() * (TAU * 8.0 * t + seed).sin();
    let axis = [0.6, -0.48, 0.64];
    (shake, axis.map(|v| v * angle))
}

fn rotated(pose: &Pose, by: [f64; 3]) -> Pose {
    let angle = (by[0] * by[0] + by[1] * by[1] + by[2] * by[2]).sqrt();
    let xyzw = QuaternionLayout::Xyzw;
    let [x, y, z, w] = xyzw.write(pose.rotation);
    let (s, c) = if angle < 1e-12 {
        (0.0, 1.0)
    } else {
        ((angle / 2.0).sin() / angle, (angle / 2.0).cos())
    };
    let [a, b, d] = by.map(|v| v * s);
    // (a, b, d, c) * (x, y, z, w)
    let q = [
        c * x + a * w + b * z - d * y,
        c * y - a * z + b * w + d * x,
        c * z + a * y - b * x + d * w,
        c * w - a * x - b * y - d * z,
    ];
    Pose {
        translation_m: pose.translation_m,
        rotation: xyzw.read(q),
    }
}

/// The targets the solve took (the operator's hands filtered, `Teleop::taken`).
fn taken_targets(solve: &Teleop) -> Option<[Pose; 2]> {
    let taken = solve.taken();
    Some([taken.tcp[0]?, taken.tcp[1]?])
}

fn document(pose: &Pose) -> [f64; 7] {
    let [x, y, z, w] = QuaternionLayout::Xyzw.write(pose.rotation);
    let t = pose.translation_m;
    [t[0], t[1], t[2], x, y, z, w]
}

fn main() {
    let out = std::env::args().nth(1).expect("an output path");
    let config = TeleopConfig::parse(include_str!("../tests/fixtures/static-teleop-config.json"))
        .expect("the station's solve config");
    let ready: BTreeMap<String, f64> =
        serde_json::from_str(include_str!("../tests/fixtures/static-ready.json")).expect("ready");
    let mut solve =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).expect("solve");
    // Answers, not a loaded host's clock: each tick's time is written beside it instead.
    solve.set_deadline(std::time::Duration::from_secs(3600));
    let names: Vec<String> = solve
        .model()
        .axes()
        .iter()
        .map(|a| a.name.clone())
        .collect();
    let mut q: Vec<f64> = names
        .iter()
        .map(|n| ready.get(n).copied().unwrap_or(0.0))
        .collect();
    let body = solve.ik().body().clone();
    let frames = body.model().poses(&q);
    let from = [body.frame(&frames, 0).pose(), body.frame(&frames, 1).pose()];
    let available = vec![true; q.len()];
    let mut ticks = Vec::new();
    solve.reset();
    let count = (END_S / DT).round() as usize;
    for tick in 1..=count {
        let t = tick as f64 * DT;
        let (offset, width) = script(t);
        let mut targets = [from[0], from[1]];
        for (hand, target) in targets.iter_mut().enumerate() {
            let (shake, turn) = tremor(t, 0.9 + 2.3 * hand as f64);
            for k in 0..3 {
                target.translation_m[k] += shake[k] + if hand == 0 { offset[k] } else { 0.0 };
            }
            *target = rotated(target, turn);
        }
        let widths = [width, OPEN_M];
        let ask = Targets {
            tcp: [Some(targets[0]), Some(targets[1])],
            gripper_width_m: [Some(widths[0]), Some(widths[1])],
            ..Targets::default()
        };
        let measured: Vec<Option<f64>> = q.iter().map(|v| Some(*v)).collect();
        let began = std::time::Instant::now();
        let step = solve
            .step(&q, &measured, &available, &ask, DT)
            .expect("a step");
        let took_us = began.elapsed().as_secs_f64() * 1e6;
        q.copy_from_slice(&step.q);
        let solve_state = format!("{:?}", step.solve);
        let taken = taken_targets(&solve).unwrap_or(targets);
        ticks.push(json!({
            "t": t,
            "q": q,
            "target": {"left": document(&targets[0]), "right": document(&targets[1])},
            "taken": {"left": document(&taken[0]), "right": document(&taken[1])},
            "width": widths,
            "solve": solve_state,
            "took_us": took_us,
        }));
    }
    let written = json!({"axes": names, "dt": DT, "ticks": ticks});
    std::fs::write(&out, written.to_string()).expect("written");
    println!("{count} ticks to {out}");
}
