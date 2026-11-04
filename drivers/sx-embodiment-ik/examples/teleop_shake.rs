//! Why teleoperation looks shaky, and how slow it feels, measured: the before/after for any
//! change to the teleoperation solve, its caps, or the transport.
//!
//! ```text
//! # a station's real sessions (control journals exported by robot-bench's
//! # control_journal_lag.py export): commanded joints, planned velocity, readings
//! cargo run --release -p sx-embodiment-ik --example teleop_shake -- journal <session.json>...
//! # a synthetic operator through the same solve the station runs, each source switched on alone
//! cargo run --release -p sx-embodiment-ik --example teleop_shake -- synth [governed|<rad/s>] [tick_s]
//! ```
//!
//! Both print one row per session or scenario, every number from the embodiment's own model
//! (forward kinematics and each hand's Jacobian, [`Body::jacobian`]):
//!
//! - **tool**: the commanded tool point's speed, its jerk (RMS while moving, m/s^3) and how its
//!   velocity's power splits over 0-2, 2-6, 6-15 and 15+ Hz (operator tremor is 8-12 Hz);
//! - **null**: each hand's joint velocity projected onto its Jacobian's null space (motion that
//!   moves no tool), as a share of the joint velocity, and the arm joint whose null-space
//!   velocity reverses most often per second;
//! - **reversals**: commanded joint velocity sign flips per second, summed over the arm joints;
//! - **ticks**: the command period histogram, stalls (> 30 ms) followed by bunched arrivals;
//! - **tracking**: measured tool point against the commanded one, and its lag;
//! - **synth only**: the target's spectrum as the station receives it, `Held`/`Relaxed` ticks,
//!   ticks a speed or acceleration cap bound, reference re-seeds, and the felt latency split into
//!   transport, tick queueing, solve and follower.
//!
//! Synthetic operator: a hand moving 8 cm by 5 cm over 4 s, the other still, sampled at the
//! headset's 90 Hz; tremor of 2 mm at 8-12 Hz (three random-phase sines per axis) and 0.3 deg;
//! the network either ideal (145 ms constant) or LiveKit-like: 145 ms plus exponential jitter
//! (mean 12 ms), a 1 % loss, and stalls (0.5/s, 30-150 ms) that release bunched, the newest
//! kept. The follower reads the command 4 ticks late (the Spark's 26-63 ms arm lag) with 0.5 mrad
//! of noise. Caps: `governed` is the static upper body's governed table (robot-bench, "How fast
//! the robot moves"); a number caps every arm joint there, as the 10-28 (0.25) and 10-30 (1.0)
//! sessions ran.

#![allow(
    clippy::too_many_lines,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::many_single_char_names,
    clippy::needless_range_loop,
    clippy::doc_markdown
)]

use std::collections::BTreeMap;

use serde_json::Value;
use sx_embodiment_ik::teleop::{REFERENCE_GAP, TELEOP_SCHEMA, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{
    Active, Body, Frames, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF, Solve,
};
use sx_frames::{Pose, Quaternion};

const HANDS: [usize; 2] = [0, 1];
/// Joint speed below this is standing still, rad/s.
const STILL: f64 = 0.005;
/// A tool moving faster than this is moving, m/s.
const MOVING: f64 = 0.005;
/// Station-tick bunching: an interval over this is a stall.
const STALL_MS: f64 = 30.0;

/// One stream of commands in the model's axis order.
struct Series {
    t: Vec<f64>,
    q: Vec<Vec<f64>>,
    v: Vec<Vec<f64>>,
}

/// What a scenario adds beside the commands.
#[derive(Default)]
struct Extra {
    /// The targets as the solve received them, per tick, per hand.
    targets: Vec<[[f64; 3]; 2]>,
    measured: Option<Series>,
    held: usize,
    relaxed: usize,
    speed_bound: usize,
    accel_bound: usize,
    reseeds: usize,
    /// Arrival time of each kept intent minus its send time, s.
    transport: Vec<f64>,
    /// Tick time minus arrival of the intent that tick used, s.
    queue: Vec<f64>,
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let body = Body::static_upper_body();
    print_dof(&body);
    match args.first().map(String::as_str) {
        Some("journal") => {
            header(false);
            for path in &args[1..] {
                journal(&body, path);
            }
        }
        Some("synth") => synth(&body, args.get(1).map(String::as_str), args.get(2)),
        _ => eprintln!(
            "usage: teleop_shake journal <session.json>... | synth [governed|<rad/s>] [tick_s]"
        ),
    }
}

/// Each limb's degrees of freedom and the null space its task leaves.
fn print_dof(body: &Body) {
    let names: Vec<&str> = body.limbs();
    let axes = body.model().axes();
    println!("limb   axes  (torso shared)  task  null space (torso held / free)");
    for (index, name) in names.iter().enumerate() {
        let own = body.limb_axes(index);
        let shared = own
            .iter()
            .filter(|a| axes[**a].name == "torso_joint")
            .count();
        let task = if *name == "head" { 2 } else { 6 };
        println!(
            "{name:6} {:4}  {:14}  {task:4}  {} / {}",
            own.len(),
            shared,
            (own.len() - shared).saturating_sub(task),
            own.len().saturating_sub(task)
        );
    }
    println!();
}

fn header(synthetic: bool) {
    print!(
        "{:22} {:>6} {:>6} {:>6} {:>19} {:>12} {:>16} {:>5} {:>18} {:>26} {:>7} {:>11} {:>6}",
        "session",
        "move s",
        "tick",
        "stall",
        "shake c/m wobble c/m",
        "jerk p50/95",
        "tool vel 0-2/2-6/6-15/15+",
        "null",
        "null flips/s (worst)",
        "revs/s (worst joint)",
        "stopgo",
        "track p50/95",
        "lag ms"
    );
    if synthetic {
        print!(
            " {:>16} {:>5} {:>5} {:>6} {:>6} {:>5}",
            "target 0-2/../15+", "held", "relax", "speed", "accel", "reseed"
        );
    }
    println!();
}

fn index_of(body: &Body, names: &[String]) -> Vec<Option<usize>> {
    names
        .iter()
        .map(|n| body.model().axes().iter().position(|a| &a.name == n))
        .collect()
}

fn journal(body: &Body, path: &str) {
    let doc: Value =
        serde_json::from_str(&std::fs::read_to_string(path).expect("read")).expect("json");
    let names: Vec<String> = doc["names"]
        .as_array()
        .expect("names")
        .iter()
        .map(|n| n.as_str().expect("name").to_owned())
        .collect();
    let map = index_of(body, &names);
    let width = body.model().axes().len();
    let floats = |v: &Value| -> Vec<f64> {
        v.as_array()
            .expect("array")
            .iter()
            .map(|x| x.as_f64().expect("number"))
            .collect()
    };
    let to_model = |values: &[f64]| {
        let mut q = vec![0.0; width];
        for (value, slot) in values.iter().zip(&map) {
            if let Some(axis) = slot {
                q[*axis] = *value;
            }
        }
        q
    };
    let mut commands = Series {
        t: Vec::new(),
        q: Vec::new(),
        v: Vec::new(),
    };
    for row in doc["commands"].as_array().expect("commands") {
        commands.t.push(row[0].as_f64().expect("time") * 1e-9);
        commands.q.push(to_model(&floats(&row[1])));
        commands.v.push(to_model(&floats(&row[2])));
    }
    let mut readings = Series {
        t: Vec::new(),
        q: Vec::new(),
        v: Vec::new(),
    };
    for row in doc["readings"].as_array().expect("readings") {
        let fresh: Vec<bool> = row[2]
            .as_array()
            .expect("fresh")
            .iter()
            .map(|b| b.as_bool().unwrap_or(false))
            .collect();
        // Arm joints only matter here; a reading with a stale arm joint is skipped.
        let arm_fresh = names.iter().zip(&fresh).all(|(n, f)| *f || !is_arm(n));
        if arm_fresh {
            readings.t.push(row[0].as_f64().expect("time") * 1e-9);
            readings.q.push(to_model(&floats(&row[1])));
        }
    }
    if commands.t.len() < 200 {
        println!("{:22} {} commands: too few", short(path), commands.t.len());
        return;
    }
    let extra = Extra {
        measured: Some(readings),
        ..Extra::default()
    };
    analyse(body, &short(path), &commands, &extra, None);
}

fn short(path: &str) -> String {
    std::path::Path::new(path).file_stem().map_or_else(
        || path.to_owned(),
        |s| s.to_string_lossy().chars().take(22).collect(),
    )
}

fn is_arm(name: &str) -> bool {
    (name.starts_with("left_") || name.starts_with("right_")) && !name.contains("finger")
}

/// Every hand's tool point at `q`.
fn tools(body: &Body, frames: &mut Frames, q: &[f64]) -> [[f64; 3]; 2] {
    body.model().poses_into(q, frames);
    HANDS.map(|hand| body.frame(frames, hand).t)
}

/// Welch power of a 3-axis velocity series over 0-2, 2-6, 6-15 and 15+ Hz.
fn bands(v: &[[f64; 3]], fs: f64) -> [f64; 4] {
    const N: usize = 128;
    let mut power = [0.0; 4];
    if v.len() < N {
        return power;
    }
    let window: Vec<f64> = (0..N)
        .map(|i| 0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / N as f64).cos())
        .collect();
    let mut start = 0;
    while start + N <= v.len() {
        for axis in 0..3 {
            let mean = (start..start + N).map(|i| v[i][axis]).sum::<f64>() / N as f64;
            for k in 1..N / 2 {
                let (mut re, mut im) = (0.0, 0.0);
                for i in 0..N {
                    let x = (v[start + i][axis] - mean) * window[i];
                    let phase = -2.0 * std::f64::consts::PI * (k * i) as f64 / N as f64;
                    re += x * phase.cos();
                    im += x * phase.sin();
                }
                let f = k as f64 * fs / N as f64;
                let band = if f < 2.0 {
                    0
                } else if f < 6.0 {
                    1
                } else if f < 15.0 {
                    2
                } else {
                    3
                };
                power[band] += re.mul_add(re, im * im);
            }
        }
        start += N / 2;
    }
    power
}

/// Band powers as shares of their sum.
fn shares(mut power: [f64; 4]) -> [f64; 4] {
    let total: f64 = power.iter().sum();
    if total > 0.0 {
        for p in &mut power {
            *p /= total;
        }
    }
    power
}

fn percentile(values: &mut [f64], p: f64) -> f64 {
    if values.is_empty() {
        return f64::NAN;
    }
    values.sort_by(f64::total_cmp);
    values[((values.len() - 1) as f64 * p).round() as usize]
}

/// `v` minus its part in the row space of `j` (6 rows over `columns`): the null-space motion.
fn null_part(j: &[[f64; 6]], v: &[f64]) -> Vec<f64> {
    let n = v.len();
    let mut m = [[0.0; 7]; 6];
    for a in 0..6 {
        for b in 0..6 {
            m[a][b] =
                (0..n).map(|c| j[c][a] * j[c][b]).sum::<f64>() + if a == b { 1e-9 } else { 0.0 };
        }
        m[a][6] = (0..n).map(|c| j[c][a] * v[c]).sum();
    }
    // Gaussian elimination with partial pivoting.
    for col in 0..6 {
        let pivot = (col..6)
            .max_by(|x, y| m[*x][col].abs().total_cmp(&m[*y][col].abs()))
            .unwrap_or(col);
        m.swap(col, pivot);
        let d = m[col][col];
        if d.abs() < 1e-15 {
            continue;
        }
        for row in 0..6 {
            if row != col {
                let f = m[row][col] / d;
                for k in col..7 {
                    m[row][k] -= f * m[col][k];
                }
            }
        }
    }
    let y: Vec<f64> = (0..6)
        .map(|a| {
            if m[a][a].abs() < 1e-15 {
                0.0
            } else {
                m[a][6] / m[a][a]
            }
        })
        .collect();
    (0..n)
        .map(|c| v[c] - (0..6).map(|a| j[c][a] * y[a]).sum::<f64>())
        .collect()
}

fn sign_flips(series: &[f64], moving: &[bool], threshold: f64) -> usize {
    let mut flips = 0;
    let mut last = 0.0_f64;
    for (x, m) in series.iter().zip(moving) {
        if !m {
            last = 0.0;
            continue;
        }
        if x.abs() > threshold {
            if last != 0.0 && x.signum() != last.signum() {
                flips += 1;
            }
            last = *x;
        }
    }
    flips
}

/// `v` averaged over 9 samples (99 ms at the station's tick), centred: the hand's own motion
/// without tremor.
fn smooth(v: &[[f64; 3]]) -> Vec<[f64; 3]> {
    (0..v.len())
        .map(|i| {
            let lo = i.saturating_sub(4);
            let hi = (i + 5).min(v.len());
            [0, 1, 2].map(|k| v[lo..hi].iter().map(|x| x[k]).sum::<f64>() / (hi - lo) as f64)
        })
        .collect()
}

/// RMS distance, mm, of each moving tick's point from its centred average over `2 half + 1`
/// ticks of a contiguous stretch: what an onlooker sees as shaking, whatever its source. At
/// the station's 11 ms tick, `half` 4 keeps what is faster than about 5 Hz (tremor, chatter),
/// 22 what is faster than about 1 Hz (wobble, stop-and-go).
fn shake_mm(points: &[Option<[[f64; 3]; 2]>], moving: &[bool], half: usize) -> f64 {
    let (mut sum, mut count) = (0.0, 0usize);
    for i in half..points.len().saturating_sub(half) {
        if !moving[i] || points[i - half..=i + half].iter().any(Option::is_none) {
            continue;
        }
        for hand in HANDS {
            let mean = [0, 1, 2].map(|k| {
                points[i - half..=i + half]
                    .iter()
                    .map(|p| p.expect("checked")[hand][k])
                    .sum::<f64>()
                    / (2 * half + 1) as f64
            });
            let here = points[i].expect("checked")[hand];
            sum += (0..3).map(|k| (here[k] - mean[k]).powi(2)).sum::<f64>();
            count += 1;
        }
    }
    if count == 0 {
        f64::NAN
    } else {
        (sum / count as f64).sqrt() * 1000.0
    }
}

/// Velocity cross-correlation lag of `b` behind `a` on a uniform grid, ms.
fn lag_ms(a: &[[f64; 3]], b: &[[f64; 3]], period: f64) -> f64 {
    let max_shift = (0.6 / period) as usize;
    let mut best = (f64::NEG_INFINITY, 0);
    for shift in 0..max_shift.min(a.len() / 2) {
        let mut dot = 0.0;
        let (mut na, mut nb) = (0.0, 0.0);
        for i in shift..a.len().min(b.len()) {
            for k in 0..3 {
                dot += a[i - shift][k] * b[i][k];
                na += a[i - shift][k] * a[i - shift][k];
                nb += b[i][k] * b[i][k];
            }
        }
        let score = dot / (na.sqrt() * nb.sqrt() + 1e-12);
        if score > best.0 {
            best = (score, shift);
        }
    }
    best.1 as f64 * period * 1000.0
}

fn interpolate(series: &Series, t: f64, at: &mut usize) -> Option<Vec<f64>> {
    while *at + 1 < series.t.len() && series.t[*at + 1] <= t {
        *at += 1;
    }
    if *at + 1 >= series.t.len() || series.t[*at] > t {
        return None;
    }
    let (t0, t1) = (series.t[*at], series.t[*at + 1]);
    if t1 - t0 > 0.1 {
        return None;
    }
    let u = (t - t0) / (t1 - t0);
    Some(
        series.q[*at]
            .iter()
            .zip(&series.q[*at + 1])
            .map(|(a, b)| a + (b - a) * u)
            .collect(),
    )
}

/// One row: every measure of `commands` (and of what `extra` carries).
fn analyse(
    body: &Body,
    label: &str,
    commands: &Series,
    extra: &Extra,
    synthetic: Option<&[f64; 4]>,
) {
    let width = body.model().axes().len();
    let mut frames = body.model().poses(&vec![0.0; width]);
    let n = commands.t.len();
    // Ticks: periods, stalls followed by a bunch.
    let periods: Vec<f64> = commands
        .t
        .windows(2)
        .map(|w| (w[1] - w[0]) * 1000.0)
        .collect();
    let running: Vec<bool> = periods.iter().map(|p| *p < 60.0).collect();
    let mut sorted: Vec<f64> = periods.iter().copied().filter(|p| *p < 60.0).collect();
    let tick = percentile(&mut sorted, 0.5);
    // Tool points, velocities on the command grid (contiguous runs only).
    let tool: Vec<[[f64; 3]; 2]> = commands
        .q
        .iter()
        .map(|q| tools(body, &mut frames, q))
        .collect();
    let mut velocity = vec![[[0.0; 3]; 2]; n];
    for i in 1..n {
        if running[i - 1] {
            let dt = commands.t[i] - commands.t[i - 1];
            for hand in HANDS {
                for k in 0..3 {
                    velocity[i][hand][k] = (tool[i][hand][k] - tool[i - 1][hand][k]) / dt;
                }
            }
        }
    }
    let speed = |i: usize, hand: usize| velocity[i][hand].iter().map(|x| x * x).sum::<f64>().sqrt();
    let moving: Vec<bool> = (0..n)
        .map(|i| i > 0 && running[i - 1] && HANDS.iter().any(|h| speed(i, *h) > MOVING))
        .collect();
    let moving_s = moving.iter().filter(|m| **m).count() as f64 * tick / 1000.0;
    // Jerk: third differences of the tool point over consecutive running ticks.
    let mut jerk = Vec::new();
    for i in 3..n {
        if moving[i] && running[i - 3] && running[i - 2] && running[i - 1] {
            let dt = (commands.t[i] - commands.t[i - 3]) / 3.0;
            for hand in HANDS {
                let j: f64 = (0..3)
                    .map(|k| {
                        let x = tool[i][hand][k] - 3.0 * tool[i - 1][hand][k]
                            + 3.0 * tool[i - 2][hand][k]
                            - tool[i - 3][hand][k];
                        (x / dt.powi(3)).powi(2)
                    })
                    .sum();
                jerk.push(j);
            }
        }
    }
    for j in &mut jerk {
        *j = j.sqrt();
    }
    let jerk_p50 = percentile(&mut jerk.clone(), 0.5);
    let jerk_p95 = percentile(&mut jerk, 0.95);
    // Stop-and-go: a hand moving (> 20 mm/s) stands (< 2 mm/s) for 30 ms or more and moves on
    // within 0.5 s: the solve caught its target and waited for the next (an intent gap).
    let mut stop_go = 0_u32;
    for hand in HANDS {
        let (mut was_moving, mut still_since): (bool, Option<f64>) = (false, None);
        for i in 1..n {
            if !running[i - 1] {
                (was_moving, still_since) = (false, None);
                continue;
            }
            let v = speed(i, hand);
            if v > 0.02 {
                if let Some(since) = still_since
                    && was_moving
                {
                    let held = commands.t[i] - since;
                    if (STALL_MS / 1000.0..0.5).contains(&held) {
                        stop_go += 1;
                    }
                }
                (was_moving, still_since) = (true, None);
            } else if v < 0.002 && still_since.is_none() {
                still_since = Some(commands.t[i]);
            }
        }
    }
    // Spectrum of the moving hand's tool velocity, over moving stretches.
    let mut stretch: Vec<[f64; 3]> = Vec::new();
    let mut tool_bands = [0.0; 4];
    for hand in HANDS {
        stretch.clear();
        for i in 0..n {
            if moving[i] {
                stretch.push(velocity[i][hand]);
            } else if stretch.len() >= 128 {
                let b = bands(&stretch, 1000.0 / tick);
                (0..4).for_each(|k| tool_bands[k] += b[k]);
                stretch.clear();
            } else {
                stretch.clear();
            }
        }
        if stretch.len() >= 128 {
            let b = bands(&stretch, 1000.0 / tick);
            (0..4).for_each(|k| tool_bands[k] += b[k]);
        }
    }
    // Shares of both hands' summed power: a still hand's numerical noise weighs nothing.
    let tool_bands = shares(tool_bands);
    // Null space per hand: the hand's own arm joints that move in this series.
    let ever_moves: Vec<bool> = (0..width)
        .map(|a| commands.v.iter().any(|v| v[a].abs() > 1e-9))
        .collect();
    let axes = body.model().axes();
    let mut linear = vec![[0.0; 3]; width];
    let mut angular = vec![[0.0; 3]; width];
    let (mut null_sq, mut all_sq) = (0.0, 0.0);
    let mut worst = (0.0, String::from("-"));
    let mut reversals = 0;
    let mut reversing = (0.0, String::from("-"));
    for hand in HANDS {
        let columns: Vec<usize> = body
            .limb_axes(hand)
            .iter()
            .copied()
            .filter(|a| ever_moves[*a])
            .collect();
        let mut null_series = vec![vec![0.0; n]; columns.len()];
        for i in 0..n {
            if !moving[i] {
                continue;
            }
            body.model().poses_into(&commands.q[i], &mut frames);
            body.jacobian(&frames, hand, &mut linear, &mut angular);
            let j: Vec<[f64; 6]> = columns
                .iter()
                .map(|&a| {
                    [
                        linear[a][0],
                        linear[a][1],
                        linear[a][2],
                        angular[a][0],
                        angular[a][1],
                        angular[a][2],
                    ]
                })
                .collect();
            let v: Vec<f64> = columns.iter().map(|&a| commands.v[i][a]).collect();
            let null = null_part(&j, &v);
            null_sq += null.iter().map(|x| x * x).sum::<f64>();
            all_sq += v.iter().map(|x| x * x).sum::<f64>();
            for (c, x) in null.iter().enumerate() {
                null_series[c][i] = *x;
            }
        }
        for (c, &a) in columns.iter().enumerate() {
            let flips = sign_flips(&null_series[c], &moving, STILL) as f64 / moving_s.max(1e-9);
            if flips > worst.0 {
                worst = (flips, axes[a].name.replace("_joint", ""));
            }
            let v: Vec<f64> = commands.v.iter().map(|v| v[a]).collect();
            let flips = sign_flips(&v, &moving, STILL);
            reversals += flips;
            let rate = flips as f64 / moving_s.max(1e-9);
            if rate > reversing.0 {
                reversing = (rate, axes[a].name.replace("_joint", ""));
            }
        }
    }
    let null_share = if all_sq > 0.0 {
        (null_sq / all_sq).sqrt()
    } else {
        0.0
    };
    // Stalls: an interval over STALL_MS followed by three intervals under a third of a tick.
    let stalls = (0..periods.len().saturating_sub(3))
        .filter(|&i| {
            periods[i] > STALL_MS
                && periods[i] < 1000.0
                && periods[i + 1..i + 4].iter().all(|p| *p < tick / 3.0)
        })
        .count();
    let late = periods
        .iter()
        .filter(|p| **p > STALL_MS && **p < 1000.0)
        .count();
    // Tracking: the measured tool point at each command's time.
    let (mut track, mut lag) = (Vec::new(), f64::NAN);
    let mut real_tool: Vec<Option<[[f64; 3]; 2]>> = vec![None; n];
    if let Some(measured) = &extra.measured {
        let mut at = 0;
        let mut a = Vec::new();
        let mut b = Vec::new();
        let mut last: Option<[[f64; 3]; 2]> = None;
        for i in 0..n {
            let Some(q) = interpolate(measured, commands.t[i], &mut at) else {
                last = None;
                continue;
            };
            let real = tools(body, &mut frames, &q);
            real_tool[i] = Some(real);
            if moving[i] {
                for hand in HANDS {
                    track.push(
                        (0..3)
                            .map(|k| (real[hand][k] - tool[i][hand][k]).powi(2))
                            .sum::<f64>()
                            .sqrt(),
                    );
                }
                if let Some(prev) = last {
                    let dt = commands.t[i] - commands.t[i - 1];
                    let hand = usize::from(speed(i, 1) > speed(i, 0));
                    a.push(velocity[i][hand]);
                    b.push([0, 1, 2].map(|k| (real[hand][k] - prev[hand][k]) / dt));
                }
            }
            last = Some(real);
        }
        if a.len() > 100 {
            lag = lag_ms(&smooth(&a), &smooth(&b), tick / 1000.0);
        }
    }
    let commanded_points: Vec<Option<[[f64; 3]; 2]>> = tool.iter().copied().map(Some).collect();
    let shake = format!(
        "{:.2}/{:.2} {:.1}/{:.1}",
        shake_mm(&commanded_points, &moving, 4),
        shake_mm(&real_tool, &moving, 4),
        shake_mm(&commanded_points, &moving, 22),
        shake_mm(&real_tool, &moving, 22)
    );
    let track_p50 = percentile(&mut track.clone(), 0.5) * 1000.0;
    let track_p95 = percentile(&mut track, 0.95) * 1000.0;
    print!(
        "{label:22} {moving_s:6.1} {tick:6.2} {:>6} {:>19} {:>12} {:>16} {:5.2} {:>18} {:>26} {:7.1} {:>11} {:>6}",
        format!("{late}/{stalls}"),
        shake,
        format!("{jerk_p50:.0}/{jerk_p95:.0}"),
        format!(
            "{:.2}/{:.2}/{:.2}/{:.2}",
            tool_bands[0], tool_bands[1], tool_bands[2], tool_bands[3]
        ),
        null_share,
        format!(
            "{:.1} {}",
            worst.0,
            worst.1.replace("left_", "L ").replace("right_", "R ")
        ),
        format!(
            "{:.1} ({} {:.1})",
            reversals as f64 / moving_s.max(1e-9),
            reversing.1.replace("left_", "L ").replace("right_", "R "),
            reversing.0
        ),
        f64::from(stop_go) / (moving_s / 60.0).max(1e-9),
        format!("{track_p50:.1}/{track_p95:.1}"),
        if lag.is_nan() {
            "-".to_owned()
        } else {
            format!("{lag:.0}")
        },
    );
    if let Some(target_bands) = synthetic {
        let ticks = n as f64;
        print!(
            " {:>16} {:5} {:5} {:5.1}% {:5.1}% {:5}",
            format!(
                "{:.2}/{:.2}/{:.2}/{:.2}",
                target_bands[0], target_bands[1], target_bands[2], target_bands[3]
            ),
            extra.held,
            extra.relaxed,
            100.0 * extra.speed_bound as f64 / ticks,
            100.0 * extra.accel_bound as f64 / ticks,
            extra.reseeds
        );
    }
    println!();
}

/// A small deterministic generator: the runs repeat exactly.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn exponential(&mut self, mean: f64) -> f64 {
        -mean * (1.0 - self.next()).ln()
    }
}

fn governed(name: &str) -> (f64, f64) {
    // robot-bench "How fast the robot moves": rated, through the gains' ceiling (2026-11-02).
    let n = name;
    if n.starts_with("so101_") || n.contains("finger") {
        (4.0, 16.0)
    } else if n == "torso_joint" {
        (0.25, 0.3)
    } else if n.contains("shoulder_pitch") || n == "left_shoulder_roll_joint" {
        (1.2, 13.5)
    } else if n == "right_shoulder_roll_joint" {
        (0.99, 9.9)
    } else if n == "left_upper_arm_yaw_joint" {
        (0.96, 15.4)
    } else if n.contains("wrist_pitch") {
        (3.22, 90.0)
    } else if n.contains("wrist_roll") {
        (4.0, 126.0)
    } else {
        (1.25, 19.5)
    }
}

#[derive(Clone, Copy)]
struct Scenario {
    name: &'static str,
    tremor_m: f64,
    tremor_rad: f64,
    network: bool,
    jump: bool,
    /// Reading noise, rad (uniform +-).
    noise: f64,
}

fn synth(body: &Body, caps: Option<&str>, tick: Option<&String>) {
    let dt: f64 = tick.and_then(|t| t.parse().ok()).unwrap_or(0.011);
    let flat: Option<f64> = caps.and_then(|c| c.parse().ok());
    let names: Vec<String> = body.model().axes().iter().map(|a| a.name.clone()).collect();
    let mut speed = BTreeMap::new();
    let mut accel = BTreeMap::new();
    for name in &names {
        let (mut s, a) = governed(name);
        if let Some(cap) = flat
            && is_arm(name)
        {
            s = cap;
        }
        // The station: acceleration at most the speed reached in RAMP_S (0.1 s).
        speed.insert(name.clone(), s);
        accel.insert(name.clone(), a.min(s / 0.1));
    }
    let max_speed = speed.values().copied().fold(0.0, f64::max);
    let config = TeleopConfig {
        schema: TELEOP_SCHEMA.into(),
        limbs: TeleopConfig::static_upper_body_limbs(),
        speed,
        accel,
        ramp_s: 0.1,
        max_step_rad: (max_speed * dt).clamp(0.03, 0.08),
        held: vec!["torso_joint".into()],
        obstacles: Vec::new(),
        posture: BTreeMap::new(),
    };
    println!(
        "caps: {}, tick {:.1} ms, max step {:.3} rad\n",
        caps.unwrap_or("governed"),
        dt * 1000.0,
        config.max_step_rad
    );
    header(true);
    let quiet = Scenario {
        name: "exact readings",
        tremor_m: 0.0,
        tremor_rad: 0.0,
        network: false,
        jump: false,
        noise: 0.0,
    };
    let clean = Scenario {
        name: "clean",
        noise: 0.0005,
        ..quiet
    };
    let tremor = Scenario {
        name: "tremor 2mm",
        tremor_m: 0.002,
        tremor_rad: 0.3_f64.to_radians(),
        ..clean
    };
    let scenarios = [
        quiet,
        clean,
        tremor,
        Scenario {
            name: "tremor 1mm",
            tremor_m: 0.001,
            tremor_rad: 0.15_f64.to_radians(),
            ..clean
        },
        Scenario {
            name: "network",
            network: true,
            ..clean
        },
        Scenario {
            name: "tremor+network",
            network: true,
            ..tremor
        },
        Scenario {
            name: "jump 10cm",
            jump: true,
            ..clean
        },
    ];
    for scenario in scenarios {
        run(body, &config, dt, scenario);
    }
}

fn start_pose(solve: &Teleop, body: &Body) -> Vec<f64> {
    // The bench's working pose: the clearest of a grid of symmetric arm poses, hands in front
    // (the step_response example's).
    let index = |name: &str| {
        body.model()
            .axes()
            .iter()
            .position(|a| a.name == name)
            .expect(name)
    };
    let rest = solve.ik().rest().to_vec();
    let mut best = (f64::NEG_INFINITY, rest.clone());
    for pitch in [-1.2, -0.9, -0.6, -0.3, 0.0] {
        for elbow in [0.4, 0.8, 1.2, 1.6] {
            for roll in [0.0, 0.2, 0.4] {
                let mut c = rest.clone();
                c[index("left_shoulder_pitch_joint")] = pitch;
                c[index("right_shoulder_pitch_joint")] = pitch;
                c[index("left_shoulder_roll_joint")] = roll;
                c[index("right_shoulder_roll_joint")] = -roll;
                c[index("left_elbow_pitch_joint")] = -elbow;
                c[index("right_elbow_pitch_joint")] = elbow;
                let clear = solve.ik().clearance(&c).0;
                if clear > best.0 {
                    best = (clear, c);
                }
            }
        }
    }
    best.1
}

fn small_turn(v: [f64; 3]) -> Quaternion {
    let angle = v.iter().map(|x| x * x).sum::<f64>().sqrt();
    if angle < 1e-12 {
        return Quaternion::IDENTITY;
    }
    let s = (angle / 2.0).sin() / angle;
    Quaternion {
        x: v[0] * s,
        y: v[1] * s,
        z: v[2] * s,
        w: (angle / 2.0).cos(),
    }
}

fn run(body: &Body, config: &TeleopConfig, dt: f64, scenario: Scenario) {
    const SECONDS: f64 = 24.0;
    const HEADSET_HZ: f64 = 90.0;
    const LATENCY_S: f64 = 0.145;
    const LAG_TICKS: usize = 4;
    let mut solve =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, config).expect("solve");
    solve.set_deadline(std::time::Duration::from_secs(1));
    let width = body.model().axes().len();
    let q0 = start_pose(&solve, body);
    let mut frames = body.model().poses(&q0);
    let start = HANDS.map(|hand| body.frame(&frames, hand).pose());
    let mut rng = Rng(0x9e37_79b9_7f4a_7c15);
    // Tremor: three sines per axis between 8 and 12 Hz, random phases.
    let mut sines = [[(0.0, 0.0); 3]; 6];
    for axis in &mut sines {
        for s in axis.iter_mut() {
            *s = (
                8.0 + 4.0 * rng.next(),
                2.0 * std::f64::consts::PI * rng.next(),
            );
        }
    }
    let tremor = |t: f64, axis: usize| -> f64 {
        sines[axis]
            .iter()
            .map(|(f, p)| (2.0 * std::f64::consts::PI * f * t + p).sin())
            .sum::<f64>()
            / 3.0_f64.sqrt()
    };
    // The headset's samples: (send time, arrival time, targets, smooth hands).
    let mut packets: Vec<(f64, f64, Targets, [[f64; 3]; 2])> = Vec::new();
    let mut stall_until = 0.0;
    let samples = (SECONDS * HEADSET_HZ) as usize;
    for i in 0..samples {
        let t = i as f64 / HEADSET_HZ;
        let mut hands = [[0.0; 3]; 2];
        let mut targets = Targets::default();
        for hand in HANDS {
            let mut p = start[hand].translation_m;
            if hand == 0 {
                if scenario.jump {
                    if t >= 2.0 {
                        p[2] += 0.10;
                    }
                } else {
                    let phase = 2.0 * std::f64::consts::PI * (t - 2.0).max(0.0) / 4.0;
                    p[1] += 0.04 * (1.0 - phase.cos());
                    p[2] += 0.025 * phase.sin();
                }
            }
            hands[hand] = p;
            let offset = 3 * hand;
            let shaken = [0, 1, 2].map(|k| p[k] + scenario.tremor_m * tremor(t, k + offset));
            let turn = small_turn(
                [0, 1, 2].map(|k| scenario.tremor_rad * tremor(t + 0.37, (k + offset + 1) % 6)),
            );
            targets.tcp[hand] = Some(Pose {
                translation_m: shaken,
                rotation: turn * start[hand].rotation,
            });
        }
        let mut arrival = t + LATENCY_S;
        if scenario.network {
            if rng.next() < 0.01 {
                continue;
            }
            if rng.next() < 0.5 / HEADSET_HZ {
                stall_until = t + LATENCY_S + 0.03 + 0.12 * rng.next();
            }
            arrival += rng.exponential(0.012);
            arrival = arrival.max(stall_until);
        }
        packets.push((t, arrival, targets, hands));
    }
    // The station: every tick takes the newest intent that has arrived (by sequence).
    let mut q = q0.clone();
    let mut history: Vec<Vec<f64>> = vec![q0.clone(); LAG_TICKS + 1];
    let available = vec![true; width];
    let mut commands = Series {
        t: Vec::new(),
        q: Vec::new(),
        v: Vec::new(),
    };
    let mut measured = Series {
        t: Vec::new(),
        q: Vec::new(),
        v: Vec::new(),
    };
    let mut extra = Extra::default();
    let mut newest: Option<usize> = None;
    let mut arrived_targets: Vec<[[f64; 3]; 2]> = Vec::new();
    let mut jump_closed: Option<f64> = None;
    solve.reset();
    let ticks = (SECONDS / dt) as usize;
    let mut used_arrival = Vec::new();
    let names: Vec<String> = body.model().axes().iter().map(|a| a.name.clone()).collect();
    let mut accel_by: BTreeMap<String, usize> = BTreeMap::new();
    let mut follow_error = Vec::new();
    for k in 0..ticks {
        let now = LATENCY_S + k as f64 * dt;
        for (index, packet) in packets.iter().enumerate() {
            if packet.1 <= now && newest.is_none_or(|n| index > n) {
                newest = Some(index);
            }
        }
        let Some(current) = newest else { continue };
        let (sent, arrival, targets, smooth) = packets[current];
        if now - arrival > 0.25 {
            continue; // stale: the station holds (counted by the tick histogram)
        }
        extra.transport.push(arrival - sent);
        used_arrival.push(now - arrival);
        let lagging = &history[0];
        let reading: Vec<Option<f64>> = lagging
            .iter()
            .map(|x| Some(x + scenario.noise * (2.0 * rng.next() - 1.0)))
            .collect();
        if q.iter().zip(&reading).enumerate().any(|(a, (s, r))| {
            is_arm(&body.model().axes()[a].name) && r.is_some_and(|r| (s - r).abs() > REFERENCE_GAP)
        }) {
            extra.reseeds += 1;
        }
        let step = solve
            .step(&q, &reading, &available, &targets, dt)
            .expect("a step");
        match step.solve {
            Solve::Held => extra.held += 1,
            Solve::Relaxed => extra.relaxed += 1,
            Solve::Solved => {}
        }
        let actives = step.limbs.iter().flat_map(|l| l.active.iter());
        let (mut s, mut a) = (false, false);
        for active in actives {
            match active {
                Active::Speed { axis } if is_arm(&names[*axis]) => s = true,
                Active::Accel { axis } if is_arm(&names[*axis]) => {
                    a = true;
                    *accel_by
                        .entry(names[*axis].replace("_joint", ""))
                        .or_insert(0) += 1;
                }
                _ => {}
            }
        }
        extra.speed_bound += usize::from(s);
        extra.accel_bound += usize::from(a);
        let step_q = step.q.clone();
        q.copy_from_slice(&step.q);
        commands.t.push(now);
        commands.q.push(step.q.clone());
        commands.v.push(step.qdot.clone());
        measured.t.push(now);
        measured.q.push(lagging.clone());
        history.remove(0);
        history.push(step.q.clone());
        let target = targets.tcp.map(|p| p.map_or([0.0; 3], |p| p.translation_m));
        arrived_targets.push(target);
        let at = tools(body, &mut frames, &step_q)[0];
        follow_error.push(
            (0..3)
                .map(|k| (at[k] - smooth[0][k]).powi(2))
                .sum::<f64>()
                .sqrt(),
        );
        if scenario.jump && sent >= 2.0 && jump_closed.is_none() {
            let at = tools(body, &mut frames, &step.q)[0];
            let error = (0..3)
                .map(|k| (at[k] - target[0][k]).powi(2))
                .sum::<f64>()
                .sqrt();
            if error < 0.005 {
                jump_closed = Some(now - (2.0 + LATENCY_S));
            }
        }
    }
    // The target as the solve received it: its spectrum, and the command's lag behind it.
    let mut target_velocity = Vec::new();
    for w in arrived_targets.windows(2) {
        target_velocity.push([0, 1, 2].map(|k| (w[1][0][k] - w[0][0][k]) / dt));
    }
    let target_bands = shares(bands(&target_velocity, 1.0 / dt));
    extra.queue = used_arrival;
    extra.targets = arrived_targets;
    extra.measured = Some(measured);
    analyse(body, scenario.name, &commands, &extra, Some(&target_bands));
    // Felt latency: target (as received) -> command, by cross-correlating velocities smoothed
    // over 99 ms, so tremor does not decide the lag of the hand's own motion.
    let mut command_velocity = Vec::new();
    for w in commands.q.windows(2) {
        let a = tools(body, &mut frames, &w[0])[0];
        let b = tools(body, &mut frames, &w[1])[0];
        command_velocity.push([0, 1, 2].map(|k| (b[k] - a[k]) / dt));
    }
    let length = target_velocity.len().min(command_velocity.len());
    let solve_lag = lag_ms(
        &smooth(&target_velocity[..length]),
        &smooth(&command_velocity[..length]),
        dt,
    );
    // Intents as they arrived: inter-arrival histogram, gaps over STALL_MS and the bunch after.
    let mut arrivals: Vec<f64> = packets.iter().map(|p| p.1).collect();
    arrivals.sort_by(f64::total_cmp);
    let gaps: Vec<f64> = arrivals
        .windows(2)
        .map(|w| (w[1] - w[0]) * 1000.0)
        .collect();
    let edges = [5.0, 9.0, 13.0, 20.0, 30.0, 60.0];
    let mut histogram = [0usize; 7];
    for g in &gaps {
        histogram[edges.iter().position(|e| g < e).unwrap_or(6)] += 1;
    }
    let bunched = (0..gaps.len().saturating_sub(2))
        .filter(|&i| gaps[i] > STALL_MS && gaps[i + 1] < 1.0 && gaps[i + 2] < 1.0)
        .count();
    let minutes = SECONDS / 60.0;
    let mut top: Vec<(&String, &usize)> = accel_by.iter().collect();
    top.sort_by(|a, b| b.1.cmp(a.1));
    let follow50 = percentile(&mut follow_error.clone(), 0.5) * 1000.0;
    let follow95 = percentile(&mut follow_error, 0.95) * 1000.0;
    println!(
        "  {:20} intents <5/5-9/9-13/13-20/20-30/30-60/60+ ms: {histogram:?}; gaps > 30 ms {:.0}/min, bunched {:.0}/min",
        "",
        gaps.iter().filter(|g| **g > STALL_MS).count() as f64 / minutes,
        bunched as f64 / minutes
    );
    println!(
        "  {:20} commanded left tool from the hand's smooth path (as received): p50 {follow50:.1} mm, p95 {follow95:.1} mm; \
         accel cap binds most on {}",
        "",
        top.iter()
            .take(3)
            .map(|(n, c)| format!(
                "{} {:.0}%",
                n.replace("left_", "L ").replace("right_", "R "),
                100.0 * **c as f64 / commands.t.len() as f64
            ))
            .collect::<Vec<_>>()
            .join(", ")
    );
    let transport = percentile(&mut extra.transport.clone(), 0.5) * 1000.0;
    let transport95 = percentile(&mut extra.transport, 0.95) * 1000.0;
    let queue = percentile(&mut extra.queue.clone(), 0.5) * 1000.0;
    let queue95 = percentile(&mut extra.queue, 0.95) * 1000.0;
    println!(
        "  {:20} felt latency: transport {transport:.0}/{transport95:.0} ms (p50/p95), tick queue {queue:.0}/{queue95:.0} ms, \
         solve {solve_lag:.0} ms, follower {} ms{}",
        "",
        (LAG_TICKS as f64 * dt * 1000.0).round(),
        jump_closed.map_or_else(String::new, |s| format!(
            ", 10 cm jump within 5 mm after {:.0} ms",
            s * 1000.0
        ))
    );
}
