//! How every motion begins and ends through the teleoperation solve: the before/after for any
//! change to how a command starts from rest and comes back to it. The static upper body from
//! its commissioned Ready (`tests/fixtures/static-ready.json`), an ideal follower (each step
//! measured where it was commanded), 100 Hz:
//!
//! - **engage**: the solve reset and the left hand's first target 10 cm or 30 cm from where it
//!   stands, as an operator's jump (no tool speed) and as a program's first row (at a tool
//!   speed): peak joint acceleration and jerk in the first 0.5 s, each as a share of the joint's
//!   cap, and the time to within 5 mm;
//! - **plan**: a program's 20 cm goal at a tool speed: peak acceleration and jerk in its first
//!   0.5 s and in the 0.5 s around its tool's arrival within 5 mm of the goal;
//! - **release**: the hand sweeping at 0.5 m/s, then no target (the operator lets go): peak
//!   acceleration and jerk in the next 0.5 s, how far the tool went on and how long it took to
//!   stand still;
//! - **jump**: a 10 cm jump mid-session (engaged and at rest for 1 s) along each of +-x, +-y,
//!   +-z: mean and worst time to within 5 mm.
//!
//! A joint's jerk cap is the config's (`jerk`, `TeleopConfig::jerk`), else its acceleration over
//! the config's `ramp_s`: the solve shapes each hand's target within it (`sx_embodiment_ik::shaper`).
//! Acceleration and jerk are the commanded positions differenced, from rest at engage. Run the
//! base revision's binary for the before.
//!
//! ```text
//! cargo run --release -p sx-embodiment-ik --example transitions -- [fixture|governed|station] [trace.json]
//! ```
//!
//! `fixture` is the station's composed config the film uses (`tests/fixtures/static-teleop-config.json`,
//! rated caps); `governed` is the static upper body's governed table (robot-bench, "How fast the
//! robot moves": rated, held to the commissioned MIT gains' ceilings), with each Damiao joint's
//! jerk its acceleration times its gains' natural frequency `sqrt(kp / J)`; `station` is that
//! with the torso held, as the resident station runs it (`HELD_UNTIL_IDENTIFIED`). With a path, the
//! engage-and-release scenario the film shows is written there as `examples/trace.rs` writes
//! its trace: 30 cm first target, a sweep, a release, and a re-engage.

#![allow(
    clippy::too_many_lines,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::many_single_char_names,
    clippy::needless_range_loop
)]

use std::collections::BTreeMap;

use serde_json::{Value, json};
use sx_embodiment_ik::teleop::{Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF, ToolSpeed};
use sx_frames::{Pose, QuaternionLayout};

const DT: f64 = 0.01;
const WINDOW_S: f64 = 0.5;
const BINDINGS: &str = include_str!("../../sx-damiao-can/bindings/sentient-static-upper-body.json");

/// Each Damiao joint's commissioned MIT gains (`sx-damiao-can` `STATIC_UPPER_BODY`).
const GAINS: [(&str, f64, f64); 15] = [
    ("torso_joint", 40.0, 1.0),
    ("left_shoulder_pitch_joint", 60.984, 1.091),
    ("left_shoulder_roll_joint", 60.984, 1.091),
    ("left_upper_arm_yaw_joint", 24.0, 5.0),
    ("left_elbow_pitch_joint", 30.492, 4.655),
    ("left_forearm_yaw_joint", 29.04, 4.75),
    ("left_wrist_pitch_joint", 20.0, 1.0),
    ("left_wrist_roll_joint", 20.0, 1.0),
    ("right_shoulder_pitch_joint", 60.984, 1.091),
    ("right_shoulder_roll_joint", 42.0, 0.98),
    ("right_upper_arm_yaw_joint", 30.492, 4.655),
    ("right_elbow_pitch_joint", 30.492, 4.655),
    ("right_forearm_yaw_joint", 30.492, 4.655),
    ("right_wrist_pitch_joint", 20.0, 1.0),
    ("right_wrist_roll_joint", 20.0, 1.0),
];

fn peak_torque(model: &str) -> f64 {
    match model {
        "damiao_dm8009p" => 40.0,
        "damiao_dm4340" => 27.0,
        "damiao_dm4310" => 7.0,
        other => panic!("no peak torque for {other}"),
    }
}

/// The fixture's config, or the governed one (speed, acceleration and, for a Damiao joint, the
/// jerk its gains' natural frequency allows), as JSON.
fn config_json(which: &str) -> Value {
    let mut config: Value =
        serde_json::from_str(include_str!("../tests/fixtures/static-teleop-config.json"))
            .expect("the station's solve config");
    if which == "fixture" {
        return config;
    }
    if which == "station" {
        // The station holds the torso until it is identified (`HELD_UNTIL_IDENTIFIED`).
        config["held"] = json!(["torso_joint"]);
    }
    let bindings: Value = serde_json::from_str(BINDINGS).expect("bindings");
    let mut jerk = serde_json::Map::new();
    for axis in bindings["axes"].as_array().expect("axes") {
        let joint = axis["joint"].as_str().expect("joint");
        let speed = axis["speed"].as_f64().expect("speed");
        let accel = axis["acceleration"].as_f64().expect("acceleration");
        let (speed, accel) = match GAINS.iter().find(|(name, ..)| *name == joint) {
            Some(&(_, kp, kd)) => {
                let j = axis["inertia"].as_f64().expect("inertia");
                let model = axis["model"].as_str().expect("model");
                let ceiling = (0.2 * kp / kd)
                    .min(0.1 * (kp / j).sqrt())
                    .min((0.2 * peak_torque(model) / j).sqrt());
                let accel = accel.min(0.1 * kp / j);
                jerk.insert(joint.to_string(), json!(accel * (kp / j).sqrt()));
                (speed.min(ceiling), accel)
            }
            None => (speed.min(4.0), accel.min(16.0)),
        };
        config["speed"][joint] = json!(speed);
        config["accel"][joint] = json!(accel);
    }
    config["jerk"] = Value::Object(jerk);
    config
}

/// The config the solve takes, and as JSON.
fn config(which: &str) -> (TeleopConfig, Value) {
    let json = config_json(which);
    (
        TeleopConfig::parse(&json.to_string()).expect("the station's solve config"),
        json,
    )
}

struct Bench {
    solve: Teleop,
    names: Vec<String>,
    ready: Vec<f64>,
    /// Per axis: (speed, acceleration, jerk) caps; the arm joints measured.
    caps: Vec<(f64, f64, f64)>,
    arm: Vec<usize>,
}

impl Bench {
    fn new(which: &str) -> Self {
        let (config, json) = config(which);
        let ready: BTreeMap<String, f64> =
            serde_json::from_str(include_str!("../tests/fixtures/static-ready.json"))
                .expect("ready");
        let mut solve = Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config)
            .expect("solve");
        solve.set_deadline(std::time::Duration::from_secs(3600));
        let names: Vec<String> = solve
            .model()
            .axes()
            .iter()
            .map(|a| a.name.clone())
            .collect();
        let ramp = json["ramp_s"].as_f64().expect("ramp");
        let caps = names
            .iter()
            .map(|n| {
                let s = json["speed"][n].as_f64().unwrap_or(0.0);
                let a = json["accel"][n].as_f64().unwrap_or(s / ramp);
                let j = json["jerk"][n].as_f64().unwrap_or(a / ramp);
                (s, a, j)
            })
            .collect();
        let arm = names
            .iter()
            .enumerate()
            .filter(|(_, n)| {
                (n.starts_with("left_")
                    || n.starts_with("right_")
                    || n.starts_with("so101_")
                    || n.starts_with("torso"))
                    && !n.contains("gripper")
            })
            .map(|(i, _)| i)
            .collect();
        let ready = names
            .iter()
            .map(|n| ready.get(n).copied().unwrap_or(0.0))
            .collect();
        Self {
            solve,
            names,
            ready,
            caps,
            arm,
        }
    }

    fn tools(&self, q: &[f64]) -> [Pose; 2] {
        let body = self.solve.ik().body();
        let frames = body.model().poses(q);
        [body.frame(&frames, 0).pose(), body.frame(&frames, 1).pose()]
    }
}

/// One run's commanded positions per tick, from `q0` at rest; `ask(t, tools at q0)` gives the
/// targets at each tick, `None` to reset (engage) at that tick first.
struct Run {
    q: Vec<Vec<f64>>,
    left: Vec<[f64; 3]>,
    target: Vec<Option<[f64; 3]>>,
}

fn run(
    bench: &mut Bench,
    seconds: f64,
    mut ask: impl FnMut(f64, &[Pose; 2]) -> (Targets, bool),
) -> Run {
    let mut q = bench.ready.clone();
    let from = bench.tools(&q);
    let available = vec![true; q.len()];
    let mut out = Run {
        q: vec![q.clone()],
        left: vec![from[0].translation_m],
        target: vec![None],
    };
    let count = (seconds / DT).round() as usize;
    for tick in 1..=count {
        let t = tick as f64 * DT;
        let (targets, engage) = ask(t, &from);
        if engage {
            bench.solve.reset();
        }
        let measured: Vec<Option<f64>> = q.iter().map(|v| Some(*v)).collect();
        let step = bench
            .solve
            .step(&q, &measured, &available, &targets, DT)
            .expect("a step");
        if std::env::var("SX_DEBUG").is_ok() {
            eprintln!("ACT {t:.2} {:?} {:?}", step.solve, step.limbs[0].active);
        }
        q.copy_from_slice(&step.q);
        let solved = step.solve;
        if std::env::var("SX_DEBUG_RUN").is_ok() {
            let shaped = bench.solve.shaped()[0].map_or([f64::NAN; 3], |p| p.translation_m);
            let tool = bench.tools(&q)[0].translation_m;
            let target = targets.tcp[0].map_or([f64::NAN; 3], |p| p.translation_m);
            eprintln!(
                "RUN {t:.2} target {:+.4} shaped {:+.4} tool {:+.4} {:?}",
                target[1] - from[0].translation_m[1],
                shaped[1] - from[0].translation_m[1],
                tool[1] - from[0].translation_m[1],
                solved
            );
        }
        out.left.push(bench.tools(&q)[0].translation_m);
        out.target.push(targets.tcp[0].map(|p| p.translation_m));
        out.q.push(q.clone());
    }
    out
}

/// Per tick from 1: each arm joint's commanded acceleration and jerk as a share of its cap,
/// the largest of them, and the largest absolute jerk (rad/s^3).
struct Peaks {
    accel_share: f64,
    jerk_share: f64,
    jerk: f64,
    accel_joint: String,
    jerk_joint: String,
}

fn peaks_of(bench: &Bench, axes: &[usize], run: &Run, from_s: f64, to_s: f64) -> Peaks {
    let mut p = Peaks {
        accel_share: 0.0,
        jerk_share: 0.0,
        jerk: 0.0,
        accel_joint: String::new(),
        jerk_joint: String::new(),
    };
    let n = run.q.len();
    for &axis in axes {
        let (_, a_cap, j_cap) = bench.caps[axis];
        // Velocity, acceleration, from rest before tick 0.
        let mut v_prev = 0.0;
        let mut a_prev = 0.0;
        for k in 1..n {
            let v = (run.q[k][axis] - run.q[k - 1][axis]) / DT;
            let a = (v - v_prev) / DT;
            let j = (a - a_prev) / DT;
            let t = k as f64 * DT;
            if std::env::var("SX_DEBUG_JERK").is_ok()
                && j.abs() / j_cap > 1.05
                && (t <= 0.6 || std::env::var("SX_DEBUG_JERK").as_deref() == Ok("all"))
            {
                eprintln!(
                    "JERK {t:.2} {} {:.1}x a {:.2} v {:.3}",
                    bench.names[axis],
                    j.abs() / j_cap,
                    a,
                    v
                );
            }
            if t > from_s && t <= to_s + 1e-9 {
                if a.abs() / a_cap > p.accel_share {
                    p.accel_share = a.abs() / a_cap;
                    p.accel_joint.clone_from(&bench.names[axis]);
                }
                if j.abs() / j_cap > p.jerk_share {
                    p.jerk_share = j.abs() / j_cap;
                    p.jerk_joint.clone_from(&bench.names[axis]);
                }
                p.jerk = p.jerk.max(j.abs());
            }
            v_prev = v;
            a_prev = a;
        }
    }
    p
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// The first time from `after_s` the left tool stays within `limit` of its target.
fn settled(run: &Run, after_s: f64, limit: f64) -> Option<f64> {
    let start = (after_s / DT).round() as usize;
    let mut since = None;
    for k in start..run.q.len() {
        match run.target[k] {
            Some(target) if distance(run.left[k], target) <= limit => {
                since.get_or_insert(k);
            }
            _ => since = None,
        }
    }
    since.map(|k| k as f64 * DT - after_s)
}

fn moved(from: &Pose, by: [f64; 3]) -> Pose {
    let mut p = *from;
    for k in 0..3 {
        p.translation_m[k] += by[k];
    }
    p
}

/// The arms' and torso's peaks, then the neck's, on one line.
fn split(bench: &Bench, label: &str, run: &Run, from_s: f64, to_s: f64, extra: &str) {
    let (arms, neck): (Vec<usize>, Vec<usize>) = bench
        .arm
        .iter()
        .partition(|&&a| !bench.names[a].starts_with("so101_"));
    let a = peaks_of(bench, &arms, run, from_s, to_s);
    let n = peaks_of(bench, &neck, run, from_s, to_s);
    println!(
        "{label:<26} arms accel {:>5.2}x jerk {:>5.2}x ({:<26}) | neck accel {:>5.2}x jerk {:>5.2}x  {extra}",
        a.accel_share, a.jerk_share, a.jerk_joint, n.accel_share, n.jerk_share
    );
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let which = args.first().map_or("fixture", String::as_str);
    let trace = args.get(1);
    let mut bench = Bench::new(which);
    println!("caps: {which}; window {WINDOW_S} s; shares of each joint's cap");
    // A program's tool speed: the order of the robot's served cap at Ready.
    let program = Some(ToolSpeed {
        m_s: 0.5,
        rad_s: 3.0,
    });
    // Engage with a far first target: as an operator's jump and as a program's first row.
    for (label, offset) in [
        ("0 cm", [0.0, 0.0, 0.0]),
        ("10 cm", [0.0, 0.10, 0.0]),
        ("30 cm", [0.10, 0.10, 0.2645]),
    ] {
        for (who, speed) in [("operator", None), ("program", program)] {
            let r = run(&mut bench, 2.5, |t, from| {
                let targets = Targets {
                    tcp: [Some(moved(&from[0], offset)), Some(from[1])],
                    tool_speed: [speed, speed],
                    ..Targets::default()
                };
                (targets, t <= DT + 1e-9)
            });
            if std::env::var("SX_DEBUG").is_ok() {
                for k in (0..r.q.len()).step_by(10) {
                    eprintln!(
                        "{:.2} {:.1} mm",
                        k as f64 * DT,
                        distance(r.left[k], r.target[k].unwrap_or(r.left[k])) * 1e3
                    );
                }
            }
            let to5 = settled(&r, 0.0, 0.005).map_or("never".into(), |s| format!("{s:.2} s"));
            split(
                &bench,
                &format!("engage {label} {who}"),
                &r,
                0.0,
                WINDOW_S,
                &format!("to 5 mm {to5}"),
            );
        }
    }
    // A program's goal: its start and its end.
    let r = run(&mut bench, 3.0, |t, from| {
        let targets = Targets {
            tcp: [Some(moved(&from[0], [0.0, 0.2, 0.0])), Some(from[1])],
            tool_speed: [program, program],
            ..Targets::default()
        };
        (targets, t <= DT + 1e-9)
    });
    // Its end: the 0.5 s around the tool's arrival within 5 mm of the goal (where it sits a
    // millimetre or two off, its posture weighed against it).
    let arrived = settled(&r, 0.0, 0.005).unwrap_or(3.0);
    if std::env::var("SX_DEBUG_PLAN").is_ok() {
        for k in (0..r.q.len()).step_by(5) {
            eprintln!(
                "PLAN {:.2} {:.1} mm",
                k as f64 * DT,
                distance(r.left[k], r.target[k].unwrap_or(r.left[k])) * 1e3
            );
        }
    }
    split(&bench, "plan start", &r, 0.0, WINDOW_S, "");
    split(
        &bench,
        "plan end",
        &r,
        arrived - WINDOW_S / 2.0,
        arrived + WINDOW_S / 2.0,
        &format!("within 5 mm at {arrived:.2} s"),
    );
    // Release mid-sweep: engaged at rest, a sweep from 0.5 s, let go at 0.7 s: at 0.2 m/s, within
    // the joints' line speed on the governed caps, and at 0.5 m/s, past it (joints at their speed
    // caps, as main holds them).
    for sweep in [0.2, 0.5] {
        let release_s = 0.7;
        let r = run(&mut bench, 2.0, |t, from| {
            let along = sweep * (t - 0.5).clamp(0.0, 1.0);
            let targets = if t < release_s {
                Targets {
                    tcp: [Some(moved(&from[0], [0.0, along, 0.0])), Some(from[1])],
                    ..Targets::default()
                }
            } else {
                Targets::default()
            };
            (targets, t <= DT + 1e-9)
        });
        let k0 = (release_s / DT).round() as usize - 1;
        // The released arm's own joints (the right hand stays driven where it stands).
        let left: Vec<usize> = bench
            .arm
            .iter()
            .copied()
            .filter(|&a| bench.names[a].starts_with("left_"))
            .collect();
        let rest = (k0 + 1..r.q.len())
            .find(|&k| {
                left.iter()
                    .all(|&a| (r.q[k][a] - r.q[k - 1][a]).abs() <= 1e-9)
            })
            .map_or(f64::NAN, |k| (k - 1 - k0) as f64 * DT);
        let rest_k = ((release_s + rest) / DT).round() as usize;
        let went = distance(r.left[k0], r.left[rest_k.min(r.left.len() - 1)]);
        split(
            &bench,
            &format!("release at {sweep} m/s"),
            &r,
            release_s - DT / 2.0,
            release_s + WINDOW_S,
            &format!("tool on {:.1} mm, still in {rest:.2} s", went * 1e3),
        );
    }
    // 10 cm jumps mid-session.
    let mut times = Vec::new();
    for axis in 0..3 {
        for sign in [1.0, -1.0] {
            let mut by = [0.0; 3];
            by[axis] = 0.10 * sign;
            let r = run(&mut bench, 2.5, |t, from| {
                let targets = Targets {
                    tcp: [
                        Some(if t >= 1.0 {
                            moved(&from[0], by)
                        } else {
                            from[0]
                        }),
                        Some(from[1]),
                    ],
                    ..Targets::default()
                };
                (targets, t <= DT + 1e-9)
            });
            times.push(settled(&r, 1.0, 0.005).unwrap_or(f64::NAN));
        }
    }
    let mean = times.iter().sum::<f64>() / times.len() as f64;
    let worst = times.iter().copied().fold(0.0, f64::max);
    println!(
        "10 cm jump mid-session to 5 mm: mean {mean:.3} s, worst {worst:.2} s  ({})",
        times
            .iter()
            .map(|t| format!("{t:.2}"))
            .collect::<Vec<_>>()
            .join(" ")
    );
    if let Some(path) = trace {
        write_trace(&mut bench, path);
    }
}

/// The film's scenario, in `examples/trace.rs`'s format: engage with the left hand's first
/// target 30 cm away (a program's first row, at its tool speed), hold; take the hands as an
/// operator and sweep 15 cm at 0.5 m/s, let go mid-sweep; engage again where the robot stands.
fn write_trace(bench: &mut Bench, path: &str) {
    const RELEASE_S: f64 = 3.15;
    const REENGAGE_S: f64 = 4.5;
    let program = Some(ToolSpeed {
        m_s: 0.5,
        rad_s: 3.0,
    });
    let mut q = bench.ready.clone();
    let start = bench.tools(&q);
    let first = moved(&start[0], [0.10, 0.10, 0.2645]);
    let available = vec![true; q.len()];
    // The robot at rest before the first tick: the gate differences from here, so the first
    // tick's own move counts (padded with the first tick's pose instead, its second tick read
    // as a start from rest a tick late, 2.6x).
    let mut ticks = vec![json!({
        "t": 0.0,
        "q": q,
        "target": {"left": document(&start[0]), "right": document(&start[1])},
        "taken": {"left": document(&start[0]), "right": document(&start[1])},
        "width": [0.07, 0.07],
        "solve": "Solved",
        "took_us": 0.0,
    })];
    let mut hold = start;
    let count = (6.0 / DT).round() as usize;
    for tick in 1..=count {
        let t = tick as f64 * DT;
        let engage = tick == 1 || (t - 2.0).abs() < 1e-9 || (t - REENGAGE_S).abs() < 1e-9;
        if (t - 2.0).abs() < 1e-9 || (t - REENGAGE_S).abs() < 1e-9 {
            // The operator takes hold where the robot stands (the station's clutch).
            hold = bench.tools(&q);
        }
        let targets = if t < 2.0 {
            Targets {
                tcp: [Some(first), Some(start[1])],
                tool_speed: [program, program],
                ..Targets::default()
            }
        } else if t < RELEASE_S {
            let along = 0.5 * (t - 3.0).clamp(0.0, 0.3);
            Targets {
                tcp: [Some(moved(&hold[0], [0.0, along, 0.0])), Some(hold[1])],
                ..Targets::default()
            }
        } else if t < REENGAGE_S {
            Targets::default()
        } else {
            Targets {
                tcp: [Some(hold[0]), Some(hold[1])],
                ..Targets::default()
            }
        };
        if engage {
            bench.solve.reset();
        }
        let measured: Vec<Option<f64>> = q.iter().map(|v| Some(*v)).collect();
        let began = std::time::Instant::now();
        let step = bench
            .solve
            .step(&q, &measured, &available, &targets, DT)
            .expect("a step");
        let took_us = began.elapsed().as_secs_f64() * 1e6;
        q.copy_from_slice(&step.q);
        let solve_state = format!("{:?}", step.solve);
        let now = bench.tools(&q);
        let shown = [
            targets.tcp[0].unwrap_or(now[0]),
            targets.tcp[1].unwrap_or(now[1]),
        ];
        ticks.push(json!({
            "t": t,
            "q": q,
            "target": {"left": document(&shown[0]), "right": document(&shown[1])},
            "taken": {"left": document(&shown[0]), "right": document(&shown[1])},
            "width": [0.07, 0.07],
            "solve": solve_state,
            "took_us": took_us,
        }));
    }
    // Where each motion starts and ends, and each joint's jerk cap: the film's gate fails a
    // window that commands more (`solver_film.py`).
    let caps: serde_json::Map<String, Value> = bench
        .names
        .iter()
        .zip(&bench.caps)
        .filter(|(n, _)| !n.contains("gripper"))
        .map(|(n, c)| (n.clone(), json!(c.2)))
        .collect();
    let transitions = json!([
        {"t": DT, "kind": "engage"},
        {"t": 2.0, "kind": "engage"},
        {"t": RELEASE_S, "kind": "release"},
        {"t": REENGAGE_S, "kind": "engage"},
    ]);
    let written = json!({
        "axes": bench.names,
        "dt": DT,
        "ticks": ticks,
        "transitions": transitions,
        "jerk_caps": caps,
    });
    std::fs::write(path, written.to_string()).expect("written");
    println!("{count} ticks to {path}");
}

fn document(pose: &Pose) -> [f64; 7] {
    let [x, y, z, w] = QuaternionLayout::Xyzw.write(pose.rotation);
    let t = pose.translation_m;
    [t[0], t[1], t[2], x, y, z, w]
}
