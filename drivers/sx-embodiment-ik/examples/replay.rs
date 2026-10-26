//! Replay real classic YUBI operator trajectories into the static upper body.
//!
//! ```text
//! cargo run --release -p sx-embodiment-ik --example replay -- episodes.json [summary.json]
//! ```
//!
//! `episodes.json` is what the superproject's
//! `docs/plans/evidence/yubi-teleop-static-upper-body-2026-10/fetch_yubi_episodes.py` writes:
//! per episode, the left and right YUBI tip poses and the headset pose over time, in the
//! Quest tracking space turned Z-up (metres, quaternions xyzw).
//!
//! Each episode is placed by one fixed `W` that puts the operator's mean eye point on the
//! robot's ZED at rest and the operator's mean heading along the robot's +x: eyes to eyes.
//! Tip poses map through `W` unchanged in shape (a YUBI tip and a robot tool frame share the
//! convention: +x along the fingers, jaws along y). The head's view ray, 1 m out, is the
//! neck's look-at target. Targets are interpolated to the station's 100 Hz tick.
//!
//! Two solvers run on the same targets: the **ghost** (caps at 4 rad/s and 400 rad/s^2, so
//! only reach, limits, collision and the solver's per-tick step bind) says which targets are reachable; the **station**
//! (the description's declared 0.5 rad/s, reached in 0.25 s) is what the robot would do. The
//! first two seconds (engaging from rest) are not scored.

// Test and evaluation code: counts as f64, exact comparisons of held values, formula names.
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::many_single_char_names,
    clippy::float_cmp,
    clippy::doc_markdown
)]

use std::time::Instant;

use serde_json::Value;
use sx_embodiment_ik::{Active, Body, Caps, Ik, Reach, Settings, Target};

/// Every axis available.
const ALL: [bool; 64] = [true; 64];

const DT: f64 = 0.01;
const SETTLE_S: f64 = 2.0;

type V3 = [f64; 3];
type Q = [f64; 4];

fn settings() -> Settings {
    // The replay scores answers, not the clock of a shared laptop.
    Settings {
        deadline: std::time::Duration::from_secs(1),
        ..Settings::default()
    }
}

fn floats<const N: usize>(v: &Value) -> [f64; N] {
    let mut out = [0.0; N];
    for (slot, x) in out.iter_mut().zip(v.as_array().expect("array")) {
        *slot = x.as_f64().expect("number");
    }
    out
}

fn series<const N: usize>(v: &Value) -> Vec<[f64; N]> {
    v.as_array()
        .expect("series")
        .iter()
        .map(floats::<N>)
        .collect()
}

fn q_mul(a: Q, b: Q) -> Q {
    let [ax, ay, az, aw] = a;
    let [bx, by, bz, bw] = b;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

fn q_rotate(q: Q, v: V3) -> V3 {
    let p = q_mul(
        q_mul(q, [v[0], v[1], v[2], 0.0]),
        [-q[0], -q[1], -q[2], q[3]],
    );
    [p[0], p[1], p[2]]
}

fn nlerp(a: Q, b: Q, s: f64) -> Q {
    let sign = if a.iter().zip(&b).map(|(x, y)| x * y).sum::<f64>() < 0.0 {
        -1.0
    } else {
        1.0
    };
    let q: Vec<f64> = (0..4).map(|k| a[k] * (1.0 - s) + sign * b[k] * s).collect();
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
}

fn lerp(a: V3, b: V3, s: f64) -> V3 {
    [
        a[0] + (b[0] - a[0]) * s,
        a[1] + (b[1] - a[1]) * s,
        a[2] + (b[2] - a[2]) * s,
    ]
}

/// The fixed placement: robot_from_operator as a yaw quaternion and a translation.
struct Placement {
    rotation: Q,
    translation: V3,
}

impl Placement {
    fn point(&self, p: V3) -> V3 {
        let r = q_rotate(self.rotation, p);
        [
            r[0] + self.translation[0],
            r[1] + self.translation[1],
            r[2] + self.translation[2],
        ]
    }
}

fn percentile(values: &mut [f64], p: f64) -> f64 {
    if values.is_empty() {
        return f64::NAN;
    }
    values.sort_by(f64::total_cmp);
    let index = ((values.len() - 1) as f64 * p).round() as usize;
    values[index]
}

#[derive(Default)]
struct Totals {
    ticks: usize,
    arm_ticks: usize,
    unreachable: usize,
    station_m: Vec<f64>,
    station_rad: Vec<f64>,
    ghost_m: Vec<f64>,
    ghost_rad: Vec<f64>,
    head_rad: Vec<f64>,
    joint_ticks: usize,
    at_limit: usize,
    collisions: usize,
    min_clearance: f64,
    solve_s: Vec<f64>,
    ghost_solve_s: Vec<f64>,
    relaxed: usize,
    held: usize,
    max_iterations: u32,
}

#[allow(clippy::too_many_lines)]
fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("episodes.json");
    let out = args.next();
    let document: Value =
        serde_json::from_str(&std::fs::read_to_string(&path).expect("read")).expect("json");
    let station_body = Body::static_upper_body();
    let station_caps = Caps::declared(&station_body, 0.25).expect("caps");
    let mut ghost_caps = station_caps.clone();
    for (speed, accel) in ghost_caps.speed.iter_mut().zip(ghost_caps.accel.iter_mut()) {
        *speed = 4.0;
        *accel = 400.0;
    }
    let solved = station_body.solved().to_vec();
    let axes = station_body.model().axes().to_vec();
    let arm_axes: Vec<usize> = solved
        .iter()
        .copied()
        .filter(|&a| axes[a].name.starts_with("left_") || axes[a].name.starts_with("right_"))
        .collect();
    let mut totals = Totals {
        min_clearance: f64::INFINITY,
        ..Totals::default()
    };
    let mut per_episode = Vec::new();
    for episode in document["episodes"].as_array().expect("episodes") {
        let id = episode["id"].as_str().unwrap_or("?").to_owned();
        let task = episode["task"].as_str().unwrap_or("?").to_owned();
        let t: Vec<f64> = episode["t"]
            .as_array()
            .expect("t")
            .iter()
            .map(|v| v.as_f64().expect("t"))
            .collect();
        let hands = [
            (
                series::<3>(&episode["left"]["p"]),
                series::<4>(&episode["left"]["q"]),
            ),
            (
                series::<3>(&episode["right"]["p"]),
                series::<4>(&episode["right"]["q"]),
            ),
        ];
        let head_p = series::<3>(&episode["head"]["p"]);
        let head_q = series::<4>(&episode["head"]["q"]);
        // The operator's mean eye point and heading (the view ray, +z of the RDF head frame,
        // flattened).
        let count = head_p.len() as f64;
        let mean = |k: usize| head_p.iter().map(|p| p[k]).sum::<f64>() / count;
        let eye = [mean(0), mean(1), mean(2)];
        let (mut fx, mut fy) = (0.0, 0.0);
        for q in &head_q {
            let f = q_rotate(*q, [0.0, 0.0, 1.0]);
            fx += f[0];
            fy += f[1];
        }
        let yaw = fy.atan2(fx);
        let rotation = [0.0, 0.0, (-yaw / 2.0).sin(), (-yaw / 2.0).cos()];
        let mut ik = Ik::new(
            station_body.clone(),
            station_caps.clone(),
            settings(),
            Vec::new(),
        )
        .expect("station");
        let mut ghost = Ik::new(
            station_body.clone(),
            ghost_caps.clone(),
            settings(),
            Vec::new(),
        )
        .expect("ghost");
        let rest = ik.rest().to_vec();
        let zed = {
            let frames = station_body.model().poses(&rest);
            station_body.frame(&frames, 2).t
        };
        let turned = q_rotate(rotation, eye);
        let placement = Placement {
            rotation,
            translation: [zed[0] - turned[0], zed[1] - turned[1], zed[2] - turned[2]],
        };
        let mut q = rest.clone();
        let mut q_ghost = rest.clone();
        let mut sample = 0;
        let end = *t.last().expect("samples");
        let mut time = 0.0;
        let mut episode_m = Vec::new();
        let mut episode_unreachable = 0;
        let mut episode_arm_ticks = 0;
        while time <= end {
            while sample + 2 < t.len() && t[sample + 1] <= time {
                sample += 1;
            }
            let s = ((time - t[sample]) / (t[sample + 1] - t[sample])).clamp(0.0, 1.0);
            let mut targets = [None, None, None];
            for (side, (p, r)) in hands.iter().enumerate() {
                let position = placement.point(lerp(p[sample], p[sample + 1], s));
                let orientation = q_mul(rotation, nlerp(r[sample], r[sample + 1], s));
                targets[side] = Some(Target::Pose {
                    translation_m: position,
                    rotation_xyzw: orientation,
                });
            }
            let view = q_rotate(
                rotation,
                q_rotate(
                    nlerp(head_q[sample], head_q[sample + 1], s),
                    [0.0, 0.0, 1.0],
                ),
            );
            let head = placement.point(lerp(head_p[sample], head_p[sample + 1], s));
            targets[2] = Some(Target::LookAt {
                point_m: [head[0] + view[0], head[1] + view[1], head[2] + view[2]],
            });
            let started = Instant::now();
            let ghost_step = ghost
                .step(&q_ghost, &ALL[..q_ghost.len()], &targets, DT)
                .expect("ghost")
                .clone();
            let ghost_s = started.elapsed().as_secs_f64();
            let started = Instant::now();
            let step = ik
                .step(&q, &ALL[..q.len()], &targets, DT)
                .expect("station")
                .clone();
            let station_s = started.elapsed().as_secs_f64();
            q.copy_from_slice(&step.q);
            q_ghost.copy_from_slice(&ghost_step.q);
            totals.solve_s.push(station_s);
            totals.max_iterations = totals.max_iterations.max(step.iterations);
            totals.ghost_solve_s.push(ghost_s);
            let (cs, a1, b1) = ik.clearance(&q);
            let (cg, a2, b2) = ghost.clearance(&q_ghost);
            if cs < 0.0 || cg < 0.0 {
                eprintln!(
                    "penetration at {time:.2} s: station {cs:.4} m {a1}/{b1}, ghost {cg:.4} m {a2}/{b2}"
                );
            }
            let clearance = cs.min(cg);
            totals.min_clearance = totals.min_clearance.min(clearance);
            if clearance < 0.0 {
                totals.collisions += 1;
            }
            if time >= SETTLE_S {
                totals.ticks += 1;
                totals.relaxed += usize::from(step.solve == sx_embodiment_ik::Solve::Relaxed);
                totals.held += usize::from(step.solve == sx_embodiment_ik::Solve::Held);
                for side in 0..2 {
                    let reach = ghost_step.limbs[side].reach;
                    totals.arm_ticks += 1;
                    episode_arm_ticks += 1;
                    if reach == Reach::Unreachable {
                        totals.unreachable += 1;
                        episode_unreachable += 1;
                    } else {
                        totals.station_m.push(step.limbs[side].residual_m);
                        totals.station_rad.push(step.limbs[side].residual_rad);
                        totals.ghost_m.push(ghost_step.limbs[side].residual_m);
                        totals.ghost_rad.push(ghost_step.limbs[side].residual_rad);
                        episode_m.push(step.limbs[side].residual_m);
                    }
                }
                totals.head_rad.push(step.limbs[2].residual_rad);
                for &axis in &arm_axes {
                    totals.joint_ticks += 1;
                    let limited = step
                        .limbs
                        .iter()
                        .flat_map(|l| &l.active)
                        .any(|a| *a == Active::Limit { axis });
                    totals.at_limit += usize::from(limited);
                }
            }
            time += DT;
        }
        let median = percentile(&mut episode_m, 0.5);
        let unreachable = episode_unreachable as f64 / episode_arm_ticks.max(1) as f64;
        println!(
            "{id} {task:40} {end:5.1} s  station median {:5.1} mm  unreachable {:5.1} %",
            median * 1000.0,
            unreachable * 100.0
        );
        per_episode.push(serde_json::json!({
            "id": id, "task": task, "seconds": end,
            "station_median_mm": median * 1000.0, "unreachable_share": unreachable,
        }));
    }
    let deg = 180.0 / std::f64::consts::PI;
    let ms = |v: &mut Vec<f64>, p: f64| percentile(v, p) * 1000.0;
    let summary = serde_json::json!({
        "episodes": per_episode,
        "scored_ticks": totals.ticks,
        "unreachable_share": totals.unreachable as f64 / totals.arm_ticks.max(1) as f64,
        "station_tip_mm": {"p50": ms(&mut totals.station_m, 0.5), "p90": ms(&mut totals.station_m, 0.9), "p99": ms(&mut totals.station_m, 0.99)},
        "station_tip_deg": {"p50": percentile(&mut totals.station_rad, 0.5) * deg, "p90": percentile(&mut totals.station_rad, 0.9) * deg},
        "ghost_tip_mm": {"p50": ms(&mut totals.ghost_m, 0.5), "p90": ms(&mut totals.ghost_m, 0.9), "p99": ms(&mut totals.ghost_m, 0.99)},
        "ghost_tip_deg": {"p50": percentile(&mut totals.ghost_rad, 0.5) * deg, "p90": percentile(&mut totals.ghost_rad, 0.9) * deg},
        "head_pointing_deg": {"p50": percentile(&mut totals.head_rad, 0.5) * deg, "p90": percentile(&mut totals.head_rad, 0.9) * deg},
        "arm_joint_ticks_at_limit_share": totals.at_limit as f64 / totals.joint_ticks.max(1) as f64,
        "ticks_with_penetration": totals.collisions,
        "min_clearance_mm": totals.min_clearance * 1000.0,
        "relaxed_ticks": totals.relaxed,
        "held_ticks": totals.held,
        "station_max_qp_iterations": totals.max_iterations,
        "station_solve_us": {"p50": percentile(&mut totals.solve_s, 0.5) * 1e6, "p99": percentile(&mut totals.solve_s, 0.99) * 1e6, "max": percentile(&mut totals.solve_s, 1.0) * 1e6},
        "ghost_solve_us": {"p50": percentile(&mut totals.ghost_solve_s, 0.5) * 1e6, "p99": percentile(&mut totals.ghost_solve_s, 0.99) * 1e6, "max": percentile(&mut totals.ghost_solve_s, 1.0) * 1e6},
    });
    let text = serde_json::to_string_pretty(&summary).expect("summary");
    println!("{text}");
    if let Some(out) = out {
        std::fs::write(out, text + "\n").expect("write");
    }
}
