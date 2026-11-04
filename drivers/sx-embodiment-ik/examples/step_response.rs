//! How fast the teleoperation solve moves the static upper body at the caps the station
//! governs it with, stepped as a ghost (each step from the last) at 10 ms:
//!
//! - a 10 cm hand jump along each of +-x, +-y, +-z: the time to within 5 mm and 1 mm;
//! - a 0.5 m/s sweep of the hand along y and z: how far the tool trails the target;
//! - the peak tool speed over all of it;
//! - and the hard bounds: no joint past its speed or acceleration cap or its range, and the
//!   least clearance between the body's parts (negative is a collision).
//!
//! Each joint's caps are what the station's governor allows (`AxisLimits::rated` held to
//! `DamiaoChain::tracking_ceilings`): its rated speed and acceleration from the bindings
//! (`sx-damiao-can/bindings`), at `share`, held to the commissioned MIT gains' ceilings
//! (`sx-damiao-can` `static_upper_body.rs` and `chain.rs`: speed `min(0.2 kp/kd, 0.1
//! sqrt(kp/J), sqrt(0.2 peak/J))`, acceleration `0.1 kp/J`). `ramp_s` above zero also holds each
//! joint's acceleration to its speed over it, as the station did before 2026-11-04.
//!
//! ```text
//! cargo run --release -p sx-embodiment-ik --example step_response -- [share] [ramp_s] [clear]
//! ```

#![allow(
    clippy::too_many_lines,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)] // One measurement, start to report.

use std::collections::BTreeMap;

use sx_embodiment_ik::teleop::{TELEOP_SCHEMA, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{Body, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};

const DT: f64 = 0.01;
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

/// Peak torque per motor model, N m (`vocabulary.json` ratings).
fn peak(model: &str) -> f64 {
    match model {
        "damiao_dm8009p" => 40.0,
        "damiao_dm4340" => 27.0,
        "damiao_dm4310" => 7.0,
        other => panic!("no peak torque for {other}"),
    }
}

/// Per joint: (speed cap rad/s, acceleration cap rad/s^2), as the station governs it.
fn caps(share: f64) -> BTreeMap<String, (f64, f64)> {
    let bindings: serde_json::Value = serde_json::from_str(BINDINGS).expect("bindings");
    let mut out = BTreeMap::new();
    for axis in bindings["axes"].as_array().expect("axes") {
        let joint = axis["joint"].as_str().expect("joint");
        let speed = axis["speed"].as_f64().expect("speed") * share;
        let accel = axis["acceleration"].as_f64().expect("acceleration") * share;
        let caps = match GAINS.iter().find(|(name, ..)| *name == joint) {
            Some(&(_, kp, kd)) => {
                let j = axis["inertia"].as_f64().expect("inertia");
                let model = axis["model"].as_str().expect("model");
                let ceiling = (0.2 * kp / kd)
                    .min(0.1 * (kp / j).sqrt())
                    .min((0.2 * peak(model) / j).sqrt());
                (speed.min(ceiling), accel.min(0.1 * kp / j))
            }
            // A Feetech servo's own approach registers (4 rad/s, 16 rad/s^2).
            None => (speed.min(4.0), accel.min(16.0)),
        };
        out.insert(joint.to_string(), caps);
    }
    out
}

struct Run {
    /// Per tick: the left tool point and the target.
    tool: Vec<[f64; 3]>,
    target: Vec<[f64; 3]>,
    peak_speed_ratio: f64,
    peak_accel_ratio: f64,
    worst_joint: String,
    out_of_range: usize,
    least_clearance: f64,
}

fn main() {
    let args: Vec<f64> = std::env::args()
        .skip(1)
        .filter_map(|a| a.parse().ok())
        .collect();
    let share = args.first().copied().unwrap_or(1.0);
    let ramp_s = args.get(1).copied().unwrap_or(0.0);
    let caps = caps(share);
    let body = Body::static_upper_body();
    let names: Vec<String> = body.model().axes().iter().map(|a| a.name.clone()).collect();
    let speed: BTreeMap<String, f64> = names
        .iter()
        .map(|n| (n.clone(), caps.get(n).map_or(0.3, |c| c.0)))
        .collect();
    let accel: BTreeMap<String, f64> = names
        .iter()
        .map(|n| {
            let (s, a) = caps.get(n).copied().unwrap_or((0.3, 3.0));
            let a = if ramp_s > 0.0 { a.min(s / ramp_s) } else { a };
            (n.clone(), a)
        })
        .collect();
    let fastest = speed.values().copied().fold(0.0, f64::max);
    let config = TeleopConfig {
        schema: TELEOP_SCHEMA.into(),
        limbs: TeleopConfig::static_upper_body_limbs(),
        speed: speed.clone(),
        accel: accel.clone(),
        ramp_s: if ramp_s > 0.0 { ramp_s } else { 0.1 },
        max_step_rad: (fastest * DT).max(0.03),
        held: vec!["torso_joint".into()],
        obstacles: Vec::new(),
        posture: BTreeMap::new(),
    };
    println!("share {share}, ramp {ramp_s} s; caps (rad/s, rad/s^2):");
    for name in names
        .iter()
        .filter(|n| n.starts_with("left_") && !n.contains("finger"))
    {
        println!("  {name:28} {:5.2} {:6.1}", speed[name], accel[name]);
    }
    let index_of = |name: &str| names.iter().position(|a| a == name).expect(name);
    let solve0 =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).expect("solve");
    // The commissioned Ready pose (`sx_embodiments.known.static_upper_body._ARM_READY`): tool
    // points at (0.465, +/-0.238, 0.210) m. `clear` instead starts from the clearest of a grid of
    // symmetric arm poses.
    let mut start: Vec<f64> = solve0.ik().rest().to_vec();
    if std::env::args().any(|a| a == "clear") {
        let mut best = (f64::NEG_INFINITY, start.clone());
        for pitch in [-1.2, -0.9, -0.6, -0.3, 0.0] {
            for elbow in [0.4, 0.8, 1.2, 1.6] {
                for roll in [0.0, 0.2, 0.4] {
                    let mut c = start.clone();
                    c[index_of("left_shoulder_pitch_joint")] = pitch;
                    c[index_of("right_shoulder_pitch_joint")] = pitch;
                    c[index_of("left_shoulder_roll_joint")] = roll;
                    c[index_of("right_shoulder_roll_joint")] = -roll;
                    c[index_of("left_elbow_pitch_joint")] = -elbow;
                    c[index_of("right_elbow_pitch_joint")] = elbow;
                    let clear = solve0.ik().clearance(&c).0;
                    if clear > best.0 {
                        best = (clear, c);
                    }
                }
            }
        }
        start = best.1;
    } else {
        let joints = [
            "shoulder_pitch",
            "shoulder_roll",
            "upper_arm_yaw",
            "elbow_pitch",
            "forearm_yaw",
            "wrist_pitch",
            "wrist_roll",
        ];
        let ready = [
            ("left", [-0.15, 0.15, 0.0, -0.3, 0.0, 0.2, 0.0]),
            ("right", [0.15, -0.15, 0.0, 0.3, 0.0, -0.2, 0.0]),
        ];
        for (side, values) in ready {
            for (joint, value) in joints.iter().zip(values) {
                start[index_of(&format!("{side}_{joint}_joint"))] = value;
            }
        }
    }
    println!(
        "start clearance {:.3} m, left tool at {:?}",
        solve0.ik().clearance(&start).0,
        body.frame(&body.model().poses(&start), 0).t
    );
    let axis_speed: Vec<f64> = names
        .iter()
        .map(|n| if n == "torso_joint" { 0.0 } else { speed[n] })
        .collect();
    let capability = body.tool_capability(&body.model().poses(&start), 0, &axis_speed);
    println!(
        "left tool capability at the start (fastest direction): {:.2} m/s, {:.2} rad/s",
        capability.linear_m_s, capability.angular_rad_s
    );
    let limits: Vec<Option<[f64; 2]>> = body.model().axes().iter().map(|a| a.limits).collect();

    // One ghost run: the left hand follows `path(t)`, the right holds.
    let run = |path: &dyn Fn(f64) -> [f64; 3], ticks: usize| -> Run {
        let mut solve = Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config)
            .expect("solve");
        let mut q = start.clone();
        let frames = body.model().poses(&q);
        let (left, right) = (body.frame(&frames, 0).pose(), body.frame(&frames, 1).pose());
        let measured = vec![None; q.len()];
        let available = vec![true; q.len()];
        let mut previous = vec![0.0; q.len()];
        let mut out = Run {
            tool: Vec::new(),
            target: Vec::new(),
            peak_speed_ratio: 0.0,
            peak_accel_ratio: 0.0,
            worst_joint: String::new(),
            out_of_range: 0,
            least_clearance: f64::INFINITY,
        };
        solve.reset();
        for tick in 1..=ticks {
            let mut target = left;
            target.translation_m = path(tick as f64 * DT);
            let targets = Targets {
                tcp: [Some(target), Some(right)],
                ..Targets::default()
            };
            let step = solve
                .step(&q, &measured, &available, &targets, DT)
                .expect("a step")
                .clone();
            for (index, v) in step.qdot.iter().enumerate() {
                let name = &names[index];
                let ratio = v.abs() / speed[name];
                let a = ((v - previous[index]) / DT).abs() / accel[name];
                if ratio > out.peak_speed_ratio {
                    out.peak_speed_ratio = ratio;
                    out.worst_joint.clone_from(name);
                }
                out.peak_accel_ratio = out.peak_accel_ratio.max(a);
                if let Some([lo, hi]) = limits[index]
                    && (step.q[index] < lo - 1e-9 || step.q[index] > hi + 1e-9)
                {
                    out.out_of_range += 1;
                }
            }
            previous.copy_from_slice(&step.qdot);
            q.copy_from_slice(&step.q);
            out.least_clearance = out.least_clearance.min(solve.ik().clearance(&q).0);
            out.tool.push(body.frame(&body.model().poses(&q), 0).t);
            out.target.push(target.translation_m);
        }
        out
    };
    let distance =
        |a: [f64; 3], b: [f64; 3]| (0..3).map(|k| (a[k] - b[k]).powi(2)).sum::<f64>().sqrt();
    let tool_speed = |run: &Run| {
        run.tool
            .windows(2)
            .map(|w| distance(w[0], w[1]) / DT)
            .fold(0.0, f64::max)
    };
    let frames = body.model().poses(&start);
    let home = body.frame(&frames, 0).t;
    let mut peak_tool = 0.0_f64;
    let (mut worst_speed, mut worst_accel, mut out_of_range, mut least) =
        (0.0_f64, 0.0_f64, 0, f64::INFINITY);
    let mut worst_joint = String::new();
    let mut account = |run: &Run| {
        peak_tool = peak_tool.max(tool_speed(run));
        if run.peak_speed_ratio > worst_speed {
            worst_speed = run.peak_speed_ratio;
            worst_joint.clone_from(&run.worst_joint);
        }
        worst_accel = worst_accel.max(run.peak_accel_ratio);
        out_of_range += run.out_of_range;
        least = least.min(run.least_clearance);
    };

    println!("10 cm hand jumps (time to within 5 mm / 1 mm):");
    let mut closes = Vec::new();
    for (label, k, by) in [
        ("+x", 0, 0.1),
        ("-x", 0, -0.1),
        ("+y", 1, 0.1),
        ("-y", 1, -0.1),
        ("+z", 2, 0.1),
        ("-z", 2, -0.1),
    ] {
        let mut goal = home;
        goal[k] += by;
        let mut u = [0.0; 3];
        u[k] = by.signum();
        let along = body.tool_speed_along(&body.model().poses(&start), 0, &axis_speed, u);
        let result = run(&move |_| goal, 300);
        let left_off = distance(*result.tool.last().expect("ticks"), goal);
        let within = |mm: f64| {
            result
                .tool
                .iter()
                .position(|t| distance(*t, goal) < mm / 1000.0)
                .map(|tick| (tick + 1) as f64 * DT)
        };
        let (five, one) = (within(5.0), within(1.0));
        println!(
            "  {label}: {} / {}, peak tool {:.2} m/s (capability along it {along:.2}){}",
            five.map_or("never".into(), |s| format!("{s:.2} s")),
            one.map_or("never".into(), |s| format!("{s:.2} s")),
            tool_speed(&result),
            if five.is_none() {
                format!(
                    " (out of reach: {:.0} mm short after 3 s)",
                    left_off * 1000.0
                )
            } else {
                String::new()
            }
        );
        closes.extend(five);
        account(&result);
    }
    let (fast, slow) = closes.iter().fold((f64::INFINITY, 0.0_f64), |(lo, hi), s| {
        (lo.min(*s), hi.max(*s))
    });
    println!("  closing to 5 mm: {fast:.2}-{slow:.2} s");

    println!("0.5 m/s sweeps, 0.15 m (lag behind the target, steady = its last 0.2 s):");
    for (label, k, sign) in [
        ("+y", 1, 1.0),
        ("-y", 1, -1.0),
        ("+z", 2, 1.0),
        ("-z", 2, -1.0),
    ] {
        let speed_m_s = 0.5;
        let length = 0.15;
        let result = run(
            &move |t| {
                let mut p = home;
                p[k] += sign * (speed_m_s * t).min(length);
                p
            },
            300,
        );
        let moving = (length / speed_m_s / DT) as usize;
        let lags: Vec<f64> = (0..moving)
            .map(|i| distance(result.tool[i], result.target[i]))
            .collect();
        let steady = &lags[moving.saturating_sub(20)..];
        let steady = steady.iter().sum::<f64>() / steady.len() as f64;
        let worst = lags.iter().copied().fold(0.0, f64::max);
        let settled = result.tool[moving..]
            .iter()
            .position(|t| distance(*t, result.target[result.target.len() - 1]) < 0.005)
            .map(|i| i as f64 * DT);
        println!(
            "  {label}: lag {:.0} mm steady ({:.0} ms), {:.0} mm worst; within 5 mm {} after the target stops",
            steady * 1000.0,
            steady / speed_m_s * 1000.0,
            worst * 1000.0,
            settled.map_or("never".into(), |s| format!("{s:.2} s"))
        );
        account(&result);
    }
    println!(
        "peak tool speed {peak_tool:.2} m/s; worst joint speed {:.3} of its cap ({worst_joint}), \
         acceleration {:.3} of its cap; {out_of_range} ticks out of range; least clearance {:.1} mm",
        worst_speed,
        worst_accel,
        least * 1000.0
    );
}
