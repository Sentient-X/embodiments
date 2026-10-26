//! Solve the static upper body's neck to look at named points, from measured joints.
//!
//! ```text
//! cargo run --release -p sx-embodiment-ik --example look_at -- joints.json
//! ```
//!
//! `joints.json` is one object of measured joint positions by name (an `sx-record` line's
//! `positions`, merged across buses); a joint it does not name is taken at 0 and held. Only
//! the neck moves: every other axis is unavailable, so the arms stay where they are given.
//! The targets are the robot's own points at those joints: 1 m straight ahead of the ZED, each
//! gripper's tool frame, and the table between them. Each is solved from the last one's answer
//! with the headset ghost's caps until the step stops moving, and the example prints one JSON
//! line per target: the neck's joints, the residual pointing error and the reach.

#![allow(clippy::cast_precision_loss)]

use serde_json::{Value, json};
use sx_embodiment_ik::{Body, Caps, Ik, Settings, Target};

const DT: f64 = 0.01;
const MAX_TICKS: usize = 2000;

fn main() {
    let path = std::env::args().nth(1).expect("joints.json");
    let measured: Value =
        serde_json::from_str(&std::fs::read_to_string(&path).expect("read")).expect("json");
    let body = Body::static_upper_body();
    let axes = body.model().axes().to_vec();
    let mut caps = Caps::declared(&body, 0.25).expect("caps");
    for (speed, accel) in caps.speed.iter_mut().zip(caps.accel.iter_mut()) {
        *speed = 4.0;
        *accel = 400.0;
    }
    let settings = Settings {
        deadline: std::time::Duration::from_secs(1),
        ..Settings::default()
    };
    let mut ik = Ik::new(body.clone(), caps, settings, Vec::new()).expect("ik");
    let mut q: Vec<f64> = axes
        .iter()
        .map(|axis| {
            measured
                .get(&axis.name)
                .and_then(Value::as_f64)
                .unwrap_or(0.0)
        })
        .collect();
    // Joints named in `SX_HOLD` (comma-separated) stay where measured.
    let hold = std::env::var("SX_HOLD").unwrap_or_default();
    let neck: Vec<bool> = axes
        .iter()
        .map(|axis| axis.name.starts_with("so101_") && !hold.split(',').any(|h| h == axis.name))
        .collect();
    let limbs = body.limbs();
    let head = limbs.iter().position(|l| *l == "head").expect("head limb");
    let frames = body.model().poses(&q);
    let left = body
        .frame(
            &frames,
            limbs.iter().position(|l| *l == "left").expect("left"),
        )
        .t;
    let right = body
        .frame(
            &frames,
            limbs.iter().position(|l| *l == "right").expect("right"),
        )
        .t;
    let zed = body.frame(&frames, head).t;
    let table = [
        (left[0] + right[0]) / 2.0,
        (left[1] + right[1]) / 2.0,
        (left[2] + right[2]) / 2.0 - 0.15,
    ];
    let targets = [
        ("ahead", [zed[0] + 1.0, zed[1], zed[2]]),
        ("left gripper", left),
        ("right gripper", right),
        ("table between the hands", table),
        ("ahead again", [zed[0] + 1.0, zed[1], zed[2]]),
    ];
    for (name, point) in targets {
        let mut wanted: Vec<Option<Target>> = vec![None; limbs.len()];
        wanted[head] = Some(Target::LookAt { point_m: point });
        ik.reset();
        let mut ticks = 0;
        let mut report = None;
        for tick in 0..MAX_TICKS {
            let step = ik.step(&q, &neck, &wanted, DT).expect("step");
            let moved = step
                .q
                .iter()
                .zip(&q)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f64::max);
            q.clone_from(&step.q);
            report = Some((
                step.limbs[head].residual_rad,
                format!("{:?}", step.limbs[head].reach),
            ));
            ticks = tick + 1;
            if moved < 1e-6 {
                break;
            }
        }
        let (residual, reach) = report.expect("one step");
        let joints: serde_json::Map<String, Value> = axes
            .iter()
            .zip(&q)
            .filter(|(axis, _)| axis.name.starts_with("so101_"))
            .map(|(axis, value)| (axis.name.clone(), json!(value)))
            .collect();
        println!(
            "{}",
            json!({ "target": name, "point_m": point, "joints": joints,
                    "residual_deg": residual.to_degrees(), "reach": reach, "ticks": ticks })
        );
    }
}
