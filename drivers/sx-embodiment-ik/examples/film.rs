//! Replay recorded classic YUBI episodes through the teleoperation solve and write every
//! link's pose at 30 frames a second, for rendering.
//!
//! ```text
//! cargo run --release -p sx-embodiment-ik --example film -- episodes.json <index> out.json \
//!     [arm_rad_s] [neck_rad_s]
//! ```
//!
//! `episodes.json` is `replay`'s input. The episode is placed as `replay` places it (the
//! operator's mean eye on the ZED, their mean heading along +x), the hands and jaws drive
//! [`Teleop`] at 100 Hz with [`HeadPolicy::Hands`] (the neck ignores the operator's head), and
//! every joint runs at `arm_rad_s` (2) except the SO-101 neck at `neck_rad_s` (3), each reached
//! in 0.1 s. The output holds the link names and, per frame: time, each link's position and
//! quaternion (xyzw), the operator's two tool points, both jaw widths, the head's pointing
//! error and the smallest proxy clearance.

#![allow(clippy::cast_precision_loss, clippy::many_single_char_names)]

use std::collections::BTreeMap;

use serde_json::{Value, json};
use sx_embodiment_ik::teleop::{HeadPolicy, Pose, TELEOP_SCHEMA, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{Body, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};

const DT: f64 = 0.01;
const FPS: f64 = 30.0;

type V3 = [f64; 3];
type Q = [f64; 4];

fn series<const N: usize>(v: &Value) -> Vec<[f64; N]> {
    v.as_array()
        .expect("series")
        .iter()
        .map(|row| {
            let mut out = [0.0; N];
            for (slot, x) in out.iter_mut().zip(row.as_array().expect("row")) {
                *slot = x.as_f64().expect("number");
            }
            out
        })
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
    [0, 1, 2].map(|k| a[k] + (b[k] - a[k]) * s)
}

/// A rotation matrix's quaternion, xyzw.
fn quaternion(r: &[[f64; 3]; 3]) -> Q {
    let trace = r[0][0] + r[1][1] + r[2][2];
    if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        [
            (r[2][1] - r[1][2]) / s,
            (r[0][2] - r[2][0]) / s,
            (r[1][0] - r[0][1]) / s,
            0.25 * s,
        ]
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let s = (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt() * 2.0;
        [
            0.25 * s,
            (r[0][1] + r[1][0]) / s,
            (r[0][2] + r[2][0]) / s,
            (r[2][1] - r[1][2]) / s,
        ]
    } else if r[1][1] > r[2][2] {
        let s = (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt() * 2.0;
        [
            (r[0][1] + r[1][0]) / s,
            0.25 * s,
            (r[1][2] + r[2][1]) / s,
            (r[0][2] - r[2][0]) / s,
        ]
    } else {
        let s = (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt() * 2.0;
        [
            (r[0][2] + r[2][0]) / s,
            (r[1][2] + r[2][1]) / s,
            0.25 * s,
            (r[1][0] - r[0][1]) / s,
        ]
    }
}

#[allow(clippy::too_many_lines)]
fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("episodes.json");
    let index: usize = args.next().expect("index").parse().expect("index");
    let out = args.next().expect("out.json");
    let arm: f64 = args.next().map_or(2.0, |v| v.parse().expect("arm rad/s"));
    let neck: f64 = args.next().map_or(3.0, |v| v.parse().expect("neck rad/s"));
    let document: Value =
        serde_json::from_str(&std::fs::read_to_string(&path).expect("read")).expect("json");
    let episode = &document["episodes"][index];
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
    let jaws: [Vec<f64>; 2] = ["jaw_left", "jaw_right"].map(|key| {
        episode[key]
            .as_array()
            .expect("jaw")
            .iter()
            .map(|v| v.as_f64().unwrap_or(0.0))
            .collect()
    });
    let head_p = series::<3>(&episode["head"]["p"]);
    let head_q = series::<4>(&episode["head"]["q"]);

    let body = Body::static_upper_body();
    let mut speed = BTreeMap::new();
    let mut accel = BTreeMap::new();
    for axis in body.model().axes() {
        let cap = if axis.name.starts_with("so101_") {
            neck
        } else {
            arm
        };
        speed.insert(axis.name.clone(), cap);
        accel.insert(axis.name.clone(), cap / 0.1);
    }
    let config = TeleopConfig {
        schema: TELEOP_SCHEMA.into(),
        limbs: TeleopConfig::static_upper_body_limbs(),
        speed,
        accel,
        ramp_s: 0.1,
        max_step_rad: 0.08,
        head: HeadPolicy::Hands,
        held: Vec::new(),
        obstacles: Vec::new(),
    };
    let mut teleop =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).expect("solve");
    let rest = teleop.ik().rest().to_vec();
    let head = body
        .limbs()
        .iter()
        .position(|l| *l == "head")
        .expect("head");
    let zed = body.frame(&body.model().poses(&rest), head).t;

    // `replay`'s placement: the mean eye on the ZED, the mean heading along +x.
    let count = head_p.len() as f64;
    let eye: V3 = [0, 1, 2].map(|k| head_p.iter().map(|p| p[k]).sum::<f64>() / count);
    let (mut fx, mut fy) = (0.0, 0.0);
    for q in &head_q {
        let f = q_rotate(*q, [0.0, 0.0, 1.0]);
        fx += f[0];
        fy += f[1];
    }
    let yaw = fy.atan2(fx);
    let rotation = [0.0, 0.0, (-yaw / 2.0).sin(), (-yaw / 2.0).cos()];
    let turned = q_rotate(rotation, eye);
    let place = |p: V3| {
        let r = q_rotate(rotation, p);
        [0, 1, 2].map(|k| r[k] + zed[k] - turned[k])
    };

    let width = rest.len();
    let measured = vec![None; width];
    let available = vec![true; width];
    let mut q = rest;
    let names: Vec<String> = body.model().links().to_vec();
    let mut frames_out = Vec::new();
    let (mut sample, mut time, mut next_frame) = (0, 0.0, 0.0);
    let end = *t.last().expect("samples");
    teleop.reset();
    while time <= end {
        while sample + 2 < t.len() && t[sample + 1] <= time {
            sample += 1;
        }
        let s = ((time - t[sample]) / (t[sample + 1] - t[sample])).clamp(0.0, 1.0);
        let mut targets = Targets::default();
        for (side, (p, r)) in hands.iter().enumerate() {
            targets.tcp[side] = Some(Pose {
                translation_m: place(lerp(p[sample], p[sample + 1], s)),
                rotation_xyzw: q_mul(rotation, nlerp(r[sample], r[sample + 1], s)),
            });
            let jaw = &jaws[side];
            targets.gripper_width_m[side] = Some(jaw[sample] + (jaw[sample + 1] - jaw[sample]) * s);
        }
        let step = teleop
            .step(&q, &measured, &available, &targets, DT)
            .expect("step");
        let head_error = step.limbs[head].residual_rad;
        q.copy_from_slice(&step.q);
        if time + 1e-9 >= next_frame {
            let poses = body.model().poses(&q);
            let links: Vec<Vec<f64>> = poses
                .links
                .iter()
                .map(|iso| {
                    let quat = quaternion(&iso.r);
                    vec![
                        iso.t[0], iso.t[1], iso.t[2], quat[0], quat[1], quat[2], quat[3],
                    ]
                })
                .collect();
            let (clearance, a, b) = teleop.ik().clearance(&q);
            frames_out.push(json!({
                "t": time,
                "links": links,
                "targets": targets.tcp.map(|p| p.map(|p| p.translation_m)),
                "jaws": targets.gripper_width_m,
                "head_error_rad": head_error,
                "clearance_m": clearance,
                "closest": [a, b],
                "q": q,
            }));
            next_frame += 1.0 / FPS;
        }
        time += DT;
    }
    let joints: Vec<String> = body.model().axes().iter().map(|a| a.name.clone()).collect();
    let document = json!({
        "episode": episode["id"], "task": episode["task"], "fps": FPS,
        "arm_rad_s": arm, "neck_rad_s": neck,
        "links": names, "joints": joints, "frames": frames_out,
    });
    std::fs::write(&out, document.to_string()).expect("write");
    println!(
        "{} frames -> {out}",
        document["frames"].as_array().map_or(0, Vec::len)
    );
}
