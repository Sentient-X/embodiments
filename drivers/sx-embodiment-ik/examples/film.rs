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
//! [`Teleop`] at 100 Hz with no head pose (the camera looks between the hands), and
//! every joint runs at `arm_rad_s` (2) except the SO-101 neck at `neck_rad_s` (3), each reached
//! in 0.1 s. The output holds the link names and, per frame: time, each link's position and
//! quaternion (xyzw), the operator's two tool points, both jaw widths, the head's pointing
//! error and the smallest proxy clearance.

#![allow(clippy::cast_precision_loss, clippy::many_single_char_names)]

use std::collections::BTreeMap;

use serde_json::{Value, json};
use sx_embodiment_ik::teleop::{TELEOP_SCHEMA, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{Body, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};
use sx_frames::{Pose, Quaternion, QuaternionLayout};

const DT: f64 = 0.01;
const FPS: f64 = 30.0;

type Vec3 = [f64; 3];

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

/// A series of xyzw quaternions.
fn quaternions(v: &Value) -> Vec<Quaternion> {
    series::<4>(v)
        .into_iter()
        .map(|q| QuaternionLayout::Xyzw.read(q))
        .collect()
}

fn lerp(a: Vec3, b: Vec3, s: f64) -> Vec3 {
    [0, 1, 2].map(|k| a[k] + (b[k] - a[k]) * s)
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
            quaternions(&episode["left"]["q"]),
        ),
        (
            series::<3>(&episode["right"]["p"]),
            quaternions(&episode["right"]["q"]),
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
    let head_q = quaternions(&episode["head"]["q"]);

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
        held: Vec::new(),
        obstacles: Vec::new(),
        posture: std::collections::BTreeMap::new(),
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
    let eye: Vec3 = [0, 1, 2].map(|k| head_p.iter().map(|p| p[k]).sum::<f64>() / count);
    let (mut fx, mut fy) = (0.0, 0.0);
    for q in &head_q {
        let f = q.rotate([0.0, 0.0, 1.0]);
        fx += f[0];
        fy += f[1];
    }
    let yaw = fy.atan2(fx);
    let rotation = QuaternionLayout::Xyzw.read([0.0, 0.0, (-yaw / 2.0).sin(), (-yaw / 2.0).cos()]);
    let turned = rotation.rotate(eye);
    let place = |p: Vec3| {
        let r = rotation.rotate(p);
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
                rotation: rotation * r[sample].slerp(r[sample + 1], s),
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
                    let quat = QuaternionLayout::Xyzw.write(Quaternion::from_matrix(&iso.r));
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
