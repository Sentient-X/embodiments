//! The solve on the static upper body as the DGX Spark drives it: each Damiao joint a follower
//! of its commissioned MIT gains (`sx-damiao-can` `static_upper_body.rs` `JointGains`) over the
//! inertia its bindings carry, its planned velocity fed forward as the MIT law's `kd (v_des -
//! v)` (capped at `MAX_COAST kp / kd`), each command reaching the arm a transport delay late
//! (the Spark's arms trail their commands by 26-63 ms, 2026-10-27..30 journals). The followers
//! are act-sim's own (`act-sim` identified the ringing shoulders within 0.5 % in omega and 4 % in
//! zeta), scaled across a band for the inertia's posture dependence.
//!
//! Measured on main ad190f0 (2026-11-03), at 50 / 90 Hz across scale 0.8-1.25, 26-63 ms,
//! feed-forward on and off, sag and 1 mrad jitter (`table_spark`, 3,168 runs, 0 Held): the nominal
//! model kept 2.24 mm at 50 Hz and -0.70 mm at 90 Hz (the left elbow's capsule into the box at
//! 63 ms); at 1.25x, -7.8 mm and -17.2 mm (a finger's own shape 4.55 mm into the counter). With
//! each pair's approach paced by its room (`Settings::approach_time_s`, 0.2 s): 8.40 / 7.33 mm
//! nominal, 4.41 mm at worst, no part inside a box. `real` is the links' convex pieces against
//! the boxes, the parts' own shape rather than their capsules.

//! `cargo test --release -p sx-embodiment-ik --test spark_followers table -- --ignored
//! --nocapture` prints every case (about 7 min for `table_spark` on 10 cores).

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::too_many_lines,
    clippy::struct_excessive_bools,
    clippy::many_single_char_names
)]

use std::collections::{BTreeMap, VecDeque};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

use sx_embodiment_ik::teleop::{Pose, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{
    READING_JITTER, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF, Solve,
};

const SUBSTEPS: usize = 20;

/// Each Damiao joint: its commissioned MIT gains (kp N m/rad, kd N m s/rad), the inertia its
/// bindings carry at the joint (kg m^2, the rotor through the reduction squared included; `kp /
/// omega^2` of embodiments `ik/follower-model`'s followers table, 2026-11-03) and its motor's peak
/// torque (N m, `vocabulary.json` ratings).
const SPARK: [(&str, f64, f64, f64, f64); 15] = [
    ("torso_joint", 40.0, 1.0, 1.58097, 40.0),
    ("left_shoulder_pitch_joint", 60.984, 1.091, 0.41584, 40.0),
    ("left_shoulder_roll_joint", 60.984, 1.091, 0.45243, 40.0),
    ("left_upper_arm_yaw_joint", 24.0, 5.0, 0.15634, 27.0),
    ("left_elbow_pitch_joint", 30.492, 4.655, 0.15579, 27.0),
    ("left_forearm_yaw_joint", 29.04, 4.75, 0.04992, 27.0),
    ("left_wrist_pitch_joint", 20.0, 1.0, 0.01933, 7.0),
    ("left_wrist_roll_joint", 20.0, 1.0, 0.00269, 7.0),
    ("right_shoulder_pitch_joint", 60.984, 1.091, 0.40244, 40.0),
    ("right_shoulder_roll_joint", 42.0, 0.98, 0.42594, 40.0),
    ("right_upper_arm_yaw_joint", 30.492, 4.655, 0.15602, 27.0),
    ("right_elbow_pitch_joint", 30.492, 4.655, 0.15579, 27.0),
    ("right_forearm_yaw_joint", 30.492, 4.655, 0.04986, 27.0),
    ("right_wrist_pitch_joint", 20.0, 1.0, 0.01933, 7.0),
    ("right_wrist_roll_joint", 20.0, 1.0, 0.00268, 7.0),
];

/// The MIT frame's damping range: kd at most this (`dm_can.rs`, 12 bits over 0..5).
const KD_MAX: f64 = 5.0;

/// A joint's gains retuned toward damping ratio `zeta` (never less damped than commissioned):
/// kd raised to reach it, at most [`KD_MAX`]; where that is not enough, kp lowered until it is.
fn retuned(kp: f64, kd: f64, inertia: f64, zeta: Option<f64>) -> (f64, f64) {
    let Some(zeta) = zeta else {
        return (kp, kd);
    };
    let want = 2.0 * zeta * (kp * inertia).sqrt();
    if want <= kd {
        return (kp, kd);
    }
    if want <= KD_MAX {
        return (kp, want);
    }
    let kp = (KD_MAX / (2.0 * zeta)).powi(2) / inertia;
    (kp, KD_MAX)
}

/// The gains of `name` on `m` as the station drives it: `(kp, kd, inertia, peak)`.
fn gains(m: &Motor, name: &str) -> Option<(f64, f64, f64, f64)> {
    SPARK
        .iter()
        .find(|(n, ..)| *n == name)
        .map(|&(_, kp, kd, j, peak)| {
            let (kp, kd) = if name == "torso_joint" || name.contains("shoulder") {
                retuned(kp, kd, j, m.zeta)
            } else {
                (kp, kd)
            };
            (kp, kd, j, peak)
        })
}

/// The simulated robot.
#[derive(Clone, Copy, Debug)]
struct Motor {
    /// The Spark's followers with omega times this; or, with `generic`, every joint (omega,
    /// zeta), shoulders 0.3x and wrists 2x when `hetero` (the earlier attack cases).
    scale: f64,
    generic: Option<(f64, f64, bool)>,
    /// Seconds each command takes to reach the motors.
    delay: f64,
    /// Planned velocity fed forward (the station sends it).
    feedforward: bool,
    /// Each arm joint resting short of its command by this many of [`sag`].
    sag: f64,
    /// Each reading off by up to this, radians.
    noise: f64,
    /// The torso's and shoulders' gains retuned toward this damping ratio ([`retuned`]).
    zeta: Option<f64>,
}

const SPARK_MOTOR: Motor = Motor {
    scale: 1.0,
    generic: None,
    delay: 0.045,
    feedforward: true,
    sag: 0.0,
    noise: 0.0,
    zeta: None,
};

/// act-sim's left arm at the end of a 0.23 m move (2026-11-03).
fn sag(name: &str) -> f64 {
    if name.contains("finger") || name.starts_with("so101") {
        0.0
    } else if name.contains("shoulder_roll") {
        0.0075
    } else if name.contains("elbow") {
        0.0055
    } else if name.contains("shoulder_pitch") {
        0.0024
    } else {
        0.003
    }
}

/// (omega, zeta, velocity cap) of `name` on `m`; the Feetech neck and grippers stiff.
fn follower(m: &Motor, name: &str) -> (f64, f64, f64) {
    if let Some((w, z, hetero)) = m.generic {
        let w = if !hetero {
            w
        } else if name.contains("shoulder") {
            w * 0.3
        } else if name.contains("elbow") {
            w
        } else {
            w * 2.0
        };
        return (w, z, f64::INFINITY);
    }
    gains(m, name).map_or((30.0, 0.7, f64::INFINITY), |(kp, kd, j, _)| {
        // The real joint's inertia off the registry's by the scale's square.
        let j = j / (m.scale * m.scale);
        let omega = (kp / j).sqrt();
        (omega, kd / (2.0 * (kp * j).sqrt()), 0.1 * kp / kd)
    })
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Scene {
    /// The left hand asked 6 cm down onto a counter 3 cm under it, sliding `slide` m in y
    /// after 2 s; a left arm joint (ordinal) blocked past rest one way.
    Counter {
        slide: f64,
        block: Option<(usize, f64)>,
    },
    /// The right hand reaching for where the left one is, the left elbow on a stop.
    Reach { stopped: bool },
    /// The left hand swept inward past a keep-out box beside its forearm.
    Box { stopped: bool },
    /// The left hand sent to the right hand, at once or over 2 s.
    Cross { sweep: bool },
    /// The left hand swept across the chest to the right and back, 1.5 s each way.
    Chest,
    /// The desk operator's hand path (`follow_lag.rs`), followed for 8 s.
    Desk,
}

#[derive(Debug, Default)]
struct Out {
    /// The least clearance where the robot really is, metres, and between what.
    clear: f64,
    pair: String,
    /// The same against the links' convex pieces (the parts' own shape, inside their capsules)
    /// and the boxes: positive the least vertex distance, negative the deepest vertex inside a
    /// box. A negative number is the part itself in the box.
    real: f64,
    /// The fastest a speed-bound tool point was commanded, as a multiple of its speed.
    tool_x: f64,
    /// When, seconds, and how the step said it went.
    tool_t: f64,
    tool_solve: Option<Solve>,
    /// The fastest the real tool point moved over a tick, as a multiple of its speed.
    real_x: f64,
    /// After the first second, the real tool point's rms and peak distance from its target,
    /// metres; over the last second, the commanded tool point's spread (a cycle), metres.
    rms: f64,
    peak: f64,
    spread: f64,
    held: usize,
    relaxed: usize,
}

/// Each link's convex pieces' vertices, link frame, by link index.
fn pieces(teleop: &Teleop) -> Vec<(usize, Vec<[f64; 3]>)> {
    #[derive(serde::Deserialize)]
    struct Hull {
        link: String,
        points: Vec<[f64; 3]>,
    }
    #[derive(serde::Deserialize)]
    struct Proxies {
        hulls: Vec<Hull>,
    }
    let proxies: Proxies =
        serde_json::from_str(STATIC_UPPER_BODY_COLLISION).expect("the collision proxies");
    proxies
        .hulls
        .into_iter()
        .filter_map(|h| {
            let l = teleop.model().links().iter().position(|l| *l == h.link)?;
            Some((l, h.points))
        })
        .collect()
}

/// [`Out::real`] at `q`: `link`'s pieces' vertices against every box (a link the solve does
/// not check against a box, one inside it at rest, is not asked).
fn real_clearance(
    teleop: &Teleop,
    pieces: &[(usize, Vec<[f64; 3]>)],
    boxes: &[[f64; 6]],
    q: &[f64],
    link: usize,
) -> f64 {
    let frames = teleop.model().poses(q);
    let mut least = f64::INFINITY;
    for (l, points) in pieces.iter().filter(|(l, _)| *l == link) {
        let f = frames.links[*l];
        for p in points {
            let w: [f64; 3] = [0, 1, 2].map(|i| {
                f.r[i][0].mul_add(p[0], f.r[i][1].mul_add(p[1], f.r[i][2] * p[2])) + f.t[i]
            });
            for b in boxes {
                let out: f64 = (0..3)
                    .map(|k| (b[k] - w[k]).max(w[k] - b[k + 3]).max(0.0).powi(2))
                    .sum::<f64>()
                    .sqrt();
                let d = if out > 0.0 {
                    out
                } else {
                    -(0..3)
                        .map(|k| (w[k] - b[k]).min(b[k + 3] - w[k]))
                        .fold(f64::INFINITY, f64::min)
                };
                least = least.min(d);
            }
        }
    }
    least
}

fn link(teleop: &Teleop, name: &str) -> usize {
    teleop
        .model()
        .links()
        .iter()
        .position(|l| l == name)
        .expect(name)
}

fn pose_at(teleop: &Teleop, q: &[f64], name: &str) -> Pose {
    let frame = teleop.model().poses(q).links[link(teleop, name)];
    let p = sx_embodiment_drivers::kinematics::pose_of(&frame.to_transform());
    Pose {
        translation_m: frame.t,
        rotation_xyzw: p.rotation_xyzw,
    }
}

fn base(obstacles: Vec<[f64; 6]>, m: &Motor) -> (Teleop, Vec<f64>) {
    let mut config = TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json"))
        .expect("the station's solve config");
    config.obstacles = obstacles;
    // The station's caps: each Damiao joint's held to its gains' tracking ceilings
    // (`DamiaoChain::tracking_ceilings`), on the registry's inertia.
    for (name, speed) in &mut config.speed {
        if let Some((kp, kd, j, peak)) = gains(m, name) {
            *speed = speed
                .min(0.2 * kp / kd)
                .min(0.1 * (kp / j).sqrt())
                .min((0.2 * peak / j).sqrt());
            let accel = config.accel.entry(name.clone()).or_insert(f64::INFINITY);
            *accel = accel.min(0.1 * kp / j);
        }
    }
    let ready: BTreeMap<String, f64> =
        serde_json::from_str(include_str!("fixtures/static-ready.json")).expect("ready");
    let mut teleop =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).expect("solve");
    teleop.set_deadline(std::time::Duration::from_secs(3600));
    let rest = teleop
        .model()
        .axes()
        .iter()
        .map(|a| ready.get(&a.name).copied().unwrap_or(0.0))
        .collect();
    (teleop, rest)
}

fn counter(clear: f64) -> Vec<[f64; 6]> {
    let (teleop, rest) = base(Vec::new(), &SPARK_MOTOR);
    let t = pose_at(&teleop, &rest, "left_gripper_tcp").translation_m;
    let at = |below: f64| {
        vec![[
            t[0] - 0.1,
            t[1] - 0.08,
            t[2] - below - 0.3,
            t[0] + 0.1,
            t[1] + 0.08,
            t[2] - below,
        ]]
    };
    let mut below = 0.0;
    loop {
        let (probe, _) = base(at(below), &SPARK_MOTOR);
        let mut frames = probe.model().poses(&rest);
        if probe.ik().clearance_into(&rest, &mut frames).scene_m >= clear {
            return at(below);
        }
        below += 0.001;
    }
}

fn run(scene: Scene, m: Motor, dt: f64, tool_speed: Option<f64>) -> Out {
    let obstacles = match scene {
        Scene::Counter { .. } => counter(0.03),
        Scene::Box { .. } => {
            let (probe, rest) = base(Vec::new(), &SPARK_MOTOR);
            let f = probe.model().poses(&rest).links[link(&probe, "left_forearm_yaw_link")].t;
            vec![[
                f[0] - 0.06,
                f[1] - 0.16,
                f[2] - 0.06,
                f[0] + 0.06,
                f[1] - 0.09,
                f[2] + 0.06,
            ]]
        }
        _ => Vec::new(),
    };
    let (mut teleop, rest) = base(obstacles.clone(), &m);
    let pieces = pieces(&teleop);
    let names: Vec<String> = teleop
        .model()
        .axes()
        .iter()
        .map(|a| a.name.clone())
        .collect();
    let width = names.len();
    let bias: Vec<f64> = names.iter().map(|n| m.sag * sag(n)).collect();
    let gains: Vec<(f64, f64, f64)> = names.iter().map(|n| follower(&m, n)).collect();
    let index = |name: &str| names.iter().position(|n| n == name).expect(name);
    let left_arm: Vec<usize> = names
        .iter()
        .enumerate()
        .filter(|(_, n)| n.starts_with("left_") && !n.contains("finger"))
        .map(|(i, _)| i)
        .collect();
    let elbow = index("left_elbow_pitch_joint");
    let blocked = match scene {
        Scene::Counter { block, .. } => block.map(|(k, s)| (left_arm[k % left_arm.len()], s)),
        Scene::Reach { stopped } | Scene::Box { stopped } => stopped.then_some((elbow, -1.0)),
        _ => None,
    };
    let left0 = pose_at(&teleop, &rest, "left_gripper_tcp");
    let right0 = pose_at(&teleop, &rest, "right_gripper_tcp");
    let seconds = match scene {
        Scene::Counter { .. } | Scene::Desk => 8.0,
        _ => 5.5,
    };
    let ticks = (seconds / dt) as usize;
    let available = vec![true; width];
    let (mut reference, mut real, mut v) = (rest.clone(), rest.clone(), vec![0.0; width]);
    // Commands in flight: (when they reach the motors, position, planned velocity).
    let mut flight: VecDeque<(f64, Vec<f64>, Vec<f64>)> = VecDeque::new();
    let (mut command, mut qdot) = (rest.clone(), vec![0.0; width]);
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut rnd = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        2.0 * ((state >> 11) as f64 / (1_u64 << 53) as f64) - 1.0
    };
    let mut out = Out {
        clear: f64::INFINITY,
        real: f64::INFINITY,
        ..Out::default()
    };
    let tcps = [
        link(&teleop, "left_gripper_tcp"),
        link(&teleop, "right_gripper_tcp"),
    ];
    let turn = match scene {
        Scene::Reach { .. } | Scene::Box { .. } | Scene::Cross { .. } | Scene::Chest => 1.5,
        Scene::Counter { .. } | Scene::Desk => 0.0,
    };
    let mut was_stalled = vec![false; width];
    let (mut errors, mut commanded) = (Vec::new(), Vec::new());
    teleop.reset();
    for tick in 0..ticks {
        let t = tick as f64 * dt;
        let mut targets = Targets::default();
        let at = |offset: [f64; 3]| Pose {
            translation_m: [0, 1, 2].map(|k| left0.translation_m[k] + offset[k]),
            rotation_xyzw: left0.rotation_xyzw,
        };
        let toward_right = |k: f64| Pose {
            translation_m: [0, 1, 2].map(|i| {
                left0.translation_m[i] + k * (right0.translation_m[i] - left0.translation_m[i])
            }),
            rotation_xyzw: left0.rotation_xyzw,
        };
        let since = (t - turn).max(0.0);
        targets.tcp[0] = Some(match scene {
            Scene::Counter { slide, .. } => {
                let y = if t > 2.0 {
                    slide * ((t - 2.0) / 2.0).min(1.0)
                } else {
                    0.0
                };
                at([0.0, y, -0.06])
            }
            Scene::Reach { .. } => at([0.03, 0.0, -0.04]),
            Scene::Box { .. } => {
                let inward = if t >= turn {
                    -0.25 * (since / 2.0).min(1.0)
                } else {
                    0.0
                };
                at([0.03, inward, -0.04])
            }
            Scene::Cross { sweep } => toward_right(if t < turn {
                0.0
            } else if sweep {
                (since / 2.0).min(1.0)
            } else {
                1.0
            }),
            Scene::Chest => {
                let k = (since / 1.5).min(2.0);
                toward_right(if k <= 1.0 { k } else { 2.0 - k })
            }
            Scene::Desk => {
                use std::f64::consts::TAU;
                at([
                    0.0,
                    0.05 * (TAU * 0.5 * t).sin(),
                    0.03 * (TAU * 0.25 * t).sin(),
                ])
            }
        });
        let goal = targets.tcp[0].expect("a goal").translation_m;
        if matches!(scene, Scene::Reach { .. }) && t >= turn {
            targets.tcp[1] = Some(Pose {
                translation_m: pose_at(&teleop, &real, "left_gripper_tcp").translation_m,
                rotation_xyzw: right0.rotation_xyzw,
            });
        }
        targets.tool_speed_m_s = [tool_speed; 2];
        let measured: Vec<Option<f64>> = real.iter().map(|x| Some(x + m.noise * rnd())).collect();
        let before = teleop.model().poses(&reference);
        let step = teleop
            .step(&reference, &measured, &available, &targets, dt)
            .expect("a step")
            .clone();
        out.held += usize::from(step.solve == Solve::Held);
        out.relaxed += usize::from(step.solve == Solve::Relaxed);
        // A stalled joint's jump to where it stands, or its release, moves no real arm: not a
        // step of the tool.
        let stall_change = step.stalled.iter().zip(&was_stalled).any(|(a, b)| a != b);
        was_stalled.copy_from_slice(&step.stalled);
        if let Some(speed) = tool_speed.filter(|_| !stall_change && tick > 0) {
            let after = teleop.model().poses(&step.q);
            for &l in &tcps {
                let d = (0..3)
                    .map(|k| (after.links[l].t[k] - before.links[l].t[k]).powi(2))
                    .sum::<f64>()
                    .sqrt();
                if d / dt / speed > out.tool_x {
                    out.tool_x = d / dt / speed;
                    out.tool_t = t;
                    out.tool_solve = Some(step.solve);
                }
            }
        }
        let real_before = teleop.model().poses(&real);
        reference.copy_from_slice(&step.q);
        flight.push_back((t + m.delay, step.q.clone(), step.qdot.clone()));
        let h = dt / SUBSTEPS as f64;
        for s in 1..=SUBSTEPS {
            let now = t + s as f64 * h;
            while flight.front().is_some_and(|(at, ..)| *at <= now + 1e-9) {
                let (_, q, qd) = flight.pop_front().expect("a command");
                command = q;
                qdot = qd;
            }
            for j in 0..width {
                let (w, z, cap) = gains[j];
                let ff = if m.feedforward {
                    qdot[j].clamp(-cap, cap)
                } else {
                    0.0
                };
                let a = w * w * (command[j] - bias[j] - real[j]) + 2.0 * z * w * (ff - v[j]);
                v[j] += a * h;
                real[j] += v[j] * h;
                if let Some((b, sgn)) = blocked
                    && j == b
                    && (real[j] - rest[j]) * sgn > 0.0
                {
                    real[j] = rest[j];
                    v[j] = 0.0;
                }
            }
        }
        if let Some(speed) = tool_speed {
            let after = teleop.model().poses(&real);
            for &l in &tcps {
                let d = (0..3)
                    .map(|k| (after.links[l].t[k] - real_before.links[l].t[k]).powi(2))
                    .sum::<f64>()
                    .sqrt();
                out.real_x = out.real_x.max(d / dt / speed);
            }
        }
        if t >= 1.0 {
            let at = pose_at(&teleop, &real, "left_gripper_tcp").translation_m;
            let e = (0..3)
                .map(|k| (at[k] - goal[k]).powi(2))
                .sum::<f64>()
                .sqrt();
            errors.push(e);
        }
        if t >= seconds - 1.0 {
            commanded.push(pose_at(&teleop, &reference, "left_gripper_tcp").translation_m);
        }
        if t >= turn {
            let (c, a, b) = teleop.ik().clearance(&real);
            if c < out.clear {
                out.clear = c;
                out.pair = format!("{a} / {b}");
            }
            if b.starts_with("obstacle") && c < 0.01 {
                let l = teleop
                    .model()
                    .links()
                    .iter()
                    .position(|n| *n == a)
                    .expect("a link");
                out.real = out
                    .real
                    .min(real_clearance(&teleop, &pieces, &obstacles, &real, l));
            }
        }
    }
    out.rms = (errors.iter().map(|e| e * e).sum::<f64>() / errors.len().max(1) as f64).sqrt();
    out.peak = errors.iter().copied().fold(0.0, f64::max);
    out.spread = (0..3)
        .map(|k| {
            let (lo, hi) = commanded
                .iter()
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), p| {
                    (l.min(p[k]), h.max(p[k]))
                });
            hi - lo
        })
        .fold(0.0, f64::max);
    out
}

fn line(scene: Scene, m: &Motor, dt: f64, tool: Option<f64>, o: &Out) -> String {
    format!(
        "{scene:?} dt {dt} scale {} zeta {:?} generic {:?} delay {} ff {} sag {} noise {} tool {tool:?}: \
         clear {:.2} mm ({}), real {:.2} mm, tool x{:.2} at {:.2} s {:?}, real x{:.2}, held {} relaxed {}, rms {:.1} peak {:.1} spread {:.1} mm",
        m.scale,
        m.zeta,
        m.generic,
        m.delay,
        m.feedforward,
        m.sag,
        m.noise,
        o.clear * 1e3,
        o.pair,
        o.real * 1e3,
        o.tool_x,
        o.tool_t,
        o.tool_solve,
        o.real_x,
        o.held,
        o.relaxed,
        o.rms * 1e3,
        o.peak * 1e3,
        o.spread * 1e3
    )
}

/// Every case on a pool of threads; each line printed, and the worst kept.
fn table(cases: &[(Scene, Motor, f64, Option<f64>)]) -> Vec<(String, Out)> {
    let next = AtomicUsize::new(0);
    let done = Mutex::new(Vec::new());
    let workers = std::thread::available_parallelism().map_or(4, std::num::NonZero::get);
    std::thread::scope(|scope| {
        for _ in 0..workers {
            scope.spawn(|| {
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    let Some(&(scene, m, dt, tool)) = cases.get(i) else {
                        break;
                    };
                    let o = run(scene, m, dt, tool);
                    let l = line(scene, &m, dt, tool, &o);
                    println!("ROW {l}");
                    done.lock().expect("rows").push((l, o));
                }
            });
        }
    });
    done.into_inner().expect("rows")
}

fn scenes() -> Vec<Scene> {
    let mut s = vec![
        Scene::Reach { stopped: true },
        Scene::Reach { stopped: false },
        Scene::Box { stopped: true },
        Scene::Box { stopped: false },
        Scene::Cross { sweep: false },
        Scene::Cross { sweep: true },
        Scene::Chest,
    ];
    for slide in [0.0, 0.15, -0.15, 0.08, -0.08] {
        s.push(Scene::Counter { slide, block: None });
    }
    for k in 0..5 {
        for sign in [1.0, -1.0] {
            s.push(Scene::Counter {
                slide: 0.0,
                block: Some((k, sign)),
            });
        }
    }
    s
}

fn spark_motors() -> Vec<Motor> {
    let mut ms = Vec::new();
    for scale in [0.8, 1.0, 1.25] {
        for delay in [0.026, 0.045, 0.063] {
            for feedforward in [true, false] {
                for (sag, noise) in [
                    (0.0, 0.0),
                    (0.0, READING_JITTER),
                    (1.0, READING_JITTER),
                    (-1.0, READING_JITTER),
                ] {
                    ms.push(Motor {
                        scale,
                        delay,
                        feedforward,
                        sag,
                        noise,
                        ..SPARK_MOTOR
                    });
                }
            }
        }
    }
    ms
}

fn worst(rows: &[(String, Out)]) {
    let w = rows
        .iter()
        .min_by(|a, b| a.1.clear.total_cmp(&b.1.clear))
        .expect("a row");
    let t = rows
        .iter()
        .max_by(|a, b| a.1.tool_x.total_cmp(&b.1.tool_x))
        .expect("a row");
    let held: usize = rows.iter().map(|r| r.1.held).sum();
    let relaxed: usize = rows.iter().map(|r| r.1.relaxed).sum();
    println!(
        "WORST clear {:.2} mm: {}\nWORST tool x{:.2}: {}\nTOTAL held {held} relaxed {relaxed} over {} rows",
        w.1.clear * 1e3,
        w.0,
        t.1.tool_x,
        t.0,
        rows.len()
    );
}

/// The Spark's followers across the band, at 50 and 90 Hz, no tool speed (teleop).
#[test]
#[ignore = "prints the table, about 10 min in release"]
fn table_spark() {
    let mut cases = Vec::new();
    for dt in [0.02, 0.011] {
        for scene in scenes() {
            for m in spark_motors() {
                cases.push((scene, m, dt, None));
            }
        }
    }
    worst(&table(&cases));
}

/// The earlier attack followers (generic PDs, the ringing shoulders), for comparison.
#[test]
#[ignore = "prints the table"]
fn table_generic() {
    let mut cases = Vec::new();
    for dt in [0.02, 0.011] {
        for scene in scenes() {
            for (w, z, hetero) in [(9.5, 0.2, true), (6.0, 0.15, false), (30.0, 0.7, false)] {
                for noise in [0.0, READING_JITTER] {
                    for delay in [0.0, 0.045] {
                        cases.push((
                            scene,
                            Motor {
                                generic: Some((w, z, hetero)),
                                delay,
                                feedforward: false,
                                noise,
                                ..SPARK_MOTOR
                            },
                            dt,
                            None,
                        ));
                    }
                }
            }
        }
    }
    worst(&table(&cases));
}

/// A program's goals at a tool speed: the Spark's followers and the generic ones.
#[test]
#[ignore = "prints the table"]
fn table_tool_speed() {
    let mut cases = Vec::new();
    for dt in [0.02, 0.011, 0.004] {
        for scene in scenes() {
            for tool in [0.05, 0.2] {
                for m in [
                    SPARK_MOTOR,
                    Motor {
                        noise: READING_JITTER,
                        sag: 1.0,
                        ..SPARK_MOTOR
                    },
                    Motor {
                        zeta: Some(0.5),
                        ..SPARK_MOTOR
                    },
                    Motor {
                        noise: READING_JITTER,
                        sag: 1.0,
                        zeta: Some(0.5),
                        ..SPARK_MOTOR
                    },
                    Motor {
                        generic: Some((9.5, 0.1, false)),
                        delay: 0.0,
                        feedforward: false,
                        noise: READING_JITTER,
                        ..SPARK_MOTOR
                    },
                    Motor {
                        generic: Some((9.5, 0.1, false)),
                        delay: 0.011,
                        feedforward: false,
                        ..SPARK_MOTOR
                    },
                ] {
                    cases.push((scene, m, dt, Some(tool)));
                }
            }
        }
    }
    worst(&table(&cases));
}

/// The torso and shoulders retuned toward a damping ratio (`retuned`), against the commissioned
/// gains: the Spark's followers across the band, 50 and 90 Hz, no tool speed.
#[test]
#[ignore = "prints the table, about 6 min in release"]
fn table_gains() {
    let mut cases = Vec::new();
    for dt in [0.02, 0.011] {
        let mut scenes = scenes();
        scenes.push(Scene::Desk);
        for scene in scenes {
            for zeta in [None, Some(0.5), Some(0.7)] {
                for scale in [0.8, 1.0, 1.25] {
                    for delay in [0.026, 0.045, 0.063] {
                        for (sag, noise) in [(0.0, 0.0), (1.0, READING_JITTER)] {
                            cases.push((
                                scene,
                                Motor {
                                    scale,
                                    delay,
                                    sag,
                                    noise,
                                    zeta,
                                    ..SPARK_MOTOR
                                },
                                dt,
                                None,
                            ));
                        }
                    }
                }
            }
        }
    }
    worst(&table(&cases));
}
