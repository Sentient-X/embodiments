//! The solve on the static upper body as the DGX Spark drives it: each Damiao joint a follower
//! of its commissioned MIT gains (`sx-damiao-can` `static_upper_body.rs` `JointGains`) over the
//! inertia its bindings carry, its planned velocity fed forward as the MIT law's `kd (v_des -
//! v)` (capped at `MAX_COAST kp / kd`), each command reaching the arm a transport delay late
//! (the Spark's arms trail their commands by 26-63 ms, 2026-10-27..30 journals). The followers
//! are act-sim's own (`act-sim` identified the ringing shoulders within 0.5 % in omega and 4 % in
//! zeta), scaled across a band for the inertia's posture dependence.
//!
//! Measured on main ad190f0 (2026-11-03), at 50 / 90 Hz across scale 0.8-1.25, 26-63 ms,
//! feed-forward on and off, sag and 1 mrad jitter (`table_spark`, 3,168 runs, 0 Held): the nominal model
//! (scale 1) keeps 2.24 mm at 50 Hz and touches at 90 Hz (-0.70 mm, the left elbow into the box
//! at 63 ms); at 1.25x, -7.8 mm at 50 Hz and -17.2 mm at 90 Hz (a finger into the counter). The
//! cases with a tool speed (`table_tool_speed`): the command's tool point up to 22-28x its
//! 0.05 m/s and the real one up to 17-27x. No bar yet: main does not meet one on these
//! followers (`docs/plans/ik-follower-model-2026-11.md` in sx, section 10).
//!
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

/// Each Damiao joint: (omega rad/s, zeta) of its MIT gains over the registry's inertia at the
/// joint (rotor through the reduction squared included), and the planned-velocity cap
/// `MAX_COAST kp / kd` (rad/s). Printed by embodiments `ik/follower-model`'s
/// `every_damiao_joint_has_a_follower_the_solve_can_plan_on`, 2026-11-03.
const SPARK: [(&str, f64, f64, f64); 15] = [
    ("torso_joint", 5.03, 0.063, 0.1 * 40.0 / 1.0),
    (
        "left_shoulder_pitch_joint",
        12.11,
        0.108,
        0.1 * 60.984 / 1.091,
    ),
    (
        "left_shoulder_roll_joint",
        11.61,
        0.104,
        0.1 * 60.984 / 1.091,
    ),
    ("left_upper_arm_yaw_joint", 12.39, 1.291, 0.1 * 24.0 / 5.0),
    ("left_elbow_pitch_joint", 13.99, 1.068, 0.1 * 30.492 / 4.655),
    ("left_forearm_yaw_joint", 24.12, 1.973, 0.1 * 29.04 / 4.75),
    ("left_wrist_pitch_joint", 32.17, 0.804, 0.1 * 20.0 / 1.0),
    ("left_wrist_roll_joint", 86.26, 2.156, 0.1 * 20.0 / 1.0),
    (
        "right_shoulder_pitch_joint",
        12.31,
        0.110,
        0.1 * 60.984 / 1.091,
    ),
    ("right_shoulder_roll_joint", 9.93, 0.116, 0.1 * 42.0 / 0.98),
    (
        "right_upper_arm_yaw_joint",
        13.98,
        1.067,
        0.1 * 30.492 / 4.655,
    ),
    (
        "right_elbow_pitch_joint",
        13.99,
        1.068,
        0.1 * 30.492 / 4.655,
    ),
    (
        "right_forearm_yaw_joint",
        24.73,
        1.888,
        0.1 * 30.492 / 4.655,
    ),
    ("right_wrist_pitch_joint", 32.17, 0.804, 0.1 * 20.0 / 1.0),
    ("right_wrist_roll_joint", 86.42, 2.161, 0.1 * 20.0 / 1.0),
];

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
}

const SPARK_MOTOR: Motor = Motor {
    scale: 1.0,
    generic: None,
    delay: 0.045,
    feedforward: true,
    sag: 0.0,
    noise: 0.0,
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
    SPARK
        .iter()
        .find(|(n, ..)| *n == name)
        .map_or((30.0, 0.7, f64::INFINITY), |&(_, w, z, cap)| {
            (w * m.scale, z, cap)
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
}

#[derive(Debug, Default)]
struct Out {
    /// The least clearance where the robot really is, metres, and between what.
    clear: f64,
    pair: String,
    /// The fastest a speed-bound tool point was commanded, as a multiple of its speed.
    tool_x: f64,
    /// When, seconds, and how the step said it went.
    tool_t: f64,
    tool_solve: Option<Solve>,
    /// The fastest the real tool point moved over a tick, as a multiple of its speed.
    real_x: f64,
    held: usize,
    relaxed: usize,
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

fn base(obstacles: Vec<[f64; 6]>) -> (Teleop, Vec<f64>) {
    let mut config = TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json"))
        .expect("the station's solve config");
    config.obstacles = obstacles;
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
    let (teleop, rest) = base(Vec::new());
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
        let (probe, _) = base(at(below));
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
            let (probe, rest) = base(Vec::new());
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
    let (mut teleop, rest) = base(obstacles);
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
        Scene::Counter { .. } => 8.0,
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
        ..Out::default()
    };
    let tcps = [
        link(&teleop, "left_gripper_tcp"),
        link(&teleop, "right_gripper_tcp"),
    ];
    let turn = match scene {
        Scene::Reach { .. } | Scene::Box { .. } | Scene::Cross { .. } | Scene::Chest => 1.5,
        Scene::Counter { .. } => 0.0,
    };
    let mut was_stalled = vec![false; width];
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
        });
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
        if t >= turn {
            let (c, a, b) = teleop.ik().clearance(&real);
            if c < out.clear {
                out.clear = c;
                out.pair = format!("{a} / {b}");
            }
        }
    }
    out
}

fn line(scene: Scene, m: &Motor, dt: f64, tool: Option<f64>, o: &Out) -> String {
    format!(
        "{scene:?} dt {dt} scale {} generic {:?} delay {} ff {} sag {} noise {} tool {tool:?}: \
         clear {:.2} mm ({}), tool x{:.2} at {:.2} s {:?}, real x{:.2}, held {} relaxed {}",
        m.scale,
        m.generic,
        m.delay,
        m.feedforward,
        m.sag,
        m.noise,
        o.clear * 1e3,
        o.pair,
        o.tool_x,
        o.tool_t,
        o.tool_solve,
        o.real_x,
        o.held,
        o.relaxed
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
