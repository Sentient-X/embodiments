//! The solve on a commissioned follower model ([`sx_embodiment_ik::Ik::set_followers`]): the
//! static upper body driven by simulated followers, its pairs kept where the arm will be.
//!
//! Every scene the measured-pose row was attacked with (`docs/plans/ik-follower-model-2026-11.md`
//! in sx): a target held past a counter, a long paced move, joints of different speeds, a
//! blocked joint, sag, a reading's jitter, calibration offsets, and arm against arm with soft
//! followers. `cargo test --release --test follower_model -- --ignored --nocapture` prints the
//! table; the bars are the tests that are not ignored.

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::too_many_lines,
    clippy::too_many_arguments,
    clippy::struct_excessive_bools,
    clippy::many_single_char_names,
    clippy::inconsistent_struct_constructor
)]

use std::collections::{BTreeMap, VecDeque};
use std::time::Instant;

use sx_embodiment_ik::teleop::{Pose, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{
    Active, READING_JITTER, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF, Solve,
};

const SUBSTEPS: usize = 10;

/// The simulated robot.
#[derive(Clone, Copy, Debug)]
struct Motor {
    omega: f64,
    zeta: f64,
    /// Ticks each command waits before the follower sees it.
    late: usize,
    /// Each arm joint resting short of its command by this many of [`sag`].
    sag: f64,
    /// Each reading off by up to this, radians, and a constant per-joint offset up to `cal`.
    noise: f64,
    cal: f64,
    /// Shoulders 0.3x, wrists 2x the natural frequency.
    hetero: bool,
    /// The command's velocity fed forward (as the MIT law's `kd (v_des - v)`).
    feedforward: bool,
    /// The solve's model of it: none, the motor itself, or off by this factor in omega.
    model: Model,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Model {
    None,
    Exact,
    Off(f64),
}

const fn motor(omega: f64, zeta: f64) -> Motor {
    Motor {
        omega,
        zeta,
        late: 0,
        sag: 0.0,
        noise: 0.0,
        cal: 0.0,
        hetero: false,
        feedforward: false,
        model: Model::Exact,
    }
}

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

fn omega_of(m: &Motor, name: &str) -> f64 {
    if !m.hetero {
        m.omega
    } else if name.contains("shoulder") {
        m.omega * 0.3
    } else if name.contains("elbow") {
        m.omega
    } else {
        m.omega * 2.0
    }
}

/// The scene.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Scene {
    /// The left hand asked 6 cm down onto a counter 3 cm under it, sliding `slide` m in y
    /// after 2 s; a left arm joint (ordinal) blocked past rest one way.
    Counter {
        slide: f64,
        block: Option<(usize, f64)>,
    },
    /// The 0.236 m move to (0.45, 0.02, 0.30) at 0.283 m/s.
    Paced,
    /// The right hand reaching for where the left one is, the left pushing its elbow into a
    /// stop (`stopped`).
    Reach {
        stopped: bool,
    },
    /// The left hand swept inward past a keep-out box beside its forearm.
    Box {
        stopped: bool,
    },
    /// The left hand sent to the right hand, at once or over 2 s.
    Cross {
        sweep: bool,
    },
    /// The desk or VR operator's hand path, followed.
    Desk,
    Vr,
}

#[derive(Debug, Default)]
struct Out {
    /// The least clearance where the robot really is, body and scene, metres, and between what.
    clear: f64,
    pair: String,
    /// Over the last second: the commanded and the real tool point's spread, metres, and ticks
    /// whose binding pairs changed.
    command_spread: f64,
    real_spread: f64,
    toggles: usize,
    /// At the end, the real tool's distance from its target, metres, and turn, radians.
    error_m: f64,
    error_rad: f64,
    /// Tracking: rms and peak tool error after a second, metres.
    rms: f64,
    peak: f64,
    held: usize,
    relaxed: usize,
    times_ms: Vec<f64>,
}

fn base(obstacles: Vec<[f64; 6]>, m: &Motor) -> (Teleop, Vec<f64>) {
    let mut config = TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json"))
        .expect("the station's solve config");
    config.obstacles = obstacles;
    let names: Vec<String> =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config)
            .expect("solve")
            .model()
            .axes()
            .iter()
            .map(|a| a.name.clone())
            .collect();
    let scale = match m.model {
        Model::None => None,
        Model::Exact => Some(1.0),
        Model::Off(f) => Some(f),
    };
    if let Some(scale) = scale {
        for name in &names {
            config.followers.insert(
                name.clone(),
                sx_embodiment_drivers::FollowerModel {
                    omega: omega_of(m, name) * scale,
                    zeta: m.zeta,
                    feedforward: m.feedforward,
                },
            );
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
        .map(|a| {
            let at = ready.get(&a.name).copied().unwrap_or(0.0);
            if a.name.ends_with("shoulder_pitch_joint") {
                1.5 * at
            } else {
                at
            }
        })
        .collect();
    (teleop, rest)
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

fn counter(clear: f64) -> Vec<[f64; 6]> {
    let (teleop, rest) = base(
        Vec::new(),
        &Motor {
            model: Model::None,
            ..motor(30.0, 0.7)
        },
    );
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
        let (probe, _) = base(
            at(below),
            &Motor {
                model: Model::None,
                ..motor(30.0, 0.7)
            },
        );
        let mut frames = probe.model().poses(&rest);
        if probe.ik().clearance_into(&rest, &mut frames).scene_m >= clear {
            return at(below);
        }
        below += 0.001;
    }
}

fn turned_about_x(angle: f64, q: [f64; 4]) -> [f64; 4] {
    let (tx, tw) = ((angle / 2.0).sin(), (angle / 2.0).cos());
    let [rx, ry, rz, rw] = q;
    [
        tw * rx + tx * rw,
        tw * ry - tx * rz,
        tw * rz + tx * ry,
        tw * rw - tx * rx,
    ]
}

fn run(scene: Scene, m: Motor, dt: f64) -> Out {
    let probe_rest = base(Vec::new(), &m).1;
    let obstacles = match scene {
        Scene::Counter { .. } => counter(0.03),
        Scene::Box { .. } => {
            let (probe, _) = base(Vec::new(), &m);
            let f = probe.model().poses(&probe_rest).links[link(&probe, "left_forearm_yaw_link")].t;
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
    let (mut teleop, rest) = base(obstacles, &m);
    let names: Vec<String> = teleop
        .model()
        .axes()
        .iter()
        .map(|a| a.name.clone())
        .collect();
    let width = names.len();
    let bias: Vec<f64> = names.iter().map(|n| m.sag * sag(n)).collect();
    let omega: Vec<f64> = names.iter().map(|n| omega_of(&m, n)).collect();
    let index = |name: &str| names.iter().position(|n| n == name).expect(name);
    let left_arm: Vec<usize> = names
        .iter()
        .enumerate()
        .filter(|(_, n)| n.starts_with("left_") && !n.contains("finger"))
        .map(|(i, _)| i)
        .collect();
    let elbow = index("left_elbow_pitch_joint");
    let (blocked, stopped) = match scene {
        Scene::Counter { block, .. } => {
            (block.map(|(k, s)| (left_arm[k % left_arm.len()], s)), false)
        }
        Scene::Reach { stopped } | Scene::Box { stopped } => {
            (stopped.then_some((elbow, -1.0)), stopped)
        }
        _ => (None, false),
    };
    let _ = stopped;
    let left0 = pose_at(&teleop, &rest, "left_gripper_tcp");
    let right0 = pose_at(&teleop, &rest, "right_gripper_tcp");
    let seconds = match scene {
        Scene::Counter { .. } => 10.0,
        Scene::Paced => 5.0,
        Scene::Desk | Scene::Vr => 8.0,
        _ => 5.5,
    };
    let ticks = (seconds / dt) as usize;
    let last = ticks.saturating_sub((1.0 / dt) as usize);
    let available = vec![true; width];
    let (mut reference, mut real, mut v) = (rest.clone(), rest.clone(), vec![0.0; width]);
    let mut sent = VecDeque::from(vec![(rest.clone(), vec![0.0; width]); m.late]);
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut rnd = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        2.0 * ((state >> 11) as f64 / (1_u64 << 53) as f64) - 1.0
    };
    let cal: Vec<f64> = (0..width).map(|_| m.cal * rnd()).collect();
    let mut frames = teleop.model().poses(&real);
    let mut out = Out {
        clear: f64::INFINITY,
        ..Out::default()
    };
    let (mut commanded, mut really) = (Vec::new(), Vec::new());
    let mut before: Vec<Active> = Vec::new();
    let mut errors = Vec::new();
    let mut target = left0;
    // The scenes that push or reach start after the stall settles; the rest at once.
    let turn = match scene {
        Scene::Reach { .. } | Scene::Box { .. } | Scene::Cross { .. } => 1.5,
        _ => 0.0,
    };
    teleop.reset();
    for tick in 0..ticks {
        let t = tick as f64 * dt;
        let mut targets = Targets::default();
        match scene {
            Scene::Counter { slide, .. } => {
                let y = if t > 2.0 {
                    slide * ((t - 2.0) / 2.0).min(1.0)
                } else {
                    0.0
                };
                target = Pose {
                    translation_m: [0, 1, 2].map(|k| left0.translation_m[k] + [0.0, y, -0.06][k]),
                    rotation_xyzw: left0.rotation_xyzw,
                };
                targets.tcp[0] = Some(target);
            }
            Scene::Paced => {
                target = Pose {
                    translation_m: [0.45, 0.02, 0.30],
                    rotation_xyzw: left0.rotation_xyzw,
                };
                targets.tcp[0] = Some(target);
                targets.tool_speed_m_s[0] = Some(0.283);
            }
            Scene::Reach { .. } | Scene::Box { .. } => {
                let inward = if matches!(scene, Scene::Box { .. }) && t >= turn {
                    -0.25 * ((t - turn) / 2.0).min(1.0)
                } else {
                    0.0
                };
                target = Pose {
                    translation_m: [0, 1, 2]
                        .map(|k| left0.translation_m[k] + [0.03, inward, -0.04][k]),
                    rotation_xyzw: left0.rotation_xyzw,
                };
                targets.tcp[0] = Some(target);
                if matches!(scene, Scene::Reach { .. }) && t >= turn {
                    targets.tcp[1] = Some(Pose {
                        translation_m: pose_at(&teleop, &real, "left_gripper_tcp").translation_m,
                        rotation_xyzw: right0.rotation_xyzw,
                    });
                }
            }
            Scene::Cross { sweep } => {
                let k = if t < turn {
                    0.0
                } else if sweep {
                    ((t - turn) / 2.0).min(1.0)
                } else {
                    1.0
                };
                target = Pose {
                    translation_m: [0, 1, 2].map(|i| {
                        left0.translation_m[i]
                            + k * (right0.translation_m[i] - left0.translation_m[i])
                    }),
                    rotation_xyzw: left0.rotation_xyzw,
                };
                targets.tcp[0] = Some(target);
            }
            Scene::Desk | Scene::Vr => {
                use std::f64::consts::TAU;
                let (offset, roll) = if scene == Scene::Desk {
                    (
                        [
                            0.0,
                            0.05 * (TAU * 0.5 * t).sin(),
                            0.03 * (TAU * 0.25 * t).sin(),
                        ],
                        0.0,
                    )
                } else {
                    (
                        [0.0, 0.08 * (TAU * t).sin(), 0.08 * ((TAU * t).cos() - 1.0)],
                        15_f64.to_radians() * (TAU * 0.5 * t).sin(),
                    )
                };
                target = Pose {
                    translation_m: [0, 1, 2].map(|k| left0.translation_m[k] + offset[k]),
                    rotation_xyzw: turned_about_x(roll, left0.rotation_xyzw),
                };
                targets.tcp[0] = Some(target);
            }
        }
        let measured: Vec<Option<f64>> = real
            .iter()
            .enumerate()
            .map(|(j, x)| Some(x + m.noise * rnd() + cal[j]))
            .collect();
        let clock = Instant::now();
        let step = teleop
            .step(&reference, &measured, &available, &targets, dt)
            .expect("a step");
        out.times_ms.push(clock.elapsed().as_secs_f64() * 1e3);
        out.held += usize::from(step.solve == Solve::Held);
        out.relaxed += usize::from(step.solve == Solve::Relaxed);
        reference.copy_from_slice(&step.q);
        let mut pairs: Vec<Active> = Vec::new();
        for a in step.limbs.iter().flat_map(|l| &l.active) {
            if matches!(a, Active::Obstacle { .. } | Active::SelfCollision { .. })
                && !pairs.contains(a)
            {
                pairs.push(*a);
            }
        }
        sent.push_back((reference.clone(), step.qdot.clone()));
        let (command, qdot) = sent.pop_front().expect("a command");
        let h = dt / SUBSTEPS as f64;
        for _ in 0..SUBSTEPS {
            for j in 0..width {
                let ff = if m.feedforward { qdot[j] } else { 0.0 };
                let a = omega[j] * omega[j] * (command[j] - bias[j] - real[j])
                    + 2.0 * m.zeta * omega[j] * (ff - v[j]);
                v[j] += a * h;
                real[j] += v[j] * h;
                if let Some((b, s)) = blocked
                    && j == b
                    && (real[j] - rest[j]) * s > 0.0
                {
                    real[j] = rest[j];
                    v[j] = 0.0;
                }
            }
        }
        if t >= turn {
            let (c, a, b) = teleop.ik().clearance(&real);
            if c < out.clear {
                out.clear = c;
                out.pair = format!("{a} / {b}");
            }
        }
        if tick >= last {
            commanded.push(pose_at(&teleop, &reference, "left_gripper_tcp").translation_m);
            really.push(pose_at(&teleop, &real, "left_gripper_tcp").translation_m);
            out.toggles += usize::from(pairs != before);
        }
        before = pairs;
        if t >= 1.0 {
            let at = pose_at(&teleop, &real, "left_gripper_tcp").translation_m;
            errors.push(
                (0..3)
                    .map(|k| (at[k] - target.translation_m[k]).powi(2))
                    .sum::<f64>()
                    .sqrt(),
            );
        }
    }
    let spread = |points: &[[f64; 3]]| {
        (0..3)
            .map(|k| {
                let (lo, hi) = points
                    .iter()
                    .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), p| {
                        (l.min(p[k]), h.max(p[k]))
                    });
                hi - lo
            })
            .fold(0.0, f64::max)
    };
    out.command_spread = spread(&commanded);
    out.real_spread = spread(&really);
    let end = pose_at(&teleop, &real, "left_gripper_tcp");
    out.error_m = (0..3)
        .map(|k| (end.translation_m[k] - target.translation_m[k]).powi(2))
        .sum::<f64>()
        .sqrt();
    let dot: f64 = (0..4)
        .map(|k| end.rotation_xyzw[k] * target.rotation_xyzw[k])
        .sum();
    out.error_rad = 2.0 * dot.abs().min(1.0).acos();
    out.rms = (errors.iter().map(|e| e * e).sum::<f64>() / errors.len().max(1) as f64).sqrt();
    out.peak = errors.iter().copied().fold(0.0, f64::max);
    let _ = &mut frames;
    out
}

fn line(scene: Scene, m: &Motor, dt: f64, o: &Out) -> String {
    let mut times = o.times_ms.clone();
    times.sort_by(f64::total_cmp);
    let p99 = times[times.len() * 99 / 100];
    format!(
        "{scene:?} dt {dt} w {} z {} late {} sag {} noise {} cal {} hetero {} ff {} model {:?}: \
         clear {:.2} mm ({}), spread cmd {:.2} real {:.2} mm, toggles {}, end {:.1} mm {:.3} rad, \
         rms {:.1} peak {:.1} mm, held {} relaxed {}, p99 {:.3} max {:.3} ms",
        m.omega,
        m.zeta,
        m.late,
        m.sag,
        m.noise,
        m.cal,
        m.hetero,
        m.feedforward,
        m.model,
        o.clear * 1e3,
        o.pair,
        o.command_spread * 1e3,
        o.real_spread * 1e3,
        o.toggles,
        o.error_m * 1e3,
        o.error_rad,
        o.rms * 1e3,
        o.peak * 1e3,
        o.held,
        o.relaxed,
        p99,
        times[times.len() - 1]
    )
}

/// Every case, in parallel (timings meaningless) unless SERIAL is set; filtered by the
/// `CASES` env var (a substring of the printed line's scene).
fn table(cases: Vec<(Scene, Motor, f64)>) {
    let serial = std::env::var("SERIAL").is_ok();
    std::thread::scope(|scope| {
        let mut runs = Vec::new();
        for (scene, m, dt) in cases {
            let h = scope.spawn(move || {
                let o = run(scene, m, dt);
                println!("ROW {}", line(scene, &m, dt, &o));
            });
            if serial {
                h.join().expect("a case");
            } else {
                runs.push(h);
            }
        }
        for r in runs {
            r.join().expect("a case");
        }
    });
}

fn models() -> Vec<Model> {
    match std::env::var("MODEL").as_deref() {
        Ok("none") => vec![Model::None],
        Ok("exact") => vec![Model::Exact],
        Ok("off") => vec![Model::Off(0.7), Model::Off(1.4)],
        _ => vec![Model::None, Model::Exact],
    }
}

#[test]
#[ignore = "prints the table"]
fn hold_table() {
    let mut cases = Vec::new();
    for model in models() {
        for dt in [0.02, 0.011, 0.004] {
            for (w, z) in [(30.0, 0.7), (9.5, 0.2)] {
                for sag in [1.0, 0.0, -1.0] {
                    for noise in [0.0, READING_JITTER] {
                        cases.push((
                            Scene::Counter {
                                slide: 0.0,
                                block: None,
                            },
                            Motor {
                                sag,
                                noise,
                                model,
                                ..motor(w, z)
                            },
                            dt,
                        ));
                    }
                }
            }
            for (w, z) in [(30.0, 0.7), (9.5, 0.2)] {
                cases.push((
                    Scene::Paced,
                    Motor {
                        model,
                        ..motor(w, z)
                    },
                    dt,
                ));
            }
        }
    }
    table(cases);
}

#[test]
#[ignore = "prints the table"]
fn attack_table() {
    let mut cases = Vec::new();
    for model in models() {
        for dt in [0.02, 0.011] {
            for (w, z) in [(30.0, 0.7), (9.5, 0.2), (15.0, 1.0)] {
                for slide in [0.15, -0.15, 0.08, -0.08] {
                    for noise in [0.0, READING_JITTER] {
                        cases.push((
                            Scene::Counter { slide, block: None },
                            Motor {
                                hetero: true,
                                noise,
                                model,
                                ..motor(w, z)
                            },
                            dt,
                        ));
                    }
                }
            }
            for k in 0..8 {
                for s in [1.0, -1.0] {
                    for slide in [0.0, 0.1, -0.1] {
                        cases.push((
                            Scene::Counter {
                                slide,
                                block: Some((k, s)),
                            },
                            Motor {
                                model,
                                ..motor(30.0, 0.7)
                            },
                            dt,
                        ));
                    }
                }
            }
            for sag in [3.0, -3.0, 6.0, -6.0] {
                for noise in [0.0, READING_JITTER] {
                    cases.push((
                        Scene::Counter {
                            slide: 0.0,
                            block: None,
                        },
                        Motor {
                            sag,
                            noise,
                            model,
                            ..motor(30.0, 0.7)
                        },
                        dt,
                    ));
                }
            }
            for cal in [0.003, 0.01] {
                for noise in [0.0, READING_JITTER] {
                    cases.push((
                        Scene::Counter {
                            slide: 0.0,
                            block: None,
                        },
                        Motor {
                            cal,
                            noise,
                            model,
                            ..motor(30.0, 0.7)
                        },
                        dt,
                    ));
                }
            }
        }
    }
    table(cases);
}

#[test]
#[ignore = "prints the table"]
fn arms_table() {
    let mut cases = Vec::new();
    for model in models() {
        for dt in [0.02, 0.011] {
            for scene in [
                Scene::Reach { stopped: true },
                Scene::Reach { stopped: false },
                Scene::Box { stopped: true },
                Scene::Box { stopped: false },
                Scene::Cross { sweep: true },
                Scene::Cross { sweep: false },
            ] {
                for (w, z) in [
                    (6.0, 0.15),
                    (8.0, 0.15),
                    (10.0, 0.2),
                    (6.0, 0.2),
                    (9.5, 0.2),
                    (9.5, 0.1),
                    (30.0, 0.7),
                ] {
                    for noise in [0.0, READING_JITTER] {
                        cases.push((
                            scene,
                            Motor {
                                noise,
                                model,
                                ..motor(w, z)
                            },
                            dt,
                        ));
                    }
                }
            }
        }
    }
    table(cases);
}

#[test]
#[ignore = "prints the table"]
fn follow_table() {
    let mut cases = Vec::new();
    for model in models() {
        for scene in [Scene::Desk, Scene::Vr] {
            for (w, z) in [(9.5, 0.2), (30.0, 0.7)] {
                cases.push((
                    scene,
                    Motor {
                        feedforward: true,
                        model,
                        ..motor(w, z)
                    },
                    0.01,
                ));
            }
        }
    }
    table(cases);
}

/// Runs `cases` in parallel and fails with every case `bad` names.
fn bars(cases: Vec<(Scene, Motor, f64)>, bad: fn(&Out) -> bool) {
    let failed: Vec<String> = std::thread::scope(|scope| {
        let runs: Vec<_> = cases
            .into_iter()
            .map(|(scene, m, dt)| {
                scope.spawn(move || {
                    let o = run(scene, m, dt);
                    bad(&o).then(|| line(scene, &m, dt, &o))
                })
            })
            .collect();
        runs.into_iter()
            .filter_map(|run| run.join().expect("a case"))
            .collect()
    });
    assert!(failed.is_empty(), "{failed:#?}");
}

/// A target held past the counter on a stiff follower, the arm sagging either way or not at
/// all, at the static body's 50 Hz and the sessions' 90 Hz under a reading's jitter, and at
/// 250 Hz without: the command and the arm still to within 0.1 mm over the last second, no
/// pair binding and letting go, the arm outside the margin (main cycled 23-88 mm here).
#[test]
fn a_target_held_past_the_counter_holds_still_on_its_follower_model() {
    let mut cases = Vec::new();
    for (dt, noises) in [
        (0.02, &[0.0, READING_JITTER][..]),
        (0.011, &[0.0, READING_JITTER][..]),
        (0.004, &[0.0][..]),
    ] {
        for &noise in noises {
            for sag in [1.0, 0.0, -1.0] {
                cases.push((
                    Scene::Counter {
                        slide: 0.0,
                        block: None,
                    },
                    Motor {
                        sag,
                        noise,
                        ..motor(30.0, 0.7)
                    },
                    dt,
                ));
            }
        }
    }
    bars(cases, |o| {
        o.command_spread > 1e-4 || o.real_spread > 1e-4 || o.toggles > 0 || o.clear < 0.0099
    });
}

/// The same on a follower that rings past its commands (act-sim-like, zeta 0.2): within half a
/// millimetre, never inside the margin (main: 33-82 mm).
#[test]
fn a_ringing_follower_held_past_the_counter_settles_on_its_model() {
    let mut cases = Vec::new();
    for dt in [0.02, 0.011, 0.004] {
        for sag in [1.0, 0.0, -1.0] {
            cases.push((
                Scene::Counter {
                    slide: 0.0,
                    block: None,
                },
                Motor {
                    sag,
                    ..motor(9.5, 0.2)
                },
                dt,
            ));
        }
    }
    bars(cases, |o| {
        o.command_spread > 5e-4 || o.real_spread > 5e-4 || o.clear < 0.0099
    });
}

/// Arm against arm and beside the box with soft followers (natural frequency 6-10 rad/s,
/// damping 0.15-0.2), at 50 and 90 Hz, readings exact or jittering: every pair at least the
/// 10 mm margin apart where the arms really are (main: 8.0 mm, the left elbow at the chest).
#[test]
fn soft_followers_keep_every_pair_10_mm_apart_on_their_model() {
    let mut cases = Vec::new();
    for dt in [0.02, 0.011] {
        for scene in [
            Scene::Reach { stopped: true },
            Scene::Reach { stopped: false },
            Scene::Box { stopped: true },
            Scene::Cross { sweep: true },
            Scene::Cross { sweep: false },
        ] {
            for (w, z) in [(6.0, 0.15), (10.0, 0.2), (6.0, 0.2)] {
                for noise in [0.0, READING_JITTER] {
                    cases.push((
                        scene,
                        Motor {
                            noise,
                            ..motor(w, z)
                        },
                        dt,
                    ));
                }
            }
        }
    }
    bars(cases, |o| o.clear < 0.010);
}

/// The refutes' attacks at 50 and 90 Hz: joints of different speeds sliding along the
/// counter, a blocked joint, sag up to 6x, calibration offsets: at least 10 mm.
#[test]
fn the_attacks_keep_10_mm_on_the_follower_model() {
    let mut cases = Vec::new();
    for dt in [0.02, 0.011] {
        for (w, z) in [(9.5, 0.2), (15.0, 1.0)] {
            for slide in [0.15, -0.15] {
                cases.push((
                    Scene::Counter { slide, block: None },
                    Motor {
                        hetero: true,
                        noise: READING_JITTER,
                        ..motor(w, z)
                    },
                    dt,
                ));
            }
        }
        for k in [0, 1, 3, 5] {
            for s in [1.0, -1.0] {
                cases.push((
                    Scene::Counter {
                        slide: 0.1,
                        block: Some((k, s)),
                    },
                    motor(30.0, 0.7),
                    dt,
                ));
            }
        }
        for sag in [6.0, -6.0] {
            cases.push((
                Scene::Counter {
                    slide: 0.0,
                    block: None,
                },
                Motor {
                    sag,
                    noise: READING_JITTER,
                    ..motor(30.0, 0.7)
                },
                dt,
            ));
        }
        cases.push((
            Scene::Counter {
                slide: 0.0,
                block: None,
            },
            Motor {
                cal: 0.01,
                noise: READING_JITTER,
                ..motor(30.0, 0.7)
            },
            dt,
        ));
    }
    bars(cases, |o| o.clear < 0.010);
}

/// Step time, one case at a time (`SERIAL=1`): the arms against each other, the box and the
/// counter, at every rate, with and without the model.
#[test]
#[ignore = "prints step times; run SERIAL=1 on a quiet host"]
fn timing_table() {
    let mut cases = Vec::new();
    for model in models() {
        for dt in [0.02, 0.011, 0.004, 0.002] {
            for scene in [
                Scene::Reach { stopped: true },
                Scene::Cross { sweep: true },
                Scene::Box { stopped: true },
                Scene::Counter {
                    slide: 0.0,
                    block: None,
                },
            ] {
                cases.push((
                    scene,
                    Motor {
                        noise: READING_JITTER,
                        model,
                        ..motor(9.5, 0.2)
                    },
                    dt,
                ));
            }
        }
    }
    table(cases);
}
