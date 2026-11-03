#![allow(clippy::all, clippy::pedantic)]
use std::collections::BTreeMap;
use sx_embodiment_ik::teleop::{Pose, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};

const SUBSTEPS: usize = 10;

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

fn setup(obstacles: Vec<[f64; 6]>) -> (Teleop, Vec<f64>) {
    let mut config =
        TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json")).unwrap();
    config.obstacles = obstacles;
    let ready: BTreeMap<String, f64> =
        serde_json::from_str(include_str!("fixtures/static-ready.json")).unwrap();
    let mut teleop =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).unwrap();
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

fn left_tcp(teleop: &Teleop, q: &[f64]) -> ([f64; 3], [f64; 4]) {
    let link = teleop
        .model()
        .links()
        .iter()
        .position(|l| l == "left_gripper_tcp")
        .unwrap();
    let frame = teleop.model().poses(q).links[link];
    let pose = sx_embodiment_drivers::kinematics::pose_of(&frame.to_transform());
    (frame.t, pose.rotation_xyzw)
}

fn counter(clear: f64) -> Vec<[f64; 6]> {
    let (teleop, rest) = setup(Vec::new());
    let (t, _) = left_tcp(&teleop, &rest);
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
        let (probe, _) = setup(at(below));
        let mut frames = probe.model().poses(&rest);
        if probe.ik().clearance_into(&rest, &mut frames).scene_m >= clear {
            return at(below);
        }
        below += 0.001;
    }
}

#[derive(Clone, Copy, Debug)]
struct Case {
    dt: f64,
    omega: f64,
    zeta: f64,
    sag: f64,
    noise: f64,
    hetero: bool,                // shoulders slow, wrists fast
    slide: f64,                  // y slide after 2 s, metres over 2 s
    block: Option<(usize, f64)>, // (left arm joint ordinal, sign): real joint cannot move past rest in sign direction
    seconds: f64,
    bias_noise: f64, // constant per-joint reading bias (calibration offset)
}

struct Out {
    min_all: f64,
    min_late: f64,
    end: f64,
    err_mm: f64,
}

fn run(c: Case) -> Out {
    let obstacles = counter(0.03);
    let (mut teleop, rest) = setup(obstacles);
    let names: Vec<String> = teleop
        .model()
        .axes()
        .iter()
        .map(|a| a.name.clone())
        .collect();
    let bias: Vec<f64> = names.iter().map(|n| c.sag * sag(n)).collect();
    let omega: Vec<f64> = names
        .iter()
        .map(|n| {
            if !c.hetero {
                c.omega
            } else if n.contains("shoulder") {
                c.omega * 0.3
            } else if n.contains("elbow") {
                c.omega
            } else {
                c.omega * 2.0
            }
        })
        .collect();
    let left_arm: Vec<usize> = names
        .iter()
        .enumerate()
        .filter(|(_, n)| n.starts_with("left_") && !n.contains("finger"))
        .map(|(i, _)| i)
        .collect();
    let blocked = c.block.map(|(k, s)| (left_arm[k % left_arm.len()], s));
    let (start, rotation) = left_tcp(&teleop, &rest);
    let width = rest.len();
    let (mut reference, mut real, mut v) = (rest.clone(), rest.clone(), vec![0.0; width]);
    let available = vec![true; width];
    let ticks = (c.seconds / c.dt) as usize;
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut rnd = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        2.0 * ((state >> 11) as f64 / (1_u64 << 53) as f64) - 1.0
    };
    let cal: Vec<f64> = (0..width).map(|_| c.bias_noise * rnd()).collect();
    let (mut min_all, mut min_late) = (f64::INFINITY, f64::INFINITY);
    let mut frames = teleop.model().poses(&real);
    teleop.reset();
    let mut target_at = start;
    for tick in 0..ticks {
        let t = tick as f64 * c.dt;
        let y = if t > 2.0 {
            c.slide * ((t - 2.0) / 2.0).min(1.0)
        } else {
            0.0
        };
        let off = [0.0, y, -0.06];
        target_at = [0, 1, 2].map(|k| start[k] + off[k]);
        let targets = Targets {
            tcp: [
                Some(Pose {
                    translation_m: target_at,
                    rotation_xyzw: rotation,
                }),
                None,
            ],
            ..Targets::default()
        };
        let measured: Vec<Option<f64>> = real
            .iter()
            .enumerate()
            .map(|(j, x)| Some(x + c.noise * rnd() + cal[j]))
            .collect();
        let step = teleop
            .step(&reference, &measured, &available, &targets, c.dt)
            .expect("step");
        reference.copy_from_slice(&step.q);
        let h = c.dt / SUBSTEPS as f64;
        for _ in 0..SUBSTEPS {
            for j in 0..width {
                let a = omega[j] * omega[j] * (reference[j] - bias[j] - real[j])
                    - 2.0 * c.zeta * omega[j] * v[j];
                v[j] += a * h;
                real[j] += v[j] * h;
                if let Some((b, s)) = blocked {
                    if j == b && (real[j] - rest[j]) * s > 0.0 {
                        real[j] = rest[j];
                        v[j] = 0.0;
                    }
                }
            }
        }
        let cl = teleop.ik().clearance_into(&real, &mut frames).scene_m;
        min_all = min_all.min(cl);
        if t > 1.0 {
            min_late = min_late.min(cl);
        }
    }
    let (at, _) = left_tcp(&teleop, &real);
    let err = (0..3)
        .map(|k| (target_at[k] - at[k]).powi(2))
        .sum::<f64>()
        .sqrt();
    Out {
        min_all,
        min_late,
        end: teleop.ik().clearance_into(&real, &mut frames).scene_m,
        err_mm: err * 1e3,
    }
}

fn show(tag: &str, c: Case) -> f64 {
    let o = run(c);
    println!(
        "{tag} dt {} w {} z {} sag {} noise {} cal {} hetero {} slide {} block {:?}: min {:.2} mm, min>1s {:.2} mm, end {:.2} mm, err {:.1} mm",
        c.dt,
        c.omega,
        c.zeta,
        c.sag,
        c.noise,
        c.bias_noise,
        c.hetero,
        c.slide,
        c.block,
        o.min_all * 1e3,
        o.min_late * 1e3,
        o.end * 1e3,
        o.err_mm
    );
    o.min_all
}

const BASE: Case = Case {
    dt: 0.01,
    omega: 30.0,
    zeta: 0.7,
    sag: 0.0,
    noise: 0.0,
    hetero: false,
    slide: 0.0,
    block: None,
    seconds: 6.0,
    bias_noise: 0.0,
};

#[test]
#[ignore = "refute attack cases, about 2 min: --ignored --nocapture"]
fn jitter_with_sag() {
    let mut worst = f64::INFINITY;
    for dt in [0.02, 0.01, 0.004, 0.002] {
        for sag in [1.0, -1.0] {
            for noise in [0.0, 0.0005, 0.001] {
                worst = worst.min(show(
                    "jitter",
                    Case {
                        dt,
                        sag,
                        noise,
                        ..BASE
                    },
                ));
            }
        }
    }
    println!("WORST jitter {:.2} mm", worst * 1e3);
}

#[test]
#[ignore = "refute attack cases, about 2 min: --ignored --nocapture"]
fn bigger_sag() {
    let mut worst = f64::INFINITY;
    for dt in [0.02, 0.01, 0.004] {
        for sag in [3.0, -3.0, 6.0, -6.0] {
            for noise in [0.0, 0.001] {
                worst = worst.min(show(
                    "bigsag",
                    Case {
                        dt,
                        sag,
                        noise,
                        ..BASE
                    },
                ));
            }
        }
    }
    println!("WORST bigsag {:.2} mm", worst * 1e3);
}

#[test]
#[ignore = "refute attack cases, about 2 min: --ignored --nocapture"]
fn calibration_bias() {
    let mut worst = f64::INFINITY;
    for dt in [0.02, 0.01, 0.004] {
        for cal in [0.003, 0.01] {
            for noise in [0.0, 0.001] {
                worst = worst.min(show(
                    "cal",
                    Case {
                        dt,
                        bias_noise: cal,
                        noise,
                        ..BASE
                    },
                ));
            }
        }
    }
    println!("WORST cal {:.2} mm", worst * 1e3);
}

#[test]
#[ignore = "refute attack cases, about 2 min: --ignored --nocapture"]
fn hetero_slide() {
    let mut worst = f64::INFINITY;
    for dt in [0.02, 0.01, 0.004] {
        for (omega, zeta) in [(30.0, 0.7), (9.5, 0.2), (15.0, 1.0)] {
            for slide in [0.15, -0.15, 0.08, -0.08] {
                for noise in [0.0, 0.001] {
                    worst = worst.min(show(
                        "slide",
                        Case {
                            dt,
                            omega,
                            zeta,
                            hetero: true,
                            slide,
                            noise,
                            ..BASE
                        },
                    ));
                }
            }
        }
    }
    println!("WORST slide {:.2} mm", worst * 1e3);
}

#[test]
#[ignore = "refute attack cases, about 2 min: --ignored --nocapture"]
fn blocked_joint() {
    let mut worst = f64::INFINITY;
    for dt in [0.02, 0.01, 0.004] {
        for k in 0..8 {
            for s in [1.0, -1.0] {
                for slide in [0.0, 0.1, -0.1] {
                    worst = worst.min(show(
                        "block",
                        Case {
                            dt,
                            block: Some((k, s)),
                            slide,
                            ..BASE
                        },
                    ));
                }
            }
        }
    }
    println!("WORST block {:.2} mm", worst * 1e3);
}
#[test]
#[ignore = "refute attack cases, about 2 min: --ignored --nocapture"]
fn names() {
    let (t, _) = setup(Vec::new());
    let n: Vec<_> = t
        .model()
        .axes()
        .iter()
        .map(|a| a.name.clone())
        .filter(|n| n.starts_with("left_") && !n.contains("finger"))
        .collect();
    println!("NAMES {n:?}");
}
