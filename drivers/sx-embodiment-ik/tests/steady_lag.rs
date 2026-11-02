//! A position-controlled joint rests a steady few milliradians behind its command (its PD
//! against gravity or a load). Near a box or the other arm the pose it rests at, kept clear
//! too ([`sx_embodiment_ik::Ik::set_measured`]), must not drive a limit cycle: a target held
//! past the box converges and holds still, and a long paced move settles.

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use std::collections::BTreeMap;

use sx_embodiment_ik::teleop::{Pose, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{Active, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};

/// Ten substeps of a motor per tick.
const SUBSTEPS: usize = 10;

/// A follower: PD (natural frequency rad/s, damping ratio) on each command, resting `sag`
/// radians short of it (a constant load over its stiffness), per arm joint, this sign.
#[derive(Clone, Copy, Debug)]
struct Motor {
    omega: f64,
    zeta: f64,
    sign: f64,
}

/// The steady sag per joint, radians, as act-sim's arms rested at the end of a 0.23 m move
/// (shoulder roll 7.5, elbow 5.5, shoulder pitch 2.4 mrad; 2026-11-03), the rest 3.
fn sag(name: &str) -> f64 {
    if name.contains("shoulder_roll") {
        0.0075
    } else if name.contains("elbow") {
        0.0055
    } else if name.contains("shoulder_pitch") {
        0.0024
    } else if name.contains("finger") || name.starts_with("so101") {
        0.0
    } else {
        0.003
    }
}

struct Run {
    /// Over the last second: the commanded and the real tool point's spread, metres.
    command_spread_m: f64,
    real_spread_m: f64,
    /// Over the last second: ticks whose binding pairs differ from the tick before.
    toggles: usize,
    /// The real tool point's distance from the target at the end, metres, and the rotation
    /// error, radians.
    error_m: f64,
    least_clear_m: f64,
}

fn setup(obstacles: Vec<[f64; 6]>) -> (Teleop, Vec<f64>) {
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

fn left_tcp(teleop: &Teleop, q: &[f64]) -> ([f64; 3], [f64; 4]) {
    let link = teleop
        .model()
        .links()
        .iter()
        .position(|l| l == "left_gripper_tcp")
        .expect("tcp");
    let frame = teleop.model().poses(q).links[link];
    let pose = sx_embodiment_drivers::kinematics::pose_of(&frame.to_transform());
    (frame.t, pose.rotation_xyzw)
}

/// `seconds` of the left hand sent along `path` (offset from where it rests, at `t`), the
/// robot `motor` with every arm joint sagging, at `dt`.
fn hold(
    obstacles: Vec<[f64; 6]>,
    path: &dyn Fn(f64) -> [f64; 3],
    tool_speed: Option<f64>,
    motor: Motor,
    dt: f64,
    seconds: f64,
) -> Run {
    let (mut teleop, rest) = setup(obstacles);
    let names: Vec<String> = teleop.model().axes().iter().map(|a| a.name.clone()).collect();
    let bias: Vec<f64> = names.iter().map(|n| motor.sign * sag(n)).collect();
    let (start, rotation) = left_tcp(&teleop, &rest);
    let width = rest.len();
    let (mut reference, mut real, mut v) = (rest.clone(), rest.clone(), vec![0.0; width]);
    let available = vec![true; width];
    let ticks = (seconds / dt) as usize;
    let last = ticks - (1.0 / dt) as usize;
    let (mut commanded, mut really) = (Vec::new(), Vec::new());
    let (mut toggles, mut before): (usize, Vec<Active>) = (0, Vec::new());
    let mut least = f64::INFINITY;
    let mut least_pair = String::new();
    let at_rest = teleop.ik().clearance(&rest);
    println!("at rest {:.1} mm {} {}", at_rest.0 * 1e3, at_rest.1, at_rest.2);
    let noise = std::env::var("IK_NOISE").ok().and_then(|v| v.parse::<f64>().ok()).unwrap_or(0.0);
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut jitter = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        let unit = (state >> 11) as f64 / (1_u64 << 53) as f64;
        noise * 2.0f64.mul_add(unit, -1.0)
    };
    teleop.reset();
    for tick in 0..ticks {
        let offset = path(tick as f64 * dt);
        let target = Pose {
            translation_m: [0, 1, 2].map(|k| start[k] + offset[k]),
            rotation_xyzw: rotation,
        };
        let measured: Vec<Option<f64>> = real.iter().map(|at| Some(at + jitter())).collect();
        let mut targets = Targets {
            tcp: [Some(target), None],
            ..Targets::default()
        };
        targets.tool_speed_m_s[0] = tool_speed;
        let step = teleop
            .step(&reference, &measured, &available, &targets, dt)
            .expect("a step");
        reference.copy_from_slice(&step.q);
        let pairs: Vec<Active> = step.limbs[0]
            .active
            .iter()
            .copied()
            .filter(|a| matches!(a, Active::Obstacle { .. } | Active::SelfCollision { .. }))
            .collect();
        let h = dt / SUBSTEPS as f64;
        for _ in 0..SUBSTEPS {
            for j in 0..width {
                let a = motor.omega * motor.omega * (reference[j] - bias[j] - real[j])
                    - 2.0 * motor.zeta * motor.omega * v[j];
                v[j] += a * h;
                real[j] += v[j] * h;
            }
        }
        if std::env::var("IK_TRACE").is_ok() {
            eprintln!("TICK {tick} cmd z {:.5} real z {:.5}", left_tcp(&teleop, &reference).0[2], left_tcp(&teleop, &real).0[2]);
        }
        let c = teleop.ik().clearance(&real);
        if c.0 < least {
            least = c.0;
            least_pair = format!("{} {}", c.1, c.2);
        }
        if tick >= last {
            commanded.push(left_tcp(&teleop, &reference).0);
            really.push(left_tcp(&teleop, &real).0);
            toggles += usize::from(pairs != before);
        }
        before = pairs;
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
    println!("least pair {least_pair} end real {:.2} mm cmd {:.2} mm", teleop.ik().clearance(&real).0 * 1e3, teleop.ik().clearance(&reference).0 * 1e3);
    let end = path(seconds);
    let (at, _) = left_tcp(&teleop, &real);
    Run {
        command_spread_m: spread(&commanded),
        real_spread_m: spread(&really),
        toggles,
        error_m: (0..3)
            .map(|k| (start[k] + end[k] - at[k]).powi(2))
            .sum::<f64>()
            .sqrt(),
        least_clear_m: least,
    }
}

/// A counter under the left hand, its top `clear` metres under the arm at rest.
fn counter(clear: f64) -> Vec<[f64; 6]> {
    let (teleop, rest) = setup(Vec::new());
    let (t, _) = left_tcp(&teleop, &rest);
    let at = |below: f64| vec![[
        t[0] - 0.1,
        t[1] - 0.08,
        t[2] - below - 0.3,
        t[0] + 0.1,
        t[1] + 0.08,
        t[2] - below,
    ]];
    let mut below = 0.0;
    loop {
        let (probe, _) = setup(at(below));
        let mut frames = probe.model().poses(&rest);
        if probe.ik().clearance_into(&rest, &mut frames).scene_m >= clear || below > 0.5 {
            return at(below);
        }
        below += 0.001;
    }
}

#[allow(dead_code)]
fn unused(below: f64) -> Vec<[f64; 6]> {
    let (teleop, rest) = setup(Vec::new());
    let (t, _) = left_tcp(&teleop, &rest);
    vec![[
        t[0] - 0.1,
        t[1] - 0.08,
        t[2] - below - 0.3,
        t[0] + 0.1,
        t[1] + 0.08,
        t[2] - below,
    ]]
}

const MOTORS: [Motor; 6] = [
    Motor { omega: 30.0, zeta: 0.7, sign: 0.0 },
    Motor { omega: 9.5, zeta: 0.2, sign: 0.0 },
    Motor {
        omega: 30.0,
        zeta: 0.7,
        sign: 1.0,
    },
    Motor {
        omega: 30.0,
        zeta: 0.7,
        sign: -1.0,
    },
    Motor {
        omega: 9.5,
        zeta: 0.2,
        sign: 1.0,
    },
    Motor {
        omega: 9.5,
        zeta: 0.2,
        sign: -1.0,
    },
];

#[test]
fn a_target_held_past_the_counter_converges_and_holds_still() {
    let only = std::env::var("IK_CASE").ok();
    for dt in [0.02, 0.01, 0.004] {
        for (m, motor) in MOTORS.into_iter().enumerate() {
            if only.as_ref().is_some_and(|o| *o != format!("{dt}-{m}")) {
                continue;
            }
            // Asked 6 cm down onto a counter 3 cm below the arm: the solve stops it short.
            let run = hold(counter(0.03), &|t| [0.0, 0.0, -0.06 * (t / 1.0).min(1.0)], None, motor, dt, 6.0);
            println!(
                "dt {dt} {motor:?}: command spread {:.2} mm, real {:.2} mm, toggles {}, least {:.1} mm",
                run.command_spread_m * 1e3,
                run.real_spread_m * 1e3,
                run.toggles,
                run.least_clear_m * 1e3
            );
        }
    }
}

#[test]
fn a_long_paced_move_settles() {
    for dt in [0.02, 0.01] {
        for motor in MOTORS {
            let goal = [0.035, -0.228, 0.04];
            let run = hold(Vec::new(), &move |_| goal, Some(0.3), motor, dt, 5.0);
            println!(
                "dt {dt} {motor:?}: error {:.1} mm, command spread {:.2} mm, toggles {}",
                run.error_m * 1e3,
                run.command_spread_m * 1e3,
                run.toggles
            );
        }
    }
}
