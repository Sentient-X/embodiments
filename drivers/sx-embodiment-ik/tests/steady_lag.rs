//! A position-controlled joint rests a steady few milliradians short of its command (its PD
//! against gravity or a load: act-sim's arms 2.4-7.5 mrad at the end of a 0.23 m move). Pairs
//! are kept clear where the arm is measured too ([`sx_embodiment_ik::Ik::set_measured`]), and
//! that must not drive a limit cycle: a target held past a counter converges and holds still,
//! and a long paced move settles.
//!
//! Before (embodiments 42567f8, the measured row counting each joint's lead only where it
//! closed the pair): the held command swung 23-77 mm at 1-2 Hz with the stiff motor, its pairs
//! binding and letting go 9-45 times a second, and 33-82 mm with the underdamped one. After:
//! 0.01-0.16 mm and none; the underdamped one rings on by 1-2 mm. `cargo test --release --test
//! steady_lag -- --nocapture` prints every case.

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]

use std::collections::BTreeMap;

use sx_embodiment_ik::teleop::{Pose, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{Active, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};

/// Substeps of a motor per tick.
const SUBSTEPS: usize = 10;

/// A follower: PD (natural frequency rad/s, damping ratio) on each command, every arm joint
/// resting [`sag`] short of it, this way (+1, -1) or not at all (0).
#[derive(Clone, Copy, Debug)]
struct Motor {
    omega: f64,
    zeta: f64,
    sag: f64,
}

/// Stiff, as act-sim's arms (they stay within 4.5 mrad of their commands).
const STIFF: (f64, f64) = (30.0, 0.7);
/// Underdamped: rings past its commands, as `teleop.rs`'s act-sim-like follower.
const RINGING: (f64, f64) = (9.5, 0.2);

/// The steady sag per joint, radians, as act-sim's left arm rested at the end of the 0.23 m
/// move (shoulder roll 7.5, elbow 5.5, shoulder pitch 2.4 mrad; 2026-11-03), the rest 3.
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

/// What one run measured.
struct Run {
    /// Over the last second: the commanded and the real tool point's spread, metres.
    command_spread_m: f64,
    real_spread_m: f64,
    /// Over the last second: ticks whose binding pairs differ from the tick before.
    toggles: usize,
    /// At the end: the real tool point's distance from the target, metres, and its turn from
    /// the target's orientation, radians.
    error_m: f64,
    error_rad: f64,
    /// At the end: the real arm's clearance from the scene, metres.
    clear_m: f64,
}

/// The station's solve config, `obstacles` its scene, and the commissioned Ready (each
/// shoulder pitched 0.15 rad since embodiments#213; the fixture's is 0.1).
fn setup(obstacles: Vec<[f64; 6]>) -> (Teleop, Vec<f64>) {
    let mut config = TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json"))
        .expect("the station's solve config");
    config.obstacles = obstacles;
    let ready: BTreeMap<String, f64> =
        serde_json::from_str(include_str!("fixtures/static-ready.json")).expect("ready");
    let mut teleop =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).expect("solve");
    // Answers, not the clock.
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
        .expect("tcp");
    let frame = teleop.model().poses(q).links[link];
    let pose = sx_embodiment_drivers::kinematics::pose_of(&frame.to_transform());
    (frame.t, pose.rotation_xyzw)
}

/// `seconds` of the left hand sent to `offset` from where it rests (at `tool_speed`, m/s), the
/// robot `motor`, at `dt`.
fn run(
    obstacles: Vec<[f64; 6]>,
    offset: [f64; 3],
    tool_speed: Option<f64>,
    motor: Motor,
    dt: f64,
    seconds: f64,
) -> Run {
    let (mut teleop, rest) = setup(obstacles);
    let bias: Vec<f64> = teleop
        .model()
        .axes()
        .iter()
        .map(|a| motor.sag * sag(&a.name))
        .collect();
    let (start, rotation) = left_tcp(&teleop, &rest);
    let target = Pose {
        translation_m: [0, 1, 2].map(|k| start[k] + offset[k]),
        rotation_xyzw: rotation,
    };
    let mut targets = Targets {
        tcp: [Some(target), None],
        ..Targets::default()
    };
    targets.tool_speed_m_s[0] = tool_speed;
    let width = rest.len();
    let (mut reference, mut real, mut v) = (rest.clone(), rest.clone(), vec![0.0; width]);
    let available = vec![true; width];
    let ticks = (seconds / dt) as usize;
    let last = ticks - (1.0 / dt) as usize;
    let (mut commanded, mut really) = (Vec::new(), Vec::new());
    let (mut toggles, mut before) = (0, Vec::new());
    teleop.reset();
    for tick in 0..ticks {
        let measured: Vec<Option<f64>> = real.iter().copied().map(Some).collect();
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
    let (at, turned) = left_tcp(&teleop, &real);
    let dot: f64 = (0..4).map(|k| turned[k] * rotation[k]).sum();
    let mut frames = teleop.model().poses(&real);
    Run {
        command_spread_m: spread(&commanded),
        real_spread_m: spread(&really),
        toggles,
        error_m: (0..3)
            .map(|k| (target.translation_m[k] - at[k]).powi(2))
            .sum::<f64>()
            .sqrt(),
        error_rad: 2.0 * dot.abs().min(1.0).acos(),
        clear_m: teleop.ik().clearance_into(&real, &mut frames).scene_m,
    }
}

/// A counter under the left hand, its top `clear` metres under the arm at Ready.
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
        assert!(below < 0.5, "no counter clear of the arm");
    }
}

/// Asked 6 cm down onto a counter 3 cm under the arm, held: the solve stops it at the margin.
fn held(motor: Motor, dt: f64) -> Run {
    run(counter(0.03), [0.0, 0.0, -0.06], None, motor, dt, 6.0)
}

fn report(dt: f64, motor: Motor, got: &Run) {
    println!(
        "dt {dt} {motor:?}: command spread {:.2} mm, real {:.2} mm, toggles {}, clear {:.1} mm, \
         error {:.1} mm {:.3} rad",
        got.command_spread_m * 1e3,
        got.real_spread_m * 1e3,
        got.toggles,
        got.clear_m * 1e3,
        got.error_m * 1e3,
        got.error_rad
    );
}

/// A target held past the counter, the arm sagging under its load either way, at the static
/// body's 50 Hz, 100 Hz and 250 Hz: the command converges and holds still (its tool point
/// within a millimetre over the last second, no pair binding and letting go), and the arm
/// rests outside the 10 mm margin where it really is, its sag counted.
#[test]
fn a_target_held_past_the_counter_converges_and_holds_still() {
    for dt in [0.02, 0.01, 0.004] {
        for sag in [1.0, -1.0] {
            let motor = Motor {
                omega: STIFF.0,
                zeta: STIFF.1,
                sag,
            };
            let got = held(motor, dt);
            report(dt, motor, &got);
            assert!(
                got.command_spread_m < 0.001 && got.real_spread_m < 0.001,
                "dt {dt} {motor:?}: spread {} / {} m",
                got.command_spread_m,
                got.real_spread_m
            );
            assert_eq!(got.toggles, 0, "dt {dt} {motor:?}: pairs toggled");
            assert!(
                got.clear_m >= 0.0099,
                "dt {dt} {motor:?}: {} m clear",
                got.clear_m
            );
        }
    }
}

/// The same with a follower that rings past its commands: it settles to a ring of a couple of
/// millimetres (main: 33-82 mm), never touching.
#[test]
fn a_ringing_follower_held_past_the_counter_settles() {
    for dt in [0.02, 0.01, 0.004] {
        for sag in [1.0, 0.0, -1.0] {
            let motor = Motor {
                omega: RINGING.0,
                zeta: RINGING.1,
                sag,
            };
            let got = held(motor, dt);
            report(dt, motor, &got);
            assert!(
                got.command_spread_m < 0.003 && got.real_spread_m < 0.003,
                "dt {dt} {motor:?}: spread {} / {} m",
                got.command_spread_m,
                got.real_spread_m
            );
            assert!(
                got.clear_m > 0.005,
                "dt {dt} {motor:?}: {} m clear",
                got.clear_m
            );
        }
    }
}

/// act-sim's 0.23 m move, from the commissioned Ready to (0.50, 0.01, 0.25) at its 0.283 m/s
/// speed cap, settles within 5 mm and 3 degrees, the command still. With each joint's lead
/// counted only where it closed a pair, act-sim stalled 0.051-0.059 rad short, held by the left
/// upper-arm yaw limit and the body pair (2, 5) (2026-11-03).
#[test]
fn the_long_paced_move_settles() {
    let (teleop, rest) = setup(Vec::new());
    let (start, _) = left_tcp(&teleop, &rest);
    let offset = [0.50 - start[0], 0.01 - start[1], 0.25 - start[2]];
    for dt in [0.02, 0.01] {
        for (omega, zeta) in [STIFF, RINGING] {
            let motor = Motor {
                omega,
                zeta,
                sag: 0.0,
            };
            let got = run(Vec::new(), offset, Some(0.283), motor, dt, 5.0);
            report(dt, motor, &got);
            assert!(
                got.error_m <= 0.005 && got.error_rad <= 3_f64.to_radians(),
                "dt {dt} {motor:?}: {} m, {} rad",
                got.error_m,
                got.error_rad
            );
            assert!(
                got.command_spread_m < 0.001,
                "dt {dt} {motor:?}: still moving"
            );
        }
    }
}
