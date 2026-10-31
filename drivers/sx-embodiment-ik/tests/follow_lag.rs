//! How far behind an operator's hand the teleoperation solve keeps the static upper body's tool
//! point, held to today's numbers so a change to the solve cannot make teleop worse unseen.
//!
//! A desk-speed and a VR-speed hand path are followed through [`Teleop`] at the station's own
//! composed solve config (`fixtures/static-teleop-config.json`: rated limits at share 1, the
//! commissioned Ready posture of `fixtures/static-ready.json`), the joints second-order motors
//! (PD with the command's velocity fed forward) that lag and ring as act-sim's do, or stiffer.
//! Measured per path and motor: the delay that best lines the measured tool point up with the
//! hand, the RMS and peak error, and stalls (none: nothing stops these arms). An operator's
//! hands carry no tool speed, so this is what an operator feels.
//!
//! Before and after the reference gap and stall rule (2026-10-31, embodiments 258f355 against
//! #194): desk unchanged at 40 ms / 6.5 mm RMS (act-sim-like) and 30 ms / 4.4 mm (stiff); VR
//! stiff unchanged at 140 ms / 66.1 mm; VR act-sim-like 390 -> 310 ms, 147.6 -> 141.4 mm.
//! The fixtures are the station's: regenerate them from `sx_station::intent::compose` when the
//! rated limits or the Ready pose change, and re-measure. `cargo test --release --test
//! follow_lag -- --nocapture` prints the table.

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::many_single_char_names
)]

use std::collections::BTreeMap;
use std::f64::consts::TAU;

use sx_embodiment_ik::teleop::{Pose, Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{Body, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};

const DT: f64 = 0.01;
const SECONDS: f64 = 8.0;
const SETTLE_S: f64 = 1.0;
const SUBSTEPS: usize = 10;

/// A hand path: the offset from the start and the wrist's roll about x, at `t` seconds.
type Path = fn(f64) -> ([f64; 3], f64);

/// A desk operator: 5 cm side to side at 0.5 Hz, 3 cm up and down at 0.25 Hz.
fn desk(t: f64) -> ([f64; 3], f64) {
    (
        [
            0.0,
            0.05 * (TAU * 0.5 * t).sin(),
            0.03 * (TAU * 0.25 * t).sin(),
        ],
        0.0,
    )
}

/// A VR operator: an 8 cm circle at 1 Hz (0.5 m/s), the wrist rolling 15 degrees at 0.5 Hz.
fn vr(t: f64) -> ([f64; 3], f64) {
    (
        [0.0, 0.08 * (TAU * t).sin(), 0.08 * ((TAU * t).cos() - 1.0)],
        15_f64.to_radians() * (TAU * 0.5 * t).sin(),
    )
}

/// `angle` about x applied after `q`.
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

/// What one path on one motor (natural frequency rad/s, damping ratio) measured.
struct Follow {
    delay_ms: f64,
    rms_mm: f64,
    peak_mm: f64,
    stalled: usize,
}

fn follow(path: Path, omega: f64, zeta: f64) -> Follow {
    let config = TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json"))
        .expect("the station's solve config");
    let ready: BTreeMap<String, f64> =
        serde_json::from_str(include_str!("fixtures/static-ready.json")).expect("ready");
    let body = Body::static_upper_body();
    let names: Vec<String> = body.model().axes().iter().map(|a| a.name.clone()).collect();
    let rest: Vec<f64> = names
        .iter()
        .map(|n| ready.get(n).copied().unwrap_or(0.0))
        .collect();
    let left = body
        .limbs()
        .iter()
        .position(|l| *l == "left")
        .expect("left");
    let tool = |q: &[f64]| body.frame(&body.model().poses(q), left);
    let start = tool(&rest);
    let start_q = sx_embodiment_drivers::kinematics::pose_of(&start.to_transform()).rotation_xyzw;
    let mut teleop =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).expect("solve");
    // Answers, not the clock: a debug build or a loaded host past the station's 4 ms budget
    // would hold steps and measure the host.
    teleop.set_deadline(std::time::Duration::from_secs(1));
    let width = names.len();
    let mut reference = rest.clone();
    let (mut q, mut v) = (rest.clone(), vec![0.0; width]);
    let available = vec![true; width];
    teleop.reset();
    let ticks = (SECONDS / DT) as usize;
    let (mut wanted, mut got) = (Vec::with_capacity(ticks), Vec::with_capacity(ticks));
    let mut stalled = 0;
    for tick in 0..ticks {
        let (offset, roll) = path(tick as f64 * DT);
        let target = Pose {
            translation_m: [0, 1, 2].map(|k| start.t[k] + offset[k]),
            rotation_xyzw: turned_about_x(roll, start_q),
        };
        let measured: Vec<Option<f64>> = q.iter().copied().map(Some).collect();
        let targets = Targets {
            tcp: [Some(target), None],
            ..Targets::default()
        };
        let step = teleop
            .step(&reference, &measured, &available, &targets, DT)
            .expect("a step");
        reference.copy_from_slice(&step.q);
        let qdot = step.qdot.clone();
        stalled += usize::from(step.stalled.iter().any(|s| *s));
        // The motors over the tick: PD on the command, its velocity fed forward.
        let h = DT / SUBSTEPS as f64;
        for _ in 0..SUBSTEPS {
            for j in 0..width {
                let a =
                    omega * omega * (reference[j] - q[j]) + 2.0 * zeta * omega * (qdot[j] - v[j]);
                v[j] += a * h;
                q[j] += v[j] * h;
            }
        }
        wanted.push(target.translation_m);
        got.push(tool(&q).t);
    }
    let first = (SETTLE_S / DT) as usize;
    let apart =
        |a: [f64; 3], b: [f64; 3]| (0..3).map(|k| (a[k] - b[k]).powi(2)).sum::<f64>().sqrt();
    let mean_at = |delay: usize| {
        (first..ticks)
            .map(|i| apart(wanted[i - delay.min(i)], got[i]))
            .sum::<f64>()
            / (ticks - first) as f64
    };
    let best = (0..=60)
        .min_by(|a, b| mean_at(*a).total_cmp(&mean_at(*b)))
        .unwrap_or(0);
    let errors: Vec<f64> = (first..ticks).map(|i| apart(wanted[i], got[i])).collect();
    Follow {
        delay_ms: best as f64 * DT * 1e3,
        rms_mm: (errors.iter().map(|e| e * e).sum::<f64>() / errors.len() as f64).sqrt() * 1e3,
        peak_mm: errors.iter().copied().fold(0.0, f64::max) * 1e3,
        stalled,
    }
}

/// Today's numbers are the bounds, a tenth of margin on each (a delay one tick more): the
/// next change to the solve meets them or says why teleop got worse.
#[test]
fn an_operators_hand_is_followed_no_worse_than_today() {
    // (path, name, motor, omega, zeta, delay ms, rms mm, peak mm) as measured 2026-10-31.
    type Case = (Path, &'static str, &'static str, f64, f64, f64, f64, f64);
    let cases: [Case; 4] = [
        (desk, "desk", "act-sim-like", 9.5, 0.2, 40.0, 6.5, 11.5),
        (desk, "desk", "stiff", 30.0, 0.7, 30.0, 4.4, 9.5),
        (vr, "vr", "act-sim-like", 9.5, 0.2, 310.0, 141.4, 210.3),
        (vr, "vr", "stiff", 30.0, 0.7, 140.0, 66.1, 86.5),
    ];
    for (path, name, motor, omega, zeta, delay, rms, peak) in cases {
        let got = follow(path, omega, zeta);
        println!(
            "{name:5} {motor:13} delay {:3.0} ms, rms {:5.1} mm, peak {:5.1} mm, stalled {}",
            got.delay_ms, got.rms_mm, got.peak_mm, got.stalled
        );
        assert!(
            got.delay_ms <= delay * 1.1 + DT * 1e3,
            "{name} {motor}: delay {} ms",
            got.delay_ms
        );
        assert!(
            got.rms_mm <= rms * 1.1,
            "{name} {motor}: rms {} mm",
            got.rms_mm
        );
        assert!(
            got.peak_mm <= peak * 1.1,
            "{name} {motor}: peak {} mm",
            got.peak_mm
        );
        assert_eq!(got.stalled, 0, "{name} {motor}: a free arm stalled");
    }
}
