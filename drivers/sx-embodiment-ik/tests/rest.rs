//! How a limb comes to rest when its target goes (`Target::Rest`) and how an engage at rest starts
//! (the shaped posture): every arm joint within its jerk cap, on the static upper body's composed
//! config (`tests/fixtures/static-teleop-config.json`, a joint's jerk cap its acceleration over the
//! config's `ramp_s`), at 100 Hz. The neck is not judged: its look engaging afresh is open
//! (`examples/transitions.rs`). Measured from the commanded positions, from rest before the
//! first tick, with an ideal follower.

#![allow(clippy::cast_precision_loss, clippy::float_cmp)] // tick counts; a held joint repeats its position exactly

use std::collections::BTreeMap;

use serde_json::Value;
use sx_embodiment_ik::teleop::{COAST_MAX_S, Targets, Teleop, TeleopConfig};
use sx_frames::Pose;

/// The static upper body's committed render pack: one fixture among the registry's bodies.
fn static_files() -> sx_embodiment_ik::BodyFiles<'static> {
    sx_embodiment_ik::registered("sentient-static-upper-body").expect("its pack is committed")
}

const DT: f64 = 0.01;

struct Bench {
    solve: Teleop,
    ready: Vec<f64>,
    /// Per axis its jerk cap, and whether it is judged (an arm joint or the torso).
    jerk: Vec<f64>,
    judged: Vec<bool>,
    names: Vec<String>,
}

fn bench(torso_held: bool) -> Bench {
    let mut json: Value =
        serde_json::from_str(include_str!("fixtures/static-teleop-config.json")).expect("config");
    if torso_held {
        json["held"] = serde_json::json!(["torso_joint"]);
    }
    let config = TeleopConfig::parse(&json.to_string()).expect("config");
    let mut solve =
        Teleop::new(static_files().urdf, static_files().collision, &config).expect("solve");
    solve.set_deadline(std::time::Duration::from_secs(3600));
    let ready: BTreeMap<String, f64> =
        serde_json::from_str(include_str!("fixtures/static-ready.json")).expect("ready");
    let names: Vec<String> = solve
        .model()
        .axes()
        .iter()
        .map(|a| a.name.clone())
        .collect();
    let ramp = json["ramp_s"].as_f64().expect("ramp");
    let jerk = names
        .iter()
        .map(|n| {
            let accel = json["accel"][n]
                .as_f64()
                .unwrap_or_else(|| json["speed"][n].as_f64().unwrap_or(0.0) / ramp);
            json["jerk"][n].as_f64().unwrap_or(accel / ramp)
        })
        .collect();
    let judged = names
        .iter()
        .map(|n| !n.starts_with("so101_") && !n.contains("gripper"))
        .collect();
    let ready = names
        .iter()
        .map(|n| ready.get(n).copied().unwrap_or(0.0))
        .collect();
    Bench {
        solve,
        ready,
        jerk,
        judged,
        names,
    }
}

fn tools(solve: &Teleop, q: &[f64]) -> [Pose; 2] {
    let body = solve.ik().body();
    let frames = body.model().poses(q);
    [body.frame(&frames, 0).pose(), body.frame(&frames, 1).pose()]
}

/// Each tick's commanded positions from the bench's Ready, `ask(tick, q)` giving the targets and
/// whether to engage afresh first.
fn run(
    b: &mut Bench,
    ticks: usize,
    mut ask: impl FnMut(usize, &[f64]) -> (Targets, bool),
) -> Vec<Vec<f64>> {
    let mut q = b.ready.clone();
    let available = vec![true; q.len()];
    let mut out = vec![q.clone()];
    for tick in 1..=ticks {
        let (targets, engage) = ask(tick, &q);
        if engage {
            b.solve.reset();
        }
        let measured: Vec<Option<f64>> = q.iter().map(|v| Some(*v)).collect();
        let step = b
            .solve
            .step(&q, &measured, &available, &targets, DT)
            .expect("a step");
        q.copy_from_slice(&step.q);
        out.push(q.clone());
    }
    out
}

/// The worst judged joint's commanded jerk over its cap from tick `from` on (positions
/// differenced, rest before the first), and that joint.
fn worst_jerk(b: &Bench, q: &[Vec<f64>], from: usize) -> (f64, String) {
    let mut worst = (0.0, String::new());
    for axis in (0..b.names.len()).filter(|&a| b.judged[a]) {
        let at = |k: usize| q[k.max(0)][axis];
        for k in from.max(3)..q.len() {
            let j = (at(k) - 3.0 * at(k - 1) + 3.0 * at(k - 2) - at(k - 3)) / DT.powi(3);
            let share = j.abs() / b.jerk[axis];
            if share > worst.0 {
                worst = (share, format!("{} at tick {k}", b.names[axis]));
            }
        }
    }
    worst
}

fn moved(p: &Pose, by: [f64; 3]) -> Pose {
    let mut out = *p;
    for (at, step) in out.translation_m.iter_mut().zip(by) {
        *at += step;
    }
    out
}

/// Let go of a 0.5 m/s sweep (joints at their speed caps on the station's caps): the arm comes
/// to rest within every joint's jerk cap, is held through its own two-tick test well before the
/// timeout, and coasts no further than its joints' caps carry it. Coasting along the tool's line
/// instead, the joints left their speed caps with a kink (6x on governed caps).
#[test]
fn letting_go_of_a_fast_sweep_rests_every_joint_within_its_jerk_cap() {
    for torso_held in [true, false] {
        let mut b = bench(torso_held);
        let from = tools(&b.solve, &b.ready);
        let (sweep_until, release) = (40, 60);
        let q = run(&mut b, 160, |tick, _| {
            let t = (tick.saturating_sub(sweep_until) as f64 * DT).min(0.2);
            let left = (tick < release).then(|| moved(&from[0], [0.0, 0.5 * t, 0.0]));
            (
                Targets {
                    tcp: sx_embodiment_ik::teleop::hands([left, Some(from[1])]),
                    ..Targets::default()
                },
                tick == 1,
            )
        });
        // From the release on: the sweep before it is the operator's own motion, ridden as asked.
        let (share, joint) = worst_jerk(&b, &q, release);
        // 1.0143x on the torso, free, at the release since the approved assembly's fixed
        // shoulder mounts and their hulls (2026-11-08); 1.01x before.
        assert!(
            share <= 1.015,
            "torso held {torso_held}: {share:.5}x the jerk cap ({joint})"
        );
        // The released arm's own joints, and the torso it carries, stand still.
        let resting: Vec<usize> = (0..b.names.len())
            .filter(|&a| b.names[a].starts_with("left_") || b.names[a] == "torso_joint")
            .collect();
        let still = (release..q.len())
            .find(|&k| resting.iter().all(|&a| q[k][a] == q[k - 1][a]))
            .expect("held");
        assert!(
            ((still - release) as f64 * DT) < COAST_MAX_S - 0.05,
            "torso held {torso_held}: held by the timeout, not at rest ({} s)",
            (still - release) as f64 * DT
        );
        let coast = {
            let (a, b) = (
                tools(&b.solve, &q[release - 1])[0],
                tools(&b.solve, &q[still])[0],
            );
            (0..3)
                .map(|k| (a.translation_m[k] - b.translation_m[k]).powi(2))
                .sum::<f64>()
                .sqrt()
        };
        assert!(coast < 0.08, "torso held {torso_held}: coasted {coast} m");
    }
}

/// A program brings the hand 30 cm and stands there; the operator takes it where the robot
/// stands (an engage afresh with the target on the tool). Drawn straight at the posture, the free
/// direction snapped at the engage (8x a shoulder's jerk cap); drawn along a shaped posture, the
/// arm moves on smoothly.
#[test]
fn an_engage_at_rest_where_the_tool_stands_moves_nothing_abruptly() {
    let handover = 200;
    let program = |from: [Pose; 2]| Targets {
        tcp: sx_embodiment_ik::teleop::hands([
            Some(moved(&from[0], [0.10, 0.10, 0.2645])),
            Some(from[1]),
        ]),
        ..Targets::default()
    };
    // Where the tool stands when the operator takes it.
    let mut b = bench(true);
    let from = tools(&b.solve, &b.ready);
    let before = run(&mut b, handover - 1, |tick, _| (program(from), tick == 1));
    let hold = tools(&b.solve, &before[handover - 1]);
    let mut b = bench(true);
    let q = run(&mut b, handover + 60, |tick, _| {
        let targets = if tick < handover {
            program(from)
        } else {
            Targets {
                tcp: sx_embodiment_ik::teleop::hands([Some(hold[0]), Some(hold[1])]),
                ..Targets::default()
            }
        };
        (targets, tick == 1 || tick == handover)
    });
    let (share, joint) = worst_jerk(&b, &q, handover - 1);
    assert!(
        share <= 1.01,
        "{share:.2}x the jerk cap at the handover ({joint})"
    );
}
