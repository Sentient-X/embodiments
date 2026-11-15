//! A deadman press with an arm already inside its collision margin: the robot was held there
//! (its capsules 23 mm into each other at session b968c187's press), and the hands engage where
//! they are. The pair is not closing, so nothing pushes it apart: each tool stays on its held
//! target, and the pair comes no closer.

use std::collections::BTreeMap;

use sx_embodiment_ik::READING_JITTER;
use sx_embodiment_ik::teleop::{Targets, Teleop, TeleopConfig, hands};

const DT: f64 = 0.01;

fn static_files() -> sx_embodiment_ik::BodyFiles<'static> {
    sx_embodiment_ik::registered("sentient-static-upper-body").expect("its pack is committed")
}

/// A press held still for a second with the left arm 5 mm inside a box below it, the body at
/// rest and its left shoulder sagged `sag` rad deeper than its command: the commanded tools'
/// travel from where they engaged at worst, the reading's own (where the body sits), and the
/// least clearance of the reading and of the command over the run, metres.
fn press(sag: f64, jitter: f64, seed: u64) -> (f64, f64, f64, f64) {
    let mut config = TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json"))
        .expect("the station's solve config");
    let ready: BTreeMap<String, f64> =
        serde_json::from_str(include_str!("fixtures/static-ready.json")).expect("ready");
    let mut probe =
        Teleop::new(static_files().urdf, static_files().collision, &config).expect("solve");
    let names: Vec<String> = probe
        .model()
        .axes()
        .iter()
        .map(|a| a.name.clone())
        .collect();
    let mut q: Vec<f64> = names
        .iter()
        .zip(probe.ik().rest())
        .map(|(name, rest)| ready.get(name).copied().unwrap_or(*rest))
        .collect();
    let tool = probe.tool_poses(&q)[0].expect("a left tool").translation_m;
    // Raise a box's top under the left tool until the arm is 5 mm inside it.
    let mut top = tool[2] - 0.3;
    let solve = loop {
        config.obstacles = vec![[
            tool[0] - 0.1,
            tool[1] - 0.1,
            tool[2] - 0.5,
            tool[0] + 0.1,
            tool[1] + 0.1,
            top,
        ]];
        let solve =
            Teleop::new(static_files().urdf, static_files().collision, &config).expect("solve");
        let clearance = solve.ik().clearance(&q).0;
        if clearance < -0.005 {
            break solve;
        }
        assert!(top < tool[2] + 0.1, "the box never reached the arm");
        top += 0.001;
    };
    let mut solve = solve;
    // Judged on the solve alone, never on this host's load.
    solve.set_deadline(std::time::Duration::from_secs(3600));
    // The left shoulder's deeper direction: whichever way takes the arm further in.
    let joint = names
        .iter()
        .position(|n| n == "left_shoulder_pitch_joint")
        .expect("a shoulder pitch");
    let at = |offset: f64| {
        let mut moved = q.clone();
        moved[joint] += offset;
        solve.ik().clearance(&moved).0
    };
    let deeper = if at(0.01) < at(-0.01) { 1.0 } else { -1.0 };
    let before = solve.ik().clearance(&q).0;
    let tools = solve.tool_poses(&q);
    let targets = Targets {
        tcp: hands([tools[0], tools[1]]),
        ..Targets::default()
    };
    let available = vec![true; q.len()];
    let (mut travel, mut least) = (0.0_f64, before);
    // The body at rest where it was held, its shoulder sagged `sag` deeper than its command.
    let mut measured: Vec<Option<f64>> = q.iter().copied().map(Some).collect();
    measured[joint] = Some(q[joint] + deeper * sag);
    let read: Vec<f64> = measured.iter().map(|m| m.expect("read")).collect();
    let sits = solve.tool_poses(&read)[0]
        .expect("a left tool")
        .translation_m;
    let engaged = tools[0].expect("a left tool").translation_m;
    let sagged = (0..3)
        .map(|k| (sits[k] - engaged[k]).powi(2))
        .sum::<f64>()
        .sqrt();
    let floor = solve.ik().clearance(&read).0.min(before);
    solve.reset();
    let mut state = seed;
    for _ in 0..100 {
        // Every reading off by up to `jitter` each tick, as an encoder's last count flickers.
        let read: Vec<Option<f64>> = measured
            .iter()
            .map(|m| {
                state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                #[allow(clippy::cast_precision_loss)]
                let unit = (state >> 11) as f64 / (1_u64 << 53) as f64;
                m.map(|v| v + jitter * (2.0 * unit - 1.0))
            })
            .collect();
        let answer = solve
            .step(&q, &read, &available, &targets, DT)
            .expect("a step");
        q.copy_from_slice(&answer.q);
        for (now, then) in solve.tool_poses(&q).iter().zip(&tools) {
            if let (Some(now), Some(then)) = (now, then) {
                let apart = (0..3)
                    .map(|k| (now.translation_m[k] - then.translation_m[k]).powi(2))
                    .sum::<f64>()
                    .sqrt();
                travel = travel.max(apart);
            }
        }
        least = least.min(solve.ik().clearance(&q).0);
    }
    (travel, sagged, floor, least)
}

#[test]
fn a_press_inside_the_margin_holds_its_tools_and_closes_nothing() {
    for (sag, jitter, seed) in [
        (0.0, 0.0, 0),
        (0.01, 0.0, 0),
        (0.03, 0.0, 0),
        (0.01, 2e-5, 1),
        (0.01, 1e-4, 2),
        (0.01, READING_JITTER, 3),
        (0.01, READING_JITTER, 4),
    ] {
        let (travel, sagged, floor, least) = press(sag, jitter, seed);
        eprintln!(
            "sag {sag} jitter {jitter}: tools travelled {:.1} mm (the reading sits {:.1} mm \
             off), clearance {:.1} at the reading -> least {:.1} mm",
            travel * 1e3,
            sagged * 1e3,
            floor * 1e3,
            least * 1e3
        );
        // Before the fix a 10 mrad press shoved the tools 100-125 mm, and gated at exactly zero
        // speed one 20 urad flicker brought it back. Now the command at most settles onto where
        // the body reads, and comes no closer than the body already is.
        assert!(
            travel < sagged + 0.002,
            "sag {sag} jitter {jitter}: the tools travelled {travel} m, the reading {sagged} m"
        );
        assert!(
            least > floor - 0.0015,
            "sag {sag} jitter {jitter}: the pair closed to {least} m from {floor} m"
        );
    }
}
