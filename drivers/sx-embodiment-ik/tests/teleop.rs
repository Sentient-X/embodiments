//! The shared teleoperation solve on the static upper body: where the camera looks, the holds,
//! the grippers, the reference rule, and one config both sides build from.

#![allow(clippy::float_cmp)]

use std::collections::BTreeMap;

use sx_embodiment_ik::teleop::{
    Clutch, GLITCH_HOLD, HANDS, REFERENCE_GAP, STALL, STEADY_S, TELEOP_SCHEMA, Targets, Teleop,
    TeleopConfig, TeleopStep,
};
use sx_embodiment_ik::{
    Body, READING_JITTER, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF, Solve,
};
use sx_frames::{Pose, QuaternionLayout};

const DT: f64 = 0.01;

/// Every joint at 1 rad/s, reached in a quarter second.
fn config() -> TeleopConfig {
    let body = Body::static_upper_body();
    let speed: BTreeMap<String, f64> = body
        .model()
        .axes()
        .iter()
        .map(|axis| (axis.name.clone(), 1.0))
        .collect();
    TeleopConfig {
        schema: TELEOP_SCHEMA.into(),
        limbs: TeleopConfig::static_upper_body_limbs(),
        speed,
        accel: BTreeMap::new(),
        ramp_s: 0.25,
        max_step_rad: 0.03,
        held: Vec::new(),
        obstacles: Vec::new(),
        posture: std::collections::BTreeMap::new(),
    }
}

fn teleop() -> Teleop {
    let mut solve = Teleop::new(
        STATIC_UPPER_BODY_URDF,
        STATIC_UPPER_BODY_COLLISION,
        &config(),
    )
    .expect("solve");
    // Answers, not the clock: a loaded host (the clearance cases on every core) past the
    // station's budget would hold steps.
    solve.set_deadline(std::time::Duration::from_secs(3600));
    solve
}

/// Every axis inside its range, the mid of zero and its range's middle.
fn start(teleop: &Teleop) -> Vec<f64> {
    teleop
        .model()
        .axes()
        .iter()
        .map(|a| {
            a.limits.map_or(0.0, |[l, u]| {
                0.0_f64.clamp(l, u).midpoint(f64::midpoint(l, u))
            })
        })
        .collect()
}

fn index(teleop: &Teleop, name: &str) -> usize {
    teleop
        .model()
        .axes()
        .iter()
        .position(|a| a.name == name)
        .expect(name)
}

/// Run `ticks` steps as a ghost would (its own reference, nothing measured).
fn run(teleop: &mut Teleop, q: &mut [f64], targets: &Targets, ticks: usize) {
    let width = q.len();
    let measured = vec![None; width];
    let available = vec![true; width];
    for _ in 0..ticks {
        let step = teleop
            .step(q, &measured, &available, targets, DT)
            .expect("a step");
        q.copy_from_slice(&step.q);
    }
}

fn camera_forward(teleop: &Teleop, q: &[f64]) -> ([f64; 3], [f64; 3]) {
    let body = teleop.ik().body();
    let head = body
        .limbs()
        .iter()
        .position(|l| *l == "head")
        .expect("a head");
    let frame = body.frame(&body.model().poses(q), head);
    ([frame.r[0][0], frame.r[1][0], frame.r[2][0]], frame.t)
}

fn tcp(teleop: &Teleop, q: &[f64], hand: usize) -> Pose {
    let body = teleop.ik().body();
    body.frame(&body.model().poses(q), hand).pose()
}

#[test]
fn the_config_round_trips_and_a_foreign_schema_is_refused() {
    let config = config();
    let json = config.to_json();
    assert_eq!(TeleopConfig::parse(&json).expect("parsed"), config);
    assert!(
        !json.contains("\"head\":"),
        "no head choice in the config: who looks decides"
    );
    let foreign = json.replace(TELEOP_SCHEMA, "sx.teleop-solve/v0");
    assert!(TeleopConfig::parse(&foreign).is_err());
}

#[test]
fn gaze_turns_the_camera_to_the_head_and_holds_the_rest() {
    let mut solve = teleop();
    let mut q = start(&solve);
    let begun = q.clone();
    let (f0, _) = camera_forward(&solve, &q);
    let yaw = f0[1].atan2(f0[0]) + 0.5;
    let gaze = Pose {
        translation_m: [0.0, 0.0, 1.0],
        rotation: QuaternionLayout::Xyzw.read([0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()]),
    };
    let targets = Targets {
        head: Some(gaze),
        ..Targets::default()
    };
    solve.reset();
    run(&mut solve, &mut q, &targets, 400);
    let (f, _) = camera_forward(&solve, &q);
    let dot = f[0] * yaw.cos() + f[1] * yaw.sin();
    assert!(dot > 0.99, "camera forward {f:?}");
    // The arms and the torso carry undriven limbs: held.
    for (axis, (now, was)) in solve.model().axes().iter().zip(q.iter().zip(&begun)) {
        if !axis.name.starts_with("so101_") {
            assert_eq!(now, was, "{} moved", axis.name);
        }
    }
    // No head pose: the neck holds too.
    let turned = q.clone();
    run(&mut solve, &mut q, &Targets::default(), 50);
    assert_eq!(q, turned);
}

#[test]
fn hands_looks_at_the_commanded_hands_even_from_facing_away() {
    let mut solve = teleop();
    let mut q: Vec<f64> = solve
        .model()
        .axes()
        .iter()
        .map(|a| a.limits.map_or(0.0, |[l, u]| 0.0_f64.clamp(l, u)))
        .collect();
    let (left, right) = (tcp(&solve, &q, 0), tcp(&solve, &q, 1));
    let middle: Vec<f64> = (0..3)
        .map(|k| f64::midpoint(left.translation_m[k], right.translation_m[k]))
        .collect();
    let targets = Targets {
        tcp: [Some(left), Some(right)],
        ..Targets::default()
    };
    solve.reset();
    run(&mut solve, &mut q, &targets, 400);
    let (forward, at) = camera_forward(&solve, &q);
    let to: Vec<f64> = (0..3).map(|k| middle[k] - at[k]).collect();
    let distance = to.iter().map(|v| v * v).sum::<f64>().sqrt();
    let dot: f64 = (0..3).map(|k| forward[k] * to[k] / distance).sum();
    assert!(dot > 0.99, "the camera looks {dot} toward the hands");
}

#[test]
fn an_idle_hand_and_the_torso_hold_while_one_hand_drives() {
    let mut solve = teleop();
    let mut q = start(&solve);
    let begun = q.clone();
    let mut target = tcp(&solve, &q, 0);
    target.translation_m[2] += 0.05;
    let targets = Targets {
        tcp: [Some(target), None],
        ..Targets::default()
    };
    solve.reset();
    run(&mut solve, &mut q, &targets, 200);
    let torso = index(&solve, "torso_joint");
    assert_eq!(q[torso], begun[torso], "the torso holds with one hand idle");
    for (axis, (now, was)) in solve.model().axes().iter().zip(q.iter().zip(&begun)) {
        if axis.name.starts_with("right_") {
            assert_eq!(now, was, "{} moved", axis.name);
        }
    }
    assert!(
        q.iter().zip(&begun).any(|(a, b)| a != b),
        "the left arm moved"
    );
}

#[test]
fn each_gripper_maps_the_fingertip_distance_onto_its_fingers() {
    let mut solve = teleop();
    for (hand, gripper) in HANDS.iter().zip(solve.grippers()) {
        let gripper = gripper.as_ref().expect(hand);
        assert_eq!(gripper.fingers.len(), 2);
        assert!(
            gripper.open_width_m > gripper.closed_width_m + 0.01,
            "{gripper:?}"
        );
    }
    let open = solve.grippers()[0].clone().expect("left").open_width_m;
    let mut q = start(&solve);
    let targets = Targets {
        gripper_width_m: [Some(open), None],
        ..Targets::default()
    };
    run(&mut solve, &mut q, &targets, 1);
    for &(axis, _, far) in &solve.grippers()[0].clone().expect("left").fingers {
        assert_eq!(q[axis], far);
        assert!(far.abs() > 0.5);
    }
}

#[test]
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // a tick count
fn the_reference_seeds_from_the_measurement_at_engage_and_stays_within_the_gap() {
    let mut solve = teleop();
    let q = start(&solve);
    let width = q.len();
    let elbow = index(&solve, "left_elbow_pitch_joint");
    let available = vec![true; width];
    let mut measured: Vec<Option<f64>> = q.iter().map(|v| Some(*v)).collect();
    let reference = vec![0.0; width];
    // Engage: every answering axis starts where it is measured, whatever the reference.
    solve.reset();
    let step = solve
        .step(&reference, &measured, &available, &Targets::default(), DT)
        .expect("a step");
    assert_eq!(step.q[elbow], q[elbow]);
    // Within the gap the reference wins; past it, the reference is held at the gap. Each new
    // reading is fed until the solve has taken it and its smoothing (STEADY_S) has settled: a
    // jump no joint makes in a tick is set aside for GLITCH_HOLD first.
    let mut reference = step.q.clone();
    let held = ((GLITCH_HOLD + STEADY_S * 15).as_secs_f64() / DT).ceil() as usize;
    let mut settle = |measured: &[Option<f64>], reference: &mut Vec<f64>| {
        for _ in 0..held {
            let step = solve
                .step(reference, measured, &available, &Targets::default(), DT)
                .expect("a step");
            reference.clone_from(&step.q);
        }
    };
    measured[elbow] = Some(q[elbow] + REFERENCE_GAP / 2.0);
    settle(&measured, &mut reference);
    assert_eq!(reference[elbow], q[elbow]);
    measured[elbow] = Some(q[elbow] + REFERENCE_GAP * 2.0);
    settle(&measured, &mut reference);
    assert!((reference[elbow] - (q[elbow] + REFERENCE_GAP)).abs() < 1e-6);
}

#[test]
fn the_clutch_takes_hold_where_the_robot_stands_and_moves_the_gripper_as_the_hand_moves() {
    let mut solve = teleop();
    let q = start(&solve);
    let tools = solve.tool_poses(&q);
    let (left, right) = (tcp(&solve, &q, 0), tcp(&solve, &q, 1));
    for (pose, at) in [(left, tools[0]), (right, tools[1])] {
        let at = at.expect("a gripper per hand");
        assert!((0..3).all(|k| (pose.translation_m[k] - at.translation_m[k]).abs() < 1e-12));
    }
    // The operator's hand is 30 cm away and turned: holding on, the target is the gripper.
    let quarter = [
        0.0,
        0.0,
        std::f64::consts::FRAC_1_SQRT_2,
        std::f64::consts::FRAC_1_SQRT_2,
    ];
    let hand = Pose {
        translation_m: [0.3, -0.2, 0.9],
        rotation: QuaternionLayout::Xyzw.read(quarter),
    };
    let mut clutch = Clutch::default();
    let mut targets = Targets {
        tcp: [Some(hand), None],
        ..Targets::default()
    };
    clutch.apply(&tools, &mut targets);
    let held = targets.tcp[0].expect("left");
    assert!((0..3).all(|k| (held.translation_m[k] - left.translation_m[k]).abs() < 1e-12));
    assert!(
        angle(
            QuaternionLayout::Xyzw.write(held.rotation),
            QuaternionLayout::Xyzw.write(left.rotation)
        ) < 1e-9
    );
    assert!(clutch.holds(0) && !clutch.holds(1));
    // The hand moves 5 cm up and turns 0.2 rad about the robot's z: the gripper does the same.
    let turn = [0.0, 0.0, 0.1_f64.sin(), 0.1_f64.cos()];
    let moved = Pose {
        translation_m: [0.3, -0.2, 0.95],
        rotation: QuaternionLayout::Xyzw.read(turned(turn, quarter)),
    };
    let mut targets = Targets {
        tcp: [Some(moved), None],
        ..Targets::default()
    };
    clutch.apply(&tools, &mut targets);
    let followed = targets.tcp[0].expect("left");
    assert!((followed.translation_m[2] - (left.translation_m[2] + 0.05)).abs() < 1e-12);
    assert!(
        (angle(
            QuaternionLayout::Xyzw.write(followed.rotation),
            turned(turn, QuaternionLayout::Xyzw.write(left.rotation))
        )) < 1e-9
    );
    // The hand drops out of tracking: it lets go, and takes hold afresh where it is seen next.
    let mut gone = Targets::default();
    clutch.apply(&tools, &mut gone);
    assert!(!clutch.holds(0));
    let mut back = Targets {
        tcp: [Some(moved), None],
        ..Targets::default()
    };
    clutch.apply(&tools, &mut back);
    let again = back.tcp[0].expect("left");
    assert!((0..3).all(|k| (again.translation_m[k] - left.translation_m[k]).abs() < 1e-12));
    // Released (a regrip): the next target takes hold afresh too.
    clutch.release();
    assert!(!clutch.holds(0));
}

/// The angle between two unit quaternions' rotations, radians.
fn angle(a: [f64; 4], b: [f64; 4]) -> f64 {
    let dot: f64 = a.iter().zip(b).map(|(x, y)| x * y).sum();
    2.0 * dot.abs().min(1.0).acos()
}

/// `turn` applied after `rotation`, both unit quaternions (x, y, z, w).
fn turned(turn: [f64; 4], rotation: [f64; 4]) -> [f64; 4] {
    let ([tx, ty, tz, tw], [rx, ry, rz, rw]) = (turn, rotation);
    [
        tw * rx + tx * rw + ty * rz - tz * ry,
        tw * ry - tx * rz + ty * rw + tz * rx,
        tw * rz + tx * ry - ty * rx + tz * rw,
        tw * rw - tx * rx - ty * ry - tz * rz,
    ]
}

/// Hands given a tool speed move their tool points no faster than that, every tick, however far
/// their targets and however much they must turn on the way, and end as near them as without
/// one; without one they go as fast as the joints allow, which let act-sim's tipped grasp reach
/// 0.55 m/s against a 0.288 m/s goal (2026-10-31). Both hands and the head are driven, so the
/// torso is free and carries both arms: the whole step is shortened, never one limb's columns
/// (which rescaled the shared torso under the other hand, and broke the rows it held). The
/// joints accelerate as act-sim's do (a twentieth of a second to full speed), where holding the
/// tool never needs a joint to slow past its cap; past that the cap gives way to the joints'.
#[test]
fn a_tool_speed_holds_every_ticks_tool_step_and_ends_as_near() {
    let peak = |speed: Option<f64>| {
        let mut solve = Teleop::new(
            STATIC_UPPER_BODY_URDF,
            STATIC_UPPER_BODY_COLLISION,
            &TeleopConfig {
                ramp_s: 0.05,
                ..config()
            },
        )
        .expect("solve");
        solve.set_deadline(std::time::Duration::from_secs(3600));
        let mut q = start(&solve);
        // The torso upright: from the suite's start it rests on the table, and the solve holds.
        q[index(&solve, "torso_joint")] = 0.0;
        let from = [tcp(&solve, &q, 0), tcp(&solve, &q, 1)];
        // 7.5 cm away and turned 20 degrees about y: a reach the joints could close fast.
        let half = 10_f64.to_radians();
        let target = |from: Pose, dy: f64| Pose {
            translation_m: [
                from.translation_m[0] + 0.05,
                from.translation_m[1] + dy,
                from.translation_m[2] - 0.04,
            ],
            rotation: QuaternionLayout::Xyzw.read(turned(
                [0.0, half.sin(), 0.0, half.cos()],
                QuaternionLayout::Xyzw.write(from.rotation),
            )),
        };
        let targets = Targets {
            tcp: [Some(target(from[0], -0.04)), Some(target(from[1], 0.04))],
            tool_speed_m_s: [speed, speed],
            head: Some(from[0]),
            ..Targets::default()
        };
        let width = q.len();
        let (measured, available) = (vec![None; width], vec![true; width]);
        solve.reset();
        let mut fastest = 0.0_f64;
        let mut last = [from[0].translation_m, from[1].translation_m];
        for _ in 0..800 {
            let step = solve
                .step(&q, &measured, &available, &targets, DT)
                .expect("a step");
            q.copy_from_slice(&step.q);
            for (hand, last) in last.iter_mut().enumerate() {
                let now = tcp(&solve, &q, hand).translation_m;
                let moved = (0..3)
                    .map(|k| (now[k] - last[k]).powi(2))
                    .sum::<f64>()
                    .sqrt();
                fastest = fastest.max(moved / DT);
                *last = now;
            }
        }
        let goal = targets.tcp[0].expect("left").translation_m;
        let off = (0..3)
            .map(|k| (last[0][k] - goal[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        (fastest, off)
    };
    let (free, free_off) = peak(None);
    let (capped, off) = peak(Some(0.05));
    assert!(free > 0.1, "unbounded, the joints go faster: {free} m/s");
    assert!(capped <= 0.05 * (1.0 + 1e-5), "{capped} m/s against 0.05");
    // It ends as near its target as the unbounded hand does, only later.
    assert!(off <= free_off + 0.001, "{off} m off, unbounded {free_off}");
}

/// A joint that something stops pushes at most the gripper's stall time: its reference held at
/// the gap, its measurement coming no nearer, it stalls within [`STALL`] and a tick, its
/// reference goes to where it stands (no push left) and stays there while the hand still asks
/// that way; when the hand asks the other way, the joint is free and moves back.
#[test]
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)] // tick counts and seconds
fn a_blocked_joint_stops_pushing_within_the_stall_time_and_backs_off_when_asked() {
    let mut solve = teleop();
    let mut q = start(&solve);
    q[index(&solve, "torso_joint")] = 0.0;
    let elbow = index(&solve, "left_elbow_pitch_joint");
    let from = tcp(&solve, &q, 0);
    let width = q.len();
    let available = vec![true; width];
    let blocked_at = q[elbow];
    // Every joint lands where it is sent but the elbow, which something holds where it is.
    let measure = |sent: &[f64]| -> Vec<Option<f64>> {
        sent.iter()
            .enumerate()
            .map(|(axis, at)| Some(if axis == elbow { blocked_at } else { *at }))
            .collect()
    };
    let reach = |dz: f64| Targets {
        tcp: [
            Some(Pose {
                translation_m: [
                    from.translation_m[0] + 0.06,
                    from.translation_m[1],
                    from.translation_m[2] + dz,
                ],
                rotation: from.rotation,
            }),
            None,
        ],
        ..Targets::default()
    };
    solve.reset();
    let mut reference = q.clone();
    let mut stalled_at = None;
    let ticks = ((STALL.as_secs_f64() + 1.0) / DT) as usize;
    for tick in 0..ticks {
        let step = solve
            .step(
                &reference,
                &measure(&reference),
                &available,
                &reach(-0.08),
                DT,
            )
            .expect("a step");
        reference.copy_from_slice(&step.q);
        if step.stalled[elbow] {
            stalled_at.get_or_insert(tick);
            assert!(
                (reference[elbow] - blocked_at).abs() < 1e-9,
                "stalled, it pushes no more: {} against {blocked_at}",
                reference[elbow]
            );
        } else {
            assert!(
                stalled_at.is_none(),
                "it stays stalled while the hand asks that way"
            );
            // At most the gap and one tick's step (max_step_rad) ahead of where it stands.
            assert!((reference[elbow] - blocked_at).abs() <= REFERENCE_GAP + 0.03 + 1e-9);
        }
    }
    let stalled_at = stalled_at.expect("the blocked elbow stalled");
    assert!(
        // The gap is reached in a few tenths, then the stall time runs.
        (stalled_at as f64) * DT <= STALL.as_secs_f64() + 0.5,
        "stalled at {:.2} s",
        stalled_at as f64 * DT
    );
    // The hand asks back up: the elbow is free and moves away from what stopped it.
    let mut freed = false;
    for _ in 0..200 {
        let step = solve
            .step(
                &reference,
                &measure(&reference),
                &available,
                &reach(0.08),
                DT,
            )
            .expect("a step");
        reference.copy_from_slice(&step.q);
        if !step.stalled[elbow] {
            freed = true;
            break;
        }
    }
    assert!(freed, "asked the other way, the stall lets go");
}

/// A blocked joint stalls though every reading jitters by [`READING_JITTER`] either way, at the
/// Spark's 90 Hz, 50 Hz and 100 Hz, and stays stalled while the hand pushes on. Its step ends
/// on the window's edge whatever the reading, so the jitter moves the gap, not the push (at
/// 8d151e9, ±0.4 mrad of noise kept it from ever stalling: embodiments#206's refute).
#[test]
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)] // tick counts and seconds
fn a_blocked_joint_stalls_through_a_readings_jitter() {
    // The Spark's rates with the jitter tests assume, and 250 and 500 Hz with twice it (there
    // 1.5 mrad kept a pushing elbow from ever stalling, embodiments#210).
    for (dt, noise) in [
        (0.011, READING_JITTER),
        (0.02, READING_JITTER),
        (0.01, READING_JITTER),
        (0.004, 2.0 * READING_JITTER),
        (0.002, 2.0 * READING_JITTER),
    ] {
        let mut solve = teleop();
        let mut q = start(&solve);
        q[index(&solve, "torso_joint")] = 0.0;
        let elbow = index(&solve, "left_elbow_pitch_joint");
        let from = tcp(&solve, &q, 0);
        let available = vec![true; q.len()];
        let blocked_at = q[elbow];
        let mut state = 0x9e37_79b9_7f4a_7c15_u64;
        let mut jitter = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let unit = (state >> 11) as f64 / (1_u64 << 53) as f64;
            noise * 2.0f64.mul_add(unit, -1.0)
        };
        let targets = Targets {
            tcp: [
                Some(Pose {
                    translation_m: [
                        from.translation_m[0] + 0.06,
                        from.translation_m[1],
                        from.translation_m[2] - 0.08,
                    ],
                    rotation: from.rotation,
                }),
                None,
            ],
            ..Targets::default()
        };
        solve.reset();
        let (mut at_gap, mut stalled_at) = (None, None);
        for tick in 0..((STALL.as_secs_f64() + 1.5) / dt) as usize {
            let measured: Vec<Option<f64>> = q
                .iter()
                .enumerate()
                .map(|(axis, at)| Some(if axis == elbow { blocked_at } else { *at } + jitter()))
                .collect();
            let step = solve
                .step(&q, &measured, &available, &targets, dt)
                .expect("a step");
            q.copy_from_slice(&step.q);
            if (q[elbow] - blocked_at).abs() >= REFERENCE_GAP - 0.003 {
                at_gap.get_or_insert(tick);
            }
            if step.stalled[elbow] {
                stalled_at.get_or_insert(tick);
            } else {
                assert!(
                    stalled_at.is_none(),
                    "at {dt} s it stays stalled while pushed"
                );
            }
        }
        let at_gap = at_gap.unwrap_or_else(|| panic!("at {dt} s the elbow never reached the gap"));
        let stalled_at = stalled_at.unwrap_or_else(|| panic!("at {dt} s the elbow never stalled"));
        // The stall time from when it reached the gap; at 100 Hz the solve swings the elbow off
        // the gap once on the way, and the stall time starts again (1.20 s).
        let late = (stalled_at - at_gap) as f64 * dt;
        assert!(
            late <= 1.3,
            "at {dt} s it stalled {late:.2} s after reaching the gap"
        );
    }
}

/// A stalled joint is held stiff where it stopped, not where it is measured each tick: under a
/// load that pulls it away from what stopped it, it stays there rather than follow the load
/// down (held at its measurement, it went limp and fell 7.9 rad in 6 s; refute, 2026-10-31).
/// Only a driven arm's solved joints stall: a finger closed on an object, written from a
/// width, never does, nor does an idle arm someone pushes.
#[test]
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss
)] // tick counts and seconds
fn a_stalled_joint_holds_under_load_and_only_a_driven_arms_joints_stall() {
    let mut solve = teleop();
    let mut q = start(&solve);
    q[index(&solve, "torso_joint")] = 0.0;
    let elbow = index(&solve, "left_elbow_pitch_joint");
    let right_elbow = index(&solve, "right_elbow_pitch_joint");
    let finger = index(&solve, "left_gripper_left_finger_joint");
    let from = tcp(&solve, &q, 0);
    let width = q.len();
    let available = vec![true; width];
    // The elbow cannot pass where it starts (the way the hand asks it), and a load sags it
    // 0.05 rad the other way from its command; the right elbow is pushed 0.15 rad by someone;
    // the left finger is held open by an object; the rest land where they are sent.
    let targets = Targets {
        tcp: [
            Some(Pose {
                translation_m: [
                    from.translation_m[0] + 0.06,
                    from.translation_m[1],
                    from.translation_m[2] - 0.08,
                ],
                rotation: from.rotation,
            }),
            None,
        ],
        gripper_width_m: [Some(0.0), None],
        ..Targets::default()
    };
    let stop_at = q[elbow];
    let reach_dir = {
        let mut probe = teleop();
        probe.reset();
        let measured: Vec<Option<f64>> = q.iter().copied().map(Some).collect();
        let mut reference = q.clone();
        for _ in 0..10 {
            let step = probe
                .step(&reference, &measured, &available, &targets, DT)
                .expect("a step");
            reference.copy_from_slice(&step.q);
        }
        (reference[elbow] - stop_at).signum()
    };
    let finger_open = q[finger];
    let measure = |sent: &[f64]| -> Vec<Option<f64>> {
        sent.iter()
            .enumerate()
            .map(|(axis, at)| {
                Some(if axis == elbow {
                    let sagged = at - 0.05 * reach_dir;
                    if (sagged - stop_at) * reach_dir > 0.0 {
                        stop_at
                    } else {
                        sagged
                    }
                } else if axis == right_elbow {
                    at + 0.15
                } else if axis == finger {
                    finger_open
                } else {
                    *at
                })
            })
            .collect()
    };
    solve.reset();
    let mut reference = q.clone();
    let mut stalled = false;
    for _ in 0..((STALL.as_secs_f64() + 3.0) / DT) as usize {
        let measured = measure(&reference);
        let step = solve
            .step(&reference, &measured, &available, &targets, DT)
            .expect("a step");
        reference.copy_from_slice(&step.q);
        stalled |= step.stalled[elbow];
        assert!(
            !step.stalled[finger],
            "a finger on an object is not a stall"
        );
        assert!(
            !step.stalled[right_elbow],
            "an idle arm someone pushes does not stall"
        );
        let elbow_at = measured[elbow].expect("measured");
        assert!(
            (elbow_at - stop_at).abs() <= 0.05 + REFERENCE_GAP + 0.03 + 1e-9,
            "the elbow fell {} rad from where it stopped",
            (elbow_at - stop_at).abs()
        );
    }
    assert!(stalled, "the elbow stalled on its stop_at");
    let measured = measure(&reference);
    assert!(
        (measured[elbow].expect("measured") - stop_at).abs() <= 0.05 + 1e-9,
        "held where it stopped, it sags only by its load"
    );
}

/// The station's own solve (`fixtures/static-teleop-config.json`, the commissioned Ready
/// posture) from Ready at a tick of `dt`, its left elbow stopped where it stands by something
/// it cannot pass the way the hand pushes it (more negative); it is free the other way, and
/// every other joint lands where it is sent. `push(t)` is where the hand's target sits from
/// the tool at Ready at `t` seconds; `check` sees each tick's time and step (the elbow's index
/// given) and returns false to stop.
fn hold_the_elbow(
    dt: f64,
    push: impl Fn(f64) -> [f64; 3],
    mut check: impl FnMut(f64, usize, &TeleopStep) -> bool,
) -> f64 {
    let config = TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json"))
        .expect("the station's solve config");
    let ready: BTreeMap<String, f64> =
        serde_json::from_str(include_str!("fixtures/static-ready.json")).expect("ready");
    let mut solve =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).expect("solve");
    // Answers, not the clock: a debug build past the station's budget would hold steps.
    solve.set_deadline(std::time::Duration::from_secs(3600));
    let rest: Vec<f64> = solve
        .model()
        .axes()
        .iter()
        .map(|a| ready.get(&a.name).copied().unwrap_or(0.0))
        .collect();
    let elbow = index(&solve, "left_elbow_pitch_joint");
    let from = tcp(&solve, &rest, 0);
    let stopped_at = rest[elbow];
    let available = vec![true; rest.len()];
    let mut reference = rest.clone();
    solve.reset();
    for tick in 0_u32.. {
        let t = f64::from(tick) * dt;
        let measured: Vec<Option<f64>> = reference
            .iter()
            .enumerate()
            .map(|(axis, at)| {
                Some(if axis == elbow {
                    at.max(stopped_at)
                } else {
                    *at
                })
            })
            .collect();
        let offset = push(t);
        let targets = Targets {
            tcp: [
                Some(Pose {
                    translation_m: [0, 1, 2].map(|k| from.translation_m[k] + offset[k]),
                    rotation: from.rotation,
                }),
                None,
            ],
            ..Targets::default()
        };
        let step = solve
            .step(&reference, &measured, &available, &targets, dt)
            .expect("a step");
        reference.copy_from_slice(&step.q);
        if !check(t, elbow, step) {
            break;
        }
    }
    stopped_at
}

/// The loop rates a station may run the solve at.
const RATES: [f64; 3] = [0.01, 0.004, 0.002];

/// Where the hand first pushes the elbow into its stop.
const INTO: [f64; 3] = [0.03, 0.0, -0.04];

/// A hand that goes on asking into a stall keeps the joint stalled, commanded exactly where it
/// stopped, for as long as it asks, in every direction that drives the elbow into its stop and
/// at every rate. Released on the sliver a one-way blocked solve left the joint inside its block
/// (1e-9 rad), the elbow pushed into the obstacle again by the whole reference gap while the
/// hand still asked that way (embodiments#198, the station's held-elbow test at 2f357e7).
#[test]
fn a_stall_holds_for_as_long_as_the_hand_pushes_into_it() {
    let pushes = [
        INTO,
        [0.06, 0.0, -0.08],
        [0.03, 0.02, -0.05],
        [0.0, 0.0, -0.06],
        [0.05, -0.02, -0.03],
        [0.02, 0.01, -0.10],
    ];
    for dt in RATES {
        for push in pushes {
            let mut stalled_at = None;
            let mut commanded = Vec::new();
            let stopped_at = hold_the_elbow(
                dt,
                |_| push,
                |t, elbow, step| {
                    if step.stalled[elbow] {
                        stalled_at.get_or_insert(t);
                        commanded.push((t, step.q[elbow], step.qdot[elbow]));
                    } else {
                        assert!(
                            stalled_at.is_none(),
                            "{push:?} at {dt} s a tick: released at {t:.3} s while the hand \
                             still asks that way"
                        );
                    }
                    t < 10.0
                },
            );
            let stalled_at =
                stalled_at.unwrap_or_else(|| panic!("{push:?} at {dt}: the held elbow stalled"));
            assert!(
                stalled_at <= STALL.as_secs_f64() + 0.6,
                "{push:?} at {dt}: stalled at {stalled_at:.2} s"
            );
            // From the stall on, the command never leaves where the elbow stopped: not by the
            // gap, not by a sliver, and it stands.
            for (t, elbow, speed) in commanded {
                assert!(
                    elbow == stopped_at && speed == 0.0,
                    "{push:?} at {dt}: at {t:.3} s commanded {elbow} at {speed} rad/s against \
                     its stop {stopped_at}"
                );
            }
        }
    }
}

/// A hand target that jumps, as tracking regained or a network gap resumed makes it, to where
/// a free arm would still drive the elbow into its stop keeps the elbow stalled: the solve's
/// first ticks toward a jumped target ask the elbow back a little while the rest of the arm
/// catches up, which is not the hand turning back. (A jump whose free path itself first draws
/// the elbow back past `STALL_RELEASE`, [0.06, 0, 0.06] at 250 Hz: 11 mm, is a release, and
/// what follows pushes the stop as a fresh contact does, at most the reference gap.)
#[test]
fn a_jumped_target_that_still_drives_into_the_stop_keeps_the_stall() {
    // Each, from Ready, drives a free elbow 0.05 to 0.07 rad into the stop.
    let jumps = [[0.0, 0.0, -0.01], [0.02, 0.0, 0.01], [0.04, 0.0, 0.03]];
    let turn = STALL.as_secs_f64() + 1.5;
    for dt in RATES {
        for jump in jumps {
            let mut stalled = false;
            hold_the_elbow(
                dt,
                |t| if t < turn { INTO } else { jump },
                |t, elbow, step| {
                    if t >= turn {
                        assert!(
                            step.stalled[elbow],
                            "{jump:?} at {dt}: released at {:.3} s after the jump",
                            t - turn
                        );
                    }
                    stalled |= step.stalled[elbow];
                    t < turn + 3.0
                },
            );
            assert!(stalled, "{jump:?} at {dt}: stalled before the jump");
        }
    }
}

/// A stalled joint lets go within 10 ms of the hand turning back (one tick at 100 Hz), at every rate, starts
/// from rest within its acceleration cap, and moves away from what stopped it.
#[test]
fn a_stall_lets_go_as_soon_as_the_hand_turns_back() {
    let config = TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json"))
        .expect("the station's solve config");
    let accel = config.accel["left_elbow_pitch_joint"];
    let turn = STALL.as_secs_f64() + 1.5;
    for dt in RATES {
        let mut stalled_before = false;
        let mut released = None;
        let mut speed = 0.0;
        let mut last = f64::NAN;
        let stopped_at = hold_the_elbow(
            dt,
            |t| if t < turn { INTO } else { [0.03, 0.0, 0.04] },
            |t, elbow, step| {
                if t < turn {
                    stalled_before = step.stalled[elbow];
                } else if !step.stalled[elbow] {
                    released.get_or_insert(t - turn);
                }
                let v = step.qdot[elbow];
                if t >= turn {
                    assert!(
                        (v - speed).abs() <= accel * dt * (1.0 + 1e-6),
                        "at {dt}: the elbow's speed jumped {speed} to {v} rad/s at {t:.3} s"
                    );
                }
                speed = v;
                last = step.q[elbow];
                t < turn + 0.5
            },
        );
        assert!(
            stalled_before,
            "at {dt}: stalled before the hand turns back"
        );
        let released = released.unwrap_or_else(|| panic!("at {dt}: released"));
        assert!(
            released <= 0.01,
            "at {dt}: released {released:.3} s after the hand turned back"
        );
        assert!(
            last - stopped_at > 0.01,
            "at {dt}: moved back {} rad",
            last - stopped_at
        );
    }
}

/// How the robot follows its commands in [`least_clearance`].
#[derive(Clone, Copy, Debug)]
enum Follower {
    /// Reaches each command exactly, this many ticks late.
    Late(usize),
    /// A PD on each command this many ticks late (natural frequency rad/s, damping ratio),
    /// as the motors of `follow_lag.rs`: act-sim-like (9.5, 0.2) rings past its commands.
    Pd(usize, f64, f64),
}

/// The least clearance, metres, where the robot really is, from 1.5 s after the left elbow
/// first stalls on a stop (when `stopped`) to 2.5 s on, the pair, and the right arm's sharpest
/// turn as a multiple of its acceleration cap: `case` "reach" sends the right gripper to where
/// the left one really is, "box" sweeps the left hand inward past a keep-out box beside its
/// forearm. The robot is `follower`, its left elbow never past the stop; it is measured where
/// it is, each reading off by up to `noise` either way.
fn least_clearance(
    dt: f64,
    case: &str,
    stopped: bool,
    follower: Follower,
    noise: f64,
    glitch: Option<(&str, f64)>,
) -> (f64, String, String, f64, u32) {
    least_clearance_at(dt, case, stopped, follower, noise, glitch, None)
}

/// [`least_clearance`] with both hands' tool points held to `tool_speed` m/s.
#[allow(clippy::too_many_lines)] // one scene, one follower, one loop
fn least_clearance_at(
    dt: f64,
    case: &str,
    stopped: bool,
    follower: Follower,
    noise: f64,
    glitch: Option<(&str, f64)>,
    tool_speed: Option<f64>,
) -> (f64, String, String, f64, u32) {
    let mut config = TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json"))
        .expect("the station's solve config");
    let ready: BTreeMap<String, f64> =
        serde_json::from_str(include_str!("fixtures/static-ready.json")).expect("ready");
    let probe =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).expect("solve");
    let rest: Vec<f64> = probe
        .model()
        .axes()
        .iter()
        .map(|a| ready.get(&a.name).copied().unwrap_or(0.0))
        .collect();
    let link = |name: &str| {
        probe
            .model()
            .links()
            .iter()
            .position(|l| l == name)
            .expect(name)
    };
    if case == "box" {
        // Beside the forearm, where the left hand's sweep inward carries it.
        let f = probe.model().poses(&rest).links[link("left_forearm_yaw_link")].t;
        config.obstacles = vec![[
            f[0] - 0.06,
            f[1] - 0.16,
            f[2] - 0.06,
            f[0] + 0.06,
            f[1] - 0.09,
            f[2] + 0.06,
        ]];
    }
    let left_tcp = link("left_gripper_tcp");
    let mut solve =
        Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config).expect("solve");
    solve.set_deadline(std::time::Duration::from_secs(3600));
    let elbow = index(&solve, "left_elbow_pitch_joint");
    let (left0, right0) = (tcp(&solve, &rest, 0), tcp(&solve, &rest, 1));
    let stop_at = rest[elbow];
    let available = vec![true; rest.len()];
    let mut q = rest.clone();
    // The commands the follower has yet to reach, oldest first.
    let (Follower::Late(lag) | Follower::Pd(lag, ..)) = follower;
    let mut sent = std::collections::VecDeque::from(vec![rest.clone(); lag + 1]);
    let mut real = rest.clone();
    let mut speed = vec![0.0; rest.len()];
    // A reading's jitter, uniform within `noise`, the same every run.
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut jitter = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        #[allow(clippy::cast_precision_loss)] // 53 bits are plenty
        let unit = (state >> 11) as f64 / (1_u64 << 53) as f64;
        noise * 2.0f64.mul_add(unit, -1.0)
    };
    let turn = STALL.as_secs_f64() + 1.5;
    let mut least = (f64::INFINITY, String::new(), String::new());
    let (mut before, mut lurch, mut relaxed) = (rest.clone(), 0.0_f64, 0_u32);
    let caps = solve.ik().caps().accel.clone();
    let arms: Vec<usize> = solve
        .model()
        .axes()
        .iter()
        .enumerate()
        .filter(|(_, a)| {
            (a.name.starts_with("right_") || a.name.starts_with("left_") || a.name == "torso_joint")
                && !a.name.contains("finger")
        })
        .map(|(axis, _)| axis)
        .collect();
    let mut was_stalled = vec![false; rest.len()];
    solve.reset();
    for tick in 0_u32.. {
        let t = f64::from(tick) * dt;
        if t > turn + 2.5 {
            break;
        }
        // One reading off by the glitch, half a second after the turn.
        let glitched = glitch.and_then(|(joint, by)| {
            ((t - turn - 0.5).abs() < dt / 2.0).then(|| (index(&solve, joint), by))
        });
        let measured: Vec<Option<f64>> = real
            .iter()
            .enumerate()
            .map(|(axis, at)| {
                let off = glitched
                    .filter(|(glitched, _)| *glitched == axis)
                    .map_or(0.0, |g| g.1);
                Some(at + jitter() + off)
            })
            .collect();
        let inward = if case == "box" && t >= turn {
            -0.25 * ((t - turn) / 2.0).min(1.0)
        } else {
            0.0
        };
        let mut targets = Targets {
            tcp: [
                Some(Pose {
                    translation_m: [0, 1, 2]
                        .map(|k| left0.translation_m[k] + [INTO[0], inward, INTO[2]][k]),
                    rotation: left0.rotation,
                }),
                None,
            ],
            ..Targets::default()
        };
        targets.tool_speed_m_s = [tool_speed; 2];
        if case == "reach" && t >= turn {
            targets.tcp[1] = Some(Pose {
                translation_m: solve.model().poses(&real).links[left_tcp].t,
                rotation: right0.rotation,
            });
        }
        let step = solve
            .step(&q, &measured, &available, &targets, dt)
            .expect("a step");
        relaxed += u32::from(step.solve == Solve::Relaxed);
        // How far past its acceleration cap any arm or torso joint's command turned this tick
        // (a stalled joint's jump to where it stands, or its release, is not a turn).
        for axis in arms.iter().copied() {
            let turned = (step.q[axis] - q[axis]) - (q[axis] - before[axis]);
            let stall_change = step.stalled[axis] || was_stalled[axis];
            was_stalled[axis] = step.stalled[axis];
            if caps[axis].is_finite() && caps[axis] > 0.0 && t >= turn && !stall_change {
                lurch = lurch.max(turned.abs() / (caps[axis] * dt * dt));
            }
        }
        before.copy_from_slice(&q);
        q.copy_from_slice(&step.q);
        sent.push_back(q.clone());
        let command = sent.pop_front().expect("a command");
        match follower {
            Follower::Late(_) => real = command,
            Follower::Pd(_, omega, zeta) => {
                // Ten substeps of the motor over the tick.
                let h = dt / 10.0;
                for _ in 0..10 {
                    for ((at, v), goal) in real.iter_mut().zip(&mut speed).zip(&command) {
                        let accel = omega * omega * (goal - *at) - 2.0 * zeta * omega * *v;
                        *v += accel * h;
                        *at += *v * h;
                    }
                    if stopped && real[elbow] < stop_at {
                        real[elbow] = stop_at;
                        speed[elbow] = 0.0;
                    }
                }
            }
        }
        if stopped {
            real[elbow] = real[elbow].max(stop_at);
        }
        if t >= turn {
            let c = solve.ik().clearance(&real);
            if c.0 < least.0 {
                least = c;
            }
        }
    }
    (least.0, least.1, least.2, lurch, relaxed)
}

/// The collision margin less a millimetre: a step's linearisation.
const MARGIN_WITHIN_A_MILLIMETRE: f64 = 0.009;
/// A command turns no harder than twice its acceleration cap, a window's most (main d41fcf2: up
/// to 23x with these followers; #207: 94x the torso, 155x a shoulder, embodiments#208).
const WITHIN_THE_CAP: f64 = 2.05;

const CASES: [(&str, bool); 4] = [
    ("reach", true),
    ("box", true),
    ("reach", false),
    ("box", false),
];

/// Every case at `rates`, each of `followers`, its readings exact and jittering by
/// [`READING_JITTER`], keeps `clear_m` metres where the robot really is, and turns no arm or
/// torso joint's command harder than `turn` times its acceleration cap.
fn keeps(rates: &[f64], followers: &[Follower], clear_m: f64, turn: f64) {
    // Each case on its own thread: a debug build runs them all in the time of the longest.
    std::thread::scope(|scope| {
        let mut runs = Vec::new();
        for &dt in rates {
            for (case, stopped) in CASES {
                for &follower in followers {
                    for noise in [0.0, READING_JITTER] {
                        runs.push(scope.spawn(move || {
                            let got = least_clearance(dt, case, stopped, follower, noise, None);
                            (dt, case, stopped, follower, noise, got)
                        }));
                    }
                }
            }
        }
        for run in runs {
            let (dt, case, stopped, follower, noise, (clear, a, b, lurch, _)) =
                run.join().expect("a case");
            assert!(
                clear >= clear_m && lurch <= turn,
                "{case} at {dt} s, stop {stopped}, {follower:?}, noise {noise}: {a} and {b} {clear} m apart, turned {lurch}x a cap"
            );
        }
    });
}

/// A pair is kept where the robot really is, a joint pushing a stop or a follower behind its
/// command alike, at the static body's 50 Hz, the 90 Hz its sessions ran at, and 100 Hz: a
/// follower 0, 2 and 4 ticks late (the Spark's arm joints trail their commands 26-63 ms, its
/// neck 80-140 ms; its 2026-10-27..30 journals) and a stiff PD, on time and two ticks late.
/// Planned from the command with the gap clamped onto it, three ticks of lag at 50 Hz brought
/// the grippers to -41 mm with the left elbow on a stop (d41fcf2).
#[test]
fn a_pushing_arm_keeps_its_pairs_clear_where_it_really_is() {
    keeps(
        &[0.02, 0.011, 0.01],
        &[
            Follower::Late(0),
            Follower::Late(2),
            Follower::Late(4),
            Follower::Pd(0, 30.0, 0.7),
            Follower::Pd(2, 30.0, 0.7),
        ],
        MARGIN_WITHIN_A_MILLIMETRE,
        WITHIN_THE_CAP,
    );
}

/// The same at 250 and 500 Hz.
#[test]
fn a_pushing_arm_keeps_its_pairs_clear_at_faster_rates() {
    keeps(
        &[0.004, 0.002],
        &[
            Follower::Late(0),
            Follower::Late(2),
            Follower::Pd(0, 30.0, 0.7),
        ],
        MARGIN_WITHIN_A_MILLIMETRE,
        WITHIN_THE_CAP,
    );
}

/// An underdamped follower (act-sim-like, zeta 0.2, and 0.1), on time or a few ticks late,
/// rings past its commands and is kept off the other arm and the box: 7.7 mm at worst
/// (main d41fcf2: 62 mm into the other gripper; without its measured speed counted, 5.1 mm at
/// zeta 0.2 and 46 mm in at 0.1, embodiments#208). Its turns are not bounded here: a ringing
/// arm's command turns as main's does.
#[test]
fn an_underdamped_follower_never_touches() {
    keeps(
        &[0.02, 0.011, 0.01],
        &[
            Follower::Pd(0, 9.5, 0.2),
            Follower::Pd(1, 9.5, 0.2),
            Follower::Pd(3, 9.5, 0.2),
            Follower::Pd(0, 9.5, 0.1),
            Follower::Pd(1, 9.5, 0.1),
        ],
        0.007,
        f64::INFINITY,
    );
}

/// One bad reading turns no arm or torso joint harder than twice its acceleration cap, at
/// 90, 250 and 500 Hz: the idle right arm's elbow or the torso read 0.15 / 0.1 rad off for one
/// tick while the left hand sweeps past the box, and the left elbow 0.05 rad off on its stop
/// while the right hand reaches for the left. Taken as it came, the idle elbow's reference
/// jumped 342x its cap and a reaching shoulder turned 139x (embodiments#210, 4ea9e84); now a
/// jump no follower makes waits for the next reading to confirm it.
#[test]
fn a_single_bad_reading_moves_nothing() {
    std::thread::scope(|scope| {
        let mut runs = Vec::new();
        for dt in [0.011, 0.004, 0.002] {
            for (case, glitch) in [
                ("box", ("right_elbow_pitch_joint", -0.15)),
                ("box", ("torso_joint", 0.1)),
                ("reach", ("left_elbow_pitch_joint", 0.05)),
            ] {
                for follower in [Follower::Late(1), Follower::Pd(0, 9.5, 0.2)] {
                    runs.push(scope.spawn(move || {
                        let got =
                            least_clearance(dt, case, true, follower, READING_JITTER, Some(glitch));
                        (dt, case, glitch, follower, got.3)
                    }));
                }
            }
        }
        for run in runs {
            let (dt, case, glitch, follower, turned) = run.join().expect("a case");
            assert!(
                turned <= WITHIN_THE_CAP,
                "{case} {glitch:?} at {dt} s, {follower:?}: turned {turned}x a cap"
            );
        }
    });
}

/// A collision row no step inside the acceleration caps keeps widens no cap up front: the
/// solve finds no step, and the elastic retry keeps the rows as nearly as the caps allow, and
/// says so (`Solve::Relaxed`). Underdamped followers that ring into the box and the other hand
/// turned 7-121x a cap with the row's axes widened to their whole range (4ea9e84;
/// embodiments#210, #211).
#[test]
fn a_row_the_caps_cannot_keep_widens_no_cap() {
    std::thread::scope(|scope| {
        let runs: Vec<_> = [
            (0.002, "box", Follower::Pd(0, 9.5, 0.1), 0.0),
            (0.004, "reach", Follower::Pd(0, 9.5, 0.15), 0.0),
            (0.011, "reach", Follower::Pd(1, 9.5, 0.1), READING_JITTER),
            // With the approach paced by its room (`Settings::approach_time_s`) the three above
            // keep their rows inside the caps; these two still need more.
            (0.002, "reach", Follower::Pd(0, 9.5, 0.1), 0.0),
            (0.02, "reach", Follower::Pd(3, 9.5, 0.1), READING_JITTER),
        ]
        .into_iter()
        .map(|(dt, case, follower, noise)| {
            scope.spawn(move || {
                let got = least_clearance(dt, case, true, follower, noise, None);
                (dt, case, follower, got.3, got.4)
            })
        })
        .collect();
        let mut gave_way = 0;
        for run in runs {
            let (dt, case, follower, turned, relaxed) = run.join().expect("a case");
            assert!(
                turned <= WITHIN_THE_CAP,
                "{case} at {dt} s, {follower:?}: turned {turned}x a cap"
            );
            gave_way += relaxed;
        }
        assert!(gave_way > 0, "rows that gave way inside the caps say so");
    });
}

/// A tool speed never shortens a step past a row that asks its pair to open: shortened, then
/// held for breaking it, an underdamped follower's step broke such rows by up to 26 mm and came
/// within 3.0 mm of the box (refute of embodiments 885f73b; 8.6 mm without the tool speed);
/// shortened past it without being held, 7.0-7.9 mm at 100 and 500 Hz (its second refute).
#[test]
fn a_tool_speed_never_shortens_a_step_past_a_row() {
    std::thread::scope(|scope| {
        let mut runs = Vec::new();
        for dt in [0.02, 0.011, 0.01, 0.002] {
            for case in ["box", "reach"] {
                for follower in [Follower::Pd(0, 9.5, 0.1), Follower::Pd(1, 9.5, 0.1)] {
                    for (tool, noise) in [
                        (0.05, READING_JITTER),
                        (0.2, READING_JITTER),
                        (0.05, 0.0),
                        (0.2, 0.0),
                    ] {
                        runs.push(scope.spawn(move || {
                            let got = least_clearance_at(
                                dt,
                                case,
                                true,
                                follower,
                                noise,
                                None,
                                Some(tool),
                            );
                            (dt, case, follower, tool, got)
                        }));
                    }
                }
            }
        }
        for run in runs {
            let (dt, case, follower, tool, (clear, a, b, _, _)) = run.join().expect("a case");
            println!("{case} at {dt} s, {follower:?}, tool {tool} m/s: {a} and {b} {clear} m");
            assert!(
                clear >= 0.0085,
                "{case} at {dt} s, {follower:?}, tool {tool} m/s: {a} and {b} {clear} m apart"
            );
        }
    });
}

/// The least clearance and the right arm's sharpest turn against its acceleration cap, every
/// rate, case, follower and jitter: `cargo test --release -p sx-embodiment-ik --test teleop
/// clearance_table -- --ignored --nocapture`.
#[test]
#[ignore = "prints the clearance table"]
fn clearance_table() {
    let followers = [
        Follower::Late(0),
        Follower::Late(1),
        Follower::Late(2),
        Follower::Late(3),
        Follower::Late(4),
        Follower::Pd(0, 9.5, 0.2),
        Follower::Pd(1, 9.5, 0.2),
        Follower::Pd(3, 9.5, 0.2),
        Follower::Pd(0, 9.5, 0.15),
        Follower::Pd(0, 9.5, 0.1),
        Follower::Pd(1, 9.5, 0.1),
        Follower::Pd(0, 30.0, 0.7),
        Follower::Pd(2, 30.0, 0.7),
    ];
    std::thread::scope(|scope| {
        let mut runs = Vec::new();
        for dt in [0.02, 0.011, 0.01, 0.004, 0.002] {
            for (case, stopped) in CASES {
                for follower in followers {
                    for noise in [0.0, 0.001] {
                        runs.push(scope.spawn(move || {
                            let (c, a, b, lurch, _) = least_clearance(dt, case, stopped, follower, noise, None);
                            format!("TABLE dt={dt} {case} stop={stopped} {follower:?} noise={noise}: {:.1} mm ({a} / {b}) accel x{lurch:.1}", c * 1e3)
                        }));
                    }
                }
            }
        }
        for run in runs {
            println!("{}", run.join().expect("a case"));
        }
    });
}
