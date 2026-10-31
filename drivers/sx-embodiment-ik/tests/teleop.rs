//! The shared teleoperation solve on the static upper body: where the camera looks, the holds,
//! the grippers, the reference rule, and one config both sides build from.

#![allow(clippy::float_cmp)]

use std::collections::BTreeMap;

use sx_embodiment_ik::teleop::{
    HANDS, Pose, REFERENCE_GAP, STALL, TELEOP_SCHEMA, Targets, Teleop, TeleopConfig,
};
use sx_embodiment_ik::{Body, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF};

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
    Teleop::new(
        STATIC_UPPER_BODY_URDF,
        STATIC_UPPER_BODY_COLLISION,
        &config(),
    )
    .expect("solve")
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
    let frame = body.frame(&body.model().poses(q), hand).to_transform();
    let p = sx_embodiment_drivers::kinematics::pose_of(&frame);
    Pose {
        translation_m: p.translation_m,
        rotation_xyzw: p.rotation_xyzw,
    }
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
        rotation_xyzw: [0.0, 0.0, (yaw / 2.0).sin(), (yaw / 2.0).cos()],
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
    // Within the gap the reference wins; past it, the reference is held at the gap.
    let mut reference = step.q.clone();
    measured[elbow] = Some(q[elbow] + REFERENCE_GAP / 2.0);
    let step = solve
        .step(&reference, &measured, &available, &Targets::default(), DT)
        .expect("a step");
    assert_eq!(step.q[elbow], q[elbow]);
    reference.clone_from(&step.q);
    measured[elbow] = Some(q[elbow] + REFERENCE_GAP * 2.0);
    let step = solve
        .step(&reference, &measured, &available, &Targets::default(), DT)
        .expect("a step");
    assert!((step.q[elbow] - (q[elbow] + REFERENCE_GAP)).abs() < 1e-12);
}

#[test]
fn following_starts_only_with_every_driven_hand_at_its_gripper() {
    let mut solve = teleop();
    let q = start(&solve);
    let (left, right) = (tcp(&solve, &q, 0), tcp(&solve, &q, 1));
    let at = |left: Option<Pose>, right: Option<Pose>| Targets {
        tcp: [left, right],
        ..Targets::default()
    };
    // No hand driven: nothing to start from.
    assert!(!solve.may_engage(&q, &at(None, None)));
    assert!(solve.may_engage(&q, &at(Some(left), Some(right))));
    assert!(solve.may_engage(&q, &at(Some(left), None)));
    let mut near = left;
    near.translation_m[0] += 0.05;
    assert!(solve.may_engage(&q, &at(Some(near), Some(right))));
    let mut far = left;
    far.translation_m[2] += 0.07;
    assert!(!solve.may_engage(&q, &at(Some(far), Some(right))));
    let gaps = solve.hand_gaps(&q, &at(Some(far), None));
    assert!((gaps[0].expect("left") - 0.07).abs() < 1e-9 && gaps[1].is_none());
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
            rotation_xyzw: turned([0.0, half.sin(), 0.0, half.cos()], from.rotation_xyzw),
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
                rotation_xyzw: from.rotation_xyzw,
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
                rotation_xyzw: from.rotation_xyzw,
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
