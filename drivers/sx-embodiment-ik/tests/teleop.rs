//! The shared teleoperation solve on the static upper body: where the camera looks, the holds,
//! the grippers, the reference rule, and one config both sides build from.

#![allow(clippy::float_cmp)]

use std::collections::BTreeMap;

use sx_embodiment_ik::teleop::{
    HANDS, Pose, REFERENCE_GAP, STALL, TELEOP_SCHEMA, Targets, Teleop, TeleopConfig, TeleopStep,
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
    solve.set_deadline(std::time::Duration::from_secs(1));
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
                    rotation_xyzw: from.rotation_xyzw,
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

/// The least clearance, metres, where the robot really is, from 1.5 s after the left elbow
/// first stalls on a stop (when `stopped`) to 4 s on: `case` "reach" sends the right gripper
/// to where the left one really is, "box" sweeps the left hand inward past a keep-out box
/// beside its forearm. The robot is a follower `lag` ticks behind its command, its left elbow
/// never past the stop; it is measured where it is.
fn least_clearance(dt: f64, case: &str, stopped: bool, lag: usize) -> (f64, String, String) {
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
    solve.set_deadline(std::time::Duration::from_secs(1));
    let elbow = index(&solve, "left_elbow_pitch_joint");
    let (left0, right0) = (tcp(&solve, &rest, 0), tcp(&solve, &rest, 1));
    let stop_at = rest[elbow];
    let available = vec![true; rest.len()];
    let mut q = rest.clone();
    // The commands the follower has yet to reach, oldest first.
    let mut sent = std::collections::VecDeque::from(vec![rest.clone(); lag + 1]);
    let mut real = rest.clone();
    let turn = STALL.as_secs_f64() + 1.5;
    let mut least = (f64::INFINITY, String::new(), String::new());
    solve.reset();
    for tick in 0_u32.. {
        let t = f64::from(tick) * dt;
        if t > turn + 4.0 {
            break;
        }
        let measured: Vec<Option<f64>> = real.iter().copied().map(Some).collect();
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
                    rotation_xyzw: left0.rotation_xyzw,
                }),
                None,
            ],
            ..Targets::default()
        };
        if case == "reach" && t >= turn {
            targets.tcp[1] = Some(Pose {
                translation_m: solve.model().poses(&real).links[left_tcp].t,
                rotation_xyzw: right0.rotation_xyzw,
            });
        }
        let step = solve
            .step(&q, &measured, &available, &targets, dt)
            .expect("a step");
        q.copy_from_slice(&step.q);
        sent.push_back(q.clone());
        real = sent.pop_front().expect("a command");
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
    least
}

const CASES: [(&str, bool); 4] = [
    ("reach", true),
    ("box", true),
    ("reach", false),
    ("box", false),
];

/// Every case at `rates`, a follower 0, 1 and 2 ticks behind, keeps the margin where the robot
/// really is, within 1 mm (a step's linearisation: 9.35 mm at worst, chest and right upper arm at 500 Hz).
fn keeps_the_margin(rates: &[f64]) {
    let margin = sx_embodiment_ik::Settings::default().collision_margin_m;
    for &dt in rates {
        for (case, stopped) in CASES {
            for lag in 0..=2 {
                let (clear, a, b) = least_clearance(dt, case, stopped, lag);
                assert!(
                    clear >= margin - 0.001,
                    "{case} at {dt} s, stop {stopped}, {lag} ticks behind: {a} and {b} {clear} m apart"
                );
            }
        }
    }
}

/// A pair is kept where the robot really is, a joint pushing a stop or a follower behind its
/// command alike, at the static body's 50 Hz and at 100 Hz. Planned from the command with the
/// gap clamped onto it, two ticks of lag brought the grippers to 0 mm with nothing blocked and
/// to -15 mm with the left elbow on a stop (embodiments#206; the Spark's a1dca43: -70 mm).
#[test]
fn a_pushing_arm_keeps_its_pairs_clear_where_it_really_is() {
    keeps_the_margin(&[0.02, 0.01]);
}

/// The same at 250 and 500 Hz.
#[test]
fn a_pushing_arm_keeps_its_pairs_clear_at_faster_rates() {
    keeps_the_margin(&[0.004, 0.002]);
}
