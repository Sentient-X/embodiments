//! The operator's arm in the teleoperation solve (IAIE Bimanual-Teleoperation, MIT, commit
//! 9c30cf8): shoulder-wrist geometric retargeting (SWGR) places the hand, adaptive elbow
//! control (AEAC) turns the elbow, and the posture falls back home at the reference's gain.
//! The reference's own tests (`tests/test_swgr_retargeting.py`, `tests/test_aeac_geometry.py`)
//! are ported where they test geometry, and a synthesized operator shows the robot's elbow
//! swivel moving to a person's while the tool tracks as before.

// Evaluation code: geometry's one-letter names, counts as f64, one long scripted run.
#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::float_cmp,
    clippy::many_single_char_names,
    clippy::too_many_lines,
    clippy::needless_range_loop
)]

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use sx_embodiment_ik::teleop::{
    Clutch, ELBOW_DEADBAND_M, ELBOW_FILTER_S, HUMAN_REACH_M, MAX_HANDS, OperatorArm, Targets,
    Teleop, TeleopConfig, elbow_weight, hands,
};
use sx_embodiment_ik::{Body, elbow_direction};
use sx_frames::Pose;

fn static_files() -> sx_embodiment_ik::BodyFiles<'static> {
    sx_embodiment_ik::registered("sentient-static-upper-body").expect("its pack is committed")
}

const DT: f64 = 0.01;

/// The station's served solve for the static body (governed caps, its Ready as the posture),
/// with the limbs the body's files derive today (each hand's arm named).
fn station_config(posture: bool) -> TeleopConfig {
    let mut config = TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json"))
        .expect("the station's solve config");
    config.limbs = static_files().limbs().expect("its limbs");
    if !posture {
        config.posture = BTreeMap::new();
    }
    config
}

fn solve(config: &TeleopConfig) -> Teleop {
    let mut solve =
        Teleop::new(static_files().urdf, static_files().collision, config).expect("solve");
    solve.set_deadline(Duration::from_secs(3600));
    solve
}

fn ready(teleop: &Teleop) -> Vec<f64> {
    let ready: BTreeMap<String, f64> =
        serde_json::from_str(include_str!("fixtures/static-ready.json")).expect("ready");
    teleop
        .model()
        .axes()
        .iter()
        .map(|a| ready.get(&a.name).copied().unwrap_or(0.0))
        .collect()
}

fn link(body: &Body, name: &str) -> usize {
    body.model().link(name).expect(name)
}

fn at(body: &Body, q: &[f64], name: &str) -> [f64; 3] {
    body.model().poses(q).links[link(body, name)].t
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm(v: [f64; 3]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

fn step_once(teleop: &mut Teleop, q: &mut [f64], targets: &Targets) {
    let measured: Vec<Option<f64>> = q.iter().map(|v| Some(*v)).collect();
    let available = vec![true; q.len()];
    let step = teleop
        .step(q, &measured, &available, targets, DT)
        .expect("a step");
    q.copy_from_slice(&step.q);
}

/// An operator's arm with its shoulder at `shoulder` and wrist at `wrist`, the elbow bent
/// between them (upper arm 0.30 m, forearm 0.28 m: 0.58 m, the reference's reach) and turned
/// `swivel` radians off straight down toward `outward`.
fn human_arm(shoulder: [f64; 3], wrist: [f64; 3], swivel: f64, outward: [f64; 3]) -> OperatorArm {
    let (upper, fore) = (0.30, 0.28);
    let axis = sub(wrist, shoulder);
    let length = norm(axis).min(upper + fore - 1e-6);
    let d = axis.map(|x| x / norm(axis));
    let a = (upper * upper - fore * fore + length * length) / (2.0 * length);
    let radius = (upper * upper - a * a).max(0.0).sqrt();
    let down = [0.0, 0.0, -1.0];
    let along: f64 = (0..3).map(|k| d[k] * down[k]).sum();
    let u = [0, 1, 2].map(|k| down[k] - d[k] * along);
    let u = u.map(|x| x / norm(u));
    let side: f64 = (0..3).map(|k| d[k] * outward[k]).sum();
    let v = [0, 1, 2].map(|k| outward[k] - d[k] * side);
    let v = {
        let ud: f64 = (0..3).map(|k| v[k] * u[k]).sum();
        let v = [0, 1, 2].map(|k| v[k] - u[k] * ud);
        v.map(|x| x / norm(v))
    };
    let n = [0, 1, 2].map(|k| swivel.cos() * u[k] + swivel.sin() * v[k]);
    OperatorArm {
        shoulder_m: shoulder,
        elbow_m: [0, 1, 2].map(|k| shoulder[k] + a * d[k] + radius * n[k]),
        wrist_m: wrist,
    }
}

// ---- The reference's geometry, ported ----

/// `test_full_extension_lands_on_robot_reach`, `test_bent_arm_scales_proportionally`,
/// `test_direction_is_preserved` and `test_vr_world_origin_cancels`, through the solve: the hand
/// the solve takes is the robot's shoulder plus the operator's shoulder-to-wrist vector times
/// the robot's reach over theirs, wherever the operator stands.
#[test]
fn swgr_places_the_hand_by_the_shoulder_wrist_vector_scaled_to_the_robots_reach() {
    let config = station_config(true);
    let mut teleop = solve(&config);
    let body = teleop.ik().body().clone();
    let q0 = ready(&teleop);
    let reach = teleop.arm_reach()[0].expect("the left hand has an arm");
    // The arm stretched straight: shoulder roll, upper arm, elbow, forearm, wrist, gripper.
    assert!((0.6..0.75).contains(&reach), "reach {reach}");
    let shoulder_robot = at(&body, &q0, "left_shoulder_roll_link");
    let tool = teleop.tool_poses(&q0)[0].expect("left tool");
    let cases: [([f64; 3], [f64; 3]); 4] = [
        // Full extension along +x lands on the robot's reach.
        ([0.0, 0.0, 1.4], [HUMAN_REACH_M, 0.0, 0.0]),
        // Half the reach, bent off every axis, lands on half the robot's.
        ([0.1, -0.3, 1.2], [0.0, 0.6 * 0.29, -0.8 * 0.29]),
        ([0.2, 0.1, 1.3], [0.31, -0.42, 0.19]),
        // The same arm 4 m away: the origin cancels.
        ([3.9, -2.0, 2.1], [0.31, -0.42, 0.19]),
    ];
    let mut placed = Vec::new();
    for (shoulder, span) in cases {
        let wrist = [0, 1, 2].map(|k| shoulder[k] + span[k]);
        let arm = human_arm(shoulder, wrist, 0.3, [0.0, 1.0, 0.0]);
        let targets = Targets {
            tcp: hands([Some(tool)]),
            arm: hands([Some(arm)]),
            ..Targets::default()
        };
        teleop.reset();
        let mut q = q0.clone();
        step_once(&mut teleop, &mut q, &targets);
        let taken = teleop.taken().tcp[0].expect("taken");
        let got = sub(taken.translation_m, shoulder_robot);
        let expected = span.map(|x| x * reach / HUMAN_REACH_M);
        for k in 0..3 {
            assert!(
                (got[k] - expected[k]).abs() < 1e-9,
                "{got:?} against {expected:?}"
            );
        }
        // The turn is the hand's own: no clutch offset, no change.
        assert_eq!(taken.rotation, tool.rotation);
        placed.push(taken.translation_m);
    }
    assert!((norm(sub(placed[0], shoulder_robot)) - reach).abs() < 1e-9);
    assert!((norm(sub(placed[1], shoulder_robot)) - reach / 2.0).abs() < 1e-9);
    assert!(norm(sub(placed[2], placed[3])) < 1e-12);
}

/// `test_elbow_direction_turns_off_when_arm_is_straight`,
/// `test_elbow_direction_is_perpendicular_to_shoulder_wrist_axis`,
/// `test_weight_ramps_after_deadband`.
#[test]
fn aeac_reads_the_elbow_across_the_shoulder_wrist_line_and_weighs_it_past_the_deadband() {
    assert!(elbow_direction([0.0; 3], [0.2, 0.0, 0.0], [0.4, 0.0, 0.0]).is_none());
    let (direction, radius) =
        elbow_direction([0.0; 3], [0.5, 0.2, 0.0], [1.0, 0.0, 0.0]).expect("bent");
    for (got, want) in direction.iter().zip([0.0, 1.0, 0.0]) {
        assert!((got - want).abs() < 1e-12);
    }
    assert!((radius - 0.2).abs() < 1e-12);
    assert!(elbow_weight(0.2, HUMAN_REACH_M) > 0.0);
    assert_eq!(elbow_weight(0.02, HUMAN_REACH_M), 0.0);
    assert_eq!(elbow_weight(ELBOW_DEADBAND_M, HUMAN_REACH_M), 0.0);
    assert_eq!(elbow_weight(1.0, HUMAN_REACH_M), 1.0);
    let mid = elbow_weight(0.2, HUMAN_REACH_M);
    assert!((mid - (0.2 - 0.04) / (0.58 - 0.04)).abs() < 1e-12);
    // The reference's filter is 0.35 of the way a tick at 60 Hz: the same time constant.
    let blend = 1.0 - (-(1.0 / 60.0) / ELBOW_FILTER_S).exp();
    assert!((blend - 0.35).abs() < 1e-4, "{blend}");
}

/// Without the operator's arm nothing changes: the same steps, to the bit, whether or not the
/// config names each hand's arm, whatever the operator's reach.
#[test]
fn a_hand_without_the_operators_arm_steps_exactly_as_before() {
    let with = station_config(true);
    let mut without = with.clone();
    for limb in &mut without.limbs {
        limb.arm = None;
    }
    without.human_reach_m = 0.7;
    let (mut a, mut b) = (solve(&with), solve(&without));
    let mut qa = ready(&a);
    let mut qb = qa.clone();
    let tools = a.tool_poses(&qa);
    for tick in 0..150 {
        let t = tick as f64 * DT;
        let mut left = tools[0].expect("left");
        left.translation_m[1] += 0.05 * (3.0 * t).sin();
        left.translation_m[2] += 0.03 * (2.0 * t).sin();
        let targets = Targets {
            tcp: hands([Some(left), tools[1]]),
            ..Targets::default()
        };
        step_once(&mut a, &mut qa, &targets);
        step_once(&mut b, &mut qb, &targets);
        assert_eq!(qa, qb, "tick {tick}");
    }
}

/// The clutch leaves a hand with an arm where its arm puts it and only turns it; a hand that
/// gains or loses its arm takes hold afresh where the gripper stands.
#[test]
fn the_clutch_never_offsets_a_hand_placed_by_its_arm() {
    let mut clutch = Clutch::default();
    let gripper = Pose {
        translation_m: [0.4, 0.2, 0.1],
        rotation: sx_frames::Quaternion::from_matrix(&sx_frames::rotation_vector_matrix([
            0.0, 0.3, 0.0,
        ])),
    };
    let hand = Pose {
        translation_m: [1.0, 1.0, 1.0],
        rotation: sx_frames::Quaternion::IDENTITY,
    };
    let arm = human_arm([0.0; 3], [0.3, 0.0, -0.2], 0.2, [0.0, 1.0, 0.0]);
    let tools: [Option<Pose>; MAX_HANDS] = hands([Some(gripper)]);
    let mut targets = Targets {
        tcp: hands([Some(hand)]),
        arm: hands([Some(arm)]),
        ..Targets::default()
    };
    assert!(clutch.takes_hold(&targets));
    clutch.apply(&tools, &mut targets);
    let held = targets.tcp[0].expect("held");
    assert_eq!(held.translation_m, hand.translation_m);
    assert!(
        norm(sub(
            held.rotation.rotate([1.0, 0.0, 0.0]),
            gripper.rotation.rotate([1.0, 0.0, 0.0])
        )) < 1e-12
    );
    // Losing the arm: taken hold of afresh, clutched where the gripper stands.
    let mut targets = Targets {
        tcp: hands([Some(hand)]),
        ..Targets::default()
    };
    assert!(clutch.takes_hold(&targets));
    clutch.apply(&tools, &mut targets);
    let held = targets.tcp[0].expect("held");
    for k in 0..3 {
        assert!((held.translation_m[k] - gripper.translation_m[k]).abs() < 1e-12);
    }
    assert!(!clutch.takes_hold(&targets));
}

// ---- The robot's elbow, against a synthesized operator ----

/// The elbow's swivel from straight down, degrees (robot-bench `scripts/elbow_swivel.rs`).
fn swivel(s: [f64; 3], e: [f64; 3], w: [f64; 3]) -> Option<f64> {
    let (off, radius) = elbow_direction(s, e, w)?;
    if radius < 0.01 {
        return None;
    }
    let axis = sub(w, s);
    let d = axis.map(|x| x / norm(axis));
    let along = -d[2];
    let across = [-d[0] * along, -d[1] * along, -1.0 - d[2] * along];
    let cos = (0..3).map(|k| off[k] * across[k]).sum::<f64>() / norm(across).max(1e-12);
    Some(cos.clamp(-1.0, 1.0).acos().to_degrees())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Scenario {
    Reach,
    Sweep,
    Table,
    /// The table's circles with the operator's elbow raised 70 degrees off down.
    ElbowOut,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Variant {
    /// Today: the hand given without the arm (as the clutch gives it), Ready preferred.
    Clutched,
    /// The same with no preferred posture: drawn to the posture at engage.
    ClutchedNoPosture,
    /// SWGR and home, the operator's elbow on their line (no aim): the reference with
    /// `elbow_gain` 0, its default.
    Swgr,
    /// SWGR, AEAC and home: the reference with `--elbow-gain 0.5`.
    SwgrAeac,
    /// The same on a robot whose config serves no posture: home is the posture at engage.
    SwgrAeacNoPosture,
}

/// The Spark's commanded arms at the first command of session 8b3e2f2b (2026-11-14): the left
/// upper arm turned -1.86 rad, its elbow out to the side, where it stayed for 25 s.
const TWISTED: [(&str, f64); 15] = [
    ("torso_joint", -0.0330),
    ("left_shoulder_pitch_joint", -0.2718),
    ("left_shoulder_roll_joint", -0.0051),
    ("left_upper_arm_yaw_joint", -1.8580),
    ("left_elbow_pitch_joint", -1.4158),
    ("left_forearm_yaw_joint", 1.2926),
    ("left_wrist_pitch_joint", -0.0250),
    ("left_wrist_roll_joint", -0.8822),
    ("right_shoulder_pitch_joint", 0.0914),
    ("right_shoulder_roll_joint", 0.0715),
    ("right_upper_arm_yaw_joint", 0.3466),
    ("right_elbow_pitch_joint", 1.3140),
    ("right_forearm_yaw_joint", 0.0467),
    ("right_wrist_pitch_joint", -0.2344),
    ("right_wrist_roll_joint", -0.0292),
];

struct Outcome {
    /// Per 30-degree bin, both arms' commands.
    bins: [usize; 6],
    under_60: f64,
    /// The tool against the target the solve took, after the first second: RMS and 95th
    /// percentile, mm; and the turn's RMS, mrad.
    track_rms_mm: f64,
    track_p95_mm: f64,
    turn_rms_mrad: f64,
    tick_median_us: f64,
    /// The arms' joints' jerk, RMS over joints and ticks, rad/s^3.
    jerk_rms: f64,
    /// The robot's mean swivel against the operator's, degrees.
    robot_mean_deg: f64,
    operator_mean_deg: f64,
}

/// The operator's wrists relative to their shoulders at `t` (x forward, y left, z up; the right
/// arm mirrored), and each elbow's swivel off down, outward.
fn operator(scenario: Scenario, t: f64, side: f64) -> ([f64; 3], f64) {
    let tau = std::f64::consts::TAU;
    let wrist = match scenario {
        Scenario::Reach => {
            let s = 0.5
                - 0.5 * (tau * (t - 1.0).max(0.0) / 2.0 + if side > 0.0 { 0.0 } else { 1.0 }).cos();
            let near = [0.22, 0.06, -0.33];
            let far = [0.46, 0.0, -0.12];
            [0, 1, 2].map(|k| near[k] + s * (far[k] - near[k]))
        }
        Scenario::Sweep => [
            0.34,
            0.04 + 0.14 * (tau * (t - 1.0).max(0.0) / 2.5).sin(),
            -0.24,
        ],
        Scenario::Table | Scenario::ElbowOut => {
            let w = tau * 0.6 * (t - 1.0).max(0.0) + if side > 0.0 { 0.0 } else { 1.5 };
            [
                0.33 + 0.07 * w.cos(),
                -0.03 + 0.07 * w.sin(),
                -0.30 + 0.02 * (2.0 * w).sin(),
            ]
        }
    };
    let wrist = [wrist[0], side * wrist[1], wrist[2]];
    let swivel = if scenario == Scenario::ElbowOut {
        70.0 + 5.0 * (1.3 * t).sin()
    } else {
        20.0 + 10.0 * (1.3 * t).sin()
    };
    (wrist, swivel.to_radians())
}

fn run(scenario: Scenario, variant: Variant, seconds: f64, twisted: bool) -> Outcome {
    let config = station_config(
        variant != Variant::ClutchedNoPosture && variant != Variant::SwgrAeacNoPosture,
    );
    let mut teleop = solve(&config);
    let body = teleop.ik().body().clone();
    let mut q = ready(&teleop);
    if twisted {
        for (name, value) in TWISTED {
            let axis = teleop
                .model()
                .axes()
                .iter()
                .position(|a| a.name == name)
                .expect(name);
            q[axis] = value;
        }
    }
    let tools = teleop.tool_poses(&q);
    let sides = [("left", 1.0), ("right", -1.0)];
    let shoulders = sides.map(|(side, _)| at(&body, &q, &format!("{side}_shoulder_roll_link")));
    let reach = teleop.arm_reach()[0].expect("arm");
    let mut bins = [0; 6];
    let (mut operator_sum, mut robot_sum, mut robot_count) = (0.0, 0.0, 0usize);
    let arm_axes: Vec<usize> = teleop
        .model()
        .axes()
        .iter()
        .enumerate()
        .filter(|(_, a)| {
            (a.name.starts_with("left_") || a.name.starts_with("right_"))
                && !a.name.contains("gripper")
        })
        .map(|(i, _)| i)
        .collect();
    let mut history: Vec<Vec<f64>> = Vec::new();
    let (mut errors, mut turns, mut times) = (Vec::new(), Vec::new(), Vec::new());
    let ticks = (seconds / DT) as usize;
    let measured_links: Vec<[usize; 3]> = sides
        .iter()
        .map(|(side, _)| {
            [
                link(&body, &format!("{side}_shoulder_roll_link")),
                link(&body, &format!("{side}_elbow_pitch_link")),
                link(&body, &format!("{side}_wrist_pitch_link")),
            ]
        })
        .collect();
    teleop.reset();
    for tick in 1..=ticks {
        let t = tick as f64 * DT;
        let mut targets = Targets::default();
        for (hand, (_, side)) in sides.iter().enumerate() {
            let (wrist, swivel_rad) = operator(scenario, t, *side);
            let tool = tools[hand].expect("tool");
            let s = shoulders[hand];
            let w = [0, 1, 2].map(|k| s[k] + wrist[k]);
            let arm = human_arm(s, w, swivel_rad, [0.0, *side, 0.0]);
            if let Some(operator_swivel) = swivel(arm.shoulder_m, arm.elbow_m, arm.wrist_m)
                && t >= 1.0
            {
                operator_sum += operator_swivel;
            }
            match variant {
                Variant::Clutched | Variant::ClutchedNoPosture => {
                    // The same hand as the arm would place, given as the clutch gives a hand:
                    // only the null space differs.
                    let mut pose = tool;
                    pose.translation_m = [0, 1, 2].map(|k| s[k] + wrist[k] * reach / HUMAN_REACH_M);
                    targets.tcp[hand] = Some(pose);
                }
                Variant::Swgr | Variant::SwgrAeac | Variant::SwgrAeacNoPosture => {
                    let mut arm = arm;
                    if variant == Variant::Swgr {
                        arm.elbow_m = [0, 1, 2].map(|k| 0.5 * (s[k] + w[k]));
                    }
                    targets.tcp[hand] = Some(tool);
                    targets.arm[hand] = Some(arm);
                }
            }
        }
        let measured: Vec<Option<f64>> = q.iter().map(|v| Some(*v)).collect();
        let available = vec![true; q.len()];
        let began = Instant::now();
        let step = teleop
            .step(&q, &measured, &available, &targets, DT)
            .expect("a step");
        times.push(began.elapsed().as_secs_f64() * 1e6);
        q.copy_from_slice(&step.q);
        if t < 1.0 {
            continue;
        }
        history.push(arm_axes.iter().map(|&i| q[i]).collect());
        let frames = body.model().poses(&q);
        let now = teleop.tool_poses(&q);
        for hand in 0..2 {
            let [s, e, w] = measured_links[hand];
            if let Some(angle) = swivel(frames.links[s].t, frames.links[e].t, frames.links[w].t) {
                bins[((angle / 30.0) as usize).min(5)] += 1;
                robot_sum += angle;
                robot_count += 1;
            }
            let (Some(target), Some(tool)) = (teleop.taken().tcp[hand], now[hand]) else {
                continue;
            };
            errors.push(norm(sub(tool.translation_m, target.translation_m)) * 1e3);
            let turn = target.rotation.conjugate() * tool.rotation;
            let sine = (turn.x * turn.x + turn.y * turn.y + turn.z * turn.z).sqrt();
            turns.push(2.0 * sine.atan2(turn.w.abs()) * 1e3);
        }
    }
    let rms = |v: &[f64]| (v.iter().map(|x| x * x).sum::<f64>() / v.len().max(1) as f64).sqrt();
    let mut sorted = errors.clone();
    sorted.sort_by(f64::total_cmp);
    times.sort_by(f64::total_cmp);
    let total: usize = bins.iter().sum();
    let mut jerks = Vec::new();
    for window in history.windows(4) {
        for j in 0..arm_axes.len() {
            let third = window[3][j] - 3.0 * window[2][j] + 3.0 * window[1][j] - window[0][j];
            jerks.push(third / DT.powi(3));
        }
    }
    Outcome {
        bins,
        under_60: (bins[0] + bins[1]) as f64 / total.max(1) as f64,
        track_rms_mm: rms(&errors),
        track_p95_mm: sorted[(0.95 * sorted.len() as f64) as usize],
        turn_rms_mrad: rms(&turns),
        tick_median_us: times[times.len() / 2],
        jerk_rms: rms(&jerks),
        robot_mean_deg: robot_sum / robot_count.max(1) as f64,
        operator_mean_deg: operator_sum / robot_count.max(1) as f64,
    }
}

/// Reaching, sweeping and working at a table with both hands: with the operator's arm the robot's
/// elbows hang as a person's do (most commands under 60 degrees off down), a raised operator
/// elbow is followed, an arm engaged twisted (session 8b3e2f2b) unwinds even with no posture
/// served, and the tool tracks what it is asked about as closely as the clutched hand does today.
#[test]
fn with_the_operators_arm_the_elbows_hang_as_a_persons_and_the_tool_tracks_as_before() {
    for scenario in [Scenario::Reach, Scenario::Sweep, Scenario::Table] {
        let today = run(scenario, Variant::Clutched, 5.0, false);
        let arm = run(scenario, Variant::SwgrAeac, 5.0, false);
        assert!(
            arm.under_60 > 0.6,
            "{scenario:?}: {:.2} under 60",
            arm.under_60
        );
        assert!(
            arm.track_rms_mm <= today.track_rms_mm * 1.1 + 0.5,
            "{scenario:?}: tracking {:.2} mm against {:.2}",
            arm.track_rms_mm,
            today.track_rms_mm
        );
    }
    let raised = run(Scenario::ElbowOut, Variant::SwgrAeac, 5.0, false);
    assert!(
        (raised.robot_mean_deg - raised.operator_mean_deg).abs() < 15.0,
        "the robot's elbow at {:.0} against the operator's {:.0}",
        raised.robot_mean_deg,
        raised.operator_mean_deg
    );
    let today = run(Scenario::Table, Variant::ClutchedNoPosture, 5.0, true);
    let arm = run(Scenario::Table, Variant::SwgrAeacNoPosture, 5.0, true);
    assert!(
        today.under_60 < 0.5 && arm.under_60 > 0.85,
        "{:.2} -> {:.2}",
        today.under_60,
        arm.under_60
    );
}

/// The table: every scenario and variant, swivel bins, tracking and tick time.
#[test]
#[ignore = "a table: cargo test --release --test arm_retarget table_elbow_swivel -- --ignored --nocapture"]
fn table_elbow_swivel() {
    println!(
        "scenario variant | swivel share 0-30 30-60 60-90 90-120 120-150 150-180 | <60 | robot / operator mean deg | track rms / p95 mm | turn rms mrad | arm jerk rms rad/s3 | tick median us"
    );
    for (scenario, twisted) in [
        (Scenario::Reach, false),
        (Scenario::Sweep, false),
        (Scenario::Table, false),
        (Scenario::ElbowOut, false),
        (Scenario::Table, true),
    ] {
        for variant in [
            Variant::Clutched,
            Variant::ClutchedNoPosture,
            Variant::Swgr,
            Variant::SwgrAeac,
            Variant::SwgrAeacNoPosture,
        ] {
            let o = run(scenario, variant, 8.0, twisted);
            let scenario = format!("{scenario:?}{}", if twisted { " from twisted" } else { "" });
            let total: usize = o.bins.iter().sum();
            let shares: Vec<String> = o
                .bins
                .iter()
                .map(|b| format!("{:3.0}", 100.0 * *b as f64 / total.max(1) as f64))
                .collect();
            println!(
                "{scenario} {variant:?} | {} | {:.0}% | {:.0} / {:.0} | {:.2} / {:.2} | {:.1} | {:.0} | {:.0}",
                shares.join(" "),
                100.0 * o.under_60,
                o.robot_mean_deg,
                o.operator_mean_deg,
                o.track_rms_mm,
                o.track_p95_mm,
                o.turn_rms_mrad,
                o.jerk_rms,
                o.tick_median_us
            );
        }
    }
}

/// The left hand held where it stands at Ready, the operator's elbow turned `aim` degrees off
/// down (outward), for `seconds`: the robot's swivel each second, degrees.
fn aimed(aim: f64, seconds: f64) -> Vec<f64> {
    let config = station_config(true);
    let mut teleop = solve(&config);
    let body = teleop.ik().body().clone();
    let mut q = ready(&teleop);
    let tools = teleop.tool_poses(&q);
    let reach = teleop.arm_reach()[0].expect("arm");
    let s = at(&body, &q, "left_shoulder_roll_link");
    let tool = tools[0].expect("left");
    let w = [0, 1, 2].map(|k| s[k] + (tool.translation_m[k] - s[k]) * HUMAN_REACH_M / reach);
    let arm = human_arm(s, w, aim.to_radians(), [0.0, 1.0, 0.0]);
    let targets = Targets {
        tcp: hands([Some(tool), tools[1]]),
        arm: hands([Some(arm)]),
        ..Targets::default()
    };
    let links = [
        "left_shoulder_roll_link",
        "left_elbow_pitch_link",
        "left_wrist_pitch_link",
    ];
    let mut out = Vec::new();
    teleop.reset();
    for tick in 1..=((seconds / DT) as usize) {
        step_once(&mut teleop, &mut q, &targets);
        if tick % 100 == 0 {
            let [a, b, c] = links.map(|n| at(&body, &q, n));
            out.push(swivel(a, b, c).unwrap_or(f64::NAN));
        }
    }
    out
}

/// The elbow alone: the hand held still, the operator's elbow turned off down by 0-120
/// degrees; the robot's swivel each second.
#[test]
#[ignore = "a table: cargo test --release --test arm_retarget table_elbow_aim -- --ignored --nocapture"]
fn table_elbow_aim() {
    let config = station_config(true);
    let teleop = solve(&config);
    let body = teleop.ik().body().clone();
    let q = ready(&teleop);
    let links = [
        "left_shoulder_roll_link",
        "left_elbow_pitch_link",
        "left_wrist_pitch_link",
    ];
    let [a, b, c] = links.map(|n| at(&body, &q, n));
    println!(
        "Ready's own swivel: {:.0} degrees",
        swivel(a, b, c).unwrap_or(f64::NAN)
    );
    for aim in [0.0, 30.0, 60.0, 90.0, 120.0] {
        let row: Vec<String> = aimed(aim, 6.0).iter().map(|v| format!("{v:5.1}")).collect();
        println!(
            "operator elbow {aim:5.0} deg: robot swivel at 1..6 s {}",
            row.join(" ")
        );
    }
}
