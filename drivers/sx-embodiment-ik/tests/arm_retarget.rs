//! The operator's arm in the teleoperation solve (IAIE Bimanual-Teleoperation, MIT, commit
//! 9c30cf8): adaptive elbow control (AEAC) turns the elbow and the posture falls back home at
//! the reference's gain, while the hand stays clutched as every hand is. The reference's
//! shoulder-wrist geometric retargeting (SWGR) placed the hand absolutely and was taken out
//! (refute of 2026-11-14): the probes it failed are tests here (a press, a head turn swinging
//! an estimated shoulder, a tracked/estimated flicker). The reference's AEAC geometry test is
//! ported, and a synthesized operator shows the robot's elbow swivel moving to a person's
//! while the tool tracks as before.

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

/// What the arms' redundancy resolves toward.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Posture {
    /// The robot's commissioned Ready, as the fixture's served config carries it.
    Commissioned,
    /// The body's own declared home (`BodyFiles::ready`, the registry chains' Ready): what the
    /// station serves since 2026-11-14 where the robot commissioned none.
    Body,
    /// None served: the posture at engage, the station before 2026-11-14 on a robot with no
    /// commissioned Ready.
    Engage,
}

/// The station's served solve for the static body (governed caps, `posture`), with the limbs
/// the body's files derive today (each hand's arm named).
fn station_config_with(posture: Posture) -> TeleopConfig {
    let mut config = TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json"))
        .expect("the station's solve config");
    config.limbs = static_files().limbs().expect("its limbs");
    match posture {
        Posture::Commissioned => {}
        Posture::Body => {
            config.posture = static_files()
                .ready()
                .expect("its home")
                .into_iter()
                .filter(|(joint, _)| config.posture.contains_key(joint))
                .collect();
        }
        Posture::Engage => config.posture = BTreeMap::new(),
    }
    config
}

fn station_config(posture: bool) -> TeleopConfig {
    station_config_with(if posture {
        Posture::Commissioned
    } else {
        Posture::Engage
    })
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

// ---- The hand is the clutch's: what the operator's arm must never do to it ----

/// The static body standing at Ready, the station's served solve with the body's home, the
/// left gripper's tool point there, and an operator's left arm standing 1 m off the robot (a
/// person beside it, or anywhere in their own room) whose hand is the controller.
fn probe_start() -> (Teleop, Vec<f64>, Pose, [f64; 3], [f64; 3]) {
    let mut teleop = solve(&station_config_with(Posture::Body));
    teleop.reset();
    let q = ready(&teleop);
    let tool = teleop.tool_poses(&q)[0].expect("left tool");
    let shoulder = [-0.6, 0.9, 1.4];
    let hand = [-0.25, 1.05, 1.05];
    (teleop, q, tool, shoulder, hand)
}

/// `ticks` steps of the left hand clutched at `hand` (its turn the tool's), the operator's arm
/// from `arm_at(tick)`, from Ready: the farthest the tool went from where it stood, millimetres,
/// and the most it moved in one tick.
fn clutched_run(
    ticks: usize,
    hand: [f64; 3],
    arm_at: impl Fn(usize) -> Option<OperatorArm>,
) -> (f64, f64) {
    let (mut teleop, mut q, tool, _, _) = probe_start();
    let mut clutch = Clutch::default();
    let (mut farthest, mut most, mut last) = (0.0_f64, 0.0_f64, tool.translation_m);
    for tick in 0..ticks {
        let mut targets = Targets {
            tcp: hands([Some(Pose {
                translation_m: hand,
                ..tool
            })]),
            arm: hands([arm_at(tick)]),
            ..Targets::default()
        };
        let tools = if clutch.takes_hold(&targets) {
            teleop.tool_poses(&q)
        } else {
            [None; MAX_HANDS]
        };
        clutch.apply(&tools, &mut targets);
        step_once(&mut teleop, &mut q, &targets);
        let now = teleop.tool_poses(&q)[0].expect("left tool").translation_m;
        farthest = farthest.max(norm(sub(now, tool.translation_m)) * 1e3);
        most = most.max(norm(sub(now, last)) * 1e3);
        last = now;
    }
    (farthest, most)
}

/// A press (the deadman taken, beside the robot or through it) with the operator's arm 1 m off
/// the robot moves the tool no more than the same press without the arm: the hand is held
/// where the gripper stands. With SWGR's absolute place it threw the tool 0.89 m at 2.1 m/s.
#[test]
fn a_press_with_the_operators_arm_moves_the_tool_no_more_than_without_it() {
    let (_, _, _, shoulder, hand) = probe_start();
    let arm = human_arm(shoulder, sub(hand, [0.0, 0.0, -0.02]), 0.3, [0.0, 1.0, 0.0]);
    let (with, _) = clutched_run(100, hand, |_| Some(arm));
    let (without, _) = clutched_run(100, hand, |_| None);
    assert!(with < 5.0, "the press moved the tool {with:.1} mm");
    assert!(
        (with - without).abs() < 0.5,
        "{with:.2} mm with the arm, {without:.2} without"
    );
}

/// A still hand while the head turns 30 degrees: the page's estimated shoulders hang off the
/// head's yaw and swing 0.10 m. The tool stays where it is (the arm only aims the elbow). With
/// SWGR's shoulder-anchored place it moved 0.12 m.
#[test]
fn a_head_turn_that_swings_the_estimated_shoulder_leaves_a_still_hand_still() {
    let (_, _, _, shoulder, hand) = probe_start();
    let neck = [shoulder[0], shoulder[1] - 0.19, shoulder[2] + 0.1];
    let (farthest, _) = clutched_run(150, hand, |tick| {
        // Turning over 0.5 s from 0.2 s, to 30 degrees.
        let yaw = (30.0_f64.to_radians() * ((tick as f64 * DT - 0.2) / 0.5)).clamp(0.0, 0.52);
        let r = sub(shoulder, neck);
        let turned = [
            neck[0] + yaw.cos() * r[0] - yaw.sin() * r[1],
            neck[1] + yaw.sin() * r[0] + yaw.cos() * r[1],
            shoulder[2],
        ];
        Some(human_arm(turned, hand, 0.3, [0.0, 1.0, 0.0]))
    });
    let swing = 2.0 * 0.19 * (15.0_f64.to_radians()).sin();
    assert!(swing > 0.09, "the shoulder swings {swing:.3} m");
    assert!(
        farthest < 5.0,
        "the still hand's tool moved {farthest:.1} mm"
    );
}

/// The page flipping between body tracking and its estimate every frame (the shoulder 5 cm
/// apart between the two) gives the tool no buzz. With SWGR each flip stepped the hand by 1.16
/// times the shoulder's jump, a buzzing hand at 1 m/s.
#[test]
fn a_flickering_shoulder_gives_the_tool_no_buzz() {
    let (_, _, _, shoulder, hand) = probe_start();
    let (farthest, most) = clutched_run(150, hand, |tick| {
        let jump = if tick % 2 == 0 { 0.0 } else { 0.05 };
        Some(human_arm(
            [shoulder[0], shoulder[1] + jump, shoulder[2]],
            hand,
            0.3,
            [0.0, 1.0, 0.0],
        ))
    });
    assert!(farthest < 5.0, "the tool wandered {farthest:.1} mm");
    assert!(most < 0.5, "the tool jumped {most:.2} mm in a tick");
}

/// A sample that comes without the arm (a page that lost its head pose for a frame) steps
/// exactly as if the last arm had come, within [`ARM_HOLD_S`]: the elbow stays aimed and the
/// home pull stays on, instead of both dropping and coming back.
#[test]
fn an_arm_missing_for_a_sample_is_kept() {
    let (mut a, mut qa, tool, shoulder, hand) = probe_start();
    let (mut b, _, _, _, _) = probe_start();
    let mut qb = qa.clone();
    let arm = human_arm(shoulder, hand, 0.8, [0.0, 1.0, 0.0]);
    for tick in 0..60 {
        let given = Targets {
            tcp: hands([Some(tool)]),
            arm: hands([Some(arm)]),
            ..Targets::default()
        };
        let dropped = Targets {
            arm: if tick % 7 == 3 {
                hands([None])
            } else {
                given.arm
            },
            ..given
        };
        step_once(&mut a, &mut qa, &given);
        step_once(&mut b, &mut qb, &dropped);
        assert_eq!(qa, qb, "tick {tick}");
    }
}

// ---- The reference's geometry, ported ----

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

/// Without the operator's arm a config that names each hand's arm hangs its elbows
/// (`ELBOW_PRIOR_WEIGHT`, drawn home): the arm moves in what the tool leaves free, and the
/// tool tracks as it does with no arms named, within 2 mm (1.04 at worst on this sweep; the aim
/// is a weight, not a priority), whatever the operator's reach.
#[test]
fn a_hand_without_the_operators_arm_hangs_its_elbow_and_tracks_as_before() {
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
        for (hand, (got, want)) in a.tool_poses(&qa).iter().zip(b.tool_poses(&qb)).enumerate() {
            let (Some(got), Some(want)) = (got, want) else {
                assert_eq!(got.is_some(), want.is_some(), "hand {hand}");
                continue;
            };
            let apart = (0..3)
                .map(|k| (got.translation_m[k] - want.translation_m[k]).powi(2))
                .sum::<f64>()
                .sqrt();
            assert!(apart < 2e-3, "tick {tick} hand {hand}: {apart} m apart");
        }
    }
}

/// The clutch holds a hand with an arm as any hand: offset onto the gripper at the hold, and
/// gaining or losing its arm takes no new hold.
#[test]
fn the_clutch_holds_a_hand_with_an_arm_as_any_hand() {
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
    for k in 0..3 {
        assert!((held.translation_m[k] - gripper.translation_m[k]).abs() < 1e-12);
    }
    let mut targets = Targets {
        tcp: hands([Some(hand)]),
        ..Targets::default()
    };
    assert!(!clutch.takes_hold(&targets));
    clutch.apply(&[None; MAX_HANDS], &mut targets);
    assert_eq!(targets.tcp[0], Some(held));
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
    /// No arm, the body's own home: the station since 2026-11-14 on a robot with no
    /// commissioned Ready, and the arm-less path everywhere the page sends no arm (the desk).
    ClutchedHome,
    /// SWGR, AEAC and the body's home with the arm as the page estimates it when body tracking
    /// is unavailable (`platform/console/src/workspaces/fleet/xr/arm.ts` `elbowOf`, ported in
    /// [`estimated_elbow`]): the elbow from where the hand is, not from the operator's own.
    SwgrEstimated,
    /// SWGR, AEAC and the body's home with the operator's tracked arm.
    SwgrHome,
}

impl Variant {
    const fn posture(self) -> Posture {
        match self {
            Self::Clutched | Self::Swgr | Self::SwgrAeac => Posture::Commissioned,
            Self::ClutchedNoPosture | Self::SwgrAeacNoPosture => Posture::Engage,
            Self::ClutchedHome | Self::SwgrEstimated | Self::SwgrHome => Posture::Body,
        }
    }
}

/// The page's estimated elbow (`arm.ts` `elbowOf`, 2026-11-14), in the robot's frame (x
/// forward, y left, z up) with the operator's body facing +x: a two-bone arm (0.30 / 0.27 m,
/// never straighter than 0.995 of its length) whose elbow swivels by a direction field over
/// where the hand points, singular only straight back from the shoulder, with 0.1 of the last
/// frame's direction kept. `side` is +1 left, -1 right; `memory` the last direction (zeros at
/// first), updated.
fn estimated_elbow(
    side: f64,
    shoulder: [f64; 3],
    wrist: [f64; 3],
    memory: &mut [f64; 3],
) -> [f64; 3] {
    let (upper, fore) = (0.30, 0.27);
    let dot = |a: [f64; 3], b: [f64; 3]| (0..3).map(|k| a[k] * b[k]).sum::<f64>();
    let unit = |v: [f64; 3]| v.map(|x| x / norm(v).max(1e-12));
    let axis = sub(wrist, shoulder);
    let d = norm(axis);
    if d < 1e-4 {
        return [shoulder[0], shoulder[1], shoulder[2] - upper];
    }
    let n = axis.map(|x| x / d);
    let k = (d / (0.995 * (upper + fore))).max(1.0);
    let (l1, l2) = (upper * k, fore * k);
    let along = ((l1 * l1 - l2 * l2 + d * d) / (2.0 * d)).clamp(-l1, l1);
    let radius = (l1 * l1 - along * along).max(0.0).sqrt();
    let (forward, out, up) = ([1.0, 0.0, 0.0], [0.0, side, 0.0], [0.0, 0.0, 1.0]);
    let q = unit([0, 1, 2].map(|i| -forward[i] * 0.9 - out[i] * 0.4 + up[i] * 0.15));
    let e = [0, 1, 2].map(|i| out[i] * 0.3 - up[i]);
    let eq = dot(e, q);
    let e = unit([0, 1, 2].map(|i| e[i] - q[i] * eq));
    let (nq, ne, mn) = (dot(n, q), dot(n, e), dot(*memory, n));
    let v =
        [0, 1, 2].map(|i| (1.0 - nq) * e[i] + ne * (q[i] - n[i]) + 0.1 * (memory[i] - n[i] * mn));
    *memory = unit(v);
    [0, 1, 2].map(|i| shoulder[i] + n[i] * along + memory[i] * radius)
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
    /// The arms' joints' jerk, RMS over joints and ticks, and its magnitude's median and 95th
    /// percentile, rad/s^3.
    jerk_rms: f64,
    jerk_p50: f64,
    jerk_p95: f64,
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
    let config = station_config_with(variant.posture());
    let mut memory = [[0.0; 3]; 2];
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
                Variant::Clutched | Variant::ClutchedNoPosture | Variant::ClutchedHome => {
                    // The same hand as the arm would place, given as the clutch gives a hand:
                    // only the null space differs.
                    let mut pose = tool;
                    pose.translation_m = [0, 1, 2].map(|k| s[k] + wrist[k] * reach / HUMAN_REACH_M);
                    targets.tcp[hand] = Some(pose);
                }
                Variant::Swgr
                | Variant::SwgrAeac
                | Variant::SwgrAeacNoPosture
                | Variant::SwgrEstimated
                | Variant::SwgrHome => {
                    let mut arm = arm;
                    if variant == Variant::Swgr {
                        arm.elbow_m = [0, 1, 2].map(|k| 0.5 * (s[k] + w[k]));
                    }
                    if variant == Variant::SwgrEstimated {
                        arm.elbow_m = estimated_elbow(*side, s, w, &mut memory[hand]);
                    }
                    // The same hand as every variant (the clutch places it, not the arm).
                    let mut pose = tool;
                    pose.translation_m = [0, 1, 2].map(|k| s[k] + wrist[k] * reach / HUMAN_REACH_M);
                    targets.tcp[hand] = Some(pose);
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
    let mut magnitudes: Vec<f64> = jerks.iter().map(|j| j.abs()).collect();
    magnitudes.sort_by(f64::total_cmp);
    let pick =
        |p: f64| magnitudes[((p * magnitudes.len() as f64) as usize).min(magnitudes.len() - 1)];
    Outcome {
        jerk_p50: pick(0.5),
        jerk_p95: pick(0.95),
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
    // Both unwind: the operator's arm, and without it the hanging prior (`ELBOW_PRIOR_WEIGHT`;
    // before it, the clutched hand left this arm twisted, under half the time under 60).
    assert!(
        today.under_60 > 0.85 && arm.under_60 > 0.85,
        "{:.2} -> {:.2}",
        today.under_60,
        arm.under_60
    );
    // With no arm at all the body's own home unwinds it too, and the page's estimated arm
    // (no body tracking) hangs the elbow as well as the tracked one at the table.
    let home = run(Scenario::Table, Variant::ClutchedHome, 5.0, true);
    let estimated = run(Scenario::Table, Variant::SwgrEstimated, 5.0, false);
    assert!(
        home.under_60 > 0.85,
        "no arm, body home: {:.2}",
        home.under_60
    );
    assert!(
        estimated.under_60 > 0.6,
        "estimated arm: {:.2}",
        estimated.under_60
    );
}

/// The table: every scenario and variant, swivel bins, tracking and tick time.
#[test]
#[ignore = "a table: cargo test --release --test arm_retarget table_elbow_swivel -- --ignored --nocapture"]
fn table_elbow_swivel() {
    println!(
        "scenario variant | swivel share 0-30 30-60 60-90 90-120 120-150 150-180 | <60 | robot / operator mean deg | track rms / p95 mm | turn rms mrad | arm jerk rms p50 p95 rad/s3 | tick median us"
    );
    for (scenario, twisted) in [
        (Scenario::Reach, false),
        (Scenario::Sweep, false),
        (Scenario::Table, false),
        (Scenario::ElbowOut, false),
        (Scenario::Table, true),
    ] {
        for variant in [
            Variant::ClutchedNoPosture,
            Variant::Clutched,
            Variant::ClutchedHome,
            Variant::SwgrAeac,
            Variant::SwgrHome,
            Variant::SwgrEstimated,
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
                "{scenario} {variant:?} | {} | {:.0}% | {:.0} / {:.0} | {:.2} / {:.2} | {:.1} | {:.0} {:.0} {:.0} | {:.0}",
                shares.join(" "),
                100.0 * o.under_60,
                o.robot_mean_deg,
                o.operator_mean_deg,
                o.track_rms_mm,
                o.track_p95_mm,
                o.turn_rms_mrad,
                o.jerk_rms,
                o.jerk_p50,
                o.jerk_p95,
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
    let hand = Pose {
        translation_m: w,
        ..tool
    };
    let targets = Targets {
        tcp: hands([Some(hand), tools[1]]),
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

/// A real session's hands through the solve with no arm: the tool points the robot was
/// commanded (forward kinematics of each exported command, robot-bench `control_journal_lag.py
/// export`) given as the hands, from the session's own first command (8b3e2f2b's twisted left
/// arm), a gap over 50 ms taken as a release (the solve takes hold afresh where it stands).
/// Only the null space differs between postures: the posture at engage (the station before
/// 2026-11-14 on a robot with no commissioned Ready) against the body's own home. Per arm: the
/// share of ticks per 30 degrees of swivel from down, tracking, and arm jerk p50 / p95.
///
/// `SESSIONS=/tmp/shake1114/json/8b3e2f2b.json,/tmp/eve/json/41eb2d39_e40e9290.json`
#[test]
#[ignore = "a table over exports: SESSIONS=a.json,b.json cargo test --release --test arm_retarget table_session_replay -- --ignored --nocapture"]
fn table_session_replay() {
    let Ok(paths) = std::env::var("SESSIONS") else {
        println!("SESSIONS names no exports");
        return;
    };
    for path in paths.split(',') {
        let export: serde_json::Value =
            serde_json::from_slice(&std::fs::read(path).expect("export")).expect("json");
        let names: Vec<&str> = export["names"]
            .as_array()
            .expect("names")
            .iter()
            .map(|n| n.as_str().expect("name"))
            .collect();
        let commands: Vec<(u64, Vec<f64>)> = export["commands"]
            .as_array()
            .expect("commands")
            .iter()
            .map(|c| {
                (
                    c[0].as_u64().expect("t"),
                    c[1].as_array()
                        .expect("q")
                        .iter()
                        .map(|v| v.as_f64().expect("v"))
                        .collect(),
                )
            })
            .collect();
        for posture in [None, Some(Posture::Engage), Some(Posture::Body)] {
            let config = station_config_with(posture.unwrap_or(Posture::Body));
            let mut teleop = solve(&config);
            let body = teleop.ik().body().clone();
            let axes: Vec<String> = teleop
                .model()
                .axes()
                .iter()
                .map(|a| a.name.clone())
                .collect();
            let map = |q_j: &[f64], into: &mut Vec<f64>| {
                for (i, a) in axes.iter().enumerate() {
                    if let Some(j) = names.iter().position(|n| n == a) {
                        into[i] = q_j[j];
                    }
                }
            };
            let mut q = ready(&teleop);
            map(&commands[0].1, &mut q);
            let mut commanded = q.clone();
            let arm_axes: Vec<usize> = axes
                .iter()
                .enumerate()
                .filter(|(_, a)| {
                    (a.starts_with("left_") || a.starts_with("right_")) && !a.contains("gripper")
                })
                .map(|(i, _)| i)
                .collect();
            let links: Vec<[usize; 3]> = ["left", "right"]
                .iter()
                .map(|side| {
                    [
                        link(&body, &format!("{side}_shoulder_roll_link")),
                        link(&body, &format!("{side}_elbow_pitch_link")),
                        link(&body, &format!("{side}_wrist_pitch_link")),
                    ]
                })
                .collect();
            let mut bins = [[0usize; 6]; 2];
            let (mut errors, mut history, mut jerks) = (Vec::new(), Vec::new(), Vec::new());
            let mut last_t = commands[0].0;
            teleop.reset();
            for (t, q_j) in &commands {
                if t.saturating_sub(last_t) > 50_000_000 {
                    teleop.reset();
                    history.clear();
                }
                last_t = *t;
                map(q_j, &mut commanded);
                if posture.is_some() {
                    let tools = teleop.tool_poses(&commanded);
                    let targets = Targets {
                        tcp: hands([tools[0], tools[1]]),
                        ..Targets::default()
                    };
                    step_once(&mut teleop, &mut q, &targets);
                    let now = teleop.tool_poses(&q);
                    for hand in 0..2 {
                        if let (Some(a), Some(b)) = (now[hand], tools[hand]) {
                            errors.push(norm(sub(a.translation_m, b.translation_m)) * 1e3);
                        }
                    }
                } else {
                    q.clone_from(&commanded);
                }
                let frames = body.model().poses(&q);
                for (hand, [s, e, w]) in links.iter().enumerate() {
                    if let Some(angle) =
                        swivel(frames.links[*s].t, frames.links[*e].t, frames.links[*w].t)
                    {
                        bins[hand][((angle / 30.0) as usize).min(5)] += 1;
                    }
                }
                history.push(arm_axes.iter().map(|&i| q[i]).collect::<Vec<f64>>());
                if history.len() >= 4 {
                    let n = history.len();
                    for j in 0..arm_axes.len() {
                        let third = history[n - 1][j] - 3.0 * history[n - 2][j]
                            + 3.0 * history[n - 3][j]
                            - history[n - 4][j];
                        jerks.push((third / DT.powi(3)).abs());
                    }
                }
            }
            jerks.sort_by(f64::total_cmp);
            errors.sort_by(f64::total_cmp);
            let pick = |v: &[f64], p: f64| {
                if v.is_empty() {
                    f64::NAN
                } else {
                    v[((p * v.len() as f64) as usize).min(v.len() - 1)]
                }
            };
            let rms =
                (errors.iter().map(|x| x * x).sum::<f64>() / errors.len().max(1) as f64).sqrt();
            let label = posture.map_or("commanded (real)".to_owned(), |p| format!("{p:?}"));
            for (hand, side) in ["left", "right"].iter().enumerate() {
                let total: usize = bins[hand].iter().sum();
                let shares: Vec<String> = bins[hand]
                    .iter()
                    .map(|b| format!("{:3.0}", 100.0 * *b as f64 / total.max(1) as f64))
                    .collect();
                println!(
                    "{} {label} {side:5} | {} | <60 {:3.0}% | track rms {:.1} p95 {:.1} mm | jerk p50 {:.0} p95 {:.0}",
                    path.rsplit('/').next().unwrap_or(path),
                    shares.join(" "),
                    100.0 * (bins[hand][0] + bins[hand][1]) as f64 / total.max(1) as f64,
                    rms,
                    pick(&errors, 0.95),
                    pick(&jerks, 0.5),
                    pick(&jerks, 0.95),
                );
            }
        }
    }
}
