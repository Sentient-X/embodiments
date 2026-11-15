//! A real session's operator stream through the station's teleoperation solve with a simulated
//! Spark follower in the loop: the solve reads the follower's (noisy) joints as the station reads
//! the arm's, so lag, ringing and friction feed back into the next step. Each solver variant
//! is scored on what the simulated arm and camera did (robot-bench `retarget_bench.py score`).
//!
//! ```text
//! cargo run --release -p sx-embodiment-ik --example retarget_bench -- \
//!     <config.json> <follower.json> <ticks.json> <variant> <out.json>
//! ```
//!
//! `config.json` may carry `vclip` (robot-bench `speed_limits.py config`): per Damiao joint, the
//! most planned velocity its MIT frame carries (`coast kp / kd`), where the follower's velocity
//! feed is clipped as the driver clips it. Without it the replay under-reads the DM4340 joints'
//! fast-motion error (41 against the journals' 58-72 mrad p50; with it, 55).
//!
//! `ticks.json` is robot-bench `teleop_replay.py ticks ... played` (the station's Trail
//! playback of the session's intents). `follower.json` is robot-bench
//! `references/spark-follower-model-2026-11-14.json`. `variant` is `+`-joined:
//!
//! - `base`: the solve as the station runs it on main;
//! - `lead`: each joint's follower lag (the model's `follower_lag_s`) given to the solve
//!   (`Teleop::set_follower_lag`): a moving hand is followed to where it will be;
//! - `boost=K,E0`: the command pushed past the solve's answer by an error-dependent share of
//!   command minus measured (PolicyIK's relative offset): `u = q + K e^2/(e^2+E0^2) e`, at
//!   most 0.05 rad;
//! - `euro=MIN,BETA,DCUT`: the operator's hands through that One-Euro filter (default
//!   `OneEuro::TOOL` 0.7, 80, 6); `raw`: no filter;
//! - `neckpose`: the head a pose limb, clutched like a hand: the camera follows the operator's
//!   head translation and rotation from the hold, the whole SO-101 chain solving it;
//! - `deadband=MM`: a hand target moving less than MM from its last kept value is not moved
//!   (dwell: a still hand stays still).

// A measurement tool run by hand (robot-bench `retarget_bench`), never shipped: its numeric
// casts and terse names are a script's, so clippy is off here.
#![allow(clippy::all, clippy::pedantic, clippy::nursery)]
#![allow(
    clippy::too_many_lines,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::many_single_char_names
)]

use serde::{Deserialize, Serialize};
use sx_embodiment_ik::smoothing::OneEuro;
use sx_embodiment_ik::teleop::{Clutch, MAX_HANDS, Targets, Teleop, TeleopConfig};
use sx_frames::{Pose, Quaternion};

#[derive(Deserialize)]
struct Tick {
    hands: Vec<Option<[f64; 7]>>,
    head: Option<[f64; 7]>,
}

#[derive(Deserialize)]
struct Ticks {
    axes: Vec<String>,
    q0: Vec<f64>,
    dt: f64,
    ticks: Vec<Tick>,
}

#[derive(Deserialize)]
struct Fit {
    delay_s: f64,
    omega_rad_s: f64,
    zeta: f64,
    viscous_per_s: f64,
    coulomb_rad_s2: f64,
    sag_rad: f64,
    follower_lag_s: f64,
    confidence: String,
}

#[derive(Deserialize)]
struct FollowerJoint {
    joint: String,
    fit: Option<Fit>,
}

#[derive(Deserialize)]
struct FollowerModel {
    joints: Vec<FollowerJoint>,
}

/// One joint's simulated follower: `q'' = w^2 (u(t-d) - q) + 2 z w (v(t-d) - q') - b q' - c
/// tanh(q'/0.02) + w^2 sag`, the fitted Spark model; `None`: follows its command exactly.
#[derive(Clone)]
struct Follower {
    omega: f64,
    zeta: f64,
    b: f64,
    c: f64,
    sag: f64,
    delay: usize,
    lag: f64,
}

#[derive(Serialize, Default)]
struct Out {
    axes: Vec<String>,
    limits: Vec<Option<[f64; 2]>>,
    posture: Vec<Option<f64>>,
    /// Per tick: commanded joints, measured (simulated) joints.
    cmd: Vec<Vec<f64>>,
    meas: Vec<Vec<f64>>,
    /// Per tick, per hand: the clutched target the solve was asked (None: not driven).
    target: Vec<Vec<Option<[f64; 7]>>>,
    /// Per tick, per hand: the measured tool pose.
    tool: Vec<Vec<Option<[f64; 7]>>>,
    /// Per tick: the measured camera pose, and the operator's head as received.
    camera: Vec<[f64; 7]>,
    head: Vec<Option<[f64; 7]>>,
    /// Per tick, per side: shoulder, elbow, wrist points (measured).
    arm: Vec<[[f64; 9]; 2]>,
    /// Per tick (every 5th, else NaN): the measured body's least clearance, metres.
    clearance: Vec<f64>,
    held: usize,
    solve_us: Vec<f64>,
    /// Per tick: every binding constraint the step reported, by name.
    active: Vec<Vec<String>>,
}

fn pose(p: [f64; 7]) -> Pose {
    Pose {
        translation_m: [p[0], p[1], p[2]],
        rotation: Quaternion {
            x: p[3],
            y: p[4],
            z: p[5],
            w: p[6],
        }
        .normalized(),
    }
}

fn flat(p: Pose) -> [f64; 7] {
    let t = p.translation_m;
    let r = p.rotation;
    [t[0], t[1], t[2], r.x, r.y, r.z, r.w]
}

/// A per-joint residual tracking policy (robot-bench `residual_policy.py`): a one-hidden-layer
/// tanh MLP from the joint's command-minus-reading history to a bounded offset.
#[derive(Deserialize)]
struct Policy {
    group: String,
    hidden: usize,
    max_push: f64,
    theta: Vec<f64>,
}

impl Policy {
    fn applies(&self, joint: &str) -> bool {
        match self.group.as_str() {
            "neck" => joint.starts_with("so101_"),
            _ => joint.starts_with("left_") || joint.starts_with("right_"),
        }
    }

    fn push(&self, x: [f64; 5]) -> f64 {
        let h = self.hidden;
        let (w1, rest) = self.theta.split_at(5 * h);
        let (b1, rest) = rest.split_at(h);
        let (w2, b2) = rest.split_at(h);
        let mut out = b2[0];
        for k in 0..h {
            let mut a = b1[k];
            for (i, xi) in x.iter().enumerate() {
                a += xi * w1[i * h + k];
            }
            out += a.tanh() * w2[k];
        }
        self.max_push * out.tanh()
    }
}

struct Variant {
    lead: bool,
    boost: Option<(f64, f64)>,
    /// Which joints the boost pushes: 0 every modelled joint, 1 the arms, 2 the neck.
    boost_group: u8,
    invert: Option<f64>,
    perturb: Option<(f64, f64, f64)>,
    policies: Vec<Policy>,
    neckpush: Option<(f64, f64, f64)>,
    elbowprior: Option<f64>,
    filter: Option<Option<OneEuro>>,
    neckpose: bool,
    deadband_m: Option<f64>,
    ideal: bool,
    nohead: bool,
    noshape: bool,
}

fn parse_variant(text: &str) -> Variant {
    let mut v = Variant {
        lead: false,
        boost: None,
        boost_group: 0,
        invert: None,
        perturb: None,
        policies: Vec::new(),
        neckpush: None,
        elbowprior: None,
        filter: None,
        neckpose: false,
        deadband_m: None,
        ideal: false,
        nohead: false,
        noshape: false,
    };
    for part in text.split('+') {
        let (name, args) = part.split_once('=').unwrap_or((part, ""));
        let nums: Vec<f64> = if name == "policy" {
            Vec::new()
        } else {
            args.split(',')
                .filter(|s| !s.is_empty())
                .map(|s| s.parse().expect("a number"))
                .collect()
        };
        match name {
            "base" => {}
            "lead" => v.lead = true,
            "boost" => v.boost = Some((nums[0], nums[1])),
            "boostarm" => {
                v.boost = Some((nums[0], nums[1]));
                v.boost_group = 1;
            }
            "boostneck" => {
                v.boost = Some((nums[0], nums[1]));
                v.boost_group = 2;
            }
            "invert" => v.invert = Some(nums[0]),
            "neckpush" => v.neckpush = Some((nums[0], nums[1], nums[2])),
            "elbowprior" => v.elbowprior = Some(nums[0]),
            "perturb" => v.perturb = Some((nums[0], nums[1], nums[2])),
            "policy" => {
                for name in args.split(',') {
                    // A bare name is `policy-<name>.json` in $POLICY_DIR (default the cwd), so
                    // a variant stays a file-name-safe word.
                    let path = if name.contains('/') {
                        name.to_string()
                    } else {
                        let dir = std::env::var("POLICY_DIR").unwrap_or_else(|_| ".".into());
                        format!("{dir}/policy-{name}.json")
                    };
                    v.policies.push(
                        serde_json::from_slice(&std::fs::read(path).expect("a policy"))
                            .expect("a policy json"),
                    );
                }
            }
            "raw" => v.filter = Some(None),
            "euro" => {
                v.filter = Some(Some(OneEuro {
                    min_cutoff_hz: nums[0],
                    beta_per_m_s: nums[1],
                    beta_per_rad_s: OneEuro::TOOL.beta_per_rad_s,
                    derivative_cutoff_hz: nums[2],
                }));
            }
            "neckpose" => v.neckpose = true,
            "ideal" => v.ideal = true,
            "nohead" => v.nohead = true,
            "noshape" => v.noshape = true,
            "deadband" => v.deadband_m = Some(nums[0] / 1000.0),
            other => panic!("unknown variant part {other}"),
        }
    }
    v
}

/// A deterministic Gaussian-ish reading noise (sum of uniforms), radians.
struct Noise(u64);
impl Noise {
    fn next(&mut self, sigma: f64) -> f64 {
        let mut s = 0.0;
        for _ in 0..4 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            s += (self.0 >> 11) as f64 / (1u64 << 53) as f64 - 0.5;
        }
        s * sigma * (3.0_f64).sqrt()
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [config, follower, ticks, variant, out] = &args[..] else {
        eprintln!(
            "usage: retarget_bench <config.json> <follower.json> <ticks.json> <variant> <out.json>"
        );
        std::process::exit(2);
    };
    let variant = parse_variant(variant);
    // The config may carry `vclip`: per joint, the most planned velocity the motor's frame
    // carries (sx-damiao-can `MAX_COAST kp / kd`, robot-bench `speed_limits.py config`). The
    // follower's velocity feed is clipped there, as `DamiaoChain::actuator_feedforward` sends it.
    let mut raw: serde_json::Value =
        serde_json::from_slice(&std::fs::read(config).expect("config")).expect("config json");
    let vclip: std::collections::BTreeMap<String, f64> = raw
        .as_object_mut()
        .and_then(|o| o.remove("vclip"))
        .map(|v| serde_json::from_value(v).expect("vclip: joint to rad/s"))
        .unwrap_or_default();
    let mut config: TeleopConfig = serde_json::from_value(raw).expect("a teleop config");
    let model: FollowerModel =
        serde_json::from_slice(&std::fs::read(follower).expect("follower")).expect("model");
    let ticks: Ticks =
        serde_json::from_slice(&std::fs::read(ticks).expect("ticks")).expect("ticks json");
    let dt = ticks.dt;
    if variant.neckpose {
        for limb in &mut config.limbs {
            if limb.look_forward.is_some() {
                limb.look_forward = None;
            }
        }
    }
    let files = sx_embodiment_ik::registered("sentient-static-upper-body").expect("its pack");
    // The limbs as the station serves them (`BodyFiles::limbs`: each hand's arm links), so the
    // elbow aim can engage; the fixture's limbs name none.
    let served = files.limbs().expect("limbs");
    for limb in &mut config.limbs {
        if let Some(own) = served.iter().find(|l| l.name == limb.name) {
            limb.arm.clone_from(&own.arm);
        }
    }
    let mut solve = Teleop::new(files.urdf, files.collision, &config).expect("solve");
    solve.set_deadline(std::time::Duration::from_secs(1));
    if let Some(filter) = variant.filter {
        solve.set_input_filter(filter);
    }
    if variant.noshape {
        solve.set_target_shaping(false);
    }
    let axes = solve.model().axes().to_vec();
    let names: Vec<String> = axes.iter().map(|a| a.name.clone()).collect();
    let links: Vec<String> = solve.model().links().to_vec();
    let width = names.len();
    let substeps = 10_usize;
    let h = dt / substeps as f64;
    let followers: Vec<Option<Follower>> = names
        .iter()
        .map(|name| {
            if variant.ideal {
                return None;
            }
            let fit = model
                .joints
                .iter()
                .find(|j| &j.joint == name)
                .and_then(|j| j.fit.as_ref())?;
            let mut f = Follower {
                omega: fit.omega_rad_s,
                zeta: fit.zeta,
                b: fit.viscous_per_s,
                c: fit.coulomb_rad_s2,
                sag: fit.sag_rad,
                delay: (fit.delay_s / h).round() as usize,
                lag: fit.follower_lag_s,
            };
            if fit.confidence == "low" {
                // A fit the windows barely constrain: an 8 rad/s follower whose viscous drag
                // gives the measured ramp lag (b / w^2).
                f.omega = 8.0;
                f.zeta = 0.6;
                f.b = fit.follower_lag_s * 64.0;
                f.c = 0.0;
            }
            Some(f)
        })
        .collect();
    // The model the variants compensate with is the nominal one; the simulated arm may differ.
    let nominal = followers.clone();
    let followers: Vec<Option<Follower>> = match variant.perturb {
        None => followers,
        Some((w, friction, delay_ms)) => followers
            .into_iter()
            .map(|f| {
                f.map(|mut f| {
                    f.omega *= w;
                    f.b *= friction;
                    f.c *= friction;
                    f.delay += (delay_ms / 1000.0 / h).round() as usize;
                    f
                })
            })
            .collect(),
    };
    if variant.lead {
        for (name, f) in names.iter().zip(&nominal) {
            if let Some(f) = f {
                solve.set_follower_lag(name, f.lag);
            }
        }
    }
    let hands_of_head = solve
        .hands()
        .iter()
        .position(|h| h == sx_embodiment_ik::HEAD);
    let head_limb = solve
        .ik()
        .body()
        .limbs()
        .iter()
        .position(|l| *l == sx_embodiment_ik::HEAD)
        .expect("a head");
    let link = |n: &str| solve.model().link(n).expect(n);
    let arm_links: [[usize; 3]; 2] = ["left", "right"].map(|s| {
        [
            link(&format!("{s}_shoulder_roll_link")),
            link(&format!("{s}_elbow_pitch_link")),
            link(&format!("{s}_wrist_pitch_link")),
        ]
    });
    let to_ours = |given: &[f64]| -> Vec<f64> {
        names
            .iter()
            .map(|name| {
                ticks
                    .axes
                    .iter()
                    .position(|n| n == name)
                    .map_or(0.0, |i| given[i])
            })
            .collect()
    };
    let mut q_meas = to_ours(&ticks.q0);
    let mut v_meas = vec![0.0; width];
    let clip: Vec<f64> = names
        .iter()
        .map(|n| vclip.get(n).copied().unwrap_or(f64::INFINITY))
        .collect();
    let mut cmd = q_meas.clone();
    let mut qdot = vec![0.0; width];
    let longest = followers
        .iter()
        .flatten()
        .map(|f| f.delay)
        .max()
        .unwrap_or(0)
        + 1;
    // The command history, substep by substep, for each follower's dead time.
    let mut history: std::collections::VecDeque<(Vec<f64>, Vec<f64>)> =
        std::iter::repeat_n((cmd.clone(), qdot.clone()), longest).collect();
    let available = vec![true; width];
    let mut clutch = Clutch::default();
    let mut driving = false;
    let mut noise = Noise(7);
    let mut kept: [Option<[f64; 3]>; MAX_HANDS] = [None; MAX_HANDS];
    let mut head_hold: Option<([f64; 3], [f64; 3])> = None;
    let mut push_state = vec![0.0; width];
    let (mut e_prev, mut u_prev, mut v_prev) =
        (vec![0.0; width], vec![0.0; width], vec![0.0; width]);
    let mut result = Out {
        axes: names.clone(),
        limits: axes.iter().map(|a| a.limits).collect(),
        posture: names
            .iter()
            .map(|n| config.posture.get(n).copied())
            .collect(),
        ..Out::default()
    };
    for (index, tick) in ticks.ticks.iter().enumerate() {
        let reading: Vec<Option<f64>> = q_meas
            .iter()
            .map(|q| Some(q + noise.next(0.0005)))
            .collect();
        let read: Vec<f64> = reading.iter().map(|r| r.unwrap_or(0.0)).collect();
        let mut targets = Targets::default();
        for (hand, given) in tick.hands.iter().enumerate().take(2) {
            targets.tcp[hand] = given.map(pose);
        }
        let driven = targets.tcp.iter().any(Option::is_some);
        if let Some(weight) = variant.elbowprior {
            // No operator arm: an elbow hanging straight down, at `weight` of the aim's full
            // weight (`elbow_weight` of its radius off a 0.4 m shoulder-wrist line).
            let radius = sx_embodiment_ik::teleop::ELBOW_DEADBAND_M
                + weight * (config.human_reach_m - sx_embodiment_ik::teleop::ELBOW_DEADBAND_M);
            for hand in 0..2 {
                if targets.tcp[hand].is_some() && targets.arm[hand].is_none() {
                    targets.arm[hand] = Some(sx_embodiment_ik::teleop::OperatorArm {
                        shoulder_m: [0.0, 0.0, 0.0],
                        elbow_m: [0.2, 0.0, -radius],
                        wrist_m: [0.4, 0.0, 0.0],
                    });
                }
            }
        }
        if !variant.neckpose && !variant.nohead {
            targets.head = tick.head.map(pose);
        }
        if let Some(band) = variant.deadband_m {
            for hand in 0..2 {
                match targets.tcp[hand] {
                    Some(mut p) => {
                        let at = p.translation_m;
                        let keep = kept[hand].filter(|k| {
                            (0..3).map(|i| (k[i] - at[i]).powi(2)).sum::<f64>().sqrt() < band
                        });
                        match keep {
                            Some(k) => p.translation_m = k,
                            None => kept[hand] = Some(at),
                        }
                        targets.tcp[hand] = Some(p);
                    }
                    None => kept[hand] = None,
                }
            }
        }
        if driven && !driving {
            solve.reset();
            clutch.release();
            head_hold = None;
        }
        driving = driven;
        let start = std::time::Instant::now();
        let mut asked = [None; MAX_HANDS];
        if driven {
            let tools = solve.tool_poses(&read);
            clutch.apply(&tools, &mut targets);
            // The head as a pose: its translation from where the camera stood at the hold,
            // moved as the operator's head has moved since; its rotation the operator's own
            // (the camera looks where they look, level as they hold their head).
            if let (true, Some(slot), Some(given)) = (variant.neckpose, hands_of_head, tick.head) {
                let given = pose(given);
                let camera = tools[slot].expect("the camera's pose");
                let (from_camera, from_head) =
                    *head_hold.get_or_insert((camera.translation_m, given.translation_m));
                targets.tcp[slot] = Some(Pose {
                    translation_m: [0, 1, 2]
                        .map(|k| from_camera[k] + given.translation_m[k] - from_head[k]),
                    rotation: given.rotation,
                });
            }
            asked = targets.tcp;
            let step = solve
                .step(&cmd, &reading, &available, &targets, dt)
                .expect("a step");
            if matches!(step.solve, sx_embodiment_ik::Solve::Held) {
                result.held += 1;
            }
            let mut seen: Vec<String> = Vec::new();
            for limb in &step.limbs {
                for a in &limb.active {
                    let name = match a {
                        sx_embodiment_ik::Active::SelfCollision { a, b } => {
                            format!("pair:{}/{}", links[*a], links[*b])
                        }
                        sx_embodiment_ik::Active::Obstacle { link, .. } => {
                            format!("obstacle:{}", links[*link])
                        }
                        sx_embodiment_ik::Active::Speed { axis } => {
                            format!("speed:{}", names[*axis])
                        }
                        sx_embodiment_ik::Active::Accel { axis } => {
                            format!("accel:{}", names[*axis])
                        }
                        sx_embodiment_ik::Active::Limit { axis } => {
                            format!("limit:{}", names[*axis])
                        }
                        other => format!("{other:?}")
                            .split(' ')
                            .next()
                            .unwrap_or("other")
                            .to_string(),
                    };
                    if !seen.contains(&name) {
                        seen.push(name);
                    }
                }
            }
            result.active.push(seen);
            cmd.copy_from_slice(&step.q);
            qdot.copy_from_slice(&step.qdot);
        } else {
            result.active.push(Vec::new());
        }
        if !driven && solve.settling() {
            clutch.release();
            let step = solve
                .step(&cmd, &reading, &available, &Targets::default(), dt)
                .expect("a rest step");
            cmd.copy_from_slice(&step.q);
            qdot.copy_from_slice(&step.qdot);
        } else if !driven {
            qdot.fill(0.0);
        }
        result.solve_us.push(start.elapsed().as_secs_f64() * 1e6);
        // What reaches the motors: the solve's answer, or pushed past it by the error.
        let mut sent = cmd.clone();
        if let (Some(scale), true) = (variant.invert, driven) {
            // The fitted follower inverted: the offset its viscous and Coulomb drag and its sag
            // hold the joint behind a moving command, added so it lands on the command.
            for axis in 0..width {
                let Some(f) = &nominal[axis] else { continue };
                let w2 = f.omega * f.omega;
                let push =
                    scale * ((f.b * qdot[axis] + f.c * (qdot[axis] / 0.02).tanh()) / w2 - f.sag);
                sent[axis] += push.clamp(-0.05, 0.05);
                if let Some([lo, hi]) = axes[axis].limits {
                    sent[axis] = sent[axis].clamp(lo, hi);
                }
            }
        }
        if driven && !variant.policies.is_empty() {
            for axis in 0..width {
                if followers[axis].is_none() || names[axis].contains("finger") {
                    continue;
                }
                let Some(policy) = variant.policies.iter().find(|p| p.applies(&names[axis])) else {
                    continue;
                };
                let e = cmd[axis] - read[axis];
                let x = [
                    e / 0.02,
                    (e - e_prev[axis]) / 0.005,
                    qdot[axis],
                    (qdot[axis] - v_prev[axis]) / 0.05,
                    u_prev[axis] / policy.max_push,
                ];
                let u = policy.push(x);
                sent[axis] += u;
                if let Some([lo, hi]) = axes[axis].limits {
                    sent[axis] = sent[axis].clamp(lo, hi);
                }
                e_prev[axis] = e;
                u_prev[axis] = u;
            }
        } else {
            e_prev.fill(0.0);
            u_prev.fill(0.0);
        }
        v_prev.copy_from_slice(&qdot);
        // The station's neck push as it lands (fleet/station intent.rs `NeckPush`): slewed, on
        // the neck while the operator drives, decaying otherwise, folded into the held command.
        if let Some((gain, e0, slew)) = variant.neckpush {
            let settling = !driven && solve.settling();
            for axis in 0..width {
                if !names[axis].starts_with("so101_") {
                    continue;
                }
                let want = if driven {
                    let e = cmd[axis] - read[axis];
                    (gain * e * e / (e * e + e0 * e0) * e).clamp(-0.05, 0.05)
                } else {
                    0.0
                };
                if !driven && !settling {
                    // A hold: the pushed command is what stands.
                    cmd[axis] += push_state[axis];
                    push_state[axis] = 0.0;
                } else {
                    push_state[axis] += (want - push_state[axis]).clamp(-slew, slew);
                }
                sent[axis] = cmd[axis] + push_state[axis];
                if let Some([lo, hi]) = axes[axis].limits {
                    sent[axis] = sent[axis].clamp(lo, hi);
                }
            }
        }
        if let Some((gain, e0)) = variant.boost {
            for axis in 0..width {
                if followers[axis].is_none() || !driven {
                    continue;
                }
                let neck = names[axis].starts_with("so101_");
                if (variant.boost_group == 1 && neck) || (variant.boost_group == 2 && !neck) {
                    continue;
                }
                let e = cmd[axis] - read[axis];
                let push = (gain * e * e / (e * e + e0 * e0) * e).clamp(-0.05, 0.05);
                sent[axis] += push;
                if let Some([lo, hi]) = axes[axis].limits {
                    sent[axis] = sent[axis].clamp(lo, hi);
                }
            }
        }
        for _ in 0..substeps {
            history.push_back((sent.clone(), qdot.clone()));
            history.pop_front();
            for axis in 0..width {
                match &followers[axis] {
                    None => {
                        q_meas[axis] = sent[axis];
                        v_meas[axis] = qdot[axis];
                    }
                    Some(f) => {
                        let (u, vff) = &history[history.len() - 1 - f.delay];
                        let (q, v) = (q_meas[axis], v_meas[axis]);
                        let a = f.omega * f.omega * (u[axis] - q + f.sag)
                            + 2.0
                                * f.zeta
                                * f.omega
                                * (vff[axis].clamp(-clip[axis], clip[axis]) - v)
                            - f.b * v
                            - f.c * (v / 0.02).tanh();
                        v_meas[axis] = v + a * h;
                        q_meas[axis] = q + v_meas[axis] * h;
                        if let Some([lo, hi]) = axes[axis].limits {
                            if q_meas[axis] < lo || q_meas[axis] > hi {
                                q_meas[axis] = q_meas[axis].clamp(lo, hi);
                                v_meas[axis] = 0.0;
                            }
                        }
                    }
                }
            }
        }
        let frames = solve.model().poses(&q_meas);
        let tools = solve.tool_poses(&q_meas);
        result
            .tool
            .push(tools[..2].iter().map(|t| t.map(flat)).collect());
        result
            .target
            .push(asked[..2].iter().map(|t| t.map(flat)).collect());
        result
            .camera
            .push(flat(solve.ik().body().frame(&frames, head_limb).pose()));
        result.head.push(tick.head);
        result.arm.push(arm_links.map(|[s, e, w]| {
            let mut p = [0.0; 9];
            for (k, l) in [s, e, w].iter().enumerate() {
                p[3 * k..3 * k + 3].copy_from_slice(&frames.links[*l].t);
            }
            p
        }));
        result.clearance.push(if index % 5 == 0 {
            solve.ik().clearance(&q_meas).0
        } else {
            f64::NAN
        });
        result.cmd.push(sent);
        result.meas.push(q_meas.clone());
    }
    std::fs::write(out, serde_json::to_vec(&result).expect("json")).expect("write");
    eprintln!("{} ticks, {} held", result.cmd.len(), result.held);
}
