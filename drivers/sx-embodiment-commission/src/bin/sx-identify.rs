//! `sx-identify`: run one part of a dynamics identification plan on the static upper body's
//! arms and record what the motors measured, for `tools/identify_dynamics.py fit`.
//!
//! ```text
//! sx-identify --plan <plan.json> --part holds|left|right|sweep:<joint>|breakaway:<joint>
//!             --record <out.csv> --drive <instance-drive.json> --instance <id>
//!             --bus <identity> [--speed 0.25]
//!             [--with-torso] [--without <joint>]... [--feedforward urdf|<dynamics.json>]
//!             [--feedforward-ramp-s 2] [--supply-volts 48|<model>=<V>,*=<V>] [--supply-source <s>]
//!             [--move] [--seconds 30]
//! ```
//!
//! `--part latency` moves nothing: it polls every chain motor through the read-only observer
//! (one status refresh each, the chain's own phase) every 20 ms for `--seconds`, and writes to
//! `--record` each motor's round trip from the poll's start to its answer being taken (p50,
//! p90, p99, p99.9, max, µs), the polls it missed by reason, the poll durations, the frames the
//! adapter received that no poll took (answers that came after their poll gave up), and the
//! adapter's counters (lost to a full ring, refused or failed batches, stalls, recoveries).
//!
//! `jog:<joint>` (sweep plan) moves the joint from its staging pose 0.5 rad at 0.3 rad/s,
//! holds 2 s, and back, twice (`jog:<n>:+|-`, `hold:<n>:+|-`): the bench test of tracking.
//! With `--feedforward`, every MIT frame carries the planned velocity and the model torque
//! from `sx_embodiment_drivers::Dynamics` over the static upper body's description, gravity at
//! the measured position plus friction and inertia for the plan (`urdf` uses the description
//! alone; a file is a `dynamics` calibration, applied only to the description it names), each
//! axis' torque bounded at a quarter of its motor's peak, through the governor
//! (`ActuatorChain::command_with`), which slews the target and caps the planned velocity.
//!
//! `sweep:` and `breakaway:` read the friction sweep plan
//! (`tools/identify_dynamics.py sweep-plan`, `docs/dynamics/static-upper-body-friction-sweeps.json`):
//! the arms go to the ready pose, then the joint's staging pose, which keeps the swept
//! joint's whole sub-range clear of the body and the table; then each pass sweeps the joint
//! from `lower` to `upper` and back at every planned speed the cap allows
//! (`sweep:<pass>:<speed>:+|-`), or ramps it from the middle of its sub-range at
//! `breakaway_rate_rad_s` for `breakaway_travel_rad` each way (`break:<pass>:+|-`); back to ready,
//! then to rest. The torso and `left_wrist_roll_joint` are left out unless asked
//! (`--with-torso`; `--without` leaves out more).
//!
//! The plan (`tools/identify_dynamics.py plan`, runbook `docs/DYNAMICS-IDENTIFICATION.md`)
//! holds absolute joint positions, so the drive must carry the unit's measured zeros. Every
//! target is computed before the bus is opened; without `--move` the tool prints the run
//! (duration, segments, fastest joint) and checks every target against each axis' commandable
//! range, sending nothing. With `--move` the arms are opened through `DamiaoChain` (each
//! motor's CAN timeout written at open) inside the `Governor` (speed cap `--speed`, at most
//! 0.5 rad/s; tracking-error stop at 0.2 rad), commanded every 20 ms with feed-forward off
//! (plain PD holds and tracks; what is fitted is what the motors measure), and stopped
//! (motors disabled) at the end, on Ctrl-C, on any refusal or fault, and when any motor's
//! torque passes 80 % of its peak. Every tick is one CSV row: time, segment, and each axis'
//! commanded target, position (rad), velocity (rad/s) and torque (N·m) at the joint.
//!
//! Logging is deferred: each tick stores one fixed-size record into a buffer reserved for
//! the whole run before the first command, and the CSV is written after the stop. Beyond
//! the governor and torque guard, a suspected contact stops the run: after a segment's
//! first 0.3 s, a step in any axis' torque over 2 N·m, or in its tracking error over 0.03 rad,
//! from one tick to the next.
//!
//! Holds: from where the arms rest, each pose is approached once from 0.05 rad below every
//! joint and once from above, each held `settle_s + record_s`; the fitter averages the last
//! `record_s` of each and the two sides, cancelling Coulomb friction. Excitations: move to the
//! trajectory's start (the other arm to the plan's posture), hold 1 s, play it `repeats` times,
//! return. Both parts end where the
//! arms rested when the chain opened.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use serde_json::Value;
use sx_damiao_can::chain::{DamiaoAxis, DamiaoChain};
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_drivers::governor::{AxisLimits, Governor};
use sx_embodiment_drivers::{
    ActuatorChain, Dynamics, DynamicsCalibration, Feedforward, FeedforwardLimit, InstanceDrive,
    Scratch, UPRIGHT,
};

const PERIOD: Duration = Duration::from_millis(20);
const MAX_SPEED: f64 = 0.5;
const MAX_TRACKING_ERROR: f64 = 0.2;
/// The share of a motor's peak torque past which the run stops.
const TORQUE_GUARD: f64 = 0.8;
/// A torque step (N·m) or tracking-error step (rad) between ticks that suggests contact.
const CONTACT_TORQUE_STEP: f64 = 2.0;
const CONTACT_TRACKING_STEP: f64 = 0.03;
/// How long after a segment starts the contact checks wait (the target's own acceleration).
const CONTACT_SETTLE: Duration = Duration::from_millis(300);
/// The most axes one record holds: the static upper body's fifteen Damiao motors.
const MAX_AXES: usize = 16;
/// The bench jog: 0.5 rad at 0.3 rad/s, as the jogs of 2026-10-26.
const JOG_TRAVEL: f64 = 0.5;
const JOG_SPEED: f64 = 0.3;
/// The share of a motor's peak torque the feed-forward may send.
const FEEDFORWARD_SHARE: f64 = 0.25;
const URDF: &str =
    include_str!("../../../../assets/sentient_static_upper_body/sentient_static_upper_body.urdf");

/// The feed-forward: the description's dynamics, buffers sized once, and each chain axis'
/// index among the tree's joints.
struct Model {
    dynamics: Dynamics,
    scratch: Scratch,
    index: Vec<usize>,
    position: Vec<f64>,
    velocity: Vec<f64>,
    acceleration: Vec<f64>,
    measured: Vec<f64>,
    planned: Vec<f64>,
    held_plan: Vec<f64>,
    held_now: Vec<f64>,
    out: Vec<Feedforward>,
}

impl Model {
    fn new(source: &str, names: &[String]) -> Result<Self, String> {
        let mut dynamics = Dynamics::of(URDF, UPRIGHT).map_err(|e| e.to_string())?;
        if source != "urdf" {
            let document = std::fs::read_to_string(source).map_err(|e| format!("{source}: {e}"))?;
            let calibration = DynamicsCalibration::parse(&document).map_err(|e| e.to_string())?;
            dynamics = dynamics
                .calibrated(&calibration)
                .map_err(|e| e.to_string())?;
        }
        let joints = dynamics.tree().joints();
        let index = names
            .iter()
            .map(|n| {
                joints
                    .iter()
                    .position(|a| &a.name == n)
                    .ok_or(format!("{n} is not in the description"))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let width = joints.len();
        Ok(Self {
            scratch: dynamics.scratch(),
            dynamics,
            index,
            position: vec![0.0; width],
            velocity: vec![0.0; width],
            acceleration: vec![0.0; width],
            measured: vec![0.0; width],
            planned: vec![0.0; width],
            held_plan: vec![0.0; width],
            held_now: vec![0.0; width],
            out: vec![Feedforward::default(); names.len()],
        })
    }

    /// Gravity at `measured`, inertia and friction for the plan: `ID(plan) - g(plan) + g(now)`,
    /// the torque scaled by `share` (the ramp in).
    fn feedforward(
        &mut self,
        target: &[f64],
        speed: &[f64],
        measured: &[f64],
        share: f64,
    ) -> Result<&[Feedforward], String> {
        for (axis, &joint) in self.index.iter().enumerate() {
            self.position[joint] = target[axis];
            self.velocity[joint] = speed[axis];
            self.measured[joint] = measured[axis];
        }
        let error = |e: sx_embodiment_drivers::DynamicsError| e.to_string();
        self.dynamics
            .inverse_into(
                &self.position,
                &self.velocity,
                &self.acceleration,
                &mut self.scratch,
                &mut self.planned,
            )
            .map_err(error)?;
        self.dynamics
            .gravity_torque_into(&self.position, &mut self.scratch, &mut self.held_plan)
            .map_err(error)?;
        self.dynamics
            .gravity_torque_into(&self.measured, &mut self.scratch, &mut self.held_now)
            .map_err(error)?;
        for (axis, &joint) in self.index.iter().enumerate() {
            self.out[axis] = Feedforward {
                velocity: speed[axis],
                torque: share
                    * (self.planned[joint] - self.held_plan[joint] + self.held_now[joint]),
            };
        }
        Ok(&self.out)
    }
}
/// Left out of the chain unless asked: the torso motor is not on the bench chain and the left
/// wrist roll rests out of its range until its range fix lands.
const LEFT_OUT: [&str; 2] = ["torso_joint", "left_wrist_roll_joint"];

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("sx-identify: {error}");
            ExitCode::FAILURE
        }
    }
}

struct Args {
    plan: String,
    part: String,
    record: String,
    drive: String,
    instance: String,
    speed: f64,
    /// The bus's commissioned identity (`<ZLG serial>:<channel>`, a `SocketCAN` interface or
    /// its adapter's serial), opened through `SystemPorts` on whichever transport carries it.
    bus: String,
    torso: bool,
    without: Vec<String>,
    feedforward: Option<String>,
    /// The supply the motors run on, recorded with every run: one voltage, or per motor model
    /// (`damiao_dm8009p=48,*=24`), as given.
    supply: Option<String>,
    /// Seconds over which feed-forward torque ramps in from zero.
    ramp: f64,
    /// What feeds the motors (`boost-converter`, `battery`, ...), recorded with every run.
    source: Option<String>,
    motion: bool,
    /// How long `--part latency` polls.
    seconds: f64,
}

fn parse() -> Result<Args, String> {
    let mut args = std::env::args().skip(1);
    let mut parsed = Args {
        plan: String::new(),
        part: String::new(),
        record: String::new(),
        drive: String::new(),
        instance: String::new(),
        speed: 0.25,
        bus: String::new(),
        torso: false,
        without: Vec::new(),
        feedforward: None,
        supply: None,
        ramp: 2.0,
        source: None,
        motion: false,
        seconds: 30.0,
    };
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} needs a value"));
        match arg.as_str() {
            "--plan" => parsed.plan = value()?,
            "--part" => parsed.part = value()?,
            "--record" => parsed.record = value()?,
            "--drive" => parsed.drive = value()?,
            "--instance" => parsed.instance = value()?,
            "--speed" => parsed.speed = value()?.parse().map_err(|_| "bad --speed")?,
            "--bus" => parsed.bus = value()?,
            "--with-torso" => parsed.torso = true,
            "--without" => parsed.without.push(value()?),
            "--feedforward" => parsed.feedforward = Some(value()?),
            "--supply-source" => parsed.source = Some(value()?),
            "--feedforward-ramp-s" => {
                parsed.ramp = value()?.parse().map_err(|_| "bad --feedforward-ramp-s")?;
                if !parsed.ramp.is_finite() || parsed.ramp <= 0.0 {
                    return Err("--feedforward-ramp-s is positive".into());
                }
            }
            "--supply-volts" => {
                let text = value()?;
                supply_for(&text, "").map_err(|()| "bad --supply-volts")?;
                parsed.supply = Some(text);
            }
            "--move" => parsed.motion = true,
            "--seconds" => {
                parsed.seconds = value()?.parse().map_err(|_| "bad --seconds")?;
                if !(parsed.seconds.is_finite() && parsed.seconds > 0.0 && parsed.seconds <= 3600.0)
                {
                    return Err("--seconds is in (0, 3600]".into());
                }
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    let checking = matches!(parsed.part.as_str(), "check" | "latency");
    if (parsed.plan.is_empty() && !checking) || parsed.record.is_empty() || parsed.drive.is_empty()
    {
        return Err("--plan, --record, --drive and --instance are required".into());
    }
    let known = matches!(
        parsed.part.as_str(),
        "holds" | "left" | "right" | "check" | "latency"
    ) || parsed.part.starts_with("sweep:")
        || parsed.part.starts_with("breakaway:")
        || parsed.part.starts_with("jog:");
    if !known {
        return Err("--part is holds, left, right, sweep:<joint> or breakaway:<joint>".into());
    }
    if !(parsed.speed > 0.0 && parsed.speed <= MAX_SPEED) {
        return Err(format!("--speed is in (0, {MAX_SPEED}] rad/s"));
    }
    Ok(parsed)
}

/// The supply `text` gives a motor of `model`: a plain voltage for every motor, or
/// `model=volts` pairs with `*` for the rest.
fn supply_for(text: &str, model: &str) -> Result<Option<f64>, ()> {
    if let Ok(volts) = text.trim().parse::<f64>() {
        return Ok(Some(volts));
    }
    let mut fallback = None;
    for pair in text.split(',') {
        let (key, volts) = pair.split_once('=').ok_or(())?;
        let volts: f64 = volts.trim().parse().map_err(|_| ())?;
        match key.trim() {
            "*" => fallback = Some(volts),
            key if key == model => return Ok(Some(volts)),
            _ => {}
        }
    }
    Ok(fallback)
}

/// One tick's target and the segment it belongs to.
struct Tick {
    segment: String,
    target: Vec<f64>,
}

fn numbers(value: &Value) -> Result<Vec<f64>, String> {
    value
        .as_array()
        .ok_or("expected an array")?
        .iter()
        .map(|v| v.as_f64().ok_or_else(|| "expected a number".to_owned()))
        .collect()
}

/// Straight-line moves at `speed`, one target per tick, ending exactly at `to`.
fn travel(ticks: &mut Vec<Tick>, segment: &str, from: &[f64], to: &[f64], speed: f64) {
    let distance = from
        .iter()
        .zip(to)
        .map(|(a, b)| (b - a).abs())
        .fold(0.0, f64::max);
    let seconds = distance / speed;
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let steps = (seconds / PERIOD.as_secs_f64()).ceil().max(1.0) as usize;
    for step in 1..=steps {
        #[allow(clippy::cast_precision_loss)]
        let s = step as f64 / steps as f64;
        ticks.push(Tick {
            segment: segment.to_owned(),
            target: from.iter().zip(to).map(|(a, b)| a + (b - a) * s).collect(),
        });
    }
}

fn hold(ticks: &mut Vec<Tick>, segment: &str, at: &[f64], seconds: f64) {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let steps = (seconds / PERIOD.as_secs_f64()).ceil() as usize;
    for _ in 0..steps {
        ticks.push(Tick {
            segment: segment.to_owned(),
            target: at.to_vec(),
        });
    }
}

/// Every tick of a sweep, breakaway or jog part (see the header), from `rest` back to `rest`.
#[allow(clippy::too_many_lines)]
fn schedule_sweep(
    plan: &Value,
    part: &str,
    names: &[String],
    rest: &[f64],
    speed: f64,
) -> Result<Vec<Tick>, String> {
    let (kind, joint) = part.split_once(':').ok_or("part")?;
    let sweep = plan["sweeps"]
        .as_array()
        .ok_or("the plan has no sweeps; is it a sweep plan?")?
        .iter()
        .find(|s| s["joint"] == joint)
        .ok_or(format!("the plan does not sweep {joint}"))?;
    let index = names
        .iter()
        .position(|n| n == joint)
        .ok_or(format!("{joint} is not in the chain"))?;
    let pose_of = |entry: &Value| -> Vec<f64> {
        names
            .iter()
            .zip(rest)
            .map(|(name, at)| entry.get(name).and_then(Value::as_f64).unwrap_or(*at))
            .collect()
    };
    let ready = pose_of(&plan["ready"]);
    let staging = pose_of(&sweep["staging"]);
    let lower = sweep["lower"].as_f64().ok_or("lower")?;
    let upper = sweep["upper"].as_f64().ok_or("upper")?;
    let passes = plan["passes"].as_u64().ok_or("passes")?;
    let dwell = plan["dwell_s"].as_f64().ok_or("dwell_s")?;
    let at = |q: f64| -> Vec<f64> {
        let mut pose = staging.clone();
        pose[index] = q;
        pose
    };
    let mut ticks = Vec::new();
    travel(&mut ticks, "move", rest, &ready, speed);
    travel(&mut ticks, "move", &ready, &at(lower), speed);
    if kind == "sweep" {
        let speeds = numbers(&plan["speeds_rad_s"])?;
        for pass in 0..passes {
            for &v in speeds.iter().filter(|v| **v <= speed + 1e-9) {
                travel(
                    &mut ticks,
                    &format!("sweep:{pass}:{v}:+"),
                    &at(lower),
                    &at(upper),
                    v,
                );
                hold(&mut ticks, "move", &at(upper), dwell);
                travel(
                    &mut ticks,
                    &format!("sweep:{pass}:{v}:-"),
                    &at(upper),
                    &at(lower),
                    v,
                );
                hold(&mut ticks, "move", &at(lower), dwell);
            }
        }
        travel(&mut ticks, "move", &at(lower), &ready, speed);
    } else if kind == "breakaway" {
        let rate = plan["breakaway_rate_rad_s"]
            .as_f64()
            .ok_or("breakaway_rate_rad_s")?;
        let reach = plan["breakaway_travel_rad"]
            .as_f64()
            .ok_or("breakaway_travel_rad")?;
        let middle = f64::midpoint(lower, upper);
        travel(&mut ticks, "move", &at(lower), &at(middle), speed);
        for pass in 0..passes {
            for (sign, label) in [(1.0, "+"), (-1.0, "-")] {
                hold(&mut ticks, "move", &at(middle), 1.0);
                let end = middle + sign * reach;
                travel(
                    &mut ticks,
                    &format!("break:{pass}:{label}"),
                    &at(middle),
                    &at(end),
                    rate,
                );
                travel(&mut ticks, "move", &at(end), &at(middle), speed.min(0.1));
            }
        }
        travel(&mut ticks, "move", &at(middle), &ready, speed);
    } else if kind == "jog" {
        let from = (lower + 0.1).min(upper);
        let to = (from + JOG_TRAVEL).min(upper);
        travel(&mut ticks, "move", &at(lower), &at(from), speed);
        for rep in 0..2 {
            travel(
                &mut ticks,
                &format!("jog:{rep}:+"),
                &at(from),
                &at(to),
                JOG_SPEED.min(speed),
            );
            hold(&mut ticks, &format!("hold:{rep}:+"), &at(to), 2.0);
            travel(
                &mut ticks,
                &format!("jog:{rep}:-"),
                &at(to),
                &at(from),
                JOG_SPEED.min(speed),
            );
            hold(&mut ticks, &format!("hold:{rep}:-"), &at(from), 2.0);
        }
        travel(&mut ticks, "move", &at(from), &ready, speed);
    } else {
        return Err(format!("unknown part {part}"));
    }
    travel(&mut ticks, "move", &ready, rest, speed);
    Ok(ticks)
}

/// Every tick of the part, from `rest` back to `rest`, over the chain's `names`.
#[allow(clippy::too_many_lines)]
fn schedule(
    plan: &Value,
    part: &str,
    names: &[String],
    rest: &[f64],
    speed: f64,
) -> Result<Vec<Tick>, String> {
    if part.starts_with("sweep:") || part.starts_with("breakaway:") || part.starts_with("jog:") {
        return schedule_sweep(plan, part, names, rest, speed);
    }
    let mut ticks = Vec::new();
    let pose_of = |entry: &Value, base: &[f64]| -> Vec<f64> {
        names
            .iter()
            .zip(base)
            .map(|(name, at)| entry.get(name).and_then(Value::as_f64).unwrap_or(*at))
            .collect()
    };
    if part == "holds" {
        let holds = &plan["holds"];
        let approach = holds["approach_rad"].as_f64().ok_or("holds.approach_rad")?;
        let settle = holds["settle_s"].as_f64().ok_or("holds.settle_s")?;
        let record = holds["record_s"].as_f64().ok_or("holds.record_s")?;
        // A plan may route every pose through one pose (the arms hanging) that the straight
        // paths to its poses were checked from.
        let via = holds.get("via").map(|v| pose_of(v, rest));
        let mut at = rest.to_vec();
        for (index, pose) in holds["poses"]
            .as_array()
            .ok_or("holds.poses")?
            .iter()
            .enumerate()
        {
            let target = pose_of(pose, rest);
            if let Some(via) = &via {
                travel(&mut ticks, "move", &at, via, speed);
                at.clone_from(via);
            }
            for (side, sign) in [("below", -1.0), ("above", 1.0)] {
                let start: Vec<f64> = target.iter().map(|q| q + sign * approach).collect();
                travel(&mut ticks, "move", &at, &start, speed);
                travel(&mut ticks, "move", &start, &target, speed.min(0.1));
                hold(
                    &mut ticks,
                    &format!("pose{index}:{side}"),
                    &target,
                    settle + record,
                );
                at.clone_from(&target);
            }
        }
        if let Some(via) = &via {
            travel(&mut ticks, "move", &at, via, speed);
            at.clone_from(via);
        }
        travel(&mut ticks, "move", &at, rest, speed);
        return Ok(ticks);
    }
    let excitation = plan["excitations"]
        .as_array()
        .ok_or("excitations")?
        .iter()
        .find(|e| e["name"] == part)
        .ok_or(format!("the plan has no {part} excitation"))?;
    let joints: Vec<String> = excitation["joints"]
        .as_array()
        .ok_or("excitation joints")?
        .iter()
        .filter_map(|j| j.as_str().map(str::to_owned))
        .collect();
    let period = excitation["period_s"].as_f64().ok_or("period_s")?;
    let repeats = excitation["repeats"].as_f64().ok_or("repeats")?;
    let centre = numbers(&excitation["centre_rad"])?;
    let sine: Vec<Vec<f64>> = excitation["sine"]
        .as_array()
        .ok_or("sine")?
        .iter()
        .map(numbers)
        .collect::<Result<_, _>>()?;
    let cosine: Vec<Vec<f64>> = excitation["cosine"]
        .as_array()
        .ok_or("cosine")?
        .iter()
        .map(numbers)
        .collect::<Result<_, _>>()?;
    let w = std::f64::consts::TAU / period;
    // Every chain joint the excitation does not swing is held at the plan's posture.
    let posture = pose_of(&excitation["posture"], rest);
    let at = |t: f64| -> Vec<f64> {
        let mut target = posture.clone();
        for (j, joint) in joints.iter().enumerate() {
            let Some(index) = names.iter().position(|n| n == joint) else {
                continue;
            };
            let mut q = centre[j];
            for (k, (a, b)) in sine[j].iter().zip(&cosine[j]).enumerate() {
                #[allow(clippy::cast_precision_loss)]
                let harmonic = w * (k + 1) as f64;
                q += a / harmonic * (harmonic * t).sin() - b / harmonic * (harmonic * t).cos();
            }
            target[index] = q;
        }
        target
    };
    let start = at(0.0);
    travel(&mut ticks, "move", rest, &start, speed);
    hold(&mut ticks, "move", &start, 1.0);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let steps = (repeats * period / PERIOD.as_secs_f64()).round() as usize;
    for step in 0..steps {
        #[allow(clippy::cast_precision_loss)]
        let t = step as f64 * PERIOD.as_secs_f64();
        ticks.push(Tick {
            segment: "excite".into(),
            target: at(t),
        });
    }
    travel(&mut ticks, "move", &start, rest, speed);
    Ok(ticks)
}

/// Whether any axis' commanded motion changes direction (or starts or stops) at `index`.
fn turns(ticks: &[Tick], index: usize) -> bool {
    if index < 2 {
        return false;
    }
    let (a, b, c) = (
        &ticks[index - 2].target,
        &ticks[index - 1].target,
        &ticks[index].target,
    );
    (0..c.len()).any(|axis| {
        let before = b[axis] - a[axis];
        let now = c[axis] - b[axis];
        let moving = |step: f64| step.abs() > 1e-9;
        moving(before) != moving(now) || (moving(now) && before.signum() != now.signum())
    })
}

/// One tick as recorded: fixed size, so the run's buffer is reserved before the first command.
#[derive(Clone, Copy)]
struct Record {
    elapsed: Duration,
    tick: usize,
    position: [f64; MAX_AXES],
    velocity: [f64; MAX_AXES],
    torque: [f64; MAX_AXES],
}

/// The CSV, written after the run: time, segment, then every target, position, velocity and
/// torque.
fn write_records(
    out: &mut impl Write,
    supply: Option<&str>,
    source: Option<&str>,
    outcome: Option<&str>,
    names: &[String],
    ticks: &[Tick],
    records: &[Record],
) -> std::io::Result<()> {
    if let Some(volts) = supply {
        writeln!(out, "# supply_volts={volts}")?;
    }
    if let Some(source) = source {
        writeln!(out, "# supply_source={source}")?;
    }
    if let Some(outcome) = outcome {
        // A motor that went silent is named (Silent(<id>)), so dropouts can be counted per
        // motor across runs.
        writeln!(out, "# outcome={}", outcome.replace('\n', " "))?;
    }
    write!(out, "time_s,segment")?;
    for kind in ["target", "position", "velocity", "torque"] {
        for name in names {
            write!(out, ",{kind}:{name}")?;
        }
    }
    writeln!(out)?;
    let width = names.len();
    for record in records {
        let tick = &ticks[record.tick];
        write!(out, "{:.4},{}", record.elapsed.as_secs_f64(), tick.segment)?;
        for value in &tick.target {
            write!(out, ",{value:.5}")?;
        }
        for value in &record.position[..width] {
            write!(out, ",{value:.5}")?;
        }
        for value in &record.velocity[..width] {
            write!(out, ",{value:.4}")?;
        }
        for value in &record.torque[..width] {
            write!(out, ",{value:.4}")?;
        }
        writeln!(out)?;
    }
    Ok(())
}

/// `--part check`: read every chain motor and send nothing that changes one: its registers
/// (`MST_ID`, `CTRL_MODE`, `PMAX`, `VMAX`, `TMAX`, the under- and over-voltage fault thresholds,
/// the torque constant and gear ratio) through the read-only scan, then one status refresh each
/// through the read-only observer (fresh, faulted with its code, or missing). Written as JSON
/// to `--record`, with the supply. Refuses (exit 1) a silent motor, a fault, an over-voltage
/// threshold at or below the supply, or a limit row the chain would refuse.
#[allow(clippy::too_many_lines, clippy::needless_pass_by_value)]
fn check(args: &Args, axes: Vec<DamiaoAxis>) -> Result<(), String> {
    use sx_damiao_can::observe::DamiaoObserver;
    use sx_damiao_can::scan::scan;
    use sx_embodiment_drivers::{AxisReading, Observe};
    let ids: Vec<u16> = axes.iter().map(|a| a.axis.binding.bus_id).collect();
    let registers = scan(open_bus(args)?, ids.clone()).map_err(|e| e.to_string())?;
    let readings = {
        let port = open_bus(args)?;
        let mut observer = DamiaoObserver::open(port, axes.clone()).map_err(|e| e.to_string())?;
        observer.read_positions()
    };
    let mut problems: Vec<String> = Vec::new();
    let mut motors = Vec::new();
    for (axis, reading) in axes.iter().zip(&readings) {
        let id = axis.axis.binding.bus_id;
        let found = registers.motors.iter().find(|m| m.slave_id == id);
        let state = match reading {
            AxisReading::Fresh { position, .. } => format!("fresh at {position:+.3} rad"),
            AxisReading::Faulted(code) => {
                problems.push(format!("{} reports fault {code:#x}", axis.axis.joint));
                format!("fault {code:#x}")
            }
            AxisReading::Missing(reason) => {
                problems.push(format!("{} is missing ({reason:?})", axis.axis.joint));
                format!("missing {reason:?}")
            }
        };
        match found {
            None => problems.push(format!(
                "{} (id {id:#x}) did not answer its registers",
                axis.axis.joint
            )),
            Some(motor) => {
                let supply = args.supply.as_deref().and_then(|text| {
                    supply_for(text, axis.axis.binding.model.wire())
                        .ok()
                        .flatten()
                });
                if let (Some(supply), Some(over)) = (supply, motor.over_voltage)
                    && f64::from(over) <= supply
                {
                    problems.push(format!(
                        "{} faults over {over} V, at or below the {supply} V supply",
                        axis.axis.joint
                    ));
                }
                if let (Some(supply), Some(under)) = (supply, motor.under_voltage)
                    && f64::from(under) >= supply
                {
                    problems.push(format!(
                        "{} faults under {under} V, at or above the {supply} V supply",
                        axis.axis.joint
                    ));
                }
                let rating = axis.axis.binding.model.rating();
                if let (Some(supply), Some(false)) =
                    (supply, supply.and_then(|volts| rating.admits_supply(volts)))
                {
                    let [low, high] = rating.supply_volts.unwrap_or_default();
                    problems.push(format!(
                        "{}'s {} is rated {low}-{high} V, not the {supply} V supply",
                        axis.axis.joint,
                        axis.axis.binding.model.wire()
                    ));
                }
            }
        }
        eprintln!(
            "{:28} id {id:#04x} {state:24} {}",
            axis.axis.joint,
            found.map_or_else(
                || "registers silent".to_owned(),
                |m| format!(
                    "mode {:?} PMAX {:?} VMAX {:?} TMAX {:?} UV {:?} V OV {:?} V KT {:?} Gr {:?}",
                    m.control_mode,
                    m.p_max,
                    m.v_max,
                    m.t_max,
                    m.under_voltage,
                    m.over_voltage,
                    m.torque_constant,
                    m.gear_ratio
                )
            )
        );
        motors.push(serde_json::json!({
            "joint": axis.axis.joint,
            "id": id,
            "state": state,
            "control_mode": found.and_then(|m| m.control_mode),
            "p_max": found.and_then(|m| m.p_max),
            "v_max": found.and_then(|m| m.v_max),
            "t_max": found.and_then(|m| m.t_max),
            "under_voltage": found.and_then(|m| m.under_voltage),
            "over_voltage": found.and_then(|m| m.over_voltage),
            "torque_constant": found.and_then(|m| m.torque_constant),
            "gear_ratio": found.and_then(|m| m.gear_ratio),
        }));
    }
    let document = serde_json::json!({
        "supply_volts": args.supply,
        "supply_source": args.source,
        "motors": motors,
        "problems": problems,
    });
    std::fs::write(&args.record, format!("{document:#}\n")).map_err(|e| e.to_string())?;
    if problems.is_empty() {
        eprintln!("check passed: {} motors read, nothing written", axes.len());
        Ok(())
    } else {
        Err(format!("check failed: {}", problems.join("; ")))
    }
}

/// Put this thread under the host's real-time policy for a 20 ms loop (`sx_realtime`: Mach's
/// time constraint on macOS, `SCHED_FIFO` on Linux), so an overloaded host does not park the
/// tick for tens of milliseconds. Refusal is reported and the loop runs as before.
fn realtime_loop() {
    if let Err(refused) = sx_realtime::elevate(sx_realtime::Role::ControlLoop { period: PERIOD }) {
        eprintln!("sx-identify: the real-time policy was {refused}; ticks may run late");
    }
}

/// The bus `--bus` names, opened on whichever transport carries that identity (a ZLG channel
/// in userspace, a `SocketCAN` interface, a USB2CAN bridge), as a station opens a commissioned
/// port.
fn open_bus(args: &Args) -> Result<sx_embodiment_bodies::Port, String> {
    use sx_embodiment_bodies::PortSource;
    if args.bus.is_empty() {
        return Err(
            "--bus names the bus by its commissioned identity, e.g. A30F115DD0120A949B80:1 or can0"
                .into(),
        );
    }
    sx_embodiment_bodies::system::SystemPorts::system()
        .open(&args.bus)
        .map_err(|e| e.to_string())
}

/// `--part latency`: see the module docs. Sends only status refreshes and register reads.
#[allow(
    clippy::too_many_lines,
    clippy::needless_pass_by_value,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss
)]
fn latency(args: &Args, axes: Vec<DamiaoAxis>) -> Result<(), String> {
    use sx_damiao_can::observe::DamiaoObserver;
    use sx_embodiment_drivers::can::CanPort;
    use sx_embodiment_drivers::{AxisReading, MissingReason, Observe};
    let port = open_bus(args)?;
    let mut observer = DamiaoObserver::open(port, axes.clone()).map_err(|e| e.to_string())?;
    let polls = (args.seconds / PERIOD.as_secs_f64()).ceil() as usize;
    let count = axes.len();
    // Reserved before the first poll: the loop allocates nothing.
    let mut trips: Vec<Vec<u32>> = (0..count).map(|_| Vec::with_capacity(polls)).collect();
    let mut durations: Vec<u32> = Vec::with_capacity(polls);
    let mut missed = vec![[0_u64; 4]; count];
    let mut faulted = vec![0_u64; count];
    let mut events: Vec<(f64, usize, MissingReason, u32)> = Vec::with_capacity(1024);
    let link_before = observer.port().counters();
    realtime_loop();
    let began = Instant::now();
    let mut next = began;
    for _ in 0..polls {
        let start = Instant::now();
        let readings = observer.read_positions();
        let took = start.elapsed();
        durations.push(u32::try_from(took.as_micros()).unwrap_or(u32::MAX));
        for (index, reading) in readings.iter().enumerate() {
            match reading {
                AxisReading::Fresh { received, .. } => trips[index].push(
                    u32::try_from(received.saturating_duration_since(start).as_micros())
                        .unwrap_or(u32::MAX),
                ),
                AxisReading::Missing(reason) => {
                    let slot = match reason {
                        MissingReason::Timeout => 0,
                        MissingReason::Stale => 1,
                        MissingReason::Corrupt => 2,
                        MissingReason::Unavailable => 3,
                    };
                    missed[index][slot] += 1;
                    if events.len() < events.capacity() {
                        events.push((
                            start.duration_since(began).as_secs_f64(),
                            index,
                            *reason,
                            u32::try_from(took.as_micros()).unwrap_or(u32::MAX),
                        ));
                    }
                }
                AxisReading::Faulted(_) => faulted[index] += 1,
            }
        }
        next += PERIOD;
        match next.checked_duration_since(Instant::now()) {
            Some(wait) => thread::sleep(wait),
            None => next = Instant::now(),
        }
    }
    let elapsed = began.elapsed().as_secs_f64();
    let link_after = observer.port().counters();
    let quantiles = |values: &mut Vec<u32>| {
        values.sort_unstable();
        let at = |q: f64| {
            values
                .get(((values.len().saturating_sub(1)) as f64 * q).round() as usize)
                .copied()
        };
        serde_json::json!({
            "n": values.len(), "p50": at(0.5), "p90": at(0.9), "p99": at(0.99),
            "p999": at(0.999), "max": values.last().copied(),
        })
    };
    let mut answered = 0_u64;
    let mut motors = Vec::new();
    for (index, axis) in axes.iter().enumerate() {
        answered += trips[index].len() as u64;
        let summary = quantiles(&mut trips[index]);
        eprintln!(
            "{:28} id {:#04x} rtt us {summary} missed t/s/c/u {:?} faulted {}",
            axis.axis.joint, axis.axis.binding.bus_id, missed[index], faulted[index]
        );
        motors.push(serde_json::json!({
            "joint": axis.axis.joint,
            "id": axis.axis.binding.bus_id,
            "rtt_us": summary,
            "missed": {"timeout": missed[index][0], "stale": missed[index][1],
                       "corrupt": missed[index][2], "unavailable": missed[index][3]},
            "faulted": faulted[index],
        }));
    }
    let count_of = |counters: &[(&str, u64)], name: &str| {
        counters
            .iter()
            .find(|(counter, _)| *counter == name)
            .map_or(0, |(_, value)| *value)
    };
    let received =
        count_of(&link_after, "rx_frames").saturating_sub(count_of(&link_before, "rx_frames"));
    let named = |counters: &[(&'static str, u64)]| {
        counters
            .iter()
            .map(|(name, value)| ((*name).to_owned(), serde_json::Value::from(*value)))
            .collect::<serde_json::Map<_, _>>()
    };
    let poll_us = quantiles(&mut durations);
    eprintln!(
        "{polls} polls in {elapsed:.1} s; poll us {poll_us}; frames received {received}, taken by a poll {answered}, late or unclaimed {}",
        received.saturating_sub(answered)
    );
    let document = serde_json::json!({
        "polls": polls,
        "seconds": elapsed,
        "load_average": std::process::Command::new("sysctl").args(["-n", "vm.loadavg"]).output()
            .ok().map(|out| String::from_utf8_lossy(&out.stdout).trim().to_owned()),
        "poll_us": poll_us,
        "frames_received": received,
        "frames_taken_by_a_poll": answered,
        "late_or_unclaimed": received.saturating_sub(answered),
        "bus": args.bus,
        "link_before": named(&link_before),
        "link_after": named(&link_after),
        "motors": motors,
        "misses": events.iter().map(|(t, index, reason, took)| serde_json::json!({
            "t": t, "joint": axes[*index].axis.joint, "reason": format!("{reason:?}"), "poll_us": took,
        })).collect::<Vec<_>>(),
    });
    std::fs::write(&args.record, format!("{document:#}\n")).map_err(|e| e.to_string())?;
    Ok(())
}

#[allow(clippy::too_many_lines)]
fn run() -> Result<(), String> {
    let args = parse()?;
    let drive = InstanceDrive::parse(
        &std::fs::read_to_string(&args.drive).map_err(|e| format!("{}: {e}", args.drive))?,
        &args.instance,
    )
    .map_err(|e| e.to_string())?;
    let axes: Vec<DamiaoAxis> = STATIC_UPPER_BODY
        .axes(&drive)
        .map_err(|e| e.to_string())?
        .into_iter()
        .filter(|axis| {
            let joint = axis.axis.joint.as_str();
            !(args.without.iter().any(|w| w == joint)
                || (LEFT_OUT.contains(&joint) && !(args.torso && joint == "torso_joint")))
        })
        .collect();
    if args.part == "check" {
        return check(&args, axes);
    }
    if args.part == "latency" {
        return latency(&args, axes);
    }
    let plan: Value = serde_json::from_str(
        &std::fs::read_to_string(&args.plan).map_err(|e| format!("{}: {e}", args.plan))?,
    )
    .map_err(|e| e.to_string())?;
    if axes.len() > MAX_AXES {
        return Err(format!("at most {MAX_AXES} axes are recorded"));
    }
    let names: Vec<String> = axes.iter().map(|a| a.axis.joint.clone()).collect();
    let ranges: Vec<_> = axes.iter().map(DamiaoAxis::commandable).collect();
    // The dry run plans from the middle of each range; the real run from where the arms rest.
    let middle: Vec<f64> = ranges
        .iter()
        .map(|r| f64::midpoint(r.lower, r.upper))
        .collect();
    let dry = schedule(&plan, &args.part, &names, &middle, args.speed)?;
    for tick in &dry {
        for ((value, range), name) in tick.target.iter().zip(&ranges).zip(&names) {
            if !range.admits(*value) && tick.segment != "move" {
                return Err(format!(
                    "{}: {name} target {value:.3} rad is outside {:.3}..{:.3}",
                    tick.segment, range.lower, range.upper
                ));
            }
        }
    }
    #[allow(clippy::cast_precision_loss)]
    let seconds = dry.len() as f64 * PERIOD.as_secs_f64();
    eprintln!(
        "{}: {} ticks, about {:.0} s, {} recorded segments, over {}",
        args.part,
        dry.len(),
        seconds,
        dry.iter()
            .filter(|t| t.segment != "move")
            .map(|t| t.segment.as_str())
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        names.join(", ")
    );
    if let Some(source) = &args.feedforward {
        let mut model = Model::new(source, &names)?;
        let still = vec![0.0; names.len()];
        let torque: Vec<String> = model
            .feedforward(&middle, &still, &middle, 1.0)?
            .iter()
            .zip(&names)
            .map(|(ff, name)| format!("{name} {:+.2}", ff.torque))
            .collect();
        eprintln!(
            "feed-forward at mid-range, holding (N·m): {}",
            torque.join(", ")
        );
    }
    if !args.motion {
        eprintln!("without --move nothing is sent");
        return Ok(());
    }
    let port = open_bus(&args)?;
    let peaks: Vec<f64> = axes
        .iter()
        .map(|a| a.axis.binding.model.rating().peak_torque_nm)
        .collect();
    let gains: Vec<f64> = axes
        .iter()
        .map(|a| a.axis.actuator_from_joint(a.axis.zero + 1.0))
        .collect();
    let mut model = match &args.feedforward {
        Some(source) => Some(Model::new(source, &names)?),
        None => None,
    };
    let chain = DamiaoChain::open(port, axes).map_err(|e| e.to_string())?;
    let width = chain.axes().len();
    let mut governor = Governor::new(
        chain,
        vec![
            AxisLimits {
                max_speed: args.speed,
                max_tracking_error: MAX_TRACKING_ERROR,
                max_acceleration: None,
            };
            width
        ],
    )
    .map_err(|e| format!("{e:?}"))?;
    if model.is_some() {
        let limits: Vec<FeedforwardLimit> = peaks
            .iter()
            .map(|peak| FeedforwardLimit {
                velocity: Some(args.speed),
                torque: Some(FEEDFORWARD_SHARE * peak),
            })
            .collect();
        governor
            .set_feedforward_limits(&limits)
            .map_err(|e| format!("{e:?}"))?;
    }
    let rest = governor
        .chain()
        .last_observed()
        .ok_or("the chain reports no rest position")?;
    let ticks = schedule(&plan, &args.part, &names, &rest, args.speed)?;
    // Feed-forward frames bypass the governor's slew: every planned step must already be
    // within the cap.
    let step_cap = args.speed * PERIOD.as_secs_f64() * 1.01;
    if model.is_some() {
        let first_step = ticks.first().map_or(0.0, |t| {
            t.target
                .iter()
                .zip(&rest)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f64::max)
        });
        let fastest = ticks
            .windows(2)
            .map(|w| {
                w[0].target
                    .iter()
                    .zip(&w[1].target)
                    .map(|(a, b)| (a - b).abs())
                    .fold(0.0, f64::max)
            })
            .fold(first_step, f64::max);
        if fastest > step_cap {
            let _ = governor.stop();
            return Err(format!(
                "a planned step of {fastest:.4} rad exceeds the cap; nothing moved"
            ));
        }
    }
    let mut speeds = vec![0.0; width];
    for tick in &ticks {
        for ((value, range), name) in tick.target.iter().zip(&ranges).zip(&names) {
            if !range.admits(*value) {
                let _ = governor.stop();
                return Err(format!(
                    "{}: {name} target {value:.3} is outside its range from this rest; nothing moved",
                    tick.segment
                ));
            }
        }
    }
    let stop = Arc::new(AtomicBool::new(false));
    {
        let stop = Arc::clone(&stop);
        let _ = ctrlc::set_handler(move || stop.store(true, Ordering::SeqCst));
    }
    let mut out = BufWriter::new(File::create(&args.record).map_err(|e| e.to_string())?);
    let mut records: Vec<Record> = Vec::with_capacity(ticks.len());
    let begin = Instant::now();
    let mut next = Instant::now();
    let mut outcome = Ok(());
    let mut segment_start = Instant::now();
    let mut last: Option<Record> = None;
    // Where each axis last answered: the model's input, carried over a tick whose answer came
    // late (NaN in the receipt and the CSV; the governor tolerates one in a row per axis).
    let mut answered = [0.0; MAX_AXES];
    answered[..width].copy_from_slice(&rest);
    realtime_loop();
    for (index, tick) in ticks.iter().enumerate() {
        if stop.load(Ordering::SeqCst) {
            outcome = Err("stopped by the operator".to_owned());
            break;
        }
        // A new segment, or any axis' target reversing or starting or stopping, restarts the
        // contact checks' settle window: friction flips sign there and steps the torque by
        // twice its band, which is not contact (2026-10-26, right elbow, about 2 N·m).
        if index == 0 || ticks[index - 1].segment != tick.segment || turns(&ticks, index) {
            segment_start = Instant::now();
        }
        let commanded = match model.as_mut() {
            Some(model) => {
                let following = ticks.get(index + 1).map_or(&tick.target, |t| &t.target);
                for (axis, speed) in speeds.iter_mut().enumerate() {
                    *speed = (following[axis] - tick.target[axis]) / PERIOD.as_secs_f64();
                }
                let measured = answered;
                let share = (begin.elapsed().as_secs_f64() / args.ramp).min(1.0);
                match model.feedforward(&tick.target, &speeds, &measured[..width], share) {
                    Ok(feedforward) => governor
                        .command_with(&tick.target, feedforward)
                        .map_err(|e| format!("{e:?}")),
                    Err(error) => Err(error),
                }
            }
            None => governor.command(&tick.target).map_err(|e| format!("{e:?}")),
        };
        let receipt = match commanded {
            Ok(receipt) => receipt,
            Err(error) => {
                outcome = Err(format!(
                    "{}: command refused or failed: {error:?}",
                    tick.segment
                ));
                break;
            }
        };
        let mut record = Record {
            elapsed: begin.elapsed(),
            tick: index,
            position: [0.0; MAX_AXES],
            velocity: [0.0; MAX_AXES],
            torque: [0.0; MAX_AXES],
        };
        for (axis, (q, answer)) in receipt.observed.iter().zip(&receipt.feedback).enumerate() {
            record.position[axis] = *q;
            record.velocity[axis] = f64::from(answer.velocity) / gains[axis];
            record.torque[axis] = f64::from(answer.torque) * gains[axis];
        }
        records.push(record);
        for (at, q) in answered.iter_mut().zip(&record.position).take(width) {
            if q.is_finite() {
                *at = *q;
            }
        }
        if let Some(axis) = (0..width)
            .find(|&axis| (tick.target[axis] - record.position[axis]).abs() > MAX_TRACKING_ERROR)
        {
            outcome = Err(format!(
                "{}: {} is {:.3} rad from its target",
                tick.segment,
                names[axis],
                tick.target[axis] - record.position[axis]
            ));
            break;
        }
        let guard = (0..width).find(|&axis| record.torque[axis].abs() > TORQUE_GUARD * peaks[axis]);
        if let Some(axis) = guard {
            outcome = Err(format!(
                "{}: {} torque {:.1} N·m passed {TORQUE_GUARD} of its peak",
                tick.segment, names[axis], record.torque[axis]
            ));
            break;
        }
        if let Some(previous) = last.filter(|_| segment_start.elapsed() > CONTACT_SETTLE) {
            let before = &ticks[previous.tick].target;
            let contact = (0..width).find(|&axis| {
                let torque_step = (record.torque[axis] - previous.torque[axis]).abs();
                let error = tick.target[axis] - record.position[axis];
                let error_before = before[axis] - previous.position[axis];
                torque_step > CONTACT_TORQUE_STEP
                    || (error - error_before).abs() > CONTACT_TRACKING_STEP
            });
            if let Some(axis) = contact {
                outcome = Err(format!(
                    "{}: suspected contact on {}: torque {:.2} -> {:.2} N·m, position {:.3} rad",
                    tick.segment,
                    names[axis],
                    previous.torque[axis],
                    record.torque[axis],
                    record.position[axis]
                ));
                break;
            }
        }
        last = Some(record);
        next += PERIOD;
        if let Some(wait) = next.checked_duration_since(Instant::now()) {
            thread::sleep(wait);
        } else {
            next = Instant::now();
        }
    }
    let report = governor.stop();
    let written = write_records(
        &mut out,
        args.supply.as_deref(),
        args.source.as_deref(),
        outcome.as_ref().err().map(String::as_str),
        &names,
        &ticks,
        &records,
    )
    .and_then(|()| out.flush());
    if let Err(error) = written {
        eprintln!("sx-identify: the record could not be written: {error}");
    }
    eprintln!("recorded {} ticks to {}", records.len(), args.record);
    eprintln!(
        "stopped: unproven {:?} faults {:?} answered disable, enable state unobservable {:?}",
        report.unproven, report.faults, report.unobserved
    );
    outcome
}
