//! `sx-identify`: run one part of a dynamics identification plan on the static upper body's
//! arms and record what the motors measured, for `tools/identify_dynamics.py fit`.
//!
//! ```text
//! sx-identify --plan <plan.json> --part holds|left|right --record <out.csv>
//!             --drive <instance-drive.json> --instance <id>
//!             [--speed 0.25] [--zlg-serial <serial>] [--channel 1] [--with-torso] [--move]
//! ```
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
//! position (rad), velocity (rad/s) and torque (N·m) at the joint.
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
use sx_embodiment_drivers::{ActuatorChain, InstanceDrive};
use sx_zlg_usbcanfd::{ChannelOptions, UsbCanFd};

const PERIOD: Duration = Duration::from_millis(20);
const MAX_SPEED: f64 = 0.5;
const MAX_TRACKING_ERROR: f64 = 0.2;
/// The share of a motor's peak torque past which the run stops.
const TORQUE_GUARD: f64 = 0.8;

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
    serial: Option<String>,
    channel: u8,
    torso: bool,
    motion: bool,
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
        serial: None,
        channel: 1,
        torso: false,
        motion: false,
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
            "--zlg-serial" => parsed.serial = Some(value()?),
            "--channel" => parsed.channel = value()?.parse().map_err(|_| "bad --channel")?,
            "--with-torso" => parsed.torso = true,
            "--move" => parsed.motion = true,
            other => return Err(format!("unknown argument {other}")),
        }
    }
    if parsed.plan.is_empty() || parsed.record.is_empty() || parsed.drive.is_empty() {
        return Err("--plan, --record, --drive and --instance are required".into());
    }
    if !matches!(parsed.part.as_str(), "holds" | "left" | "right") {
        return Err("--part is holds, left or right".into());
    }
    if !(parsed.speed > 0.0 && parsed.speed <= MAX_SPEED) {
        return Err(format!("--speed is in (0, {MAX_SPEED}] rad/s"));
    }
    Ok(parsed)
}

/// Peak torque by model, N·m: what the guard is a share of.
fn peak(model: &str) -> f64 {
    match model {
        "damiao_dm8009" => 40.0,
        "damiao_dm4340" => 27.0,
        _ => 7.0,
    }
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

/// Every tick of the part, from `rest` back to `rest`, over the chain's `names`.
#[allow(clippy::too_many_lines)]
fn schedule(
    plan: &Value,
    part: &str,
    names: &[String],
    rest: &[f64],
    speed: f64,
) -> Result<Vec<Tick>, String> {
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
        let mut at = rest.to_vec();
        for (index, pose) in holds["poses"]
            .as_array()
            .ok_or("holds.poses")?
            .iter()
            .enumerate()
        {
            let target = pose_of(pose, rest);
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

/// One CSV row: time, segment, every position, then every velocity and torque `at` answers.
fn record(
    out: &mut impl Write,
    elapsed: Duration,
    tick: &Tick,
    observed: &[f64],
    mut at: impl FnMut(usize) -> (f64, f64),
) -> std::io::Result<()> {
    write!(out, "{:.4},{}", elapsed.as_secs_f64(), tick.segment)?;
    for q in observed {
        write!(out, ",{q:.5}")?;
    }
    let answers: Vec<(f64, f64)> = (0..observed.len()).map(&mut at).collect();
    for (velocity, _) in &answers {
        write!(out, ",{velocity:.4}")?;
    }
    for (_, torque) in &answers {
        write!(out, ",{torque:.4}")?;
    }
    writeln!(out)
}

#[allow(clippy::too_many_lines)]
fn run() -> Result<(), String> {
    let args = parse()?;
    let plan: Value = serde_json::from_str(
        &std::fs::read_to_string(&args.plan).map_err(|e| format!("{}: {e}", args.plan))?,
    )
    .map_err(|e| e.to_string())?;
    let drive = InstanceDrive::parse(
        &std::fs::read_to_string(&args.drive).map_err(|e| format!("{}: {e}", args.drive))?,
        &args.instance,
    )
    .map_err(|e| e.to_string())?;
    let axes: Vec<DamiaoAxis> = STATIC_UPPER_BODY
        .axes(&drive)
        .map_err(|e| e.to_string())?
        .into_iter()
        .filter(|axis| args.torso || axis.axis.joint != "torso_joint")
        .collect();
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
    if !args.motion {
        eprintln!("without --move nothing is sent");
        return Ok(());
    }
    let adapter = UsbCanFd::open(args.serial.as_deref()).map_err(|e| e.to_string())?;
    let port = adapter
        .open_channel(args.channel, ChannelOptions::default())
        .map_err(|e| e.to_string())?;
    let peaks: Vec<f64> = axes
        .iter()
        .map(|a| peak(a.axis.binding.model.wire()))
        .collect();
    let gains: Vec<f64> = axes
        .iter()
        .map(|a| a.axis.actuator_from_joint(a.axis.zero + 1.0))
        .collect();
    let chain = DamiaoChain::open(port, axes).map_err(|e| e.to_string())?;
    let width = chain.axes().len();
    let mut governor = Governor::new(
        chain,
        vec![
            AxisLimits {
                max_speed: args.speed,
                max_tracking_error: MAX_TRACKING_ERROR,
            };
            width
        ],
    )
    .map_err(|e| format!("{e:?}"))?;
    let rest = governor
        .chain()
        .last_observed()
        .ok_or("the chain reports no rest position")?;
    let ticks = schedule(&plan, &args.part, &names, &rest, args.speed)?;
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
    let header: Vec<String> = ["time_s".to_owned(), "segment".to_owned()]
        .into_iter()
        .chain(names.iter().map(|n| format!("position:{n}")))
        .chain(names.iter().map(|n| format!("velocity:{n}")))
        .chain(names.iter().map(|n| format!("torque:{n}")))
        .collect();
    writeln!(out, "{}", header.join(",")).map_err(|e| e.to_string())?;
    let begin = Instant::now();
    let mut next = Instant::now();
    let mut outcome = Ok(());
    for tick in &ticks {
        if stop.load(Ordering::SeqCst) {
            outcome = Err("stopped by the operator".to_owned());
            break;
        }
        let receipt = match governor.command(&tick.target) {
            Ok(receipt) => receipt,
            Err(error) => {
                outcome = Err(format!(
                    "{}: command refused or failed: {error:?}",
                    tick.segment
                ));
                break;
            }
        };
        let mut guarded = None;
        let written = record(
            &mut out,
            begin.elapsed(),
            tick,
            &receipt.observed,
            |index| {
                let answer = receipt.feedback[index];
                let torque = f64::from(answer.torque) * gains[index];
                if torque.abs() > TORQUE_GUARD * peaks[index] {
                    guarded = Some(format!(
                        "{} torque {torque:.1} N·m passed {TORQUE_GUARD} of its peak",
                        names[index]
                    ));
                }
                (f64::from(answer.velocity) / gains[index], torque)
            },
        );
        if let Err(error) = written {
            outcome = Err(format!("the record could not be written: {error}"));
            break;
        }
        if let Some(reason) = guarded {
            outcome = Err(reason);
            break;
        }
        next += PERIOD;
        if let Some(wait) = next.checked_duration_since(Instant::now()) {
            thread::sleep(wait);
        } else {
            next = Instant::now();
        }
    }
    let report = governor.stop();
    let _ = out.flush();
    eprintln!(
        "stopped: unproven {:?} faults {:?} answered disable, enable state unobservable {:?}",
        report.unproven, report.faults, report.unobserved
    );
    outcome
}
