//! `sx-jog`: move a body's joints by relative amounts, supervised, through the embodiment layer.
//!
//! ```text
//! sx-jog --drive <drive.json> --commissioning <commissioning.json>
//!        [--calibration <port>=<lerobot.json>]... [--allow-provisional <port>]...
//!        [--exclude <joint,...>] [--speed 0.2] [--log <ticks.jsonl>]
//!        [--at-end home|start|release]
//!        (--joint <joint> --delta-deg <deg> [--hold-seconds <s>]
//!         | --sequence <joint:deg|pause:s,...>)
//! ```
//!
//! The body is opened as a station opens it (`sx_embodiment_bodies::open_with` on the bridges
//! the commissioning names), inside the `Governor`: each command slewed at `--speed` (at most
//! 0.5 rad/s) from where the body rests, a 0.2 rad tracking-error stop. A joint in the
//! commissioning's `out_of_service`, or in `--exclude` for this run, keeps its place in the
//! vector and is never addressed. Relative moves need no zero calibration. When the moves are
//! done, `--at-end` decides (`sx_embodiment_drivers::AtEnd`): `home`, the default, returns every
//! joint under the governor to its registry home (where the calibrated zeros put it), then stops
//! the body; `start` returns to where the body rested when it was opened; `release` stops it
//! where it is. A return that cannot run or finish stops the body where it is, says why and
//! exits non-zero, as does a stop that is not proven. On any refusal or fault, and on Ctrl-C at
//! any time (during the moves or the return), the body is stopped where it is. `--log` writes every tick as one JSON
//! line (the target asked for, the slewed command sent, what was observed, each motor's
//! position, velocity, torque, temperatures and state) from a writer thread; the control loop
//! only copies a fixed-size record into a preallocated queue.

use std::fs::File;
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use sx_embodiment_commission::bench::BodyFlags;
use sx_embodiment_commission::jog::{self, Step, TickLog};
use sx_embodiment_drivers::{ActuatorChain, AtEnd, AxisLimits, Governor};

const PERIOD: Duration = Duration::from_millis(20);
const TRACKING: f64 = 0.2;
/// Ten seconds of ticks queued for the log writer.
const LOG_CAPACITY: usize = 500;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("sx-jog: {error}");
            ExitCode::FAILURE
        }
    }
}

#[derive(Default)]
struct Args {
    body: BodyFlags,
    speed: Option<f64>,
    log: Option<String>,
    joint: Option<String>,
    delta_deg: Option<f64>,
    hold: f64,
    sequence: Option<String>,
    at_end: AtEnd,
}

fn parse() -> Result<Args, String> {
    let mut parsed = Args::default();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} needs a value"));
        if parsed.body.take(&arg, &mut value)? {
            continue;
        }
        match arg.as_str() {
            "--speed" => parsed.speed = Some(value()?.parse().map_err(|_| "bad --speed")?),
            "--log" => parsed.log = Some(value()?),
            "--joint" => parsed.joint = Some(value()?),
            "--delta-deg" => {
                parsed.delta_deg = Some(value()?.parse().map_err(|_| "bad --delta-deg")?);
            }
            "--hold-seconds" => parsed.hold = value()?.parse().map_err(|_| "bad --hold-seconds")?,
            "--sequence" => parsed.sequence = Some(value()?),
            "--at-end" => parsed.at_end = value()?.parse()?,
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(parsed)
}

/// The run's steps: a sequence, or one jog and its hold.
fn steps(args: &Args, joints: &[String]) -> Result<Vec<Step>, String> {
    Ok(match (&args.sequence, &args.joint, args.delta_deg) {
        (Some(sequence), None, None) => jog::parse_sequence(sequence, joints)?,
        (None, Some(joint), Some(delta)) => {
            if delta.abs() > 45.0 {
                return Err("--delta-deg is at most 45 degrees per jog".into());
            }
            let mut steps = jog::parse_sequence(&format!("{joint}:{delta}"), joints)?;
            if args.hold > 0.0 {
                steps.push(Step::Pause(
                    Duration::try_from_secs_f64(args.hold).map_err(|_| "bad --hold-seconds")?,
                ));
            }
            steps
        }
        _ => return Err("give --sequence, or --joint with --delta-deg".into()),
    })
}

fn run() -> Result<(), String> {
    let args = parse()?;
    let speed = args.speed.unwrap_or(0.2);
    if !(speed > 0.0 && speed <= 0.5) {
        return Err("--speed is above 0 and at most 0.5 rad/s".into());
    }
    let body = args.body.open()?;
    let joints: Vec<String> = body.axes().iter().map(|axis| axis.joint.clone()).collect();
    let steps = steps(&args, &joints)?;
    let width = body.axes().len();
    let mut governor = Governor::new(
        body,
        vec![
            AxisLimits {
                max_speed: speed,
                max_tracking_error: TRACKING,
                max_acceleration: None,
            };
            width
        ],
    )
    .map_err(|error| error.to_string())?;
    let log = match &args.log {
        Some(path) => Some(TickLog::spawn(
            Box::new(File::create(path).map_err(|error| format!("{path}: {error}"))?),
            joints,
            LOG_CAPACITY,
        )),
        None => None,
    };
    let stop = Arc::new(AtomicBool::new(false));
    {
        let stop = Arc::clone(&stop);
        ctrlc::set_handler(move || stop.store(true, Ordering::SeqCst))
            .map_err(|error| error.to_string())?;
    }
    let mut next = Instant::now();
    let mut pace = || {
        next += PERIOD;
        match next.checked_duration_since(Instant::now()) {
            Some(wait) => std::thread::sleep(wait),
            None => next = Instant::now(),
        }
    };
    let ended = jog::run(
        &mut governor,
        &steps,
        PERIOD,
        &stop,
        log.as_ref(),
        args.at_end,
        &mut pace,
    )?;
    eprintln!("at end: {}", ended.ending);
    let stop = ended.ending.stop();
    eprintln!(
        "stopped: unproven {:?} faults {:?} answered disable, enable state unobservable {:?}",
        stop.unproven, stop.faults, stop.unobserved
    );
    if let Some(log) = log {
        let (written, dropped) = log.finish().map_err(|error| error.to_string())?;
        eprintln!("log: {written} ticks written, {dropped} dropped");
    }
    if let Some(failure) = ended.failure {
        return Err(failure);
    }
    if ended.ending.is_clean() {
        Ok(())
    } else {
        Err(format!(
            "the end of the run: {}; stop proven: {}",
            ended.ending,
            ended.ending.stop().is_proven()
        ))
    }
}
