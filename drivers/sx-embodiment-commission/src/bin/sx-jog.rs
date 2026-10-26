//! `sx-jog`: move a body's joints by relative amounts, supervised, through the embodiment layer.
//!
//! ```text
//! sx-jog --drive <drive.json> --commissioning <commissioning.json>
//!        [--calibration <port>=<lerobot.json>]... [--allow-provisional <port>]...
//!        [--exclude <joint,...>] [--speed 0.2] [--log <ticks.jsonl>]
//!        (--joint <joint> --delta-deg <deg> [--hold-seconds <s>]
//!         | --sequence <joint:deg|pause:s,...>)
//! ```
//!
//! The body is opened as a station opens it (`sx_embodiment_bodies::open_with` on the bridges
//! the commissioning names), inside the `Governor`: each command slewed at `--speed` (at most
//! 0.5 rad/s) from where the body rests, a 0.2 rad tracking-error stop. A joint in the
//! commissioning's `out_of_service`, or in `--exclude` for this run, keeps its place in the
//! vector and is never addressed. Relative moves need no zero calibration. The body is stopped
//! at the end, on any refusal or fault, and on Ctrl-C. `--log` writes every tick as one JSON
//! line (the target asked for, the slewed command sent, what was observed, each motor's
//! position, velocity, torque, temperatures and state) from a writer thread; the control loop
//! only copies a fixed-size record into a preallocated queue.

use std::fs::File;
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use sx_embodiment_bodies::system::SystemPorts;
use sx_embodiment_bodies::{Body, Commissioning, OpenOptions, ServoCalibrations, open_with};
use sx_embodiment_commission::jog::{self, Step, TickLog};
use sx_embodiment_drivers::{ActuatorChain, AxisLimits, Governor, InstanceDrive};

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
    drive: String,
    commissioning: String,
    calibrations: Vec<(String, String)>,
    provisional: Vec<String>,
    exclude: Vec<String>,
    speed: Option<f64>,
    log: Option<String>,
    joint: Option<String>,
    delta_deg: Option<f64>,
    hold: f64,
    sequence: Option<String>,
}

fn parse() -> Result<Args, String> {
    let mut parsed = Args::default();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} needs a value"));
        match arg.as_str() {
            "--drive" => parsed.drive = value()?,
            "--commissioning" => parsed.commissioning = value()?,
            "--calibration" => {
                let entry = value()?;
                let (port, file) = entry
                    .split_once('=')
                    .ok_or("--calibration is <port>=<lerobot.json>")?;
                parsed.calibrations.push((port.to_owned(), file.to_owned()));
            }
            "--allow-provisional" => parsed.provisional.push(value()?),
            "--exclude" => parsed
                .exclude
                .extend(value()?.split(',').map(str::to_owned)),
            "--speed" => parsed.speed = Some(value()?.parse().map_err(|_| "bad --speed")?),
            "--log" => parsed.log = Some(value()?),
            "--joint" => parsed.joint = Some(value()?),
            "--delta-deg" => {
                parsed.delta_deg = Some(value()?.parse().map_err(|_| "bad --delta-deg")?);
            }
            "--hold-seconds" => parsed.hold = value()?.parse().map_err(|_| "bad --hold-seconds")?,
            "--sequence" => parsed.sequence = Some(value()?),
            other => return Err(format!("unknown argument {other}")),
        }
    }
    if parsed.drive.is_empty() || parsed.commissioning.is_empty() {
        return Err("--drive and --commissioning are required".into());
    }
    Ok(parsed)
}

fn read(path: &str) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|error| format!("{path}: {error}"))
}

/// The body the drive and commissioning name, opened as a station opens it, with this run's
/// `--exclude` joints out of service beside the commissioning's.
fn open_body(args: &Args) -> Result<Body, String> {
    let document = read(&args.drive)?;
    let instance = serde_json::from_str::<serde_json::Value>(&document)
        .map_err(|error| format!("{}: {error}", args.drive))?["instance"]
        .as_str()
        .ok_or("the drive names no instance")?
        .to_owned();
    let drive = InstanceDrive::parse(&document, &instance).map_err(|error| error.to_string())?;
    let mut commissioning = Commissioning::parse(&read(&args.commissioning)?, &instance)
        .map_err(|error| error.to_string())?;
    commissioning
        .out_of_service
        .extend(args.exclude.iter().cloned());
    let mut calibrations = ServoCalibrations::new();
    for (port, file) in &args.calibrations {
        calibrations = calibrations
            .with_lerobot(port, &read(file)?)
            .map_err(|error| error.to_string())?;
    }
    let provisional: Vec<&str> = args.provisional.iter().map(String::as_str).collect();
    let options = OpenOptions {
        allow_provisional: &provisional,
    };
    open_with(
        &drive,
        &commissioning,
        &calibrations,
        options,
        &mut SystemPorts::system(),
    )
    .map_err(|error| error.to_string())
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
    let body = open_body(&args)?;
    let joints: Vec<String> = body.axes().iter().map(|axis| axis.joint.clone()).collect();
    let steps = steps(&args, &joints)?;
    let width = body.axes().len();
    let mut governor = Governor::new(
        body,
        vec![
            AxisLimits {
                max_speed: speed,
                max_tracking_error: TRACKING,
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
        speed,
        PERIOD,
        &stop,
        log.as_ref(),
        &mut pace,
    )?;
    eprintln!(
        "stopped: unproven {:?} faults {:?} answered disable, enable state unobservable {:?}",
        ended.stop.unproven, ended.stop.faults, ended.stop.unobserved
    );
    if let Some(log) = log {
        let (written, dropped) = log.finish().map_err(|error| error.to_string())?;
        eprintln!("log: {written} ticks written, {dropped} dropped");
    }
    ended.failure.map_or(Ok(()), Err)
}
