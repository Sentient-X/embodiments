//! `sx-commission calibrate`: its arguments, and each mode run over whatever opens the bus.

use std::io;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::AtomicUsize;
use std::time::Duration;

use serde_json::{Value, json};
use sx_embodiment_drivers::serial::SerialPort;
use sx_feetech_serial::scan::{ReadOnlyPort, STS3215_BAUD};
use sx_feetech_serial::scservo_sdk::{
    ProtocolPacketHandler, SMS_STS_MAX_ANGLE_LIMIT_L, SMS_STS_MIN_ANGLE_LIMIT_L, SMS_STS_OFS_L,
};

use super::CalibrateError;
use super::journal::Journal;
use super::motion::{Closed, JogEvent, JogPlan, Sweep, Watch, go_to, jog, sweep_finger};
use super::range::{self, Prompt, RangePlan, StdinPrompt};
use super::read::{self, read_bus};
use super::session::{Session, Slow};

pub const USAGE: &str = "usage:
  sx-commission calibrate read --bus <tty> [--ids 1-20] [--json]
  sx-commission calibrate gripper --bus <tty> --ids 1,2 [--move] [--widen <ticks>]
                                  [--closed 1:low,2:high] [--json] [motion settings]
  sx-commission calibrate jog --bus <tty> --id <n> --delta <ticks> [--hold-seconds <s>]
                              [--widen <ticks>] [--move] [--json]
                              [motion settings]
  sx-commission calibrate goto --bus <tty> --id <n> --to <ticks> [--widen <ticks>] [--move]
  sx-commission calibrate range --bus <tty> --ids 1-6 [--joints a,b,..] [--full-turn 5]
                                [--out <file>]
  sx-commission calibrate apply --bus <tty> --calibration <file> --move
                                [--robot <robot dir> --port <port> | --publish --port <port>]
                                [--protection neck-defaults]
  sx-commission calibrate --restore <journal>

  --from-station --port <port> (any mode): the station running on this host lends the bus for
  the run, between sessions, and takes it back after; it never stops.

  Without --move, gripper and jog only read and print what they would do.
  range writes nothing: with every servo limp, a person shows each joint's middle (Enter),
  then moves each through its full travel (Enter ends); it prints and writes (--out) a LeRobot
  calibration keyed by joint. apply --move writes a file's homing offsets and limits, and
  with --robot/--port installs it as the robot's commissioned <port>.calibration.json, which
  the station and the bench tools load by default. With --publish/--port it goes to the
  station's calibration inbox instead, and the station publishes it to Fleet with its own key:
  the robot runs it within seconds, between sessions, with no other step.
  motion settings (defaults): --goal-speed 20 --acceleration 5 --torque-limit 250
    --load-threshold 150 --lead 20 --backoff 30 --stall-ms 600 --min-progress 3
    --max-travel 1500 --inset 10 --max-seconds 180 --baud 1000000 --journal-dir <dir>
    --trace <file> (one JSON line per motion step)";

/// Where an installed station takes calibrations measured on its robot and publishes them to
/// Fleet (`fleet/station`, `CalibrationInbox`).
pub const STATION_CALIBRATION_INBOX: &str = "/var/lib/sx-station/calibrations";

/// The largest nudge `jog` takes, ticks (about 35 degrees).
pub const MAX_JOG: i32 = 400;

/// What `calibrate` was asked.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Mode {
    Read,
    Gripper,
    Jog {
        id: u8,
        delta: i32,
    },
    /// Move one servo to a position and leave it there, limp.
    Goto {
        id: u8,
        to: u16,
    },
    /// Record each joint's middle and travel, shown by hand, as a calibration; writes nothing.
    Range,
    /// Write a measured calibration's homing offsets and position limits to its servos, kept
    /// across power cycles.
    Apply(PathBuf),
    Restore(PathBuf),
}

/// `calibrate`'s arguments.
#[derive(Clone, Debug)]
pub struct Args {
    pub mode: Mode,
    pub bus: Option<String>,
    pub baud: u32,
    pub ids: Vec<u8>,
    pub moves: bool,
    pub json: bool,
    pub widen: Option<u16>,
    pub closed: Vec<(u8, Closed)>,
    pub sweep: Sweep,
    pub inset: u16,
    pub max_seconds: u64,
    pub journal_dir: PathBuf,
    /// One JSON line per motion step, for a moving run.
    pub trace: Option<PathBuf>,
    /// How long `jog` holds at its target.
    pub hold: Duration,
    /// `range`: each id's joint name, the calibration's keys.
    pub joints: Vec<String>,
    /// `range`: joints that turn without end, given 0..4095.
    pub full_turn: Vec<u8>,
    /// `range`: where the calibration is written.
    pub out: Option<PathBuf>,
    /// `apply`: protection registers to set as well.
    pub protection: Option<Protection>,
    /// `apply`: the robot directory, and the port in it, whose commissioned calibration the
    /// applied file becomes.
    pub install: Option<(PathBuf, String)>,
    /// The port the station running on this host lends this run (`--from-station`), so the
    /// station keeps running while the bus is calibrated.
    pub borrow: Option<String>,
    /// `range`: redraw the live lines in place (standard output is a terminal).
    pub redraw: bool,
}

/// A servo's torque protection, as `Max_Torque_Limit` (16), `Protection_Current` (28) and
/// `Overload_Torque` (36), with `Torque_Limit` (48, RAM, loaded from 16 at power-on) set to
/// match.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Protection {
    pub max_torque: u16,
    pub protection_current: u16,
    pub overload_torque: u8,
}

/// The SO-101 neck's arm servos as found on the static upper body on 2026-10-26 (ids 1..5):
/// what a servo still holding the SO-101 gripper's 500 / 250 / 25 is reset to.
pub const NECK_DEFAULTS: Protection = Protection {
    max_torque: 1000,
    protection_current: 310,
    overload_torque: 80,
};

const MAX_TORQUE_LIMIT: u8 = 16;
const PROTECTION_CURRENT: u8 = 28;
const OVERLOAD_TORQUE: u8 = 36;

/// The settings a gripper sweep starts from: slow, weak and quick to give up.
#[must_use]
pub fn default_sweep() -> Sweep {
    Sweep {
        slow: Slow {
            goal_speed: 20,
            acceleration: 5,
            torque_limit: 250,
        },
        watch: Watch {
            load_threshold: 150,
            stall_window: Duration::from_millis(600),
            min_progress: 3,
            period: Duration::from_millis(20),
            backoff: 30,
            tolerance: 4,
            settle: Duration::from_millis(300),
        },
        lead: 20,
        max_travel: 1500,
        limit_margin: 3,
    }
}

fn ids(text: &str) -> Result<Vec<u8>, String> {
    let number = |part: &str| {
        part.trim()
            .parse::<u8>()
            .map_err(|_| format!("{part} is not a servo id"))
    };
    let mut found = Vec::new();
    for part in text.split(',') {
        if let Some((low, high)) = part.split_once('-') {
            found.extend(number(low)?..=number(high)?);
        } else {
            found.push(number(part)?);
        }
    }
    Ok(found)
}

fn closed(text: &str) -> Result<Vec<(u8, Closed)>, String> {
    text.split(',')
        .map(|part| {
            let (id, end) = part
                .split_once(':')
                .ok_or(format!("--closed takes id:low or id:high, not {part}"))?;
            let id = id
                .trim()
                .parse()
                .map_err(|_| format!("{id} is not an id"))?;
            let end = match end.trim() {
                "low" => Closed::Low,
                "high" => Closed::High,
                other => return Err(format!("{other} is neither low nor high")),
            };
            Ok((id, end))
        })
        .collect()
}

fn number<T: std::str::FromStr>(flag: &str, text: &str) -> Result<T, String> {
    text.parse()
        .map_err(|_| format!("{flag} takes a number, not {text}"))
}

/// Parse the words after `calibrate`.
///
/// # Errors
///
/// What is wrong with them.
#[allow(clippy::too_many_lines)] // one flag table and one mode table, read top to bottom
pub fn parse(words: &[String]) -> Result<Args, String> {
    let mut words = words.iter().map(String::as_str).peekable();
    let mode_word = match words.peek() {
        Some(word) if !word.starts_with("--") => words.next(),
        _ => None,
    };
    let mut args = Args {
        mode: Mode::Read,
        bus: None,
        baud: STS3215_BAUD,
        ids: Vec::new(),
        moves: false,
        json: false,
        widen: None,
        closed: Vec::new(),
        sweep: default_sweep(),
        inset: 10,
        max_seconds: 180,
        journal_dir: Journal::default_dir(),
        trace: None,
        hold: Duration::ZERO,
        joints: Vec::new(),
        full_turn: Vec::new(),
        out: None,
        protection: None,
        install: None,
        borrow: None,
        redraw: false,
    };
    let (mut id, mut delta, mut restore, mut to) = (None, None, None, None);
    let mut calibration = None;
    let (mut robot, mut port, mut publish, mut from_station) = (None, None, false, false);
    while let Some(flag) = words.next() {
        let mut value = || words.next().ok_or(format!("{flag} needs a value"));
        let sweep = &mut args.sweep;
        match flag {
            "--bus" => args.bus = Some(value()?.to_owned()),
            "--baud" => args.baud = number(flag, value()?)?,
            "--ids" => args.ids = ids(value()?)?,
            "--id" => id = Some(number::<u8>(flag, value()?)?),
            "--delta" => delta = Some(number::<i32>(flag, value()?)?),
            "--to" => to = Some(number::<u16>(flag, value()?)?),
            "--move" => args.moves = true,
            "--json" => args.json = true,
            "--widen" => args.widen = Some(number(flag, value()?)?),
            "--closed" => args.closed = closed(value()?)?,
            "--restore" => restore = Some(PathBuf::from(value()?)),
            "--calibration" => calibration = Some(PathBuf::from(value()?)),
            "--journal-dir" => args.journal_dir = PathBuf::from(value()?),
            "--trace" => args.trace = Some(PathBuf::from(value()?)),
            "--hold-seconds" => {
                args.hold =
                    Duration::from_secs_f64(number::<f64>(flag, value()?)?.clamp(0.0, 600.0));
            }
            "--goal-speed" => sweep.slow.goal_speed = number(flag, value()?)?,
            "--acceleration" => sweep.slow.acceleration = number(flag, value()?)?,
            "--torque-limit" => sweep.slow.torque_limit = number(flag, value()?)?,
            "--load-threshold" => sweep.watch.load_threshold = number(flag, value()?)?,
            "--lead" => sweep.lead = number(flag, value()?)?,
            "--backoff" => sweep.watch.backoff = number(flag, value()?)?,
            "--stall-ms" => {
                sweep.watch.stall_window = Duration::from_millis(number(flag, value()?)?);
            }
            "--min-progress" => sweep.watch.min_progress = number(flag, value()?)?,
            "--max-travel" => sweep.max_travel = number(flag, value()?)?,
            "--inset" => args.inset = number(flag, value()?)?,
            "--max-seconds" => args.max_seconds = number(flag, value()?)?,
            "--joints" => {
                args.joints = value()?
                    .split(',')
                    .map(|name| name.trim().to_owned())
                    .collect();
            }
            "--full-turn" => args.full_turn = ids(value()?)?,
            "--out" => args.out = Some(PathBuf::from(value()?)),
            "--robot" => robot = Some(PathBuf::from(value()?)),
            "--publish" => publish = true,
            "--from-station" => from_station = true,
            "--port" => port = Some(value()?.to_owned()),
            "--protection" => match value()? {
                "neck-defaults" => args.protection = Some(NECK_DEFAULTS),
                other => return Err(format!("--protection knows neck-defaults, not {other}")),
            },
            other => return Err(format!("unknown argument {other}")),
        }
    }
    args.mode = match (mode_word, restore) {
        (None, Some(path)) => Mode::Restore(path),
        (Some(_), Some(_)) => return Err("--restore is a mode of its own".to_owned()),
        (Some("read"), None) => Mode::Read,
        (Some("gripper"), None) => {
            if args.ids.is_empty() {
                return Err("gripper needs --ids".to_owned());
            }
            Mode::Gripper
        }
        (Some("jog"), None) => {
            let (Some(id), Some(delta)) = (id, delta) else {
                return Err("jog needs --id and --delta".to_owned());
            };
            if delta == 0 || delta.abs() > MAX_JOG {
                return Err(format!("--delta is 1 to {MAX_JOG} ticks either way"));
            }
            Mode::Jog { id, delta }
        }
        (Some("goto"), None) => {
            let (Some(id), Some(to)) = (id, to) else {
                return Err("goto needs --id and --to".to_owned());
            };
            if to > super::MAX_TICK {
                return Err(format!("--to is 0 to {}", super::MAX_TICK));
            }
            Mode::Goto { id, to }
        }
        (Some("range"), None) => {
            if args.ids.is_empty() {
                return Err("range needs --ids".to_owned());
            }
            if !args.joints.is_empty() && args.joints.len() != args.ids.len() {
                return Err(format!(
                    "--joints names {} joints for {} ids",
                    args.joints.len(),
                    args.ids.len()
                ));
            }
            if args.moves {
                return Err("range writes nothing; apply --move writes its file".to_owned());
            }
            Mode::Range
        }
        (Some("apply"), None) => {
            let Some(path) = calibration else {
                return Err("apply needs --calibration <file>".to_owned());
            };
            Mode::Apply(path)
        }
        (Some(other), None) => return Err(format!("unknown calibrate mode {other}")),
        (None, None) => return Err("calibrate needs a mode".to_owned()),
    };
    args.borrow = match (from_station, &port) {
        (true, Some(port)) => Some(port.clone()),
        (true, None) => return Err("--from-station needs --port".to_owned()),
        (false, _) => None,
    };
    args.install = match (robot, port, publish) {
        (Some(robot), Some(port), false) => Some((robot, port)),
        (None, Some(port), true) => Some((PathBuf::from(STATION_CALIBRATION_INBOX), port)),
        (None, None, false) => None,
        (None, Some(_), false) if from_station => {
            if matches!(args.mode, Mode::Apply(_)) {
                // The servos would hold a calibration the station does not: it could not
                // open the bus again when it takes it back.
                return Err("apply --from-station also needs --publish".to_owned());
            }
            None
        }
        (Some(_), _, true) => {
            return Err("--publish installs into the station's inbox; drop --robot".to_owned());
        }
        (None, None, true) => return Err("--publish needs --port".to_owned()),
        _ => return Err("--robot and --port go together".to_owned()),
    };
    if (args.protection.is_some() || args.install.is_some()) && !matches!(args.mode, Mode::Apply(_))
    {
        return Err("--protection, --robot and --port go with apply".to_owned());
    }
    if !matches!(args.mode, Mode::Restore(_)) && args.bus.is_none() {
        return Err("--bus <tty> is required".to_owned());
    }
    if args.max_seconds == 0 {
        return Err("--max-seconds must be above 0".to_owned());
    }
    Ok(args)
}

/// Where a run prints, and the exit code it reached.
pub struct Out<'a> {
    pub stdout: &'a mut dyn io::Write,
    pub stderr: &'a mut dyn io::Write,
    /// 0 done, 1 the run stopped or was refused but everything was undone, 3 something may
    /// still be powered or changed.
    pub code: u8,
}

/// Run `args`, opening the bus with `open`; `stop` carries a caught signal's number. Returns
/// the exit code.
pub fn run<P: SerialPort>(
    args: &Args,
    open: impl FnMut(&str, u32) -> io::Result<P>,
    stop: &Arc<AtomicUsize>,
    stdout: &mut dyn io::Write,
    stderr: &mut dyn io::Write,
) -> u8 {
    if matches!(args.mode, Mode::Range) {
        return run_with(args, open, stop, &mut StdinPrompt::new(), stdout, stderr);
    }
    run_with(args, open, stop, &mut NoPrompt, stdout, stderr)
}

/// The prompt of a mode that asks nothing.
struct NoPrompt;

impl Prompt for NoPrompt {
    fn wait(&mut self) -> Result<(), CalibrateError> {
        Err(CalibrateError::Refused("this mode asks nothing".to_owned()))
    }

    fn pressed(&mut self) -> bool {
        true
    }
}

/// [`run`] with the person's Enter presses read from `prompt`.
pub fn run_with<P: SerialPort>(
    args: &Args,
    mut open: impl FnMut(&str, u32) -> io::Result<P>,
    stop: &Arc<AtomicUsize>,
    prompt: &mut dyn Prompt,
    stdout: &mut dyn io::Write,
    stderr: &mut dyn io::Write,
) -> u8 {
    let mut out = Out {
        stdout,
        stderr,
        code: 0,
    };
    let result = match &args.mode {
        Mode::Restore(path) => restore(args, path, &mut open, stop, &mut out),
        Mode::Read => read_mode(args, &mut open, &mut out),
        Mode::Range => range_mode(args, &mut open, stop, prompt, &mut out),
        Mode::Apply(_) if !args.moves => Err(CalibrateError::Refused(
            "apply writes the servos' limits: pass --move".to_owned(),
        )),
        Mode::Gripper | Mode::Jog { .. } | Mode::Goto { .. } if !args.moves => {
            dry_run(args, &mut open, &mut out)
        }
        Mode::Gripper | Mode::Jog { .. } | Mode::Goto { .. } | Mode::Apply(_) => {
            moving(args, &mut open, stop, &mut out)
        }
    };
    if let Err(error) = result {
        let _ = writeln!(out.stderr, "{error}");
        out.code = out.code.max(1);
    }
    out.code
}

fn bus(args: &Args) -> &str {
    args.bus.as_deref().unwrap_or_default()
}

fn open_bus<P>(
    open: &mut impl FnMut(&str, u32) -> io::Result<P>,
    bus: &str,
    baud: u32,
) -> Result<P, CalibrateError> {
    open(bus, baud).map_err(|e| CalibrateError::Refused(format!("{bus}: {e}")))
}

fn read_mode<P: SerialPort>(
    args: &Args,
    open: &mut impl FnMut(&str, u32) -> io::Result<P>,
    out: &mut Out<'_>,
) -> Result<(), CalibrateError> {
    let mut port = open_bus(open, bus(args), args.baud)?;
    let ids = if args.ids.is_empty() {
        (1..=20).collect()
    } else {
        args.ids.clone()
    };
    let (found, readings) = read_bus(&mut port, ids);
    if args.json {
        let value = json!({
            "bus": bus(args),
            "servos": readings.iter().map(read::ServoReading::json).collect::<Vec<_>>(),
            "failed": found.failed.iter().map(|f| json!({
                "id": f.id, "address": f.address, "result": format!("{:?}", f.result),
            })).collect::<Vec<_>>(),
            "silent": found.silent,
        });
        let _ = writeln!(out.stdout, "{value:#}");
    } else {
        let _ = writeln!(out.stdout, "Feetech bus {} (read only):", bus(args));
        for reading in &readings {
            let _ = writeln!(out.stdout, "  {}", reading.line());
        }
        for failure in &found.failed {
            let _ = writeln!(
                out.stdout,
                "  id {:2}  answered its ping, then {:?} reading register {}",
                failure.id, failure.result, failure.address
            );
        }
    }
    if readings.is_empty() {
        let _ = writeln!(out.stderr, "no servo answered on {}", bus(args));
        out.code = 1;
    }
    Ok(())
}

/// The ids a moving mode touches.
fn targets(args: &Args) -> Vec<u8> {
    match args.mode {
        Mode::Jog { id, .. } | Mode::Goto { id, .. } => vec![id],
        _ => args.ids.clone(),
    }
}

fn dry_run<P: SerialPort>(
    args: &Args,
    open: &mut impl FnMut(&str, u32) -> io::Result<P>,
    out: &mut Out<'_>,
) -> Result<(), CalibrateError> {
    let port = open_bus(open, bus(args), args.baud)?;
    // The plan is read through the scan's read-only port: no write can leave the host.
    let mut ph = ProtocolPacketHandler::new(ReadOnlyPort::new(port), 0);
    let journal = Journal::path_for(&args.journal_dir, bus(args));
    if journal.exists() {
        let _ = writeln!(
            out.stdout,
            "warning: {} holds an earlier run's changes; --move refuses until \
             sx-commission calibrate --restore {} has undone them",
            journal.display(),
            journal.display()
        );
    }
    let s = &args.sweep;
    let _ = writeln!(
        out.stdout,
        "dry run on {} (nothing is written; pass --move): Goal_Speed {}, Acceleration {}, \
         Torque_Limit {}/1000, load threshold {}, at most {} s",
        bus(args),
        s.slow.goal_speed,
        s.slow.acceleration,
        s.slow.torque_limit,
        s.watch.load_threshold,
        args.max_seconds
    );
    let mut plan = Vec::new();
    for id in targets(args) {
        let limits = read::limits(&mut ph, id)?;
        let state = read::state(&mut ph, id)?;
        let control = read::control(&mut ph, id)?;
        let line = if let Mode::Goto { to, .. } = args.mode {
            let (min, max) = args.widen.map_or(limits, |w| {
                (
                    limits.0.saturating_sub(w),
                    limits.1.saturating_add(w).min(super::MAX_TICK),
                )
            });
            format!(
                "servo {id}: rests at {}, limits {}..{} (for the move {min}..{max}); would power \
                 it there, move it to {to}{} and leave it there with torque off",
                state.position,
                limits.0,
                limits.1,
                if (min..=max).contains(&to) {
                    ""
                } else {
                    " (REFUSED: outside its limits)"
                }
            )
        } else if let Mode::Jog { delta, .. } = args.mode {
            let target = state.position + delta;
            let inside = target >= i32::from(limits.0) && target <= i32::from(limits.1);
            format!(
                "servo {id}: rests at {}, limits {}..{}; would power it there, move to \
                     {target}{} and back, then turn torque off",
                state.position,
                limits.0,
                limits.1,
                if inside {
                    ""
                } else {
                    " (REFUSED: outside its limits)"
                }
            )
        } else {
            let widened = args.widen.map_or_else(String::new, |w| {
                format!(
                    ", widened for the sweep to {}..{} (Lock 1, restored after)",
                    limits.0.saturating_sub(w),
                    limits.1.saturating_add(w).min(super::MAX_TICK)
                )
            });
            format!(
                "servo {id}: rests at {}, limits {}..{}{widened}; would sweep it down \
                     and up {} ticks ahead of itself until it stalls, backing off {} ticks, \
                     then return it to {} and turn torque off",
                state.position, limits.0, limits.1, s.lead, s.watch.backoff, state.position
            )
        };
        let _ = writeln!(
            out.stdout,
            "  {line}\n    now: torque {} goal_speed {} acceleration {} torque_limit {} lock {} \
             load {}",
            control.torque_enable,
            control.goal_speed,
            control.acceleration,
            control.torque_limit,
            control.lock,
            state.load
        );
        plan.push(json!({ "id": id, "position": state.position, "limits": [limits.0, limits.1] }));
    }
    if args.json {
        let _ = writeln!(
            out.stdout,
            "{:#}",
            json!({ "bus": bus(args), "dry_run": true, "servos": plan })
        );
    }
    Ok(())
}

fn run_jog<P: SerialPort>(
    args: &Args,
    session: &mut Session<P>,
    id: u8,
    delta: i32,
    out: &mut Out<'_>,
) -> Result<Value, CalibrateError> {
    let plan = JogPlan {
        widen: args.widen,
        hold: args.hold,
    };
    let mut notify = |event: JogEvent| {
        match event {
            JogEvent::Holding { id, at } => {
                let _ = writeln!(out.stdout, "HOLDING id {id} at {at}");
            }
            JogEvent::Returned { id, at } => {
                let _ = writeln!(out.stdout, "RETURNED id {id} at {at}, torque off");
            }
        }
        let _ = out.stdout.flush();
    };
    let jogged = jog(
        session,
        id,
        delta,
        &args.sweep.slow,
        &args.sweep.watch,
        &plan,
        &mut notify,
    );
    jogged.map(|jog| {
        let speed = jog
            .out
            .ticks_per_second()
            .map_or_else(String::new, |v| format!(", {v:.0} ticks/s"));
        let _ = writeln!(
            out.stdout,
            "servo {id}: start {}, out to {} ({:?} after {:.2} s{speed}), back to {} \
             ({:?}), torque off",
            jog.start,
            jog.out.at,
            jog.out.end,
            jog.out.elapsed.as_secs_f64(),
            jog.back.at,
            jog.back.end
        );
        jog.json()
    })
}

fn moving<P: SerialPort>(
    args: &Args,
    open: &mut impl FnMut(&str, u32) -> io::Result<P>,
    stop: &Arc<AtomicUsize>,
    out: &mut Out<'_>,
) -> Result<(), CalibrateError> {
    let port = open_bus(open, bus(args), args.baud)?;
    let journal = Journal::create(&args.journal_dir, bus(args), args.baud)?;
    let _ = writeln!(
        out.stderr,
        "journal {}; SIGINT, SIGTERM and SIGHUP stop the run and undo it, SIGKILL cannot: \
         then run sx-commission calibrate --restore {}",
        journal.path().display(),
        journal.path().display()
    );
    let mut session = Session::new(
        port,
        journal,
        Duration::from_secs(args.max_seconds),
        Arc::clone(stop),
    );
    if let Some(path) = &args.trace {
        match std::fs::File::create(path) {
            Ok(file) => session = session.with_trace(Box::new(file)),
            Err(error) => {
                // Nothing is powered yet; the session closes on the way out.
                return Err(CalibrateError::Refused(format!(
                    "trace {}: {error}",
                    path.display()
                )));
            }
        }
    }
    let result = match args.mode {
        Mode::Jog { id, delta } => run_jog(args, &mut session, id, delta, out),
        Mode::Goto { id, to } => go_to(
            &mut session,
            id,
            to,
            &args.sweep.slow,
            &args.sweep.watch,
            args.widen,
        )
        .map(|arrival| {
            let _ = writeln!(
                out.stdout,
                "servo {id}: from {} to {} ({:?} after {:.2} s), left there with torque off",
                arrival.from,
                arrival.at,
                arrival.end,
                arrival.elapsed.as_secs_f64()
            );
            json!({ "id": id, "to": to, "from": arrival.from, "at": arrival.at,
                    "seconds": arrival.elapsed.as_secs_f64(),
                    "ticks_per_second": arrival.ticks_per_second() })
        }),
        Mode::Apply(ref path) => {
            apply(path, args.protection, &mut session, out).and_then(|value| match &args.install {
                Some((robot, port)) => install(path, robot, port, out).map(|()| value),
                None => Ok(value),
            })
        }
        _ => gripper(args, &mut session, out),
    };
    let report = session.close();
    let _ = write!(out.stderr, "{report}");
    if !report.is_clean() {
        out.code = 3;
    }
    let value = result?;
    if args.json {
        let _ = writeln!(out.stdout, "{value:#}");
    }
    Ok(())
}

fn gripper<P: SerialPort>(
    args: &Args,
    session: &mut Session<P>,
    out: &mut Out<'_>,
) -> Result<Value, CalibrateError> {
    let mut fingers = Vec::new();
    let mut calibration = serde_json::Map::new();
    for &id in &args.ids {
        let finger = sweep_finger(session, id, &args.sweep, args.widen)?;
        let closed = args
            .closed
            .iter()
            .find(|(closed_id, _)| *closed_id == id)
            .map(|(_, end)| *end);
        let _ = writeln!(
            out.stdout,
            "servo {id}: started at {}; low end {} ({:?}), high end {} ({:?}); returned to {}; \
             torque off",
            finger.start,
            finger.low.reached,
            finger.low.end,
            finger.high.reached,
            finger.high.end,
            finger.returned.at
        );
        match finger.calibration(closed, args.inset) {
            Some(c) => {
                let closing = match closed {
                    Some(Closed::Low) => "decreasing ticks close it",
                    Some(Closed::High) => "increasing ticks close it (drive_mode 1)",
                    None => "which end closes is unknown: pass --closed id:low|high",
                };
                let _ = writeln!(
                    out.stdout,
                    "  calibration: range {}..{} (inset {} from each stop), homing_offset {}; \
                     {closing}",
                    c.range_min, c.range_max, args.inset, c.homing_offset
                );
                calibration.insert(
                    id.to_string(),
                    json!({
                        "id": id,
                        "drive_mode": u8::from(c.drive_mode),
                        "homing_offset": c.homing_offset,
                        "range_min": c.range_min,
                        "range_max": c.range_max,
                    }),
                );
            }
            None => {
                let _ = writeln!(
                    out.stdout,
                    "  no calibration: an end was not a stop (widen the search with --widen \
                     or --max-travel)"
                );
            }
        }
        fingers.push(finger.json(closed, args.inset));
    }
    Ok(json!({
        "bus": bus(args),
        "fingers": fingers,
        "calibration": calibration,
        "note": "range_min/range_max are in the servo's present-position frame at its current \
                 homing_offset; nothing was written to keep them",
    }))
}

fn restore<P: SerialPort>(
    args: &Args,
    path: &std::path::Path,
    open: &mut impl FnMut(&str, u32) -> io::Result<P>,
    stop: &Arc<AtomicUsize>,
    out: &mut Out<'_>,
) -> Result<(), CalibrateError> {
    let journal = Journal::load(path)?;
    let record = journal.record().clone();
    let bus = args.bus.clone().unwrap_or(record.bus);
    let port = open_bus(open, &bus, record.baud)?;
    let _ = writeln!(
        out.stdout,
        "undoing {} on {bus}: servos {:?} powered, {} registers saved",
        path.display(),
        record.powered,
        record.saved.len()
    );
    // Undoing never checks the stop flag or the deadline.
    let session = Session::new(port, journal, Duration::from_secs(60), Arc::clone(stop));
    let report = session.close();
    let _ = write!(out.stdout, "{report}");
    if !report.is_clean() {
        out.code = 3;
    }
    Ok(())
}

/// One servo's entry in a calibration file.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Entry {
    id: u8,
    /// `Homing_Offset`, where the file carries one (a `LeRobot` calibration does; a `gripper`
    /// run's output's is the offset the servo already held).
    homing_offset: Option<i16>,
    range_min: u16,
    range_max: u16,
}

/// The entries of a calibration file: `sx-commission calibrate gripper --json`'s output, or a
/// `LeRobot` calibration keyed by joint (`calibrate range`'s), each entry carrying `id`,
/// `range_min`, `range_max` and optionally `homing_offset`.
fn entries_in(path: &std::path::Path) -> Result<Vec<Entry>, CalibrateError> {
    let refuse = |why: String| CalibrateError::Refused(format!("{}: {why}", path.display()));
    let text = std::fs::read_to_string(path).map_err(|e| refuse(e.to_string()))?;
    let value: serde_json::Value =
        serde_json::from_str(&text).map_err(|e| refuse(e.to_string()))?;
    let entries = value.get("calibration").unwrap_or(&value);
    let serde_json::Value::Object(entries) = entries else {
        return Err(refuse("not a calibration object".to_owned()));
    };
    entries
        .iter()
        .map(|(key, entry)| {
            let field = |name: &str| {
                entry
                    .get(name)
                    .and_then(serde_json::Value::as_u64)
                    .ok_or_else(|| refuse(format!("{key} has no {name}")))
            };
            let narrow = |name: &str| {
                field(name).and_then(|n| {
                    u16::try_from(n).map_err(|_| refuse(format!("{key}'s {name} {n}")))
                })
            };
            let id = u8::try_from(field("id")?).map_err(|_| refuse(format!("{key}'s id")))?;
            let homing_offset = match entry.get("homing_offset") {
                None => None,
                Some(offset) => Some(
                    offset
                        .as_i64()
                        .and_then(|n| i16::try_from(n).ok())
                        .filter(|n| n.unsigned_abs() <= 2047)
                        .ok_or_else(|| refuse(format!("{key}'s homing_offset {offset}")))?,
                ),
            };
            let (range_min, range_max) = (narrow("range_min")?, narrow("range_max")?);
            if range_min >= range_max || range_max > super::MAX_TICK {
                return Err(refuse(format!("{key}'s range {range_min}..{range_max}")));
            }
            Ok(Entry {
                id,
                homing_offset,
                range_min,
                range_max,
            })
        })
        .collect()
}

/// `Homing_Offset`'s register value: sign-magnitude at bit 11.
fn offset_register(offset: i16) -> u16 {
    let magnitude = offset.unsigned_abs() & 0x07FF;
    if offset < 0 {
        magnitude | (1 << super::HOMING_OFFSET_SIGN_BIT)
    } else {
        magnitude
    }
}

fn apply<P: SerialPort>(
    path: &std::path::Path,
    protection: Option<Protection>,
    session: &mut Session<P>,
    out: &mut Out<'_>,
) -> Result<serde_json::Value, CalibrateError> {
    let entries = entries_in(path)?;
    let mut applied = Vec::new();
    for entry in &entries {
        let id = entry.id;
        let mut writes = Vec::new();
        if let Some(offset) = entry.homing_offset {
            writes.push((SMS_STS_OFS_L, 2, offset_register(offset)));
        }
        writes.push((SMS_STS_MIN_ANGLE_LIMIT_L, 2, entry.range_min));
        writes.push((SMS_STS_MAX_ANGLE_LIMIT_L, 2, entry.range_max));
        let before = session.commit(id, &writes)?;
        let was = |address: u8| {
            writes
                .iter()
                .position(|(a, _, _)| *a == address)
                .map(|index| before[index])
        };
        let was_offset = was(SMS_STS_OFS_L).map(|raw| {
            ProtocolPacketHandler::<()>::scs_tohost(raw & 0x0FFF, super::HOMING_OFFSET_SIGN_BIT)
        });
        let (was_min, was_max) = (
            was(SMS_STS_MIN_ANGLE_LIMIT_L).unwrap_or_default(),
            was(SMS_STS_MAX_ANGLE_LIMIT_L).unwrap_or_default(),
        );
        let offset_text = match (was_offset, entry.homing_offset) {
            (Some(was), Some(now)) => format!("homing_offset {was} -> {now}, "),
            _ => String::new(),
        };
        let _ = writeln!(
            out.stdout,
            "servo {id}: {offset_text}limits {was_min}..{was_max} -> {}..{}, kept across power \
             cycles",
            entry.range_min, entry.range_max
        );
        applied.push(json!({
            "id": id,
            "was": { "homing_offset": was_offset, "range_min": was_min, "range_max": was_max },
            "now": { "homing_offset": entry.homing_offset, "range_min": entry.range_min,
                     "range_max": entry.range_max },
        }));
    }
    let mut protected = Vec::new();
    if let Some(p) = protection {
        for entry in &entries {
            let id = entry.id;
            let current = (
                session.register(id, MAX_TORQUE_LIMIT, 2)?,
                session.register(id, PROTECTION_CURRENT, 2)?,
                session.register(id, OVERLOAD_TORQUE, 1)?,
            );
            let wanted = (
                p.max_torque,
                p.protection_current,
                u16::from(p.overload_torque),
            );
            if current == wanted {
                let _ = writeln!(
                    out.stdout,
                    "servo {id}: protection already {}/{}/{}",
                    wanted.0, wanted.1, wanted.2
                );
                continue;
            }
            let _ = writeln!(
                out.stdout,
                "servo {id}: protection Max_Torque {} -> {}, Protection_Current {} -> {}, \
                 Overload_Torque {} -> {}, Torque_Limit -> {}",
                current.0, wanted.0, current.1, wanted.1, current.2, wanted.2, wanted.0
            );
            session.commit(
                id,
                &[
                    (MAX_TORQUE_LIMIT, 2, wanted.0),
                    (PROTECTION_CURRENT, 2, wanted.1),
                    (OVERLOAD_TORQUE, 1, wanted.2),
                    (super::TORQUE_LIMIT, 2, wanted.0),
                ],
            )?;
            protected.push(json!({ "id": id, "was": [current.0, current.1, current.2],
                                   "now": [wanted.0, wanted.1, wanted.2] }));
        }
    }
    Ok(json!({ "applied": applied, "protection": protected }))
}

/// Make the applied file the robot directory's commissioned calibration of `port`, written
/// beside its `commissioning.json` and renamed into place, once every servo holds it.
fn install(
    applied: &std::path::Path,
    robot: &std::path::Path,
    port: &str,
    out: &mut Out<'_>,
) -> Result<(), CalibrateError> {
    let refuse = |why: String| CalibrateError::Refused(why);
    let text = std::fs::read_to_string(applied)
        .map_err(|e| refuse(format!("{}: {e}", applied.display())))?;
    let value: Value =
        serde_json::from_str(&text).map_err(|e| refuse(format!("{}: {e}", applied.display())))?;
    let entries = value.get("calibration").unwrap_or(&value);
    // The file the station reads: a LeRobot calibration, checked as the station parses it.
    let document = format!("{entries:#}\n");
    sx_embodiment_bodies::ServoCalibrations::new()
        .with_lerobot(port, &document)
        .map_err(|e| refuse(format!("{}: {e}", applied.display())))?;
    let inbox = robot == std::path::Path::new(STATION_CALIBRATION_INBOX);
    if inbox && !robot.is_dir() {
        // The station makes its inbox, so it owns it and can clear what it publishes; one made
        // here, as root, it could not clear.
        return Err(refuse(format!(
            "{} is missing: no station runs here, or it has not started since it was installed",
            robot.display()
        )));
    }
    std::fs::create_dir_all(robot).map_err(|e| refuse(format!("{}: {e}", robot.display())))?;
    let path = robot.join(sx_embodiment_bodies::calibration_file(port));
    let staging = path.with_extension("json.next");
    std::fs::write(&staging, &document)
        .and_then(|()| std::fs::rename(&staging, &path))
        .map_err(|e| refuse(format!("{}: {e}", path.display())))?;
    if inbox {
        let _ = writeln!(
            out.stdout,
            "published {}: the station sends it to Fleet with its own key, and the robot runs it \
             within seconds, between sessions (Fleet's GetStationRobot names the new digest as \
             running)",
            path.display()
        );
    } else {
        let _ = writeln!(
            out.stdout,
            "installed {} as {port}'s commissioned calibration: the station and the bench tools \
             load it by default",
            path.display()
        );
    }
    Ok(())
}

fn range_mode<P: SerialPort>(
    args: &Args,
    open: &mut impl FnMut(&str, u32) -> io::Result<P>,
    stop: &Arc<AtomicUsize>,
    prompt: &mut dyn Prompt,
    out: &mut Out<'_>,
) -> Result<(), CalibrateError> {
    let port = open_bus(open, bus(args), args.baud)?;
    let plan = RangePlan {
        ids: args.ids.clone(),
        joints: args.joints.clone(),
        full_turn: args.full_turn.clone(),
        period: Duration::from_millis(20),
        print_every: 5,
        redraw: args.redraw,
    };
    let joints = range::measure(port, &plan, prompt, stop, out.stdout)?;
    let calibration = range::calibration_json(&joints);
    let _ = writeln!(out.stdout, "\nMeasured (nothing was written to any servo):");
    for joint in &joints {
        let _ = writeln!(
            out.stdout,
            "  {:>28} id {:2}: homing_offset {:5}  range {:4}..{:4}  ({:+.1}° .. {:+.1}°)",
            joint.joint,
            joint.id,
            joint.homing_offset,
            joint.range_min,
            joint.range_max,
            range::degrees(joint.range_min),
            range::degrees(joint.range_max)
        );
        for warning in range::warnings(joint, args.full_turn.contains(&joint.id)) {
            let _ = writeln!(out.stdout, "    warning: {warning}");
        }
    }
    let text = format!("{calibration:#}\n");
    if let Some(path) = &args.out {
        std::fs::write(path, &text)
            .map_err(|e| CalibrateError::Refused(format!("{}: {e}", path.display())))?;
        let _ = writeln!(
            out.stdout,
            "wrote {}; to make it the servos' own: sx-commission calibrate apply --bus {} \
             --calibration {} --move",
            path.display(),
            bus(args),
            path.display()
        );
    } else {
        let _ = write!(out.stdout, "{text}");
    }
    Ok(())
}
