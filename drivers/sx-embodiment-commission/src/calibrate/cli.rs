//! `sx-commission calibrate`: its arguments, and each mode run over whatever opens the bus.

use std::io;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::AtomicUsize;
use std::time::Duration;

use serde_json::{Value, json};
use sx_embodiment_drivers::serial::SerialPort;
use sx_feetech_serial::scan::{ReadOnlyPort, STS3215_BAUD};
use sx_feetech_serial::scservo_sdk::ProtocolPacketHandler;

use super::CalibrateError;
use super::journal::Journal;
use super::motion::{Closed, Sweep, Watch, jog, sweep_finger};
use super::read::{self, read_bus};
use super::session::{Session, Slow};

pub const USAGE: &str = "usage:
  sx-commission calibrate read --bus <tty> [--ids 1-20] [--json]
  sx-commission calibrate gripper --bus <tty> --ids 1,2 [--move] [--widen <ticks>]
                                  [--closed 1:low,2:high] [--json] [motion settings]
  sx-commission calibrate jog --bus <tty> --id <n> --delta <ticks> [--move] [--json]
                              [motion settings]
  sx-commission calibrate --restore <journal>

  Without --move, gripper and jog only read and print what they would do.
  motion settings (defaults): --goal-speed 20 --acceleration 5 --torque-limit 250
    --load-threshold 150 --lead 20 --backoff 30 --stall-ms 600 --min-progress 3
    --max-travel 1500 --inset 10 --max-seconds 180 --baud 1000000 --journal-dir <dir>
    --trace <file> (one JSON line per motion step)";

/// The largest nudge `jog` takes, ticks (about 35 degrees).
pub const MAX_JOG: i32 = 400;

/// What `calibrate` was asked.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Mode {
    Read,
    Gripper,
    Jog { id: u8, delta: i32 },
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
}

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
    };
    let (mut id, mut delta, mut restore) = (None, None, None);
    while let Some(flag) = words.next() {
        let mut value = || words.next().ok_or(format!("{flag} needs a value"));
        let sweep = &mut args.sweep;
        match flag {
            "--bus" => args.bus = Some(value()?.to_owned()),
            "--baud" => args.baud = number(flag, value()?)?,
            "--ids" => args.ids = ids(value()?)?,
            "--id" => id = Some(number::<u8>(flag, value()?)?),
            "--delta" => delta = Some(number::<i32>(flag, value()?)?),
            "--move" => args.moves = true,
            "--json" => args.json = true,
            "--widen" => args.widen = Some(number(flag, value()?)?),
            "--closed" => args.closed = closed(value()?)?,
            "--restore" => restore = Some(PathBuf::from(value()?)),
            "--journal-dir" => args.journal_dir = PathBuf::from(value()?),
            "--trace" => args.trace = Some(PathBuf::from(value()?)),
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
        (Some(other), None) => return Err(format!("unknown calibrate mode {other}")),
        (None, None) => return Err("calibrate needs a mode".to_owned()),
    };
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
    mut open: impl FnMut(&str, u32) -> io::Result<P>,
    stop: &Arc<AtomicUsize>,
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
        Mode::Gripper | Mode::Jog { .. } if !args.moves => dry_run(args, &mut open, &mut out),
        Mode::Gripper | Mode::Jog { .. } => moving(args, &mut open, stop, &mut out),
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
        Mode::Jog { id, .. } => vec![id],
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
        let line = if let Mode::Jog { delta, .. } = args.mode {
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
        Mode::Jog { id, delta } => {
            jog(&mut session, id, delta, &args.sweep.slow, &args.sweep.watch).map(|jog| {
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
