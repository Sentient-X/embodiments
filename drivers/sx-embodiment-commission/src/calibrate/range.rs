//! `calibrate range`: a person shows each joint's middle and limits by hand, and the tool
//! records them as a `LeRobot` calibration, writing nothing to any servo.
//!
//! It follows `LeRobot`'s `SOFollower.calibrate` (`v0.6.0`,
//! `src/lerobot/robots/so_follower/so_follower.py`) without its writes:
//!
//! 1. Every listed servo must read `Torque_Enable` 0; a powered one refuses the run.
//! 2. The person puts every joint at the middle of its travel and presses Enter. `LeRobot`
//!    resets each `Homing_Offset` to 0, reads the raw position and writes
//!    `Homing_Offset = raw - 2047` (`FeetechMotorsBus._get_half_turn_homings`, `int(4095 / 2)`),
//!    so that pose then reads 2047. Here the raw position is computed instead of reset:
//!    `Present_Position` is `raw - Homing_Offset` on the STS, so `raw = (present + offset) mod
//!    4096` under the offset the servo holds now, and the new offset is `raw - 2047`.
//! 3. The person moves each joint slowly through its full travel; the tool polls every servo
//!    (about 50 Hz) and keeps each one's least and greatest position *in the new frame*,
//!    `(present + held_offset - new_offset) mod 4096`, printing a live line per joint. Enter
//!    ends it. A full-turn joint (`--full-turn`, `LeRobot`'s wrist roll) is given 0..4095.
//! 4. The result is a `LeRobot` calibration keyed by joint (`homing_offset`, `range_min`,
//!    `range_max`, `drive_mode` 0, `id`): the file the station loads, and what
//!    `calibrate apply --move` writes to the servos.
//!
//! Every packet goes through the scan's [`ReadOnlyPort`], so this mode cannot write a register
//! however it is driven; it opens no journal and needs no `--move`.

use std::io::{self, BufRead as _};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use serde_json::{Value, json};
use sx_embodiment_drivers::serial::SerialPort;
use sx_feetech_serial::scan::ReadOnlyPort;
use sx_feetech_serial::scservo_sdk::{ProtocolPacketHandler, SMS_STS_TORQUE_ENABLE};

use super::read;
use super::session::READ_ATTEMPTS;
use super::{CalibrateError, MAX_TICK};

/// `LeRobot`'s half turn: the position the middle pose reads once its offset is written.
pub const MIDDLE: i32 = 2047;
/// Encoder steps a turn.
const TURN: i32 = 4096;
/// The largest `Homing_Offset` magnitude: eleven bits, sign at bit 11.
const OFFSET_MAX: i32 = 2047;
/// A joint whose recorded travel is shorter than this was probably not moved.
pub const SHORT_TRAVEL: i32 = 100;
/// Within this many ticks of 0 or 4095 a joint may have wrapped past the frame's edge.
const EDGE: i32 = 20;

/// Where the person's Enter presses come from.
pub trait Prompt {
    /// Block until the person presses Enter.
    ///
    /// # Errors
    ///
    /// The input closed before Enter.
    fn wait(&mut self) -> Result<(), CalibrateError>;
    /// Whether Enter was pressed since the last call, without blocking.
    fn pressed(&mut self) -> bool;
}

/// Enter presses read from standard input on a thread of their own, so polling never waits on
/// the terminal.
pub struct StdinPrompt {
    lines: mpsc::Receiver<()>,
}

impl StdinPrompt {
    #[must_use]
    pub fn new() -> Self {
        let (send, lines) = mpsc::channel();
        std::thread::spawn(move || {
            let stdin = io::stdin();
            for line in stdin.lock().lines() {
                if line.is_err() || send.send(()).is_err() {
                    break;
                }
            }
        });
        Self { lines }
    }
}

impl Default for StdinPrompt {
    fn default() -> Self {
        Self::new()
    }
}

impl Prompt for StdinPrompt {
    fn wait(&mut self) -> Result<(), CalibrateError> {
        self.lines
            .recv()
            .map_err(|_| CalibrateError::Refused("standard input closed".to_owned()))
    }

    fn pressed(&mut self) -> bool {
        self.lines.try_recv().is_ok()
    }
}

/// What `calibrate range` was asked.
#[derive(Clone, Debug)]
pub struct RangePlan {
    pub ids: Vec<u8>,
    /// One name per id, the calibration's keys; the id itself where none is given.
    pub joints: Vec<String>,
    /// Joints that turn without end, given 0..4095.
    pub full_turn: Vec<u8>,
    /// How often every servo is read.
    pub period: Duration,
    /// Every this many polls the live lines are printed again.
    pub print_every: u32,
    /// Redraw the live lines in place (a terminal), rather than append them.
    pub redraw: bool,
}

/// One joint's measured calibration.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct JointRange {
    pub joint: String,
    pub id: u8,
    /// The offset the servo held during the run.
    pub held_offset: i32,
    /// The offset that makes the middle pose read [`MIDDLE`].
    pub homing_offset: i32,
    pub range_min: i32,
    pub range_max: i32,
    /// Readings that failed during the range loop, after retries.
    pub missed: u32,
}

impl JointRange {
    /// The `LeRobot` calibration entry.
    #[must_use]
    pub fn json(&self) -> Value {
        json!({
            "id": self.id,
            "drive_mode": 0,
            "homing_offset": self.homing_offset,
            "range_min": self.range_min,
            "range_max": self.range_max,
        })
    }
}

/// `ticks` in the new frame as `LeRobot`'s degrees about the middle.
#[must_use]
pub fn degrees(ticks: i32) -> f64 {
    f64::from(ticks - MIDDLE) * 360.0 / f64::from(TURN)
}

/// The encoder's own position for a `present` reading under `held_offset`.
#[must_use]
pub const fn raw(present: i32, held_offset: i32) -> i32 {
    (present + held_offset).rem_euclid(TURN)
}

/// The `Homing_Offset` that makes a servo resting at `present` under `held_offset` read
/// [`MIDDLE`]; `None` when it does not fit the register's eleven bits (the joint rests at the
/// one raw position a half turn away, 4095: move it a tick).
#[must_use]
pub const fn middle_offset(present: i32, held_offset: i32) -> Option<i32> {
    let offset = raw(present, held_offset) - MIDDLE;
    if offset.abs() > OFFSET_MAX {
        None
    } else {
        Some(offset)
    }
}

/// A `present` reading under `held_offset`, as the servo will read it under `new_offset`.
#[must_use]
pub const fn reframe(present: i32, held_offset: i32, new_offset: i32) -> i32 {
    (present + held_offset - new_offset).rem_euclid(TURN)
}

fn present<P: SerialPort>(
    ph: &mut ProtocolPacketHandler<P>,
    id: u8,
) -> Result<i32, CalibrateError> {
    let mut attempt = 1;
    loop {
        match read::state(ph, id) {
            Ok(state) => return Ok(state.position),
            Err(CalibrateError::Comm { .. }) if attempt < READ_ATTEMPTS => attempt += 1,
            Err(error) => return Err(error),
        }
    }
}

fn stopped(stop: &AtomicUsize) -> Result<(), CalibrateError> {
    match stop.load(Ordering::SeqCst) {
        0 => Ok(()),
        signal => Err(CalibrateError::Signal(signal)),
    }
}

fn live_line(joint: &JointRange, now: Option<i32>) -> String {
    let now = now.map_or_else(
        || "   --- (missed)   ".to_owned(),
        |ticks| format!("{ticks:4} {:+7.1}°", degrees(ticks)),
    );
    format!(
        "  {:>28} id {:2}  now {now}   min {:4} {:+7.1}°   max {:4} {:+7.1}°",
        joint.joint,
        joint.id,
        joint.range_min,
        degrees(joint.range_min),
        joint.range_max,
        degrees(joint.range_max),
    )
}

/// Every joint's middle, read where the person holds it: the offset that makes it read
/// [`MIDDLE`], under the offset `held` each servo holds now.
fn middles<P: SerialPort>(
    ph: &mut ProtocolPacketHandler<P>,
    plan: &RangePlan,
    held: &[i32],
    out: &mut dyn io::Write,
) -> Result<Vec<JointRange>, CalibrateError> {
    let mut joints = Vec::new();
    for (index, &id) in plan.ids.iter().enumerate() {
        let at = present(ph, id)?;
        let held_offset = held[index];
        let homing_offset = middle_offset(at, held_offset).ok_or_else(|| {
            CalibrateError::Refused(format!(
                "servo {id} rests a half turn from the middle frame (raw 4095): move it a tick and \
                 run again"
            ))
        })?;
        let joint = plan
            .joints
            .get(index)
            .cloned()
            .unwrap_or_else(|| id.to_string());
        let _ = writeln!(
            out,
            "  {joint:>28} id {id:2}: middle at raw {}, homing_offset {homing_offset} (held \
             {held_offset})",
            raw(at, held_offset)
        );
        joints.push(JointRange {
            joint,
            id,
            held_offset,
            homing_offset,
            range_min: MIDDLE,
            range_max: MIDDLE,
            missed: 0,
        });
    }
    Ok(joints)
}

/// Run the hand-guided calibration on `port`: refuse a powered servo, take the middle at the
/// first Enter, then record every joint's travel until the next.
///
/// # Errors
///
/// A powered servo, a servo that does not answer, a middle whose offset does not fit, the
/// input closing, or a caught signal.
pub fn measure<P: SerialPort>(
    port: P,
    plan: &RangePlan,
    prompt: &mut dyn Prompt,
    stop: &Arc<AtomicUsize>,
    out: &mut dyn io::Write,
) -> Result<Vec<JointRange>, CalibrateError> {
    let mut ph = ProtocolPacketHandler::new(ReadOnlyPort::new(port), 0);
    let mut powered = Vec::new();
    let mut held = Vec::new();
    for &id in &plan.ids {
        let torque = read::register(&mut ph, id, SMS_STS_TORQUE_ENABLE, 1)?;
        if torque != 0 {
            powered.push(id);
        }
        held.push(i32::from(read::homing_offset(&mut ph, id)?));
    }
    if !powered.is_empty() {
        return Err(CalibrateError::Refused(format!(
            "servos {powered:?} hold torque: a joint is shown by hand only with every servo limp \
             (stop whatever powered them first)"
        )));
    }
    let _ = writeln!(
        out,
        "Every servo is limp; nothing will be written.\n\
         Step 1: put every joint at the MIDDLE of its travel, then press Enter."
    );
    let _ = out.flush();
    prompt.wait()?;
    stopped(stop)?;
    let mut joints = middles(&mut ph, plan, &held, out)?;
    let _ = writeln!(
        out,
        "Step 2: move each joint SLOWLY through its full travel, end to end. Press Enter when \
         done (Ctrl-C abandons the run)."
    );
    let _ = out.flush();
    let mut polls: u32 = 0;
    let mut now: Vec<Option<i32>> = vec![None; joints.len()];
    let mut drawn = false;
    loop {
        let tick = Instant::now();
        stopped(stop)?;
        for (index, joint) in joints.iter_mut().enumerate() {
            now[index] = match present(&mut ph, joint.id) {
                Ok(at) => {
                    let ticks = reframe(at, joint.held_offset, joint.homing_offset);
                    joint.range_min = joint.range_min.min(ticks);
                    joint.range_max = joint.range_max.max(ticks);
                    Some(ticks)
                }
                Err(CalibrateError::Comm { .. }) => {
                    joint.missed += 1;
                    None
                }
                Err(error) => return Err(error),
            };
        }
        let done = prompt.pressed();
        if polls.is_multiple_of(plan.print_every.max(1)) || done {
            if plan.redraw && drawn {
                let _ = write!(out, "\x1b[{}A", joints.len());
            }
            let clear = if plan.redraw { "\x1b[2K" } else { "" };
            for (joint, at) in joints.iter().zip(&now) {
                let _ = writeln!(out, "{clear}{}", live_line(joint, *at));
            }
            let _ = out.flush();
            drawn = true;
        }
        if done {
            break;
        }
        polls = polls.wrapping_add(1);
        if let Some(rest) = plan.period.checked_sub(tick.elapsed()) {
            std::thread::sleep(rest);
        }
    }
    for joint in &mut joints {
        if plan.full_turn.contains(&joint.id) {
            joint.range_min = 0;
            joint.range_max = i32::from(MAX_TICK);
        }
    }
    Ok(joints)
}

/// What to tell the person about a measured joint before trusting it.
#[must_use]
pub fn warnings(joint: &JointRange, full_turn: bool) -> Vec<String> {
    let mut found = Vec::new();
    if full_turn {
        return found;
    }
    if joint.range_max - joint.range_min < SHORT_TRAVEL {
        found.push(format!(
            "{} travelled only {} ticks: was it moved?",
            joint.joint,
            joint.range_max - joint.range_min
        ));
    }
    if joint.range_min <= EDGE || joint.range_max >= i32::from(MAX_TICK) - EDGE {
        found.push(format!(
            "{} reached the frame's edge ({}..{}): it may turn more than half a turn either side \
             of its middle; if it turns without end, pass --full-turn {}",
            joint.joint, joint.range_min, joint.range_max, joint.id
        ));
    }
    if joint.missed > 0 {
        found.push(format!(
            "{} missed {} readings (after retries)",
            joint.joint, joint.missed
        ));
    }
    found
}

/// The calibration file: every joint's entry, keyed by joint.
#[must_use]
pub fn calibration_json(joints: &[JointRange]) -> Value {
    Value::Object(
        joints
            .iter()
            .map(|joint| (joint.joint.clone(), joint.json()))
            .collect(),
    )
}
