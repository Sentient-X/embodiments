//! The motions a calibration makes, all through a [`Session`]: a finger swept to its stops,
//! and a joint nudged and brought back.
//!
//! A sweep never commands far ahead of the servo: each step reads where it is and sets its
//! goal `lead` ticks further, so against a stop the servo pushes with at most `lead` ticks of
//! position error, under the session's lowered `Torque_Limit`. It ends at the first of: a
//! `Present_Load` at or above the threshold, a position that has not advanced `min_progress`
//! ticks within `stall_window`, the servo's position limit, or `max_travel` ticks from where it
//! started. At a stall it backs off `backoff` ticks at once, in the same step.

use std::time::{Duration, Instant};

use serde_json::{Value, json};
use sx_embodiment_drivers::lerobot::MotorCalibration;
use sx_embodiment_drivers::serial::SerialPort;

use super::CalibrateError;
use super::session::{RestoreReport, Session, Slow};

/// What a motion watches, and how it reacts.
#[derive(Clone, Copy, Debug)]
pub struct Watch {
    /// `|Present_Load|` at or above this, tenths of a percent, is a stall.
    pub load_threshold: u16,
    /// A position that has not advanced `min_progress` ticks within this is a stall.
    pub stall_window: Duration,
    pub min_progress: u16,
    /// Between steps.
    pub period: Duration,
    /// Ticks backed off at a stall.
    pub backoff: u16,
    /// A move is done within this many ticks of its target.
    pub tolerance: u16,
    /// Waited after backing off, for the servo to leave the stop.
    pub settle: Duration,
}

/// A sweep's settings.
#[derive(Clone, Copy, Debug)]
pub struct Sweep {
    pub slow: Slow,
    pub watch: Watch,
    /// Ticks the goal runs ahead of where the servo is.
    pub lead: u16,
    /// The farthest a sweep goes from where it started.
    pub max_travel: u16,
    /// Within this many ticks of its position limit, a sweep has reached it.
    pub limit_margin: u16,
}

/// Which way a sweep goes, in encoder ticks.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Direction {
    Decreasing,
    Increasing,
}

impl Direction {
    const fn sign(self) -> i32 {
        match self {
            Self::Decreasing => -1,
            Self::Increasing => 1,
        }
    }

    const fn name(self) -> &'static str {
        match self {
            Self::Decreasing => "decreasing",
            Self::Increasing => "increasing",
        }
    }
}

/// Why a sweep or a move ended.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum End {
    /// `Present_Load` reached the threshold: a stop.
    Load(i32),
    /// The position stopped advancing: a stop, or friction the lowered torque cannot pass.
    NoProgress,
    /// The position never advanced as far as the lead: the lowered torque did not move it,
    /// so where it rests says nothing about a stop. Raise the lead or the torque limit.
    Stuck,
    /// The servo's position limit, before any stop.
    Limit(u16),
    /// `max_travel` from the start, before any stop.
    MaxTravel,
    /// A move reached its target.
    Arrived,
}

impl End {
    /// Whether the servo stopped against something.
    #[must_use]
    pub const fn is_stop(self) -> bool {
        matches!(self, Self::Load(_) | Self::NoProgress)
    }

    fn json(self) -> Value {
        match self {
            Self::Load(load) => json!({ "kind": "load", "load": load }),
            Self::NoProgress => json!({ "kind": "no_progress" }),
            Self::Stuck => json!({ "kind": "stuck" }),
            Self::Limit(limit) => json!({ "kind": "limit", "limit": limit }),
            Self::MaxTravel => json!({ "kind": "max_travel" }),
            Self::Arrived => json!({ "kind": "arrived" }),
        }
    }
}

/// One end of a sweep.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SweepEnd {
    pub direction: Direction,
    /// The farthest position reached, ticks.
    pub reached: i32,
    pub end: End,
    /// Where it rested after backing off.
    pub backed_off_to: i32,
}

/// A move to a target.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Arrival {
    pub from: i32,
    pub at: i32,
    pub elapsed: Duration,
    pub end: End,
}

impl Arrival {
    /// Ticks a second over the move, when it moved.
    #[must_use]
    pub fn ticks_per_second(&self) -> Option<f64> {
        let seconds = self.elapsed.as_secs_f64();
        (seconds > 0.0 && self.at != self.from)
            .then(|| f64::from((self.at - self.from).abs()) / seconds)
    }
}

fn clamp_to(goal: i32, (min, max): (u16, u16)) -> i32 {
    goal.clamp(i32::from(min), i32::from(max))
}

/// Back off from a stall in the same step, then wait for the servo to leave it.
fn back_off<P: SerialPort>(
    session: &mut Session<P>,
    id: u8,
    at: i32,
    sign: i32,
    watch: &Watch,
) -> Result<i32, CalibrateError> {
    let limits = session.powered_limits(id)?;
    session.goal(id, clamp_to(at - sign * i32::from(watch.backoff), limits))?;
    session.sleep(watch.settle)?;
    Ok(session.state(id)?.position)
}

/// Sweep the powered servo `id` toward `direction` until it stops; see the module header.
///
/// # Errors
///
/// A signal, the deadline, or the bus.
pub fn sweep<P: SerialPort>(
    session: &mut Session<P>,
    id: u8,
    direction: Direction,
    settings: &Sweep,
) -> Result<SweepEnd, CalibrateError> {
    let watch = &settings.watch;
    let sign = direction.sign();
    let limits = session.powered_limits(id)?;
    let limit = if sign > 0 { limits.1 } else { limits.0 };
    let origin = session.state(id)?.position;
    let (mut farthest, mut mark, mut mark_at) = (origin, origin, Instant::now());
    loop {
        let state = session.state(id)?;
        let at = state.position;
        if sign * (at - farthest) > 0 {
            farthest = at;
        }
        if sign * (at - mark) >= i32::from(watch.min_progress) {
            (mark, mark_at) = (at, Instant::now());
        }
        let end = if state.load.unsigned_abs() >= u32::from(watch.load_threshold) {
            Some(End::Load(state.load))
        } else if sign * (i32::from(limit) - at) <= i32::from(settings.limit_margin) {
            Some(End::Limit(limit))
        } else if (at - origin).abs() >= i32::from(settings.max_travel) {
            Some(End::MaxTravel)
        } else if mark_at.elapsed() >= watch.stall_window {
            if sign * (i32::from(limit) - at) < i32::from(settings.lead) {
                // Within a lead of its limit the goal is clamped there, and a servo a few
                // ticks short of a goal may not move at all: that is the limit, not a stop.
                Some(End::Limit(limit))
            } else if sign * (farthest - origin) < i32::from(settings.lead) {
                Some(End::Stuck)
            } else {
                Some(End::NoProgress)
            }
        } else {
            None
        };
        if let Some(end) = end {
            let backed_off_to = if end.is_stop() {
                back_off(session, id, at, sign, watch)?
            } else {
                session.goal(id, clamp_to(at, limits))?;
                at
            };
            return Ok(SweepEnd {
                direction,
                reached: farthest,
                end,
                backed_off_to,
            });
        }
        session.goal(id, clamp_to(at + sign * i32::from(settings.lead), limits))?;
        session.sleep(watch.period)?;
    }
}

/// Move the powered servo `id` to `target` at its own `Goal_Speed`, watching it as a sweep
/// does: at a load stall it backs off, at no progress it holds, and the move ends there.
///
/// # Errors
///
/// A signal, the deadline, a target outside its limits, or the bus.
pub fn drive_to<P: SerialPort>(
    session: &mut Session<P>,
    id: u8,
    target: i32,
    watch: &Watch,
) -> Result<Arrival, CalibrateError> {
    let started = Instant::now();
    let from = session.state(id)?.position;
    let sign = (target - from).signum();
    session.goal(id, target)?;
    let (mut mark, mut mark_at) = (from, Instant::now());
    loop {
        session.sleep(watch.period)?;
        let state = session.state(id)?;
        let at = state.position;
        if (at - target).abs() <= i32::from(watch.tolerance) {
            return Ok(Arrival {
                from,
                at,
                elapsed: started.elapsed(),
                end: End::Arrived,
            });
        }
        if sign * (at - mark) >= i32::from(watch.min_progress) {
            (mark, mark_at) = (at, Instant::now());
        }
        let end = if state.load.unsigned_abs() >= u32::from(watch.load_threshold) {
            Some(End::Load(state.load))
        } else if mark_at.elapsed() >= watch.stall_window {
            Some(End::NoProgress)
        } else {
            None
        };
        if let Some(end) = end {
            let elapsed = started.elapsed();
            // Only a load stall pushes against something to back away from. A move that stops
            // advancing without load (friction a few ticks short) holds where it is: backing
            // off would send it the other way, toward wherever it came from.
            if matches!(end, End::Load(_)) {
                back_off(session, id, at, sign, watch)?;
            } else {
                let limits = session.powered_limits(id)?;
                session.goal(id, clamp_to(at, limits))?;
            }
            return Ok(Arrival {
                from,
                at,
                elapsed,
                end,
            });
        }
    }
}

/// Whether a move back to where a finger started got there: arrived, or stopped by friction
/// within `backoff` ticks of it with no load stall. Where it rests between sweeps needs no
/// precision; it only must not be left against a stop.
fn returned_to(arrival: &Arrival, start: i32, watch: &Watch) -> bool {
    match arrival.end {
        End::Arrived => true,
        End::NoProgress | End::Stuck => (arrival.at - start).abs() <= i32::from(watch.backoff),
        _ => false,
    }
}

/// A finger swept to both stops.
#[derive(Clone, Debug)]
pub struct FingerSweep {
    pub id: u8,
    pub start: i32,
    /// The limits it was swept within (widened, where asked).
    pub limits: (u16, u16),
    pub homing_offset: i16,
    pub low: SweepEnd,
    pub high: SweepEnd,
    pub returned: Arrival,
    pub undone: RestoreReport,
}

/// Which end of a finger's travel closes it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Closed {
    Low,
    High,
}

impl FingerSweep {
    /// The `LeRobot` calibration the stops give, inset `inset` ticks from each, when both ends
    /// are stops. `drive_mode` is set when the closed end is the high one, so `LeRobot`'s
    /// percent reads 0 closed either way; unknown when `closed` is.
    #[must_use]
    pub fn calibration(&self, closed: Option<Closed>, inset: u16) -> Option<MotorCalibration> {
        if !(self.low.end.is_stop() && self.high.end.is_stop()) {
            return None;
        }
        let low = u16::try_from(self.low.reached + i32::from(inset)).ok()?;
        let high = u16::try_from(self.high.reached - i32::from(inset)).ok()?;
        (low < high).then_some(MotorCalibration {
            drive_mode: closed == Some(Closed::High),
            homing_offset: self.homing_offset,
            range_min: low,
            range_max: high,
        })
    }

    #[must_use]
    pub fn json(&self, closed: Option<Closed>, inset: u16) -> Value {
        let end = |end: &SweepEnd| {
            json!({
                "direction": end.direction.name(),
                "reached": end.reached,
                "end": end.end.json(),
                "backed_off_to": end.backed_off_to,
            })
        };
        let closing = closed.map(|end| match end {
            Closed::Low => "decreasing ticks close it",
            Closed::High => "increasing ticks close it",
        });
        json!({
            "id": self.id,
            "start": self.start,
            "limits": [self.limits.0, self.limits.1],
            "low": end(&self.low),
            "high": end(&self.high),
            "returned_to": self.returned.at,
            "closes": closing,
            "calibration": self.calibration(closed, inset).map(|c| json!({
                "id": self.id,
                "drive_mode": u8::from(c.drive_mode),
                "homing_offset": c.homing_offset,
                "range_min": c.range_min,
                "range_max": c.range_max,
            })),
        })
    }
}

/// Sweep finger `id` down to its low stop, back to where it started, up to its high stop and
/// back, then undo what was done to it. `widen` ticks widen its limits on both sides first.
///
/// # Errors
///
/// A signal, the deadline, the bus, or a finger that cannot be brought back.
pub fn sweep_finger<P: SerialPort>(
    session: &mut Session<P>,
    id: u8,
    settings: &Sweep,
    widen: Option<u16>,
) -> Result<FingerSweep, CalibrateError> {
    let (min, max) = session.limits(id)?;
    let homing_offset = session.homing_offset(id)?;
    if let Some(widen) = widen {
        session.widen_limits(
            id,
            min.saturating_sub(widen),
            max.saturating_add(widen).min(super::MAX_TICK),
        )?;
    }
    let start = session.enable(id, &settings.slow)?;
    let limits = session.powered_limits(id)?;
    let low = sweep(session, id, Direction::Decreasing, settings)?;
    let back = drive_to(session, id, start, &settings.watch)?;
    if !returned_to(&back, start, &settings.watch) {
        return Err(CalibrateError::Refused(format!(
            "servo {id} stopped at {} on its way back to {start}",
            back.at
        )));
    }
    let high = sweep(session, id, Direction::Increasing, settings)?;
    let returned = drive_to(session, id, start, &settings.watch)?;
    if !returned_to(&returned, start, &settings.watch) {
        return Err(CalibrateError::Refused(format!(
            "servo {id} stopped at {} on its way back to {start}",
            returned.at
        )));
    }
    let undone = session.finish_servo(id)?;
    Ok(FingerSweep {
        id,
        start,
        limits,
        homing_offset,
        low,
        high,
        returned,
        undone,
    })
}

/// Move servo `id` to `target` ticks and leave it there limp: widen its limits by `widen`
/// first where asked (a servo resting outside them is otherwise refused), power it where it
/// rests, drive it, and undo what was done to it. Its gears hold it where it was left.
///
/// # Errors
///
/// A target outside its limits, a signal, the deadline, or the bus.
pub fn go_to<P: SerialPort>(
    session: &mut Session<P>,
    id: u8,
    target: u16,
    slow: &Slow,
    watch: &Watch,
    widen: Option<u16>,
) -> Result<Arrival, CalibrateError> {
    let (min, max) = session.limits(id)?;
    if let Some(widen) = widen {
        session.widen_limits(
            id,
            min.saturating_sub(widen),
            max.saturating_add(widen).min(super::MAX_TICK),
        )?;
    }
    let (min, max) = session.limits(id)?;
    if target < min || target > max {
        return Err(CalibrateError::OutsideLimits {
            id,
            goal: i32::from(target),
            min,
            max,
        });
    }
    session.enable(id, slow)?;
    let arrival = drive_to(session, id, i32::from(target), watch)?;
    session.finish_servo(id)?;
    Ok(arrival)
}

/// Seconds since the Unix epoch, for aligning a hold with another recorder's clock.
fn unix_now() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0.0, |since| since.as_secs_f64())
}

/// A jog's hold at its target, with the wall clock at each end.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hold {
    pub started_unix: f64,
    pub ended_unix: f64,
    /// `Present_Position` at the start and end of the hold, and its extremes during it.
    pub at_start: i32,
    pub at_end: i32,
    pub min: i32,
    pub max: i32,
}

/// What a jog announces as it holds.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum JogEvent {
    Holding { id: u8, at: i32 },
    Returned { id: u8, at: i32 },
}

/// How a jog runs beyond its delta.
#[derive(Clone, Copy, Debug, Default)]
pub struct JogPlan {
    /// Widen the servo's limits by this many ticks for the jog, as a sweep does.
    pub widen: Option<u16>,
    /// Hold at the target this long before coming back.
    pub hold: Duration,
}

/// A joint nudged and brought back.
#[derive(Clone, Debug)]
pub struct Jog {
    pub id: u8,
    pub start: i32,
    pub target: i32,
    pub out: Arrival,
    pub hold: Option<Hold>,
    pub back: Arrival,
    pub slow: Slow,
    pub undone: RestoreReport,
}

impl Jog {
    #[must_use]
    pub fn json(&self) -> Value {
        let arrival = |a: &Arrival| {
            json!({
                "from": a.from,
                "at": a.at,
                "seconds": a.elapsed.as_secs_f64(),
                "ticks_per_second": a.ticks_per_second(),
                "end": a.end.json(),
            })
        };
        json!({
            "id": self.id,
            "start": self.start,
            "target": self.target,
            "goal_speed": self.slow.goal_speed,
            "acceleration": self.slow.acceleration,
            "torque_limit": self.slow.torque_limit,
            "out": arrival(&self.out),
            "hold": self.hold.map(|h| json!({
                "started_unix": h.started_unix,
                "ended_unix": h.ended_unix,
                "at_start": h.at_start,
                "at_end": h.at_end,
                "min": h.min,
                "max": h.max,
            })),
            "back": arrival(&self.back),
        })
    }
}

/// Power servo `id` where it rests, move it `delta` ticks, hold it there `plan.hold` while
/// reading where it is, bring it back, and undo what was done to it. `notify` hears the hold
/// start and the return. Whatever stops the move out, the servo is still brought back.
///
/// # Errors
///
/// A target outside its limits, a signal, the deadline, or the bus.
pub fn jog<P: SerialPort>(
    session: &mut Session<P>,
    id: u8,
    delta: i32,
    slow: &Slow,
    watch: &Watch,
    plan: &JogPlan,
    notify: &mut dyn FnMut(JogEvent),
) -> Result<Jog, CalibrateError> {
    if let Some(widen) = plan.widen {
        let (min, max) = session.limits(id)?;
        session.widen_limits(
            id,
            min.saturating_sub(widen),
            max.saturating_add(widen).min(super::MAX_TICK),
        )?;
    }
    let (min, max) = session.limits(id)?;
    let start = session.state(id)?.position;
    let target = start + delta;
    if target < i32::from(min) || target > i32::from(max) {
        return Err(CalibrateError::OutsideLimits {
            id,
            goal: target,
            min,
            max,
        });
    }
    let start = session.enable(id, slow)?;
    let out = drive_to(session, id, start + delta, watch)?;
    let hold = if plan.hold.is_zero() {
        None
    } else {
        let at_start = session.state(id)?.position;
        notify(JogEvent::Holding { id, at: at_start });
        let started_unix = unix_now();
        let until = Instant::now() + plan.hold;
        let (mut low, mut high, mut at_end) = (at_start, at_start, at_start);
        while Instant::now() < until {
            session.sleep(watch.period.max(Duration::from_millis(50)))?;
            at_end = session.state(id)?.position;
            (low, high) = (low.min(at_end), high.max(at_end));
        }
        Some(Hold {
            started_unix,
            ended_unix: unix_now(),
            at_start,
            at_end,
            min: low,
            max: high,
        })
    };
    let back = drive_to(session, id, start, watch)?;
    let undone = session.finish_servo(id)?;
    notify(JogEvent::Returned { id, at: back.at });
    Ok(Jog {
        id,
        start,
        target,
        out,
        hold,
        back,
        slow: *slow,
        undone,
    })
}
