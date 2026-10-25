//! The guard every calibration write goes through; see the module header of
//! [`crate::calibrate`].

use std::collections::BTreeSet;
use std::fmt;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use sx_embodiment_drivers::serial::SerialPort;
use sx_feetech_serial::scservo_sdk::{
    CommResult, ProtocolPacketHandler, SMS_STS_ACC, SMS_STS_GOAL_POSITION_L, SMS_STS_GOAL_SPEED_L,
    SMS_STS_LOCK, SMS_STS_MAX_ANGLE_LIMIT_L, SMS_STS_MIN_ANGLE_LIMIT_L, SMS_STS_TORQUE_ENABLE,
};

use super::journal::{Journal, SavedRegister};
use super::read::{self, ServoState};
use super::{CalibrateError, MAX_TICK, TORQUE_LIMIT};

/// How a powered servo is held back, servo-side: the registers [`Session::enable`] writes and
/// reads back before `Torque_Enable` 1.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Slow {
    /// `Goal_Speed` (46); 0 would be the servo's full speed and is refused.
    pub goal_speed: u16,
    /// `Acceleration` (41); 0 would be no limit and is refused.
    pub acceleration: u8,
    /// `Torque_Limit` (48), tenths of a percent.
    pub torque_limit: u16,
}

/// How many times an undo is tried on a servo that does not answer.
const UNDO_ATTEMPTS: usize = 3;
/// The longest one wait goes without checking for a signal or the deadline.
const SLICE: Duration = Duration::from_millis(5);

/// What undoing a run found.
#[derive(Clone, Debug, Default)]
pub struct RestoreReport {
    /// Servos written `Torque_Enable` 0 and read back 0.
    pub disabled: Vec<u8>,
    /// Servos whose torque-off was not proven, and why.
    pub torque_unproven: Vec<(u8, String)>,
    /// Registers written back and read back.
    pub restored: Vec<SavedRegister>,
    /// Registers not restored, and why.
    pub unrestored: Vec<(SavedRegister, String)>,
    /// The journal, when it is kept because something is not undone.
    pub journal_kept: Option<PathBuf>,
    /// A failure to update or remove the journal.
    pub journal_error: Option<String>,
}

impl RestoreReport {
    /// Every servo proven limp and every register restored.
    #[must_use]
    pub fn is_clean(&self) -> bool {
        self.torque_unproven.is_empty()
            && self.unrestored.is_empty()
            && self.journal_error.is_none()
    }
}

impl fmt::Display for RestoreReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if !self.disabled.is_empty() {
            writeln!(f, "torque off, read back 0: servos {:?}", self.disabled)?;
        }
        for saved in &self.restored {
            writeln!(
                f,
                "restored: servo {} register {} = {}",
                saved.id, saved.address, saved.value
            )?;
        }
        for (id, why) in &self.torque_unproven {
            writeln!(
                f,
                "DANGER: servo {id} may still hold torque: its torque-off was not proven ({why})"
            )?;
        }
        for (saved, why) in &self.unrestored {
            writeln!(
                f,
                "NOT RESTORED: servo {} register {} should be {} ({why})",
                saved.id, saved.address, saved.value
            )?;
        }
        if let Some(error) = &self.journal_error {
            writeln!(f, "journal: {error}")?;
        }
        if let Some(path) = &self.journal_kept {
            writeln!(
                f,
                "journal kept at {}; undo with: sx-commission calibrate --restore {}",
                path.display(),
                path.display()
            )?;
        }
        Ok(())
    }
}

/// A calibration run's hold on one bus.
pub struct Session<P: SerialPort> {
    ph: ProtocolPacketHandler<P>,
    journal: Journal,
    /// The one servo powered now, with the limits its goals are kept inside.
    powered: Option<(u8, (u16, u16))>,
    started: Instant,
    budget: Duration,
    stop: Arc<AtomicUsize>,
    closed: bool,
}

impl<P: SerialPort> Session<P> {
    /// A session over `port` recording into `journal`, stopping when `stop` holds a signal
    /// number or once `budget` has passed.
    pub fn new(port: P, journal: Journal, budget: Duration, stop: Arc<AtomicUsize>) -> Self {
        Self {
            ph: ProtocolPacketHandler::new(port, 0),
            journal,
            powered: None,
            started: Instant::now(),
            budget,
            stop,
            closed: false,
        }
    }

    #[must_use]
    pub const fn journal(&self) -> &Journal {
        &self.journal
    }

    /// The one servo powered now.
    #[must_use]
    pub fn powered(&self) -> Option<u8> {
        self.powered.map(|(id, _)| id)
    }

    /// Whether the run may go on.
    ///
    /// # Errors
    ///
    /// [`CalibrateError::Signal`] once a signal arrived, [`CalibrateError::Deadline`] once the
    /// budget is spent.
    pub fn check(&self) -> Result<(), CalibrateError> {
        let signal = self.stop.load(Ordering::SeqCst);
        if signal != 0 {
            return Err(CalibrateError::Signal(signal));
        }
        if self.started.elapsed() >= self.budget {
            return Err(CalibrateError::Deadline(self.budget));
        }
        Ok(())
    }

    /// Wait `duration`, checking ([`Session::check`]) every few milliseconds.
    ///
    /// # Errors
    ///
    /// As [`Session::check`].
    pub fn sleep(&self, duration: Duration) -> Result<(), CalibrateError> {
        let until = Instant::now() + duration;
        loop {
            self.check()?;
            let now = Instant::now();
            if now >= until {
                return Ok(());
            }
            std::thread::sleep(SLICE.min(until - now));
        }
    }

    /// # Errors
    ///
    /// The bus's failure or the servo's error flags.
    pub fn state(&mut self, id: u8) -> Result<ServoState, CalibrateError> {
        read::state(&mut self.ph, id)
    }

    /// # Errors
    ///
    /// The bus's failure or the servo's error flags.
    pub fn limits(&mut self, id: u8) -> Result<(u16, u16), CalibrateError> {
        read::limits(&mut self.ph, id)
    }

    /// # Errors
    ///
    /// The bus's failure or the servo's error flags.
    pub fn homing_offset(&mut self, id: u8) -> Result<i16, CalibrateError> {
        read::homing_offset(&mut self.ph, id)
    }

    fn write(&mut self, id: u8, address: u8, width: u8, value: u16) -> Result<(), CalibrateError> {
        let (result, flags) = if width == 1 {
            self.ph
                .write1_byte_tx_rx(id, address, u8::try_from(value).unwrap_or(u8::MAX))
        } else {
            self.ph.write2_byte_tx_rx(id, address, value)
        };
        if result != CommResult::Success {
            return Err(CalibrateError::Comm {
                id,
                address,
                result,
            });
        }
        if flags != 0 {
            return Err(CalibrateError::Flags { id, address, flags });
        }
        Ok(())
    }

    /// Save `address`'s value to the journal, the first time this run changes it.
    fn save(&mut self, id: u8, address: u8, width: u8) -> Result<(), CalibrateError> {
        let saved = &self.journal.record.saved;
        if saved.iter().any(|s| s.id == id && s.address == address) {
            return Ok(());
        }
        let value = read::register(&mut self.ph, id, address, width)?;
        self.journal.record.saved.push(SavedRegister {
            id,
            address,
            width,
            value,
        });
        self.journal.persist()
    }

    /// Save, write and read back one register.
    fn set(&mut self, id: u8, address: u8, width: u8, value: u16) -> Result<(), CalibrateError> {
        self.save(id, address, width)?;
        self.write(id, address, width, value)?;
        let read = read::register(&mut self.ph, id, address, width)?;
        if read != value {
            return Err(CalibrateError::ReadBack {
                id,
                address,
                wrote: value,
                read,
            });
        }
        Ok(())
    }

    /// Widen servo `id`'s position limits to `min..max` for this run, with its torque off:
    /// `Lock` is set to 1 first, so the servo does not keep them across a power cycle, and
    /// both are journaled and restored as every register is.
    ///
    /// # Errors
    ///
    /// A powered servo, a range that is not one, or a write that does not read back.
    pub fn widen_limits(&mut self, id: u8, min: u16, max: u16) -> Result<(), CalibrateError> {
        self.check()?;
        if self.powered().is_some() {
            return Err(CalibrateError::Refused(format!(
                "limits change only with every servo limp (servo {id})"
            )));
        }
        if min >= max || max > MAX_TICK {
            return Err(CalibrateError::Refused(format!(
                "{min}..{max} is not a position range"
            )));
        }
        self.set(id, SMS_STS_LOCK, 1, 1)?;
        self.set(id, SMS_STS_MIN_ANGLE_LIMIT_L, 2, min)?;
        self.set(id, SMS_STS_MAX_ANGLE_LIMIT_L, 2, max)
    }

    /// Power servo `id` where it rests, held back by `slow`: its goal is written to its present
    /// position, `slow`'s registers are written, and each is read back before `Torque_Enable`
    /// 1, which is read back too. Returns where it was held.
    ///
    /// # Errors
    ///
    /// Another servo powered, a `slow` of 0, a servo resting outside its limits, or any write
    /// that does not read back. A servo whose `Torque_Enable` did not read back stays recorded
    /// as powered, so closing disables it.
    pub fn enable(&mut self, id: u8, slow: &Slow) -> Result<i32, CalibrateError> {
        self.check()?;
        if let Some(powered) = self.powered() {
            return Err(CalibrateError::OnePowered { id, powered });
        }
        if slow.goal_speed == 0 || slow.acceleration == 0 || slow.torque_limit == 0 {
            return Err(CalibrateError::Refused(
                "Goal_Speed, Acceleration and Torque_Limit of 0 are no limit or no torque"
                    .to_owned(),
            ));
        }
        let limits = self.limits(id)?;
        let at = self.state(id)?.position;
        if at < i32::from(limits.0) || at > i32::from(limits.1) {
            return Err(CalibrateError::Refused(format!(
                "servo {id} rests at {at}, outside its limits {}..{}: the firmware would drive \
                 it to the nearest limit the moment torque came on",
                limits.0, limits.1
            )));
        }
        let held = u16::try_from(at).unwrap_or_default();
        // Goal_Position is not journaled: it means nothing once torque is off.
        self.write(id, SMS_STS_GOAL_POSITION_L, 2, held)?;
        self.set(id, SMS_STS_ACC, 1, u16::from(slow.acceleration))?;
        self.set(id, SMS_STS_GOAL_SPEED_L, 2, slow.goal_speed)?;
        self.set(id, TORQUE_LIMIT, 2, slow.torque_limit)?;
        let control = read::control(&mut self.ph, id)?;
        let expected = (held, slow.acceleration, slow.goal_speed, slow.torque_limit);
        let found = (
            control.goal_position,
            control.acceleration,
            control.goal_speed,
            control.torque_limit,
        );
        if found != expected {
            return Err(CalibrateError::Refused(format!(
                "servo {id} holds goal, acceleration, speed, torque limit {found:?}, not \
                 {expected:?}"
            )));
        }
        self.check()?;
        if !self.journal.record.powered.contains(&id) {
            self.journal.record.powered.push(id);
        }
        self.journal.persist()?;
        self.powered = Some((id, limits));
        self.write(id, SMS_STS_TORQUE_ENABLE, 1, 1)?;
        let torque = read::register(&mut self.ph, id, SMS_STS_TORQUE_ENABLE, 1)?;
        if torque != 1 {
            return Err(CalibrateError::ReadBack {
                id,
                address: SMS_STS_TORQUE_ENABLE,
                wrote: 1,
                read: torque,
            });
        }
        Ok(at)
    }

    /// The powered servo's limits.
    ///
    /// # Errors
    ///
    /// [`CalibrateError::NotPowered`] when `id` is not the powered servo.
    pub fn powered_limits(&self, id: u8) -> Result<(u16, u16), CalibrateError> {
        match self.powered {
            Some((powered, limits)) if powered == id => Ok(limits),
            _ => Err(CalibrateError::NotPowered(id)),
        }
    }

    /// Send the powered servo `id` toward `goal` ticks, inside its limits.
    ///
    /// # Errors
    ///
    /// A signal or the deadline, a servo not powered, a goal outside its limits, or the bus.
    pub fn goal(&mut self, id: u8, goal: i32) -> Result<(), CalibrateError> {
        self.check()?;
        let (min, max) = self.powered_limits(id)?;
        if goal < i32::from(min) || goal > i32::from(max) {
            return Err(CalibrateError::OutsideLimits { id, goal, min, max });
        }
        self.write(
            id,
            SMS_STS_GOAL_POSITION_L,
            2,
            u16::try_from(goal).unwrap_or_default(),
        )
    }

    /// Torque off on `id`, read back; ignores error flags, which must not keep a servo
    /// powered, and tries again on a servo that does not answer.
    fn torque_off(&mut self, id: u8) -> Result<(), String> {
        let mut why = String::new();
        for _ in 0..UNDO_ATTEMPTS {
            let (result, _) = self.ph.write1_byte_tx_rx(id, SMS_STS_TORQUE_ENABLE, 0);
            if result != CommResult::Success {
                why = format!("write: {result:?}");
                continue;
            }
            let (value, result, _) = self.ph.read1_byte_tx_rx(id, SMS_STS_TORQUE_ENABLE);
            match (result, value) {
                (CommResult::Success, 0) => return Ok(()),
                (CommResult::Success, value) => why = format!("reads {value}"),
                (result, _) => why = format!("read-back: {result:?}"),
            }
        }
        Err(why)
    }

    /// Write one saved register back and read it back.
    fn put_back(&mut self, saved: SavedRegister) -> Result<(), String> {
        let mut why = String::new();
        for _ in 0..UNDO_ATTEMPTS {
            let (result, _) = if saved.width == 1 {
                self.ph.write1_byte_tx_rx(
                    saved.id,
                    saved.address,
                    u8::try_from(saved.value).unwrap_or(u8::MAX),
                )
            } else {
                self.ph
                    .write2_byte_tx_rx(saved.id, saved.address, saved.value)
            };
            if result != CommResult::Success {
                why = format!("write: {result:?}");
                continue;
            }
            let (data, result, _) = self.ph.read_tx_rx(saved.id, saved.address, saved.width);
            let value = match data.as_slice() {
                [low] => Some(u16::from(*low)),
                [low, high] => Some(u16::from_le_bytes([*low, *high])),
                _ => None,
            };
            match (result, value) {
                (CommResult::Success, Some(value)) if value == saved.value => return Ok(()),
                (CommResult::Success, value) => why = format!("reads {value:?}"),
                (result, _) => why = format!("read-back: {result:?}"),
            }
        }
        Err(why)
    }

    /// Undo everything the run did to the servos in `ids` (all of them for `None`): torque off
    /// and read back, then each saved register written back in reverse order and read back,
    /// skipping the registers of a servo whose torque-off was not proven.
    fn undo(&mut self, ids: Option<&[u8]>, report: &mut RestoreReport) {
        let wanted = |id: &u8| ids.is_none_or(|ids| ids.contains(id));
        let record = &self.journal.record;
        let touched: BTreeSet<u8> = record
            .powered
            .iter()
            .copied()
            .chain(record.saved.iter().map(|s| s.id))
            .filter(wanted)
            .collect();
        let mut limp = BTreeSet::new();
        for id in touched {
            match self.torque_off(id) {
                Ok(()) => {
                    limp.insert(id);
                    report.disabled.push(id);
                    if self.powered() == Some(id) {
                        self.powered = None;
                    }
                }
                Err(why) => report.torque_unproven.push((id, why)),
            }
        }
        self.journal.record.powered.retain(|id| !limp.contains(id));
        let saved: Vec<SavedRegister> = self.journal.record.saved.clone();
        for register in saved.into_iter().rev().filter(|s| wanted(&s.id)) {
            if !limp.contains(&register.id) {
                report
                    .unrestored
                    .push((register, "torque not proven off".to_owned()));
                continue;
            }
            match self.put_back(register) {
                Ok(()) => {
                    self.journal.record.saved.retain(|s| *s != register);
                    report.restored.push(register);
                }
                Err(why) => report.unrestored.push((register, why)),
            }
        }
        if let Err(error) = self.journal.persist() {
            report.journal_error = Some(error.to_string());
        }
    }

    /// Undo what the run did to servo `id` alone, before the run moves on to another.
    ///
    /// # Errors
    ///
    /// [`CalibrateError::Refused`] naming what was not undone; closing tries again.
    pub fn finish_servo(&mut self, id: u8) -> Result<RestoreReport, CalibrateError> {
        let mut report = RestoreReport::default();
        self.undo(Some(&[id]), &mut report);
        if report.is_clean() {
            Ok(report)
        } else {
            Err(CalibrateError::Refused(format!(
                "servo {id} was not fully undone:\n{report}"
            )))
        }
    }

    fn restore_all(&mut self) -> RestoreReport {
        self.closed = true;
        let mut report = RestoreReport::default();
        self.undo(None, &mut report);
        let record = &self.journal.record;
        if report.is_clean() && record.powered.is_empty() && record.saved.is_empty() {
            if let Err(error) = self.journal.discard() {
                report.journal_error = Some(error.to_string());
                report.journal_kept = Some(self.journal.path().to_owned());
            }
        } else {
            report.journal_kept = Some(self.journal.path().to_owned());
        }
        report
    }

    /// Undo everything this run did, prove it, and remove the journal when nothing is left.
    #[must_use]
    pub fn close(mut self) -> RestoreReport {
        self.restore_all()
    }
}

impl<P: SerialPort> Drop for Session<P> {
    /// The way out a [`Session::close`] does not reach: a panic's unwinding, or a caller that
    /// forgot. The report goes to standard error.
    fn drop(&mut self) {
        if !self.closed {
            let report = self.restore_all();
            eprint!("calibration session dropped without closing; undone:\n{report}");
        }
    }
}
