//! A chain of bound Damiao motors as one `ActuatorChain`, driven through [`crate::dm_can`].
//!
//! Every bus operation is a `DM_CAN.py` call; the order they are called in is this chain's.
//! Open follows the vendor demo's bring-up for each motor (`DM_Motor_Test.py:14-26`, same
//! commit: `switchControlMode`, then `read_motor_param` of `PMAX`, `VMAX` and `TMAX`, which the
//! demo prints), then `disable`s it. Two deviations from the demo: it switches to `POS_VEL`
//! where the chain switches to `MIT`, and it refuses nothing where the chain refuses a register
//! value (below). Command `enable`s each motor when the chain is disabled, then sends one `controlMIT` per
//! motor holding its target with zero velocity and feed-forward torque. Stop `disable`s every
//! motor.
//!
//! What this chain adds over the vendor's calls, and why:
//!
//! - It refuses before any frame: a target outside the axis's commandable range (the declared
//!   bounds intersected with the motor's configured range), ids or gains the protocol cannot
//!   carry, a model or bus this crate is not qualified for. `DM_CAN.py` clamps silently.
//! - It reads each motor's `PMAX`, `VMAX` and `TMAX` registers at open and scales that motor's
//!   frames by them, refusing values no Damiao table carries for its model, since a wrong row
//!   rescales every position, velocity and torque on the wire. Damiao's own tables disagree on
//!   the DM4340's velocity range — 10 rad/s in `DM_CAN.py`, 8 rad/s in its C++ SDK
//!   (`C++例程/u2can/include/damiao.h`, same commit) — so either is accepted for that model, and
//!   [`DamiaoChain::limits`] records which the motor reported. The vendor's own answer to a
//!   motor on another table is `DM_CAN.py`'s `change_limit_param`, which rewrites the row every
//!   motor of that model shares; this crate rescales the one motor instead
//!   ([`crate::dm_can::MotorControl::change_limit_param`]).
//! - After the vendor's own read it waits briefly for each motor's answer, and judges it by the
//!   feedback's first-byte high nibble ([`MotorState`]). `DM_CAN.py` stores the nibble unread.
//! - A stop is proven motor by motor: an answer to the disable frames reporting disabled. Answers
//!   already on the link when the stop begins are drained first, so a late answer to an earlier
//!   command never counts as the disable's. A silent motor is asked once more with
//!   `refresh_motor_status`. Where firmware does not report its enable state
//!   ([`DamiaoAxis::reports_enable_state`]), a fault-free answer to the disable proves the stop
//!   only weakly, and the motor is named in [`StopReport::unobserved`]. After an unproven stop
//!   every command is refused until a stop is proven.
//! - **Communication timeout.** An axis that carries a `can_timeout` has it written to the
//!   motor's `TIMEOUT` register (0x09) at open and again before every enable (a motor
//!   power-cycled since comes back with whatever its flash holds), in RAM and never saved to
//!   flash, then confirmed by the motor's echo and by reading the register back. No motor is
//!   enabled until every motor confirmed. Damiao's "DM-J4340-2EC User Manual
//!   V1.1" (2026-09-08; gitee `kit-miao/DM-J4340-2EC` at `c1d567db`, `说明书/`), "Register Map"
//!   (p. 20: `0x09 TIMEOUT`, uint32) and "Parameter Management" (p. 30: "Time base: 50 μs per
//!   count. Only effective when the motor is enabled."): an enabled motor that hears no command
//!   for that long leaves enable mode and reports fault 0xD, communication lost. "Write
//!   Parameters" (p. 18): a write "takes effect immediately" and is "lost upon power-off" unless
//!   saved. This crate never sends the save. A motor that tripped keeps reporting 0xD until it
//!   is power-cycled or sent Damiao's clear frame (`FF..FB`, p. 17), which `DM_CAN.py` does not
//!   carry: its stop stays unproven. Open's settle step sends the clear frame
//!   ([`DamiaoChain::clear_fault`]) once to each motor answering 0xD while disabled, then asks
//!   again, and still refuses a motor whose fault persists. No other fault is cleared at open.
//! - **One turn.** Damiao's joint motors report the output shaft's position from a single-turn
//!   absolute encoder (same manual, "Motor Features", p. 5), so after power-up a joint sitting
//!   more than half a turn from zero reads as its alias within one turn. Where an axis's motor
//!   range spans more than one turn, its commandable range is cut to the motor's single-turn
//!   window, -pi..pi in actuator coordinates, so this chain never drives a joint where its
//!   position would read ambiguously after a power cycle. Where the range spans less than a
//!   turn, every alias of a position inside it lies outside it. Either way open refuses a motor
//!   whose disabled answer lies more than [`REST_TOLERANCE`] outside its axis's commandable
//!   range, the motor's own reading of an aliased or out-of-range joint. A joint turned by hand
//!   past the window while unpowered, onto an alias inside it, cannot be told apart.

use std::time::{Duration, Instant};

use sx_embodiment_drivers::can::CanPort;
use sx_embodiment_drivers::{
    ActuatorBus, ActuatorChain, ActuatorModel, AxisReading, BindingError, BodyError, BoundAxis,
    ChainError, ChainFailure, CommandRange, Evidence, MissingReason, Receipt, StopReport,
    TargetError, check_targets, validate_chain,
};
use thiserror::Error;

use crate::dm_can::{
    ControlType, DISABLE_WAIT, DmCanError, DmMotorType, DmVariable, Motor, MotorControl, ParamValue,
};

/// `ActuatorModel.DAMIAO_DM4310` in the registry.
pub const DAMIAO_DM4310: ActuatorModel = ActuatorModel::new("damiao_dm4310");
/// `ActuatorModel.DAMIAO_DM4340` in the registry.
pub const DAMIAO_DM4340: ActuatorModel = ActuatorModel::new("damiao_dm4340");
/// `ActuatorModel.DAMIAO_DM8009` in the registry.
pub const DAMIAO_DM8009: ActuatorModel = ActuatorModel::new("damiao_dm8009");
/// `ActuatorBus.DAMIAO_CAN` in the registry.
pub const DAMIAO_CAN: ActuatorBus = ActuatorBus::new("damiao_can");

const KP_MAX: f64 = 500.0;
const KD_MAX: f64 = 5.0;
/// `0x7FF` addresses register requests; no motor may sit on it.
const MAX_MOTOR_ID: u16 = 0x7FE;
const ANSWER_POLLS: usize = 20;
const ANSWER_POLL_INTERVAL: Duration = Duration::from_millis(1);
/// How far outside its commandable range, in actuator radians, a motor may rest at open: a
/// joint against its hard stop reads a little past the limit the stop sits at.
pub const REST_TOLERANCE: f64 = 0.05;
/// A range of exactly one turn, as the humanoid's shoulder pitches declare to the URDF's
/// rounding, is not cut: it aliases only at its two ends.
const TURN_SLACK: f64 = 1e-6;
/// The `TIMEOUT` register's time base: 50 µs per count.
const TIMEOUT_COUNT: Duration = Duration::from_micros(50);
/// The DM4340 velocity ranges Damiao publishes: `DM_CAN.py`'s row, then `damiao.h`'s; and 20,
/// which the static upper body's upper-arm yaw and elbow motors (0x1A, 0x1B, 0x11, 0x12) report
/// (read 2026-10-25). `VMAX` only scales the velocity field of MIT frames and feedback, and each
/// motor is scaled by the value it reports.
/// The feedback fault code for communication lost: the motor's own `TIMEOUT` watchdog tripped.
/// The one fault open clears by itself.
pub const COMMUNICATION_LOST: u8 = 0xD;
const DM4340_VMAX: [f64; 3] = [10.0, 8.0, 20.0];
/// `PMAX` values reported beside a model's table row: 12.57 (≈ 4π) on the static upper body's
/// right shoulder roll (0x10, read 2026-10-25). Positions are scaled by the value reported.
const EXTRA_PMAX: [f64; 1] = [12.57];

/// The `DM_CAN.py` limit row a qualified registry model is driven with.
#[must_use]
pub fn motor_type(model: ActuatorModel) -> Option<DmMotorType> {
    if model == DAMIAO_DM4310 {
        Some(DmMotorType::DM4310)
    } else if model == DAMIAO_DM4340 {
        Some(DmMotorType::DM4340)
    } else if model == DAMIAO_DM8009 {
        Some(DmMotorType::DM8009)
    } else {
        None
    }
}

/// The `VMAX` values a Damiao table carries for `model`.
fn accepted_vmax(model: DmMotorType) -> Vec<f64> {
    match model {
        DmMotorType::DM4340 => DM4340_VMAX.to_vec(),
        other => vec![other.limit_param()[1]],
    }
}

/// One bound axis with what its Damiao motor needs beyond the registry's binding.
#[derive(Clone, Debug, PartialEq)]
pub struct DamiaoAxis {
    pub axis: BoundAxis,
    /// The id the motor answers on (`MasterID`).
    pub master_id: u16,
    /// MIT position gain (0..=500) and damping (0..=5).
    pub kp: f64,
    pub kd: f64,
    /// The motor's own commandable range, radians, inside its model's `Q_MAX`.
    pub motor_lower: f64,
    pub motor_upper: f64,
    /// How long the enabled motor may hear no command before it disables itself, written to
    /// its `TIMEOUT` register at open; `None` leaves the register as the motor holds it.
    pub can_timeout: Option<Duration>,
    /// Whether the motor's feedback state field reports enable and disable. Firmware on the
    /// static upper body's motors answers state 1 whether enabled or not (verified 2026-10-25:
    /// a wrist motor held at kp 5 and released on disable, LED green then red, while the field
    /// read 1 throughout). Where it does not, a fresh answer with no fault code is accepted for
    /// either state, and stop is proven by that answer to the disable.
    pub reports_enable_state: bool,
}

impl DamiaoAxis {
    /// The motor range this axis is driven within, radians: the configured range, cut to the
    /// motor's single-turn window when it spans more than one turn.
    #[must_use]
    pub fn motor_window(&self) -> (f64, f64) {
        let turn = std::f64::consts::TAU;
        if self.motor_upper - self.motor_lower > turn + TURN_SLACK {
            let half = std::f64::consts::PI;
            (self.motor_lower.max(-half), self.motor_upper.min(half))
        } else {
            (self.motor_lower, self.motor_upper)
        }
    }

    /// The joint range this axis is commanded within: the declared bounds intersected with the
    /// motor's window ([`DamiaoAxis::motor_window`]) mapped back through the binding.
    #[must_use]
    pub fn commandable(&self) -> CommandRange {
        let (motor_lower, motor_upper) = self.motor_window();
        let ends = [
            self.axis.joint_from_actuator(motor_lower),
            self.axis.joint_from_actuator(motor_upper),
        ];
        CommandRange {
            lower: self.axis.lower.max(ends[0].min(ends[1])),
            upper: self.axis.upper.min(ends[0].max(ends[1])),
        }
    }
}

/// The state a feedback frame's first-byte high nibble reports.
///
/// The codes are Damiao's: "DM-J4310-2EC V1.2 Geared Motor User Manual V1.4 2026-09-14" and
/// "DM-J4340P-2EC V1.1 Geared Motor User Manual V1.4 2026-09-14", chapter "CAN Communication",
/// subsection "Feedback Frames" (p. 14-15; gitee `kit-miao/DM-J4310-2EC` at `dc15860d`,
/// `kit-miao/DM-J4340P-2EC` at `0f93050a`, `说明书/`), and `status_name` in
/// `motorbridge/motorbridge` at `e8b3ac66f73a4a5a82e79c48a1707935ca4c8b96`,
/// `motor_vendors/damiao/src/protocol.rs:29-41`, the driver Seeed's controller runs: 0 disabled
/// (the power-on state), 1 enabled, 8 over-voltage, 9 under-voltage, 0xA over-current, 0xB MOSFET
/// over-temperature, 0xC motor coil over-temperature, 0xD communication lost, 0xE overload. The
/// Chinese editions of both manuals (subsection "反馈帧") also list 3 output-shaft calibration
/// error, 4 sensor output error and 5 motor encoder calibration error. Every code other than 0
/// and 1 is a [`MotorState::Fault`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotorState {
    Disabled,
    Enabled,
    Fault(u8),
}

impl MotorState {
    #[must_use]
    pub const fn from_err(err: u8) -> Self {
        match err {
            0 => Self::Disabled,
            1 => Self::Enabled,
            code => Self::Fault(code),
        }
    }
}

/// One motor's latest answer, in the motor's own units (rad, rad/s, N·m, °C).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorFeedback {
    pub bus_id: u16,
    pub state: MotorState,
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub mos_celsius: u8,
    pub rotor_celsius: u8,
}

/// The limit row a motor reported at open, and its frames are scaled by.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorLimits {
    pub bus_id: u16,
    pub p_max: f32,
    pub v_max: f32,
    pub t_max: f32,
}

#[derive(Debug, Error)]
pub enum DamiaoChainError {
    #[error(transparent)]
    Binding(#[from] BindingError),
    #[error(transparent)]
    Body(#[from] BodyError),
    #[error(transparent)]
    Target(#[from] TargetError),
    #[error(
        "bus_id {0} is not on the damiao_can bus or binds a model this crate is not qualified for"
    )]
    Unqualified(u16),
    #[error("bus_id {0} or its master id is 0, 0x7FF or above, or shared with another motor")]
    Ids(u16),
    #[error("bus_id {0} has MIT gains outside kp 0..=500, kd 0..=5")]
    Gains(u16),
    #[error("bus_id {0} has a motor range that is not finite lower < upper inside its Q_MAX")]
    MotorRange(u16),
    #[error("joint {joint} target {value} is outside its commandable range")]
    Uncommandable { joint: String, value: f64 },
    #[error("bus_id {0} did not confirm MIT control mode")]
    ModeNotConfirmed(u16),
    #[error(
        "{refused}; and the disable of bus_ids {unsent:?} could not be sent, so they may hold torque"
    )]
    DisableUnsent {
        refused: Box<DamiaoChainError>,
        unsent: Vec<u16>,
    },
    #[error(
        "bus_id {bus_id} reports {register:?} = {motor:?}, which no Damiao table carries for its model"
    )]
    LimitMismatch {
        bus_id: u16,
        register: DmVariable,
        motor: Option<ParamValue>,
    },
    #[error("bus_id {0} did not answer")]
    Silent(u16),
    #[error("bus_id {bus_id} reports {state:?} where {expected:?} was required")]
    UnexpectedState {
        bus_id: u16,
        state: MotorState,
        expected: MotorState,
    },
    #[error("an earlier stop is unproven; motion is refused until a stop is proven")]
    StopPending,
    #[error("bus_id {bus_id} still reports fault {code:#x} after its fault was cleared")]
    FaultPersists { bus_id: u16, code: u8 },
    #[error("a fault is cleared only while the chain is disabled")]
    ClearWhileEnabled,
    #[error("bus_id {0} has a CAN timeout its TIMEOUT register cannot hold")]
    TimeoutRange(u16),
    #[error("bus_id {bus_id} did not confirm its TIMEOUT register at {counts} counts of 50 µs")]
    TimeoutNotConfirmed { bus_id: u16, counts: u32 },
    #[error(
        "bus_id {bus_id} rests at {position} rad, outside its commandable motor range {lower}..{upper} rad; move the joint into range, or check for a joint more than half a turn from zero"
    )]
    RestingOutside {
        bus_id: u16,
        position: f64,
        lower: f64,
        upper: f64,
    },
    #[error(transparent)]
    DmCan(#[from] DmCanError),
}

/// Bound Damiao motors on one CAN link, in native state order.
pub struct DamiaoChain<P> {
    control: MotorControl<P>,
    motors: Vec<DamiaoAxis>,
    axes: Vec<BoundAxis>,
    commandable: Vec<CommandRange>,
    limits: Vec<MotorLimits>,
    enabled: bool,
    stop_pending: bool,
}

/// Bring each motor up as the vendor does and read its limit registers; see
/// [`DamiaoChain::open`].
// A register holds the exact float32 of a table literal; equality is the comparison meant.
#[allow(clippy::float_cmp)]
fn bring_up<P: CanPort>(
    control: &mut MotorControl<P>,
    motors: &[DamiaoAxis],
) -> Result<Vec<MotorLimits>, DamiaoChainError> {
    let mut limits = Vec::with_capacity(motors.len());
    for motor in motors {
        let bus_id = motor.axis.binding.bus_id;
        if !control.switch_control_mode(bus_id, ControlType::MIT)? {
            return Err(DamiaoChainError::ModeNotConfirmed(bus_id));
        }
        let model = control.motor(bus_id)?.motor_type;
        let row = model.limit_param();
        let mut found = [0.0_f32; 3];
        for (index, register) in [DmVariable::PMAX, DmVariable::VMAX, DmVariable::TMAX]
            .into_iter()
            .enumerate()
        {
            let accepted = match register {
                DmVariable::VMAX => accepted_vmax(model),
                DmVariable::PMAX => std::iter::once(row[index]).chain(EXTRA_PMAX).collect(),
                _ => vec![row[index]],
            };
            let value = control.read_motor_param(bus_id, register)?;
            match value {
                Some(ParamValue::Float(reported))
                    if accepted.iter().any(|table| reported == narrow(*table)) =>
                {
                    found[index] = reported;
                }
                _ => {
                    return Err(DamiaoChainError::LimitMismatch {
                        bus_id,
                        register,
                        motor: value,
                    });
                }
            }
        }
        let reported = found.map(f64::from);
        if reported != row.map(|table| f64::from(narrow(table))) {
            control.change_limit_param(bus_id, reported[0], reported[1], reported[2])?;
        }
        limits.push(MotorLimits {
            bus_id,
            p_max: found[0],
            v_max: found[1],
            t_max: found[2],
        });
        if let Some(timeout) = motor.can_timeout {
            let counts = timeout_counts(timeout).ok_or(DamiaoChainError::TimeoutRange(bus_id))?;
            write_timeout(control, bus_id, counts)?;
        }
    }
    Ok(limits)
}

/// `timeout` in the `TIMEOUT` register's counts of 50 µs, when it is at least one count and
/// the register can hold it.
fn timeout_counts(timeout: Duration) -> Option<u32> {
    u32::try_from(timeout.as_micros() / TIMEOUT_COUNT.as_micros())
        .ok()
        .filter(|counts| *counts > 0)
}

/// Write `counts` to the motor's `TIMEOUT` register in RAM, then require the motor's echo and
/// a fresh read of the register to carry it.
fn write_timeout<P: CanPort>(
    control: &mut MotorControl<P>,
    bus_id: u16,
    counts: u32,
) -> Result<(), DamiaoChainError> {
    let refused = DamiaoChainError::TimeoutNotConfirmed { bus_id, counts };
    control.forget_param(bus_id, DmVariable::TIMEOUT)?;
    if !control.change_motor_param(bus_id, DmVariable::TIMEOUT, f64::from(counts))? {
        return Err(refused);
    }
    control.forget_param(bus_id, DmVariable::TIMEOUT)?;
    if control.read_motor_param(bus_id, DmVariable::TIMEOUT)? != Some(ParamValue::Uint32(counts)) {
        return Err(refused);
    }
    Ok(())
}

/// Send every named motor its disable frame, whatever any answered: the last word of an open
/// that failed, returned as `refused` unless a disable could not be sent, which is then named
/// beside it, since that motor may still hold torque.
fn disable_all<P: CanPort>(
    control: &mut MotorControl<P>,
    motors: &[DamiaoAxis],
    refused: DamiaoChainError,
) -> DamiaoChainError {
    let unsent: Vec<u16> = motors
        .iter()
        .map(|motor| motor.axis.binding.bus_id)
        .filter(|bus_id| control.disable(*bus_id).is_err())
        .collect();
    if unsent.is_empty() {
        refused
    } else {
        DamiaoChainError::DisableUnsent {
            refused: Box::new(refused),
            unsent,
        }
    }
}

impl<P: CanPort> DamiaoChain<P> {
    /// Validate every axis, bring each motor up as the vendor does, scale each by its own limit
    /// registers, and leave the chain disabled with every motor's disabled answer in hand.
    ///
    /// # Errors
    ///
    /// Refuses an invalid or unqualified axis, a motor that does not confirm MIT mode or whose
    /// limit registers no Damiao table carries, and a motor that does not answer disabled.
    // A register holds the exact float32 of a table literal; equality is the comparison meant.
    #[allow(clippy::float_cmp)]
    pub fn open(port: P, motors: Vec<DamiaoAxis>) -> Result<Self, DamiaoChainError> {
        let axes: Vec<BoundAxis> = motors.iter().map(|motor| motor.axis.clone()).collect();
        validate_chain(&axes)?;
        let mut ids: Vec<u16> = Vec::new();
        for motor in &motors {
            validate(motor, &mut ids)?;
        }
        let mut control = MotorControl::new(port);
        for motor in &motors {
            let binding = motor.axis.binding;
            let model =
                motor_type(binding.model).ok_or(DamiaoChainError::Unqualified(binding.bus_id))?;
            control.add_motor(Motor::new(model, binding.bus_id, motor.master_id));
        }
        // Disable every motor before anything is written. A motor left enabled by an earlier
        // run (which firmware that does not report its enable state hides) would otherwise
        // start counting the communication timeout the moment bring-up writes it, and trip
        // (fault 0xD) while the rest are still being brought up (seen 2026-10-25 on the static
        // upper body: bring-up of 14 motors takes over 2 s against a 0.75 s timeout).
        for motor in &motors {
            control.disable(motor.axis.binding.bus_id)?;
        }
        // A motor brought up before a later one failed may hold MIT mode with torque enabled
        // (a motor left enabled by a crashed run answers its mode switch as it is): every motor
        // the chain names is disabled before the failure is returned.
        let limits = match bring_up(&mut control, &motors) {
            Ok(limits) => limits,
            Err(error) => return Err(disable_all(&mut control, &motors, error)),
        };
        let commandable = motors.iter().map(DamiaoAxis::commandable).collect();
        let mut chain = Self {
            control,
            motors,
            axes,
            commandable,
            limits,
            enabled: false,
            stop_pending: false,
        };
        if let Err(error) = chain.settle_disabled() {
            return Err(disable_all(&mut chain.control, &chain.motors, error));
        }
        chain.check_resting()?;
        Ok(chain)
    }

    /// Refuse a motor whose disabled answer lies outside its commandable range by more than
    /// [`REST_TOLERANCE`], in actuator coordinates.
    fn check_resting(&self) -> Result<(), DamiaoChainError> {
        for (motor, range) in self.motors.iter().zip(&self.commandable) {
            let bus_id = motor.axis.binding.bus_id;
            let position = f64::from(self.control.motor(bus_id)?.get_position());
            let ends = [
                motor.axis.actuator_from_joint(range.lower),
                motor.axis.actuator_from_joint(range.upper),
            ];
            let (lower, upper) = (ends[0].min(ends[1]), ends[0].max(ends[1]));
            if !(lower - REST_TOLERANCE..=upper + REST_TOLERANCE).contains(&position) {
                return Err(DamiaoChainError::RestingOutside {
                    bus_id,
                    position,
                    lower,
                    upper,
                });
            }
        }
        Ok(())
    }

    /// Disable every motor and hold each one's disabled answer. A motor answering
    /// [`COMMUNICATION_LOST`] is cleared once ([`DamiaoChain::clear_fault`]) and asked again.
    fn settle_disabled(&mut self) -> Result<(), DamiaoChainError> {
        self.control.recv()?;
        let before = self.feedback_counts()?;
        for index in 0..self.motors.len() {
            self.control.disable(self.bus_id(index))?;
        }
        self.await_answers(&before)?;
        for (index, fresh) in self.fresh(&before)?.into_iter().enumerate() {
            let bus_id = self.bus_id(index);
            let state = MotorState::from_err(self.control.motor(bus_id)?.get_error());
            if fresh && state == MotorState::Fault(COMMUNICATION_LOST) {
                self.clear_fault(bus_id)?;
            }
        }
        self.expect_all(&before, MotorState::Disabled)
    }

    /// Send one motor Damiao's clear-error frame (`FF..FB`) after its disable, then ask for its
    /// state, while the chain is disabled.
    ///
    /// # Errors
    ///
    /// Refuses while the chain is enabled; returns a motor that stays silent, or whose answer
    /// still carries a fault, as [`DamiaoChainError::FaultPersists`].
    pub fn clear_fault(&mut self, bus_id: u16) -> Result<(), DamiaoChainError> {
        if self.enabled {
            return Err(DamiaoChainError::ClearWhileEnabled);
        }
        self.control.disable(bus_id)?;
        self.control.clear_error(bus_id)?;
        self.control.recv()?;
        let before = self.control.motor(bus_id)?.feedback_count;
        for attempt in 0..ANSWER_POLLS {
            if attempt == 0 {
                self.control.refresh_motor_status(bus_id)?;
            } else {
                self.control.port_mut().sleep(ANSWER_POLL_INTERVAL);
                self.control.recv()?;
            }
            if self.control.motor(bus_id)?.feedback_count > before {
                break;
            }
        }
        let motor = self.control.motor(bus_id)?;
        if motor.feedback_count == before {
            return Err(DamiaoChainError::Silent(bus_id));
        }
        match MotorState::from_err(motor.get_error()) {
            MotorState::Fault(code) => Err(DamiaoChainError::FaultPersists { bus_id, code }),
            MotorState::Disabled | MotorState::Enabled => Ok(()),
        }
    }

    /// The limit row each motor reported at open, in native state order.
    #[must_use]
    pub fn limits(&self) -> &[MotorLimits] {
        &self.limits
    }

    /// Each motor's latest answer, as cached from its last feedback frame.
    ///
    /// # Errors
    ///
    /// Never for a chain that opened; the lookup is by the chain's own ids.
    pub fn feedback(&self) -> Result<Vec<MotorFeedback>, DamiaoChainError> {
        self.motors
            .iter()
            .map(|motor| {
                let bus_id = motor.axis.binding.bus_id;
                let cached = self.control.motor(bus_id)?;
                Ok(MotorFeedback {
                    bus_id,
                    state: MotorState::from_err(cached.get_error()),
                    position: cached.get_position(),
                    velocity: cached.get_velocity(),
                    torque: cached.get_torque(),
                    mos_celsius: cached.state_t_mos,
                    rotor_celsius: cached.state_t_rotor,
                })
            })
            .collect()
    }

    #[must_use]
    pub const fn port(&self) -> &P {
        self.control.port()
    }

    fn bus_id(&self, index: usize) -> u16 {
        self.motors[index].axis.binding.bus_id
    }

    fn feedback_counts(&self) -> Result<Vec<u64>, DmCanError> {
        self.motors
            .iter()
            .map(|motor| {
                Ok(self
                    .control
                    .motor(motor.axis.binding.bus_id)?
                    .feedback_count)
            })
            .collect()
    }

    fn fresh(&self, before: &[u64]) -> Result<Vec<bool>, DmCanError> {
        Ok(self
            .feedback_counts()?
            .iter()
            .zip(before)
            .map(|(now, then)| now > then)
            .collect())
    }

    /// Read until every motor has answered since `before`, polling a few milliseconds at most.
    fn await_answers(&mut self, before: &[u64]) -> Result<(), DmCanError> {
        for attempt in 0..ANSWER_POLLS {
            if self.fresh(before)?.iter().all(|fresh| *fresh) {
                return Ok(());
            }
            if attempt > 0 {
                self.control.port_mut().sleep(ANSWER_POLL_INTERVAL);
            }
            self.control.recv()?;
        }
        Ok(())
    }

    /// Stamp `now` on every motor first seen answering since `before`.
    fn stamp_answers(
        &self,
        before: &[u64],
        received: &mut [Option<Instant>],
    ) -> Result<(), DmCanError> {
        let now = Instant::now();
        for (stamp, fresh) in received.iter_mut().zip(self.fresh(before)?) {
            if fresh && stamp.is_none() {
                *stamp = Some(now);
            }
        }
        Ok(())
    }

    fn expect_all(&self, before: &[u64], expected: MotorState) -> Result<(), DamiaoChainError> {
        for (index, fresh) in self.fresh(before)?.into_iter().enumerate() {
            let bus_id = self.bus_id(index);
            if !fresh {
                return Err(DamiaoChainError::Silent(bus_id));
            }
            let state = MotorState::from_err(self.control.motor(bus_id)?.get_error());
            let unreported = !self.motors[index].reports_enable_state
                && matches!(state, MotorState::Enabled | MotorState::Disabled);
            if state != expected && !unreported {
                return Err(DamiaoChainError::UnexpectedState {
                    bus_id,
                    state,
                    expected,
                });
            }
        }
        Ok(())
    }
}

#[allow(clippy::cast_possible_truncation)]
fn narrow(value: f64) -> f32 {
    value as f32
}

fn validate(motor: &DamiaoAxis, ids: &mut Vec<u16>) -> Result<(), DamiaoChainError> {
    let binding = motor.axis.binding;
    let bus_id = binding.bus_id;
    let Some(model) = motor_type(binding.model).filter(|_| binding.bus == DAMIAO_CAN) else {
        return Err(DamiaoChainError::Unqualified(bus_id));
    };
    for id in [bus_id, motor.master_id] {
        if id == 0 || id > MAX_MOTOR_ID || ids.contains(&id) {
            return Err(DamiaoChainError::Ids(bus_id));
        }
        ids.push(id);
    }
    if !((0.0..=KP_MAX).contains(&motor.kp) && (0.0..=KD_MAX).contains(&motor.kd)) {
        return Err(DamiaoChainError::Gains(bus_id));
    }
    if motor
        .can_timeout
        .is_some_and(|timeout| timeout_counts(timeout).is_none())
    {
        return Err(DamiaoChainError::TimeoutRange(bus_id));
    }
    let q_max = model.limit_param()[0];
    let range = motor.commandable();
    if !(motor.motor_lower.is_finite()
        && motor.motor_upper.is_finite()
        && -q_max <= motor.motor_lower
        && motor.motor_lower < motor.motor_upper
        && motor.motor_upper <= q_max
        && range.lower < range.upper)
    {
        return Err(DamiaoChainError::MotorRange(bus_id));
    }
    Ok(())
}

impl<P: CanPort> ActuatorChain for DamiaoChain<P> {
    type Error = DamiaoChainError;
    type Feedback = MotorFeedback;

    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    fn commandable(&self) -> &[CommandRange] {
        &self.commandable
    }

    /// Drain older answers, take each motor's feedback count, request status with the
    /// vendor's `0xCC` operation, and read each motor's answer to that request. A motor that
    /// answered since the counts were taken is fresh, faulted when it reports a fault code or a
    /// state contradicting the chain's; one that did not is stale (or, never having answered,
    /// timed out); a link that failed is unavailable for every motor it could not reach.
    fn read_positions(&mut self) -> Vec<AxisReading> {
        let count = self.motors.len();
        let unavailable = vec![AxisReading::Missing(MissingReason::Unavailable); count];
        // Drain older answers before taking the high-water mark: a cached open/command answer
        // cannot masquerade as a fresh read. 0xCC requests feedback only.
        if self.control.recv().is_err() {
            return unavailable;
        }
        let Ok(before) = self.feedback_counts() else {
            return unavailable;
        };
        let sent: Vec<bool> = (0..count)
            .map(|index| {
                self.control
                    .refresh_motor_status(self.bus_id(index))
                    .is_ok()
            })
            .collect();
        let mut received: Vec<Option<Instant>> = vec![None; count];
        let mut link_failed = false;
        for attempt in 0..ANSWER_POLLS {
            if self.stamp_answers(&before, &mut received).is_err() {
                link_failed = true;
                break;
            }
            if received
                .iter()
                .zip(&sent)
                .all(|(answered, sent)| answered.is_some() || !sent)
            {
                break;
            }
            if attempt > 0 {
                self.control.port_mut().sleep(ANSWER_POLL_INTERVAL);
            }
            if self.control.recv().is_err() {
                link_failed = true;
                break;
            }
        }
        if !link_failed && self.stamp_answers(&before, &mut received).is_err() {
            link_failed = true;
        }
        let expected = if self.enabled {
            MotorState::Enabled
        } else {
            MotorState::Disabled
        };
        (0..count)
            .map(|index| {
                let bus_id = self.bus_id(index);
                let Ok(cached) = self.control.motor(bus_id) else {
                    return AxisReading::Missing(MissingReason::Unavailable);
                };
                let Some(at) = received[index] else {
                    return AxisReading::Missing(if !sent[index] || link_failed {
                        MissingReason::Unavailable
                    } else if before[index] > 0 {
                        MissingReason::Stale
                    } else {
                        MissingReason::Timeout
                    });
                };
                let code = cached.get_error();
                let state = MotorState::from_err(code);
                let unreported = !self.motors[index].reports_enable_state
                    && matches!(state, MotorState::Enabled | MotorState::Disabled);
                if state != expected && !unreported {
                    return AxisReading::Faulted(code);
                }
                AxisReading::Fresh {
                    position: self.motors[index]
                        .axis
                        .joint_from_actuator(f64::from(cached.get_position())),
                    received: at,
                }
            })
            .collect()
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt<MotorFeedback>, DamiaoChainError> {
        if self.stop_pending {
            return Err(DamiaoChainError::StopPending);
        }
        check_targets(&self.axes, joints)?;
        if let Some((motor, joint)) = self
            .motors
            .iter()
            .zip(&self.commandable)
            .zip(joints)
            .find(|((_, range), joint)| !range.admits(**joint))
            .map(|((motor, _), joint)| (motor, joint))
        {
            return Err(DamiaoChainError::Uncommandable {
                joint: motor.axis.joint.clone(),
                value: *joint,
            });
        }
        let targets: Vec<f64> = self
            .motors
            .iter()
            .zip(joints)
            .map(|(motor, joint)| motor.axis.actuator_from_joint(*joint))
            .collect();
        if !self.enabled {
            // A motor power-cycled since open holds its flash TIMEOUT (often none): every
            // motor's watchdog is rewritten and read back before any motor is enabled.
            for motor in &self.motors {
                if let Some(timeout) = motor.can_timeout {
                    let bus_id = motor.axis.binding.bus_id;
                    let counts =
                        timeout_counts(timeout).ok_or(DamiaoChainError::TimeoutRange(bus_id))?;
                    write_timeout(&mut self.control, bus_id, counts)?;
                }
            }
            self.control.recv()?;
            let before = self.feedback_counts()?;
            for index in 0..self.motors.len() {
                self.control.enable(self.bus_id(index))?;
            }
            self.await_answers(&before)?;
            self.expect_all(&before, MotorState::Enabled)?;
            self.enabled = true;
        }
        self.control.recv()?;
        let before = self.feedback_counts()?;
        for (index, target) in targets.into_iter().enumerate() {
            let DamiaoAxis { kp, kd, .. } = self.motors[index];
            self.control
                .control_mit(self.bus_id(index), kp, kd, target, 0.0, 0.0)?;
        }
        self.await_answers(&before)?;
        self.expect_all(&before, MotorState::Enabled)?;
        let feedback = self.feedback()?;
        let observed = self
            .motors
            .iter()
            .zip(&feedback)
            .map(|(motor, answer)| motor.axis.joint_from_actuator(f64::from(answer.position)))
            .collect();
        Ok(Receipt {
            evidence: Evidence::BusObservedComplete,
            observed,
            feedback,
        })
    }

    fn stop(&mut self) -> StopReport {
        self.enabled = false;
        // Only answers to the disable frames count: whatever is already on the link (a late
        // answer to the last command) is decoded before the high-water mark is taken.
        let drained = self.control.recv().is_ok();
        let Some(before) = self.feedback_counts().ok().filter(|_| drained) else {
            self.stop_pending = true;
            return StopReport {
                unproven: self.motors.iter().map(|m| m.axis.binding.bus_id).collect(),
                ..StopReport::default()
            };
        };
        for index in 0..self.motors.len() {
            let _ = self.control.disable(self.bus_id(index));
        }
        let _ = self.await_answers(&before);
        let silent: Vec<u16> = self
            .fresh(&before)
            .unwrap_or_default()
            .into_iter()
            .enumerate()
            .filter(|(_, fresh)| !fresh)
            .map(|(index, _)| self.bus_id(index))
            .collect();
        if !silent.is_empty() {
            for bus_id in silent {
                let _ = self.control.refresh_motor_status(bus_id);
            }
            let _ = self.await_answers(&before);
        }
        let fresh = self.fresh(&before).unwrap_or_default();
        let mut report = StopReport::default();
        for index in 0..self.motors.len() {
            let bus_id = self.bus_id(index);
            let state = self
                .control
                .motor(bus_id)
                .map(|motor| MotorState::from_err(motor.get_error()));
            match (fresh.get(index).copied().unwrap_or(false), state) {
                (true, Ok(MotorState::Disabled)) => {}
                (true, Ok(MotorState::Enabled)) if !self.motors[index].reports_enable_state => {
                    report.unobserved.push(bus_id);
                }
                (true, Ok(MotorState::Fault(code))) => {
                    report.faults.push((bus_id, code));
                    report.unproven.push(bus_id);
                }
                _ => report.unproven.push(bus_id),
            }
        }
        self.stop_pending = !report.is_proven();
        report
    }

    /// The longest [`ActuatorChain::stop`] takes on this chain, from its own constants: every
    /// motor's `disable` and its 0.01 s wait, an answer poll, a `refresh_motor_status` for
    /// every motor that stayed silent, a second poll, and the link's own allowance for each of
    /// those frames (`CanPort::frame_allowance`, which counts a send that waits for queue room).
    fn stop_budget(&self) -> Duration {
        let motors = u32::try_from(self.motors.len()).unwrap_or(u32::MAX);
        let polls = u32::try_from(ANSWER_POLLS - 1).unwrap_or(u32::MAX);
        let frame = self.control.port().frame_allowance();
        DISABLE_WAIT * motors + ANSWER_POLL_INTERVAL * polls * 2 + frame * motors * 2
    }

    /// Every motor's latest answered position, in joint coordinates: from open, each stop and
    /// each command.
    fn last_observed(&self) -> Option<Vec<f64>> {
        let feedback = self.feedback().ok()?;
        Some(
            self.motors
                .iter()
                .zip(feedback)
                .map(|(motor, answer)| motor.axis.joint_from_actuator(f64::from(answer.position)))
                .collect(),
        )
    }
}

impl ChainError for DamiaoChainError {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::Body(_)
            | Self::Target(_)
            | Self::Uncommandable { .. }
            | Self::StopPending
            | Self::ClearWhileEnabled
            | Self::RestingOutside { .. } => ChainFailure::Refused,
            Self::Silent(_) => ChainFailure::Timeout,
            _ => ChainFailure::Fault,
        }
    }
}
