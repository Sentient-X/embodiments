//! A chain of bound Damiao motors as one `ActuatorChain`, driven through [`crate::dm_can`].
//!
//! Every frame is one `DM_CAN.py` sends; the order they are sent in is this chain's. Each
//! motor gets the vendor demo's bring-up (`DM_Motor_Test.py:14-26`, same commit:
//! `switchControlMode`, then `read_motor_param` of `PMAX`, `VMAX` and `TMAX`, which the demo
//! prints), then a disable. Two deviations from the demo: it switches to `POS_VEL` where the
//! chain switches to `MIT`, and it refuses nothing where the chain refuses a register value
//! (below). Command arms each motor's watchdog and `enable`s it when the chain is disabled,
//! then sends one `controlMIT` per motor holding its target. `ActuatorChain::command` sends zero
//! velocity and feed-forward torque; `DamiaoChain::command_with` sends each axis' planned
//! velocity and model torque, mapped to the motor through its sign and reduction, bounded per
//! axis (each part off until bounded) and clamped to the motor's own `VMAX` and `TMAX`. Stop
//! `disable`s every motor.
//!
//! **Phases, not one motor at a time.** The vendor's calls bring one motor up at a time, each
//! frame followed by its own wait (0.05 s per register, 0.1 s per enable): fifteen motors took
//! over 4 s to open and 2.25 s to enable, and the first motor enabled sat silent for 1.4 s
//! while the other fourteen were, past the static upper body's 0.75 s `TIMEOUT`, so it latched
//! fault 0xD, communication lost, before its first command (2026-10-26, ZLG USBCANFD-200U, 14
//! motors). Here every multi-motor operation is a sequence of phases: one frame to every motor
//! back to back, then every answer awaited once against one deadline, the phase judged only
//! once all answered or the deadline passed. Open: disable all; switch all to MIT; read every
//! motor's `PMAX`, `VMAX` and `TMAX`; disable all and settle. The enable a command starts with:
//! write every `TIMEOUT`; read every one back; enable all; and the command's `controlMIT` for
//! all. Stop: disable all; ask the silent once more. Each motor's own frames keep the vendor's
//! order; only the interleaving across motors and the waits differ. A frame the link refuses
//! for a full transmit queue (`WouldBlock`, as the ZLG adapter's bounded queue does) is sent
//! again once the bus drains, within the link's `frame_allowance`. Bring-up of the static upper
//! body's fifteen is now eight round trips instead of over a hundred, and no motor is ever
//! silent while enabled for longer than one phase.
//!
//! What this chain adds over the vendor's calls, and why:
//!
//! - It refuses before any frame: ids or gains the protocol cannot carry, a model or bus this
//!   crate is not qualified for, and a target outside where its axis may be commanded now
//!   (below). `DM_CAN.py` clamps silently.
//! - It reads each motor's `PMAX`, `VMAX` and `TMAX` registers at open and scales that motor's
//!   frames by them, refusing values no Damiao table carries for its model, since a wrong row
//!   rescales every position, velocity and torque on the wire. Damiao's own tables disagree on
//!   the DM4340's velocity range — 10 rad/s in `DM_CAN.py`, 8 rad/s in its C++ SDK
//!   (`C++例程/u2can/include/damiao.h`, same commit) — so either is accepted for that model, and
//!   [`DamiaoChain::limits`] records which the motor reported. The vendor's own answer to a
//!   motor on another table is `DM_CAN.py`'s `change_limit_param`, which rewrites the row every
//!   motor of that model shares; this crate rescales the one motor instead
//!   ([`crate::dm_can::MotorControl::change_limit_param`]).
//! - Every answer is awaited fresh: a register's cached value is forgotten before it is asked
//!   for, and feedback counts only when it arrived after the phase's frames were sent. Each is
//!   judged by the feedback's first-byte high nibble ([`MotorState`]). `DM_CAN.py` stores the
//!   nibble unread and answers a lost register read from its cache.
//! - A stop is proven motor by motor: an answer to the disable frames reporting disabled. Answers
//!   already on the link when the stop begins are drained first, so a late answer to an earlier
//!   command never counts as the disable's. A silent motor is asked once more with
//!   `refresh_motor_status`. Where firmware does not report its enable state
//!   ([`DamiaoAxis::reports_enable_state`]), a fault-free answer to the disable proves the stop
//!   only weakly, and the motor is named in [`StopReport::unobserved`]. After an unproven stop
//!   every command is refused until a stop is proven.
//! - **Communication timeout.** An axis that carries a `can_timeout` has it written to the
//!   motor's `TIMEOUT` register (0x09) right before every enable (a motor power-cycled since
//!   comes back with whatever its flash holds), in RAM and never saved to flash, then confirmed
//!   by the motor's echo and by reading the register back. No motor is enabled until every
//!   motor confirmed, and the first command follows the enable at once. Damiao's "DM-J4340-2EC
//!   User Manual V1.1" (2026-09-08; gitee `kit-miao/DM-J4340-2EC` at `c1d567db`, `说明书/`),
//!   "Register Map" (p. 20: `0x09 TIMEOUT`, uint32) and "Parameter Management" (p. 30: "Time
//!   base: 50 μs per count. Only effective when the motor is enabled."): an enabled motor that
//!   hears no command for that long leaves enable mode and reports fault 0xD, communication
//!   lost. "Write Parameters" (p. 18): a write "takes effect immediately" and is "lost upon
//!   power-off" unless saved. This crate never sends the save. A motor that tripped keeps
//!   reporting 0xD until it is power-cycled or sent Damiao's clear frame (`FF..FB`, p. 17),
//!   which `DM_CAN.py` does not carry: its stop stays unproven. Open's settle phase sends the
//!   clear frame ([`DamiaoChain::clear_fault`]) once to each motor answering 0xD while
//!   disabled, then asks again, and still refuses a motor whose fault persists. No other fault
//!   is cleared at open.
//! - **Range limits motion, not state.** Each axis is commanded within its commandable range
//!   widened to where its motor last answered ([`CommandRange::reaching`]): a joint resting
//!   outside can be held or moved toward the range, never further out, and the interval
//!   shrinks as it comes in. `sx_embodiment_drivers::Governor` steps within the same interval.
//!   Nothing is refused for where a joint rests.
//! - **One turn.** Damiao's joint motors report the output shaft's position from a single-turn
//!   absolute encoder (same manual, "Motor Features", p. 5), so after power-up a joint sitting
//!   more than half a turn from zero reads as its alias within one turn. Where an axis's motor
//!   range spans more than one turn, its commandable range is cut to the motor's single-turn
//!   window, -pi..pi in actuator coordinates, so this chain never drives a joint into where its
//!   position would read ambiguously after a power cycle. A joint that rests on an alias reads
//!   as resting there, outside or inside its range, and is commanded from that reading.

use std::io;
use std::time::{Duration, Instant};

use sx_embodiment_drivers::can::{CanFrame, CanPort};
use sx_embodiment_drivers::{
    ActuatorBus, ActuatorChain, ActuatorModel, AxisReading, BindingError, BodyError, BoundAxis,
    ChainError, ChainFailure, CommandRange, Evidence, MissingReason, Receipt, StopReport,
    TargetError, check_reach, validate_chain,
};
use thiserror::Error;

use crate::dm_can::{
    ControlType, DmCanError, DmMotorType, DmVariable, ENABLE_WAIT, Motor, MotorCommand,
    MotorControl, ParamValue,
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
/// How long a phase awaits feedback (to a disable, a refresh or an MIT frame) once its frames
/// are sent.
pub(crate) const FEEDBACK_WAIT: Duration = Duration::from_millis(20);
/// How long a phase awaits register answers: `DM_CAN.py`'s one 0.05 s poll per register.
pub(crate) const REGISTER_WAIT: Duration = Duration::from_millis(50);
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

/// What a command adds to one axis' position target, at the joint: the planned velocity
/// (rad/s) and the model's feed-forward torque (N·m), both in joint coordinates.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Feedforward {
    pub velocity: f64,
    pub torque: f64,
}

/// How much [`Feedforward`] one axis accepts, at the joint. `None` switches that part off: the
/// MIT frame carries zero, as the chain did before feed-forward existed. A value is clamped to
/// its bound, then to the motor's own `VMAX` and `TMAX` registers read at open.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FeedforwardLimit {
    /// The largest planned speed sent, rad/s; the governor's speed cap is the natural bound.
    pub velocity: Option<f64>,
    /// The largest feed-forward torque sent, N·m.
    pub torque: Option<f64>,
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
    #[error("joint {joint} has a non-finite feed-forward velocity or torque")]
    Feedforward { joint: String },
    #[error("joint {joint} has a feed-forward bound that is not finite and non-negative")]
    FeedforwardLimit { joint: String },
    #[error("expected {expected} feed-forward entries, one per axis, got {found}")]
    FeedforwardLength { expected: usize, found: usize },
    #[error(transparent)]
    DmCan(#[from] DmCanError),
}

/// Bound Damiao motors on one CAN link, in native state order.
pub struct DamiaoChain<P> {
    control: MotorControl<P>,
    motors: Vec<DamiaoAxis>,
    /// Every motor's bus id, in native state order.
    ids: Vec<u16>,
    axes: Vec<BoundAxis>,
    commandable: Vec<CommandRange>,
    limits: Vec<MotorLimits>,
    enabled: bool,
    stop_pending: bool,
    /// Every motor's answer to the last command, all fresh (a command that misses one fails);
    /// none since open or a stop.
    command_readings: Option<Vec<AxisReading>>,
    /// Per axis; every part off until [`DamiaoChain::set_feedforward_limits`].
    feedforward_limits: Vec<FeedforwardLimit>,
    /// What the last command sent, at the joint, after every clamp.
    applied: Vec<Feedforward>,
}

/// What a phase awaits from the motor it sends one frame to.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum Awaits {
    /// A feedback frame that arrived after the phase's frames were sent.
    Feedback,
    /// The motor's answer about one register, its cached value forgotten first.
    Register(DmVariable),
}

/// One frame of a phase: the motor it goes to and what is awaited back.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Request {
    pub(crate) bus_id: u16,
    pub(crate) awaits: Awaits,
}

/// One request per motor of `ids`, each awaiting the same.
pub(crate) fn every(ids: &[u16], awaits: Awaits) -> Vec<Request> {
    ids.iter()
        .map(|bus_id| Request {
            bus_id: *bus_id,
            awaits,
        })
        .collect()
}

/// What one phase saw, request by request.
pub(crate) struct Phase {
    /// Whether the link took the request's frame.
    pub(crate) sent: Vec<bool>,
    /// Each request's motor's feedback count before the phase's frames.
    pub(crate) before: Vec<u64>,
    /// When each request's answer was first seen.
    pub(crate) received: Vec<Option<Instant>>,
    /// Whether every answer already on the link was taken before the frames were sent, so none
    /// of them can pass for this phase's.
    drained: bool,
    /// The first link failure; every frame was still attempted.
    pub(crate) failure: Option<DmCanError>,
}

impl Phase {
    fn answered(&self, index: usize) -> bool {
        self.received[index].is_some()
    }

    /// The phase, unless the link failed in it.
    pub(crate) fn checked(mut self) -> Result<Self, DmCanError> {
        self.failure.take().map_or(Ok(self), Err)
    }
}

/// How long a full transmit queue is waited out in one slice, taking answers meanwhile.
const QUEUE_SLICE: Duration = Duration::from_millis(1);

/// Decode one received frame as the phase's kind of answer.
fn take<P: CanPort>(control: &mut MotorControl<P>, frame: CanFrame, registers: bool) {
    if registers {
        control.take_param(frame);
    } else {
        control.take_feedback(frame);
    }
}

/// Stamp `now` on every request whose answer is in.
fn stamp<P: CanPort>(control: &MotorControl<P>, requests: &[Request], phase: &mut Phase) {
    let now = Instant::now();
    for (index, request) in requests.iter().enumerate() {
        if phase.received[index].is_some() || !phase.sent[index] {
            continue;
        }
        let Ok(motor) = control.motor(request.bus_id) else {
            continue;
        };
        let answered = match request.awaits {
            Awaits::Feedback => motor.feedback_count > phase.before[index],
            Awaits::Register(rid) => motor.get_param(rid).is_some(),
        };
        if answered {
            phase.received[index] = Some(now);
        }
    }
}

/// One phase: every request's frame sent back to back by `frame`, then the answers read until
/// each request has its own or `wait` (plus the link's frame allowance) has passed since the
/// last frame. A frame the link refuses for a full transmit queue is sent again as the queue
/// drains, for up to the link's frame allowance. Every frame is attempted whatever an earlier
/// one met.
pub(crate) fn phase<P: CanPort>(
    control: &mut MotorControl<P>,
    requests: &[Request],
    wait: Duration,
    mut frame: impl FnMut(&mut MotorControl<P>, usize, Request) -> Result<(), DmCanError>,
) -> Phase {
    let registers = requests
        .iter()
        .any(|request| matches!(request.awaits, Awaits::Register(_)));
    let drained = if registers {
        control.recv_set_param_data()
    } else {
        control.recv()
    };
    let mut phase = Phase {
        sent: vec![false; requests.len()],
        before: Vec::with_capacity(requests.len()),
        received: vec![None; requests.len()],
        drained: drained.is_ok(),
        failure: drained.err(),
    };
    for request in requests {
        phase.before.push(
            control
                .motor(request.bus_id)
                .map_or(0, |motor| motor.feedback_count),
        );
        if let Awaits::Register(rid) = request.awaits {
            let _ = control.forget_param(request.bus_id, rid);
        }
    }
    let allowance = control.port().frame_allowance();
    for (index, request) in requests.iter().enumerate() {
        let give_up = Instant::now() + allowance;
        let result = loop {
            match frame(control, index, *request) {
                Err(DmCanError::Io(error))
                    if error.kind() == io::ErrorKind::WouldBlock && Instant::now() < give_up =>
                {
                    let until = (Instant::now() + QUEUE_SLICE).min(give_up);
                    match control.port_mut().receive_until(until) {
                        Ok(Some(answer)) => take(control, answer, registers),
                        Ok(None) => {}
                        Err(error) => break Err(DmCanError::Io(error)),
                    }
                }
                other => break other,
            }
        };
        match result {
            Ok(()) => phase.sent[index] = true,
            Err(error) => {
                phase.failure.get_or_insert(error);
            }
        }
    }
    let deadline = Instant::now() + wait + allowance;
    loop {
        stamp(control, requests, &mut phase);
        let pending = (0..requests.len()).any(|index| phase.sent[index] && !phase.answered(index));
        if !pending {
            break;
        }
        match control.port_mut().receive_until(deadline) {
            Ok(Some(answer)) => take(control, answer, registers),
            Ok(None) => break,
            Err(error) => {
                phase.failure.get_or_insert(DmCanError::Io(error));
                break;
            }
        }
    }
    phase
}

/// Check the `PMAX`, `VMAX` and `TMAX` a motor last answered against the Damiao tables for its
/// model, and scale its frames by them.
///
/// # Errors
///
/// [`DamiaoChainError::LimitMismatch`] for a register unanswered (`None`) or off every table.
// A register holds the exact float32 of a table literal; equality is the comparison meant.
#[allow(clippy::float_cmp)]
pub(crate) fn held_limits<P: CanPort>(
    control: &mut MotorControl<P>,
    bus_id: u16,
) -> Result<MotorLimits, DamiaoChainError> {
    let model = control.motor(bus_id)?.motor_type;
    let row = model.limit_param();
    let mut found = [0.0_f32; 3];
    for (index, register) in LIMIT_REGISTERS.into_iter().enumerate() {
        let accepted = match register {
            DmVariable::VMAX => accepted_vmax(model),
            DmVariable::PMAX => std::iter::once(row[index]).chain(EXTRA_PMAX).collect(),
            _ => vec![row[index]],
        };
        let value = control.motor(bus_id)?.get_param(register);
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
    Ok(MotorLimits {
        bus_id,
        p_max: found[0],
        v_max: found[1],
        t_max: found[2],
    })
}

/// The registers open reads from every motor, in the vendor demo's order.
pub(crate) const LIMIT_REGISTERS: [DmVariable; 3] =
    [DmVariable::PMAX, DmVariable::VMAX, DmVariable::TMAX];

/// Bring every motor up as the vendor does, in two phases: MIT mode for all, then every limit
/// register of every motor; see [`DamiaoChain::open`].
// A register holds the exact float32 of a table literal; equality is the comparison meant.
#[allow(clippy::float_cmp)]
fn bring_up<P: CanPort>(
    control: &mut MotorControl<P>,
    ids: &[u16],
) -> Result<Vec<MotorLimits>, DamiaoChainError> {
    let mode = ControlType::MIT as u8;
    phase(
        control,
        &every(ids, Awaits::Register(DmVariable::CTRL_MODE)),
        REGISTER_WAIT,
        |control, _, request| {
            control.send_write_param(request.bus_id, DmVariable::CTRL_MODE, f64::from(mode))
        },
    )
    .checked()?;
    for bus_id in ids {
        if control.motor(*bus_id)?.get_param(DmVariable::CTRL_MODE)
            != Some(ParamValue::Uint32(u32::from(mode)))
        {
            return Err(DamiaoChainError::ModeNotConfirmed(*bus_id));
        }
    }
    let reads: Vec<Request> = ids
        .iter()
        .flat_map(|bus_id| {
            LIMIT_REGISTERS.map(|register| Request {
                bus_id: *bus_id,
                awaits: Awaits::Register(register),
            })
        })
        .collect();
    phase(control, &reads, REGISTER_WAIT, |control, index, request| {
        control.send_read_param(
            request.bus_id,
            LIMIT_REGISTERS[index % LIMIT_REGISTERS.len()],
        )
    })
    .checked()?;
    let mut limits = Vec::with_capacity(ids.len());
    for bus_id in ids.iter().copied() {
        limits.push(held_limits(control, bus_id)?);
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

/// Send every named motor its disable frame, whatever any answered: the last word of an open
/// that failed, returned as `refused` unless a disable could not be sent, which is then named
/// beside it, since that motor may still hold torque.
fn disable_all<P: CanPort>(
    control: &mut MotorControl<P>,
    ids: &[u16],
    refused: DamiaoChainError,
) -> DamiaoChainError {
    let disabled = phase(
        control,
        &every(ids, Awaits::Feedback),
        FEEDBACK_WAIT,
        |control, _, request| control.send_command(request.bus_id, MotorCommand::Disable),
    );
    let unsent: Vec<u16> = ids
        .iter()
        .zip(&disabled.sent)
        .filter(|(_, sent)| !**sent)
        .map(|(bus_id, _)| *bus_id)
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
    /// Validate every axis, bring every motor up as the vendor does, scale each by its own
    /// limit registers, and leave the chain disabled with every motor's disabled answer in
    /// hand, in phases: disable all, MIT mode for all, every limit register, disable and
    /// settle all.
    ///
    /// # Errors
    ///
    /// Refuses an invalid or unqualified axis, a motor that does not confirm MIT mode or whose
    /// limit registers no Damiao table carries, and a motor that does not answer disabled.
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
        let ids: Vec<u16> = motors
            .iter()
            .map(|motor| motor.axis.binding.bus_id)
            .collect();
        // Disable every motor before anything is written: a motor left enabled by an earlier
        // run (which firmware that does not report its enable state hides) would otherwise
        // hold torque through bring-up. Its answers prove nothing yet; they are taken here so
        // none is mistaken for a register's.
        phase(
            &mut control,
            &every(&ids, Awaits::Feedback),
            FEEDBACK_WAIT,
            |control, _, request| control.send_command(request.bus_id, MotorCommand::Disable),
        )
        .checked()?;
        // A motor brought up before a later one failed may hold MIT mode with torque enabled
        // (a motor left enabled by a crashed run answers its mode switch as it is): every motor
        // the chain names is disabled before the failure is returned.
        let limits = match bring_up(&mut control, &ids) {
            Ok(limits) => limits,
            Err(error) => return Err(disable_all(&mut control, &ids, error)),
        };
        let commandable = motors.iter().map(DamiaoAxis::commandable).collect();
        let count = motors.len();
        let mut chain = Self {
            control,
            motors,
            ids,
            axes,
            commandable,
            limits,
            enabled: false,
            stop_pending: false,
            command_readings: None,
            feedforward_limits: vec![FeedforwardLimit::default(); count],
            applied: vec![Feedforward::default(); count],
        };
        if let Err(error) = chain.settle_disabled() {
            return Err(disable_all(&mut chain.control, &chain.ids, error));
        }
        Ok(chain)
    }

    /// Disable every motor and hold each one's disabled answer. Motors answering
    /// [`COMMUNICATION_LOST`] are cleared once ([`DamiaoChain::clear_fault`]) and asked again.
    fn settle_disabled(&mut self) -> Result<(), DamiaoChainError> {
        let disabled = phase(
            &mut self.control,
            &every(&self.ids, Awaits::Feedback),
            FEEDBACK_WAIT,
            |control, _, request| control.send_command(request.bus_id, MotorCommand::Disable),
        )
        .checked()?;
        let mut lost = Vec::new();
        for (index, bus_id) in self.ids.iter().enumerate() {
            let state = MotorState::from_err(self.control.motor(*bus_id)?.get_error());
            if disabled.answered(index) && state == MotorState::Fault(COMMUNICATION_LOST) {
                lost.push(*bus_id);
            }
        }
        if !lost.is_empty() {
            self.clear(&lost)?;
        }
        self.expect_all(&disabled, MotorState::Disabled)
    }

    /// Send one motor Damiao's clear-error frame (`FF..FB`) after its disable, then ask for its
    /// state, while the chain is disabled.
    ///
    /// # Errors
    ///
    /// Refuses while the chain is enabled; returns a motor that stays silent, or whose answer
    /// still carries a fault, as [`DamiaoChainError::FaultPersists`].
    pub fn clear_fault(&mut self, bus_id: u16) -> Result<(), DamiaoChainError> {
        self.clear(&[bus_id])
    }

    /// [`DamiaoChain::clear_fault`] for every motor of `ids` at once: a disable phase, a clear
    /// phase, then a refresh phase, each awaiting its answers so none passes for the next's.
    fn clear(&mut self, ids: &[u16]) -> Result<(), DamiaoChainError> {
        if self.enabled {
            return Err(DamiaoChainError::ClearWhileEnabled);
        }
        let requests = every(ids, Awaits::Feedback);
        for command in [MotorCommand::Disable, MotorCommand::ClearError] {
            phase(
                &mut self.control,
                &requests,
                FEEDBACK_WAIT,
                |control, _, request| control.send_command(request.bus_id, command),
            )
            .checked()?;
        }
        let asked = phase(
            &mut self.control,
            &requests,
            FEEDBACK_WAIT,
            |control, _, request| control.send_refresh(request.bus_id),
        )
        .checked()?;
        for (index, bus_id) in ids.iter().copied().enumerate() {
            if !asked.answered(index) {
                return Err(DamiaoChainError::Silent(bus_id));
            }
            if let MotorState::Fault(code) =
                MotorState::from_err(self.control.motor(bus_id)?.get_error())
            {
                return Err(DamiaoChainError::FaultPersists { bus_id, code });
            }
        }
        Ok(())
    }

    /// Arm every motor's watchdog, then enable every motor, the last step before the first
    /// command: a `TIMEOUT` write phase, a read-back phase, then an enable phase, so no motor
    /// is ever silent while enabled for longer than one phase.
    fn arm_and_enable(&mut self) -> Result<(), DamiaoChainError> {
        let mut armed: Vec<(u16, u32)> = Vec::new();
        for motor in &self.motors {
            if let Some(timeout) = motor.can_timeout {
                let bus_id = motor.axis.binding.bus_id;
                let counts =
                    timeout_counts(timeout).ok_or(DamiaoChainError::TimeoutRange(bus_id))?;
                armed.push((bus_id, counts));
            }
        }
        if !armed.is_empty() {
            let ids: Vec<u16> = armed.iter().map(|(bus_id, _)| *bus_id).collect();
            let requests = every(&ids, Awaits::Register(DmVariable::TIMEOUT));
            // A motor power-cycled since open holds its flash TIMEOUT (often none): every
            // motor's watchdog is written, echoed and read back before any motor is enabled.
            for write in [true, false] {
                phase(
                    &mut self.control,
                    &requests,
                    REGISTER_WAIT,
                    |control, index, request| {
                        if write {
                            control.send_write_param(
                                request.bus_id,
                                DmVariable::TIMEOUT,
                                f64::from(armed[index].1),
                            )
                        } else {
                            control.send_read_param(request.bus_id, DmVariable::TIMEOUT)
                        }
                    },
                )
                .checked()?;
                for (bus_id, counts) in armed.iter().copied() {
                    if self.control.motor(bus_id)?.get_param(DmVariable::TIMEOUT)
                        != Some(ParamValue::Uint32(counts))
                    {
                        return Err(DamiaoChainError::TimeoutNotConfirmed { bus_id, counts });
                    }
                }
            }
        }
        let enabled = phase(
            &mut self.control,
            &every(&self.ids, Awaits::Feedback),
            ENABLE_WAIT,
            |control, _, request| control.send_command(request.bus_id, MotorCommand::Enable),
        )
        .checked()?;
        self.expect_all(&enabled, MotorState::Enabled)?;
        self.enabled = true;
        Ok(())
    }

    /// Bound, or switch off, each axis' feed-forward, in native state order.
    ///
    /// # Errors
    ///
    /// Refuses a list of another length and a bound that is not finite and non-negative; the
    /// bounds in force are kept.
    pub fn set_feedforward_limits(
        &mut self,
        limits: &[FeedforwardLimit],
    ) -> Result<(), DamiaoChainError> {
        if limits.len() != self.motors.len() {
            return Err(DamiaoChainError::FeedforwardLength {
                expected: self.motors.len(),
                found: limits.len(),
            });
        }
        for (motor, limit) in self.motors.iter().zip(limits) {
            let valid = |bound: Option<f64>| bound.is_none_or(|b| b.is_finite() && b >= 0.0);
            if !(valid(limit.velocity) && valid(limit.torque)) {
                return Err(DamiaoChainError::FeedforwardLimit {
                    joint: motor.axis.joint.clone(),
                });
            }
        }
        self.feedforward_limits.copy_from_slice(limits);
        Ok(())
    }

    /// The feed-forward each axis' last MIT frame carried, at the joint, after every clamp:
    /// what a journal records as the commanded torque beside the measured one.
    #[must_use]
    pub fn applied_feedforward(&self) -> &[Feedforward] {
        &self.applied
    }

    /// [`ActuatorChain::command`] with each axis' planned velocity and feed-forward torque in
    /// its MIT frame, each bounded by [`DamiaoChain::set_feedforward_limits`] (a part switched
    /// off sends zero) and then by the motor's `VMAX` and `TMAX`.
    ///
    /// # Errors
    ///
    /// As [`ActuatorChain::command`], and refuses before any frame a feed-forward list of
    /// another length or a non-finite entry.
    pub fn command_with(
        &mut self,
        joints: &[f64],
        feedforward: &[Feedforward],
    ) -> Result<Receipt<MotorFeedback>, DamiaoChainError> {
        if feedforward.len() != self.motors.len() {
            return Err(DamiaoChainError::FeedforwardLength {
                expected: self.motors.len(),
                found: feedforward.len(),
            });
        }
        if let Some((motor, _)) = self
            .motors
            .iter()
            .zip(feedforward)
            .find(|(_, ff)| !(ff.velocity.is_finite() && ff.torque.is_finite()))
        {
            return Err(DamiaoChainError::Feedforward {
                joint: motor.axis.joint.clone(),
            });
        }
        self.drive(joints, Some(feedforward))
    }

    /// One axis' feed-forward in the motor's own coordinates, `(velocity, torque)`, and what
    /// that is at the joint. `actuator = g (joint - zero)` with `g = sign * direction *
    /// reduction`, so velocity scales by `g` and torque, by power, by `1 / g`.
    ///
    /// Feed-forward moves where the motor settles: unloaded, `(kd dq + tau) / kp` past the
    /// target. Both parts are scaled down together so that excursion stays inside the axis'
    /// commandable range from `target` (actuator coordinates); with `kp` zero nothing is sent.
    /// A load the feed-forward balances (gravity held near a limit) makes this conservative:
    /// the joint may sag, away from the push, never past the range.
    fn actuator_feedforward(
        &self,
        index: usize,
        target: f64,
        wanted: Feedforward,
    ) -> ((f64, f64), Feedforward) {
        let DamiaoAxis { kp, kd, .. } = self.motors[index];
        let axis = &self.motors[index].axis;
        let gain = axis.actuator_from_joint(axis.zero + 1.0);
        let FeedforwardLimit { velocity, torque } = self.feedforward_limits[index];
        let MotorLimits { v_max, t_max, .. } = self.limits[index];
        let (v_max, t_max) = (f64::from(v_max), f64::from(t_max));
        let speed = velocity.map_or(0.0, |bound| wanted.velocity.clamp(-bound, bound));
        let effort = torque.map_or(0.0, |bound| wanted.torque.clamp(-bound, bound));
        let mut dq = (gain * speed).clamp(-v_max, v_max);
        let mut tau = (effort / gain).clamp(-t_max, t_max);
        let range = self.commandable[index];
        let ends = [
            axis.actuator_from_joint(range.lower),
            axis.actuator_from_joint(range.upper),
        ];
        let push = kd.mul_add(dq, tau);
        let margin = if push > 0.0 {
            (ends[0].max(ends[1]) - target).max(0.0)
        } else {
            (target - ends[0].min(ends[1])).max(0.0)
        };
        let allowed = kp * margin;
        if push.abs() > allowed {
            let scale = allowed / push.abs();
            dq *= scale;
            tau *= scale;
        }
        (
            (dq, tau),
            Feedforward {
                velocity: dq / gain,
                torque: tau * gain,
            },
        )
    }

    /// Command each axis' target, with `feedforward` in its MIT frame where given: refuse a
    /// target outside its axis's range widened to where its motor last answered, then (when
    /// disabled) arm and enable every motor, then one MIT phase.
    fn drive(
        &mut self,
        joints: &[f64],
        feedforward: Option<&[Feedforward]>,
    ) -> Result<Receipt<MotorFeedback>, DamiaoChainError> {
        if self.stop_pending {
            return Err(DamiaoChainError::StopPending);
        }
        check_reach(
            &self.axes,
            &self.commandable,
            self.last_observed().as_deref(),
            joints,
        )?;
        let targets: Vec<f64> = self
            .motors
            .iter()
            .zip(joints)
            .map(|(motor, joint)| motor.axis.actuator_from_joint(*joint))
            .collect();
        // Each axis' feed-forward in the motor's coordinates, and what that is at the joint.
        let (sent, applied): (Vec<(f64, f64)>, Vec<Feedforward>) = (0..self.motors.len())
            .map(|index| {
                self.actuator_feedforward(
                    index,
                    targets[index],
                    feedforward.map_or(Feedforward::default(), |all| all[index]),
                )
            })
            .unzip();
        if !self.enabled {
            self.arm_and_enable()?;
        }
        self.command_readings = None;
        let motors = &self.motors;
        let answered = phase(
            &mut self.control,
            &every(&self.ids, Awaits::Feedback),
            FEEDBACK_WAIT,
            |control, index, request| {
                let DamiaoAxis { kp, kd, .. } = motors[index];
                let (dq, tau) = sent[index];
                control.send_mit(request.bus_id, kp, kd, targets[index], dq, tau)
            },
        )
        .checked()?;
        // Recorded only once every frame went out.
        self.applied.copy_from_slice(&applied);
        self.expect_all(&answered, MotorState::Enabled)?;
        let feedback = self.feedback()?;
        let observed: Vec<f64> = self
            .motors
            .iter()
            .zip(&feedback)
            .map(|(motor, answer)| motor.axis.joint_from_actuator(f64::from(answer.position)))
            .collect();
        self.command_readings = Some(
            observed
                .iter()
                .zip(&answered.received)
                .map(|(position, received)| AxisReading::Fresh {
                    position: *position,
                    // Every motor answered: `expect_all` refuses a silent one.
                    received: received.unwrap_or_else(Instant::now),
                })
                .collect(),
        );
        Ok(Receipt {
            evidence: Evidence::BusObservedComplete,
            observed,
            feedback,
        })
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

    /// Whether motor `index`'s answered state is `expected`, or (where its firmware does not
    /// report its enable state) any state but a fault.
    fn in_state(&self, index: usize, state: MotorState, expected: MotorState) -> bool {
        state == expected
            || (!self.motors[index].reports_enable_state
                && matches!(state, MotorState::Enabled | MotorState::Disabled))
    }

    /// Require every motor to have answered `phase` (one request per motor, in native state
    /// order) and to report `expected`.
    fn expect_all(&self, phase: &Phase, expected: MotorState) -> Result<(), DamiaoChainError> {
        for (index, bus_id) in self.ids.iter().copied().enumerate() {
            if !phase.answered(index) {
                return Err(DamiaoChainError::Silent(bus_id));
            }
            let state = MotorState::from_err(self.control.motor(bus_id)?.get_error());
            if !self.in_state(index, state, expected) {
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

pub(crate) fn validate(motor: &DamiaoAxis, ids: &mut Vec<u16>) -> Result<(), DamiaoChainError> {
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

    /// One refresh phase: drain older answers, request status from every motor with the
    /// vendor's `0xCC` operation, and read the answers. A motor that answered since its frame
    /// was sent is fresh, faulted when it reports a fault code or a state contradicting the
    /// chain's; one that did not is stale (or, never having answered, timed out); a link that
    /// failed is unavailable for every motor it could not reach.
    fn read_positions(&mut self) -> Vec<AxisReading> {
        let asked = phase(
            &mut self.control,
            &every(&self.ids, Awaits::Feedback),
            FEEDBACK_WAIT,
            |control, _, request| control.send_refresh(request.bus_id),
        );
        let link_failed = asked.failure.is_some();
        let expected = if self.enabled {
            MotorState::Enabled
        } else {
            MotorState::Disabled
        };
        (0..self.ids.len())
            .map(|index| {
                let Ok(cached) = self.control.motor(self.ids[index]) else {
                    return AxisReading::Missing(MissingReason::Unavailable);
                };
                let Some(at) = asked.received[index] else {
                    return AxisReading::Missing(if !asked.sent[index] || link_failed {
                        MissingReason::Unavailable
                    } else if asked.before[index] > 0 {
                        MissingReason::Stale
                    } else {
                        MissingReason::Timeout
                    });
                };
                let code = cached.get_error();
                if !self.in_state(index, MotorState::from_err(code), expected) {
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
        self.drive(joints, None)
    }

    fn command_readings(&self) -> Option<Vec<AxisReading>> {
        self.command_readings.clone()
    }

    /// A disable phase to every motor, then a refresh phase to any that stayed silent.
    fn stop(&mut self) -> StopReport {
        self.command_readings = None;
        self.applied.fill(Feedforward::default());
        self.enabled = false;
        // Only answers to the disable frames count: whatever is already on the link (a late
        // answer to the last command) is decoded before the phase's frames are sent.
        let disabled = phase(
            &mut self.control,
            &every(&self.ids, Awaits::Feedback),
            FEEDBACK_WAIT,
            |control, _, request| control.send_command(request.bus_id, MotorCommand::Disable),
        );
        let mut answered: Vec<bool> = (0..self.ids.len())
            .map(|index| disabled.drained && disabled.answered(index))
            .collect();
        let silent: Vec<usize> = (0..self.ids.len()).filter(|i| !answered[*i]).collect();
        if disabled.drained && !silent.is_empty() {
            let ids: Vec<u16> = silent.iter().map(|index| self.ids[*index]).collect();
            let asked = phase(
                &mut self.control,
                &every(&ids, Awaits::Feedback),
                FEEDBACK_WAIT,
                |control, _, request| control.send_refresh(request.bus_id),
            );
            for (slot, index) in silent.into_iter().enumerate() {
                answered[index] = asked.drained && asked.answered(slot);
            }
        }
        let mut report = StopReport::default();
        for (index, bus_id) in self.ids.iter().copied().enumerate() {
            let state = self
                .control
                .motor(bus_id)
                .map(|motor| MotorState::from_err(motor.get_error()));
            match (answered[index], state) {
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

    /// The longest [`ActuatorChain::stop`] takes on this chain, from its own constants: two
    /// phases, each a frame per motor that may wait the link's own allowance for queue room
    /// (`CanPort::frame_allowance`), then the answer wait and one more allowance.
    fn stop_budget(&self) -> Duration {
        let motors = u32::try_from(self.motors.len()).unwrap_or(u32::MAX);
        let frame = self.control.port().frame_allowance();
        (frame * motors + FEEDBACK_WAIT + frame) * 2
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
            | Self::StopPending
            | Self::ClearWhileEnabled
            | Self::Feedforward { .. }
            | Self::FeedforwardLimit { .. }
            | Self::FeedforwardLength { .. } => ChainFailure::Refused,
            Self::Silent(_) => ChainFailure::Timeout,
            _ => ChainFailure::Fault,
        }
    }
}
