//! A chain of bound Dynamixel X-series servos as one `ActuatorChain`, driven through
//! [`crate::dynamixel_sdk`].
//!
//! **Untested on the bench.** The registry's Dynamixel models were added on 2026-10-25 without
//! a servo on the bench; this chain is checked against the SDK's own packets and the e-manual's
//! examples only. A model is qualified when one servo of it has been driven through this chain
//! and has safe-stopped with its torque read back off.
//!
//! Every bus operation is a `DynamixelSDK` call. The order they are called in, and what a goal
//! position means on the wire, are those of `LeRobot`'s Dynamixel bus
//! (<https://github.com/huggingface/lerobot> at `v0.6.0`,
//! `30da8e687a6dfc617fcd94afc367ac7071c376ce`, `src/lerobot/motors/dynamixel/dynamixel.py`),
//! as `sx-feetech-serial`'s chain follows its Feetech bus, with the control table of the
//! X series' e-manual (<https://github.com/ROBOTIS-GIT/emanual> at
//! `0807b1b5f7472bbbbb2c9751f1d3406678e86d37`, `docs/en/dxl/x/xl330-m288.md` and
//! `xm430-w350.md`, which share the addresses used here), the same addresses as `LeRobot`'s
//! `X_SERIES_CONTROL_TABLE` (`src/lerobot/motors/dynamixel/tables.py`):
//!
//! - Open, without `LeRobot`'s interactive calibration:
//!   1. It pings every servo (`ping`) and refuses one whose model number is not its binding's:
//!      XL330-M288 1200, XM430-W350 1020 (each e-manual's `Model Number(0)` default, and
//!      `LeRobot`'s `MODEL_NUMBER_TABLE`).
//!   2. It reads each servo's `Hardware Error Status` (70) and refuses one that reports any:
//!      such a servo will not take torque until it is rebooted, which is a person's call.
//!   3. It ports `DynamixelMotorsBus.is_calibrated` over `read_calibration` (`dynamixel.py:
//!      161-180`): per servo `Homing_Offset` (20, four bytes, two's complement),
//!      `Min_Position_Limit` (52), `Max_Position_Limit` (48) and `Drive_Mode` (10), each compared
//!      with the [`MotorCalibration`] supplied. Where `LeRobot` would recalibrate
//!      interactively, open refuses ([`DynamixelChainError::Uncalibrated`]).
//!   4. Inside `torque_disabled`: `Torque_Enable` (64) 0 on every servo, then
//!      `configure_motors` (`dynamixel.py:154-159`): `Return_Delay_Time` (9) 0 on every servo;
//!      then each servo's `Operating_Mode` (11), as `KochFollower.configure` writes it
//!      (`src/lerobot/robots/koch_follower/koch_follower.py`). Every write is acknowledged and
//!      an error byte refuses. Where `torque_disabled` re-enables torque on leaving, open leaves
//!      the chain disabled: the first command enables it.
//!   5. Its own limits, which `LeRobot` does not write: per servo its `Goal Current` (102,
//!      where declared), `Profile Acceleration` (108) and `Profile Velocity` (112), then one
//!      READ of 102..=135 confirming each, then every servo's `Present_Position` (132), which is
//!      [`ActuatorChain::last_observed`].
//! - Command enables a disabled chain as `enable_torque` does (`dynamixel.py:199-201`):
//!   `Torque_Enable` 1 per servo, each acknowledged, after proving every limit still holds
//!   and writing each servo's `Present_Position` to its `Goal_Position`. It then sends one
//!   broadcast SYNC WRITE of every `Goal_Position` (116, four bytes, two's complement as
//!   `_encode_sign` writes it), as `sync_write("Goal_Position", ...)` does. The bus answers
//!   nothing to it, so a command's receipt claims only a dispatch; right after it one SYNC READ
//!   (`GroupSyncRead`) of 102..=135 is the receipt's observed joints.
//! - Stop disables as `disable_torque` does (`dynamixel.py:191-193`): `Torque_Enable` 0 on
//!   every servo, each acknowledged, whatever an earlier one answered. Then it reads every
//!   servo's seven bytes from `Torque_Enable` (64) to `Hardware_Error_Status` (70) in one READ;
//!   a servo is proven stopped only when its `Torque_Enable` reads 0. Then every servo's
//!   `Present_Position`, the stop's [`ActuatorChain::last_observed`].
//!
//! A goal position is the registry's drive map, then `LeRobot`'s normalization through the
//! unit's calibration ([`sx_embodiment_drivers::lerobot::unnormalize`], with
//! `apply_drive_mode` false: a Dynamixel reverses itself through its `Drive_Mode` register).
//!
//! What this chain adds over those calls, and why:
//!
//! - It refuses before any frame: a wrong width, a target outside the axis's declared bounds
//!   or not finite, a goal outside the unit's calibrated range (the servo's own position
//!   limits do not apply in the extended and current-based position modes), a model or bus
//!   this crate does not know, an address the protocol cannot carry, an operating mode that
//!   does not control position, and an empty or out-of-resolution calibration.
//! - A stop is proven servo by servo by the torque read-back, not by the acknowledgements. A
//!   `Hardware_Error_Status` read while stopping, or else an error byte an answer carried, is a
//!   fault of a stop that may still be proven; faults are latched, and every command is refused
//!   until [`ActuatorChain::clear_faults`] clears them. After an unproven stop every command is
//!   refused until a stop is proven.
//! - **Every servo declares an approach limit** ([`DynamixelServo::approach`]), written to its
//!   `Profile Velocity` and `Profile Acceleration` (e-manual: velocity-based profile, 0.229
//!   rev/min and 214.577 rev/min² per step; `Drive_Mode` bit 2 clear, which open's calibration
//!   check holds). Both read 0 as no limit at all, so a limit that rounds below one step, or
//!   past 32,767, is refused at open. The servo itself then approaches any goal at that pace.
//!   The registers are RAM, reset at power-on, so they are never trusted for the chain's life:
//!   every enable reads them back (rewriting and reading again where they differ, refusing a
//!   servo that still does not hold them), and every command's SYNC READ covers them, so a
//!   servo found unlimited is `NaN` and the next command enables afresh.
//! - **Current-based position** (mode 5) pushes with whatever `Goal Current` (102) holds, which
//!   power-on resets to `Current Limit`: such a servo must declare its `Goal Current`, held and
//!   checked as its profile is, and any other servo must not.
//! - **Multi-turn modes are declared.** Extended position (4) and current-based position (5)
//!   ignore `Min/Max_Position_Limit` (e-manual `Operating Mode(11)`); a servo is refused either
//!   unless its body declares [`DynamixelServo::multi_turn`].
//! - **Enabling never drives toward a stale goal.** Before `Torque_Enable` 1, each servo's
//!   `Present_Position` is read and written to its `Goal_Position`; `LeRobot` enables with
//!   whatever goal the register last held.
//! - **A command observes.** Each command's SYNC READ reports every servo's position, `NaN`
//!   for one that misses it or no longer holds its limits; either marks the chain disabled.

use std::time::{Duration, Instant};

pub use sx_embodiment_drivers::lerobot::{MotorCalibration, MotorNormMode};
use sx_embodiment_drivers::lerobot::{normalize, unnormalize};
use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{
    ActuatorBus, ActuatorChain, ActuatorModel, ApproachLimit, Axes, AxisReading, BindingError,
    BodyError, BoundAxis, ChainError, ChainFailure, CommandRange, Evidence, MissingReason, Receipt,
    StopReport, TargetError, fresh_positions, validate_chain,
};
use thiserror::Error;

use crate::dynamixel_sdk::{
    CommResult, GroupSyncRead, GroupSyncWrite, MAX_ID, Protocol2PacketHandler, dxl_hiword,
    dxl_loword,
};

/// `ActuatorModel.DYNAMIXEL_XL330_M288` in the registry.
pub const DYNAMIXEL_XL330_M288: ActuatorModel = ActuatorModel::new("dynamixel_xl330_m288");
/// `ActuatorModel.DYNAMIXEL_XM430_W350` in the registry.
pub const DYNAMIXEL_XM430_W350: ActuatorModel = ActuatorModel::new("dynamixel_xm430_w350");
/// `ActuatorBus.DYNAMIXEL_SERIAL` in the registry.
pub const DYNAMIXEL_SERIAL: ActuatorBus = ActuatorBus::new("dynamixel_serial");

/// Each known model's `Model Number(0)`, from its e-manual page.
pub const MODEL_NUMBERS: [(ActuatorModel, u16); 2] =
    [(DYNAMIXEL_XL330_M288, 1200), (DYNAMIXEL_XM430_W350, 1020)];

/// The model number a servo of `model` reports, when this crate knows the model.
#[must_use]
pub fn model_number(model: ActuatorModel) -> Option<u16> {
    MODEL_NUMBERS
        .iter()
        .find(|(known, _)| *known == model)
        .map(|(_, number)| *number)
}

// The X series' control table (e-manual "Control Table"; `LeRobot` `X_SERIES_CONTROL_TABLE`).
pub const MODEL_NUMBER: u16 = 0;
pub const RETURN_DELAY_TIME: u16 = 9;
pub const DRIVE_MODE: u16 = 10;
pub const OPERATING_MODE: u16 = 11;
pub const HOMING_OFFSET: u16 = 20;
pub const MAX_POSITION_LIMIT: u16 = 48;
pub const MIN_POSITION_LIMIT: u16 = 52;
pub const TORQUE_ENABLE: u16 = 64;
pub const HARDWARE_ERROR_STATUS: u16 = 70;
pub const GOAL_CURRENT: u16 = 102;
pub const PROFILE_ACCELERATION: u16 = 108;
pub const PROFILE_VELOCITY: u16 = 112;
pub const GOAL_POSITION: u16 = 116;
pub const PRESENT_CURRENT: u16 = 126;
pub const PRESENT_POSITION: u16 = 132;

/// Encoder steps per turn (`LeRobot` `MODEL_RESOLUTION`, 4096 for every X-series model here).
const RESOLUTION: f64 = 4096.0;
/// `DynamixelMotorsBus.apply_drive_mode`: the servo applies its own `Drive_Mode`.
const APPLY_DRIVE_MODE: bool = false;
/// `configure_motors`' `return_delay_time`: 2 µs, down from the 500 µs default.
const RETURN_DELAY: u8 = 0;
/// Exchanges a stop makes per servo: torque off, the read-back, and the position read.
const STOP_EXCHANGES_PER_SERVO: u32 = 3;
/// One `Profile Velocity` step: 0.229 rev/min (e-manual `Profile Velocity(112)`, velocity-based
/// profile), in rad/s.
pub const PROFILE_VELOCITY_STEP_RAD_PER_S: f64 = 0.229 * std::f64::consts::TAU / 60.0;
/// One `Profile Acceleration` step: 214.577 rev/min² (e-manual `Profile Acceleration(108)`,
/// velocity-based profile), in rad/s².
pub const PROFILE_ACCELERATION_STEP_RAD_PER_S2: f64 = 214.577 * std::f64::consts::TAU / 3600.0;
/// Both profile registers' largest value (e-manual: 0 ~ 32,767).
const PROFILE_MAX: u32 = 32_767;
/// The registers each command's SYNC READ and each enable's read-back cover: `Goal Current`
/// (102) through `Present Position`'s last byte (135), one contiguous range of the RAM area.
pub const LIMITS_READ_LENGTH: u16 = PRESENT_POSITION + 4 - GOAL_CURRENT;
/// The bytes a stop reads back, `Torque_Enable` (64) through `Hardware_Error_Status` (70).
const STOP_READ_LENGTH: u16 = HARDWARE_ERROR_STATUS - TORQUE_ENABLE + 1;

/// The position-controlling members of `Operating_Mode` (11), by register value (`LeRobot`
/// `OperatingMode`); the current, velocity and PWM modes are not offered.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum OperatingMode {
    /// 3: within `Min_Position_Limit` and `Max_Position_Limit`.
    Position,
    /// 4: multi-turn; the position limits do not apply.
    ExtendedPosition,
    /// 5: multi-turn position under `Goal_Current`, `LeRobot`'s choice for a gripper.
    CurrentPosition,
}

impl OperatingMode {
    #[must_use]
    pub const fn register(self) -> u8 {
        match self {
            Self::Position => 3,
            Self::ExtendedPosition => 4,
            Self::CurrentPosition => 5,
        }
    }
}

/// One bound axis with what its Dynamixel servo needs beyond the registry's binding.
#[derive(Clone, Debug, PartialEq)]
pub struct DynamixelServo {
    pub axis: BoundAxis,
    pub norm_mode: MotorNormMode,
    pub calibration: MotorCalibration,
    pub operating_mode: OperatingMode,
    /// The speed the servo itself approaches any goal at, written to its `Profile Velocity`
    /// and `Profile Acceleration`, which read 0 as no limit at all: every servo declares one.
    pub approach: ApproachLimit,
    /// `Goal Current` (102) in the servo's own units: the most current a current-based
    /// position servo may push with, required in that mode and refused in any other.
    pub goal_current: Option<u16>,
    /// Whether the body declares this servo may turn more than once: the extended and
    /// current-based position modes ignore the position limits and are refused without it.
    pub multi_turn: bool,
}

impl DynamixelServo {
    /// The `Profile Velocity` and `Profile Acceleration` that hold this servo to its approach
    /// limit, in the velocity-based profile's steps (the `Drive_Mode` open checks is 0 or 1,
    /// so its time-based bit 2 is clear).
    #[must_use]
    pub fn profile(&self) -> Option<(u32, u32)> {
        self.approach.steps(
            self.axis.binding.reduction,
            (PROFILE_VELOCITY_STEP_RAD_PER_S, PROFILE_MAX),
            (PROFILE_ACCELERATION_STEP_RAD_PER_S2, PROFILE_MAX),
        )
    }

    /// The joint coordinate a `Present_Position` reading of `raw` ticks is at, unclamped.
    #[must_use]
    pub fn joint_from_raw(&self, raw: i32) -> f64 {
        let axis = &self.axis;
        let ends = [
            axis.actuator_from_joint(axis.lower),
            axis.actuator_from_joint(axis.upper),
        ];
        axis.joint_from_actuator(normalize(
            self.norm_mode,
            &self.calibration,
            APPLY_DRIVE_MODE,
            RESOLUTION,
            f64::from(raw),
            (ends[0].min(ends[1]), ends[0].max(ends[1])),
        ))
    }

    pub(crate) fn dxl_id(&self) -> u8 {
        // Open refuses an address above `MAX_ID`, so the narrowing never saturates.
        u8::try_from(self.axis.binding.bus_id).unwrap_or(u8::MAX)
    }

    /// The joint range this servo is commanded within: its declared bounds.
    #[must_use]
    pub const fn commandable(&self) -> CommandRange {
        CommandRange {
            lower: self.axis.lower,
            upper: self.axis.upper,
        }
    }

    /// The `Goal_Position` register value that drives this servo to `joint`.
    ///
    /// # Errors
    ///
    /// Refuses a target outside the declared bounds or not finite, and a goal outside the
    /// unit's calibrated range.
    pub fn goal_position(&self, joint: f64) -> Result<u32, DynamixelChainError> {
        let axis = &self.axis;
        if !axis.admits(joint) {
            return Err(TargetError::OutOfBounds {
                joint: axis.joint.clone(),
                value: joint,
            }
            .into());
        }
        let ends = [
            axis.actuator_from_joint(axis.lower),
            axis.actuator_from_joint(axis.upper),
        ];
        let raw = unnormalize(
            self.norm_mode,
            &self.calibration,
            APPLY_DRIVE_MODE,
            RESOLUTION,
            axis.actuator_from_joint(joint),
            (ends[0].min(ends[1]), ends[0].max(ends[1])),
        );
        let range = f64::from(self.calibration.range_min)..=f64::from(self.calibration.range_max);
        if !range.contains(&raw) {
            return Err(DynamixelChainError::OutsideCalibration {
                bus_id: axis.binding.bus_id,
                raw,
            });
        }
        // `raw` is an integer within a calibrated range below 4096, so the conversion is exact
        // and non-negative: its two's complement is itself.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        Ok(raw as u32)
    }
}

#[derive(Debug, Error)]
pub enum DynamixelChainError {
    #[error(transparent)]
    Binding(#[from] BindingError),
    #[error(transparent)]
    Body(#[from] BodyError),
    #[error(transparent)]
    Target(#[from] TargetError),
    #[error(
        "bus_id {0} is not on the dynamixel_serial bus or binds a model this crate does not know"
    )]
    Unqualified(u16),
    #[error("bus_id {0} is above the protocol's last device id, 252")]
    Ids(u16),
    #[error("bus_id {0}'s approach limit rounds to no limit or overflows its profile registers")]
    ApproachLimit(u16),
    #[error(
        "bus_id {0} turns more than once in its operating mode, which its body does not declare"
    )]
    MultiTurn(u16),
    #[error("bus_id {0} needs a Goal Current exactly when it runs current-based position")]
    GoalCurrent(u16),
    #[error("bus_id {bus_id} reads back {register} {read}, not the {written} written")]
    Unconfirmed {
        bus_id: u16,
        register: &'static str,
        written: u32,
        read: u32,
    },
    #[error("bus_id {0} could not be added to a group packet: its id is already in it")]
    GroupParam(u16),
    #[error("the chain's axes are on ports {0:?}; one chain drives one bus")]
    Ports(Vec<String>),
    #[error("bus_id {0} has an empty or out-of-resolution calibrated range")]
    Calibration(u16),
    #[error("{found} servo calibrations supplied for {expected} servos")]
    CalibrationCount { expected: usize, found: usize },
    #[error("bus_id {bus_id} reports model number {model}; its binding names {expected}")]
    UnqualifiedServo {
        bus_id: u16,
        model: u16,
        expected: u16,
    },
    #[error(
        "bus_id {bus_id} reports hardware error status {status:#04x}; reboot it before opening"
    )]
    HardwareError { bus_id: u16, status: u8 },
    #[error("bus_id {bus_id} target needs raw position {raw}, outside its calibrated range")]
    OutsideCalibration { bus_id: u16, raw: f64 },
    #[error("bus_id {bus_id}: {result:?}")]
    Comm { bus_id: u16, result: CommResult },
    #[error("the goal-position SYNC WRITE failed: {0:?}")]
    SyncWrite(CommResult),
    #[error("bus_id {bus_id} answered with error byte {error:#04x}")]
    ServoError { bus_id: u16, error: u8 },
    #[error(
        "bus_id {bus_id} reports homing offset {homing_offset}, range {range_min}..{range_max} \
         and drive mode {drive_mode}, which is not the calibration supplied"
    )]
    Uncalibrated {
        bus_id: u16,
        homing_offset: i32,
        range_min: u32,
        range_max: u32,
        drive_mode: u8,
    },
    #[error("an earlier stop is unproven; motion is refused until a stop is proven")]
    StopPending,
    #[error(
        "a stop reported faults (id, status) {faults:x?}; motion is refused until they are cleared"
    )]
    FaultedAtStop { faults: Vec<(u16, u8)> },
}

/// Bound Dynamixel servos on one serial link, in native state order.
pub struct DynamixelChain<P> {
    ph: Protocol2PacketHandler<P>,
    servos: Vec<DynamixelServo>,
    axes: Vec<BoundAxis>,
    commandable: Vec<CommandRange>,
    enabled: bool,
    stop_pending: bool,
    faults: Vec<(u16, u8)>,
    /// Every servo's position, read at open and after each stop; `None` once a command may
    /// have moved it, or when a read failed.
    observed: Option<Axes<f64>>,
    /// Each servo's reading from the last command's SYNC READ; none since open or a stop.
    command_readings: Option<Axes<AxisReading>>,
    /// The command's SYNC READ of every servo, its servos added at open and reused.
    reader: GroupSyncRead,
    /// The command's SYNC WRITE of every goal, refilled each command in buffers it keeps.
    writer: GroupSyncWrite,
}

/// An acknowledged exchange's `(result, error)`, as the chain requires it: success, no error.
fn expect(bus_id: u16, (result, error): (CommResult, u8)) -> Result<(), DynamixelChainError> {
    if result != CommResult::Success {
        return Err(DynamixelChainError::Comm { bus_id, result });
    }
    if error != 0 {
        return Err(DynamixelChainError::ServoError { bus_id, error });
    }
    Ok(())
}

/// A read-back that must equal what was written.
fn confirm(
    bus_id: u16,
    register: &'static str,
    written: u32,
    read: u32,
) -> Result<(), DynamixelChainError> {
    if read == written {
        Ok(())
    } else {
        Err(DynamixelChainError::Unconfirmed {
            bus_id,
            register,
            written,
            read,
        })
    }
}

/// What one servo must hold in its RAM before it is enabled.
#[derive(Clone, Copy)]
struct Limits {
    goal_current: Option<u16>,
    velocity: u32,
    acceleration: u32,
}

impl Limits {
    fn held_by(&self, read: &LimitsRead) -> bool {
        self.goal_current
            .is_none_or(|current| read.word(GOAL_CURRENT, 2) == Some(u32::from(current)))
            && read.word(PROFILE_ACCELERATION, 4) == Some(self.acceleration)
            && read.word(PROFILE_VELOCITY, 4) == Some(self.velocity)
    }
}

/// One servo's bytes from `Goal Current` (102) on.
struct LimitsRead(Vec<u8>);

impl LimitsRead {
    /// The little-endian `length`-byte register at `address`.
    fn word(&self, address: u16, length: usize) -> Option<u32> {
        let start = usize::from(address.checked_sub(GOAL_CURRENT)?);
        let bytes = self.0.get(start..start + length)?;
        Some(
            bytes
                .iter()
                .rev()
                .fold(0, |word, byte| (word << 8) | u32::from(*byte)),
        )
    }
}

/// A four-byte register's two's-complement value.
#[allow(clippy::cast_possible_wrap)]
const fn signed(value: u32) -> i32 {
    value as i32
}

/// Refuse axes that are not one valid chain on one port of known models at addressable ids,
/// before any byte.
pub(crate) fn qualify(axes: &[BoundAxis]) -> Result<(), DynamixelChainError> {
    validate_chain(axes)?;
    let mut ports: Vec<String> = axes.iter().map(|axis| axis.port.clone()).collect();
    ports.dedup();
    if ports.len() > 1 {
        return Err(DynamixelChainError::Ports(ports));
    }
    for axis in axes {
        let binding = axis.binding;
        if binding.bus != DYNAMIXEL_SERIAL || model_number(binding.model).is_none() {
            return Err(DynamixelChainError::Unqualified(binding.bus_id));
        }
        if binding.bus_id > u16::from(MAX_ID) {
            return Err(DynamixelChainError::Ids(binding.bus_id));
        }
    }
    Ok(())
}

/// Refuse a calibration whose range is empty or beyond the encoder's resolution.
pub(crate) fn check_calibration(
    bus_id: u16,
    calibration: MotorCalibration,
) -> Result<(), DynamixelChainError> {
    if calibration.range_min >= calibration.range_max
        || f64::from(calibration.range_max) >= RESOLUTION
    {
        return Err(DynamixelChainError::Calibration(bus_id));
    }
    Ok(())
}

/// The calibration a servo's registers hold, as `DynamixelMotorsBus.read_calibration` reads
/// it: `Homing Offset`, the position limits and `Drive Mode`. A value its field cannot hold is
/// [`DynamixelChainError::Uncalibrated`].
pub(crate) fn held_calibration<P: SerialPort>(
    ph: &mut Protocol2PacketHandler<P>,
    bus_id: u16,
    dxl_id: u8,
) -> Result<MotorCalibration, DynamixelChainError> {
    let mut read4 = |address: u16| {
        let (value, result, error) = ph.read4_byte_tx_rx(dxl_id, address);
        expect(bus_id, (result, error)).map(|()| value)
    };
    let homing_offset = signed(read4(HOMING_OFFSET)?);
    let range_min = read4(MIN_POSITION_LIMIT)?;
    let range_max = read4(MAX_POSITION_LIMIT)?;
    let (drive_mode, result, error) = ph.read1_byte_tx_rx(dxl_id, DRIVE_MODE);
    expect(bus_id, (result, error))?;
    let unheld = || DynamixelChainError::Uncalibrated {
        bus_id,
        homing_offset,
        range_min,
        range_max,
        drive_mode,
    };
    Ok(MotorCalibration {
        drive_mode: match drive_mode {
            0 => false,
            1 => true,
            _ => return Err(unheld()),
        },
        homing_offset: i16::try_from(homing_offset).map_err(|_| unheld())?,
        range_min: u16::try_from(range_min).map_err(|_| unheld())?,
        range_max: u16::try_from(range_max).map_err(|_| unheld())?,
    })
}

/// What a read that did not succeed says about its axis: faulted with the servo's error byte
/// (its hardware-error alert included) when it answered, else missing for why it did not.
pub(crate) const fn unanswered(result: CommResult, error: u8) -> AxisReading {
    match result {
        CommResult::Success => AxisReading::Faulted(error),
        CommResult::RxTimeout => AxisReading::Missing(MissingReason::Timeout),
        CommResult::RxCorrupt => AxisReading::Missing(MissingReason::Corrupt),
        CommResult::TxFail
        | CommResult::RxFail
        | CommResult::TxError
        | CommResult::NotAvailable => AxisReading::Missing(MissingReason::Unavailable),
    }
}

/// One servo's `Present Position` (132), one `READ`, as its axis's reading: fresh, missing for
/// a servo that is silent, corrupt or unreachable, or faulted with its error. Reading writes no
/// register, so nothing moves and torque stays as it was.
pub(crate) fn present_position<P: SerialPort>(
    ph: &mut Protocol2PacketHandler<P>,
    servo: &DynamixelServo,
) -> AxisReading {
    let (raw, result, error) = ph.read4_byte_tx_rx(servo.dxl_id(), PRESENT_POSITION);
    let received = Instant::now();
    if result != CommResult::Success || error != 0 {
        return unanswered(result, error);
    }
    AxisReading::Fresh {
        position: servo.joint_from_raw(signed(raw)),
        received,
    }
}

impl<P: SerialPort> DynamixelChain<P> {
    /// Validate every servo, prove each answers as its binding's model holding the calibration
    /// supplied and no hardware error, disable torque and configure every servo, and leave the
    /// chain disabled. With torque off, a held pose slumps under gravity.
    ///
    /// # Errors
    ///
    /// Refuses an invalid or unknown axis, an unaddressable id, an invalid calibration, a servo
    /// that does not answer or answers with an error, any servo whose model number is not its
    /// binding's or that reports a hardware error, and any whose registers disagree with its
    /// calibration.
    pub fn open(port: P, servos: Vec<DynamixelServo>) -> Result<Self, DynamixelChainError> {
        let axes: Vec<BoundAxis> = servos.iter().map(|servo| servo.axis.clone()).collect();
        qualify(&axes)?;
        for servo in &servos {
            let binding = servo.axis.binding;
            check_calibration(binding.bus_id, servo.calibration)?;
            if servo.profile().is_none() {
                return Err(DynamixelChainError::ApproachLimit(binding.bus_id));
            }
            let multi_turn = servo.operating_mode != OperatingMode::Position;
            if multi_turn && !servo.multi_turn {
                return Err(DynamixelChainError::MultiTurn(binding.bus_id));
            }
            if (servo.operating_mode == OperatingMode::CurrentPosition)
                != servo.goal_current.is_some()
            {
                return Err(DynamixelChainError::GoalCurrent(binding.bus_id));
            }
        }
        let commandable = servos.iter().map(DynamixelServo::commandable).collect();
        let mut chain = Self {
            ph: Protocol2PacketHandler::new(port),
            servos,
            axes,
            commandable,
            enabled: false,
            stop_pending: false,
            faults: Vec::new(),
            observed: None,
            command_readings: None,
            reader: GroupSyncRead::new(GOAL_CURRENT, LIMITS_READ_LENGTH),
            writer: GroupSyncWrite::new(GOAL_POSITION, 4),
        };
        for index in 0..chain.servos.len() {
            if !chain.reader.add_param(chain.servos[index].dxl_id()) {
                return Err(DynamixelChainError::GroupParam(
                    chain.servos[index].axis.binding.bus_id,
                ));
            }
        }
        chain.identify()?;
        chain.is_calibrated()?;
        chain.configure()?;
        Ok(chain)
    }

    fn ids(&self) -> Vec<(u16, u8)> {
        self.servos
            .iter()
            .map(|servo| (servo.axis.binding.bus_id, servo.dxl_id()))
            .collect()
    }

    /// Every servo answers its ping as its binding's model, with no hardware error.
    fn identify(&mut self) -> Result<(), DynamixelChainError> {
        let expected: Vec<(u16, u8, u16)> = self
            .servos
            .iter()
            .map(|servo| {
                let binding = servo.axis.binding;
                (
                    binding.bus_id,
                    servo.dxl_id(),
                    model_number(binding.model).unwrap_or(0),
                )
            })
            .collect();
        for (bus_id, dxl_id, expected) in expected {
            let (model, result, error) = self.ph.ping(dxl_id);
            // The alert bit is the hardware error the next read names.
            expect(
                bus_id,
                (result, error & !crate::dynamixel_sdk::ERRBIT_ALERT),
            )?;
            if model != expected {
                return Err(DynamixelChainError::UnqualifiedServo {
                    bus_id,
                    model,
                    expected,
                });
            }
            let (status, result, error) = self.ph.read1_byte_tx_rx(dxl_id, HARDWARE_ERROR_STATUS);
            expect(
                bus_id,
                (result, error & !crate::dynamixel_sdk::ERRBIT_ALERT),
            )?;
            if status != 0 {
                return Err(DynamixelChainError::HardwareError { bus_id, status });
            }
        }
        Ok(())
    }

    fn read4(&mut self, bus_id: u16, dxl_id: u8, address: u16) -> Result<u32, DynamixelChainError> {
        let (value, result, error) = self.ph.read4_byte_tx_rx(dxl_id, address);
        expect(bus_id, (result, error))?;
        Ok(value)
    }

    /// `DynamixelMotorsBus.is_calibrated` over `read_calibration`.
    fn is_calibrated(&mut self) -> Result<(), DynamixelChainError> {
        let ids: Vec<(u16, u8, MotorCalibration)> = self
            .servos
            .iter()
            .map(|servo| (servo.axis.binding.bus_id, servo.dxl_id(), servo.calibration))
            .collect();
        for (bus_id, dxl_id, calibration) in ids {
            let held = held_calibration(&mut self.ph, bus_id, dxl_id)?;
            if held != calibration {
                return Err(DynamixelChainError::Uncalibrated {
                    bus_id,
                    homing_offset: i32::from(held.homing_offset),
                    range_min: u32::from(held.range_min),
                    range_max: u32::from(held.range_max),
                    drive_mode: u8::from(held.drive_mode),
                });
            }
        }
        Ok(())
    }

    /// `configure_motors` and each servo's operating mode inside `torque_disabled`, leaving
    /// torque disabled.
    fn configure(&mut self) -> Result<(), DynamixelChainError> {
        let ids = self.ids();
        for (bus_id, dxl_id) in &ids {
            expect(
                *bus_id,
                self.ph.write1_byte_tx_rx(*dxl_id, TORQUE_ENABLE, 0),
            )?;
        }
        for (bus_id, dxl_id) in &ids {
            expect(
                *bus_id,
                self.ph
                    .write1_byte_tx_rx(*dxl_id, RETURN_DELAY_TIME, RETURN_DELAY),
            )?;
        }
        let modes: Vec<(u16, u8, u8)> = self
            .servos
            .iter()
            .map(|servo| {
                (
                    servo.axis.binding.bus_id,
                    servo.dxl_id(),
                    servo.operating_mode.register(),
                )
            })
            .collect();
        for (bus_id, dxl_id, mode) in modes {
            expect(
                bus_id,
                self.ph.write1_byte_tx_rx(dxl_id, OPERATING_MODE, mode),
            )?;
        }
        for index in 0..self.servos.len() {
            self.hold_limits(index, true)?;
        }
        self.observed = fresh_positions(&self.present_positions()).map(Axes::from);
        Ok(())
    }

    /// The `Goal Current` (where declared), `Profile Velocity` and `Profile Acceleration`
    /// servo `index` must hold.
    fn limits(&self, index: usize) -> Result<Limits, DynamixelChainError> {
        let servo = &self.servos[index];
        let (velocity, acceleration) = servo.profile().ok_or(
            DynamixelChainError::ApproachLimit(servo.axis.binding.bus_id),
        )?;
        Ok(Limits {
            goal_current: servo.goal_current,
            velocity,
            acceleration,
        })
    }

    /// Prove servo `index` holds its limits: read `Goal Current` through `Present Position`
    /// in one READ, and where they differ (or `rewrite`) write them and read again. All three
    /// are RAM (e-manual "Control Table": the RAM area is reset at power-on, `Profile
    /// Velocity` to 0, which is no limit at all, and `Goal Current` to `Current Limit`), so a
    /// servo whose power cycled while its link stayed up has lost them. One that does not read
    /// them back is refused.
    fn hold_limits(&mut self, index: usize, rewrite: bool) -> Result<(), DynamixelChainError> {
        let limits = self.limits(index)?;
        let (bus_id, dxl_id) = {
            let servo = &self.servos[index];
            (servo.axis.binding.bus_id, servo.dxl_id())
        };
        if !rewrite && limits.held_by(&self.read_limits(bus_id, dxl_id)?) {
            return Ok(());
        }
        if let Some(current) = limits.goal_current {
            expect(
                bus_id,
                self.ph.write2_byte_tx_rx(dxl_id, GOAL_CURRENT, current),
            )?;
        }
        expect(
            bus_id,
            self.ph
                .write4_byte_tx_rx(dxl_id, PROFILE_ACCELERATION, limits.acceleration),
        )?;
        expect(
            bus_id,
            self.ph
                .write4_byte_tx_rx(dxl_id, PROFILE_VELOCITY, limits.velocity),
        )?;
        let read = self.read_limits(bus_id, dxl_id)?;
        if let Some(current) = limits.goal_current {
            confirm(
                bus_id,
                "Goal Current",
                u32::from(current),
                read.word(GOAL_CURRENT, 2).unwrap_or(0),
            )?;
        }
        confirm(
            bus_id,
            "Profile Acceleration",
            limits.acceleration,
            read.word(PROFILE_ACCELERATION, 4).unwrap_or(0),
        )?;
        confirm(
            bus_id,
            "Profile Velocity",
            limits.velocity,
            read.word(PROFILE_VELOCITY, 4).unwrap_or(0),
        )?;
        Ok(())
    }

    /// One servo's registers from `Goal Current` (102) to `Present Position` (132..=135).
    fn read_limits(&mut self, bus_id: u16, dxl_id: u8) -> Result<LimitsRead, DynamixelChainError> {
        let (data, result, error) = self.ph.read_tx_rx(dxl_id, GOAL_CURRENT, LIMITS_READ_LENGTH);
        expect(bus_id, (result, error))?;
        if data.len() != usize::from(LIMITS_READ_LENGTH) {
            return Err(DynamixelChainError::Comm {
                bus_id,
                result: CommResult::RxCorrupt,
            });
        }
        Ok(LimitsRead(data.to_vec()))
    }

    /// Every servo's `Present_Position` (132..=135), one `READ` per servo, as an
    /// [`AxisReading`] per axis in native state order. Each answer is fresh: the packet handler
    /// clears the port before it writes each `READ`. A servo that is silent, answers corrupt or
    /// cannot be reached is missing; one that answers with an error, the hardware-error alert
    /// included, is faulted with it. The other servos' readings stand. Reading writes no
    /// register, so nothing moves and torque stays as it was.
    fn present_positions(&mut self) -> sx_embodiment_drivers::Axes<AxisReading> {
        self.servos
            .iter()
            .map(|servo| present_position(&mut self.ph, servo))
            .collect()
    }

    /// One SYNC READ (`GroupSyncRead`, as `sync_read("Present_Position")` reads it, widened)
    /// of every servo's `Goal Current` (102) through `Present_Position` (132..=135), as joint
    /// coordinates in native state order. A servo whose answer is missing or corrupt, that
    /// reports an error (the hardware-error alert included), or that no longer holds its limits
    /// is `NaN`, which no tracking bound admits; any of those marks the chain disabled, so the
    /// next enable proves every limit again and holds every servo where it rests. Each servo's
    /// [`AxisReading`] is kept as the command's readings: missing (`Timeout`; the vendor's read
    /// stops at the first servo that fails, so every servo is missing then), faulted with its
    /// error, or faulted with 0 for a limit it no longer holds.
    fn sync_read_positions(&mut self) -> Result<Axes<f64>, DynamixelChainError> {
        let _ = self.reader.tx_rx_packet(&mut self.ph);
        let reader = &self.reader;
        let received = Instant::now();
        let mut readings: Axes<AxisReading> = Axes::new();
        for index in 0..self.servos.len() {
            let limits = self.limits(index)?;
            let servo = &self.servos[index];
            let id = servo.dxl_id();
            let read = |address: u16, length: u16| reader.get_data(id, address, length);
            let held = limits.goal_current.map_or_else(
                || read(GOAL_CURRENT, 2).is_some(),
                |current| read(GOAL_CURRENT, 2) == Some(u32::from(current)),
            ) && read(PROFILE_ACCELERATION, 4) == Some(limits.acceleration)
                && read(PROFILE_VELOCITY, 4) == Some(limits.velocity);
            let reading = match (read(PRESENT_POSITION, 4), reader.get_error(id)) {
                (None, _) | (_, None) => AxisReading::Missing(MissingReason::Timeout),
                (Some(_), Some(error)) if error != 0 => AxisReading::Faulted(error),
                (Some(_), Some(_)) if !held => AxisReading::Faulted(0),
                (Some(raw), Some(_)) => AxisReading::Fresh {
                    position: servo.joint_from_raw(signed(raw)),
                    received,
                },
            };
            if !reading.is_fresh() {
                self.enabled = false;
            }
            readings.push(reading);
        }
        let observed = readings
            .iter()
            .map(|reading| reading.position().unwrap_or(f64::NAN))
            .collect();
        self.command_readings = Some(readings);
        Ok(observed)
    }

    /// `enable_torque`: `Torque_Enable` 1 on every servo, once every servo is proven to hold
    /// its limits and holds where it rests.
    fn enable_torque(&mut self) -> Result<(), DynamixelChainError> {
        for index in 0..self.servos.len() {
            self.hold_limits(index, false)?;
        }
        // Hold every servo where it is first, so enabling never drives toward a stale goal:
        // `Goal Position` keeps whatever it last held (e-manual `Goal Position(116)`), and the
        // instant torque comes on the servo drives toward it.
        for (bus_id, dxl_id) in self.ids() {
            let present = self.read4(bus_id, dxl_id, PRESENT_POSITION)?;
            expect(
                bus_id,
                self.ph.write4_byte_tx_rx(dxl_id, GOAL_POSITION, present),
            )?;
        }
        for (bus_id, dxl_id) in self.ids() {
            expect(bus_id, self.ph.write1_byte_tx_rx(dxl_id, TORQUE_ENABLE, 1))?;
        }
        Ok(())
    }

    /// The faults every stop since the last [`ActuatorChain::clear_faults`] reported, proven
    /// or not, as (bus id, hardware error status or error byte).
    #[must_use]
    pub fn faults(&self) -> &[(u16, u8)] {
        &self.faults
    }

    /// The servos, in native state order.
    #[must_use]
    pub fn servos(&self) -> &[DynamixelServo] {
        &self.servos
    }

    /// The serial link, for inspection.
    #[must_use]
    pub const fn port(&self) -> &P {
        self.ph.port()
    }

    pub const fn port_mut(&mut self) -> &mut P {
        self.ph.port_mut()
    }
}

impl<P: SerialPort> ActuatorChain for DynamixelChain<P> {
    type Error = DynamixelChainError;
    /// The SYNC WRITE that carries a command has no answer.
    type Feedback = ();

    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    fn commandable(&self) -> &[CommandRange] {
        &self.commandable
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt<()>, DynamixelChainError> {
        if self.stop_pending {
            return Err(DynamixelChainError::StopPending);
        }
        if !self.faults.is_empty() {
            return Err(DynamixelChainError::FaultedAtStop {
                faults: self.faults.clone(),
            });
        }
        if joints.len() != self.servos.len() {
            return Err(TargetError::Width {
                expected: self.servos.len(),
                actual: joints.len(),
            }
            .into());
        }
        let goals = self
            .servos
            .iter()
            .zip(joints)
            .map(|(servo, joint)| servo.goal_position(*joint))
            .collect::<Result<Axes<u32>, _>>()?;
        self.writer.clear_param();
        for (servo, goal) in self.servos.iter().zip(goals) {
            let (low, high) = (dxl_loword(goal), dxl_hiword(goal));
            if !self.writer.add_param(
                servo.dxl_id(),
                &[
                    crate::dynamixel_sdk::dxl_lobyte(low),
                    crate::dynamixel_sdk::dxl_hibyte(low),
                    crate::dynamixel_sdk::dxl_lobyte(high),
                    crate::dynamixel_sdk::dxl_hibyte(high),
                ],
            ) {
                return Err(DynamixelChainError::GroupParam(servo.axis.binding.bus_id));
            }
        }
        // From here the servos may move: what open or the last stop read no longer holds.
        self.observed = None;
        self.command_readings = None;
        if !self.enabled {
            self.enable_torque()?;
            self.enabled = true;
        }
        match self.writer.tx_packet(&mut self.ph) {
            CommResult::Success => Ok(Receipt {
                evidence: Evidence::DispatchAttempted,
                observed: self.sync_read_positions()?,
                feedback: sx_embodiment_drivers::Axes::new(),
            }),
            result => {
                self.enabled = false;
                Err(DynamixelChainError::SyncWrite(result))
            }
        }
    }

    fn stop(&mut self) -> StopReport {
        fn note(fault: (u16, u8), faults: &mut Vec<(u16, u8)>) {
            if fault.1 != 0 && !faults.contains(&fault) {
                faults.push(fault);
            }
        }
        self.enabled = false;
        self.stop_pending = true;
        let ids = self.ids();
        let mut report = StopReport::default();
        for (bus_id, dxl_id) in &ids {
            let (result, error) = self.ph.write1_byte_tx_rx(*dxl_id, TORQUE_ENABLE, 0);
            if result == CommResult::Success {
                note((*bus_id, error), &mut report.faults);
            } else {
                report.unproven.push(*bus_id);
            }
        }
        for (bus_id, dxl_id) in ids {
            let (data, result, error) = self.ph.read_tx_rx(dxl_id, TORQUE_ENABLE, STOP_READ_LENGTH);
            let proven = match (result, data.as_slice()) {
                (CommResult::Success, [torque, .., hardware]) => {
                    note(
                        (bus_id, if *hardware != 0 { *hardware } else { error }),
                        &mut report.faults,
                    );
                    *torque == 0
                }
                _ => false,
            };
            if !proven && !report.unproven.contains(&bus_id) {
                report.unproven.push(bus_id);
            }
        }
        self.stop_pending = !report.is_proven();
        for fault in &report.faults {
            if !self.faults.contains(fault) {
                self.faults.push(*fault);
            }
        }
        self.observed = fresh_positions(&self.present_positions()).map(Axes::from);
        self.command_readings = None;
        report
    }

    /// Every servo's `Present_Position`, freshly read; see `present_positions`.
    fn read_positions(&mut self) -> sx_embodiment_drivers::Axes<AxisReading> {
        self.present_positions()
    }

    fn command_readings(&self) -> Option<Axes<AxisReading>> {
        self.command_readings
    }

    fn last_observed(&self) -> Option<Axes<f64>> {
        self.observed
    }

    /// The longest [`ActuatorChain::stop`] takes on this chain: three exchanges per servo, each
    /// bounded by the link's [`SerialPort::timeout`].
    fn stop_budget(&self) -> Duration {
        let servos = u32::try_from(self.servos.len()).unwrap_or(u32::MAX);
        self.ph
            .port()
            .timeout()
            .saturating_mul(STOP_EXCHANGES_PER_SERVO.saturating_mul(servos))
    }

    /// Release the latched stop faults, letting motion resume. Whoever calls this has decided
    /// the servos that reported them are fit to move.
    fn clear_faults(&mut self) {
        self.faults.clear();
    }

    /// Each servo no faster or harder than its `Profile Velocity` and `Profile Acceleration`
    /// move its joint, through its calibrated map (a percent servo's calibrated span is not its
    /// binding's reduction): as `sx_feetech_serial`'s chain bounds its servos.
    fn tracking_ceilings(&self) -> Vec<(usize, sx_embodiment_drivers::TrackingCeiling)> {
        self.servos
            .iter()
            .enumerate()
            .filter_map(|(index, servo)| {
                let (velocity, acceleration) = servo.profile()?;
                let joint_per_shaft = sx_embodiment_drivers::lerobot::joint_per_shaft_radian(
                    &servo.calibration,
                    RESOLUTION,
                    |raw| servo.joint_from_raw(raw),
                )?;
                Some((
                    index,
                    sx_embodiment_drivers::TrackingCeiling::of_shaft(
                        f64::from(velocity) * PROFILE_VELOCITY_STEP_RAD_PER_S,
                        f64::from(acceleration) * PROFILE_ACCELERATION_STEP_RAD_PER_S2,
                        joint_per_shaft,
                    ),
                ))
            })
            .collect()
    }
}

impl ChainError for DynamixelChainError {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::Body(_)
            | Self::Target(_)
            | Self::OutsideCalibration { .. }
            | Self::CalibrationCount { .. }
            | Self::GroupParam(_)
            | Self::ApproachLimit(_)
            | Self::MultiTurn(_)
            | Self::GoalCurrent(_)
            | Self::Ports(_)
            | Self::StopPending
            | Self::FaultedAtStop { .. } => ChainFailure::Refused,
            Self::Comm {
                result: CommResult::RxTimeout,
                ..
            } => ChainFailure::Timeout,
            Self::Comm {
                result: CommResult::TxFail | CommResult::RxFail,
                ..
            }
            | Self::SyncWrite(CommResult::TxFail) => ChainFailure::Unavailable,
            _ => ChainFailure::Fault,
        }
    }
}
