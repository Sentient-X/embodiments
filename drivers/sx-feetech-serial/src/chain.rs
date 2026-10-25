//! A chain of bound Feetech STS3215 servos as one `ActuatorChain`, driven through
//! [`crate::scservo_sdk`].
//!
//! Every bus operation is an `scservo_sdk` call. The order they are called in, and what a
//! goal position means on the wire, are those of the driver the SO-101 is run with today,
//! `LeRobot`'s Feetech bus (<https://github.com/huggingface/lerobot> at `v0.6.0`,
//! `30da8e687a6dfc617fcd94afc367ac7071c376ce`):
//!
//! - Open follows `SOFollower.connect` (`src/lerobot/robots/so_follower/so_follower.py:91-108`),
//!   without its interactive calibration:
//!   1. It reads each servo's model number (`read2ByteTxRx` at `SMS_STS_MODEL_L`) and refuses
//!      anything but the STS3215's 777 (`src/lerobot/motors/feetech/tables.py`,
//!      `MODEL_NUMBER_TABLE`).
//!   2. It ports `FeetechMotorsBus.is_calibrated` and `read_calibration`
//!      (`src/lerobot/motors/feetech/feetech.py:228-266`): per servo, `Min_Position_Limit`
//!      (`SMS_STS_MIN_ANGLE_LIMIT_L`), `Max_Position_Limit` (`SMS_STS_MAX_ANGLE_LIMIT_L`) and
//!      `Homing_Offset` (`SMS_STS_OFS_L`, sign-magnitude at bit 11, `tables.py`
//!      `STS_SMS_SERIES_ENCODINGS_TABLE`, decoded by the vendor's `scs_tohost`), each compared
//!      with the [`MotorCalibration`] the caller supplied. Where `LeRobot` would recalibrate
//!      interactively, open refuses ([`FeetechChainError::Uncalibrated`]): the station never
//!      writes a calibration.
//!   3. It ports `SOFollower.configure` (`so_follower.py:159-173`) inside
//!      `torque_disabled` (`src/lerobot/motors/motors_bus.py:676-690`): `disable_torque` on
//!      every servo; then `configure_motors` (`feetech.py:209-226`) per servo:
//!      `Return_Delay_Time` 0, `Maximum_Acceleration` 254, `Acceleration` 254, and the
//!      `Phase` register read with its bit 4 cleared if set; then per servo `Operating_Mode`
//!      `POSITION` (0), `P_Coefficient` 16, `I_Coefficient` 0, `D_Coefficient` 32, and for a
//!      servo with a [`TorqueProtection`] (the SO-101 gripper) `Max_Torque_Limit` 500,
//!      `Protection_Current` 250 and `Overload_Torque` 25. Every write is acknowledged and an
//!      error flag refuses, as `LeRobot`'s `write` raises. The addresses the vendor's table
//!      does not carry are `LeRobot`'s `STS_SMS_SERIES_CONTROL_TABLE`. Where `torque_disabled`
//!      re-enables torque on leaving, open leaves the chain disabled: the first command
//!      enables it.
//! - Command enables a disabled chain as `FeetechMotorsBus.enable_torque` does
//!   (`src/lerobot/motors/feetech/feetech.py:302-305`): per servo, `Torque_Enable` 1, then
//!   `Lock` 1 (`sms_sts.LockEprom`), each acknowledged. It then sends one broadcast SYNC WRITE
//!   of every goal position (`GroupSyncWrite` at `SMS_STS_GOAL_POSITION_L`, two bytes), as
//!   `SOFollower.send_action` does (`sync_write("Goal_Position", ...)`,
//!   `src/lerobot/robots/so_follower/so_follower.py:231`). The bus answers nothing to it, so a
//!   command's receipt claims only a dispatch. Right after it, one SYNC READ of every servo's
//!   `Present_Position` (`GroupSyncRead` at `SMS_STS_PRESENT_POSITION_L`, two bytes, as
//!   `sms_sts/sync_read.py` reads it) is the receipt's observed joints, in native state order;
//!   a servo that misses it is reported `NaN`, so a tracking check fails closed.
//! - Stop disables as `disable_torque` does (`feetech.py:291-294`): per servo, `Torque_Enable`
//!   0, then `Lock` 0 (`sms_sts.unLockEprom`), each acknowledged, on every servo whatever an
//!   earlier one answered. Then it reads every servo's `Torque_Enable` back
//!   (`read1ByteTxRx`); a servo is proven stopped only when it reads 0.
//! - After open and after every stop it reads each servo's `Present_Position`
//!   (`read2ByteTxRx`, sign-magnitude at bit 15), as `LeRobot` reads an observation, and
//!   reports it as [`ActuatorChain::last_observed`] until the next command, so a caller can
//!   start from where the body rests. A servo resting outside its calibrated range is reported
//!   there, unclamped. A position read that fails reports nothing; it does not fail open.
//!
//! A goal position is computed from a joint target in three steps:
//!
//! 1. The registry's drive map gives the actuator coordinate
//!    (`BoundAxis::actuator_from_joint`: `sign * direction * (joint - zero) * reduction`).
//! 2. That coordinate becomes `LeRobot`'s wire value. For a [`MotorNormMode::Degrees`] servo it
//!    is in degrees. For a [`MotorNormMode::Range0100`] servo (the gripper) it is the percent
//!    of the axis's declared range, mapped through the binding.
//! 3. `MotorsBus._unnormalize` (`src/lerobot/motors/motors_bus.py:879-907`) takes it to raw
//!    ticks with the servo's calibration. Degrees are `int(val * 4095 / 360 + (min + max) / 2)`,
//!    with no drive mode applied. Percent is `int(clamp(val or 100 - val, 0, 100) / 100 *
//!    (max - min) + min)`, inverted when `drive_mode` is set. The vendor's `scs_toscs(ticks, 15)`
//!    encodes the result as the register's sign-magnitude value.
//!
//! What this chain adds over those calls, and why:
//!
//! - **The servo bounds its own approach speed, where its body declares one.** At the end of
//!   open, with torque still off, the `Acceleration` (41) and `Goal_Speed` (46, `sms_sts.py`'s
//!   memory table) of each servo whose [`ServoConfiguration`] carries an approach limit are
//!   written from its [`ApproachLimit`] (the joint's rad/s and rad/s², through the binding's
//!   reduction, in the vendor's units of 0.732 rpm and 8.7 °/s² per step, rounded down) and
//!   read back; open refuses a servo that does not hold them. `LeRobot` leaves `Goal_Speed` at
//!   0, which the servo reads as its full speed, and writes `Acceleration` 254; here they
//!   replace that, so a goal the servo's own position limits clamp, or any first goal far from
//!   where it rests, is approached at the limit: [`ActuatorChain::approach_speed_limited`]
//!   reports it. A servo without one (the SO-101 follower's) keeps `LeRobot`'s registers and
//!   reports `false`, so a governor refuses to move it from outside its range.
//! - **Enabling never drives toward a stale goal.** Before `Torque_Enable` 1, each servo's
//!   `Present_Position` is read and written to its `Goal_Position`, so the instant torque comes
//!   on it holds where it rests; `LeRobot` enables with whatever goal the register last held.
//! - It refuses before any frame: a wrong width, a target outside the axis's commandable range
//!   or not finite, a magnitude that reaches the sign bit (`LeRobot`'s `encode_sign_magnitude`
//!   raises there; the vendor's `scs_toscs` would set overlapping bits), a model or bus this
//!   crate is not qualified for, an address the protocol cannot carry, and an empty or
//!   out-of-resolution calibration.
//! - A target is converted at float32 precision, as the station's earlier Python follower
//!   did: `sx_drivers.feetech.radians_to_wire` (`packages/sx-drivers/sx_drivers/feetech.py:
//!   291-313` in `Sentient-X/sx` at `d6d4a3ea6`, the source the station's golden was first
//!   minted from) took a float32 target and compared it with each bound at float32 under
//!   `NumPy`'s promotion rules. Each axis's commandable range is the union of its declared
//!   bounds and those bounds rounded to float32: `[min(lower, f32(lower)), max(upper,
//!   f32(upper))]`. Every declared bound is admitted, and so is a bound's float32 rounding
//!   where that lands outside it, by less than half a float32 step, which moves no servo by a
//!   tick; that much departs from the trait's "inside the declared bounds". Admission is that
//!   range, in `f64`, exactly as [`ActuatorChain::commandable`] reports it, and every later step
//!   runs on the target rounded to `f32`, in `f64`, as Python does; rounding is monotone, so an
//!   admitted target rounds to within the bounds' float32 roundings. This matches the Python
//!   follower on every target it could receive (it received float32 values, so a bound's
//!   rounding at most). It refuses an `f64` target outside both a bound and that bound's
//!   rounding, which the Python follower never saw: rounded to float32 first, such a target
//!   could have been admitted there.
//! - A stop is proven servo by servo by the torque read-back, not by the acknowledgements. The
//!   error bytes the servos report while stopping, such as overload or overheating, are faults
//!   of a stop that may still be proven; they are latched, and every command is refused until
//!   [`FeetechChain::clear_faults`] clears them. After an unproven stop every command is refused
//!   until a stop is proven.

use std::time::Duration;

pub use sx_embodiment_drivers::lerobot::{MotorCalibration, MotorNormMode};
use sx_embodiment_drivers::lerobot::{normalize, unnormalize};
use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{
    ActuatorBus, ActuatorChain, ActuatorModel, BindingError, BodyError, BoundAxis, ChainError,
    ChainFailure, CommandRange, Evidence, Receipt, StopReport, TargetError, validate_chain,
};
use thiserror::Error;

use crate::scservo_sdk::{
    CommResult, GroupSyncRead, GroupSyncWrite, MAX_ID, ProtocolPacketHandler, SMS_STS_ACC,
    SMS_STS_GOAL_POSITION_L, SMS_STS_GOAL_SPEED_L, SMS_STS_MAX_ANGLE_LIMIT_L,
    SMS_STS_MIN_ANGLE_LIMIT_L, SMS_STS_MODE, SMS_STS_MODEL_L, SMS_STS_OFS_L,
    SMS_STS_PRESENT_POSITION_L, SMS_STS_TORQUE_ENABLE, SmsSts,
};

/// `ActuatorModel.FEETECH_STS3215` in the registry.
pub const FEETECH_STS3215: ActuatorModel = ActuatorModel::new("feetech_sts3215");
/// `ActuatorBus.FEETECH_SERIAL` in the registry.
pub const FEETECH_SERIAL: ActuatorBus = ActuatorBus::new("feetech_serial");
/// The STS3215's model number (`LeRobot` `MODEL_NUMBER_TABLE["sts3215"]`).
pub const STS3215_MODEL_NUMBER: u16 = 777;
/// Encoder steps per turn (`LeRobot` `MODEL_RESOLUTION["sts3215"]`).
const RESOLUTION: f64 = 4096.0;
/// `FeetechMotorsBus.apply_drive_mode`: an inverted percent servo is inverted in software.
const APPLY_DRIVE_MODE: bool = true;
/// `Goal_Position`'s sign bit: `sms_sts`'s `scs_toscs(position, 15)`, and `LeRobot`'s
/// `STS_SMS_SERIES_ENCODINGS_TABLE`.
const GOAL_POSITION_SIGN_BIT: u32 = 15;
/// The register's largest magnitude below its sign bit.
const GOAL_POSITION_MAGNITUDE: f64 = 32_767.0;
/// Exchanges a stop makes per servo: torque off, unlock, the torque read-back, and the
/// position read.
const STOP_EXCHANGES_PER_SERVO: u32 = 4;
/// `Present_Position`'s sign bit (`LeRobot` `STS_SMS_SERIES_ENCODINGS_TABLE`).
const PRESENT_POSITION_SIGN_BIT: u32 = 15;
/// `Homing_Offset`'s sign bit (`LeRobot` `STS_SMS_SERIES_ENCODINGS_TABLE`).
const HOMING_OFFSET_SIGN_BIT: u32 = 11;

// The control-table addresses `sms_sts.py` does not name, from `LeRobot`'s
// `STS_SMS_SERIES_CONTROL_TABLE` (`src/lerobot/motors/feetech/tables.py:41-100`).
const RETURN_DELAY_TIME: u8 = 7;
const MAX_TORQUE_LIMIT: u8 = 16;
const PHASE: u8 = 18;
const P_COEFFICIENT: u8 = 21;
const D_COEFFICIENT: u8 = 22;
const I_COEFFICIENT: u8 = 23;
const PROTECTION_CURRENT: u8 = 28;
const OVERLOAD_TORQUE: u8 = 36;
const MAXIMUM_ACCELERATION: u8 = 85;

/// `configure_motors`' defaults (`feetech.py:209`) and `OperatingMode.POSITION`.
const RETURN_DELAY: u8 = 0;
const ACCELERATION: u8 = 254;
const OPERATING_MODE_POSITION: u8 = 0;
/// `Phase` bit 4: angle feedback mode, cleared on the STS3215 (`feetech.py:219-225`).
const PHASE_ANGLE_FEEDBACK: u8 = 0x10;

/// The torque and current limits `SOFollower.configure` writes to the gripper "to avoid
/// burnout".
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TorqueProtection {
    pub max_torque_limit: u16,
    pub protection_current: u16,
    pub overload_torque: u8,
}

/// The position loop `SOFollower.configure` writes to every servo, and the speed the servo
/// itself approaches any goal at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoConfiguration {
    pub p_coefficient: u8,
    pub i_coefficient: u8,
    pub d_coefficient: u8,
    pub protection: Option<TorqueProtection>,
    /// The speed the servo itself approaches any goal at, for a body that declares one;
    /// `None` leaves `LeRobot`'s registers (`Goal_Speed` 0, full speed; `Acceleration` 254).
    pub approach: Option<ApproachLimit>,
}

/// The fastest a servo moves toward any goal, written to its own `Goal_Speed` and
/// `Acceleration` at open, in the joint's units: rad/s and rad/s².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ApproachLimit {
    pub speed: f64,
    pub acceleration: f64,
}

/// The static upper body's bring-up speed (its received driver's 0.25 rad/s), reached over
/// half a second.
pub const BRING_UP_APPROACH: ApproachLimit = ApproachLimit {
    speed: 0.25,
    acceleration: 0.5,
};

/// One `Goal_Speed` step: 0.732 rpm, in rad/s (`sms_sts/write.py`: "V=60 * 0.732=43.92rpm").
pub const SPEED_STEP_RAD_PER_S: f64 = 0.732 * std::f64::consts::TAU / 60.0;
/// One `Acceleration` step: 8.7 °/s², in rad/s² (`sms_sts/write.py`: "A=50 * 8.7deg/s^2").
pub const ACCELERATION_STEP_RAD_PER_S2: f64 = 8.7 * std::f64::consts::PI / 180.0;
/// `Goal_Speed`'s largest magnitude below its sign bit.
const GOAL_SPEED_MAX: f64 = 32_767.0;

impl ApproachLimit {
    /// The `Goal_Speed` and `Acceleration` register values that move an actuator geared at
    /// `reduction` no faster than this, rounded down; `None` when a value rounds to 0, which
    /// the servo reads as no limit at all, or does not fit its register.
    #[must_use]
    pub fn registers(&self, reduction: f64) -> Option<(u16, u8)> {
        let speed = (self.speed * reduction / SPEED_STEP_RAD_PER_S).floor();
        let acceleration = (self.acceleration * reduction / ACCELERATION_STEP_RAD_PER_S2).floor();
        if !(1.0..=GOAL_SPEED_MAX).contains(&speed) || !(1.0..=254.0).contains(&acceleration) {
            return None;
        }
        // Both are integers checked inside their registers just above.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        Some((speed as u16, acceleration as u8))
    }
}

/// One bound axis with what its Feetech servo needs beyond the registry's binding.
#[derive(Clone, Debug, PartialEq)]
pub struct FeetechServo {
    pub axis: BoundAxis,
    pub norm_mode: MotorNormMode,
    pub calibration: MotorCalibration,
    pub configuration: ServoConfiguration,
}

impl FeetechServo {
    fn scs_id(&self) -> u8 {
        // Open refuses an address above `MAX_ID`, so the narrowing never saturates.
        u8::try_from(self.axis.binding.bus_id).unwrap_or(u8::MAX)
    }

    /// The joint range this servo is commanded within: its declared bounds, widened to their
    /// float32 roundings where those land outside them.
    #[must_use]
    pub fn commandable(&self) -> CommandRange {
        CommandRange {
            lower: self.axis.lower.min(float32(self.axis.lower)),
            upper: self.axis.upper.max(float32(self.axis.upper)),
        }
    }

    /// The goal-position register value that drives this servo to `joint`.
    ///
    /// # Errors
    ///
    /// Refuses a target outside the commandable range or not finite, and a raw value whose
    /// magnitude reaches the sign bit.
    pub fn goal_position(&self, joint: f64) -> Result<u16, FeetechChainError> {
        let axis = &self.axis;
        if !self.commandable().admits(joint) {
            return Err(TargetError::OutOfBounds {
                joint: axis.joint.clone(),
                value: joint,
            }
            .into());
        }
        let single = float32(joint);
        let binding = axis.binding;
        let ends = [
            axis.actuator_from_joint(axis.lower),
            axis.actuator_from_joint(axis.upper),
        ];
        let raw = unnormalize(
            self.norm_mode,
            &self.calibration,
            APPLY_DRIVE_MODE,
            RESOLUTION,
            axis.actuator_from_joint(single),
            (ends[0].min(ends[1]), ends[0].max(ends[1])),
        );
        if raw.abs() > GOAL_POSITION_MAGNITUDE {
            return Err(FeetechChainError::Unencodable(binding.bus_id));
        }
        // Both conversions are exact: `raw` is an integer below 2^15 in magnitude, and so is
        // its sign-magnitude encoding below 2^16.
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let encoded =
            ProtocolPacketHandler::<()>::scs_toscs(raw as i32, GOAL_POSITION_SIGN_BIT) as u16;
        Ok(encoded)
    }
}

impl FeetechServo {
    /// The joint coordinate a `Present_Position` reading of `raw` ticks is at, not clamped to
    /// the calibrated range ([`sx_embodiment_drivers::lerobot::normalize`]).
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
}

/// `value` rounded to float32, the precision targets are converted at.
fn float32(value: f64) -> f64 {
    // Rounding to float32 is the intended conversion.
    #[allow(clippy::cast_possible_truncation)]
    let single = value as f32;
    f64::from(single)
}

#[derive(Debug, Error)]
pub enum FeetechChainError {
    #[error(transparent)]
    Binding(#[from] BindingError),
    #[error(transparent)]
    Body(#[from] BodyError),
    #[error(transparent)]
    Target(#[from] TargetError),
    #[error(
        "bus_id {0} is not on the feetech_serial bus or binds a model this crate is not qualified for"
    )]
    Unqualified(u16),
    #[error("bus_id {0} is above the protocol's last servo id, 252")]
    Ids(u16),
    #[error("{found} servo calibrations supplied for {expected} servos")]
    CalibrationCount { expected: usize, found: usize },
    #[error("bus_id {0}'s approach limit rounds to no limit or overflows its registers")]
    ApproachLimit(u16),
    #[error(
        "bus_id {bus_id} reads back Goal_Speed {speed} and Acceleration {acceleration}, not the \
         approach limit written"
    )]
    ApproachUnconfirmed {
        bus_id: u16,
        speed: u16,
        acceleration: u8,
    },
    #[error("bus_id {0} could not be added to a group packet: its id is already in it")]
    GroupParam(u16),
    #[error("the chain's axes are on ports {0:?}; one chain drives one bus")]
    Ports(Vec<String>),
    #[error("bus_id {0} has an empty or out-of-resolution calibrated range")]
    Calibration(u16),
    #[error("bus_id {bus_id} reports model {model}; only the STS3215 (777) is qualified")]
    UnqualifiedServo { bus_id: u16, model: u16 },
    #[error("bus_id {0} target does not fit its sign-magnitude register")]
    Unencodable(u16),
    #[error("bus_id {bus_id}: {result:?}")]
    Comm { bus_id: u16, result: CommResult },
    #[error("the goal-position SYNC WRITE failed: {0:?}")]
    SyncWrite(CommResult),
    #[error("bus_id {bus_id} reported error flags {error:#04x}")]
    ServoError { bus_id: u16, error: u8 },
    #[error(
        "bus_id {bus_id} reports homing offset {homing_offset} and range {range_min}..{range_max}, \
         which is not the calibration supplied"
    )]
    Uncalibrated {
        bus_id: u16,
        homing_offset: i32,
        range_min: u16,
        range_max: u16,
    },
    #[error("an earlier stop is unproven; motion is refused until a stop is proven")]
    StopPending,
    #[error(
        "a stop reported status flags (id, flags) {faults:x?}; motion is refused until they \
         are cleared"
    )]
    FaultedAtStop { faults: Vec<(u16, u8)> },
}

/// Bound Feetech servos on one serial link, in native state order.
pub struct FeetechChain<P> {
    sms: SmsSts<P>,
    servos: Vec<FeetechServo>,
    axes: Vec<BoundAxis>,
    commandable: Vec<CommandRange>,
    enabled: bool,
    stop_pending: bool,
    faults: Vec<(u16, u8)>,
    /// Every servo's position, read at open and after each stop; `None` once a command may
    /// have moved it, or when a read failed.
    observed: Option<Vec<f64>>,
    /// Per servo, whether its approach limit was written and read back.
    approach_limited: Vec<bool>,
}

/// An acknowledged exchange's `(result, error)`, as the chain requires it: success, no flags.
fn expect(bus_id: u16, (result, error): (CommResult, u8)) -> Result<(), FeetechChainError> {
    if result != CommResult::Success {
        return Err(FeetechChainError::Comm { bus_id, result });
    }
    if error != 0 {
        return Err(FeetechChainError::ServoError { bus_id, error });
    }
    Ok(())
}

impl<P: SerialPort> FeetechChain<P> {
    /// Validate every servo, prove each answers as a qualified STS3215 holding the calibration
    /// supplied, disable torque and configure every servo, and leave the chain disabled.
    /// The arm must be at rest: with torque off, a held pose slumps under gravity.
    ///
    /// # Errors
    ///
    /// Refuses an invalid or unqualified axis, an unaddressable id, an invalid calibration, a
    /// servo that does not answer or reports error flags, any servo whose model is not the
    /// STS3215, and any servo whose registers disagree with its calibration.
    pub fn open(port: P, servos: Vec<FeetechServo>) -> Result<Self, FeetechChainError> {
        let axes: Vec<BoundAxis> = servos.iter().map(|servo| servo.axis.clone()).collect();
        validate_chain(&axes)?;
        let mut ports: Vec<String> = axes.iter().map(|axis| axis.port.clone()).collect();
        ports.dedup();
        if ports.len() > 1 {
            return Err(FeetechChainError::Ports(ports));
        }
        for servo in &servos {
            let binding = servo.axis.binding;
            if binding.bus != FEETECH_SERIAL || binding.model != FEETECH_STS3215 {
                return Err(FeetechChainError::Unqualified(binding.bus_id));
            }
            if binding.bus_id > u16::from(MAX_ID) {
                return Err(FeetechChainError::Ids(binding.bus_id));
            }
            let MotorCalibration {
                range_min,
                range_max,
                ..
            } = servo.calibration;
            if range_min >= range_max || f64::from(range_max) >= RESOLUTION {
                return Err(FeetechChainError::Calibration(binding.bus_id));
            }
            if servo
                .configuration
                .approach
                .is_some_and(|approach| approach.registers(binding.reduction).is_none())
            {
                return Err(FeetechChainError::ApproachLimit(binding.bus_id));
            }
        }
        let mut sms = SmsSts::new(port);
        for servo in &servos {
            let bus_id = servo.axis.binding.bus_id;
            let (model, result, error) = sms.ph.read2_byte_tx_rx(servo.scs_id(), SMS_STS_MODEL_L);
            expect(bus_id, (result, error))?;
            if model != STS3215_MODEL_NUMBER {
                return Err(FeetechChainError::UnqualifiedServo { bus_id, model });
            }
        }
        let commandable = servos.iter().map(FeetechServo::commandable).collect();
        let mut chain = Self {
            sms,
            servos,
            axes,
            commandable,
            enabled: false,
            stop_pending: false,
            faults: Vec::new(),
            observed: None,
            approach_limited: Vec::new(),
        };
        chain.is_calibrated()?;
        chain.configure()?;
        chain.observed = chain.read_positions();
        Ok(chain)
    }

    fn read2(&mut self, bus_id: u16, scs_id: u8, address: u8) -> Result<u16, FeetechChainError> {
        let (value, result, error) = self.sms.ph.read2_byte_tx_rx(scs_id, address);
        expect(bus_id, (result, error))?;
        Ok(value)
    }

    fn write1(
        &mut self,
        bus_id: u16,
        scs_id: u8,
        address: u8,
        value: u8,
    ) -> Result<(), FeetechChainError> {
        expect(
            bus_id,
            self.sms.ph.write1_byte_tx_rx(scs_id, address, value),
        )
    }

    /// `FeetechMotorsBus.is_calibrated` over `read_calibration`: every servo's position limits
    /// and homing offset equal the calibration supplied.
    fn is_calibrated(&mut self) -> Result<(), FeetechChainError> {
        let ids: Vec<(u16, u8, MotorCalibration)> = self
            .servos
            .iter()
            .map(|servo| (servo.axis.binding.bus_id, servo.scs_id(), servo.calibration))
            .collect();
        for (bus_id, scs_id, calibration) in ids {
            let range_min = self.read2(bus_id, scs_id, SMS_STS_MIN_ANGLE_LIMIT_L)?;
            let range_max = self.read2(bus_id, scs_id, SMS_STS_MAX_ANGLE_LIMIT_L)?;
            let homing_offset = ProtocolPacketHandler::<()>::scs_tohost(
                self.read2(bus_id, scs_id, SMS_STS_OFS_L)?,
                HOMING_OFFSET_SIGN_BIT,
            );
            if range_min != calibration.range_min
                || range_max != calibration.range_max
                || homing_offset != i32::from(calibration.homing_offset)
            {
                return Err(FeetechChainError::Uncalibrated {
                    bus_id,
                    homing_offset,
                    range_min,
                    range_max,
                });
            }
        }
        Ok(())
    }

    /// `SOFollower.configure` inside `torque_disabled`, leaving torque disabled.
    fn configure(&mut self) -> Result<(), FeetechChainError> {
        let ids: Vec<(u16, u8, ServoConfiguration)> = self
            .servos
            .iter()
            .map(|servo| {
                (
                    servo.axis.binding.bus_id,
                    servo.scs_id(),
                    servo.configuration,
                )
            })
            .collect();
        for (bus_id, scs_id, _) in &ids {
            self.write1(*bus_id, *scs_id, SMS_STS_TORQUE_ENABLE, 0)?;
            expect(*bus_id, self.sms.un_lock_eprom(*scs_id))?;
        }
        // configure_motors
        for (bus_id, scs_id, _) in &ids {
            self.write1(*bus_id, *scs_id, RETURN_DELAY_TIME, RETURN_DELAY)?;
            self.write1(*bus_id, *scs_id, MAXIMUM_ACCELERATION, ACCELERATION)?;
            self.write1(*bus_id, *scs_id, SMS_STS_ACC, ACCELERATION)?;
            let (phase, result, error) = self.sms.ph.read1_byte_tx_rx(*scs_id, PHASE);
            expect(*bus_id, (result, error))?;
            if phase & PHASE_ANGLE_FEEDBACK != 0 {
                self.write1(*bus_id, *scs_id, PHASE, phase & !PHASE_ANGLE_FEEDBACK)?;
            }
        }
        for (bus_id, scs_id, configuration) in ids {
            self.write1(bus_id, scs_id, SMS_STS_MODE, OPERATING_MODE_POSITION)?;
            self.write1(bus_id, scs_id, P_COEFFICIENT, configuration.p_coefficient)?;
            self.write1(bus_id, scs_id, I_COEFFICIENT, configuration.i_coefficient)?;
            self.write1(bus_id, scs_id, D_COEFFICIENT, configuration.d_coefficient)?;
            if let Some(protection) = configuration.protection {
                let ph = &mut self.sms.ph;
                expect(
                    bus_id,
                    ph.write2_byte_tx_rx(scs_id, MAX_TORQUE_LIMIT, protection.max_torque_limit),
                )?;
                expect(
                    bus_id,
                    ph.write2_byte_tx_rx(scs_id, PROTECTION_CURRENT, protection.protection_current),
                )?;
                self.write1(bus_id, scs_id, OVERLOAD_TORQUE, protection.overload_torque)?;
            }
        }
        self.limit_approach()?;
        Ok(())
    }

    /// Write the `Acceleration` and `Goal_Speed` of every servo that declares an approach limit
    /// and read both back, so the servo itself moves slowly toward whatever goal it is given.
    fn limit_approach(&mut self) -> Result<(), FeetechChainError> {
        self.approach_limited = vec![false; self.servos.len()];
        let limits: Vec<(u16, u8, (u16, u8))> = self
            .servos
            .iter()
            .filter_map(|servo| {
                let binding = servo.axis.binding;
                let approach = servo.configuration.approach?;
                Some(
                    approach
                        .registers(binding.reduction)
                        .map(|registers| (binding.bus_id, servo.scs_id(), registers))
                        .ok_or(FeetechChainError::ApproachLimit(binding.bus_id)),
                )
            })
            .collect::<Result<_, FeetechChainError>>()?;
        for (bus_id, scs_id, (speed, acceleration)) in &limits {
            self.write1(*bus_id, *scs_id, SMS_STS_ACC, *acceleration)?;
            expect(
                *bus_id,
                self.sms
                    .ph
                    .write2_byte_tx_rx(*scs_id, SMS_STS_GOAL_SPEED_L, *speed),
            )?;
        }
        for (bus_id, scs_id, (speed, acceleration)) in limits {
            let (read_acceleration, result, error) =
                self.sms.ph.read1_byte_tx_rx(scs_id, SMS_STS_ACC);
            expect(bus_id, (result, error))?;
            let read_speed = self.read2(bus_id, scs_id, SMS_STS_GOAL_SPEED_L)?;
            if (read_speed, read_acceleration) != (speed, acceleration) {
                return Err(FeetechChainError::ApproachUnconfirmed {
                    bus_id,
                    speed: read_speed,
                    acceleration: read_acceleration,
                });
            }
        }
        for (servo, limited) in self.servos.iter().zip(&mut self.approach_limited) {
            *limited = servo.configuration.approach.is_some();
        }
        Ok(())
    }

    /// Every servo's `Present_Position` as a joint coordinate, or `None` when any servo did
    /// not answer.
    fn read_positions(&mut self) -> Option<Vec<f64>> {
        let ids: Vec<u8> = self.servos.iter().map(FeetechServo::scs_id).collect();
        let mut observed = Vec::with_capacity(ids.len());
        for (servo, scs_id) in self.servos.iter().zip(ids) {
            let (raw, result, _) = self
                .sms
                .ph
                .read2_byte_tx_rx(scs_id, SMS_STS_PRESENT_POSITION_L);
            if result != CommResult::Success {
                return None;
            }
            let raw = ProtocolPacketHandler::<()>::scs_tohost(raw, PRESENT_POSITION_SIGN_BIT);
            observed.push(servo.joint_from_raw(raw));
        }
        Some(observed)
    }

    /// One SYNC READ of every servo's `Present_Position`, as joint coordinates in native state
    /// order; a servo whose answer is missing or corrupt is `NaN`, which no tracking bound
    /// admits.
    fn sync_read_positions(&mut self) -> Result<Vec<f64>, FeetechChainError> {
        let mut reader = GroupSyncRead::new(SMS_STS_PRESENT_POSITION_L, 2);
        for servo in &self.servos {
            if !reader.add_param(servo.scs_id()) {
                return Err(FeetechChainError::GroupParam(servo.axis.binding.bus_id));
            }
        }
        let _ = reader.tx_rx_packet(&mut self.sms.ph);
        let end = self.sms.ph.scs_getend();
        Ok(self
            .servos
            .iter()
            .map(|servo| {
                reader
                    .get_data2(end, servo.scs_id(), SMS_STS_PRESENT_POSITION_L)
                    .map_or(f64::NAN, |raw| {
                        servo.joint_from_raw(ProtocolPacketHandler::<()>::scs_tohost(
                            raw,
                            PRESENT_POSITION_SIGN_BIT,
                        ))
                    })
            })
            .collect())
    }

    /// The status flags every stop since the last [`ActuatorChain::clear_faults`] reported,
    /// proven or not, as (bus id, flags).
    #[must_use]
    pub fn faults(&self) -> &[(u16, u8)] {
        &self.faults
    }

    /// The servos, in native state order.
    #[must_use]
    pub fn servos(&self) -> &[FeetechServo] {
        &self.servos
    }

    /// The serial link, for inspection.
    #[must_use]
    pub const fn port(&self) -> &P {
        self.sms.ph.port()
    }

    pub const fn port_mut(&mut self) -> &mut P {
        self.sms.ph.port_mut()
    }

    /// `FeetechMotorsBus.enable_torque`: `Torque_Enable` 1, then `Lock` 1, on every servo.
    fn enable_torque(&mut self) -> Result<(), FeetechChainError> {
        // Hold every servo where it is first, so enabling never drives toward a stale goal.
        let ids: Vec<(u16, u8)> = self
            .servos
            .iter()
            .map(|servo| (servo.axis.binding.bus_id, servo.scs_id()))
            .collect();
        for (bus_id, scs_id) in ids {
            let present = self.read2(bus_id, scs_id, SMS_STS_PRESENT_POSITION_L)?;
            expect(
                bus_id,
                self.sms
                    .ph
                    .write2_byte_tx_rx(scs_id, SMS_STS_GOAL_POSITION_L, present),
            )?;
        }
        for servo in &self.servos {
            let (bus_id, scs_id) = (servo.axis.binding.bus_id, servo.scs_id());
            expect(
                bus_id,
                self.sms
                    .ph
                    .write1_byte_tx_rx(scs_id, SMS_STS_TORQUE_ENABLE, 1),
            )?;
            expect(bus_id, self.sms.lock_eprom(scs_id))?;
        }
        Ok(())
    }
}

impl<P: SerialPort> ActuatorChain for FeetechChain<P> {
    type Error = FeetechChainError;
    /// The SYNC WRITE that carries a command has no answer; its observed joints are the SYNC
    /// READ that follows it.
    type Feedback = ();

    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    fn commandable(&self) -> &[CommandRange] {
        &self.commandable
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt<()>, FeetechChainError> {
        if self.stop_pending {
            return Err(FeetechChainError::StopPending);
        }
        if !self.faults.is_empty() {
            return Err(FeetechChainError::FaultedAtStop {
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
            .collect::<Result<Vec<u16>, _>>()?;
        let mut writer = GroupSyncWrite::new(SMS_STS_GOAL_POSITION_L, 2);
        for (servo, goal) in self.servos.iter().zip(goals) {
            if !writer.add_param(
                servo.scs_id(),
                &[self.sms.ph.scs_lobyte(goal), self.sms.ph.scs_hibyte(goal)],
            ) {
                return Err(FeetechChainError::GroupParam(servo.axis.binding.bus_id));
            }
        }
        // From here the servos may move: what open or the last stop read no longer holds.
        self.observed = None;
        if !self.enabled {
            self.enable_torque()?;
            self.enabled = true;
        }
        match writer.tx_packet(&mut self.sms.ph) {
            CommResult::Success => Ok(Receipt {
                evidence: Evidence::DispatchAttempted,
                observed: self.sync_read_positions()?,
                feedback: Vec::new(),
            }),
            result => Err(FeetechChainError::SyncWrite(result)),
        }
    }

    fn stop(&mut self) -> StopReport {
        fn note(bus_id: u16, error: u8, faults: &mut Vec<(u16, u8)>) {
            if error != 0 && !faults.contains(&(bus_id, error)) {
                faults.push((bus_id, error));
            }
        }
        self.enabled = false;
        self.stop_pending = true;
        let ids: Vec<(u16, u8)> = self
            .servos
            .iter()
            .map(|servo| (servo.axis.binding.bus_id, servo.scs_id()))
            .collect();
        let mut report = StopReport::default();
        for (bus_id, scs_id) in &ids {
            let torque_off = self
                .sms
                .ph
                .write1_byte_tx_rx(*scs_id, SMS_STS_TORQUE_ENABLE, 0);
            let unlocked = self.sms.un_lock_eprom(*scs_id);
            let mut answered = true;
            for (result, error) in [torque_off, unlocked] {
                if result == CommResult::Success {
                    note(*bus_id, error, &mut report.faults);
                } else {
                    answered = false;
                }
            }
            if !answered {
                report.unproven.push(*bus_id);
            }
        }
        for (bus_id, scs_id) in ids {
            let (value, result, error) =
                self.sms.ph.read1_byte_tx_rx(scs_id, SMS_STS_TORQUE_ENABLE);
            let proven = result == CommResult::Success && value == 0;
            if result == CommResult::Success {
                note(bus_id, error, &mut report.faults);
            }
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
        self.observed = self.read_positions();
        report
    }

    /// A servo's, once the approach limit it declares was confirmed.
    fn approach_speed_limited(&self, axis: usize) -> bool {
        self.approach_limited.get(axis).copied().unwrap_or(false)
    }

    /// Where every servo was read at open or after the last stop, when no command has been
    /// sent since and every servo answered.
    fn last_observed(&self) -> Option<Vec<f64>> {
        self.observed.clone()
    }

    /// The longest [`ActuatorChain::stop`] takes on this chain: four exchanges per servo,
    /// each bounded by the link's [`SerialPort::timeout`].
    fn stop_budget(&self) -> Duration {
        let servos = u32::try_from(self.servos.len()).unwrap_or(u32::MAX);
        self.sms
            .ph
            .port()
            .timeout()
            .saturating_mul(STOP_EXCHANGES_PER_SERVO.saturating_mul(servos))
    }

    /// Release the latched stop faults, letting motion resume. Whoever calls this has decided
    /// the servos that reported them are fit to move.
    fn clear_faults(&mut self) {
        self.faults.clear();
    }
}

impl ChainError for FeetechChainError {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::Body(_)
            | Self::Target(_)
            | Self::Unencodable(_)
            | Self::CalibrationCount { .. }
            | Self::ApproachLimit(_)
            | Self::GroupParam(_)
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
