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
//!   a servo that misses it is reported `NaN`, so a tracking check fails closed. The read
//!   covers `Torque_Enable` (40) through `Present_Position` (56..=57) in one range.
//! - Stop disables as `disable_torque` does (`feetech.py:291-294`): per servo, `Torque_Enable`
//!   0, then `Lock` 0 (`sms_sts.unLockEprom`), each acknowledged, on every servo whatever an
//!   earlier one answered. Then it reads every servo's `Torque_Enable` back
//!   (`read1ByteTxRx`); a servo is proven stopped only when it reads 0.
//! - After open and after every stop it reads each servo's `Present_Position`
//!   (`read2ByteTxRx`, sign-magnitude at bit 15), as `LeRobot` reads an observation, and
//!   reports it as [`ActuatorChain::last_observed`], as it does every command's SYNC READ where
//!   every servo answered, so a caller can start from where the body rests. A servo resting
//!   outside its calibrated range is reported there, unclamped. A position read that fails
//!   reports nothing; it does not fail open.
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
//!   reduction, in encoder steps per second and 100 steps/s² per step, rounded down) and
//!   read back; open refuses a servo that does not hold them. `LeRobot` leaves `Goal_Speed` at
//!   0, which the servo reads as its full speed, and writes `Acceleration` 254; here they
//!   replace that, so a goal the servo's own position limits clamp, or any first goal far from
//!   where it rests, is approached at the limit. A servo without one (the SO-101 follower's)
//!   keeps `LeRobot`'s registers and is never given a goal its limits would clamp.
//!
//!   **`Goal_Speed` is in encoder steps per second** (1/4096 rev/s), measured on the static
//!   upper body's STS3215s (firmware 3.10) on 2026-10-26: a timed 100-tick move ran at 50
//!   steps/s with `Goal_Speed` 3 and with 20, and a finger given `Goal_Speed` 52 moved one tick
//!   per 20 ms in the station's log. Below 50 the firmware still runs at 50 steps/s
//!   ([`SLOWEST_GOAL_SPEED`]), so a limit that rounds below it is refused rather than run
//!   faster than declared. The vendor's `sms_sts/write.py` comment ("V=60 * 0.732=43.92rpm")
//!   reads one step as 0.732 rpm, which is 50 steps/s: taken per register step, as this chain
//!   did before, it made every declared approach about 50 times slower than declared (4 rad/s
//!   wrote 52, about 0.08 rad/s, and the governor's tracking stop fired on every gripper move).
//!
//!   Those registers are RAM: a servo whose power cycles while its USB link stays up comes back with
//!   `Goal_Speed` 0 (full speed) and torque off. So the limit is never trusted for the
//!   chain's life: every enable reads it back (and rewrites and reads again where it differs,
//!   refusing a servo that still does not hold it), and every command's SYNC READ covers
//!   `Torque_Enable` through `Present_Position`, so a servo found limp or unlimited is `NaN`
//!   and no longer approach-limited, and the chain enables afresh on the next command.
//! - **Enabling never drives toward a stale goal.** Before `Torque_Enable` 1, each servo's
//!   `Present_Position` is read and written to its `Goal_Position`; `LeRobot` enables with
//!   whatever goal the register last held. That holds a servo where it rests only when it rests
//!   inside its `Min_Angle_Limit`..`Max_Angle_Limit` (9, 11): the firmware clamps every goal
//!   into them, so a servo resting outside drives to its nearest limit the instant torque comes
//!   on. The chain makes that move explicit: such a servo's goal is written as `Present_Position`
//!   clamped into its calibrated limits, recorded in [`FeetechChain::held_inside`], and only
//!   when the servo's approach limit was confirmed by this same enable's read-back. That
//!   pull-in is the one move no governor bounds, so it runs at [`BRING_UP_APPROACH`] (or the
//!   servo's own limit where that is slower), written and read back before torque comes on;
//!   the first command whose SYNC READ finds the servo inside its limits raises it to its
//!   declared approach ([`FeetechChain::pulling_in`] lists the servos still coming in). A
//!   raise that fails leaves the servo at the slow limit and is tried again on the next
//!   command. A servo resting outside without one (the SO-101 follower's)
//!   refuses the enable ([`FeetechChainError::RestsOutside`]): move it inside by hand. Open and
//!   every stop report the unclamped rest through [`ActuatorChain::last_observed`], outside the
//!   axis's commandable range.
//! - **Range limits motion, not state.** Each axis is commanded within its commandable range
//!   widened to where it was last observed (`CommandRange::reaching`), as every chain is, so a
//!   governor holds a servo resting outside or steps it in, never further out. The firmware
//!   cannot hold a goal outside its limits: a servo whose approach limit is confirmed is given
//!   such a target's nearest in-range goal, which it approaches at that limit; one without is
//!   refused it.
//! - **The commandable range is what the unit reaches.** A [`MotorNormMode::Degrees`] axis is
//!   commanded within its declared bounds intersected with its unit's calibrated reach,
//!   `normalize(range_min)..normalize(range_max)` through the drive map in the axis's unit
//!   ([`FeetechServo::calibrated_reach`]). A goal past the limit registers is one the firmware
//!   clamps, so the servo would sit at its limit while a governor tracked it toward the
//!   declared bound and faulted; narrowed here, a governor holds the target at the reachable
//!   edge instead. An axis whose two ranges do not overlap is refused at open
//!   ([`FeetechChainError::RangeMismatch`]). This applies to every Degrees servo, the SO-101
//!   follower's arm included, whose commandable range was its declared bounds before. A
//!   [`MotorNormMode::Range0100`] servo (a gripper) cannot mismatch: its percent maps the
//!   calibrated range onto the declared bounds by construction.
//! - It refuses before any frame: a wrong width, a target outside the axis's commandable range
//!   or not finite, an axis whose declared and calibrated ranges do not overlap, a magnitude that reaches the sign bit (`LeRobot`'s `encode_sign_magnitude`
//!   raises there; the vendor's `scs_toscs` would set overlapping bits), a model or bus this
//!   crate is not qualified for, an address the protocol cannot carry, and an empty or
//!   out-of-resolution calibration.
//! - A target is converted at float32 precision, as the station's earlier Python follower
//!   did: `sx_drivers.feetech.radians_to_wire` (`packages/sx-drivers/sx_drivers/feetech.py:
//!   291-313` in `Sentient-X/sx` at `d6d4a3ea6`, the source the station's golden was first
//!   minted from) took a float32 target and compared it with each bound at float32 under
//!   `NumPy`'s promotion rules. Each axis's commandable range is the union of its declared
//!   bounds and those bounds rounded to float32: `[min(lower, f32(lower)), max(upper,
//!   f32(upper))]`, intersected for a Degrees axis with its calibrated reach as above. Every declared bound is admitted, and so is a bound's float32 rounding
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

use std::time::{Duration, Instant};

pub use sx_embodiment_drivers::lerobot::{MotorCalibration, MotorNormMode};
use sx_embodiment_drivers::lerobot::{normalize, unnormalize};
use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{
    ActuatorBus, ActuatorChain, ActuatorModel, Axes, AxisReading, BindingError, BodyError,
    BoundAxis, ChainError, ChainFailure, CommandRange, Evidence, MissingReason, Receipt,
    StopReport, TargetError, check_reach, fresh_positions, validate_chain,
};
pub use sx_embodiment_drivers::{ApproachLimit, BRING_UP_APPROACH};
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
/// The registers each command's SYNC READ covers: `Torque_Enable` (40) to `Present_Position`'s
/// high byte (57).
const SYNC_READ_LENGTH: u8 = SMS_STS_PRESENT_POSITION_L + 2 - SMS_STS_TORQUE_ENABLE;
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

/// One `Goal_Speed` step: one encoder step per second, in rad/s (measured; see the module's
/// header).
pub const SPEED_STEP_RAD_PER_S: f64 = std::f64::consts::TAU / RESOLUTION;
/// The slowest the firmware moves: a `Goal_Speed` from 1 to 49 runs at 50 steps/s (measured),
/// the vendor's "0.732 rpm".
pub const SLOWEST_GOAL_SPEED: u16 = 50;
/// One `Acceleration` step: 100 steps/s², in rad/s² (the vendor's "8.7deg/s^2", 8.79 exactly).
pub const ACCELERATION_STEP_RAD_PER_S2: f64 = 100.0 * std::f64::consts::TAU / RESOLUTION;
/// `Goal_Speed`'s largest magnitude below its sign bit.
const GOAL_SPEED_MAX: u32 = 32_767;

/// The `Goal_Speed` and `Acceleration` register values that move an actuator geared at
/// `reduction` no faster than `approach`, rounded down; `None` when a value rounds to 0, which
/// the servo reads as no limit at all, when the speed rounds below [`SLOWEST_GOAL_SPEED`],
/// which the firmware would run faster than asked, or when a value does not fit its register.
#[must_use]
pub fn approach_registers(approach: &ApproachLimit, reduction: f64) -> Option<(u16, u8)> {
    let (speed, acceleration) = approach.steps(
        reduction,
        (SPEED_STEP_RAD_PER_S, GOAL_SPEED_MAX),
        (ACCELERATION_STEP_RAD_PER_S2, 254),
    )?;
    let speed = u16::try_from(speed).ok()?;
    if speed < SLOWEST_GOAL_SPEED {
        return None;
    }
    Some((speed, u8::try_from(acceleration).ok()?))
}

/// The registers a servo resting outside its limits is pulled in at: [`BRING_UP_APPROACH`],
/// or `declared` where that is slower, each register the slower of the two. `None` when
/// `declared` is already no faster, so there is nothing to raise once the servo is inside.
#[must_use]
pub fn pull_in_registers(declared: (u16, u8), reduction: f64) -> Option<(u16, u8)> {
    let bring_up = approach_registers(&BRING_UP_APPROACH, reduction)?;
    let slower = (declared.0.min(bring_up.0), declared.1.min(bring_up.1));
    (slower != declared).then_some(slower)
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
    pub(crate) fn scs_id(&self) -> u8 {
        // Open refuses an address above `MAX_ID`, so the narrowing never saturates.
        u8::try_from(self.axis.binding.bus_id).unwrap_or(u8::MAX)
    }

    /// The joint range this servo is commanded within: its declared bounds, widened to their
    /// float32 roundings where those land outside them, and for a Degrees servo intersected
    /// with its [`FeetechServo::calibrated_reach`]. Empty (`lower > upper`) when the two do not
    /// overlap, which open refuses.
    #[must_use]
    pub fn commandable(&self) -> CommandRange {
        let declared = CommandRange {
            lower: self.axis.lower.min(float32(self.axis.lower)),
            upper: self.axis.upper.max(float32(self.axis.upper)),
        };
        self.calibrated_reach()
            .map_or(declared, |reach| CommandRange {
                lower: declared.lower.max(reach.lower),
                upper: declared.upper.min(reach.upper),
            })
    }

    /// The joint range the unit's calibrated limit registers reach, in the axis's unit:
    /// `normalize(range_min)..normalize(range_max)` through the drive map, ordered. `None` for
    /// a `RANGE_0_100` servo, whose percent maps the calibrated range onto the declared bounds,
    /// so the two cannot disagree.
    #[must_use]
    pub fn calibrated_reach(&self) -> Option<CommandRange> {
        if self.norm_mode != MotorNormMode::Degrees {
            return None;
        }
        let ends = [
            self.joint_from_raw(i32::from(self.calibration.range_min)),
            self.joint_from_raw(i32::from(self.calibration.range_max)),
        ];
        Some(CommandRange {
            lower: ends[0].min(ends[1]),
            upper: ends[0].max(ends[1]),
        })
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

impl FeetechServo {
    /// Whether `joint` lies within the unit's calibrated limit registers
    /// (`range_min..=range_max` through [`FeetechServo::joint_from_raw`]), where the firmware
    /// holds any goal: a servo resting there is no longer being pulled in.
    #[must_use]
    pub fn within_limits(&self, joint: f64) -> bool {
        let ends = [
            self.joint_from_raw(i32::from(self.calibration.range_min)),
            self.joint_from_raw(i32::from(self.calibration.range_max)),
        ];
        (ends[0].min(ends[1])..=ends[0].max(ends[1])).contains(&joint)
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
    #[error(
        "joint {joint} is declared {declared_lower}..{declared_upper} but its unit's calibrated \
         range reaches {calibrated_lower}..{calibrated_upper}: the two do not overlap"
    )]
    RangeMismatch {
        joint: String,
        declared_lower: f64,
        declared_upper: f64,
        calibrated_lower: f64,
        calibrated_upper: f64,
    },
    #[error(
        "bus_id {bus_id} rests at tick {present}, outside its calibrated limits \
         {range_min}..{range_max}, with no confirmed approach limit to bring it in slowly; move \
         it inside by hand"
    )]
    RestsOutside {
        bus_id: u16,
        present: i32,
        range_min: u16,
        range_max: u16,
    },
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
    observed: Option<Axes<f64>>,
    /// Each servo's reading from the last command's SYNC READ; none since open or a stop.
    command_readings: Option<Axes<AxisReading>>,
    /// The command's SYNC READ of every servo, its servos added at open and reused.
    reader: GroupSyncRead,
    /// The command's SYNC WRITE of every goal, refilled each command in buffers it keeps.
    writer: GroupSyncWrite,
    /// Per servo, whether its approach limit was written and read back.
    approach_limited: Vec<bool>,
    /// Per servo, whether it is being pulled in from outside its limits at its pull-in
    /// registers, not yet raised to its declared approach.
    pulling_in: Vec<bool>,
    /// The servos the last enable found resting outside their calibrated limits, and the
    /// goal each was given inside them.
    held_inside: Vec<HeldInside>,
}

/// A servo an enable found resting outside its calibrated limits: its goal was written as its
/// `Present_Position` clamped into them, which it then approached at its confirmed limit.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct HeldInside {
    pub bus_id: u16,
    /// Where it rested, in ticks.
    pub present: i32,
    /// The goal written before `Torque_Enable`, in ticks.
    pub goal: u16,
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

/// Refuse axes that are not one valid chain on one port of qualified STS3215s at addressable
/// ids, before any byte.
pub(crate) fn qualify(axes: &[BoundAxis]) -> Result<(), FeetechChainError> {
    validate_chain(axes)?;
    let mut ports: Vec<String> = axes.iter().map(|axis| axis.port.clone()).collect();
    ports.dedup();
    if ports.len() > 1 {
        return Err(FeetechChainError::Ports(ports));
    }
    for axis in axes {
        let binding = axis.binding;
        if binding.bus != FEETECH_SERIAL || binding.model != FEETECH_STS3215 {
            return Err(FeetechChainError::Unqualified(binding.bus_id));
        }
        if binding.bus_id > u16::from(MAX_ID) {
            return Err(FeetechChainError::Ids(binding.bus_id));
        }
    }
    Ok(())
}

/// Refuse a calibration whose range is empty or beyond the encoder's resolution.
pub(crate) fn check_calibration(
    bus_id: u16,
    calibration: MotorCalibration,
) -> Result<(), FeetechChainError> {
    if calibration.range_min >= calibration.range_max
        || f64::from(calibration.range_max) >= RESOLUTION
    {
        return Err(FeetechChainError::Calibration(bus_id));
    }
    Ok(())
}

/// The calibration a servo's registers hold, read as `LeRobot`'s `read_calibration` reads it:
/// `Min_Position_Limit`, `Max_Position_Limit` and `Homing_Offset` (sign-magnitude at bit 11).
/// The STS has no drive-mode register; `LeRobot` records 0, and so does this.
pub(crate) fn held_calibration<P: SerialPort>(
    ph: &mut ProtocolPacketHandler<P>,
    bus_id: u16,
    scs_id: u8,
) -> Result<MotorCalibration, FeetechChainError> {
    let mut read2 = |address: u8| {
        let (value, result, error) = ph.read2_byte_tx_rx(scs_id, address);
        expect(bus_id, (result, error)).map(|()| value)
    };
    let range_min = read2(SMS_STS_MIN_ANGLE_LIMIT_L)?;
    let range_max = read2(SMS_STS_MAX_ANGLE_LIMIT_L)?;
    let offset =
        ProtocolPacketHandler::<()>::scs_tohost(read2(SMS_STS_OFS_L)?, HOMING_OFFSET_SIGN_BIT);
    Ok(MotorCalibration {
        drive_mode: false,
        // Eleven bits of magnitude always fit.
        homing_offset: i16::try_from(offset).unwrap_or(i16::MAX),
        range_min,
        range_max,
    })
}

/// What a read that did not succeed says about its axis: faulted with the servo's error flags
/// when it answered, else missing for why it did not.
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

/// One servo's `Present_Position` (56..=57), one `READ`, as its axis's reading. The answer is
/// fresh: the packet handler clears the port before it writes the `READ`. A servo that is
/// silent, answers corrupt or cannot be reached is missing; one that answers with error flags
/// is faulted with them. Reading writes no register, so nothing moves and torque stays as it
/// was.
pub(crate) fn present_position<P: SerialPort>(
    ph: &mut ProtocolPacketHandler<P>,
    servo: &FeetechServo,
) -> AxisReading {
    let (raw, result, error) = ph.read2_byte_tx_rx(servo.scs_id(), SMS_STS_PRESENT_POSITION_L);
    let received = Instant::now();
    if result != CommResult::Success || error != 0 {
        return unanswered(result, error);
    }
    AxisReading::Fresh {
        position: servo.joint_from_raw(ProtocolPacketHandler::<()>::scs_tohost(
            raw,
            PRESENT_POSITION_SIGN_BIT,
        )),
        received,
    }
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
        qualify(&axes)?;
        for servo in &servos {
            let binding = servo.axis.binding;
            check_calibration(binding.bus_id, servo.calibration)?;
            if servo
                .configuration
                .approach
                .is_some_and(|approach| approach_registers(&approach, binding.reduction).is_none())
            {
                return Err(FeetechChainError::ApproachLimit(binding.bus_id));
            }
            if let Some(reach) = servo.calibrated_reach() {
                let commandable = servo.commandable();
                if commandable.lower > commandable.upper {
                    return Err(FeetechChainError::RangeMismatch {
                        joint: servo.axis.joint.clone(),
                        declared_lower: servo.axis.lower,
                        declared_upper: servo.axis.upper,
                        calibrated_lower: reach.lower,
                        calibrated_upper: reach.upper,
                    });
                }
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
            command_readings: None,
            reader: GroupSyncRead::new(SMS_STS_TORQUE_ENABLE, SYNC_READ_LENGTH),
            writer: GroupSyncWrite::new(SMS_STS_GOAL_POSITION_L, 2),
            approach_limited: Vec::new(),
            pulling_in: Vec::new(),
            held_inside: Vec::new(),
        };
        chain.is_calibrated()?;
        chain.configure()?;
        for servo in &chain.servos {
            if !chain.reader.add_param(servo.scs_id()) {
                return Err(FeetechChainError::GroupParam(servo.axis.binding.bus_id));
            }
        }
        chain.observed = fresh_positions(&chain.present_positions()).map(Axes::from);
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
            let held = held_calibration(&mut self.sms.ph, bus_id, scs_id)?;
            if held.range_min != calibration.range_min
                || held.range_max != calibration.range_max
                || held.homing_offset != calibration.homing_offset
            {
                return Err(FeetechChainError::Uncalibrated {
                    bus_id,
                    homing_offset: i32::from(held.homing_offset),
                    range_min: held.range_min,
                    range_max: held.range_max,
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
        self.pulling_in = vec![false; self.servos.len()];
        for index in 0..self.servos.len() {
            self.hold_approach(index, true)?;
        }
        Ok(())
    }

    /// The approach registers servo `index` declares: `None` when it declares none.
    fn approach_registers(&self, index: usize) -> Result<Option<(u16, u8)>, FeetechChainError> {
        let servo = &self.servos[index];
        let binding = servo.axis.binding;
        servo
            .configuration
            .approach
            .map(|approach| {
                approach_registers(&approach, binding.reduction)
                    .ok_or(FeetechChainError::ApproachLimit(binding.bus_id))
            })
            .transpose()
    }

    /// The approach registers servo `index` should hold now: its pull-in registers while it
    /// is being pulled in from outside its limits, else the ones it declares.
    fn expected_registers(&self, index: usize) -> Result<Option<(u16, u8)>, FeetechChainError> {
        let declared = self.approach_registers(index)?;
        if !self.pulling_in.get(index).copied().unwrap_or(false) {
            return Ok(declared);
        }
        let reduction = self.servos[index].axis.binding.reduction;
        Ok(declared.map(|declared| pull_in_registers(declared, reduction).unwrap_or(declared)))
    }

    /// Prove servo `index` holds the approach limit it should now ([`Self::expected_registers`]):
    /// read its `Acceleration` and `Goal_Speed` (one read of 41..=47), and where they differ
    /// (or `rewrite`) write them and read them again. A servo whose power cycled while its link
    /// stayed up has lost them: the registers are RAM and come back as `Goal_Speed` 0, full
    /// speed. Only a servo that reads them back is approach-limited; one that does not is
    /// refused.
    fn hold_approach(&mut self, index: usize, rewrite: bool) -> Result<(), FeetechChainError> {
        let Some((speed, acceleration)) = self.expected_registers(index)? else {
            return Ok(());
        };
        let (bus_id, scs_id) = {
            let servo = &self.servos[index];
            (servo.axis.binding.bus_id, servo.scs_id())
        };
        let mut held = !rewrite && self.reads_approach(bus_id, scs_id)? == (speed, acceleration);
        if !held {
            self.approach_limited[index] = false;
            self.write1(bus_id, scs_id, SMS_STS_ACC, acceleration)?;
            expect(
                bus_id,
                self.sms
                    .ph
                    .write2_byte_tx_rx(scs_id, SMS_STS_GOAL_SPEED_L, speed),
            )?;
            let (read_speed, read_acceleration) = self.reads_approach(bus_id, scs_id)?;
            if (read_speed, read_acceleration) != (speed, acceleration) {
                return Err(FeetechChainError::ApproachUnconfirmed {
                    bus_id,
                    speed: read_speed,
                    acceleration: read_acceleration,
                });
            }
            held = true;
        }
        self.approach_limited[index] = held;
        Ok(())
    }

    /// A servo's `Goal_Speed` and `Acceleration`, read as one range from `Acceleration` (41) to
    /// `Goal_Speed`'s high byte (47).
    fn reads_approach(&mut self, bus_id: u16, scs_id: u8) -> Result<(u16, u8), FeetechChainError> {
        let length = SMS_STS_GOAL_SPEED_L + 2 - SMS_STS_ACC;
        let (data, result, error) = self.sms.ph.read_tx_rx(scs_id, SMS_STS_ACC, length);
        expect(bus_id, (result, error))?;
        let speed_at = usize::from(SMS_STS_GOAL_SPEED_L - SMS_STS_ACC);
        match (data.first(), data.get(speed_at), data.get(speed_at + 1)) {
            (Some(acceleration), Some(low), Some(high)) => {
                Ok((self.sms.ph.scs_makeword(*low, *high), *acceleration))
            }
            _ => Err(FeetechChainError::Comm {
                bus_id,
                result: CommResult::RxCorrupt,
            }),
        }
    }

    /// Every servo's `Present_Position` (56..=57), one `READ` per servo, as an
    /// [`AxisReading`] per axis in native state order. Each answer is fresh: the packet handler
    /// clears the port before it writes each `READ`. A servo that is silent, answers corrupt or
    /// cannot be reached is missing; one that answers with error flags is faulted with them. The
    /// other servos' readings stand. Reading writes no register, so nothing moves and torque
    /// stays as it was.
    fn present_positions(&mut self) -> sx_embodiment_drivers::Axes<AxisReading> {
        self.servos
            .iter()
            .map(|servo| present_position(&mut self.sms.ph, servo))
            .collect()
    }

    /// One SYNC READ of every servo's registers from `Torque_Enable` (40) to `Present_Position`
    /// (56..=57), as joint coordinates in native state order. A servo whose answer is missing
    /// or corrupt, that reports error flags, that no longer holds torque, or that no longer
    /// holds its declared approach limit is `NaN`, which no tracking bound admits; any of those
    /// marks the chain disabled, so the next enable holds every servo where it rests and proves
    /// its limits again. Each servo's [`AxisReading`] is kept as the command's readings: missing
    /// (`Timeout`: the vendor's read cannot tell a corrupt answer from none), faulted with its
    /// error flags, or faulted with 0 for a torque or approach limit it no longer holds.
    /// A servo being pulled in that this read finds inside its limits is then raised to its
    /// declared approach ([`Self::raise_pulled_in`]).
    fn sync_read_positions(&mut self) -> Result<Axes<f64>, FeetechChainError> {
        let _ = self.reader.tx_rx_packet(&mut self.sms.ph);
        let reader = &self.reader;
        let received = Instant::now();
        let end = self.sms.ph.scs_getend();
        let mut readings: Axes<AxisReading> = Axes::new();
        let mut arrived = false;
        for index in 0..self.servos.len() {
            let servo = &self.servos[index];
            let id = servo.scs_id();
            let torque = reader.get_data1(id, SMS_STS_TORQUE_ENABLE);
            let approach = match self.expected_registers(index)? {
                None => true,
                Some((speed, acceleration)) => {
                    reader.get_data1(id, SMS_STS_ACC) == Some(acceleration)
                        && reader.get_data2(end, id, SMS_STS_GOAL_SPEED_L) == Some(speed)
                }
            };
            let reading = match (
                reader.get_data2(end, id, SMS_STS_PRESENT_POSITION_L),
                reader.get_error(id),
            ) {
                (None, _) | (_, None) => AxisReading::Missing(MissingReason::Timeout),
                (Some(_), Some(error)) if error != 0 => AxisReading::Faulted(error),
                (Some(_), Some(_)) if torque != Some(1) || !approach => AxisReading::Faulted(0),
                (Some(raw), Some(_)) => AxisReading::Fresh {
                    position: servo.joint_from_raw(ProtocolPacketHandler::<()>::scs_tohost(
                        raw,
                        PRESENT_POSITION_SIGN_BIT,
                    )),
                    received,
                },
            };
            arrived |= self.pulling_in[index]
                && reading
                    .position()
                    .is_some_and(|joint| servo.within_limits(joint));
            if !approach {
                self.approach_limited[index] = false;
            }
            if !reading.is_fresh() {
                self.enabled = false;
            }
            readings.push(reading);
        }
        let observed = readings
            .iter()
            .map(|reading| reading.position().unwrap_or(f64::NAN))
            .collect();
        if arrived {
            self.raise_pulled_in(&readings);
        }
        self.command_readings = Some(readings);
        Ok(observed)
    }

    /// Raise every servo being pulled in whose `reading` is inside its limits to its declared
    /// approach, written and read back. A raise that fails leaves the servo at its pull-in
    /// registers, still pulling in, and is tried on the next command; a servo the failure left
    /// holding neither is caught by the next SYNC READ and enabled afresh.
    fn raise_pulled_in(&mut self, readings: &[AxisReading]) {
        for (index, reading) in readings.iter().enumerate() {
            let inside = reading
                .position()
                .is_some_and(|joint| self.servos[index].within_limits(joint));
            if !self.pulling_in[index] || !inside {
                continue;
            }
            self.pulling_in[index] = false;
            if self.hold_approach(index, true).is_err() {
                self.pulling_in[index] = true;
            }
        }
    }

    /// The servos being pulled in from outside their limits at their pull-in registers, by bus
    /// id, not yet raised to their declared approach.
    pub fn pulling_in(&self) -> impl Iterator<Item = u16> + '_ {
        self.servos
            .iter()
            .zip(&self.pulling_in)
            .filter(|(_, pulling)| **pulling)
            .map(|(servo, _)| servo.axis.binding.bus_id)
    }

    /// The status flags every stop since the last [`ActuatorChain::clear_faults`] reported,
    /// proven or not, as (bus id, flags).
    #[must_use]
    pub fn faults(&self) -> &[(u16, u8)] {
        &self.faults
    }

    /// The servos the last enable found resting outside their calibrated limits, each with
    /// the goal it was given inside them.
    #[must_use]
    pub fn held_inside(&self) -> &[HeldInside] {
        &self.held_inside
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
        // Prove every declared approach limit still holds: a servo that power-cycled has lost it,
        // and one an earlier enable was pulling in may still hold its slower pull-in registers.
        self.pulling_in.fill(false);
        for index in 0..self.servos.len() {
            self.hold_approach(index, false)?;
        }
        // Hold every servo where it is first, so enabling never drives toward a stale goal; a
        // servo resting outside its limits is given the nearest one, which the firmware would
        // drive it to anyway, and only at an approach limit this enable just confirmed.
        self.held_inside.clear();
        let ids: Vec<(u16, u8, MotorCalibration)> = self
            .servos
            .iter()
            .map(|servo| (servo.axis.binding.bus_id, servo.scs_id(), servo.calibration))
            .collect();
        for (index, (bus_id, scs_id, calibration)) in ids.into_iter().enumerate() {
            let register = self.read2(bus_id, scs_id, SMS_STS_PRESENT_POSITION_L)?;
            let present =
                ProtocolPacketHandler::<()>::scs_tohost(register, PRESENT_POSITION_SIGN_BIT);
            let (range_min, range_max) = (calibration.range_min, calibration.range_max);
            let goal = if (i32::from(range_min)..=i32::from(range_max)).contains(&present) {
                register
            } else if self.approach_limited[index] {
                // The pull-in is the one move no governor bounds: at the slower pull-in
                // registers, confirmed before torque comes on, until a command finds it inside.
                let declared = self.approach_registers(index)?;
                let reduction = self.servos[index].axis.binding.reduction;
                if declared.is_some_and(|declared| pull_in_registers(declared, reduction).is_some())
                {
                    self.pulling_in[index] = true;
                    self.hold_approach(index, true)?;
                }
                // Inside `range_min..=range_max`, both below 4096: positive, so its
                // sign-magnitude encoding is itself.
                let goal = if present < i32::from(range_min) {
                    range_min
                } else {
                    range_max
                };
                self.held_inside.push(HeldInside {
                    bus_id,
                    present,
                    goal,
                });
                goal
            } else {
                return Err(FeetechChainError::RestsOutside {
                    bus_id,
                    present,
                    range_min,
                    range_max,
                });
            };
            expect(
                bus_id,
                self.sms
                    .ph
                    .write2_byte_tx_rx(scs_id, SMS_STS_GOAL_POSITION_L, goal),
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
        check_reach(
            &self.axes,
            &self.commandable,
            self.observed.as_deref(),
            joints,
        )?;
        // A target outside the range, toward it from where the servo is, goes to the servo as
        // its nearest in-range goal, which the firmware would clamp it to anyway, only where the
        // servo approaches any goal at its confirmed limit.
        let goals = self
            .servos
            .iter()
            .zip(joints)
            .zip(&self.commandable)
            .enumerate()
            .map(|(index, ((servo, joint), range))| {
                let limited = self.approach_limited.get(index).copied().unwrap_or(false);
                let goal = if limited {
                    joint.clamp(range.lower, range.upper)
                } else {
                    *joint
                };
                servo.goal_position(goal)
            })
            .collect::<Result<Axes<u16>, _>>()?;
        self.writer.clear_param();
        for (servo, goal) in self.servos.iter().zip(goals) {
            if !self.writer.add_param(
                servo.scs_id(),
                &[self.sms.ph.scs_lobyte(goal), self.sms.ph.scs_hibyte(goal)],
            ) {
                return Err(FeetechChainError::GroupParam(servo.axis.binding.bus_id));
            }
        }
        // From here the servos may move: what open or the last stop read no longer holds.
        self.observed = None;
        self.command_readings = None;
        if !self.enabled {
            self.enable_torque()?;
            self.enabled = true;
        }
        match self.writer.tx_packet(&mut self.sms.ph) {
            CommResult::Success => {
                let observed = self.sync_read_positions()?;
                self.observed = observed
                    .iter()
                    .all(|joint| joint.is_finite())
                    .then_some(observed);
                Ok(Receipt {
                    evidence: Evidence::DispatchAttempted,
                    observed,
                    feedback: sx_embodiment_drivers::Axes::new(),
                })
            }
            result => {
                // Whatever the servos hold now, the next command proves it again.
                self.enabled = false;
                Err(FeetechChainError::SyncWrite(result))
            }
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

    /// Where every servo was read at open, after the last stop, or by the last command's SYNC
    /// READ, when every servo answered.
    fn last_observed(&self) -> Option<Axes<f64>> {
        self.observed
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
            | Self::RangeMismatch { .. }
            | Self::RestsOutside { .. }
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
