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
//!   command's receipt claims only a dispatch.
//! - Stop disables as `disable_torque` does (`feetech.py:291-294`): per servo, `Torque_Enable`
//!   0, then `Lock` 0 (`sms_sts.unLockEprom`), each acknowledged, on every servo whatever an
//!   earlier one answered. Then it reads every servo's `Torque_Enable` back
//!   (`read1ByteTxRx`); a servo is proven stopped only when it reads 0.
//!
//! A goal position is computed from a joint target in three steps:
//!
//! 1. The registry's drive map gives the actuator coordinate
//!    (`ActuatorBinding::actuator_from_joint`: `sign * (joint - zero_offset) * reduction`).
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
//! - It refuses before any frame: a wrong width, a target outside the axis's commandable range
//!   or not finite, a magnitude that reaches the sign bit (`LeRobot`'s `encode_sign_magnitude`
//!   raises there; the vendor's `scs_toscs` would set overlapping bits), a model or bus this
//!   crate is not qualified for, an address the protocol cannot carry, and an empty or
//!   out-of-resolution calibration.
//! - A target is converted at float32 precision, as the station's earlier Python follower
//!   did: `sx_drivers.feetech.radians_to_wire` (`packages/sx-drivers/sx_drivers/feetech.py:
//!   291-313` in `Sentient-X/sx` at `d6d4a3ea6`, the source the station's golden was first
//!   minted from) took a float32 target and compared it with each bound at float32 under
//!   `NumPy`'s promotion rules. Each axis's commandable range is therefore its declared bounds
//!   rounded to float32 (within half a float32 step outside the declared bounds, which moves
//!   no servo by a tick, so it departs from the trait's "inside the declared bounds"),
//!   admission is that range exactly, and every later step runs on the target rounded to
//!   `f32`, in `f64`, as Python does.
//! - A stop is proven servo by servo by the torque read-back, not by the acknowledgements. The
//!   error bytes the servos report while stopping, such as overload or overheating, are faults
//!   of a stop that may still be proven; they are latched, and every command is refused until
//!   [`FeetechChain::clear_faults`] clears them. After an unproven stop every command is refused
//!   until a stop is proven.

use std::time::Duration;

use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{
    ActuatorBus, ActuatorChain, ActuatorModel, BindingError, BoundAxis, CommandRange, Evidence,
    Receipt, StopReport, TargetError, validate_chain,
};
use thiserror::Error;

use crate::scservo_sdk::{
    CommResult, GroupSyncWrite, MAX_ID, ProtocolPacketHandler, SMS_STS_ACC,
    SMS_STS_GOAL_POSITION_L, SMS_STS_MAX_ANGLE_LIMIT_L, SMS_STS_MIN_ANGLE_LIMIT_L, SMS_STS_MODE,
    SMS_STS_MODEL_L, SMS_STS_OFS_L, SMS_STS_TORQUE_ENABLE, SmsSts,
};

/// `ActuatorModel.FEETECH_STS3215` in the registry.
pub const FEETECH_STS3215: ActuatorModel = ActuatorModel::new("feetech_sts3215");
/// `ActuatorBus.FEETECH_SERIAL` in the registry.
pub const FEETECH_SERIAL: ActuatorBus = ActuatorBus::new("feetech_serial");
/// The STS3215's model number (`LeRobot` `MODEL_NUMBER_TABLE["sts3215"]`).
pub const STS3215_MODEL_NUMBER: u16 = 777;
/// Encoder steps per turn (`LeRobot` `MODEL_RESOLUTION["sts3215"]`).
const RESOLUTION: f64 = 4096.0;
/// `Goal_Position`'s sign bit: `sms_sts`'s `scs_toscs(position, 15)`, and `LeRobot`'s
/// `STS_SMS_SERIES_ENCODINGS_TABLE`.
const GOAL_POSITION_SIGN_BIT: u32 = 15;
/// The register's largest magnitude below its sign bit.
const GOAL_POSITION_MAGNITUDE: f64 = 32_767.0;
/// Exchanges a stop makes per servo: torque off, unlock, and the torque read-back.
const STOP_EXCHANGES_PER_SERVO: u32 = 3;
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

/// How `LeRobot` normalizes a servo's coordinate on the wire (`MotorNormMode`).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotorNormMode {
    /// `DEGREES`: degrees about the middle of the calibrated range.
    Degrees,
    /// `RANGE_0_100`: percent of the calibrated range.
    Range0100,
}

/// One servo's per-unit calibration, as `LeRobot`'s `MotorCalibration` records it. Open
/// refuses a servo whose own registers disagree with it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MotorCalibration {
    /// An inverted drive; applied to percent servos only, as `_unnormalize` does.
    pub drive_mode: bool,
    /// The servo's `Homing_Offset` register, in encoder ticks.
    pub homing_offset: i16,
    /// The calibrated raw range, in encoder ticks: the servo's position limits.
    pub range_min: u16,
    pub range_max: u16,
}

/// The torque and current limits `SOFollower.configure` writes to the gripper "to avoid
/// burnout".
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TorqueProtection {
    pub max_torque_limit: u16,
    pub protection_current: u16,
    pub overload_torque: u8,
}

/// The position loop `SOFollower.configure` writes to every servo.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ServoConfiguration {
    pub p_coefficient: u8,
    pub i_coefficient: u8,
    pub d_coefficient: u8,
    pub protection: Option<TorqueProtection>,
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

    /// The joint range this servo is commanded within: its declared bounds rounded to float32.
    #[must_use]
    pub fn commandable(&self) -> CommandRange {
        CommandRange {
            lower: float32(self.axis.lower),
            upper: float32(self.axis.upper),
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
        let actuator = binding.actuator_from_joint(single);
        let min = f64::from(self.calibration.range_min);
        let max = f64::from(self.calibration.range_max);
        let raw = match self.norm_mode {
            // Python's `(min_ + max_) / 2`, written the same way so every rounding step matches.
            #[allow(clippy::manual_midpoint)]
            MotorNormMode::Degrees => {
                let val = actuator.to_degrees();
                let mid = (min + max) / 2.0;
                let max_res = RESOLUTION - 1.0;
                (val * max_res / 360.0 + mid).trunc()
            }
            MotorNormMode::Range0100 => {
                let ends = [
                    binding.actuator_from_joint(axis.lower),
                    binding.actuator_from_joint(axis.upper),
                ];
                let (low, high) = (ends[0].min(ends[1]), ends[0].max(ends[1]));
                let val = (actuator - low) * 100.0 / (high - low);
                let val = if self.calibration.drive_mode {
                    100.0 - val
                } else {
                    val
                };
                let bounded_val = val.clamp(0.0, 100.0);
                ((bounded_val / 100.0) * (max - min) + min).trunc()
            }
        };
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
    Target(#[from] TargetError),
    #[error(
        "bus_id {0} is not on the feetech_serial bus or binds a model this crate is not qualified for"
    )]
    Unqualified(u16),
    #[error("bus_id {0} is above the protocol's last servo id, 252")]
    Ids(u16),
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
    ///
    /// # Errors
    ///
    /// Refuses an invalid or unqualified axis, an unaddressable id, an invalid calibration, a
    /// servo that does not answer or reports error flags, any servo whose model is not the
    /// STS3215, and any servo whose registers disagree with its calibration.
    pub fn open(port: P, servos: Vec<FeetechServo>) -> Result<Self, FeetechChainError> {
        let axes: Vec<BoundAxis> = servos.iter().map(|servo| servo.axis.clone()).collect();
        validate_chain(&axes)?;
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
        };
        chain.is_calibrated()?;
        chain.configure()?;
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
        Ok(())
    }

    /// The status flags proven stops reported and the chain holds, as (bus id, flags).
    #[must_use]
    pub fn faults(&self) -> &[(u16, u8)] {
        &self.faults
    }

    /// Release the latched stop faults, letting motion resume. Whoever calls this has decided
    /// the servos that reported them are fit to move.
    pub fn clear_faults(&mut self) {
        self.faults.clear();
    }

    /// The longest [`ActuatorChain::stop`] takes on this chain: three exchanges per servo,
    /// each bounded by the link's [`SerialPort::timeout`].
    #[must_use]
    pub fn stop_budget(&self) -> Duration {
        let servos = u32::try_from(self.servos.len()).unwrap_or(u32::MAX);
        self.sms
            .ph
            .port()
            .timeout()
            .saturating_mul(STOP_EXCHANGES_PER_SERVO.saturating_mul(servos))
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
    /// The SYNC WRITE that carries a command has no answer.
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
        if !self.enabled {
            self.enable_torque()?;
            self.enabled = true;
        }
        let mut writer = GroupSyncWrite::new(SMS_STS_GOAL_POSITION_L, 2);
        for (servo, goal) in self.servos.iter().zip(goals) {
            writer.add_param(
                servo.scs_id(),
                &[self.sms.ph.scs_lobyte(goal), self.sms.ph.scs_hibyte(goal)],
            );
        }
        match writer.tx_packet(&mut self.sms.ph) {
            CommResult::Success => Ok(Receipt {
                evidence: Evidence::DispatchAttempted,
                observed: Vec::new(),
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
        report
    }
}
