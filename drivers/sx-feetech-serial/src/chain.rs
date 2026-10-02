//! A chain of bound Feetech STS3215 servos as one `ActuatorChain`, driven through
//! [`crate::scservo_sdk`].
//!
//! Every bus operation is an `scservo_sdk` call. The order they are called in, and what a
//! goal position means on the wire, are those of the driver the SO-101 is run with today,
//! `LeRobot`'s Feetech bus (<https://github.com/huggingface/lerobot> at `v0.6.0`,
//! `30da8e687a6dfc617fcd94afc367ac7071c376ce`):
//!
//! - Open reads each servo's model number (`read2ByteTxRx` at `SMS_STS_MODEL_L`) and refuses
//!   anything but the STS3215's 777 (`src/lerobot/motors/feetech/tables.py`,
//!   `MODEL_NUMBER_TABLE`). It leaves torque as it finds it, and the chain disabled.
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
//! - It refuses before any frame: a wrong width, a target outside the axis's declared bounds
//!   or not finite, a magnitude that reaches the sign bit (`LeRobot`'s `encode_sign_magnitude`
//!   raises there; the vendor's `scs_toscs` would set overlapping bits), a model or bus this
//!   crate is not qualified for, an address the protocol cannot carry, and an empty or
//!   out-of-resolution calibration.
//! - A target is admitted and converted at float32 precision, the precision `LeRobot`'s
//!   follower carries actions in: the target and the bounds are rounded to `f32` before
//!   comparison, and every later step runs on the rounded target in `f64`, as Python does. A
//!   target within half a float32 step of a bound is therefore admitted, which moves no servo
//!   by a tick.
//! - A stop is proven servo by servo by the torque read-back, not by the acknowledgements. The
//!   error bytes the servos report while stopping, such as overload or overheating, are faults
//!   of a stop that may still be proven. After an unproven stop every command is refused until
//!   a stop is proven.

use std::time::Duration;

use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{
    ActuatorBus, ActuatorChain, ActuatorModel, BindingError, BoundAxis, CommandRange, Evidence,
    Receipt, StopReport, TargetError, validate_chain,
};
use thiserror::Error;

use crate::scservo_sdk::{
    CommResult, GroupSyncWrite, MAX_ID, ProtocolPacketHandler, SMS_STS_GOAL_POSITION_L,
    SMS_STS_MODEL_L, SMS_STS_TORQUE_ENABLE, SmsSts,
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

/// How `LeRobot` normalizes a servo's coordinate on the wire (`MotorNormMode`).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotorNormMode {
    /// `DEGREES`: degrees about the middle of the calibrated range.
    Degrees,
    /// `RANGE_0_100`: percent of the calibrated range.
    Range0100,
}

/// One servo's per-unit calibration, as `LeRobot`'s `MotorCalibration` records it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MotorCalibration {
    /// An inverted drive; applied to percent servos only, as `_unnormalize` does.
    pub drive_mode: bool,
    /// The calibrated raw range, in encoder ticks.
    pub range_min: u16,
    pub range_max: u16,
}

/// One bound axis with what its Feetech servo needs beyond the registry's binding.
#[derive(Clone, Debug, PartialEq)]
pub struct FeetechServo {
    pub axis: BoundAxis,
    pub norm_mode: MotorNormMode,
    pub calibration: MotorCalibration,
}

impl FeetechServo {
    fn scs_id(&self) -> u8 {
        // Open refuses an address above `MAX_ID`, so the narrowing never saturates.
        u8::try_from(self.axis.binding.bus_id).unwrap_or(u8::MAX)
    }

    /// The goal-position register value that drives this servo to `joint`.
    ///
    /// # Errors
    ///
    /// Refuses a target outside the declared bounds or not finite, and a raw value whose
    /// magnitude reaches the sign bit.
    pub fn goal_position(&self, joint: f64) -> Result<u16, FeetechChainError> {
        let axis = &self.axis;
        // Rounding to the follower's float32 action precision is the intended conversion.
        #[allow(clippy::cast_possible_truncation)]
        let (single, lower, upper) = (joint as f32, axis.lower as f32, axis.upper as f32);
        if !single.is_finite() || single < lower || single > upper {
            return Err(TargetError::OutOfBounds {
                joint: axis.joint.clone(),
                value: joint,
            }
            .into());
        }
        let binding = axis.binding;
        let actuator = binding.actuator_from_joint(f64::from(single));
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
    #[error("an earlier stop is unproven; motion is refused until a stop is proven")]
    StopPending,
}

/// Bound Feetech servos on one serial link, in native state order.
pub struct FeetechChain<P> {
    sms: SmsSts<P>,
    servos: Vec<FeetechServo>,
    axes: Vec<BoundAxis>,
    commandable: Vec<CommandRange>,
    enabled: bool,
    stop_pending: bool,
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
    /// Validate every servo, then prove each answers as a qualified STS3215.
    ///
    /// # Errors
    ///
    /// Refuses an invalid or unqualified axis, an unaddressable id, an invalid calibration, a
    /// servo that does not answer or reports error flags, and any servo whose model is not
    /// the STS3215.
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
        let commandable = axes
            .iter()
            .map(|axis| CommandRange {
                lower: axis.lower,
                upper: axis.upper,
            })
            .collect();
        Ok(Self {
            sms,
            servos,
            axes,
            commandable,
            enabled: false,
            stop_pending: false,
        })
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
        report
    }
}
