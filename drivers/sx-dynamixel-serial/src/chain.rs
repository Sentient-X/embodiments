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
//! - Command enables a disabled chain as `enable_torque` does (`dynamixel.py:199-201`):
//!   `Torque_Enable` 1 per servo, each acknowledged. It then sends one broadcast SYNC WRITE of
//!   every `Goal_Position` (116, four bytes, two's complement as `_encode_sign` writes it), as
//!   `sync_write("Goal_Position", ...)` does. The bus answers nothing to it, so a command's
//!   receipt claims only a dispatch.
//! - Stop disables as `disable_torque` does (`dynamixel.py:191-193`): `Torque_Enable` 0 on
//!   every servo, each acknowledged, whatever an earlier one answered. Then it reads every
//!   servo's seven bytes from `Torque_Enable` (64) to `Hardware_Error_Status` (70) in one READ;
//!   a servo is proven stopped only when its `Torque_Enable` reads 0.
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

use std::time::Duration;

use sx_embodiment_drivers::lerobot::unnormalize;
pub use sx_embodiment_drivers::lerobot::{MotorCalibration, MotorNormMode};
use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{
    ActuatorBus, ActuatorChain, ActuatorModel, BindingError, BodyError, BoundAxis, ChainError,
    ChainFailure, CommandRange, Evidence, Receipt, StopReport, TargetError, validate_chain,
};
use thiserror::Error;

use crate::dynamixel_sdk::{
    CommResult, GroupSyncWrite, MAX_ID, Protocol2PacketHandler, dxl_hiword, dxl_loword,
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
/// Exchanges a stop makes per servo: torque off, and the read-back.
const STOP_EXCHANGES_PER_SERVO: u32 = 2;
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
}

impl DynamixelServo {
    fn dxl_id(&self) -> u8 {
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

/// A four-byte register's two's-complement value.
#[allow(clippy::cast_possible_wrap)]
const fn signed(value: u32) -> i32 {
    value as i32
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
        validate_chain(&axes)?;
        for servo in &servos {
            let binding = servo.axis.binding;
            if binding.bus != DYNAMIXEL_SERIAL || model_number(binding.model).is_none() {
                return Err(DynamixelChainError::Unqualified(binding.bus_id));
            }
            if binding.bus_id > u16::from(MAX_ID) {
                return Err(DynamixelChainError::Ids(binding.bus_id));
            }
            let MotorCalibration {
                range_min,
                range_max,
                ..
            } = servo.calibration;
            if range_min >= range_max || f64::from(range_max) >= RESOLUTION {
                return Err(DynamixelChainError::Calibration(binding.bus_id));
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
        };
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
            let homing_offset = signed(self.read4(bus_id, dxl_id, HOMING_OFFSET)?);
            let range_min = self.read4(bus_id, dxl_id, MIN_POSITION_LIMIT)?;
            let range_max = self.read4(bus_id, dxl_id, MAX_POSITION_LIMIT)?;
            let (drive_mode, result, error) = self.ph.read1_byte_tx_rx(dxl_id, DRIVE_MODE);
            expect(bus_id, (result, error))?;
            if homing_offset != i32::from(calibration.homing_offset)
                || range_min != u32::from(calibration.range_min)
                || range_max != u32::from(calibration.range_max)
                || drive_mode != u8::from(calibration.drive_mode)
            {
                return Err(DynamixelChainError::Uncalibrated {
                    bus_id,
                    homing_offset,
                    range_min,
                    range_max,
                    drive_mode,
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
        Ok(())
    }

    /// `enable_torque`: `Torque_Enable` 1 on every servo.
    fn enable_torque(&mut self) -> Result<(), DynamixelChainError> {
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
            .collect::<Result<Vec<u32>, _>>()?;
        if !self.enabled {
            self.enable_torque()?;
            self.enabled = true;
        }
        let mut writer = GroupSyncWrite::new(GOAL_POSITION, 4);
        for (servo, goal) in self.servos.iter().zip(goals) {
            let (low, high) = (dxl_loword(goal), dxl_hiword(goal));
            writer.add_param(
                servo.dxl_id(),
                &[
                    crate::dynamixel_sdk::dxl_lobyte(low),
                    crate::dynamixel_sdk::dxl_hibyte(low),
                    crate::dynamixel_sdk::dxl_lobyte(high),
                    crate::dynamixel_sdk::dxl_hibyte(high),
                ],
            );
        }
        match writer.tx_packet(&mut self.ph) {
            CommResult::Success => Ok(Receipt {
                evidence: Evidence::DispatchAttempted,
                observed: Vec::new(),
                feedback: Vec::new(),
            }),
            result => Err(DynamixelChainError::SyncWrite(result)),
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
        report
    }

    /// The longest [`ActuatorChain::stop`] takes on this chain: two exchanges per servo, each
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
}

impl ChainError for DynamixelChainError {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::Body(_)
            | Self::Target(_)
            | Self::OutsideCalibration { .. }
            | Self::CalibrationCount { .. }
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
