//! A Dynamixel body on one serial bus, opened from data: its rendered bindings, its bus's
//! rate, and each joint's wire normalization and operating mode.
//!
//! The Dynamixel counterpart of `sx_feetech_serial::body::FeetechBody`. No registered body
//! binds a Dynamixel yet, so [`BODIES`] is empty: the static upper body's grippers are bound
//! once the robot is probed (their model, ids, rate and whether one servo drives both fingers
//! are unknown), and then are a registry binding, a rendered bindings file beside this crate,
//! and one [`DynamixelBody`] constant listed here, with no code.

use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{ApproachLimit, BodyBindings, BodyError, BoundAxis, InstanceDrive};

use crate::chain::{
    DYNAMIXEL_SERIAL, DynamixelChain, DynamixelChainError, DynamixelServo, MotorCalibration,
    MotorNormMode, OperatingMode,
};

/// What one joint's servo needs beyond the registry's binding and the unit's calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoSetup {
    pub joint: &'static str,
    pub norm_mode: MotorNormMode,
    pub operating_mode: OperatingMode,
    /// The speed the servo itself approaches any goal at (`DynamixelServo::approach`).
    pub approach: ApproachLimit,
    /// `Goal Current`, declared exactly for a current-based position servo.
    pub goal_current: Option<u16>,
    /// Whether the servo may turn more than once (the extended and current-based modes).
    pub multi_turn: bool,
}

/// One Dynamixel body on one bus.
#[derive(Clone, Copy, Debug)]
pub struct DynamixelBody {
    /// The registry's rendered bindings (`bindings/<body>.json`).
    pub bindings: &'static str,
    /// The bus's rate, in baud.
    pub baud: u32,
    /// Every Dynamixel joint's setup, in the bindings' native state order.
    pub servos: &'static [ServoSetup],
}

/// Every Dynamixel body this crate opens: none until a body is probed and bound.
pub const BODIES: [&DynamixelBody; 0] = [];

impl DynamixelBody {
    /// The body among [`BODIES`] whose bindings carry `embodiment`'s content id.
    ///
    /// # Errors
    ///
    /// Returns why a body's bindings do not parse.
    pub fn of(embodiment: &str) -> Result<Option<&'static Self>, BodyError> {
        for body in BODIES {
            if BodyBindings::parse(body.bindings)?.id == embodiment {
                return Ok(Some(body));
            }
        }
        Ok(None)
    }

    /// `body`'s Dynamixel axes, refused unless they are on one bus and are the joints set up
    /// here.
    ///
    /// # Errors
    ///
    /// Refuses other joints and Dynamixel axes on more than one port.
    pub fn axes(&self, body: BodyBindings) -> Result<Vec<BoundAxis>, BodyError> {
        let body = body.on(DYNAMIXEL_SERIAL);
        let joints: Vec<&str> = self.servos.iter().map(|setup| setup.joint).collect();
        body.require_joints(&joints)?;
        body.one_bus()?;
        Ok(body.axes)
    }

    /// The rendered body's Dynamixel axes as `drive`'s robot drives them.
    ///
    /// # Errors
    ///
    /// Refuses a drive of another body and what [`DynamixelBody::axes`] refuses.
    pub fn calibrated(&self, drive: &InstanceDrive) -> Result<Vec<BoundAxis>, BodyError> {
        self.axes(BodyBindings::parse(self.bindings)?.calibrated(drive)?)
    }

    /// Each axis with its setup and its unit's calibration, `calibration` in native state
    /// order.
    ///
    /// # Errors
    ///
    /// Refuses a calibration list that is not one per servo.
    pub fn servos(
        &self,
        axes: Vec<BoundAxis>,
        calibration: &[MotorCalibration],
    ) -> Result<Vec<DynamixelServo>, DynamixelChainError> {
        if calibration.len() != axes.len() {
            return Err(DynamixelChainError::CalibrationCount {
                expected: axes.len(),
                found: calibration.len(),
            });
        }
        Ok(axes
            .into_iter()
            .zip(self.servos)
            .zip(calibration)
            .map(|((axis, setup), calibration)| DynamixelServo {
                axis,
                norm_mode: setup.norm_mode,
                calibration: *calibration,
                operating_mode: setup.operating_mode,
                approach: setup.approach,
                goal_current: setup.goal_current,
                multi_turn: setup.multi_turn,
            })
            .collect())
    }

    /// Open every servo on `port` as `drive`'s robot drives it, each unit calibrated by
    /// `calibration` in native state order, configured and left with torque off.
    ///
    /// # Errors
    ///
    /// Returns why the chain did not open.
    pub fn open<P: SerialPort>(
        &self,
        port: P,
        drive: &InstanceDrive,
        calibration: &[MotorCalibration],
    ) -> Result<DynamixelChain<P>, DynamixelChainError> {
        DynamixelChain::open(port, self.servos(self.calibrated(drive)?, calibration)?)
    }
}
