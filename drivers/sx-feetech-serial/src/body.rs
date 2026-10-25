//! A Feetech body on one serial bus, opened from data: its rendered bindings and each joint's
//! wire normalization and servo configuration.
//!
//! Every body whose Feetech servos share one bus is one [`FeetechBody`] constant: the SO-101
//! follower ([`crate::so101::SO101`]) and the static upper body's neck
//! ([`crate::static_upper_body::NECK`]), as `sx_damiao_can::body::DamiaoBody` is for Damiao
//! bodies. No bound, address, model, sign or reduction is spelled in Rust; the bindings are
//! the registry's, rendered by `tools/render_driver_bindings.py`. What a constant adds is keyed
//! by joint name and refused unless its joints are the bindings' Feetech joints in their
//! order. A servo's `LeRobot` calibration is per unit, so the caller supplies one per servo, in
//! native state order, and [`FeetechChain::open`] refuses a servo whose registers disagree.

use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{BodyBindings, BodyError, BoundAxis, InstanceDrive};

use crate::chain::{
    FEETECH_SERIAL, FeetechChain, FeetechChainError, FeetechServo, MotorCalibration, MotorNormMode,
    ServoConfiguration,
};

/// What one joint's servo needs beyond the registry's binding and the unit's calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoSetup {
    pub joint: &'static str,
    pub norm_mode: MotorNormMode,
    pub configuration: ServoConfiguration,
}

/// One Feetech body on one bus.
#[derive(Clone, Copy, Debug)]
pub struct FeetechBody {
    /// The registry's rendered bindings (`bindings/<body>.json`).
    pub bindings: &'static str,
    /// The bus's rate, in baud.
    pub baud: u32,
    /// Every Feetech joint's setup, in the bindings' native state order.
    pub servos: &'static [ServoSetup],
}

/// Every Feetech body this crate opens.
pub const BODIES: [&FeetechBody; 2] = [&crate::so101::SO101, &crate::static_upper_body::NECK];

impl FeetechBody {
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

    /// `body`'s Feetech axes, refused unless they are on one bus and are the joints set up
    /// here. `body` is this constant's bindings, calibrated by an instance drive or not.
    ///
    /// # Errors
    ///
    /// Refuses other joints and Feetech axes on more than one port.
    pub fn axes(&self, body: BodyBindings) -> Result<Vec<BoundAxis>, BodyError> {
        let body = body.on(FEETECH_SERIAL);
        let joints: Vec<&str> = self.servos.iter().map(|setup| setup.joint).collect();
        body.require_joints(&joints)?;
        body.one_bus()?;
        Ok(body.axes)
    }

    /// The rendered body's Feetech axes as `drive`'s robot drives them.
    ///
    /// # Errors
    ///
    /// Refuses a drive of another body and what [`FeetechBody::axes`] refuses.
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
    ) -> Result<Vec<FeetechServo>, FeetechChainError> {
        if calibration.len() != axes.len() {
            return Err(FeetechChainError::CalibrationCount {
                expected: axes.len(),
                found: calibration.len(),
            });
        }
        Ok(axes
            .into_iter()
            .zip(self.servos)
            .zip(calibration)
            .map(|((axis, setup), calibration)| FeetechServo {
                axis,
                norm_mode: setup.norm_mode,
                calibration: *calibration,
                configuration: setup.configuration,
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
    ) -> Result<FeetechChain<P>, FeetechChainError> {
        FeetechChain::open(port, self.servos(self.calibrated(drive)?, calibration)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_bodys_setups_name_its_bindings_feetech_joints_in_order() {
        for body in BODIES {
            let bindings = BodyBindings::parse(body.bindings).expect("bindings");
            let axes = body.axes(bindings.clone()).expect("joints");
            assert_eq!(axes.len(), body.servos.len());
            assert_eq!(
                FeetechBody::of(&bindings.id)
                    .expect("parses")
                    .map(|found| found.bindings),
                Some(body.bindings)
            );
        }
        assert!(FeetechBody::of("another").expect("parses").is_none());
    }
}
