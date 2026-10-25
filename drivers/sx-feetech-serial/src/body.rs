//! A Feetech body on one serial bus, opened from data: its rendered bindings and each joint's
//! wire normalization and servo configuration.
//!
//! Every Feetech bus of a body is one [`FeetechBody`] constant, named by the port its bindings
//! put it on: the SO-101 follower ([`crate::so101::SO101`], port `serial`), and the static
//! upper body's neck and two grippers ([`crate::static_upper_body`], ports `neck`,
//! `right_gripper` and `left_gripper`), as `sx_damiao_can::body::DamiaoBody` is for a Damiao
//! bus. No bound, address, model, sign or reduction is spelled in Rust; the bindings are
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

/// One Feetech bus of a body.
#[derive(Clone, Copy, Debug)]
pub struct FeetechBody {
    /// The registry's rendered bindings (`bindings/<body>.json`).
    pub bindings: &'static str,
    /// The port the bindings put this bus's axes on.
    pub port: &'static str,
    /// The bus's rate, in baud.
    pub baud: u32,
    /// Every Feetech joint's setup, in the bindings' native state order.
    pub servos: &'static [ServoSetup],
}

/// Every Feetech body this crate opens.
pub const BODIES: [&FeetechBody; 4] = [
    &crate::so101::SO101,
    &crate::static_upper_body::NECK,
    &crate::static_upper_body::RIGHT_GRIPPER,
    &crate::static_upper_body::LEFT_GRIPPER,
];

impl FeetechBody {
    /// The bus among [`BODIES`] on `port` whose bindings carry `embodiment`'s content id.
    ///
    /// # Errors
    ///
    /// Returns why a body's bindings do not parse.
    pub fn of(embodiment: &str, port: &str) -> Result<Option<&'static Self>, BodyError> {
        for body in BODIES {
            if body.port == port && BodyBindings::parse(body.bindings)?.id == embodiment {
                return Ok(Some(body));
            }
        }
        Ok(None)
    }

    /// The rendered bindings of `embodiment`, when any bus among [`BODIES`] is one of its.
    ///
    /// # Errors
    ///
    /// Returns why a body's bindings do not parse.
    pub fn bindings_of(embodiment: &str) -> Result<Option<&'static str>, BodyError> {
        for body in BODIES {
            if BodyBindings::parse(body.bindings)?.id == embodiment {
                return Ok(Some(body.bindings));
            }
        }
        Ok(None)
    }

    /// `body`'s Feetech axes on this port, refused unless they are the joints set up here.
    /// `body` is this constant's bindings, calibrated by an instance drive or not.
    ///
    /// # Errors
    ///
    /// Refuses other joints.
    pub fn axes(&self, body: BodyBindings) -> Result<Vec<BoundAxis>, BodyError> {
        let mut body = body.on(FEETECH_SERIAL);
        body.axes.retain(|axis| axis.port == self.port);
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
            assert!(axes.iter().all(|axis| axis.port == body.port));
            assert_eq!(
                FeetechBody::of(&bindings.id, body.port)
                    .expect("parses")
                    .map(|found| found.port),
                Some(body.port)
            );
        }
        assert!(
            FeetechBody::of("another", "serial")
                .expect("parses")
                .is_none()
        );
        assert!(
            FeetechBody::bindings_of("another")
                .expect("parses")
                .is_none()
        );
    }
}
