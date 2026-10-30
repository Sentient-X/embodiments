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
use crate::observe::FeetechObserver;

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
    /// Whether this bus's servos take their approach limit from the registry (the bindings'
    /// `approach` for the port) and refuse to open without one: a body whose servos must never
    /// run at the firmware's full speed. The SO-101 follower keeps `LeRobot`'s registers.
    pub registry_approach: bool,
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
            if body.port == port && BodyBindings::parse(body.bindings)?.is(embodiment) {
                return Ok(Some(body));
            }
        }
        Ok(None)
    }

    /// The bus among [`BODIES`] on `port` of the body the registry names `body`: the twin a
    /// composed body's bus repeats (`BodyBindings::twin_bus`).
    ///
    /// # Errors
    ///
    /// Returns why a body's bindings do not parse.
    pub fn named(body: &str, port: &str) -> Result<Option<&'static Self>, BodyError> {
        for candidate in BODIES {
            if candidate.port == port && BodyBindings::parse(candidate.bindings)?.embodiment == body
            {
                return Ok(Some(candidate));
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
            if BodyBindings::parse(body.bindings)?.is(embodiment) {
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

    /// Every joint's setup, in native state order, its servo's approach limit the registry's
    /// for this port (the bindings' `approach`), else the setup's own.
    ///
    /// # Errors
    ///
    /// Bindings that do not parse, and a bus that takes its approach from the registry when
    /// the registry declares none for its port.
    pub fn setups(&self) -> Result<Vec<ServoSetup>, BodyError> {
        let bindings = BodyBindings::parse(self.bindings)?;
        let approach = bindings.approach.get(self.port).copied();
        if self.registry_approach && approach.is_none() {
            return Err(BodyError::NoApproach {
                embodiment: bindings.embodiment,
                port: self.port.to_owned(),
            });
        }
        Ok(self
            .servos
            .iter()
            .map(|setup| ServoSetup {
                configuration: ServoConfiguration {
                    approach: approach.or(setup.configuration.approach),
                    ..setup.configuration
                },
                ..*setup
            })
            .collect())
    }

    /// Each axis with its setup and its unit's calibration, `calibration` in native state
    /// order. The servos' own approach limit is the registry's for this port (the bindings'
    /// `approach`), else the setup's.
    ///
    /// # Errors
    ///
    /// Refuses a calibration list that is not one per servo, and bindings that do not parse.
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
            .zip(self.setups()?)
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

    /// Read every servo on `port` as `drive`'s robot drives it, never commanding one: the
    /// servos [`FeetechBody::open`] opens, in its order, each read with the calibration
    /// `calibration` supplies for it in native state order, or with its registers' own where it
    /// supplies none ([`FeetechObserver`]).
    ///
    /// # Errors
    ///
    /// Returns why the observer did not open.
    pub fn observe<P: SerialPort>(
        &self,
        port: P,
        drive: &InstanceDrive,
        calibration: &[Option<MotorCalibration>],
    ) -> Result<FeetechObserver<P>, FeetechChainError> {
        FeetechObserver::open(port, self.calibrated(drive)?, &self.setups()?, calibration)
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

    #[test]
    fn a_registry_approach_bus_takes_its_ports_limit_and_refuses_to_open_without_one() {
        for body in BODIES {
            let bindings = BodyBindings::parse(body.bindings).expect("bindings");
            let declared = bindings.approach.get(body.port).copied();
            let setups = body.setups().expect("setups");
            if body.registry_approach {
                assert!(declared.is_some(), "{} has no approach", body.port);
                assert!(
                    setups
                        .iter()
                        .all(|setup| setup.configuration.approach == declared)
                );
            } else {
                assert!(
                    setups
                        .iter()
                        .all(|setup| setup.configuration.approach.is_none())
                );
            }
        }
        // The static upper body's neck and both grippers, with the registry's limits gone.
        let stripped: &'static str = Box::leak(
            crate::static_upper_body::BINDINGS
                .split("\n  \"approach\"")
                .next()
                .expect("axes")
                .trim_end_matches(',')
                .to_owned()
                .into_boxed_str(),
        );
        let stripped: &'static str = Box::leak(format!("{stripped}\n}}\n").into_boxed_str());
        for body in [
            crate::static_upper_body::NECK,
            crate::static_upper_body::RIGHT_GRIPPER,
            crate::static_upper_body::LEFT_GRIPPER,
        ] {
            let body = FeetechBody {
                bindings: stripped,
                ..body
            };
            assert!(
                matches!(body.setups(), Err(BodyError::NoApproach { .. })),
                "{}",
                body.port
            );
        }
    }
}
