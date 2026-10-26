//! Feetech servos read and never commanded: a [`FeetechObserver`] reads the servos a
//! [`FeetechChain`](crate::chain::FeetechChain) would open, in the same order and joint
//! coordinates, through [`ReadOnlyPort`], which refuses any packet but a `PING` or a `READ`.
//!
//! Each servo is first identified, by `READ`s only: its model number must be the STS3215's, and
//! its calibration registers (`Min_Position_Limit`, `Max_Position_Limit`, `Homing_Offset`) are
//! read as [`FeetechChain::open`](crate::chain::FeetechChain::open) reads them. A calibration
//! supplied for the servo must equal them, as the chain requires, and is then used with its
//! drive mode; with none supplied the registers are the calibration (drive mode 0, as `LeRobot`
//! records for the STS, which has no drive-mode register). Each poll then reads every
//! identified servo's `Present_Position` with the chain's own code. No torque, goal, lock or
//! configuration register is written, so a servo stays as it was.
//!
//! A servo that does not answer its identification is not refused: it reads missing until it
//! answers, when it is identified then. A servo that answers as another model, or whose
//! registers disagree with the calibration supplied, is refused at open and read as corrupt if
//! it is first found so later.

use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{AxisReading, BoundAxis, MissingReason, Observe};

use crate::body::ServoSetup;
use crate::chain::{
    FeetechChainError, FeetechServo, MotorCalibration, STS3215_MODEL_NUMBER, check_calibration,
    held_calibration, present_position, qualify, unanswered,
};
use crate::scan::ReadOnlyPort;
use crate::scservo_sdk::{CommResult, ProtocolPacketHandler, SMS_STS_MODEL_L};

/// One servo, identified or not.
enum Unit {
    /// Not yet answered: its servo as set up, and the calibration supplied for it.
    Unidentified {
        axis: BoundAxis,
        setup: ServoSetup,
        supplied: Option<MotorCalibration>,
    },
    /// Answering as an STS3215 holding this calibration.
    Identified(FeetechServo),
    /// Answering as another model, or holding another calibration than the one supplied.
    Refused,
}

/// Feetech servos on one serial link, read only, in native state order.
pub struct FeetechObserver<P> {
    ph: ProtocolPacketHandler<ReadOnlyPort<P>>,
    units: Vec<Unit>,
    axes: Vec<BoundAxis>,
}

/// Why a servo is not identified.
enum Unidentified {
    /// It did not answer, or answered with error flags: its reading this poll.
    Silent(AxisReading),
    /// It is not the servo it is bound as.
    Refused(FeetechChainError),
}

impl<P: SerialPort> FeetechObserver<P> {
    /// Validate `axes` as [`FeetechChain::open`](crate::chain::FeetechChain::open) does
    /// before any byte, and identify each servo. `setups` and `supplied` give each axis's setup
    /// and the calibration supplied for it, if any, in the same order.
    ///
    /// # Errors
    ///
    /// Refuses an invalid or unqualified axis, a supplied calibration that is not one per servo
    /// or is out of range, a servo answering as another model, and a servo whose registers
    /// disagree with the calibration supplied for it.
    pub fn open(
        port: P,
        axes: Vec<BoundAxis>,
        setups: &[ServoSetup],
        supplied: &[Option<MotorCalibration>],
    ) -> Result<Self, FeetechChainError> {
        qualify(&axes)?;
        if supplied.len() != axes.len() || setups.len() != axes.len() {
            return Err(FeetechChainError::CalibrationCount {
                expected: axes.len(),
                found: supplied.len(),
            });
        }
        for (axis, calibration) in axes.iter().zip(supplied) {
            if let Some(calibration) = calibration {
                check_calibration(axis.binding.bus_id, *calibration)?;
            }
        }
        let mut observer = Self {
            ph: ProtocolPacketHandler::new(ReadOnlyPort::new(port), 0),
            units: axes
                .iter()
                .zip(setups)
                .zip(supplied)
                .map(|((axis, setup), supplied)| Unit::Unidentified {
                    axis: axis.clone(),
                    setup: *setup,
                    supplied: *supplied,
                })
                .collect(),
            axes,
        };
        for index in 0..observer.units.len() {
            if let Err(Unidentified::Refused(error)) = observer.identify(index) {
                return Err(error);
            }
        }
        Ok(observer)
    }

    /// Identify unit `index` if it is not yet: its model, then its calibration registers.
    fn identify(&mut self, index: usize) -> Result<(), Unidentified> {
        let Unit::Unidentified {
            axis,
            setup,
            supplied,
        } = &self.units[index]
        else {
            return Ok(());
        };
        let (axis, setup, supplied) = (axis.clone(), *setup, *supplied);
        match self.calibration(axis.binding.bus_id, supplied) {
            Ok(calibration) => {
                self.units[index] = Unit::Identified(FeetechServo {
                    axis,
                    norm_mode: setup.norm_mode,
                    calibration,
                    configuration: setup.configuration,
                });
                Ok(())
            }
            Err(Unidentified::Refused(error)) => {
                self.units[index] = Unit::Refused;
                Err(Unidentified::Refused(error))
            }
            Err(silent) => Err(silent),
        }
    }

    /// The calibration the servo at `bus_id` is read with, after its model is checked.
    fn calibration(
        &mut self,
        bus_id: u16,
        supplied: Option<MotorCalibration>,
    ) -> Result<MotorCalibration, Unidentified> {
        // `qualify` refused an id above the protocol's last.
        let scs_id = u8::try_from(bus_id).unwrap_or(u8::MAX);
        let (model, result, error) = self.ph.read2_byte_tx_rx(scs_id, SMS_STS_MODEL_L);
        if result != CommResult::Success || error != 0 {
            return Err(Unidentified::Silent(unanswered(result, error)));
        }
        if model != STS3215_MODEL_NUMBER {
            return Err(Unidentified::Refused(FeetechChainError::UnqualifiedServo {
                bus_id,
                model,
            }));
        }
        let held = match held_calibration(&mut self.ph, bus_id, scs_id) {
            Ok(held) => held,
            Err(FeetechChainError::Comm { result, .. }) => {
                return Err(Unidentified::Silent(unanswered(result, 0)));
            }
            Err(FeetechChainError::ServoError { error, .. }) => {
                return Err(Unidentified::Silent(AxisReading::Faulted(error)));
            }
            Err(error) => return Err(Unidentified::Refused(error)),
        };
        match supplied {
            Some(calibration)
                if (
                    calibration.range_min,
                    calibration.range_max,
                    calibration.homing_offset,
                ) != (held.range_min, held.range_max, held.homing_offset) =>
            {
                Err(Unidentified::Refused(FeetechChainError::Uncalibrated {
                    bus_id,
                    homing_offset: i32::from(held.homing_offset),
                    range_min: held.range_min,
                    range_max: held.range_max,
                }))
            }
            Some(calibration) => Ok(calibration),
            None => check_calibration(bus_id, held)
                .map(|()| held)
                .map_err(Unidentified::Refused),
        }
    }

    /// The calibration each servo is read with, in native state order; `None` for a servo
    /// not identified.
    #[must_use]
    pub fn calibrations(&self) -> Vec<Option<MotorCalibration>> {
        self.units
            .iter()
            .map(|unit| match unit {
                Unit::Identified(servo) => Some(servo.calibration),
                Unit::Unidentified { .. } | Unit::Refused => None,
            })
            .collect()
    }

    #[must_use]
    pub const fn port(&self) -> &P {
        self.ph.port().inner()
    }
}

impl<P: SerialPort> Observe for FeetechObserver<P> {
    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    /// Each identified servo's `Present_Position`, read as the chain reads it; a servo not yet
    /// identified is identified first, and reads missing (or faulted with its flags) while it
    /// does not answer; a refused servo reads corrupt.
    fn read_positions(&mut self) -> sx_embodiment_drivers::Axes<AxisReading> {
        (0..self.units.len())
            .map(|index| match self.identify(index) {
                Err(Unidentified::Silent(reading)) => reading,
                Err(Unidentified::Refused(_)) => AxisReading::Missing(MissingReason::Corrupt),
                Ok(()) => match &self.units[index] {
                    Unit::Identified(servo) => present_position(&mut self.ph, servo),
                    Unit::Unidentified { .. } | Unit::Refused => {
                        AxisReading::Missing(MissingReason::Corrupt)
                    }
                },
            })
            .collect()
    }
}
