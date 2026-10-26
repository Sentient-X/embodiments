//! Dynamixel servos read and never commanded: a [`DynamixelObserver`] reads the servos a
//! [`DynamixelChain`](crate::chain::DynamixelChain) would open, in the same order and joint
//! coordinates, through [`ReadOnlyPort`], which refuses any packet but a `PING` or a `READ`
//! addressed to one servo.
//!
//! Each servo is first identified: its ping must answer its binding's model number, and its
//! calibration registers (`Homing Offset`, the position limits, `Drive Mode`) are read as
//! [`DynamixelChain::open`](crate::chain::DynamixelChain::open) reads them. A calibration
//! supplied for the servo must equal them, as the chain requires; with none supplied the
//! registers are the calibration. Each poll then reads every identified servo's `Present
//! Position` with the chain's own code. Nothing is written, so a servo stays as it was.
//!
//! A servo that does not answer its identification reads missing until it answers. One that
//! answers as another model or holds another calibration is refused at open, and reads corrupt
//! if it is first found so later.

use std::io;
use std::time::Duration;

use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{AxisReading, BoundAxis, MissingReason, Observe};

use crate::body::ServoSetup;
use crate::chain::{
    DynamixelChainError, DynamixelServo, MotorCalibration, check_calibration, held_calibration,
    model_number, present_position, qualify, unanswered,
};
use crate::dynamixel_sdk::{
    BROADCAST_ID, CommResult, ERRBIT_ALERT, INST_PING, INST_READ, MAX_ID, PKT_ID, PKT_INSTRUCTION,
    PKT_LENGTH_H, PKT_LENGTH_L, Protocol2PacketHandler,
};

/// Whether `packet` only asks: a Protocol 2.0 `PING` or `READ` addressed to one servo.
#[must_use]
pub fn is_read_only(packet: &[u8]) -> bool {
    if packet.len() <= PKT_INSTRUCTION || packet[..4] != [0xFF, 0xFF, 0xFD, 0x00] {
        return false;
    }
    let length = usize::from(u16::from_le_bytes([
        packet[PKT_LENGTH_L],
        packet[PKT_LENGTH_H],
    ]));
    packet.len() == length + PKT_INSTRUCTION
        && packet[PKT_ID] <= MAX_ID
        && packet[PKT_ID] != BROADCAST_ID
        && match packet[PKT_INSTRUCTION] {
            INST_PING => length == 3,
            INST_READ => length == 7,
            _ => false,
        }
}

/// A port that carries only [`is_read_only`] packets; any other write is refused before the
/// link sees a byte.
pub struct ReadOnlyPort<P>(P);

impl<P> ReadOnlyPort<P> {
    pub const fn new(port: P) -> Self {
        Self(port)
    }

    pub const fn inner(&self) -> &P {
        &self.0
    }
}

impl<P: SerialPort> SerialPort for ReadOnlyPort<P> {
    fn clear_port(&mut self) -> io::Result<()> {
        self.0.clear_port()
    }

    fn write_port(&mut self, packet: &[u8]) -> io::Result<usize> {
        if !is_read_only(packet) {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "an observer sends only PING and READ",
            ));
        }
        self.0.write_port(packet)
    }

    fn read_port(&mut self, length: usize) -> io::Result<Vec<u8>> {
        self.0.read_port(length)
    }

    fn timeout(&self) -> Duration {
        self.0.timeout()
    }

    fn set_packet_timeout(&mut self, packet_length: usize) {
        self.0.set_packet_timeout(packet_length);
    }

    fn is_packet_timeout(&mut self) -> bool {
        self.0.is_packet_timeout()
    }
}

/// One servo, identified or not.
enum Unit {
    Unidentified {
        axis: BoundAxis,
        setup: ServoSetup,
        supplied: Option<MotorCalibration>,
    },
    Identified(DynamixelServo),
    Refused,
}

/// Why a servo is not identified.
enum Unidentified {
    /// It did not answer, or answered with an error: its reading this poll.
    Silent(AxisReading),
    /// It is not the servo it is bound as.
    Refused(DynamixelChainError),
}

/// Dynamixel servos on one serial link, read only, in native state order.
pub struct DynamixelObserver<P> {
    ph: Protocol2PacketHandler<ReadOnlyPort<P>>,
    units: Vec<Unit>,
    axes: Vec<BoundAxis>,
}

impl<P: SerialPort> DynamixelObserver<P> {
    /// Validate `axes` as [`DynamixelChain::open`](crate::chain::DynamixelChain::open) does
    /// before any byte, and identify each servo. `setups` and `supplied` give each axis's setup
    /// and the calibration supplied for it, if any, in the same order.
    ///
    /// # Errors
    ///
    /// Refuses an invalid or unknown axis, lists that are not one per axis, an out-of-range
    /// calibration, a servo answering as another model, and a servo whose registers disagree
    /// with the calibration supplied for it.
    pub fn open(
        port: P,
        axes: Vec<BoundAxis>,
        setups: &[ServoSetup],
        supplied: &[Option<MotorCalibration>],
    ) -> Result<Self, DynamixelChainError> {
        qualify(&axes)?;
        if supplied.len() != axes.len() || setups.len() != axes.len() {
            return Err(DynamixelChainError::CalibrationCount {
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
            ph: Protocol2PacketHandler::new(ReadOnlyPort::new(port)),
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
        match self.calibration(&axis, supplied) {
            Ok(calibration) => {
                self.units[index] = Unit::Identified(DynamixelServo {
                    axis,
                    norm_mode: setup.norm_mode,
                    calibration,
                    operating_mode: setup.operating_mode,
                    approach: setup.approach,
                    goal_current: setup.goal_current,
                    multi_turn: setup.multi_turn,
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

    fn calibration(
        &mut self,
        axis: &BoundAxis,
        supplied: Option<MotorCalibration>,
    ) -> Result<MotorCalibration, Unidentified> {
        let bus_id = axis.binding.bus_id;
        // `qualify` refused an id above the protocol's last.
        let dxl_id = u8::try_from(bus_id).unwrap_or(u8::MAX);
        let (model, result, error) = self.ph.ping(dxl_id);
        let error = error & !ERRBIT_ALERT;
        if result != CommResult::Success || error != 0 {
            return Err(Unidentified::Silent(unanswered(result, error)));
        }
        let expected = model_number(axis.binding.model).unwrap_or(0);
        if model != expected {
            return Err(Unidentified::Refused(
                DynamixelChainError::UnqualifiedServo {
                    bus_id,
                    model,
                    expected,
                },
            ));
        }
        let held = match held_calibration(&mut self.ph, bus_id, dxl_id) {
            Ok(held) => held,
            Err(DynamixelChainError::Comm { result, .. }) => {
                return Err(Unidentified::Silent(unanswered(result, 0)));
            }
            Err(DynamixelChainError::ServoError { error, .. }) => {
                return Err(Unidentified::Silent(AxisReading::Faulted(error)));
            }
            Err(error) => return Err(Unidentified::Refused(error)),
        };
        match supplied {
            Some(calibration) if calibration != held => {
                Err(Unidentified::Refused(DynamixelChainError::Uncalibrated {
                    bus_id,
                    homing_offset: i32::from(held.homing_offset),
                    range_min: u32::from(held.range_min),
                    range_max: u32::from(held.range_max),
                    drive_mode: u8::from(held.drive_mode),
                }))
            }
            Some(calibration) => Ok(calibration),
            None => check_calibration(bus_id, held)
                .map(|()| held)
                .map_err(Unidentified::Refused),
        }
    }

    #[must_use]
    pub const fn port(&self) -> &P {
        self.ph.port().inner()
    }
}

impl<P: SerialPort> Observe for DynamixelObserver<P> {
    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    /// Each identified servo's `Present Position`, read as the chain reads it; a servo not yet
    /// identified is identified first and reads missing (or faulted) while it does not answer;
    /// a refused servo reads corrupt.
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
