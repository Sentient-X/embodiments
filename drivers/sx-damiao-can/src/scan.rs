//! What answers on a Damiao bus, read without enabling a motor or writing a register.
//!
//! [`scan`] asks each id the caller names for its `MST_ID` register with `DM_CAN.py`'s
//! `read_motor_param` ([`MotorControl::read_motor_param`]), and from every motor that answers
//! reads `ESC_ID`, `CTRL_MODE`, `PMAX`, `VMAX` and `TMAX` the same way. That is the register
//! half of [`crate::chain`]'s open without its mode switch (a register write) or its enable: a
//! register read needs nothing else from the motor, so nothing else is sent.
//!
//! The scan cannot write. It speaks through [`ReadOnlyCanPort`], which refuses any frame but a
//! register read (`0x7FF`, command byte `0x33`) before the link sees it: an enable (`FF..FC`),
//! a disable, a zero, an MIT command, a register write (`0x55`) or a flash save (`0xAA`) cannot
//! leave the host however the vendor layer is driven. The port also records each reply, so a
//! value is taken only from a frame naming the asked id and register, and the arbitration id
//! that frame arrived on (the motor's `MasterID`) is reported beside the register's own value.

use std::io;
use std::time::Duration;

use sx_embodiment_drivers::can::{CanFrame, CanPort};

use crate::dm_can::{
    DmCanError, DmMotorType, DmVariable, Motor, MotorControl, ParamValue, is_in_ranges,
    uint8s_to_float, uint8s_to_uint32,
};

/// The id register reads and writes are sent to.
const PARAM_ID: u32 = 0x7FF;
/// A register read's command byte; `0x55` writes and `0xAA` saves to flash.
const READ_REGISTER: u8 = 0x33;

/// Whether `frame` only asks: a register read on `0x7FF`.
#[must_use]
pub const fn is_read_only(frame: &CanFrame) -> bool {
    frame.id == PARAM_ID && frame.data[2] == READ_REGISTER
}

/// A port that carries only [`is_read_only`] frames, keeping every frame it receives.
pub struct ReadOnlyCanPort<P> {
    port: P,
    received: Vec<CanFrame>,
}

impl<P> ReadOnlyCanPort<P> {
    pub const fn new(port: P) -> Self {
        Self {
            port,
            received: Vec::new(),
        }
    }

    pub fn into_inner(self) -> P {
        self.port
    }
}

impl<P: CanPort> CanPort for ReadOnlyCanPort<P> {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        if !is_read_only(frame) {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "a scan sends only register reads",
            ));
        }
        self.port.send(frame)
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        let frame = self.port.receive()?;
        if let Some(frame) = frame {
            self.received.push(frame);
        }
        Ok(frame)
    }

    fn sleep(&mut self, duration: Duration) {
        self.port.sleep(duration);
    }
}

/// One motor that answered, as its registers read.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorScan {
    /// The id it was asked on (`SlaveID`, its `ESC_ID`).
    pub slave_id: u16,
    /// The arbitration id its replies arrived on.
    pub reply_id: u32,
    /// Its `MST_ID` register: the id its feedback is sent on.
    pub master_id: u32,
    pub esc_id: Option<u32>,
    /// `CTRL_MODE`: 1 MIT, 2 position-velocity, 3 velocity, 4 force-position.
    pub control_mode: Option<u32>,
    pub p_max: Option<f32>,
    pub v_max: Option<f32>,
    pub t_max: Option<f32>,
}

impl MotorScan {
    /// Whether the motor's position and torque ranges are `model`'s `DM_CAN.py` row. The
    /// torque range tells the qualified models apart (10, 28 and 54 N·m).
    #[must_use]
    pub fn fits(&self, model: DmMotorType) -> bool {
        let [p_max, _, t_max] = model.limit_param();
        #[allow(clippy::cast_possible_truncation)]
        let narrow = |value: f64| value as f32;
        self.p_max == Some(narrow(p_max)) && self.t_max == Some(narrow(t_max))
    }
}

/// What a bus answered.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BusScan {
    pub motors: Vec<MotorScan>,
    /// Ids that did not answer their `MST_ID` read.
    pub silent: Vec<u16>,
}

impl BusScan {
    #[must_use]
    pub fn motor(&self, slave_id: u16) -> Option<&MotorScan> {
        self.motors.iter().find(|motor| motor.slave_id == slave_id)
    }
}

/// The reply to `slave_id`'s read of `rid` among `received`: its arbitration id and value.
fn reply(received: &[CanFrame], slave_id: u16, rid: DmVariable) -> Option<(u32, ParamValue)> {
    let [low, high] = slave_id.to_le_bytes();
    received
        .iter()
        .rev()
        .find(|frame| {
            frame.data[..4] == [low, high, READ_REGISTER, rid as u8] && frame.id != PARAM_ID
        })
        .map(|frame| {
            let data = frame.data;
            let value = if is_in_ranges(rid as u8) {
                ParamValue::Uint32(uint8s_to_uint32(data[4], data[5], data[6], data[7]))
            } else {
                ParamValue::Float(uint8s_to_float(data[4], data[5], data[6], data[7]))
            };
            (frame.id, value)
        })
}

fn read<P: CanPort>(
    control: &mut MotorControl<ReadOnlyCanPort<P>>,
    slave_id: u16,
    rid: DmVariable,
) -> Result<Option<(u32, ParamValue)>, DmCanError> {
    control.port_mut().received.clear();
    control.read_motor_param(slave_id, rid)?;
    Ok(reply(&control.port().received, slave_id, rid))
}

const fn uint(value: Option<(u32, ParamValue)>) -> Option<u32> {
    match value {
        Some((_, ParamValue::Uint32(value))) => Some(value),
        _ => None,
    }
}

const fn float(value: Option<(u32, ParamValue)>) -> Option<f32> {
    match value {
        Some((_, ParamValue::Float(value))) => Some(value),
        _ => None,
    }
}

/// Ask each of `ids` on `port` for its registers. The port is the caller's, its bitrate the
/// operating system's.
///
/// # Errors
///
/// Returns the link's failure.
pub fn scan<P: CanPort>(
    port: P,
    ids: impl IntoIterator<Item = u16>,
) -> Result<BusScan, DmCanError> {
    let ids: Vec<u16> = ids.into_iter().collect();
    let mut control = MotorControl::new(ReadOnlyCanPort::new(port));
    for &id in &ids {
        // The model scales feedback frames only, which a register read never decodes; a
        // master id of 0 registers the motor under its slave id alone.
        control.add_motor(Motor::new(DmMotorType::DM4310, id, 0));
    }
    let mut found = BusScan::default();
    for slave_id in ids {
        let Some((reply_id, ParamValue::Uint32(master_id))) =
            read(&mut control, slave_id, DmVariable::MST_ID)?
        else {
            found.silent.push(slave_id);
            continue;
        };
        found.motors.push(MotorScan {
            slave_id,
            reply_id,
            master_id,
            esc_id: uint(read(&mut control, slave_id, DmVariable::ESC_ID)?),
            control_mode: uint(read(&mut control, slave_id, DmVariable::CTRL_MODE)?),
            p_max: float(read(&mut control, slave_id, DmVariable::PMAX)?),
            v_max: float(read(&mut control, slave_id, DmVariable::VMAX)?),
            t_max: float(read(&mut control, slave_id, DmVariable::TMAX)?),
        });
    }
    Ok(found)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_register_reads_are_read_only() {
        let read = CanFrame {
            id: PARAM_ID,
            data: [0x09, 0, 0x33, 7, 0, 0, 0, 0],
        };
        assert!(is_read_only(&read));
        for refused in [
            CanFrame {
                id: PARAM_ID,
                data: [0x09, 0, 0x55, 10, 1, 0, 0, 0],
            },
            CanFrame {
                id: PARAM_ID,
                data: [0x09, 0, 0xAA, 0, 0, 0, 0, 0],
            },
            CanFrame {
                id: 0x09,
                data: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
            },
            CanFrame {
                id: 0x09,
                data: [0x7F, 0xFF, 0x7F, 0xF0, 0, 0, 0x7F, 0xF0],
            },
        ] {
            assert!(!is_read_only(&refused));
        }
    }
}
