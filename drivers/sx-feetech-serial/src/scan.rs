//! What answers on a Feetech bus, read without writing a register.
//!
//! [`scan`] pings each id the caller names and reads, from every servo that answers, what a
//! robot is commissioned by: its model number and firmware, the calibration `LeRobot` writes
//! (position limits and homing offset, as a [`MotorCalibration`]), its operating mode, position
//! loop and torque limit, and its live state (torque enable, position, voltage, temperature).
//! The addresses are `LeRobot`'s `STS_SMS_SERIES_CONTROL_TABLE`
//! (`src/lerobot/motors/feetech/tables.py:41-100` at `v0.6.0`), the same table
//! [`crate::chain`] reads at open.
//!
//! The scan cannot write. It speaks through [`ReadOnlyPort`], which refuses at the byte level
//! any packet whose instruction is not `PING` or `READ`, or that is addressed to the broadcast
//! id, before it reaches the link; the refusal is the vendor's `COMM_TX_FAIL`, so no write,
//! torque enable or EEPROM unlock can leave the host however the protocol layer above is
//! driven. The packets themselves are built by [`crate::scservo_sdk`]; `PING` is the vendor's
//! `ping` (`protocol_packet_handler.py`), a two-byte-length packet through `txRxPacket`.

use std::io;
use std::time::Duration;

use sx_embodiment_drivers::serial::SerialPort;

use crate::chain::MotorCalibration;
use crate::scservo_sdk::{
    BROADCAST_ID, CommResult, INST_PING, INST_READ, MAX_ID, PKT_ID, PKT_INSTRUCTION, PKT_LENGTH,
    ProtocolPacketHandler, SMS_STS_MAX_ANGLE_LIMIT_L, SMS_STS_MIN_ANGLE_LIMIT_L, SMS_STS_MODE,
    SMS_STS_MODEL_L, SMS_STS_OFS_L, SMS_STS_PRESENT_POSITION_L, SMS_STS_PRESENT_TEMPERATURE,
    SMS_STS_PRESENT_VOLTAGE, SMS_STS_TORQUE_ENABLE, SmsSts,
};

/// The rate `LeRobot` opens an STS3215 bus at (`FeetechMotorsBus.default_baudrate`).
pub const STS3215_BAUD: u32 = 1_000_000;

// `LeRobot`'s control table, where `sms_sts.py` does not name the address.
const FIRMWARE_MAJOR_VERSION: u8 = 0;
const FIRMWARE_MINOR_VERSION: u8 = 1;
const MAX_TORQUE_LIMIT: u8 = 16;
const P_COEFFICIENT: u8 = 21;
const D_COEFFICIENT: u8 = 22;
const I_COEFFICIENT: u8 = 23;
/// `Homing_Offset`'s sign bit (`STS_SMS_SERIES_ENCODINGS_TABLE`).
const HOMING_OFFSET_SIGN_BIT: u32 = 11;

/// Whether `packet` only asks: a `PING` or a `READ` addressed to one servo.
#[must_use]
pub fn is_read_only(packet: &[u8]) -> bool {
    packet.len() > PKT_INSTRUCTION
        && packet.len() == usize::from(packet[PKT_LENGTH]) + 4
        && packet[..2] == [0xFF, 0xFF]
        && packet[PKT_ID] <= MAX_ID
        && packet[PKT_ID] != BROADCAST_ID
        && match packet[PKT_INSTRUCTION] {
            INST_PING => packet[PKT_LENGTH] == 2,
            INST_READ => packet[PKT_LENGTH] == 4,
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

    pub fn into_inner(self) -> P {
        self.0
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
                "a scan sends only PING and READ",
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

/// One servo that answered, as its registers read.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ServoScan {
    pub id: u8,
    /// `Model_Number`; the STS3215 is [`crate::STS3215_MODEL_NUMBER`].
    pub model: u16,
    pub firmware_major: u8,
    pub firmware_minor: u8,
    /// The calibration its registers hold. The STS has no drive-mode register; `LeRobot`'s
    /// `read_calibration` records 0, and so does this.
    pub calibration: MotorCalibration,
    /// `Operating_Mode`: 0 position, 1 velocity, 2 PWM, 3 step.
    pub operating_mode: u8,
    pub p_coefficient: u8,
    pub d_coefficient: u8,
    pub i_coefficient: u8,
    pub max_torque_limit: u16,
    pub torque_enabled: bool,
    /// `Present_Position`, in encoder ticks.
    pub present_position: u16,
    /// `Present_Voltage`, in tenths of a volt.
    pub voltage_decivolts: u8,
    /// `Present_Temperature`, degrees Celsius.
    pub temperature_celsius: u8,
    /// The status packet's error flags at the ping (`ERRBIT_*`); 0 when healthy.
    pub status: u8,
}

/// One id that answered its ping but not a later read.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ReadFailure {
    pub id: u8,
    pub address: u8,
    pub result: CommResult,
}

/// What a bus answered.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct BusScan {
    pub servos: Vec<ServoScan>,
    pub failed: Vec<ReadFailure>,
    /// Ids that did not answer their ping.
    pub silent: Vec<u8>,
}

impl BusScan {
    /// The servo answering as `id`.
    #[must_use]
    pub fn servo(&self, id: u8) -> Option<&ServoScan> {
        self.servos.iter().find(|servo| servo.id == id)
    }
}

/// `ping` from `protocol_packet_handler.py`: the result and the status packet's error byte.
fn ping<P: SerialPort>(ph: &mut ProtocolPacketHandler<P>, scs_id: u8) -> (CommResult, u8) {
    let mut txpacket = [0_u8; 6];
    txpacket[PKT_ID] = scs_id;
    txpacket[PKT_LENGTH] = 2;
    txpacket[PKT_INSTRUCTION] = INST_PING;
    let (_, result, error) = ph.tx_rx_packet(&mut txpacket);
    (result, error)
}

struct Reader<'a, P> {
    ph: &'a mut ProtocolPacketHandler<ReadOnlyPort<P>>,
    id: u8,
}

impl<P: SerialPort> Reader<'_, P> {
    fn one(&mut self, address: u8) -> Result<u8, ReadFailure> {
        let (value, result, _) = self.ph.read1_byte_tx_rx(self.id, address);
        self.check(address, result).map(|()| value)
    }

    fn two(&mut self, address: u8) -> Result<u16, ReadFailure> {
        let (value, result, _) = self.ph.read2_byte_tx_rx(self.id, address);
        self.check(address, result).map(|()| value)
    }

    fn check(&self, address: u8, result: CommResult) -> Result<(), ReadFailure> {
        if result == CommResult::Success {
            Ok(())
        } else {
            Err(ReadFailure {
                id: self.id,
                address,
                result,
            })
        }
    }
}

fn read_servo<P: SerialPort>(
    ph: &mut ProtocolPacketHandler<ReadOnlyPort<P>>,
    id: u8,
    status: u8,
) -> Result<ServoScan, ReadFailure> {
    let mut read = Reader { ph, id };
    let model = read.two(SMS_STS_MODEL_L)?;
    let firmware_major = read.one(FIRMWARE_MAJOR_VERSION)?;
    let firmware_minor = read.one(FIRMWARE_MINOR_VERSION)?;
    let range_min = read.two(SMS_STS_MIN_ANGLE_LIMIT_L)?;
    let range_max = read.two(SMS_STS_MAX_ANGLE_LIMIT_L)?;
    let offset = read.two(SMS_STS_OFS_L)?;
    let operating_mode = read.one(SMS_STS_MODE)?;
    let p_coefficient = read.one(P_COEFFICIENT)?;
    let d_coefficient = read.one(D_COEFFICIENT)?;
    let i_coefficient = read.one(I_COEFFICIENT)?;
    let max_torque_limit = read.two(MAX_TORQUE_LIMIT)?;
    let torque_enable = read.one(SMS_STS_TORQUE_ENABLE)?;
    let present_position = read.two(SMS_STS_PRESENT_POSITION_L)?;
    let voltage_decivolts = read.one(SMS_STS_PRESENT_VOLTAGE)?;
    let temperature_celsius = read.one(SMS_STS_PRESENT_TEMPERATURE)?;
    // Sign-magnitude below bit 11, so the magnitude fits an i16.
    let homing_offset = i16::try_from(ProtocolPacketHandler::<()>::scs_tohost(
        offset & 0x0FFF,
        HOMING_OFFSET_SIGN_BIT,
    ))
    .unwrap_or_default();
    Ok(ServoScan {
        id,
        model,
        firmware_major,
        firmware_minor,
        calibration: MotorCalibration {
            drive_mode: false,
            homing_offset,
            range_min,
            range_max,
        },
        operating_mode,
        p_coefficient,
        d_coefficient,
        i_coefficient,
        max_torque_limit,
        torque_enabled: torque_enable != 0,
        present_position,
        voltage_decivolts,
        temperature_celsius,
        status,
    })
}

/// Ping each of `ids` on `port` and read every servo that answers. The port is the caller's,
/// opened at the bus's rate ([`STS3215_BAUD`]).
pub fn scan<P: SerialPort>(port: P, ids: impl IntoIterator<Item = u8>) -> BusScan {
    let mut sms = SmsSts::new(ReadOnlyPort::new(port));
    let mut found = BusScan::default();
    for id in ids {
        if id > MAX_ID {
            found.silent.push(id);
            continue;
        }
        let (result, status) = ping(&mut sms.ph, id);
        if result != CommResult::Success {
            found.silent.push(id);
            continue;
        }
        match read_servo(&mut sms.ph, id, status) {
            Ok(servo) => found.servos.push(servo),
            Err(failure) => found.failed.push(failure),
        }
    }
    found
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_addressed_pings_and_reads_are_read_only() {
        assert!(is_read_only(&[0xFF, 0xFF, 1, 2, INST_PING, 0xFB]));
        assert!(is_read_only(&[0xFF, 0xFF, 1, 4, INST_READ, 56, 2, 0xC0]));
        assert!(!is_read_only(&[0xFF, 0xFF, BROADCAST_ID, 2, INST_PING, 0]));
        assert!(!is_read_only(&[0xFF, 0xFF, 1, 4, 3, 40, 1, 0]));
        assert!(!is_read_only(&[0xFF, 0xFF, 1, 4, INST_READ, 56, 2]));
    }
}
