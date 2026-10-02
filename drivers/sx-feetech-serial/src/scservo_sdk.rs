//! Feetech's own Python servo SDK, `scservo_sdk`, translated.
//!
//! Reference: <https://github.com/ftservo/FTServo_Python> at
//! `cbcfa64674d592f7e5028ae72af42580f60500b4`, Feetech's official repository, files
//! `scservo_sdk/scservo_def.py`, `scservo_sdk/protocol_packet_handler.py`,
//! `scservo_sdk/group_sync_write.py` and `scservo_sdk/sms_sts.py`. `PyPI`'s
//! `feetech-servo-sdk` 1.0.0, which `LeRobot` 0.6.0 installs, is an earlier cut of the same
//! files; it builds the same bytes.
//!
//! Vendor-derived: Copyright (c) 2024 ftservo, MIT License. The full permission notice is in
//! `LICENSE-THIRD-PARTY` beside this crate's `Cargo.toml`; Sentient-X's changes are Apache-2.0.
//!
//! Same structure, names and arithmetic: [`ProtocolPacketHandler`] is
//! `protocol_packet_handler`, [`GroupSyncWrite`] is `GroupSyncWrite`, [`SmsSts`] is
//! `sms_sts`, [`CommResult`] the `COMM_*` codes, and the constants the vendor's. `txPacket`
//! clears the port's input before every write; `rxPacket` reads a six-byte minimum, finds the
//! `FF FF` header, drops a byte at a time past an invalid id, length or error, re-reads to the
//! length the packet declares, and checks the checksum; `txRxPacket` sends, returns at once
//! for the broadcast id, and otherwise reads until a packet from the addressed id arrives or
//! the read fails. `tests/fixtures/scservo/` pins this file to transcripts recorded by running
//! `scservo_sdk` itself.
//!
//! Where this translation departs from `scservo_sdk`, and why:
//!
//! - The port is `sx_embodiment_drivers::serial::SerialPort`, the shape of the vendor's
//!   `PortHandler` (`port_handler.py`, same commit). Its packet timeout is the link's own, at
//!   most [`SerialPort::timeout`]; the vendor's is the bytes' transmit time plus a fixed 50 ms
//!   `LATENCY_TIMER`. The chain's stop budget needs one constant bound per status packet.
//! - `portHandler.is_using`, the vendor's busy flag, is not kept: a handler owns its port
//!   mutably, so two exchanges cannot overlap and `COMM_PORT_BUSY` cannot happen.
//! - A port I/O error is reported as `COMM_TX_FAIL` (writing) or `COMM_RX_FAIL` (reading);
//!   pyserial raises instead. A failed `clearPort` does not stop the write: stale input can
//!   at worst fail the reply's checks, and a stop's torque-off must go out regardless.
//! - `rxPacket` also drops a header whose LENGTH is below 2, which cannot carry the error byte
//!   every status packet has; the vendor then indexes past the packet.
//! - `rxPacket` checks the deadline once on every pass that does not finish the packet: also
//!   after dropping a byte past an invalid header, after skipping to a header, and after
//!   learning the packet's length, where the vendor reads again without checking. `txRxPacket`
//!   checks it after every packet from another id, where the vendor reads the next one. A line
//!   that keeps delivering foreign or corrupt bytes therefore still times out within the
//!   port's timeout, which the chain's stop budget relies on; the vendor loops while bytes
//!   arrive.
//! - `txPacket` also refuses (`COMM_TX_ERROR`) a LENGTH that claims more bytes than the packet
//!   buffer holds; the vendor indexes past it.
//! - `writeTxRx`, `syncWriteTxOnly` and `GroupSyncWrite::add_param` take their data as a slice
//!   and count its length themselves, where the vendor passes `length` or `param_length`
//!   beside the list and trusts it.
//! - `read1ByteTxRx` and `read2ByteTxRx` report a reply with fewer data bytes than asked as
//!   `COMM_RX_CORRUPT`; the vendor raises `IndexError`.
//! - [`GroupSyncWrite::tx_packet`] takes the handler it sends through as an argument, where
//!   the vendor's `GroupSyncWrite` keeps a reference to it.
//! - Translated only where a consumer drives it: `txPacket`, `rxPacket`, `txRxPacket`,
//!   `readTxRx`, `read1ByteTxRx`, `read2ByteTxRx`, `writeTxRx`, `write1ByteTxRx`,
//!   `write2ByteTxRx`, `syncWriteTxOnly`, the byte helpers, `scs_toscs` and `scs_tohost`,
//!   `GroupSyncWrite`'s parameter handling, and `sms_sts`'s register table, `LockEprom` and
//!   `unLockEprom`. Ping, action, reg-write, sync-read, offset calibration, reset, the
//!   four-byte reads and writes, and `sms_sts`'s position, speed and wheel-mode helpers are
//!   left out.

use sx_embodiment_drivers::serial::SerialPort;

// scservo_def.py
pub const BROADCAST_ID: u8 = 0xFE;
pub const MAX_ID: u8 = 0xFC;
pub const SCS_END: u8 = 0;

pub const INST_PING: u8 = 1;
pub const INST_READ: u8 = 2;
pub const INST_WRITE: u8 = 3;
pub const INST_REG_WRITE: u8 = 4;
pub const INST_ACTION: u8 = 5;
pub const INST_SYNC_WRITE: u8 = 131;
pub const INST_SYNC_READ: u8 = 130;
pub const INST_RESET: u8 = 10;
pub const INST_OFSCAL: u8 = 11;

/// The vendor's communication results (`COMM_*`), by their codes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CommResult {
    /// `COMM_SUCCESS`, 0: the packet went out and, where one was due, its answer came back.
    Success,
    /// `COMM_TX_FAIL`, -2: the packet could not be written whole.
    TxFail,
    /// `COMM_RX_FAIL`, -3: the link failed while reading the answer.
    RxFail,
    /// `COMM_TX_ERROR`, -4: the packet is longer than the protocol carries.
    TxError,
    /// `COMM_RX_TIMEOUT`, -6: no byte of an answer arrived.
    RxTimeout,
    /// `COMM_RX_CORRUPT`, -7: an answer arrived incomplete or failing its checks.
    RxCorrupt,
    /// `COMM_NOT_AVAILABLE`, -9: the request cannot be made, such as reading from the
    /// broadcast id.
    NotAvailable,
}

impl CommResult {
    /// The vendor's numeric code.
    #[must_use]
    pub const fn code(self) -> i8 {
        match self {
            Self::Success => 0,
            Self::TxFail => -2,
            Self::RxFail => -3,
            Self::TxError => -4,
            Self::RxTimeout => -6,
            Self::RxCorrupt => -7,
            Self::NotAvailable => -9,
        }
    }
}

// protocol_packet_handler.py
pub const TXPACKET_MAX_LEN: usize = 250;
pub const RXPACKET_MAX_LEN: usize = 250;

pub const PKT_HEADER0: usize = 0;
pub const PKT_HEADER1: usize = 1;
pub const PKT_ID: usize = 2;
pub const PKT_LENGTH: usize = 3;
pub const PKT_INSTRUCTION: usize = 4;
pub const PKT_ERROR: usize = 4;
pub const PKT_PARAMETER0: usize = 5;

pub const ERRBIT_VOLTAGE: u8 = 1;
pub const ERRBIT_ANGLE: u8 = 2;
pub const ERRBIT_OVERHEAT: u8 = 4;
pub const ERRBIT_OVERELE: u8 = 8;
pub const ERRBIT_OVERLOAD: u8 = 32;

/// The SCS protocol over one port: `protocol_packet_handler`.
pub struct ProtocolPacketHandler<P> {
    port_handler: P,
    scs_end: u8,
}

impl<P> ProtocolPacketHandler<P> {
    /// `protocol_end` is the byte order: 0 for STS and SMS servos, 1 for SCS.
    pub const fn new(port_handler: P, protocol_end: u8) -> Self {
        Self {
            port_handler,
            scs_end: protocol_end,
        }
    }

    #[must_use]
    pub const fn port(&self) -> &P {
        &self.port_handler
    }

    pub const fn port_mut(&mut self) -> &mut P {
        &mut self.port_handler
    }

    #[must_use]
    pub const fn scs_getend(&self) -> u8 {
        self.scs_end
    }

    /// A sign-magnitude register value as a signed number: bit `b` is the sign.
    #[must_use]
    pub fn scs_tohost(a: u16, b: u32) -> i32 {
        if a & (1 << b) != 0 {
            -i32::from(a & !(1 << b))
        } else {
            i32::from(a)
        }
    }

    /// A signed number as a sign-magnitude register value: the magnitude, with bit `b` set
    /// when negative. The vendor masks nothing; [`crate::chain`] refuses a magnitude that
    /// reaches bit `b` before calling this.
    #[must_use]
    pub const fn scs_toscs(a: i32, b: u32) -> i32 {
        if a < 0 { -a | (1 << b) } else { a }
    }

    #[must_use]
    pub fn scs_makeword(&self, a: u8, b: u8) -> u16 {
        if self.scs_end == 0 {
            u16::from(a) | (u16::from(b) << 8)
        } else {
            u16::from(b) | (u16::from(a) << 8)
        }
    }

    // A word's low and high bytes; truncation to a byte is the operation.
    #[allow(clippy::cast_possible_truncation)]
    #[must_use]
    pub const fn scs_lobyte(&self, w: u16) -> u8 {
        if self.scs_end == 0 {
            (w & 0xFF) as u8
        } else {
            ((w >> 8) & 0xFF) as u8
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    #[must_use]
    pub const fn scs_hibyte(&self, w: u16) -> u8 {
        if self.scs_end == 0 {
            ((w >> 8) & 0xFF) as u8
        } else {
            (w & 0xFF) as u8
        }
    }
}

/// `~sum & 0xFF` over `bytes`, the protocol's checksum.
fn checksum(bytes: &[u8]) -> u8 {
    !bytes
        .iter()
        .fold(0_u8, |total, byte| total.wrapping_add(*byte))
}

impl<P: SerialPort> ProtocolPacketHandler<P> {
    /// Complete `txpacket`'s header and checksum and write it.
    pub fn tx_packet(&mut self, txpacket: &mut [u8]) -> CommResult {
        let total_packet_length = usize::from(txpacket[PKT_LENGTH]) + 4;

        // check max packet length
        if total_packet_length > TXPACKET_MAX_LEN || total_packet_length > txpacket.len() {
            return CommResult::TxError;
        }

        // make packet header
        txpacket[PKT_HEADER0] = 0xFF;
        txpacket[PKT_HEADER1] = 0xFF;

        // add a checksum to the packet
        txpacket[total_packet_length - 1] = checksum(&txpacket[2..total_packet_length - 1]);

        // tx packet
        let _ = self.port_handler.clear_port();
        match self
            .port_handler
            .write_port(&txpacket[..total_packet_length])
        {
            Ok(written) if written == total_packet_length => CommResult::Success,
            _ => CommResult::TxFail,
        }
    }

    /// Receive one status packet.
    pub fn rx_packet(&mut self) -> (Vec<u8>, CommResult) {
        let mut rxpacket: Vec<u8> = Vec::new();

        let result;
        let mut wait_length: usize = 6; // minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)

        loop {
            let Ok(bytes) = self
                .port_handler
                .read_port(wait_length.saturating_sub(rxpacket.len()))
            else {
                result = CommResult::RxFail;
                break;
            };
            rxpacket.extend(bytes);
            let rx_length = rxpacket.len();
            if rx_length >= wait_length {
                // find packet header
                let mut idx = 0;
                for index in 0..rx_length - 1 {
                    idx = index;
                    if rxpacket[index] == 0xFF && rxpacket[index + 1] == 0xFF {
                        break;
                    }
                }

                if idx == 0 {
                    // found at the beginning of the packet
                    if rxpacket[PKT_ID] > 0xFD
                        || usize::from(rxpacket[PKT_LENGTH]) > RXPACKET_MAX_LEN
                        || rxpacket[PKT_LENGTH] < 2
                        || rxpacket[PKT_ERROR] > 0x7F
                    {
                        // unavailable ID or unavailable Length or unavailable Error
                        // remove the first byte in the packet
                        rxpacket.remove(0);
                        if self.port_handler.is_packet_timeout() {
                            result = timed_out(rxpacket.len());
                            break;
                        }
                        continue;
                    }

                    // re-calculate the exact length of the rx packet
                    let exact = usize::from(rxpacket[PKT_LENGTH]) + PKT_LENGTH + 1;
                    if wait_length != exact {
                        wait_length = exact;
                        if rx_length < wait_length && self.port_handler.is_packet_timeout() {
                            result = timed_out(rx_length);
                            break;
                        }
                        continue;
                    }

                    if rx_length < wait_length {
                        // check timeout
                        if self.port_handler.is_packet_timeout() {
                            result = timed_out(rx_length);
                            break;
                        }
                        continue;
                    }

                    // verify checksum
                    result = if rxpacket[wait_length - 1] == checksum(&rxpacket[2..wait_length - 1])
                    {
                        CommResult::Success
                    } else {
                        CommResult::RxCorrupt
                    };
                    break;
                }
                // remove unnecessary packets
                rxpacket.drain(0..idx);
                if self.port_handler.is_packet_timeout() {
                    result = timed_out(rxpacket.len());
                    break;
                }
            } else if self.port_handler.is_packet_timeout() {
                // check timeout
                result = timed_out(rx_length);
                break;
            }
        }

        (rxpacket, result)
    }

    /// Send `txpacket` and, unless it is broadcast, receive the addressed servo's answer.
    ///
    /// Returns the answer (`None` when none was due or the send failed), the result, and the
    /// answer's error byte.
    pub fn tx_rx_packet(&mut self, txpacket: &mut [u8]) -> (Option<Vec<u8>>, CommResult, u8) {
        let mut error = 0;

        // tx packet
        let result = self.tx_packet(txpacket);
        if result != CommResult::Success {
            return (None, result, error);
        }

        // (ID == Broadcast ID) == no need to wait for status packet or not available
        if txpacket[PKT_ID] == BROADCAST_ID {
            return (None, result, error);
        }

        // set packet timeout
        if txpacket[PKT_INSTRUCTION] == INST_READ {
            self.port_handler
                .set_packet_timeout(usize::from(txpacket[PKT_PARAMETER0 + 1]) + 6);
        } else {
            // HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM
            self.port_handler.set_packet_timeout(6);
        }

        // rx packet
        let (rxpacket, result) = loop {
            let (rxpacket, result) = self.rx_packet();
            if result != CommResult::Success || txpacket[PKT_ID] == rxpacket[PKT_ID] {
                break (rxpacket, result);
            }
            // A packet from another servo: give up once the wait has run out.
            if self.port_handler.is_packet_timeout() {
                break (rxpacket, CommResult::RxTimeout);
            }
        };

        if result == CommResult::Success && txpacket[PKT_ID] == rxpacket[PKT_ID] {
            error = rxpacket[PKT_ERROR];
        }

        (Some(rxpacket), result, error)
    }

    /// Read `length` bytes from `address`: the data, the result, the servo's error byte.
    pub fn read_tx_rx(&mut self, scs_id: u8, address: u8, length: u8) -> (Vec<u8>, CommResult, u8) {
        let mut txpacket = [0_u8; 8];
        let mut data = Vec::new();

        if scs_id > BROADCAST_ID {
            return (data, CommResult::NotAvailable, 0);
        }

        txpacket[PKT_ID] = scs_id;
        txpacket[PKT_LENGTH] = 4;
        txpacket[PKT_INSTRUCTION] = INST_READ;
        txpacket[PKT_PARAMETER0] = address;
        txpacket[PKT_PARAMETER0 + 1] = length;

        let (rxpacket, result, mut error) = self.tx_rx_packet(&mut txpacket);
        if let (CommResult::Success, Some(rxpacket)) = (result, rxpacket) {
            error = rxpacket[PKT_ERROR];

            let end = rxpacket.len().min(PKT_PARAMETER0 + usize::from(length));
            data.extend_from_slice(&rxpacket[PKT_PARAMETER0.min(end)..end]);
        }

        (data, result, error)
    }

    pub fn read1_byte_tx_rx(&mut self, scs_id: u8, address: u8) -> (u8, CommResult, u8) {
        let (data, result, error) = self.read_tx_rx(scs_id, address, 1);
        match (result, data.as_slice()) {
            (CommResult::Success, [value, ..]) => (*value, result, error),
            (CommResult::Success, _) => (0, CommResult::RxCorrupt, error),
            _ => (0, result, error),
        }
    }

    pub fn read2_byte_tx_rx(&mut self, scs_id: u8, address: u8) -> (u16, CommResult, u8) {
        let (data, result, error) = self.read_tx_rx(scs_id, address, 2);
        match (result, data.as_slice()) {
            (CommResult::Success, [low, high, ..]) => {
                (self.scs_makeword(*low, *high), result, error)
            }
            (CommResult::Success, _) => (0, CommResult::RxCorrupt, error),
            _ => (0, result, error),
        }
    }

    /// Write `data` at `address` and receive the acknowledgement: the result and the servo's
    /// error byte.
    pub fn write_tx_rx(&mut self, scs_id: u8, address: u8, data: &[u8]) -> (CommResult, u8) {
        let Ok(length) = u8::try_from(data.len()) else {
            return (CommResult::TxError, 0);
        };
        let mut txpacket = vec![0_u8; data.len() + 7];

        txpacket[PKT_ID] = scs_id;
        txpacket[PKT_LENGTH] = length.saturating_add(3);
        txpacket[PKT_INSTRUCTION] = INST_WRITE;
        txpacket[PKT_PARAMETER0] = address;

        txpacket[PKT_PARAMETER0 + 1..PKT_PARAMETER0 + 1 + data.len()].copy_from_slice(data);
        let (_, result, error) = self.tx_rx_packet(&mut txpacket);

        (result, error)
    }

    pub fn write1_byte_tx_rx(&mut self, scs_id: u8, address: u8, data: u8) -> (CommResult, u8) {
        self.write_tx_rx(scs_id, address, &[data])
    }

    pub fn write2_byte_tx_rx(&mut self, scs_id: u8, address: u8, data: u16) -> (CommResult, u8) {
        let data_write = [self.scs_lobyte(data), self.scs_hibyte(data)];
        self.write_tx_rx(scs_id, address, &data_write)
    }

    /// One broadcast SYNC WRITE of `param` (id, then `data_length` bytes, per servo).
    pub fn sync_write_tx_only(
        &mut self,
        start_address: u8,
        data_length: u8,
        param: &[u8],
    ) -> CommResult {
        let Ok(param_length) = u8::try_from(param.len()) else {
            return CommResult::TxError;
        };
        // 8: HEADER0 HEADER1 ID LEN INST START_ADDR DATA_LEN ... CHKSUM
        let mut txpacket = vec![0_u8; param.len() + 8];

        txpacket[PKT_ID] = BROADCAST_ID;
        // 4: INST START_ADDR DATA_LEN ... CHKSUM
        txpacket[PKT_LENGTH] = param_length.saturating_add(4);
        txpacket[PKT_INSTRUCTION] = INST_SYNC_WRITE;
        txpacket[PKT_PARAMETER0] = start_address;
        txpacket[PKT_PARAMETER0 + 1] = data_length;

        txpacket[PKT_PARAMETER0 + 2..PKT_PARAMETER0 + 2 + param.len()].copy_from_slice(param);

        let (_, result, _) = self.tx_rx_packet(&mut txpacket);

        result
    }
}

fn timed_out(rx_length: usize) -> CommResult {
    if rx_length == 0 {
        CommResult::RxTimeout
    } else {
        CommResult::RxCorrupt
    }
}

/// `GroupSyncWrite`: one register range written on many servos by one broadcast packet.
pub struct GroupSyncWrite {
    start_address: u8,
    data_length: u8,
    is_param_changed: bool,
    param: Vec<u8>,
    /// In insertion order, as the vendor's `dict` keeps it.
    data_dict: Vec<(u8, Vec<u8>)>,
}

impl GroupSyncWrite {
    #[must_use]
    pub const fn new(start_address: u8, data_length: u8) -> Self {
        Self {
            start_address,
            data_length,
            is_param_changed: false,
            param: Vec::new(),
            data_dict: Vec::new(),
        }
    }

    fn make_param(&mut self) {
        if self.data_dict.is_empty() {
            return;
        }

        self.param = Vec::new();

        for (scs_id, data) in &self.data_dict {
            if data.is_empty() {
                return;
            }

            self.param.push(*scs_id);
            self.param.extend_from_slice(data);
        }
    }

    /// Add one servo's bytes; refused (`false`) for an id already added or data too long.
    pub fn add_param(&mut self, scs_id: u8, data: &[u8]) -> bool {
        if self.data_dict.iter().any(|(id, _)| *id == scs_id) {
            // scs_id already exist
            return false;
        }

        if data.len() > usize::from(self.data_length) {
            // input data is longer than set
            return false;
        }

        self.data_dict.push((scs_id, data.to_vec()));

        self.is_param_changed = true;
        true
    }

    pub fn clear_param(&mut self) {
        self.data_dict.clear();
    }

    /// Send the packet through `ph`.
    pub fn tx_packet<P: SerialPort>(&mut self, ph: &mut ProtocolPacketHandler<P>) -> CommResult {
        if self.data_dict.is_empty() {
            return CommResult::NotAvailable;
        }

        if self.is_param_changed || self.param.is_empty() {
            self.make_param();
        }

        ph.sync_write_tx_only(self.start_address, self.data_length, &self.param)
    }
}

// sms_sts.py: the STS and SMS memory table.
// EPROM, read only
pub const SMS_STS_MODEL_L: u8 = 3;
pub const SMS_STS_MODEL_H: u8 = 4;

// EPROM, read and write
pub const SMS_STS_ID: u8 = 5;
pub const SMS_STS_BAUD_RATE: u8 = 6;
pub const SMS_STS_MIN_ANGLE_LIMIT_L: u8 = 9;
pub const SMS_STS_MIN_ANGLE_LIMIT_H: u8 = 10;
pub const SMS_STS_MAX_ANGLE_LIMIT_L: u8 = 11;
pub const SMS_STS_MAX_ANGLE_LIMIT_H: u8 = 12;
pub const SMS_STS_CW_DEAD: u8 = 26;
pub const SMS_STS_CCW_DEAD: u8 = 27;
pub const SMS_STS_OFS_L: u8 = 31;
pub const SMS_STS_OFS_H: u8 = 32;
pub const SMS_STS_MODE: u8 = 33;

// SRAM, read and write
pub const SMS_STS_TORQUE_ENABLE: u8 = 40;
pub const SMS_STS_ACC: u8 = 41;
pub const SMS_STS_GOAL_POSITION_L: u8 = 42;
pub const SMS_STS_GOAL_POSITION_H: u8 = 43;
pub const SMS_STS_GOAL_TIME_L: u8 = 44;
pub const SMS_STS_GOAL_TIME_H: u8 = 45;
pub const SMS_STS_GOAL_SPEED_L: u8 = 46;
pub const SMS_STS_GOAL_SPEED_H: u8 = 47;
pub const SMS_STS_LOCK: u8 = 55;

// SRAM, read only
pub const SMS_STS_PRESENT_POSITION_L: u8 = 56;
pub const SMS_STS_PRESENT_POSITION_H: u8 = 57;
pub const SMS_STS_PRESENT_SPEED_L: u8 = 58;
pub const SMS_STS_PRESENT_SPEED_H: u8 = 59;
pub const SMS_STS_PRESENT_LOAD_L: u8 = 60;
pub const SMS_STS_PRESENT_LOAD_H: u8 = 61;
pub const SMS_STS_PRESENT_VOLTAGE: u8 = 62;
pub const SMS_STS_PRESENT_TEMPERATURE: u8 = 63;
pub const SMS_STS_MOVING: u8 = 66;
pub const SMS_STS_PRESENT_CURRENT_L: u8 = 69;
pub const SMS_STS_PRESENT_CURRENT_H: u8 = 70;

/// `sms_sts`: the packet handler for STS and SMS servos (byte order 0).
pub struct SmsSts<P> {
    pub ph: ProtocolPacketHandler<P>,
}

impl<P: SerialPort> SmsSts<P> {
    pub const fn new(port_handler: P) -> Self {
        Self {
            ph: ProtocolPacketHandler::new(port_handler, 0),
        }
    }

    pub fn lock_eprom(&mut self, scs_id: u8) -> (CommResult, u8) {
        self.ph.write1_byte_tx_rx(scs_id, SMS_STS_LOCK, 1)
    }

    pub fn un_lock_eprom(&mut self, scs_id: u8) -> (CommResult, u8) {
        self.ph.write1_byte_tx_rx(scs_id, SMS_STS_LOCK, 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sign_magnitude_is_the_vendors() {
        assert_eq!(ProtocolPacketHandler::<()>::scs_toscs(-5, 15), 0x8005);
        assert_eq!(ProtocolPacketHandler::<()>::scs_toscs(5, 15), 5);
        assert_eq!(ProtocolPacketHandler::<()>::scs_tohost(0x0805, 11), -5);
        assert_eq!(ProtocolPacketHandler::<()>::scs_tohost(0x0405, 11), 0x0405);
    }

    #[test]
    fn the_byte_order_follows_protocol_end() {
        let sts = ProtocolPacketHandler::new((), 0);
        assert_eq!(
            (sts.scs_lobyte(0x1234), sts.scs_hibyte(0x1234)),
            (0x34, 0x12)
        );
        assert_eq!(sts.scs_makeword(0x34, 0x12), 0x1234);
        let scs = ProtocolPacketHandler::new((), 1);
        assert_eq!(
            (scs.scs_lobyte(0x1234), scs.scs_hibyte(0x1234)),
            (0x12, 0x34)
        );
        assert_eq!(scs.scs_makeword(0x12, 0x34), 0x1234);
    }
}
