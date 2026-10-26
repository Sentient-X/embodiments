//! ROBOTIS's own `DynamixelSDK`, its Protocol 2.0 packet handler, translated.
//!
//! Reference: <https://github.com/ROBOTIS-GIT/DynamixelSDK> at
//! `a8d35d51c87e7d253ece1baa963f498a8d933fa0` (`python/setup.py` version 4.1.0), ROBOTIS's
//! official repository, files `python/src/dynamixel_sdk/robotis_def.py`,
//! `protocol2_packet_handler.py`, `group_sync_write.py` and `group_sync_read.py`. Byte
//! stuffing follows the same commit's `c++/src/dynamixel_sdk/protocol2_packet_handler.cpp`
//! (`addStuffing`), as the next list says. The packet layout, error byte and stuffing rule are
//! the e-manual's (<https://github.com/ROBOTIS-GIT/emanual> at
//! `0807b1b5f7472bbbbb2c9751f1d3406678e86d37`, `docs/en/dxl/protocol2.md`), whose worked
//! examples this module's tests reproduce byte for byte.
//!
//! Vendor-derived: Copyright 2017 ROBOTIS CO., LTD., Apache License 2.0. The notice is in
//! `LICENSE-THIRD-PARTY` beside this crate's `Cargo.toml`; Sentient-X's changes are Apache-2.0.
//!
//! Same structure, names and arithmetic: [`Protocol2PacketHandler`] is
//! `Protocol2PacketHandler`, [`GroupSyncWrite`] and [`GroupSyncRead`] are theirs, [`CommResult`]
//! the `COMM_*` codes, `update_crc` the vendor's CRC-16 (polynomial 0x8005, its own table), and
//! the constants the vendor's. `txPacket` stuffs, completes the `FF FF FD 00` header and CRC,
//! clears the port's input and writes; `rxPacket` reads an eleven-byte minimum, finds a header
//! not followed by a stuffing `FD`, drops a byte at a time past a reserved byte, id, length or
//! instruction that cannot be a status packet, re-reads to the length the packet declares,
//! checks the CRC and removes the stuffing; `txRxPacket` sends, returns at once for the
//! broadcast id, and otherwise reads until a packet from the addressed id arrives or the read
//! fails. `tests/fixtures/dynamixel_sdk/` pins this file to a transcript recorded by running
//! the Python SDK itself.
//!
//! Where this translation departs from the Python SDK, and why:
//!
//! - **Byte stuffing is applied.** The Python `txPacket` calls `addStuffing(txpacket)` and
//!   discards its result; when a packet needs stuffing, `addStuffing` builds a new list, so the
//!   caller's packet goes out unstuffed with its original LENGTH, which a servo can take for a
//!   header mid-packet. Here every `FF FF FD` in the instruction and parameters is followed by
//!   an extra `FD` and LENGTH counts it, as the C++ SDK's `addStuffing` and the e-manual's
//!   "Packet Process" do. The C++ SDK skips a LENGTH below 8; such a packet has too few
//!   parameter bytes to carry the pattern, so the rule here is the e-manual's, unconditional.
//!   Packets without the pattern, which every packet this crate's chain sends is, are the
//!   Python SDK's byte for byte.
//! - The port is `sx_embodiment_drivers::serial::SerialPort`, the shape of the vendor's
//!   `PortHandler` (`port_handler.py`, same commit). Its packet timeout is the link's own, at
//!   most [`SerialPort::timeout`]; the vendor's is the bytes' transmit time plus twice a 16 ms
//!   `LATENCY_TIMER` plus 2 ms. The chain's stop budget needs one constant bound per status
//!   packet.
//! - `port.is_using`, the vendor's busy flag, is not kept: a handler owns its port mutably, so
//!   two exchanges cannot overlap and `COMM_PORT_BUSY` cannot happen.
//! - A port I/O error is reported as `COMM_TX_FAIL` (writing) or `COMM_RX_FAIL` (reading);
//!   pyserial raises instead. A failed `clearPort` does not stop the write: stale input can at
//!   worst fail the reply's checks, and a stop's torque-off must go out regardless.
//! - `rxPacket` also drops a header whose LENGTH is below 4, which cannot carry the
//!   instruction, error and CRC every status packet has; the vendor then indexes past it.
//! - `rxPacket` checks the deadline once on every pass that does not finish the packet: also
//!   after dropping a byte past an invalid header, after skipping to a header, and after
//!   learning the packet's length, where the vendor reads again without checking.
//!   `txRxPacket` and `readRx` check it after every packet from another id, where the vendor
//!   reads the next one. A line that keeps delivering foreign or corrupt bytes therefore still
//!   times out within the port's timeout, which the chain's stop budget relies on; the vendor
//!   loops while bytes arrive.
//! - `rxPacket` takes no `fast_option`: the fast sync and bulk reads are not translated.
//! - `writeTxRx`, `syncWriteTxOnly`, `syncReadTx` and `GroupSyncWrite::add_param` take their
//!   data as a slice and count its length themselves, where the vendor passes `length` or
//!   `param_length` beside the list and trusts it.
//! - `read1ByteTxRx`, `read2ByteTxRx`, `read4ByteTxRx` and `GroupSyncRead::get_data` report a
//!   reply with fewer data bytes than asked as `COMM_RX_CORRUPT` (or `None`); the vendor raises
//!   `IndexError`.
//! - [`GroupSyncWrite::tx_packet`] and [`GroupSyncRead::tx_rx_packet`] take the handler they
//!   send through as an argument, where the vendor's groups keep a reference to it.
//! - Translated only where a consumer drives it, or the task the crate was written for named
//!   it: `addStuffing`, `removeStuffing`, `updateCRC`, `txPacket`, `rxPacket`, `txRxPacket`,
//!   `ping`, `readTxRx`, `readRx`, the one-, two- and four-byte `TxRx` reads and writes,
//!   `syncReadTx`, `syncWriteTxOnly`, the byte helpers, and `GroupSyncWrite`'s and
//!   `GroupSyncRead`'s parameter handling. Broadcast ping, action, reboot, clear, factory reset,
//!   reg-write, the `TxOnly` writes, the fast and bulk reads and writes, and Protocol 1.0 are
//!   left out.

use sx_embodiment_drivers::serial::SerialPort;

// robotis_def.py
pub const BROADCAST_ID: u8 = 0xFE;
pub const MAX_ID: u8 = 0xFC;

pub const INST_PING: u8 = 1;
pub const INST_READ: u8 = 2;
pub const INST_WRITE: u8 = 3;
pub const INST_STATUS: u8 = 85;
pub const INST_SYNC_READ: u8 = 130;
pub const INST_SYNC_WRITE: u8 = 131;
pub const INST_BULK_READ: u8 = 146;

/// The vendor's communication results (`COMM_*`), by their codes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CommResult {
    /// `COMM_SUCCESS`, 0: the packet went out and, where one was due, its answer came back.
    Success,
    /// `COMM_TX_FAIL`, -1001: the packet could not be written whole.
    TxFail,
    /// `COMM_RX_FAIL`, -1002: the link failed while reading the answer.
    RxFail,
    /// `COMM_TX_ERROR`, -2000: the packet is longer than the protocol carries.
    TxError,
    /// `COMM_RX_TIMEOUT`, -3001: no byte of an answer arrived.
    RxTimeout,
    /// `COMM_RX_CORRUPT`, -3002: an answer arrived incomplete or failing its checks.
    RxCorrupt,
    /// `COMM_NOT_AVAILABLE`, -9000: the request cannot be made, such as reading from the
    /// broadcast id.
    NotAvailable,
}

impl CommResult {
    /// The vendor's numeric code.
    #[must_use]
    pub const fn code(self) -> i16 {
        match self {
            Self::Success => 0,
            Self::TxFail => -1001,
            Self::RxFail => -1002,
            Self::TxError => -2000,
            Self::RxTimeout => -3001,
            Self::RxCorrupt => -3002,
            Self::NotAvailable => -9000,
        }
    }
}

// robotis_def.py: the byte helpers. Truncation to a byte or word is the operation.
#[must_use]
pub const fn dxl_makeword(a: u8, b: u8) -> u16 {
    (a as u16) | ((b as u16) << 8)
}

#[must_use]
pub const fn dxl_makedword(a: u16, b: u16) -> u32 {
    (a as u32) | ((b as u32) << 16)
}

#[allow(clippy::cast_possible_truncation)]
#[must_use]
pub const fn dxl_loword(l: u32) -> u16 {
    (l & 0xFFFF) as u16
}

#[allow(clippy::cast_possible_truncation)]
#[must_use]
pub const fn dxl_hiword(l: u32) -> u16 {
    ((l >> 16) & 0xFFFF) as u16
}

#[allow(clippy::cast_possible_truncation)]
#[must_use]
pub const fn dxl_lobyte(w: u16) -> u8 {
    (w & 0xFF) as u8
}

#[allow(clippy::cast_possible_truncation)]
#[must_use]
pub const fn dxl_hibyte(w: u16) -> u8 {
    ((w >> 8) & 0xFF) as u8
}

// protocol2_packet_handler.py
pub const TXPACKET_MAX_LEN: usize = 1024;
pub const RXPACKET_MAX_LEN: usize = 1024;

pub const PKT_HEADER0: usize = 0;
pub const PKT_HEADER1: usize = 1;
pub const PKT_HEADER2: usize = 2;
pub const PKT_RESERVED: usize = 3;
pub const PKT_ID: usize = 4;
pub const PKT_LENGTH_L: usize = 5;
pub const PKT_LENGTH_H: usize = 6;
pub const PKT_INSTRUCTION: usize = 7;
pub const PKT_ERROR: usize = 8;
pub const PKT_PARAMETER0: usize = 8;

// Protocol 2.0 error numbers, the error byte's low seven bits.
pub const ERRNUM_RESULT_FAIL: u8 = 1;
pub const ERRNUM_INSTRUCTION: u8 = 2;
pub const ERRNUM_CRC: u8 = 3;
pub const ERRNUM_DATA_RANGE: u8 = 4;
pub const ERRNUM_DATA_LENGTH: u8 = 5;
pub const ERRNUM_DATA_LIMIT: u8 = 6;
pub const ERRNUM_ACCESS: u8 = 7;

/// The error byte's alert bit: the device has a hardware error; its `Hardware Error Status`
/// register says which.
pub const ERRBIT_ALERT: u8 = 128;

/// The vendor's CRC-16 table (`updateCRC`'s `crc_table`), polynomial 0x8005.
const CRC_TABLE: [u16; 256] = [
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033, 0x0036, 0x003C, 0x8039,
    0x0028, 0x802D, 0x8027, 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
    0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099,
    0x0088, 0x808D, 0x8087, 0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA,
    0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179,
    0x0168, 0x816D, 0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309,
    0x0318, 0x831D, 0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C, 0x8359,
    0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9,
    0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
    0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243, 0x0246, 0x024C, 0x8249,
    0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
    0x0208, 0x820D, 0x8207, 0x0202,
];

/// `updateCRC`: the CRC-16 of `data_blk` continuing from `crc_accum`.
#[must_use]
pub fn update_crc(crc_accum: u16, data_blk: &[u8]) -> u16 {
    data_blk.iter().fold(crc_accum, |crc_accum, byte| {
        let i = usize::from(dxl_hibyte(crc_accum) ^ byte);
        (crc_accum << 8) ^ CRC_TABLE[i]
    })
}

/// `addStuffing` as the C++ SDK and the e-manual apply it: an `FD` after every `FF FF FD` in
/// the instruction and parameters, LENGTH counting each. `packet` is a whole packet with its
/// LENGTH set and two trailing bytes for the CRC.
#[must_use]
pub fn add_stuffing(packet: &[u8]) -> Vec<u8> {
    let packet_length_in = usize::from(dxl_makeword(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]));
    let end = (PKT_INSTRUCTION + packet_length_in).min(packet.len());
    let body_end = end.saturating_sub(2).max(PKT_INSTRUCTION);
    let mut stuffed = packet[..PKT_INSTRUCTION].to_vec();
    for &byte in &packet[PKT_INSTRUCTION..body_end] {
        stuffed.push(byte);
        if stuffed.len() >= PKT_INSTRUCTION + 3 && stuffed.ends_with(&[0xFF, 0xFF, 0xFD]) {
            // FF FF FD
            stuffed.push(0xFD);
        }
    }
    stuffed.extend_from_slice(&packet[body_end..end]);
    let packet_length_out = packet_length_in + (stuffed.len() - end);
    let packet_length_out = u16::try_from(packet_length_out).unwrap_or(u16::MAX);
    stuffed[PKT_LENGTH_L] = dxl_lobyte(packet_length_out);
    stuffed[PKT_LENGTH_H] = dxl_hibyte(packet_length_out);
    stuffed
}

/// `removeStuffing`: drop the `FD` the sender added after each `FF FF FD`.
pub fn remove_stuffing(packet: &mut Vec<u8>) {
    let packet_length_in = usize::from(dxl_makeword(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]));
    let mut packet_length_out = packet_length_in;

    let mut index = PKT_INSTRUCTION;
    for i in 0..packet_length_in - 2 {
        // except CRC
        if packet[i + PKT_INSTRUCTION] == 0xFD
            && packet[i + PKT_INSTRUCTION + 1] == 0xFD
            && packet[i + PKT_INSTRUCTION - 1] == 0xFF
            && packet[i + PKT_INSTRUCTION - 2] == 0xFF
        {
            // FF FF FD FD
            packet_length_out -= 1;
        } else {
            packet[index] = packet[i + PKT_INSTRUCTION];
            index += 1;
        }
    }

    packet[index] = packet[PKT_INSTRUCTION + packet_length_in - 2];
    packet[index + 1] = packet[PKT_INSTRUCTION + packet_length_in - 1];
    packet.truncate(index + 2);

    let packet_length_out = u16::try_from(packet_length_out).unwrap_or(u16::MAX);
    packet[PKT_LENGTH_L] = dxl_lobyte(packet_length_out);
    packet[PKT_LENGTH_H] = dxl_hibyte(packet_length_out);
}

/// Protocol 2.0 over one port: `Protocol2PacketHandler`, with the port it talks through.
pub struct Protocol2PacketHandler<P> {
    port_handler: P,
}

impl<P> Protocol2PacketHandler<P> {
    pub const fn new(port_handler: P) -> Self {
        Self { port_handler }
    }

    #[must_use]
    pub const fn port(&self) -> &P {
        &self.port_handler
    }

    pub const fn port_mut(&mut self) -> &mut P {
        &mut self.port_handler
    }
}

fn timed_out(rx_length: usize) -> CommResult {
    if rx_length == 0 {
        CommResult::RxTimeout
    } else {
        CommResult::RxCorrupt
    }
}

impl<P: SerialPort> Protocol2PacketHandler<P> {
    /// Stuff `txpacket`, complete its header and CRC, and write it.
    pub fn tx_packet(&mut self, txpacket: &[u8]) -> CommResult {
        // byte stuffing for header
        let mut txpacket = add_stuffing(txpacket);

        // check max packet length
        let total_packet_length =
            usize::from(dxl_makeword(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H])) + 7;
        // 7: HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H

        if total_packet_length > TXPACKET_MAX_LEN || total_packet_length > txpacket.len() {
            return CommResult::TxError;
        }

        // make packet header
        txpacket[PKT_HEADER0] = 0xFF;
        txpacket[PKT_HEADER1] = 0xFF;
        txpacket[PKT_HEADER2] = 0xFD;
        txpacket[PKT_RESERVED] = 0x00;

        // add CRC16
        let crc = update_crc(0, &txpacket[..total_packet_length - 2]); // 2: CRC16

        txpacket[total_packet_length - 2] = dxl_lobyte(crc);
        txpacket[total_packet_length - 1] = dxl_hibyte(crc);

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

    /// Receive one status packet, its stuffing removed.
    pub fn rx_packet(&mut self) -> (Vec<u8>, CommResult) {
        let mut rxpacket: Vec<u8> = Vec::new();
        let packet_id = MAX_ID;

        let mut result;
        // minimum length (HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR
        // CRC16_L CRC16_H)
        let mut wait_length: usize = 11;

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
                for index in 0..rx_length - 3 {
                    idx = index;
                    if rxpacket[index] == 0xFF
                        && rxpacket[index + 1] == 0xFF
                        && rxpacket[index + 2] == 0xFD
                        && rxpacket[index + 3] != 0xFD
                    {
                        break;
                    }
                }

                if idx == 0 {
                    let length =
                        usize::from(dxl_makeword(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]));
                    if rxpacket[PKT_RESERVED] != 0x00
                        || rxpacket[PKT_ID] > packet_id
                        || !(4..=RXPACKET_MAX_LEN).contains(&length)
                        || rxpacket[PKT_INSTRUCTION] != INST_STATUS
                    {
                        // remove the first byte in the packet
                        rxpacket.remove(0);
                        if self.port_handler.is_packet_timeout() {
                            result = timed_out(rxpacket.len());
                            break;
                        }
                        continue;
                    }

                    let exact = length + PKT_LENGTH_H + 1;
                    if wait_length != exact {
                        wait_length = exact;
                        if rx_length < wait_length && self.port_handler.is_packet_timeout() {
                            result = timed_out(rx_length);
                            break;
                        }
                        continue;
                    }

                    if rx_length < wait_length {
                        if self.port_handler.is_packet_timeout() {
                            result = timed_out(rx_length);
                            break;
                        }
                        continue;
                    }

                    let crc = dxl_makeword(rxpacket[wait_length - 2], rxpacket[wait_length - 1]);

                    result = if update_crc(0, &rxpacket[..wait_length - 2]) == crc {
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
                result = timed_out(rx_length);
                break;
            }
        }

        if result == CommResult::Success {
            remove_stuffing(&mut rxpacket);
            // A packet whose stuffing claims more than it carries fails its own checks.
            if rxpacket.len() < 11 {
                result = CommResult::RxCorrupt;
            }
        }

        (rxpacket, result)
    }

    /// Send `txpacket` and, unless it is broadcast, receive the addressed device's answer.
    ///
    /// Returns the answer (`None` when none was due or the send failed), the result, and the
    /// answer's error byte.
    pub fn tx_rx_packet(&mut self, txpacket: &[u8]) -> (Option<Vec<u8>>, CommResult, u8) {
        let mut error = 0;

        // tx packet
        let mut result = self.tx_packet(txpacket);
        if result != CommResult::Success {
            return (None, result, error);
        }

        // (Instruction == BulkRead or SyncRead) == this function is not available.
        if txpacket[PKT_INSTRUCTION] == INST_BULK_READ
            || txpacket[PKT_INSTRUCTION] == INST_SYNC_READ
        {
            result = CommResult::NotAvailable;
        }

        // (ID == Broadcast ID) == no need to wait for status packet or not available.
        if txpacket[PKT_ID] == BROADCAST_ID {
            return (None, result, error);
        }

        // set packet timeout
        if txpacket[PKT_INSTRUCTION] == INST_READ {
            self.port_handler.set_packet_timeout(
                usize::from(dxl_makeword(
                    txpacket[PKT_PARAMETER0 + 2],
                    txpacket[PKT_PARAMETER0 + 3],
                )) + 11,
            );
        } else {
            // HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H
            self.port_handler.set_packet_timeout(11);
        }

        // rx packet
        let (rxpacket, result) = loop {
            let (rxpacket, result) = self.rx_packet();
            if result != CommResult::Success || txpacket[PKT_ID] == rxpacket[PKT_ID] {
                break (rxpacket, result);
            }
            // A packet from another device: give up once the wait has run out.
            if self.port_handler.is_packet_timeout() {
                break (rxpacket, CommResult::RxTimeout);
            }
        };

        if result == CommResult::Success && txpacket[PKT_ID] == rxpacket[PKT_ID] {
            error = rxpacket[PKT_ERROR];
        }

        (Some(rxpacket), result, error)
    }

    /// `ping`: the device's model number, the result, and its error byte.
    pub fn ping(&mut self, dxl_id: u8) -> (u16, CommResult, u8) {
        let mut model_number = 0;

        let mut txpacket = [0_u8; 10];

        if dxl_id >= BROADCAST_ID {
            return (model_number, CommResult::NotAvailable, 0);
        }

        txpacket[PKT_ID] = dxl_id;
        txpacket[PKT_LENGTH_L] = 3;
        txpacket[PKT_LENGTH_H] = 0;
        txpacket[PKT_INSTRUCTION] = INST_PING;

        let (rxpacket, mut result, error) = self.tx_rx_packet(&txpacket);
        if result == CommResult::Success {
            match rxpacket.as_deref() {
                Some(rxpacket) if rxpacket.len() > PKT_PARAMETER0 + 2 => {
                    model_number =
                        dxl_makeword(rxpacket[PKT_PARAMETER0 + 1], rxpacket[PKT_PARAMETER0 + 2]);
                }
                _ => result = CommResult::RxCorrupt,
            }
        }

        (model_number, result, error)
    }

    /// The data bytes of a status packet that answered a read of `length` bytes.
    fn data(rxpacket: &[u8], length: u16) -> Option<Vec<u8>> {
        let start = PKT_PARAMETER0 + 1;
        let end = start + usize::from(length);
        // The data stop before the CRC.
        (rxpacket.len() >= end + 2).then(|| rxpacket[start..end].to_vec())
    }

    /// `readRx`: receive the status packet `dxl_id` answers a read or sync read with.
    pub fn read_rx(&mut self, dxl_id: u8, length: u16) -> (Vec<u8>, CommResult, u8) {
        let mut error = 0;
        let mut data = Vec::new();

        let (rxpacket, mut result) = loop {
            let (rxpacket, result) = self.rx_packet();
            if result != CommResult::Success || rxpacket[PKT_ID] == dxl_id {
                break (rxpacket, result);
            }
            if self.port_handler.is_packet_timeout() {
                break (rxpacket, CommResult::RxTimeout);
            }
        };

        if result == CommResult::Success && rxpacket[PKT_ID] == dxl_id {
            error = rxpacket[PKT_ERROR];
            match Self::data(&rxpacket, length) {
                Some(bytes) => data = bytes,
                None => result = CommResult::RxCorrupt,
            }
        }

        (data, result, error)
    }

    /// `readTxRx`: read `length` bytes from `address`: the data, the result, the error byte.
    pub fn read_tx_rx(
        &mut self,
        dxl_id: u8,
        address: u16,
        length: u16,
    ) -> (Vec<u8>, CommResult, u8) {
        let mut txpacket = [0_u8; 14];
        let mut data = Vec::new();

        if dxl_id >= BROADCAST_ID {
            return (data, CommResult::NotAvailable, 0);
        }

        txpacket[PKT_ID] = dxl_id;
        txpacket[PKT_LENGTH_L] = 7;
        txpacket[PKT_LENGTH_H] = 0;
        txpacket[PKT_INSTRUCTION] = INST_READ;
        txpacket[PKT_PARAMETER0] = dxl_lobyte(address);
        txpacket[PKT_PARAMETER0 + 1] = dxl_hibyte(address);
        txpacket[PKT_PARAMETER0 + 2] = dxl_lobyte(length);
        txpacket[PKT_PARAMETER0 + 3] = dxl_hibyte(length);

        let (rxpacket, mut result, mut error) = self.tx_rx_packet(&txpacket);
        if let (CommResult::Success, Some(rxpacket)) = (result, rxpacket) {
            error = rxpacket[PKT_ERROR];
            match Self::data(&rxpacket, length) {
                Some(bytes) => data = bytes,
                None => result = CommResult::RxCorrupt,
            }
        }

        (data, result, error)
    }

    pub fn read1_byte_tx_rx(&mut self, dxl_id: u8, address: u16) -> (u8, CommResult, u8) {
        let (data, result, error) = self.read_tx_rx(dxl_id, address, 1);
        match (result, data.as_slice()) {
            (CommResult::Success, [value, ..]) => (*value, result, error),
            _ => (0, result, error),
        }
    }

    pub fn read2_byte_tx_rx(&mut self, dxl_id: u8, address: u16) -> (u16, CommResult, u8) {
        let (data, result, error) = self.read_tx_rx(dxl_id, address, 2);
        match (result, data.as_slice()) {
            (CommResult::Success, [low, high, ..]) => (dxl_makeword(*low, *high), result, error),
            _ => (0, result, error),
        }
    }

    pub fn read4_byte_tx_rx(&mut self, dxl_id: u8, address: u16) -> (u32, CommResult, u8) {
        let (data, result, error) = self.read_tx_rx(dxl_id, address, 4);
        match (result, data.as_slice()) {
            (CommResult::Success, [a, b, c, d, ..]) => (
                dxl_makedword(dxl_makeword(*a, *b), dxl_makeword(*c, *d)),
                result,
                error,
            ),
            _ => (0, result, error),
        }
    }

    /// `writeTxRx`: write `data` at `address` and receive the acknowledgement: the result and
    /// the error byte.
    pub fn write_tx_rx(&mut self, dxl_id: u8, address: u16, data: &[u8]) -> (CommResult, u8) {
        let Some(length) = u16::try_from(data.len())
            .ok()
            .and_then(|length| length.checked_add(5))
        else {
            return (CommResult::TxError, 0);
        };
        let mut txpacket = vec![0_u8; data.len() + 12];

        txpacket[PKT_ID] = dxl_id;
        txpacket[PKT_LENGTH_L] = dxl_lobyte(length);
        txpacket[PKT_LENGTH_H] = dxl_hibyte(length);
        txpacket[PKT_INSTRUCTION] = INST_WRITE;
        txpacket[PKT_PARAMETER0] = dxl_lobyte(address);
        txpacket[PKT_PARAMETER0 + 1] = dxl_hibyte(address);

        txpacket[PKT_PARAMETER0 + 2..PKT_PARAMETER0 + 2 + data.len()].copy_from_slice(data);
        let (_, result, error) = self.tx_rx_packet(&txpacket);

        (result, error)
    }

    pub fn write1_byte_tx_rx(&mut self, dxl_id: u8, address: u16, data: u8) -> (CommResult, u8) {
        self.write_tx_rx(dxl_id, address, &[data])
    }

    pub fn write2_byte_tx_rx(&mut self, dxl_id: u8, address: u16, data: u16) -> (CommResult, u8) {
        self.write_tx_rx(dxl_id, address, &[dxl_lobyte(data), dxl_hibyte(data)])
    }

    pub fn write4_byte_tx_rx(&mut self, dxl_id: u8, address: u16, data: u32) -> (CommResult, u8) {
        let data_write = [
            dxl_lobyte(dxl_loword(data)),
            dxl_hibyte(dxl_loword(data)),
            dxl_lobyte(dxl_hiword(data)),
            dxl_hibyte(dxl_hiword(data)),
        ];
        self.write_tx_rx(dxl_id, address, &data_write)
    }

    /// `syncReadTx`: ask every device in `param` (one id each) for `data_length` bytes from
    /// `start_address`; each answers with its own status packet, read by [`Self::read_rx`].
    pub fn sync_read_tx(
        &mut self,
        start_address: u16,
        data_length: u16,
        param: &[u8],
    ) -> CommResult {
        let Some(length) = u16::try_from(param.len())
            .ok()
            .and_then(|length| length.checked_add(7))
        else {
            return CommResult::TxError;
        };
        // 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H
        // DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        let mut txpacket = vec![0_u8; param.len() + 14];

        txpacket[PKT_ID] = BROADCAST_ID;
        // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        txpacket[PKT_LENGTH_L] = dxl_lobyte(length);
        txpacket[PKT_LENGTH_H] = dxl_hibyte(length);
        txpacket[PKT_INSTRUCTION] = INST_SYNC_READ;
        txpacket[PKT_PARAMETER0] = dxl_lobyte(start_address);
        txpacket[PKT_PARAMETER0 + 1] = dxl_hibyte(start_address);
        txpacket[PKT_PARAMETER0 + 2] = dxl_lobyte(data_length);
        txpacket[PKT_PARAMETER0 + 3] = dxl_hibyte(data_length);

        txpacket[PKT_PARAMETER0 + 4..PKT_PARAMETER0 + 4 + param.len()].copy_from_slice(param);

        let result = self.tx_packet(&txpacket);
        if result == CommResult::Success {
            self.port_handler
                .set_packet_timeout((11 + usize::from(data_length)) * param.len());
        }

        result
    }

    /// `syncWriteTxOnly`: one broadcast SYNC WRITE of `param` (id, then `data_length` bytes,
    /// per device).
    pub fn sync_write_tx_only(
        &mut self,
        start_address: u16,
        data_length: u16,
        param: &[u8],
    ) -> CommResult {
        let Some(length) = u16::try_from(param.len())
            .ok()
            .and_then(|length| length.checked_add(7))
        else {
            return CommResult::TxError;
        };
        let mut txpacket = vec![0_u8; param.len() + 14];

        txpacket[PKT_ID] = BROADCAST_ID;
        // 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        txpacket[PKT_LENGTH_L] = dxl_lobyte(length);
        txpacket[PKT_LENGTH_H] = dxl_hibyte(length);
        txpacket[PKT_INSTRUCTION] = INST_SYNC_WRITE;
        txpacket[PKT_PARAMETER0] = dxl_lobyte(start_address);
        txpacket[PKT_PARAMETER0 + 1] = dxl_hibyte(start_address);
        txpacket[PKT_PARAMETER0 + 2] = dxl_lobyte(data_length);
        txpacket[PKT_PARAMETER0 + 3] = dxl_hibyte(data_length);

        txpacket[PKT_PARAMETER0 + 4..PKT_PARAMETER0 + 4 + param.len()].copy_from_slice(param);

        let (_, result, _) = self.tx_rx_packet(&txpacket);

        result
    }
}

/// `GroupSyncWrite`: one register range written on many devices by one broadcast packet.
pub struct GroupSyncWrite {
    start_address: u16,
    data_length: u16,
    is_param_changed: bool,
    param: Vec<u8>,
    /// In insertion order, as the vendor's `dict` keeps it.
    data_dict: Vec<(u8, Vec<u8>)>,
}

impl GroupSyncWrite {
    #[must_use]
    pub const fn new(start_address: u16, data_length: u16) -> Self {
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

        for (dxl_id, data) in &self.data_dict {
            if data.is_empty() {
                return;
            }

            self.param.push(*dxl_id);
            self.param.extend_from_slice(data);
        }
    }

    /// Add one device's bytes; refused (`false`) for an id already added or data too long.
    pub fn add_param(&mut self, dxl_id: u8, data: &[u8]) -> bool {
        if self.data_dict.iter().any(|(id, _)| *id == dxl_id) {
            // dxl_id already exist
            return false;
        }

        if data.len() > usize::from(self.data_length) {
            // input data is longer than set
            return false;
        }

        self.data_dict.push((dxl_id, data.to_vec()));

        self.is_param_changed = true;
        true
    }

    pub fn clear_param(&mut self) {
        self.data_dict.clear();
    }

    /// Send the packet through `ph`.
    pub fn tx_packet<P: SerialPort>(&mut self, ph: &mut Protocol2PacketHandler<P>) -> CommResult {
        if self.data_dict.is_empty() {
            return CommResult::NotAvailable;
        }

        if self.is_param_changed || self.param.is_empty() {
            self.make_param();
        }

        ph.sync_write_tx_only(self.start_address, self.data_length, &self.param)
    }
}

/// `GroupSyncRead`: one register range read from many devices, each answering in turn.
pub struct GroupSyncRead {
    start_address: u16,
    data_length: u16,
    last_result: bool,
    /// In insertion order, as the vendor's `dict` keeps it.
    data_dict: Vec<(u8, Vec<u8>)>,
    /// Each device's status error byte from the last read, in the same order; the vendor's
    /// `txRxPacket` drops it.
    errors: Vec<u8>,
}

impl GroupSyncRead {
    #[must_use]
    pub const fn new(start_address: u16, data_length: u16) -> Self {
        Self {
            start_address,
            data_length,
            last_result: false,
            data_dict: Vec::new(),
            errors: Vec::new(),
        }
    }

    /// Add one device; refused (`false`) for an id already added.
    pub fn add_param(&mut self, dxl_id: u8) -> bool {
        if self.data_dict.iter().any(|(id, _)| *id == dxl_id) {
            return false;
        }
        self.data_dict.push((dxl_id, Vec::new()));
        true
    }

    pub fn clear_param(&mut self) {
        self.data_dict.clear();
        self.errors.clear();
    }

    /// `txRxPacket`: send the SYNC READ through `ph`, then read every device's answer in the
    /// order they were added, stopping at the first that fails.
    pub fn tx_rx_packet<P: SerialPort>(
        &mut self,
        ph: &mut Protocol2PacketHandler<P>,
    ) -> CommResult {
        self.last_result = false;
        if self.data_dict.is_empty() {
            return CommResult::NotAvailable;
        }
        let param: Vec<u8> = self.data_dict.iter().map(|(dxl_id, _)| *dxl_id).collect();
        let result = ph.sync_read_tx(self.start_address, self.data_length, &param);
        if result != CommResult::Success {
            return result;
        }
        self.errors.clear();
        for (dxl_id, data) in &mut self.data_dict {
            let (bytes, result, error) = ph.read_rx(*dxl_id, self.data_length);
            *data = bytes;
            self.errors.push(error);
            if result != CommResult::Success {
                return result;
            }
        }
        self.last_result = true;
        CommResult::Success
    }

    /// The status error byte `dxl_id` answered the last read with, when the whole read succeeded.
    #[must_use]
    pub fn get_error(&self, dxl_id: u8) -> Option<u8> {
        if !self.last_result {
            return None;
        }
        let index = self.data_dict.iter().position(|(id, _)| *id == dxl_id)?;
        self.errors.get(index).copied()
    }

    /// `getData`: `data_length` (1, 2 or 4) bytes at `address` from `dxl_id`'s last answer, or
    /// `None` when the last read failed or does not cover them.
    #[must_use]
    pub fn get_data(&self, dxl_id: u8, address: u16, data_length: u16) -> Option<u32> {
        if !self.last_result
            || address < self.start_address
            || self.start_address + self.data_length < address + data_length
        {
            return None;
        }
        let data = &self.data_dict.iter().find(|(id, _)| *id == dxl_id)?.1;
        let start = usize::from(address - self.start_address);
        let byte = |offset: usize| data.get(start + offset).copied();
        match data_length {
            1 => byte(0).map(u32::from),
            2 => Some(u32::from(dxl_makeword(byte(0)?, byte(1)?))),
            4 => Some(dxl_makedword(
                dxl_makeword(byte(0)?, byte(1)?),
                dxl_makeword(byte(2)?, byte(3)?),
            )),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stuffing_adds_an_fd_after_every_header_pattern_and_removing_restores_it() {
        // A write of FF FF FD 00 to address 0x0074 on id 1, LENGTH 9 before stuffing.
        let packet = [
            0, 0, 0, 0, 1, 9, 0, INST_WRITE, 0x74, 0x00, 0xFF, 0xFF, 0xFD, 0x00, 0, 0,
        ];
        let stuffed = add_stuffing(&packet);
        assert_eq!(
            stuffed,
            [
                0, 0, 0, 0, 1, 10, 0, INST_WRITE, 0x74, 0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0x00, 0, 0
            ]
        );
        // A packet without the pattern is unchanged.
        let plain = [0, 0, 0, 0, 1, 3, 0, INST_PING, 0, 0];
        assert_eq!(add_stuffing(&plain), plain);
        let mut received = stuffed;
        received[PKT_INSTRUCTION] = INST_STATUS;
        remove_stuffing(&mut received);
        let mut expected = packet.to_vec();
        expected[PKT_INSTRUCTION] = INST_STATUS;
        assert_eq!(received, expected);
    }

    #[test]
    fn the_crc_is_the_vendors_over_the_e_manual_ping() {
        let ping = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01];
        assert_eq!(update_crc(0, &ping), dxl_makeword(0x19, 0x4E));
    }
}
