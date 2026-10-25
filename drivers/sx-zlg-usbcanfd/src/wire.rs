//! The device's bulk messages, encoded and decoded without I/O (`docs/PROTOCOL.md` §2–§5).
//!
//! Everything here is a pure function of bytes, so every worked example of the specification is
//! a test. Nothing allocates: encoders write into caller buffers, the receive decoder hands each
//! record to a closure.

use thiserror::Error;

use crate::crc::crc16;

/// Every bulk message starts with this byte.
pub const SYNC: u8 = 0x55;
/// The request header on 0x01, 0x02 and 0x83.
pub const HEADER_LEN: usize = 8;
/// The command reply header on 0x81.
pub const REPLY_HEADER_LEN: usize = 12;
/// A command's data block is always exactly this long (§3.1).
pub const COMMAND_BLOCK: usize = 1024;
/// The transmit reply on 0x82 (§4.4).
pub const TX_REPLY_LEN: usize = 8;
/// A CAN-FD-capable frame record (type 0xF2).
pub const FD_RECORD: usize = 80;
/// A classic or error record (types 0xF1, 0xF3, 0xF4).
pub const SHORT_RECORD: usize = 24;
/// The most frames one transmit transfer may carry (§4.1).
pub const MAX_BATCH: usize = 96;
/// The largest transmit transfer: header, count, 96 records, CRC.
pub const MAX_TX_TRANSFER: usize = HEADER_LEN + 4 + MAX_BATCH * FD_RECORD + 2;

/// Classic CAN frames on 0x83.
pub const TYPE_CLASSIC: u8 = 0xF1;
/// CAN-FD-capable frames, on 0x83 and the only transmit type.
pub const TYPE_FD: u8 = 0xF2;
/// CAN error events on 0x83.
pub const TYPE_ERROR: u8 = 0xF3;
/// LIN frames on 0x83, skipped.
pub const TYPE_LIN: u8 = 0xF4;

/// Flags word bits (§4.3).
pub mod flags {
    /// Bits 4–7: 0 classic, 1 CAN FD.
    pub const FORMAT_SHIFT: u32 = 4;
    pub const FORMAT_MASK: u32 = 0xF << FORMAT_SHIFT;
    pub const REMOTE: u32 = 1 << 8;
    pub const EXTENDED: u32 = 1 << 9;
    pub const ERROR: u32 = 1 << 10;
    pub const BRS: u32 = 1 << 11;
    pub const ESI: u32 = 1 << 12;
    pub const OVERFLOW_SHIFT: u32 = 16;
    pub const OVERFLOW_MASK: u32 = 0xF << OVERFLOW_SHIFT;
    pub const MICROSECONDS: u32 = 1 << 20;
}

/// Identifier bits 0–28.
pub const ID_MASK: u32 = 0x1FFF_FFFF;

/// A message the device sent that this crate cannot accept, or a request it cannot encode.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum WireError {
    #[error("message of {0} bytes is shorter than its header")]
    Short(usize),
    #[error("sync byte {0:#04x} is not 0x55")]
    Sync(u8),
    #[error("message type {0:#04x} is none this crate knows")]
    Type(u8),
    #[error("declared payload of {declared} bytes does not fit the {available} received")]
    Length { declared: usize, available: usize },
    #[error(
        "record count {count} of {size}-byte records does not fill the declared {length} bytes"
    )]
    Count {
        count: usize,
        size: usize,
        length: usize,
    },
    #[error("a command payload of {0} bytes does not fit its 1024-byte block")]
    Payload(usize),
    #[error("a transmit batch of {0} frames is outside 1..=96")]
    Batch(usize),
    #[error("frame length {0} is not a CAN length for its format")]
    FrameLength(usize),
    #[error("identifier {id:#x} does not fit a {} identifier", if *extended { "29-bit" } else { "11-bit" })]
    Identifier { id: u32, extended: bool },
}

/// The 8-byte request header: sync, code, argument, sequence 0, payload length.
#[must_use]
pub fn request_header(code: u8, argument: u8, length: u32) -> [u8; HEADER_LEN] {
    let length = length.to_le_bytes();
    [
        SYNC, code, argument, 0, length[0], length[1], length[2], length[3],
    ]
}

/// The configuration commands this crate sends (§3.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Command {
    DeviceOpen = 0x01,
    DeviceClose = 0x02,
    ChannelInit = 0x11,
    ChannelStart = 0x12,
    ChannelStop = 0x13,
    Termination = 0x18,
    DeviceInformation = 0x41,
    TransmitTimeout = 0x44,
    ReadTransferProperties = 0x46,
    WriteTransferProperties = 0x47,
}

impl Command {
    /// The reply payload length the device must return for this command.
    #[must_use]
    pub const fn reply_len(self) -> usize {
        match self {
            Self::DeviceInformation => DEVICE_INFO_LEN,
            Self::ReadTransferProperties => 1,
            _ => 0,
        }
    }
}

/// One command's two OUT transfers: the header, then the 1024-byte block of payload, CRC and
/// zero fill (§3.1).
///
/// # Errors
///
/// Refuses a payload that leaves no room for the CRC in the block.
pub fn command(
    command: Command,
    argument: u8,
    payload: &[u8],
) -> Result<([u8; HEADER_LEN], [u8; COMMAND_BLOCK]), WireError> {
    if payload.len() > COMMAND_BLOCK - 2 {
        return Err(WireError::Payload(payload.len()));
    }
    let length = u32::try_from(payload.len()).map_err(|_| WireError::Payload(payload.len()))?;
    let header = request_header(command as u8, argument, length);
    let mut block = [0_u8; COMMAND_BLOCK];
    block[..payload.len()].copy_from_slice(payload);
    let crc = crc_over(header, payload);
    block[payload.len()..payload.len() + 2].copy_from_slice(&crc.to_le_bytes());
    Ok((header, block))
}

/// The CRC over a header and the payload that follows it, without joining them.
fn crc_over(header: [u8; HEADER_LEN], payload: &[u8]) -> u16 {
    let mut joined = [0_u8; HEADER_LEN + COMMAND_BLOCK];
    joined[..HEADER_LEN].copy_from_slice(&header);
    joined[HEADER_LEN..HEADER_LEN + payload.len()].copy_from_slice(payload);
    crc16(&joined[..HEADER_LEN + payload.len()])
}

/// A command reply's 12-byte header (§2.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReplyHeader {
    pub code: u8,
    pub argument: u8,
    pub sequence: u8,
    pub status: u8,
    pub length: usize,
}

/// Why a command's reply was not accepted.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum ReplyError {
    #[error(transparent)]
    Wire(#[from] WireError),
    #[error("reply echoes command {found:#04x} for request {expected:#04x}")]
    Code { expected: u8, found: u8 },
    #[error("reply echoes sequence {0}, not the request's 0")]
    Sequence(u8),
    #[error("device refused command {code:#04x} with status {status} (meaning undocumented)")]
    Status { code: u8, status: u8 },
    #[error("reply to {code:#04x} carries {found} payload bytes, not {expected}")]
    PayloadLength {
        code: u8,
        expected: usize,
        found: usize,
    },
    #[error("reply data phase returned {0} bytes, fewer than its payload and CRC")]
    DataShort(usize),
}

impl ReplyHeader {
    /// Parse the 12-byte header.
    ///
    /// # Errors
    ///
    /// Refuses a short message or a wrong sync byte.
    pub fn parse(bytes: &[u8]) -> Result<Self, WireError> {
        if bytes.len() < REPLY_HEADER_LEN {
            return Err(WireError::Short(bytes.len()));
        }
        if bytes[0] != SYNC {
            return Err(WireError::Sync(bytes[0]));
        }
        Ok(Self {
            code: bytes[1],
            argument: bytes[2],
            sequence: bytes[3],
            status: bytes[4],
            length: u32_at(bytes, 8) as usize,
        })
    }

    /// Accept this reply to `command` as the reference does: same code and sequence, status
    /// 0, the command's payload length (§3.1).
    ///
    /// # Errors
    ///
    /// Names the first check the reply fails.
    pub fn accept(&self, command: Command) -> Result<(), ReplyError> {
        let expected = command as u8;
        if self.code != expected {
            return Err(ReplyError::Code {
                expected,
                found: self.code,
            });
        }
        if self.sequence != 0 {
            return Err(ReplyError::Sequence(self.sequence));
        }
        if self.status != 0 {
            return Err(ReplyError::Status {
                code: expected,
                status: self.status,
            });
        }
        if self.length != command.reply_len() {
            return Err(ReplyError::PayloadLength {
                code: expected,
                expected: command.reply_len(),
                found: self.length,
            });
        }
        Ok(())
    }
}

/// The device information reply's length (§3.6).
pub const DEVICE_INFO_LEN: usize = 68;

/// The device information reply (§3.6).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceInformation {
    pub hardware_version: u16,
    pub firmware_version: u16,
    pub bootloader_version: u16,
    pub identity: String,
    pub serial: String,
    pub can_ports: u8,
    pub lin_ports: u8,
}

impl DeviceInformation {
    /// Parse the 68-byte payload.
    ///
    /// # Errors
    ///
    /// Refuses a payload shorter than 68 bytes.
    pub fn parse(bytes: &[u8]) -> Result<Self, WireError> {
        if bytes.len() < DEVICE_INFO_LEN {
            return Err(WireError::Short(bytes.len()));
        }
        let text = |range: std::ops::Range<usize>| {
            let field = &bytes[range];
            let end = field
                .iter()
                .position(|byte| *byte == 0)
                .unwrap_or(field.len());
            String::from_utf8_lossy(&field[..end]).trim().to_owned()
        };
        Ok(Self {
            hardware_version: u16::from_le_bytes([bytes[0], bytes[1]]),
            firmware_version: u16::from_le_bytes([bytes[2], bytes[3]]),
            bootloader_version: u16::from_le_bytes([bytes[4], bytes[5]]),
            identity: text(6..46),
            serial: text(46..66),
            can_ports: bytes[66],
            lin_ports: bytes[67],
        })
    }

    /// The CAN controller clock the device runs (§1.3 step 6): 80 MHz from hardware 3.00 or
    /// on a four-port unit, else 60 MHz.
    #[must_use]
    pub const fn clock_hz(&self) -> u32 {
        if self.hardware_version >= 0x0300 || self.can_ports == 4 {
            80_000_000
        } else {
            60_000_000
        }
    }

    /// The CAN channels this crate drives: at most four (§1.3 step 7).
    #[must_use]
    pub fn channels(&self) -> u8 {
        self.can_ports.min(4)
    }
}

/// One phase's bit timing in real (not decremented) values (§3.3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timing {
    pub prescaler: u16,
    pub tseg1: u16,
    pub tseg2: u16,
    pub sjw: u16,
}

/// The channel initialise payload (§3.3): clock, mode bits, nominal then data-phase timing,
/// each field decremented by one.
#[must_use]
pub fn init_payload(clock_hz: u32, mode: u32, nominal: Timing, data: Timing) -> [u8; 20] {
    let mut payload = [0_u8; 20];
    payload[..4].copy_from_slice(&clock_hz.to_le_bytes());
    payload[4..8].copy_from_slice(&mode.to_le_bytes());
    for (offset, timing) in [(8, nominal), (14, data)] {
        payload[offset] = low_byte(timing.tseg1 - 1);
        payload[offset + 1] = low_byte(timing.tseg2 - 1);
        payload[offset + 2] = low_byte(timing.sjw - 1);
        payload[offset + 3] = 0;
        payload[offset + 4..offset + 6].copy_from_slice(&(timing.prescaler - 1).to_le_bytes());
    }
    payload
}

fn low_byte(value: u16) -> u8 {
    value.to_le_bytes()[0]
}

/// The data lengths a CAN FD frame may carry.
const FD_LENGTHS: [usize; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];

/// One frame to transmit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxFrame {
    pub id: u32,
    pub extended: bool,
    pub fd: bool,
    pub brs: bool,
    pub len: u8,
    pub data: [u8; 64],
}

impl TxFrame {
    /// A classic data frame.
    ///
    /// # Errors
    ///
    /// Refuses more than 8 bytes and an identifier wider than its format.
    pub fn classic(id: u32, extended: bool, data: &[u8]) -> Result<Self, WireError> {
        Self::new(id, extended, false, false, data)
    }

    /// A CAN FD data frame, with or without bit-rate switch.
    ///
    /// # Errors
    ///
    /// Refuses a length CAN FD cannot carry and an identifier wider than its format.
    pub fn fd(id: u32, extended: bool, brs: bool, data: &[u8]) -> Result<Self, WireError> {
        Self::new(id, extended, true, brs, data)
    }

    fn new(id: u32, extended: bool, fd: bool, brs: bool, data: &[u8]) -> Result<Self, WireError> {
        let limit = if extended { ID_MASK } else { 0x7FF };
        if id > limit {
            return Err(WireError::Identifier { id, extended });
        }
        let allowed = if fd {
            FD_LENGTHS.contains(&data.len())
        } else {
            data.len() <= 8
        };
        if !allowed {
            return Err(WireError::FrameLength(data.len()));
        }
        let mut bytes = [0_u8; 64];
        bytes[..data.len()].copy_from_slice(data);
        Ok(Self {
            id,
            extended,
            fd,
            brs: fd && brs,
            len: low_byte(u16::try_from(data.len()).unwrap_or(0)),
            data: bytes,
        })
    }

    /// The record's flags word: normal transmission, format, extended, BRS for FD.
    #[must_use]
    pub const fn flags(&self) -> u32 {
        let mut word = 0;
        if self.fd {
            word |= 1 << flags::FORMAT_SHIFT;
            if self.brs {
                word |= flags::BRS;
            }
        }
        if self.extended {
            word |= flags::EXTENDED;
        }
        word
    }
}

/// The size of a transmit transfer of `count` frames: `14 + 80 × count` (§4.1).
#[must_use]
pub const fn transmit_len(count: usize) -> usize {
    HEADER_LEN + 4 + count * FD_RECORD + 2
}

/// Encode one transmit transfer of `frames` on `channel` into `out` (§4.1, §4.2): type 0xF2,
/// 80-byte records, CRC over everything before it. Returns the transfer's length.
///
/// # Errors
///
/// Refuses an empty batch, more than [`MAX_BATCH`] frames, and an `out` too small.
pub fn encode_transmit(
    channel: u8,
    frames: &[TxFrame],
    out: &mut [u8],
) -> Result<usize, WireError> {
    let count = frames.len();
    if count == 0 || count > MAX_BATCH {
        return Err(WireError::Batch(count));
    }
    let total = transmit_len(count);
    if out.len() < total {
        return Err(WireError::Length {
            declared: total,
            available: out.len(),
        });
    }
    let payload = u32::try_from(4 + count * FD_RECORD).map_err(|_| WireError::Batch(count))?;
    out[..HEADER_LEN].copy_from_slice(&request_header(TYPE_FD, channel, payload));
    let count16 = u16::try_from(count).map_err(|_| WireError::Batch(count))?;
    out[8..10].copy_from_slice(&count16.to_le_bytes());
    out[10..12].fill(0);
    for (index, frame) in frames.iter().enumerate() {
        let record = &mut out[12 + index * FD_RECORD..12 + (index + 1) * FD_RECORD];
        record[..4].fill(0);
        record[4..8].copy_from_slice(&(frame.id & ID_MASK).to_le_bytes());
        record[8..12].copy_from_slice(&frame.flags().to_le_bytes());
        record[12..14].fill(0);
        record[14] = channel;
        record[15] = frame.len;
        record[16..80].copy_from_slice(&frame.data);
    }
    let crc = crc16(&out[..total - 2]);
    out[total - 2..total].copy_from_slice(&crc.to_le_bytes());
    Ok(total)
}

/// The 8-byte transmit reply (§4.4).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxReply {
    pub channel: u8,
    pub status: u8,
    pub sent: u8,
}

impl TxReply {
    /// Parse a transmit reply; the reference accepts it on sync, type 0xF2 and status 0, and
    /// does not check its CRC.
    ///
    /// # Errors
    ///
    /// Refuses a short reply, a wrong sync byte or type.
    pub fn parse(bytes: &[u8]) -> Result<Self, WireError> {
        if bytes.len() < 6 {
            return Err(WireError::Short(bytes.len()));
        }
        if bytes[0] != SYNC {
            return Err(WireError::Sync(bytes[0]));
        }
        if bytes[1] != TYPE_FD {
            return Err(WireError::Type(bytes[1]));
        }
        Ok(Self {
            channel: bytes[2],
            status: bytes[4],
            sent: bytes[5],
        })
    }
}

/// One received data frame (§5.2).
// One field per flag the record carries, named as the specification names them.
#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RxFrame {
    pub channel: u8,
    pub id: u32,
    pub extended: bool,
    pub remote: bool,
    pub fd: bool,
    pub brs: bool,
    pub esi: bool,
    pub len: u8,
    pub data: [u8; 64],
    /// Device time in microseconds since its own epoch (§5.3).
    pub timestamp_us: u64,
}

impl RxFrame {
    pub const EMPTY: Self = Self {
        channel: 0,
        id: 0,
        extended: false,
        remote: false,
        fd: false,
        brs: false,
        esi: false,
        len: 0,
        data: [0; 64],
        timestamp_us: 0,
    };
}

/// One record of a received transfer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Record {
    Frame(RxFrame),
    /// An error event on a channel; its 8 data bytes are undocumented (§5.4) and kept raw.
    Error {
        channel: u8,
        data: [u8; 8],
    },
}

/// What one received transfer held besides what reached the sink.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Decoded {
    /// Records whose channel byte is not a port of this device; skipped, as the reference does.
    pub foreign_channel: usize,
    /// LIN records, skipped whole.
    pub lin: usize,
    /// Data records claiming more data bytes than their record holds.
    pub malformed: usize,
}

/// A record's timestamp in microseconds (§5.3).
#[must_use]
pub fn timestamp_us(ts32: u32, high16: u16, word: u32) -> u64 {
    let overflow = u64::from((word & flags::OVERFLOW_MASK) >> flags::OVERFLOW_SHIFT);
    if word & flags::MICROSECONDS != 0 {
        u64::from(ts32) | (u64::from(high16) << 32) | (overflow << 48)
    } else {
        (u64::from(ts32) | (overflow << 32)) * 100
    }
}

/// Decode one transfer received on 0x83 (§5.1), handing every record on a port of this device
/// to `sink`, in order. `buffer` is the host buffer size the transfer was read into.
///
/// # Errors
///
/// Refuses the whole transfer, as the reference does, on a short message, a wrong sync byte, an
/// unknown type, a payload longer than `buffer − 10` or than the bytes received, or a record
/// count that does not fill the declared payload.
pub fn decode_receive(
    bytes: &[u8],
    buffer: usize,
    can_ports: u8,
    mut sink: impl FnMut(Record),
) -> Result<Decoded, WireError> {
    if bytes.len() < HEADER_LEN + 4 {
        return Err(WireError::Short(bytes.len()));
    }
    if bytes[0] != SYNC {
        return Err(WireError::Sync(bytes[0]));
    }
    let kind = bytes[1];
    let size = match kind {
        TYPE_FD => FD_RECORD,
        TYPE_CLASSIC | TYPE_ERROR | TYPE_LIN => SHORT_RECORD,
        other => return Err(WireError::Type(other)),
    };
    let length = u32_at(bytes, 4) as usize;
    let available = bytes.len() - HEADER_LEN;
    if length > buffer.saturating_sub(10) || length > available {
        return Err(WireError::Length {
            declared: length,
            available: available.min(buffer.saturating_sub(10)),
        });
    }
    let count = usize::from(u16::from_le_bytes([bytes[8], bytes[9]]));
    if length != 4 + count * size {
        return Err(WireError::Count {
            count,
            size,
            length,
        });
    }
    let mut decoded = Decoded::default();
    if kind == TYPE_LIN {
        decoded.lin = count;
        return Ok(decoded);
    }
    for index in 0..count {
        let record = &bytes[12 + index * size..12 + (index + 1) * size];
        let channel = record[14];
        if channel >= can_ports {
            decoded.foreign_channel += 1;
            continue;
        }
        if kind == TYPE_ERROR {
            let mut data = [0_u8; 8];
            data.copy_from_slice(&record[16..24]);
            sink(Record::Error { channel, data });
            continue;
        }
        let word = u32_at(record, 8);
        let len = usize::from(record[15]);
        if len > size - 16 || len > 64 {
            decoded.malformed += 1;
            continue;
        }
        let mut data = [0_u8; 64];
        data[..len].copy_from_slice(&record[16..16 + len]);
        sink(Record::Frame(RxFrame {
            channel,
            id: u32_at(record, 4) & ID_MASK,
            extended: word & flags::EXTENDED != 0,
            remote: word & flags::REMOTE != 0,
            fd: word & flags::FORMAT_MASK != 0,
            brs: word & flags::BRS != 0,
            esi: word & flags::ESI != 0,
            len: record[15],
            data,
            timestamp_us: timestamp_us(
                u32_at(record, 0),
                u16::from_le_bytes([record[12], record[13]]),
                word,
            ),
        }));
    }
    Ok(decoded)
}

fn u32_at(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(text: &str) -> Vec<u8> {
        text.split_whitespace()
            .map(|byte| u8::from_str_radix(byte, 16).expect("hex"))
            .collect()
    }

    #[test]
    fn the_channel_open_sequence_is_the_specs_worked_example() {
        // §7.1: each command's header and the start of its 1024-byte block.
        let cases: [(Command, &[u8], &str, &str); 5] = [
            (Command::DeviceOpen, &[], "55 01 00 00 00 00 00 00", "95 C8"),
            (
                Command::ChannelInit,
                &init_payload(
                    80_000_000,
                    0,
                    Timing {
                        prescaler: 2,
                        tseg1: 29,
                        tseg2: 10,
                        sjw: 2,
                    },
                    Timing {
                        prescaler: 2,
                        tseg1: 29,
                        tseg2: 10,
                        sjw: 2,
                    },
                ),
                "55 11 00 00 14 00 00 00",
                "00 B4 C4 04 00 00 00 00 1C 09 01 00 01 00 1C 09 01 00 01 00 97 EA",
            ),
            (
                Command::TransmitTimeout,
                &20_u32.to_le_bytes(),
                "55 44 00 00 04 00 00 00",
                "14 00 00 00 EC 3C",
            ),
            (
                Command::Termination,
                &[0x55],
                "55 18 00 00 01 00 00 00",
                "55 34 F6",
            ),
            (
                Command::ChannelStart,
                &[],
                "55 12 00 00 00 00 00 00",
                "B7 09",
            ),
        ];
        for (code, payload, header, block) in cases {
            let (encoded_header, encoded_block) = command(code, 0, payload).expect("encodes");
            assert_eq!(encoded_header.to_vec(), hex(header), "{code:?}");
            let block = hex(block);
            assert_eq!(&encoded_block[..block.len()], block.as_slice(), "{code:?}");
            assert!(encoded_block[block.len()..].iter().all(|byte| *byte == 0));
        }
        // The other zero-payload commands of §2.3's table.
        for (code, crc) in [
            (Command::DeviceInformation, [0xD4, 0x0C]),
            (Command::ChannelStop, [0xA7, 0xC9]),
            (Command::DeviceClose, [0xA6, 0xC8]),
            (Command::ReadTransferProperties, [0xA2, 0xCC]),
        ] {
            assert_eq!(command(code, 0, &[]).expect("encodes").1[..2], crc);
        }
        let (_, block) = command(Command::WriteTransferProperties, 0, &[1]).expect("encodes");
        assert_eq!(block[..3], [0x01, 0x70, 0x75]);
        assert!(command(Command::DeviceOpen, 0, &[0; 1023]).is_err());
    }

    #[test]
    fn a_classic_frame_transmit_is_the_specs_worked_example() {
        // §7.2: standard id 0x7FF, a Damiao register read, on channel 0.
        let frame =
            TxFrame::classic(0x7FF, false, &[0x01, 0x00, 0x33, 0x0A, 0, 0, 0, 0]).expect("frame");
        let mut out = [0_u8; MAX_TX_TRANSFER];
        let len = encode_transmit(0, &[frame], &mut out).expect("encodes");
        assert_eq!(len, 94);
        let mut expected = hex(
            "55 F2 00 00 54 00 00 00 01 00 00 00 00 00 00 00 FF 07 00 00 00 00 00 00 00 00 00 08 \
             01 00 33 0A 00 00 00 00",
        );
        expected.extend([0; 56]);
        expected.extend([0x3B, 0x0E]);
        assert_eq!(&out[..len], expected.as_slice());
    }

    #[test]
    fn batches_carry_one_channel_and_up_to_96_frames() {
        let frame = TxFrame::classic(0x123, false, &[1, 2]).expect("frame");
        let mut out = [0_u8; MAX_TX_TRANSFER];
        let len = encode_transmit(1, &[frame; 12], &mut out).expect("twelve");
        assert_eq!(len, 974);
        assert_eq!(u16::from_le_bytes([out[8], out[9]]), 12);
        for index in 0..12 {
            assert_eq!(
                out[12 + index * FD_RECORD + 14],
                1,
                "every record's channel"
            );
        }
        assert_eq!(
            crc16(&out[..len - 2]).to_le_bytes(),
            [out[len - 2], out[len - 1]]
        );
        assert_eq!(encode_transmit(1, &[frame; 96], &mut out), Ok(7694));
        assert_eq!(
            encode_transmit(1, &[frame; 97], &mut out),
            Err(WireError::Batch(97))
        );
        assert_eq!(encode_transmit(1, &[], &mut out), Err(WireError::Batch(0)));
        assert!(encode_transmit(1, &[frame; 2], &mut out[..100]).is_err());
    }

    #[test]
    fn frames_are_refused_outside_their_format() {
        assert!(TxFrame::classic(0x800, false, &[]).is_err());
        assert!(TxFrame::classic(0x800, true, &[]).is_ok());
        assert!(TxFrame::classic(0x2000_0000, true, &[]).is_err());
        assert!(TxFrame::classic(1, false, &[0; 9]).is_err());
        assert!(TxFrame::fd(1, false, true, &[0; 9]).is_err());
        let fd = TxFrame::fd(1, true, true, &[0; 64]).expect("fd");
        assert_eq!(fd.flags(), (1 << 4) | flags::BRS | flags::EXTENDED);
        assert_eq!(TxFrame::classic(1, false, &[]).expect("classic").flags(), 0);
    }

    #[test]
    fn a_received_classic_frame_is_the_specs_worked_example() {
        // §7.3.
        let bytes = hex(
            "55 F1 00 00 1C 00 00 00 01 00 00 00 40 E2 01 00 23 01 00 00 00 00 10 00 00 00 01 08 \
             DE AD BE EF 01 02 03 04 E3 53",
        );
        assert_eq!(crc16(&bytes[..36]), 0x53E3);
        let mut records = Vec::new();
        let decoded =
            decode_receive(&bytes, 16384, 2, |record| records.push(record)).expect("decodes");
        assert_eq!(decoded, Decoded::default());
        let [Record::Frame(frame)] = records.as_slice() else {
            panic!("one frame, got {records:?}");
        };
        assert_eq!(frame.channel, 1);
        assert_eq!(frame.id, 0x123);
        assert!(!frame.extended && !frame.remote && !frame.fd);
        assert_eq!(usize::from(frame.len), 8);
        assert_eq!(frame.data[..8], [0xDE, 0xAD, 0xBE, 0xEF, 1, 2, 3, 4]);
        assert_eq!(frame.timestamp_us, 123_456);
    }

    /// A receive transfer of `records` of one type, with a trailing CRC.
    fn transfer(kind: u8, records: &[Vec<u8>]) -> Vec<u8> {
        let size = records.first().map_or(0, Vec::len);
        let payload = 4 + records.len() * size;
        let mut bytes = request_header(kind, 0, u32::try_from(payload).expect("fits")).to_vec();
        bytes.extend(u16::try_from(records.len()).expect("fits").to_le_bytes());
        bytes.extend([0, 0]);
        for record in records {
            bytes.extend(record);
        }
        let crc = crc16(&bytes);
        bytes.extend(crc.to_le_bytes());
        bytes
    }

    fn record(size: usize, channel: u8, id: u32, word: u32, data: &[u8]) -> Vec<u8> {
        let mut record = vec![0_u8; size];
        record[4..8].copy_from_slice(&id.to_le_bytes());
        record[8..12].copy_from_slice(&word.to_le_bytes());
        record[14] = channel;
        record[15] = u8::try_from(data.len()).expect("fits");
        record[16..16 + data.len()].copy_from_slice(data);
        record
    }

    #[test]
    fn fd_records_errors_lin_and_foreign_channels_decode_as_the_reference_does() {
        let fd_word = (1 << 4) | flags::BRS | flags::EXTENDED | flags::MICROSECONDS;
        let bytes = transfer(
            TYPE_FD,
            &[
                record(FD_RECORD, 0, 0x1234_5678, fd_word, &[7; 64]),
                record(FD_RECORD, 3, 0x10, 0, &[1]),
                record(FD_RECORD, 1, 0x7FF, 0, &[1, 2, 3]),
            ],
        );
        let mut records = Vec::new();
        let decoded = decode_receive(&bytes, 16384, 2, |r| records.push(r)).expect("decodes");
        assert_eq!(decoded.foreign_channel, 1);
        assert_eq!(records.len(), 2);
        let Record::Frame(first) = records[0] else {
            panic!("a frame")
        };
        assert!(first.fd && first.brs && first.extended);
        assert_eq!(first.id, 0x1234_5678);
        assert_eq!(first.data, [7; 64]);
        let errors = transfer(TYPE_ERROR, &[record(SHORT_RECORD, 1, 0, 0, &[9, 8])]);
        records.clear();
        decode_receive(&errors, 16384, 2, |r| records.push(r)).expect("decodes");
        assert!(matches!(records[0], Record::Error { channel: 1, data } if data[..2] == [9, 8]));
        let lin = transfer(TYPE_LIN, &vec![record(SHORT_RECORD, 0, 0, 0, &[]); 3]);
        records.clear();
        assert_eq!(
            decode_receive(&lin, 16384, 2, |r| records.push(r))
                .expect("decodes")
                .lin,
            3
        );
        assert!(records.is_empty());
        // A classic record claiming more bytes than it holds is counted, not read past.
        let mut bad = record(SHORT_RECORD, 0, 1, 0, &[]);
        bad[15] = 9;
        let bytes = transfer(TYPE_CLASSIC, &[bad]);
        assert_eq!(
            decode_receive(&bytes, 16384, 2, |_| ())
                .expect("decodes")
                .malformed,
            1
        );
    }

    #[test]
    fn invalid_transfers_are_refused_whole() {
        let good = transfer(TYPE_CLASSIC, &[record(SHORT_RECORD, 0, 1, 0, &[1])]);
        let mut sync = good.clone();
        sync[0] = 0x56;
        assert_eq!(
            decode_receive(&sync, 16384, 2, |_| ()),
            Err(WireError::Sync(0x56))
        );
        let mut kind = good.clone();
        kind[1] = 0xF5;
        assert_eq!(
            decode_receive(&kind, 16384, 2, |_| ()),
            Err(WireError::Type(0xF5))
        );
        assert!(matches!(
            decode_receive(&good[..20], 16384, 2, |_| ()),
            Err(WireError::Length { .. })
        ));
        assert!(matches!(
            decode_receive(&good, 30, 2, |_| ()),
            Err(WireError::Length { .. })
        ));
        let mut count = good.clone();
        count[8] = 2;
        assert!(matches!(
            decode_receive(&count, 16384, 2, |_| ()),
            Err(WireError::Count { .. })
        ));
        assert!(matches!(
            decode_receive(&good[..5], 16384, 2, |_| ()),
            Err(WireError::Short(5))
        ));
    }

    #[test]
    fn timestamps_follow_both_units() {
        let micro = flags::MICROSECONDS | (2 << flags::OVERFLOW_SHIFT);
        assert_eq!(timestamp_us(5, 3, micro), 5 | (3 << 32) | (2 << 48));
        let centi = 1 << flags::OVERFLOW_SHIFT;
        assert_eq!(timestamp_us(7, 0xFFFF, centi), (7 | (1 << 32)) * 100);
    }

    #[test]
    fn replies_and_device_information_parse() {
        let header = ReplyHeader::parse(&hex("55 11 00 00 00 00 00 00 00 00 00 00")).expect("ok");
        assert_eq!(header.accept(Command::ChannelInit), Ok(()));
        assert!(matches!(
            header.accept(Command::ChannelStart),
            Err(ReplyError::Code { .. })
        ));
        let refused = ReplyHeader::parse(&hex("55 11 00 00 03 00 00 00 00 00 00 00")).expect("ok");
        assert_eq!(
            refused.accept(Command::ChannelInit),
            Err(ReplyError::Status {
                code: 0x11,
                status: 3
            })
        );
        let long = ReplyHeader::parse(&hex("55 41 00 00 00 00 00 00 40 00 00 00")).expect("ok");
        assert!(matches!(
            long.accept(Command::DeviceInformation),
            Err(ReplyError::PayloadLength { found: 64, .. })
        ));
        let mut info = vec![0_u8; DEVICE_INFO_LEN];
        info[..2].copy_from_slice(&0x0301_u16.to_le_bytes());
        info[6..15].copy_from_slice(b"USBCANFD\0");
        info[46..66].copy_from_slice(b"A30F115DD0120A949B80");
        info[66] = 2;
        let info = DeviceInformation::parse(&info).expect("parses");
        assert_eq!(info.serial, "A30F115DD0120A949B80");
        assert_eq!(info.identity, "USBCANFD");
        assert_eq!(info.clock_hz(), 80_000_000);
        assert_eq!(info.channels(), 2);
        let reply = TxReply::parse(&hex("55 F2 01 00 00 0C 12 34")).expect("reply");
        assert_eq!((reply.channel, reply.status, reply.sent), (1, 0, 12));
        assert_eq!(
            TxReply::parse(&hex("55 F1 00 00 00 01")),
            Err(WireError::Type(0xF1))
        );
    }
}
