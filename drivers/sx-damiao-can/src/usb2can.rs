//! Damiao's USB2CAN serial bridge as a `CanPort`, translated from `DM_CAN.py`.
//!
//! Vendor-derived: Copyright (c) 2024 cmjang, MIT License. The full permission notice is in
//! `LICENSE-THIRD-PARTY` beside this crate's `Cargo.toml`; Sentient-X's changes are Apache-2.0.
//!
//! Reference: <https://gitee.com/kit-miao/motor-sdk> at
//! `fb0e9fc5455ecb02ed13cc1f43de58078be61b07`, file `Python例程/u2can/DM_CAN.py` (Damiao's copy of
//! cmjang's `DM_Control_Python` at `7da93877ba844d9587149d6f3a6385453aa8379f`):
//! `MotorControl.send_data_frame` and `__send_data` (the 30-byte transmit packet with the CAN id
//! at bytes 13..15 and the payload at 21..29), `recv` and `__extract_packets` (16-byte receive
//! packets framed `0xAA .. 0x55`, command at byte 1, CAN id little-endian at 3..7, payload at
//! 7..15, and the unparsed remainder kept for the next read).
//!
//! This is the transport the reBot B601-DM ships with: Seeed's controller opens the bridge at
//! `/dev/ttyACM0`, 921600 baud (`reBotArm_control_py` `config/rebotarm_dm.yaml`). The serial
//! device itself is the caller's: any `Read + Write` stream whose reads return what is buffered,
//! reporting an empty buffer as `WouldBlock`, `TimedOut` or zero bytes.

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::time::Duration;

use sx_embodiment_drivers::can::{CanFrame, CanPort};

/// `MotorControl.send_data_frame`, the transmit packet before the id and payload are written.
const SEND_DATA_FRAME: [u8; 30] = [
    0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00,
    0x08, 0x00, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00,
];
const HEADER: u8 = 0xAA;
const TAIL: u8 = 0x55;
const FRAME_LENGTH: usize = 16;
/// The receive packet's command byte for a CAN frame received from the bus.
const CMD_RECEIVED: u8 = 0x11;
const READ_CHUNK: usize = 1024;

/// A USB2CAN bridge over a serial stream.
pub struct Usb2CanPort<S> {
    serial: S,
    /// `data_save`: bytes after the last whole packet, carried into the next read.
    data_save: Vec<u8>,
    received: VecDeque<CanFrame>,
    pause: Box<dyn FnMut(Duration) + Send>,
}

impl<S: Read + Write + Send> Usb2CanPort<S> {
    #[must_use]
    pub fn new(serial: S) -> Self {
        Self::with_pause(serial, Box::new(std::thread::sleep))
    }

    /// A bridge whose waits run through `pause`; a transcript replay records them instead.
    #[must_use]
    pub fn with_pause(serial: S, pause: Box<dyn FnMut(Duration) + Send>) -> Self {
        Self {
            serial,
            data_save: Vec::new(),
            received: VecDeque::new(),
            pause,
        }
    }

    #[must_use]
    pub const fn serial(&self) -> &S {
        &self.serial
    }

    /// `serial_.read_all()`: what the stream has buffered, nothing when it has nothing.
    fn read_all(&mut self) -> io::Result<Vec<u8>> {
        let mut chunk = [0_u8; READ_CHUNK];
        let mut data = Vec::new();
        loop {
            match self.serial.read(&mut chunk) {
                Ok(0) => return Ok(data),
                Ok(count) => {
                    data.extend_from_slice(&chunk[..count]);
                    if count < READ_CHUNK {
                        return Ok(data);
                    }
                }
                Err(error)
                    if matches!(
                        error.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) =>
                {
                    return Ok(data);
                }
                Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
                Err(error) => return Err(error),
            }
        }
    }
}

/// `__extract_packets`: every whole packet in `data`, and the remainder after the last one.
#[must_use]
pub fn extract_packets(data: &[u8]) -> (Vec<[u8; FRAME_LENGTH]>, &[u8]) {
    let mut frames = Vec::new();
    let mut i = 0;
    let mut remainder_pos = 0;
    while i + FRAME_LENGTH <= data.len() {
        if data[i] == HEADER && data[i + FRAME_LENGTH - 1] == TAIL {
            let mut frame = [0; FRAME_LENGTH];
            frame.copy_from_slice(&data[i..i + FRAME_LENGTH]);
            frames.push(frame);
            i += FRAME_LENGTH;
            remainder_pos = i;
        } else {
            i += 1;
        }
    }
    (frames, &data[remainder_pos..])
}

/// `__send_data`'s packet for one CAN frame.
#[must_use]
pub fn send_data_frame(frame: &CanFrame) -> [u8; 30] {
    let mut packet = SEND_DATA_FRAME;
    let [id_low, id_high, ..] = frame.id.to_le_bytes();
    packet[13] = id_low;
    packet[14] = id_high;
    packet[21..29].copy_from_slice(&frame.data);
    packet
}

impl<S: Read + Write + Send> CanPort for Usb2CanPort<S> {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        self.serial.write_all(&send_data_frame(frame))?;
        self.serial.flush()
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        if let Some(frame) = self.received.pop_front() {
            return Ok(Some(frame));
        }
        let mut data_recv = std::mem::take(&mut self.data_save);
        data_recv.extend(self.read_all()?);
        let (packets, remainder) = extract_packets(&data_recv);
        self.data_save = remainder.to_vec();
        for packet in packets {
            if packet[1] != CMD_RECEIVED {
                continue;
            }
            let mut data = [0; 8];
            data.copy_from_slice(&packet[7..15]);
            self.received.push_back(CanFrame {
                id: u32::from_le_bytes([packet[3], packet[4], packet[5], packet[6]]),
                data,
            });
        }
        Ok(self.received.pop_front())
    }

    fn sleep(&mut self, duration: Duration) {
        (self.pause)(duration);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_packet_split_across_reads_is_kept_for_the_next() {
        let packet = [
            0xAA, 0x11, 0x08, 0x11, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x80, 0x08, 0x00, 31, 33,
            0x55,
        ];
        let mut stream = vec![0x00, 0xAA];
        stream.extend_from_slice(&packet);
        let (frames, remainder) = extract_packets(&stream[..10]);
        assert!(frames.is_empty());
        assert_eq!(remainder, &stream[..10]);
        let (frames, remainder) = extract_packets(&stream);
        assert_eq!(frames, vec![packet]);
        assert!(remainder.is_empty());
    }

    #[test]
    fn the_transmit_packet_carries_id_and_payload() {
        let packet = send_data_frame(&CanFrame {
            id: 0x7FF,
            data: [1, 2, 3, 4, 5, 6, 7, 8],
        });
        assert_eq!(&packet[13..15], &[0xFF, 0x07]);
        assert_eq!(&packet[21..29], &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(&packet[..3], &[0x55, 0xAA, 0x1e]);
    }
}
