//! Linux `SocketCAN` interfaces as `sx_embodiment_drivers::can::CanPort`s.
//!
//! A `SocketCAN` interface (`can0`, `can1`) is the kernel's own view of a CAN bus, whichever
//! adapter carries it: a USB adapter on the kernel's `gs_usb` or `peak_usb` driver, or a board's
//! own controller. It is not any motor vendor's, so it lives beside the transport-neutral
//! trait rather than in a vendor crate; `sx-damiao-can`'s `usb2can` is the other transport,
//! Damiao's own serial framing.
//!
//! `SocketCanPort` (Linux only) opens one raw socket (`AF_CAN`, `SOCK_RAW`, `CAN_RAW`),
//! non-blocking, bound to one interface, holding that interface's [`InterfaceLock`] so a second
//! of our processes cannot open it too. The interface's bitrate and state are the operating
//! system's: it must already be up (`ip link set can1 up type can bitrate 1000000`). The frame
//! layout is [`encode`] and [`decode`], the kernel's `struct can_frame`, kept portable so it is
//! tested on every host.

use std::io;
use std::path::PathBuf;

use sx_embodiment_drivers::can::CanFrame;
use thiserror::Error;

#[cfg(unix)]
mod lock;
#[cfg(target_os = "linux")]
mod socket;
#[cfg(unix)]
pub use lock::InterfaceLock;
#[cfg(target_os = "linux")]
pub use socket::SocketCanPort;

/// Why an interface could not be opened.
#[derive(Debug, Error)]
pub enum SocketCanError {
    #[error("{0:?} is not a network interface name")]
    Name(String),
    #[error("SocketCAN interface {interface} is already open in another process ({})", lock.display())]
    Busy { interface: String, lock: PathBuf },
    #[error("SocketCAN interface {interface}: {source}")]
    Io {
        interface: String,
        source: io::Error,
    },
}

/// The size of the kernel's `struct can_frame`: id, length, three reserved bytes, data.
pub const FRAME_BYTES: usize = 16;

/// The id flag marking a 29-bit extended frame (`CAN_EFF_FLAG`).
const EXTENDED: u32 = 0x8000_0000;
/// The id flag marking a remote request (`CAN_RTR_FLAG`).
const REMOTE: u32 = 0x4000_0000;
/// The id flag marking an error frame (`CAN_ERR_FLAG`).
const ERROR: u32 = 0x2000_0000;
const STANDARD_MASK: u32 = 0x7FF;
const EXTENDED_MASK: u32 = 0x1FFF_FFFF;

/// `frame` as the kernel's `struct can_frame`, eight data bytes, in this host's byte order. An
/// id above 11 bits is sent as an extended frame.
///
/// # Errors
///
/// Refuses an id wider than 29 bits.
pub fn encode(frame: &CanFrame) -> io::Result<[u8; FRAME_BYTES]> {
    let id = match frame.id {
        id if id <= STANDARD_MASK => id,
        id if id <= EXTENDED_MASK => id | EXTENDED,
        id => {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("CAN id {id:#x} is wider than 29 bits"),
            ));
        }
    };
    let mut bytes = [0; FRAME_BYTES];
    bytes[..4].copy_from_slice(&id.to_ne_bytes());
    bytes[4] = 8;
    bytes[8..].copy_from_slice(&frame.data);
    Ok(bytes)
}

/// The data frame in a kernel `struct can_frame`, or `None` for a remote request or an error
/// frame. A frame shorter than eight bytes reads its missing bytes as zero.
#[must_use]
pub fn decode(bytes: &[u8; FRAME_BYTES]) -> Option<CanFrame> {
    let raw = u32::from_ne_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    if raw & (REMOTE | ERROR) != 0 {
        return None;
    }
    let id = if raw & EXTENDED == 0 {
        raw & STANDARD_MASK
    } else {
        raw & EXTENDED_MASK
    };
    let length = usize::from(bytes[4].min(8));
    let mut data = [0; 8];
    data[..length].copy_from_slice(&bytes[8..8 + length]);
    Some(CanFrame { id, data })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_frame_round_trips_through_the_kernels_layout() {
        let frame = CanFrame {
            id: 0x09,
            data: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
        };
        let bytes = encode(&frame).expect("standard id");
        assert_eq!(bytes[4], 8);
        assert_eq!(&bytes[5..8], &[0, 0, 0]);
        assert_eq!(decode(&bytes), Some(frame));
        let extended = CanFrame {
            id: 0x1234_5678,
            data: [1; 8],
        };
        let bytes = encode(&extended).expect("extended id");
        assert_ne!(
            u32::from_ne_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) & EXTENDED,
            0
        );
        assert_eq!(decode(&bytes), Some(extended));
        assert!(
            encode(&CanFrame {
                id: 0x2000_0000,
                data: [0; 8]
            })
            .is_err()
        );
    }

    #[test]
    fn remote_and_error_frames_are_not_data_and_short_frames_are_zero_filled() {
        let mut bytes = [0xAA; FRAME_BYTES];
        bytes[..4].copy_from_slice(&(0x29 | REMOTE).to_ne_bytes());
        assert_eq!(decode(&bytes), None);
        bytes[..4].copy_from_slice(&(0x29 | ERROR).to_ne_bytes());
        assert_eq!(decode(&bytes), None);
        bytes[..4].copy_from_slice(&0x29_u32.to_ne_bytes());
        bytes[4] = 2;
        assert_eq!(
            decode(&bytes),
            Some(CanFrame {
                id: 0x29,
                data: [0xAA, 0xAA, 0, 0, 0, 0, 0, 0]
            })
        );
    }
}
