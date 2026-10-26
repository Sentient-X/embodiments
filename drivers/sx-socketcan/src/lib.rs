//! Linux `SocketCAN` interfaces as `sx_embodiment_drivers::can::CanPort`s.
//!
//! A `SocketCAN` interface (`can0`, `can1`) is the kernel's own view of a CAN bus, whichever
//! adapter carries it: a USB adapter on the kernel's `gs_usb` (candleLight) or `peak_usb`
//! driver, or a board's own controller (`mttcan` on NVIDIA Jetson and Thor, `rockchip_canfd`
//! on Rockchip, `m_can`, `flexcan`). It is not any motor vendor's, so it lives beside the
//! transport-neutral trait rather than in a vendor crate; `sx-zlg-usbcanfd` (the ZLG adapter
//! in userspace, which has no kernel driver) and `sx-damiao-can`'s `usb2can` are the other
//! transports, and the same Damiao chain runs over each.
//!
//! `SocketCanPort` (Linux only) opens one raw socket (`AF_CAN`, `SOCK_RAW`, `CAN_RAW`),
//! non-blocking, bound to one interface, holding that interface's [`InterfaceLock`] so a second
//! of our processes cannot open it too. The socket reads CAN FD as well as classic frames
//! (`CAN_RAW_FD_FRAMES`), receives the controller's error frames (`CAN_RAW_ERR_FILTER`), and
//! asks for kernel and hardware receive timestamps (`SO_TIMESTAMPING`) and the socket's drop
//! count (`SO_RXQ_OVFL`). Receives drain the socket with one `recvmmsg` into buffers
//! allocated at open; `receive_until` sleeps in `ppoll` until a frame or the deadline, never
//! spinning. The interface's bitrate and state are the operating system's: it must already be
//! up (`ip link set can1 up type can bitrate 1000000 restart-ms 100`), a host need the robot
//! declares. The frame layout ([`encode`], [`decode`]) and error-frame meaning ([`BusEvent`])
//! are the kernel's `struct can_frame` / `struct canfd_frame` and `linux/can/error.h`, kept
//! portable so they are tested on every host.

use std::io;
use std::path::PathBuf;

use sx_embodiment_drivers::Holder;
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

/// The bitrate a robot's CAN bus runs at unless its commissioning says otherwise.
pub const BITRATE: u32 = 1_000_000;
/// How soon a bus-off controller restarts by itself.
pub const RESTART_MS: u32 = 100;
/// The transmit queue a chain's burst of one frame per motor fits (the kernel's default is 10).
pub const TXQUEUE_LEN: u32 = 64;

/// What the host must provide to open `interface` as it was measured
/// (`sx_embodiment_drivers::host_needs`): the interface up with its bitrate, an automatic
/// bus-off restart and a queue a chain's burst fits; `can_raw`; `SCHED_FIFO` for the control
/// loop and the CPU wake-latency hold the open port keeps; and for a USB adapter
/// (`usb = Some((vendor, product))`), its autosuspend off.
#[must_use]
pub fn host_needs(
    interface: &str,
    usb: Option<(u16, u16)>,
) -> Vec<sx_embodiment_drivers::HostNeed> {
    use sx_embodiment_drivers::HostNeed;
    let mut needs = vec![
        HostNeed::CanInterface {
            interface: interface.to_owned(),
            bitrate: BITRATE,
            restart_ms: RESTART_MS,
            txqueue_len: TXQUEUE_LEN,
        },
        HostNeed::KernelModule {
            name: "can_raw".to_owned(),
        },
        HostNeed::RealtimePriority {
            priority: u8::try_from(sx_realtime::ANSWER_PATH_PRIORITY).unwrap_or(u8::MAX),
        },
        HostNeed::CpuWakeLatency,
    ];
    if let Some((vendor_id, product_id)) = usb {
        needs.push(HostNeed::UsbAutosuspendOff {
            vendor_id,
            product_id,
        });
    }
    needs
}

/// Why an interface could not be opened.
#[derive(Debug, Error)]
pub enum SocketCanError {
    #[error("{0:?} is not a network interface name")]
    Name(String),
    /// Another of our processes holds the interface's lock. The kernel itself lets any number
    /// of sockets bind one interface, so only processes taking this lock are excluded.
    #[error("SocketCAN interface {interface} is already open in another process, {holder} ({})", lock.display())]
    Busy {
        interface: String,
        lock: PathBuf,
        holder: Holder,
    },
    #[error("SocketCAN interface {interface}: {source}")]
    Io {
        interface: String,
        source: io::Error,
    },
}

/// A send the link refused, carried inside the `io::Error` (`LinkError::of`). Its kinds are the
/// contract every transport keeps (`sx-zlg-usbcanfd`'s `LinkError` too): `WouldBlock`, send
/// again once the bus drains; `NotConnected`, the link needs action outside this process.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum LinkError {
    /// `WouldBlock`: the interface's transmit queue stayed full for the send's whole patience.
    #[error(
        "SocketCAN {interface}: the transmit queue stayed full for {waited_us} µs; the frame was not sent, send it once the bus has drained"
    )]
    QueueFull { interface: String, waited_us: u64 },
    /// `WouldBlock`: the transmit queue stayed full and the controller last reported bus-off;
    /// it restarts on its own only if the interface has `restart-ms` set.
    #[error(
        "SocketCAN {interface}: the controller is bus-off (too many transmit errors: wiring, termination, bitrate or no node acknowledging); the frame was not sent; it restarts only if the interface has restart-ms set"
    )]
    BusOff { interface: String },
    /// `NotConnected`: the interface is down (`ENETDOWN`).
    #[error("SocketCAN {interface}: the interface is down; bring it up with its bitrate")]
    Down { interface: String },
}

impl LinkError {
    #[must_use]
    pub const fn kind(&self) -> io::ErrorKind {
        match self {
            Self::QueueFull { .. } | Self::BusOff { .. } => io::ErrorKind::WouldBlock,
            Self::Down { .. } => io::ErrorKind::NotConnected,
        }
    }

    /// The link error inside `error`, if it carries one.
    #[must_use]
    pub fn of(error: &io::Error) -> Option<&Self> {
        error.get_ref()?.downcast_ref()
    }
}

impl From<LinkError> for io::Error {
    fn from(error: LinkError) -> Self {
        Self::new(error.kind(), error)
    }
}

/// The size of the kernel's `struct can_frame`: id, length, three reserved bytes, data.
pub const FRAME_BYTES: usize = 16;
/// The size of the kernel's `struct canfd_frame`: id, length, flags, two reserved, 64 bytes.
pub const FD_FRAME_BYTES: usize = 72;

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

/// What one read from a raw CAN socket holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Received {
    /// A data frame of at most eight bytes, classic or FD; missing bytes read as zero.
    Data(CanFrame),
    /// A CAN FD frame longer than eight bytes, which a classic port reads past.
    Long,
    /// A remote request, which is no answer.
    Remote,
    /// The controller's error frame.
    Error(BusEvent),
}

/// The frame in `bytes`, a kernel `struct can_frame` (16 bytes) or `struct canfd_frame` (72).
/// `None` for any other length.
#[must_use]
pub fn decode(bytes: &[u8]) -> Option<Received> {
    if bytes.len() != FRAME_BYTES && bytes.len() != FD_FRAME_BYTES {
        return None;
    }
    let raw = u32::from_ne_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    let length = usize::from(bytes[4]);
    let mut data = [0; 8];
    data[..length.min(8)].copy_from_slice(&bytes[8..8 + length.min(8)]);
    if raw & ERROR != 0 {
        return Some(Received::Error(BusEvent::of(raw & EXTENDED_MASK, &data)));
    }
    if raw & REMOTE != 0 {
        return Some(Received::Remote);
    }
    if length > 8 {
        return Some(Received::Long);
    }
    let id = if raw & EXTENDED == 0 {
        raw & STANDARD_MASK
    } else {
        raw & EXTENDED_MASK
    };
    Some(Received::Data(CanFrame { id, data }))
}

/// One error frame's meaning (`linux/can/error.h`): the classes it reports, and the
/// controller's state and error counters when it carries them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)]
pub struct BusEvent {
    /// The controller left the bus (`CAN_ERR_BUSOFF`); nothing is sent until it restarts.
    pub bus_off: bool,
    /// The controller restarted after bus-off (`CAN_ERR_RESTARTED`).
    pub restarted: bool,
    /// No node acknowledged a transmitted frame (`CAN_ERR_ACK`): nothing else on the bus, or
    /// a bitrate mismatch.
    pub no_ack: bool,
    /// A protocol or bus error (`CAN_ERR_PROT`, `CAN_ERR_BUSERROR`): bit, stuff, form or CRC.
    pub bus_error: bool,
    /// A transmit timed out (`CAN_ERR_TX_TIMEOUT`).
    pub tx_timeout: bool,
    /// The controller's receive or transmit buffer overflowed (`CAN_ERR_CRTL_*_OVERFLOW`).
    pub overflow: bool,
    /// The controller reached error-warning (`CAN_ERR_CRTL_*_WARNING`).
    pub warning: bool,
    /// The controller reached error-passive (`CAN_ERR_CRTL_*_PASSIVE`).
    pub passive: bool,
    /// The controller is back to error-active (`CAN_ERR_CRTL_ACTIVE`).
    pub active: bool,
    /// Transmit and receive error counters (`CAN_ERR_CNT`).
    pub counters: Option<(u8, u8)>,
}

impl BusEvent {
    /// The event an error frame with class bits `class` and payload `data` reports.
    #[must_use]
    pub fn of(class: u32, data: &[u8; 8]) -> Self {
        const TX_TIMEOUT: u32 = 0x001;
        const CRTL: u32 = 0x004;
        const PROT: u32 = 0x008;
        const ACK: u32 = 0x020;
        const BUSOFF: u32 = 0x040;
        const BUSERROR: u32 = 0x080;
        const RESTARTED: u32 = 0x100;
        const CNT: u32 = 0x200;
        let controller = if class & CRTL == 0 { 0 } else { data[1] };
        Self {
            bus_off: class & BUSOFF != 0,
            restarted: class & RESTARTED != 0,
            no_ack: class & ACK != 0,
            bus_error: class & (PROT | BUSERROR) != 0,
            tx_timeout: class & TX_TIMEOUT != 0,
            overflow: controller & 0x03 != 0,
            warning: controller & 0x0C != 0,
            passive: controller & 0x30 != 0,
            active: controller & 0x40 != 0,
            counters: (class & CNT != 0).then_some((data[6], data[7])),
        }
    }
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
        assert_eq!(decode(&bytes), Some(Received::Data(frame)));
        let extended = CanFrame {
            id: 0x1234_5678,
            data: [1; 8],
        };
        let bytes = encode(&extended).expect("extended id");
        assert_ne!(
            u32::from_ne_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]) & EXTENDED,
            0
        );
        assert_eq!(decode(&bytes), Some(Received::Data(extended)));
        assert!(
            encode(&CanFrame {
                id: 0x2000_0000,
                data: [0; 8]
            })
            .is_err()
        );
    }

    #[test]
    fn remote_long_fd_and_short_frames_are_told_apart() {
        let mut bytes = [0xAA; FRAME_BYTES];
        bytes[..4].copy_from_slice(&(0x29 | REMOTE).to_ne_bytes());
        assert_eq!(decode(&bytes), Some(Received::Remote));
        bytes[..4].copy_from_slice(&0x29_u32.to_ne_bytes());
        bytes[4] = 2;
        assert_eq!(
            decode(&bytes),
            Some(Received::Data(CanFrame {
                id: 0x29,
                data: [0xAA, 0xAA, 0, 0, 0, 0, 0, 0]
            }))
        );
        let mut fd = [0x55; FD_FRAME_BYTES];
        fd[..4].copy_from_slice(&0x29_u32.to_ne_bytes());
        fd[4] = 8;
        assert_eq!(
            decode(&fd),
            Some(Received::Data(CanFrame {
                id: 0x29,
                data: [0x55; 8]
            }))
        );
        fd[4] = 12;
        assert_eq!(decode(&fd), Some(Received::Long));
        assert_eq!(decode(&fd[..20]), None);
    }

    #[test]
    fn error_frames_say_what_the_controller_saw() {
        let mut bytes = [0; FRAME_BYTES];
        bytes[..4].copy_from_slice(&(ERROR | 0x040 | 0x004 | 0x200).to_ne_bytes());
        bytes[4] = 8;
        bytes[8 + 1] = 0x20;
        bytes[8 + 6] = 255;
        bytes[8 + 7] = 3;
        let Some(Received::Error(event)) = decode(&bytes) else {
            panic!("an error frame");
        };
        assert!(event.bus_off && event.passive && !event.no_ack && !event.restarted);
        assert_eq!(event.counters, Some((255, 3)));
        bytes[..4].copy_from_slice(&(ERROR | 0x020 | 0x100).to_ne_bytes());
        let Some(Received::Error(event)) = decode(&bytes) else {
            panic!("an error frame");
        };
        assert!(event.no_ack && event.restarted && !event.bus_off && !event.passive);
    }
}
