//! ZLG USBCANFD-200U/400U CAN-FD adapters, driven in userspace over USB.
//!
//! Implemented clean-room from `docs/PROTOCOL.md` beside this crate, a functional
//! specification of the device's USB protocol; no vendor source was read. [`wire`] is that
//! protocol as pure functions; [`engine`] the data path (queued receive reads, batched
//! transmits with the reference's one-outstanding flow control, preallocated rings) over an
//! endpoint abstraction; [`usb`] its binding to `nusb`; [`device`] the adapter and its channels,
//! each a `sx_embodiment_drivers::can::CanPort`.

pub mod bench;
pub mod crc;
pub mod device;
pub mod engine;
#[cfg(target_os = "macos")]
pub mod macos;
pub mod ring;
pub mod timing;
pub mod usb;
pub mod wire;

pub use device::{
    Channel, ChannelOptions, Found, LinkError, Options, Probe, ReceiveWait, Recovery, UsbCanFd,
    ZlgError, list,
};
pub use engine::{ChannelStats, DeviceStats, Stall, TxBudget, TxFailure};
pub use timing::Bitrate;

use sx_embodiment_drivers::HostNeed;

/// What the host must provide to open the adapter `vendor_id:product_id` with USB serial
/// `serial` as it was measured (`sx_embodiment_drivers::host_needs`): its usbfs node for the
/// station's group, USB autosuspend off, `SCHED_FIFO` for the I/O and `nusb-events` threads,
/// and the CPU wake-latency hold the open adapter keeps.
#[must_use]
pub fn host_needs(vendor_id: u16, product_id: u16, serial: &str) -> Vec<HostNeed> {
    vec![
        HostNeed::UsbDevice {
            vendor_id,
            product_id,
            serial: Some(serial.to_owned()),
        },
        HostNeed::UsbAutosuspendOff {
            vendor_id,
            product_id,
        },
        HostNeed::RealtimePriority {
            priority: u8::try_from(sx_realtime::ANSWER_PATH_PRIORITY).unwrap_or(u8::MAX),
        },
        HostNeed::CpuWakeLatency,
    ]
}
