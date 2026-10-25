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

pub use device::{Channel, ChannelOptions, Found, Options, Probe, UsbCanFd, ZlgError, list};
pub use engine::{ChannelStats, DeviceStats};
pub use timing::Bitrate;
