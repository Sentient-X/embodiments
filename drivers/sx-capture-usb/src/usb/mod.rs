//! USB below the rig: descriptors (pure) and the usbfs transport (Linux and Android).
//!
//! Android forbids an app `/dev/video*` and `/dev/ttyACM*`; what it grants is a
//! `UsbDeviceConnection`, whose file descriptor is an open usbfs node
//! (`/dev/bus/usb/BBB/DDD`). Everything the rig does to a device goes through that one fd
//! as usbfs ioctls, so the same code runs on the headset and on a Linux laptop.

pub mod descriptor;
#[cfg(test)]
pub mod fixtures;
#[cfg(any(target_os = "linux", target_os = "android"))]
pub mod usbfs;

use std::io;

/// bmRequestType for class requests to an interface, host-to-device and device-to-host.
pub const CLASS_INTERFACE_OUT: u8 = 0x21;
pub const CLASS_INTERFACE_IN: u8 = 0xa1;
/// bmRequestType for standard requests to the device, device-to-host.
pub const STANDARD_DEVICE_IN: u8 = 0x80;
pub const REQUEST_GET_DESCRIPTOR: u8 = 0x06;
/// The en-US language id string descriptors are read in.
pub const LANG_EN_US: u16 = 0x0409;

/// The control pipe of one device. The usbfs device implements it; tests replace it.
pub trait Control {
    /// Device-to-host: fill `data`, return the bytes the device sent.
    fn control_in(
        &self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        data: &mut [u8],
    ) -> io::Result<usize>;
    /// Host-to-device: send `data`.
    fn control_out(
        &self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> io::Result<()>;

    /// A string descriptor in en-US; index 0 is "no string".
    fn string_descriptor(&self, index: u8) -> io::Result<String> {
        if index == 0 {
            return Err(io::Error::new(
                io::ErrorKind::NotFound,
                "the device declares no such string",
            ));
        }
        let mut buffer = [0_u8; 255];
        let read = self.control_in(
            STANDARD_DEVICE_IN,
            REQUEST_GET_DESCRIPTOR,
            (0x03 << 8) | u16::from(index),
            LANG_EN_US,
            &mut buffer,
        )?;
        descriptor::string(&buffer[..read])
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error))
    }
}
