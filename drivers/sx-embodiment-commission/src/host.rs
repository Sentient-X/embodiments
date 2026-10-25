//! What a host reports about its devices, and how its buses are opened.
//!
//! [`Host`] is the seam: [`System`] is this machine (USB through `nusb`, ttys through
//! `serialport`, `SocketCAN` through `sysfs` and `sx-socketcan` on Linux); a suite supplies a
//! fake one replaying servos and motors.

use std::io;
use std::path::PathBuf;

use serde::Serialize;
use sx_embodiment_drivers::can::CanPort;
use sx_embodiment_drivers::serial::SerialPort;

use crate::sysfs;
use crate::tty::TtyPort;

/// One USB device as its descriptors report it.
#[derive(Clone, Debug, Default, Eq, PartialEq, Serialize)]
pub struct UsbDevice {
    pub vendor_id: u16,
    pub product_id: u16,
    pub serial: Option<String>,
    pub manufacturer: Option<String>,
    pub product: Option<String>,
    pub device_class: u8,
    /// Each interface's class (`0x02` CDC, `0x03` HID, `0x0E` video), in interface order.
    pub interface_classes: Vec<u8>,
}

/// One serial device node the host offers, with the USB device behind it.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct Tty {
    pub path: String,
    pub vendor_id: u16,
    pub product_id: u16,
    pub serial: Option<String>,
}

/// One `SocketCAN` interface and the serial number of the USB device it sits in.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct CanInterface {
    pub name: String,
    pub usb_serial: Option<String>,
}

/// A host's devices, and its buses opened.
pub trait Host {
    type Serial: SerialPort;
    type Can: CanPort;

    /// Every USB device attached.
    ///
    /// # Errors
    ///
    /// Returns the host's failure to enumerate.
    fn usb_devices(&self) -> io::Result<Vec<UsbDevice>>;

    /// Every USB serial device node.
    ///
    /// # Errors
    ///
    /// Returns the host's failure to enumerate.
    fn ttys(&self) -> io::Result<Vec<Tty>>;

    /// Every `SocketCAN` interface; none off Linux.
    ///
    /// # Errors
    ///
    /// Returns the host's failure to read its kernel view.
    fn can_interfaces(&self) -> io::Result<Vec<CanInterface>>;

    /// Open `tty` at `baud`.
    ///
    /// # Errors
    ///
    /// Returns why it cannot be opened.
    fn open_serial(&mut self, tty: &str, baud: u32) -> io::Result<Self::Serial>;

    /// Open the `SocketCAN` interface `name`.
    ///
    /// # Errors
    ///
    /// Returns why it cannot be opened, always off Linux.
    fn open_can(&mut self, name: &str) -> io::Result<Self::Can>;
}

/// This machine.
pub struct System {
    net: PathBuf,
    by_id: PathBuf,
}

impl Default for System {
    fn default() -> Self {
        Self {
            net: PathBuf::from("/sys/class/net"),
            by_id: PathBuf::from("/dev/serial/by-id"),
        }
    }
}

#[cfg(target_os = "linux")]
type SystemCan = sx_socketcan::SocketCanPort;
#[cfg(not(target_os = "linux"))]
type SystemCan = NoCan;

/// The CAN port of a host without `SocketCAN`: it is never opened.
#[cfg(not(target_os = "linux"))]
pub enum NoCan {}

#[cfg(not(target_os = "linux"))]
impl CanPort for NoCan {
    fn send(&mut self, _frame: &sx_embodiment_drivers::can::CanFrame) -> io::Result<()> {
        match *self {}
    }

    fn receive(&mut self) -> io::Result<Option<sx_embodiment_drivers::can::CanFrame>> {
        match *self {}
    }
}

impl Host for System {
    type Serial = TtyPort;
    type Can = SystemCan;

    fn usb_devices(&self) -> io::Result<Vec<UsbDevice>> {
        use nusb::MaybeFuture as _;
        Ok(nusb::list_devices()
            .wait()
            .map_err(io::Error::other)?
            .map(|device| UsbDevice {
                vendor_id: device.vendor_id(),
                product_id: device.product_id(),
                serial: device.serial_number().map(str::to_owned),
                manufacturer: device.manufacturer_string().map(str::to_owned),
                product: device.product_string().map(str::to_owned),
                device_class: device.class(),
                interface_classes: device
                    .interfaces()
                    .map(nusb::InterfaceInfo::class)
                    .collect(),
            })
            .collect())
    }

    fn ttys(&self) -> io::Result<Vec<Tty>> {
        let ports = serialport::available_ports().map_err(io::Error::other)?;
        let aliases = sysfs::by_id(&self.by_id);
        Ok(ports
            .into_iter()
            .filter_map(|port| match port.port_type {
                serialport::SerialPortType::UsbPort(usb) => Some(Tty {
                    path: aliases
                        .get(&port.port_name)
                        .cloned()
                        .unwrap_or(port.port_name),
                    vendor_id: usb.vid,
                    product_id: usb.pid,
                    serial: usb.serial_number,
                }),
                _ => None,
            })
            // macOS lists each device twice; the call-out node is the one a host opens.
            .filter(|tty| !tty.path.starts_with("/dev/tty."))
            .collect())
    }

    fn can_interfaces(&self) -> io::Result<Vec<CanInterface>> {
        sysfs::can_interfaces(&self.net)
    }

    fn open_serial(&mut self, tty: &str, baud: u32) -> io::Result<TtyPort> {
        TtyPort::open(tty, baud)
    }

    #[cfg(target_os = "linux")]
    fn open_can(&mut self, name: &str) -> io::Result<SystemCan> {
        sx_socketcan::SocketCanPort::open(name)
    }

    #[cfg(not(target_os = "linux"))]
    fn open_can(&mut self, name: &str) -> io::Result<SystemCan> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            format!("SocketCAN interface {name} exists only on Linux"),
        ))
    }
}
