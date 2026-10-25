//! Every USB device a host carries, as a bridge to a bus, a camera or neither.
//!
//! A device is classed by what the host offers for it, then by the few devices a robot is
//! known to carry: a tty makes it a serial bus, a `SocketCAN` interface on its serial makes it a
//! CAN bus, a video-class interface (UVC) a camera, a HID interface an input. A known CAN bridge
//! with no interface is present but unusable here: the ZLG USBCANFD-200U becomes `SocketCAN`
//! only under ZLG's own Linux driver, and has none on macOS.

use serde::Serialize;

use crate::host::{CanInterface, Host, Tty, UsbDevice};

/// `bDeviceClass` of a hub; hubs carry nothing a robot is made of and are not listed.
const HUB: u8 = 0x09;
const HID: u8 = 0x03;
const VIDEO: u8 = 0x0E;

/// What a device is to a robot.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum BridgeKind {
    SerialBus,
    CanBus,
    Camera,
    Input,
    Other,
}

/// Whether, and through what, this host can open a device's bus.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case", tag = "kind")]
pub enum Access {
    /// Its tty, opened by a serial scan.
    Tty { path: String },
    /// Its `SocketCAN` interfaces, one per channel.
    SocketCan { interfaces: Vec<String> },
    /// Attached, but this host has no driver that makes it a bus.
    NoDriver { reason: String },
    /// Not a bus.
    None,
}

/// A device the robot may be carrying.
#[derive(Clone, Debug, Eq, PartialEq, Serialize)]
pub struct Bridge {
    pub usb: UsbDevice,
    pub kind: BridgeKind,
    /// The device's name, from the known table or its own product string.
    pub name: String,
    pub access: Access,
}

impl Bridge {
    /// The identity commissioning records it by: its USB serial number.
    #[must_use]
    pub fn identity(&self) -> Option<&str> {
        self.usb.serial.as_deref()
    }

    /// Whether a Feetech scan may be sent to it: a tty on a USB-serial chip a servo bus adapter
    /// is built on. Another device's tty (a Damiao USB2CAN, a modem) is never written to.
    #[must_use]
    pub fn carries_servo_bus(&self) -> bool {
        matches!(self.access, Access::Tty { .. }) && SERVO_BUS_CHIPS.contains(&self.usb.vendor_id)
    }
}

/// The USB-serial chip makers servo bus adapters use: WCH (CH340, CH343), FTDI (U2D2), Silicon
/// Labs (`CP210x`).
const SERVO_BUS_CHIPS: [u16; 3] = [0x1a86, 0x0403, 0x10c4];

/// Devices a robot is known to carry, by vendor and product id.
const KNOWN: [(u16, u16, &str, BridgeKind); 12] = [
    (
        0x1a86,
        0x55d3,
        "WCH CH343 USB serial",
        BridgeKind::SerialBus,
    ),
    (
        0x1a86,
        0x55d4,
        "WCH CH9102 USB serial",
        BridgeKind::SerialBus,
    ),
    (
        0x1a86,
        0x7523,
        "WCH CH340 USB serial",
        BridgeKind::SerialBus,
    ),
    (
        0x0403,
        0x6014,
        "FTDI FT232H (Dynamixel U2D2)",
        BridgeKind::SerialBus,
    ),
    (
        0x0403,
        0x6001,
        "FTDI FT232R USB serial",
        BridgeKind::SerialBus,
    ),
    (
        0x10c4,
        0xea60,
        "Silicon Labs CP210x USB serial",
        BridgeKind::SerialBus,
    ),
    (0x3068, 0x0009, "ZLG USBCANFD-200U", BridgeKind::CanBus),
    (
        0x1d50,
        0x606f,
        "candleLight (gs_usb) CAN",
        BridgeKind::CanBus,
    ),
    (0x2b03, 0xf880, "Stereolabs ZED 2i", BridgeKind::Camera),
    (
        0x2b03,
        0xf881,
        "Stereolabs ZED 2i sensors",
        BridgeKind::Other,
    ),
    (0x3553, 0xb001, "PCsensor FootSwitch", BridgeKind::Input),
    (0x8086, 0x0b3a, "Intel RealSense D435i", BridgeKind::Camera),
];

fn known(usb: &UsbDevice) -> Option<(&'static str, BridgeKind)> {
    KNOWN
        .iter()
        .find(|(vendor, product, _, _)| *vendor == usb.vendor_id && *product == usb.product_id)
        .map(|&(_, _, name, kind)| (name, kind))
}

/// Why a known CAN bridge without an interface cannot be opened here.
fn no_can_driver(usb: &UsbDevice) -> String {
    let driver = if usb.vendor_id == 0x3068 {
        "ZLG's own driver"
    } else {
        "a SocketCAN driver"
    };
    if cfg!(target_os = "linux") {
        format!("no SocketCAN interface: {driver} is not loaded")
    } else {
        format!("SocketCAN is Linux only; on Linux it needs {driver}")
    }
}

/// Class one device from what the host offers for it.
#[must_use]
pub fn classify(usb: UsbDevice, ttys: &[Tty], can: &[CanInterface]) -> Bridge {
    let same = |vendor: u16, product: u16, serial: &Option<String>| {
        vendor == usb.vendor_id && product == usb.product_id && *serial == usb.serial
    };
    let tty = ttys
        .iter()
        .find(|tty| same(tty.vendor_id, tty.product_id, &tty.serial));
    let interfaces: Vec<String> = can
        .iter()
        .filter(|interface| usb.serial.is_some() && interface.usb_serial == usb.serial)
        .map(|interface| interface.name.clone())
        .collect();
    let (known_name, known_kind) = known(&usb).map_or((None, None), |(n, k)| (Some(n), Some(k)));
    let kind = if tty.is_some() {
        BridgeKind::SerialBus
    } else if !interfaces.is_empty() {
        BridgeKind::CanBus
    } else if let Some(kind) = known_kind {
        kind
    } else if usb.interface_classes.contains(&VIDEO) {
        BridgeKind::Camera
    } else if usb.interface_classes.contains(&HID) {
        BridgeKind::Input
    } else {
        BridgeKind::Other
    };
    let access = match (tty, kind) {
        (Some(tty), _) => Access::Tty {
            path: tty.path.clone(),
        },
        (None, BridgeKind::CanBus) if !interfaces.is_empty() => Access::SocketCan { interfaces },
        (None, BridgeKind::CanBus) => Access::NoDriver {
            reason: no_can_driver(&usb),
        },
        (None, BridgeKind::SerialBus) => Access::NoDriver {
            reason: "no serial device node".to_owned(),
        },
        _ => Access::None,
    };
    let name = known_name
        .map(str::to_owned)
        .or_else(|| usb.product.clone())
        .unwrap_or_else(|| format!("{:04x}:{:04x}", usb.vendor_id, usb.product_id));
    Bridge {
        usb,
        kind,
        name,
        access,
    }
}

/// Every non-hub USB device on `host`, classed, sorted by vendor, product and serial.
///
/// # Errors
///
/// Returns the host's failure to enumerate.
pub fn enumerate<H: Host>(host: &H) -> std::io::Result<Vec<Bridge>> {
    let ttys = host.ttys()?;
    let can = host.can_interfaces()?;
    let mut bridges: Vec<Bridge> = host
        .usb_devices()?
        .into_iter()
        .filter(|usb| usb.device_class != HUB)
        .map(|usb| classify(usb, &ttys, &can))
        .collect();
    bridges.sort_by(|a, b| {
        (a.usb.vendor_id, a.usb.product_id, &a.usb.serial).cmp(&(
            b.usb.vendor_id,
            b.usb.product_id,
            &b.usb.serial,
        ))
    });
    Ok(bridges)
}
