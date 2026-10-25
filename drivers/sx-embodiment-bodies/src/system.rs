//! Every bridge this host carries, whatever its transport, found by its commissioned identity.
//!
//! A station names no transport. Commissioning records each port's bridge by an identity, a USB
//! serial number or a board CAN interface's name, and [`SystemPorts`] looks that identity up in
//! every transport compiled in: Damiao USB2CAN serial bridges ([`crate::usb_serial`], feature
//! `usb-serial`) and `SocketCAN` interfaces ([`crate::socketcan`], feature `socketcan`). Exactly
//! one bridge carrying the identity is opened. None is [`OpenError::NotAttached`]; two, on one
//! transport or across both, is [`OpenError::Ambiguous`], naming each, and nothing is opened.
//! A serial-servo bus is carried by a USB serial adapter alone, so its identity is looked up
//! the same way and refused unless it is that adapter's tty.
//! The module builds when either transport does.

#[cfg(feature = "usb-serial")]
use std::path::PathBuf;

#[cfg(feature = "socketcan")]
use crate::socketcan::SocketCanPorts;
#[cfg(feature = "usb-serial")]
use crate::usb_serial::{self, UsbSerialPorts};
use crate::{OpenError, Port, PortSource, SerialLink};

/// One bridge an identity resolved to.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Bridge {
    /// A USB serial bridge's tty: a Damiao USB2CAN bridge, or a serial-servo adapter.
    #[cfg(feature = "usb-serial")]
    Usb2Can(PathBuf),
    /// A `SocketCAN` interface.
    #[cfg(feature = "socketcan")]
    SocketCan(String),
}

impl std::fmt::Display for Bridge {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            #[cfg(feature = "usb-serial")]
            Self::Usb2Can(tty) => write!(formatter, "USB2CAN {}", tty.display()),
            #[cfg(feature = "socketcan")]
            Self::SocketCan(interface) => write!(formatter, "SocketCAN {interface}"),
        }
    }
}

/// The bridges of every transport compiled in.
pub struct SystemPorts {
    #[cfg(feature = "usb-serial")]
    usb_serial: UsbSerialPorts,
    #[cfg(feature = "socketcan")]
    socketcan: SocketCanPorts,
}

impl SystemPorts {
    /// This host's kernel view, for every transport compiled in.
    #[must_use]
    pub fn system() -> Self {
        Self {
            #[cfg(feature = "usb-serial")]
            usb_serial: UsbSerialPorts::system(),
            #[cfg(feature = "socketcan")]
            socketcan: SocketCanPorts::system(),
        }
    }

    /// The one bridge carrying `identity`.
    ///
    /// # Errors
    ///
    /// Returns [`OpenError::NotAttached`] when no transport carries it, [`OpenError::Ambiguous`]
    /// when more than one bridge does, and a transport's own error when its view cannot be read.
    pub fn bridge(&self, identity: &str) -> Result<Bridge, OpenError> {
        let mut found: Vec<Bridge> = Vec::new();
        #[cfg(feature = "usb-serial")]
        found.extend(attached(self.usb_serial.tty(identity))?.map(Bridge::Usb2Can));
        #[cfg(feature = "socketcan")]
        found.extend(attached(self.socketcan.interface(identity))?.map(Bridge::SocketCan));
        let mut found = found.into_iter();
        match (found.next(), found.next()) {
            (None, _) => Err(OpenError::NotAttached(identity.to_owned())),
            (Some(bridge), None) => Ok(bridge),
            (Some(first), Some(second)) => Err(OpenError::Ambiguous {
                identity: identity.to_owned(),
                bridges: [first, second]
                    .into_iter()
                    .chain(found)
                    .map(|bridge| bridge.to_string())
                    .collect(),
            }),
        }
    }
}

/// A transport's answer, with "not on this transport" as `None`.
fn attached<T>(found: Result<T, OpenError>) -> Result<Option<T>, OpenError> {
    match found {
        Ok(bridge) => Ok(Some(bridge)),
        Err(OpenError::NotAttached(_)) => Ok(None),
        Err(error) => Err(error),
    }
}

impl PortSource for SystemPorts {
    fn open_serial(&mut self, serial: &str, baud: u32) -> Result<SerialLink, OpenError> {
        match self.bridge(serial)? {
            #[cfg(feature = "usb-serial")]
            Bridge::Usb2Can(tty) => usb_serial::open_servo_tty(&tty, serial, baud),
            #[cfg(feature = "socketcan")]
            Bridge::SocketCan(interface) => Err(OpenError::Port {
                serial: serial.to_owned(),
                reason: format!(
                    "{interface} is a SocketCAN interface, not a serial bus at {baud} baud"
                ),
            }),
        }
    }

    fn open(&mut self, serial: &str) -> Result<Port, OpenError> {
        match self.bridge(serial)? {
            #[cfg(feature = "usb-serial")]
            Bridge::Usb2Can(tty) => usb_serial::open_tty(&tty, serial),
            #[cfg(all(feature = "socketcan", target_os = "linux"))]
            Bridge::SocketCan(interface) => sx_socketcan::SocketCanPort::open(&interface)
                .map(|port| Box::new(port) as Port)
                .map_err(|error| OpenError::Port {
                    serial: serial.to_owned(),
                    reason: error.to_string(),
                }),
            #[cfg(all(feature = "socketcan", not(target_os = "linux")))]
            Bridge::SocketCan(interface) => Err(OpenError::Port {
                serial: serial.to_owned(),
                reason: format!("{interface}: SocketCAN opens only on Linux"),
            }),
        }
    }
}

#[cfg(all(test, feature = "usb-serial", feature = "socketcan"))]
mod tests {
    use std::fs;
    use std::path::Path;

    use super::*;

    /// A USB device reporting `serial`, with a tty interface when it is a serial bridge.
    fn usb(root: &Path, name: &str, serial: &str, tty: Option<&str>) -> PathBuf {
        let device = root.join("usb").join(name);
        let interface = device.join(format!("{name}:1.0"));
        fs::create_dir_all(&interface).expect("interface");
        fs::write(device.join("idVendor"), "2e88\n").expect("vendor");
        fs::write(device.join("serial"), format!("{serial}\n")).expect("serial");
        if let Some(tty) = tty {
            fs::create_dir_all(interface.join("tty").join(tty)).expect("tty");
        }
        interface
    }

    /// A CAN interface whose device is `device`.
    fn can(root: &Path, name: &str, device: &Path) {
        let net = root.join("net").join(name);
        fs::create_dir_all(&net).expect("net");
        fs::write(net.join("type"), "280\n").expect("type");
        std::os::unix::fs::symlink(device, net.join("device")).expect("device link");
    }

    #[test]
    fn an_identity_opens_the_one_bridge_carrying_it_on_any_transport() {
        let root = std::env::temp_dir().join(format!(
            "sx-system-ports-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |elapsed| elapsed.as_nanos())
        ));
        usb(&root, "1-1", "B601-ARM", Some("ttyACM0"));
        let adapter = usb(&root, "1-2", "UB-CAN", None);
        can(&root, "can1", &adapter);
        let board = root.join("platform/can0");
        fs::create_dir_all(&board).expect("board");
        can(&root, "can0", &board);
        let mut ports = SystemPorts {
            usb_serial: UsbSerialPorts::rooted(root.join("usb"), PathBuf::from("/dev")),
            socketcan: SocketCanPorts::rooted(root.join("net")),
        };
        assert_eq!(
            ports.bridge("B601-ARM").expect("serial bridge"),
            Bridge::Usb2Can(PathBuf::from("/dev/ttyACM0"))
        );
        // A gs_usb adapter is a USB device with a serial but no tty: SocketCAN alone has it.
        assert_eq!(
            ports.bridge("UB-CAN").expect("adapter"),
            Bridge::SocketCan("can1".to_owned())
        );
        assert_eq!(
            ports.bridge("can0").expect("board controller"),
            Bridge::SocketCan("can0".to_owned())
        );
        assert!(matches!(
            ports.bridge("absent"),
            Err(OpenError::NotAttached(identity)) if identity == "absent"
        ));
        // A serial bridge and a CAN adapter reporting one serial: neither is opened.
        usb(&root, "1-3", "TWIN", Some("ttyACM1"));
        let twin = usb(&root, "1-4", "TWIN", None);
        can(&root, "can2", &twin);
        match ports.bridge("TWIN") {
            Err(OpenError::Ambiguous { identity, bridges }) => {
                assert_eq!(identity, "TWIN");
                assert_eq!(bridges.len(), 2);
            }
            other => panic!("expected an ambiguity, got {other:?}"),
        }
        // A serial-servo bus is refused on an ambiguous identity and on a CAN interface.
        assert!(matches!(
            ports.open_serial("TWIN", 1_000_000),
            Err(OpenError::Ambiguous { .. })
        ));
        assert!(matches!(
            ports.open_serial("UB-CAN", 1_000_000),
            Err(OpenError::Port { serial, .. }) if serial == "UB-CAN"
        ));
        let _ = fs::remove_dir_all(root);
    }
}
