//! Commissioned `SocketCAN` interfaces on Linux, found by their adapter's USB serial number.
//!
//! As with the USB serial bridges, a bus is commissioned by a kernel fact that does not depend
//! on enumeration order: `can0` and `can1` name whichever adapter the kernel found first. Each
//! CAN interface under `/sys/class/net` (link type 280, `ARPHRD_CAN`) has one identity: the
//! `serial` of the USB device its `device` link sits in, for a USB adapter (`gs_usb`,
//! `peak_usb`), or else its own name, for a controller on the board itself, which has no
//! serial and which the device tree names the same on every boot. Commissioning records that
//! identity, and the one interface carrying it is opened as a [`SocketCanPort`].
//!
//! The interface's bitrate and up state are the operating system's, set before the station
//! starts; an interface that is down opens and then fails its first send.

use std::fs;
use std::path::{Path, PathBuf};

#[cfg(target_os = "linux")]
use sx_socketcan::SocketCanPort;

use crate::OpenError;
#[cfg(target_os = "linux")]
use crate::{Port, PortSource};

/// The kernel's link type for a CAN interface (`ARPHRD_CAN`).
const CAN_LINK_TYPE: &str = "280";

/// The `SocketCAN` interfaces attached to this host.
pub struct SocketCanPorts {
    net: PathBuf,
}

impl SocketCanPorts {
    /// This host's kernel view.
    #[must_use]
    pub fn system() -> Self {
        Self {
            net: PathBuf::from("/sys/class/net"),
        }
    }

    /// The interface whose identity is `identity`: its USB adapter's serial number, or, for an
    /// interface on no USB device, its own name.
    ///
    /// # Errors
    ///
    /// Returns [`OpenError::NotAttached`] when no CAN interface carries the identity, and
    /// [`OpenError::Port`] when two do or the kernel's view cannot be read.
    pub fn interface(&self, identity: &str) -> Result<String, OpenError> {
        let port = |reason: String| OpenError::Port {
            serial: identity.to_owned(),
            reason,
        };
        let mut found: Vec<String> = Vec::new();
        for entry in fs::read_dir(&self.net).map_err(|error| port(error.to_string()))? {
            let entry = entry.map_err(|error| port(error.to_string()))?;
            let path = entry.path();
            if !fs::read_to_string(path.join("type")).is_ok_and(|kind| kind.trim() == CAN_LINK_TYPE)
            {
                continue;
            }
            let name = entry.file_name().to_string_lossy().into_owned();
            if usb_serial(&path).unwrap_or_else(|| name.clone()) == identity {
                found.push(name);
            }
        }
        match found.as_slice() {
            [] => Err(OpenError::NotAttached(identity.to_owned())),
            [interface] => Ok(interface.clone()),
            _ => Err(port(format!(
                "CAN interfaces {} all carry this identity",
                found.join(", ")
            ))),
        }
    }
}

/// The serial number of the USB device an interface's `device` link sits in: the link names
/// the USB interface (`1-1:1.0`), whose parent is the device.
fn usb_serial(interface: &Path) -> Option<String> {
    let device = fs::canonicalize(interface.join("device")).ok()?;
    let serial = fs::read_to_string(device.parent()?.join("serial")).ok()?;
    Some(serial.trim().to_owned()).filter(|serial| !serial.is_empty())
}

#[cfg(target_os = "linux")]
impl PortSource for SocketCanPorts {
    fn open(&mut self, serial: &str) -> Result<Port, OpenError> {
        let interface = self.interface(serial)?;
        let port = SocketCanPort::open(&interface).map_err(|error| OpenError::Port {
            serial: serial.to_owned(),
            reason: format!("{interface}: {error}"),
        })?;
        Ok(Box::new(port))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A CAN interface, on a USB adapter reporting `serial` when one is given.
    fn interface(root: &Path, name: &str, kind: &str, serial: Option<&str>) {
        let net = root.join("net").join(name);
        fs::create_dir_all(&net).expect("net");
        fs::write(net.join("type"), format!("{kind}\n")).expect("type");
        let device = if let Some(serial) = serial {
            let usb = root.join("usb").join(format!("1-{name}"));
            fs::create_dir_all(usb.join(format!("1-{name}:1.0"))).expect("usb");
            fs::write(usb.join("serial"), format!("{serial}\n")).expect("serial");
            usb.join(format!("1-{name}:1.0"))
        } else {
            let platform = root.join("platform").join(name);
            fs::create_dir_all(&platform).expect("platform");
            platform
        };
        std::os::unix::fs::symlink(device, net.join("device")).expect("device link");
    }

    #[test]
    fn an_interface_is_found_by_its_adapters_serial_or_its_own_name() {
        let root = std::env::temp_dir().join(format!(
            "sx-socketcan-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |elapsed| elapsed.as_nanos())
        ));
        interface(&root, "can0", CAN_LINK_TYPE, None);
        interface(&root, "can1", CAN_LINK_TYPE, Some("UB-CAN"));
        interface(&root, "eth0", "1", Some("NOT-CAN"));
        let ports = SocketCanPorts {
            net: root.join("net"),
        };
        assert_eq!(ports.interface("UB-CAN").expect("usb"), "can1");
        assert_eq!(ports.interface("can0").expect("on the board"), "can0");
        // A USB adapter is commissioned by its serial, never the name it enumerated as.
        assert!(matches!(
            ports.interface("can1"),
            Err(OpenError::NotAttached(_))
        ));
        assert!(matches!(
            ports.interface("NOT-CAN"),
            Err(OpenError::NotAttached(_))
        ));
        interface(&root, "can2", CAN_LINK_TYPE, Some("UB-CAN"));
        assert!(matches!(
            ports.interface("UB-CAN"),
            Err(OpenError::Port { .. })
        ));
        let _ = fs::remove_dir_all(root);
    }
}
