//! Commissioned `SocketCAN` interfaces on Linux, found by their adapter's USB serial number.
//!
//! As with the USB serial bridges, a bus is commissioned by a kernel fact that does not depend
//! on enumeration order: `can0` and `can1` name whichever adapter the kernel found first. Each
//! CAN interface under `/sys/class/net` (link type 280, `ARPHRD_CAN`) has at most one identity:
//! for a USB adapter (`gs_usb`, `peak_usb`; its `device` link sits in a USB device, the one
//! carrying `idVendor`), that device's `serial`; for a controller on the board itself, or an
//! interface on no device, its own name, which the device tree names the same on every boot.
//! A USB adapter whose serial cannot be read has no identity and is never opened: its name
//! follows enumeration order, so it could be a different adapter on the next boot.
//! Commissioning records the identity, and the one interface carrying it is opened as an
//! `sx_socketcan::SocketCanPort`, which also holds that interface's lock against a second of
//! our processes.
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

    /// The kernel view under `net` (`/sys/class/net`).
    #[cfg(test)]
    pub(crate) fn rooted(net: PathBuf) -> Self {
        Self { net }
    }

    /// The interface whose identity is `identity`: its USB adapter's serial number, or, for an
    /// interface on no USB device, its own name.
    ///
    /// # Errors
    ///
    /// Returns [`OpenError::NotAttached`] when no CAN interface carries the identity,
    /// [`OpenError::Ambiguous`] when two do, and [`OpenError::Port`] when the kernel's view
    /// cannot be read. A host with no kernel network view at all has no interface: not
    /// attached, so another transport (a ZLG adapter on macOS) can carry the identity.
    pub fn interface(&self, identity: &str) -> Result<String, OpenError> {
        let port = |reason: String| OpenError::Port {
            serial: identity.to_owned(),
            reason,
        };
        let entries = match fs::read_dir(&self.net) {
            Ok(entries) => entries,
            // A host with no kernel network view (macOS has no `/sys`) carries no interface.
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
                return Err(OpenError::NotAttached(identity.to_owned()));
            }
            Err(error) => return Err(port(error.to_string())),
        };
        let mut found: Vec<String> = Vec::new();
        for entry in entries {
            let entry = entry.map_err(|error| port(error.to_string()))?;
            let path = entry.path();
            if !fs::read_to_string(path.join("type")).is_ok_and(|kind| kind.trim() == CAN_LINK_TYPE)
            {
                continue;
            }
            let name = entry.file_name().to_string_lossy().into_owned();
            if commissioned_as(&path, &name).as_deref() == Some(identity) {
                found.push(name);
            }
        }
        match found.as_slice() {
            [] => Err(OpenError::NotAttached(identity.to_owned())),
            [interface] => Ok(interface.clone()),
            _ => Err(OpenError::Ambiguous {
                identity: identity.to_owned(),
                bridges: found,
            }),
        }
    }
}

/// The identity an interface is commissioned by: its USB device's serial when its `device`
/// link sits in a USB device (the ancestor carrying `idVendor`), else its own name; `None` for
/// a USB adapter whose serial cannot be read.
fn commissioned_as(interface: &Path, name: &str) -> Option<String> {
    let Ok(device) = fs::canonicalize(interface.join("device")) else {
        return Some(name.to_owned());
    };
    let Some(usb) = device
        .ancestors()
        .find(|dir| dir.join("idVendor").is_file())
    else {
        return Some(name.to_owned());
    };
    let serial = fs::read_to_string(usb.join("serial")).ok()?;
    Some(serial.trim().to_owned()).filter(|serial| !serial.is_empty())
}

#[cfg(target_os = "linux")]
impl PortSource for SocketCanPorts {
    fn open(&mut self, serial: &str) -> Result<Port, OpenError> {
        let interface = self.interface(serial)?;
        let port = SocketCanPort::open(&interface).map_err(|error| OpenError::Port {
            serial: serial.to_owned(),
            reason: error.to_string(),
        })?;
        Ok(Box::new(port))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_host_without_a_kernel_network_view_carries_no_interface() {
        let ports = SocketCanPorts::rooted(PathBuf::from("/nonexistent/sys/class/net"));
        assert!(matches!(
            ports.interface("A30F115DD0120A949B80:1"),
            Err(OpenError::NotAttached(_))
        ));
    }

    /// Where an interface's device sits.
    #[derive(Clone, Copy)]
    enum On<'a> {
        Board,
        Usb(Option<&'a str>),
    }

    /// A CAN interface, on the board or on a USB adapter reporting `serial` when it has one.
    fn interface(root: &Path, name: &str, kind: &str, on: On<'_>) {
        let net = root.join("net").join(name);
        fs::create_dir_all(&net).expect("net");
        fs::write(net.join("type"), format!("{kind}\n")).expect("type");
        let device = if let On::Usb(serial) = on {
            let usb = root.join("usb").join(format!("1-{name}"));
            fs::create_dir_all(usb.join(format!("1-{name}:1.0"))).expect("usb");
            fs::write(usb.join("idVendor"), "1d50\n").expect("vendor");
            if let Some(serial) = serial {
                fs::write(usb.join("serial"), format!("{serial}\n")).expect("serial");
            }
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
        interface(&root, "can0", CAN_LINK_TYPE, On::Board);
        interface(&root, "can1", CAN_LINK_TYPE, On::Usb(Some("UB-CAN")));
        interface(&root, "eth0", "1", On::Usb(Some("NOT-CAN")));
        interface(&root, "can3", CAN_LINK_TYPE, On::Usb(None));
        let ports = SocketCanPorts::rooted(root.join("net"));
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
        // A USB adapter without a readable serial is never opened, not even by its name.
        assert!(matches!(
            ports.interface("can3"),
            Err(OpenError::NotAttached(_))
        ));
        interface(&root, "can2", CAN_LINK_TYPE, On::Usb(Some("UB-CAN")));
        assert!(matches!(
            ports.interface("UB-CAN"),
            Err(OpenError::Ambiguous { .. })
        ));
        let _ = fs::remove_dir_all(root);
    }
}
