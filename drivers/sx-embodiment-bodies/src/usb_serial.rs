//! Commissioned USB serial bridges on Linux, found by their USB serial number.
//!
//! A bridge is identified by the serial number its USB descriptor reports, the same kernel fact
//! the station's inventory reports devices by, never by a device path: `/dev/ttyACM0` names
//! whichever bridge enumerated first. The kernel's `sysfs` tree is read for the USB device
//! whose `serial` is the commissioned one, and its one tty interface is opened raw. Damiao's
//! USB2CAN bridge is opened at the rate Seeed's controller uses, 921600 baud.

use std::fs;
use std::path::{Path, PathBuf};
use std::time::Duration;

use sx_damiao_can::usb2can::Usb2CanPort;

use crate::{OpenError, Port, PortSource};

/// The rate Seeed's `reBotArm_control_py` opens the USB2CAN bridge at.
pub const USB2CAN_BAUD: u32 = 921_600;
/// How long a read waits for bytes before reporting the buffer empty.
const READ_TIMEOUT: Duration = Duration::from_millis(1);

/// USB2CAN bridges attached to this host.
pub struct UsbSerialPorts {
    usb_devices: PathBuf,
    dev: PathBuf,
}

impl UsbSerialPorts {
    /// This host's kernel view.
    #[must_use]
    pub fn system() -> Self {
        Self {
            usb_devices: PathBuf::from("/sys/bus/usb/devices"),
            dev: PathBuf::from("/dev"),
        }
    }

    /// The kernel view under `usb_devices` (`/sys/bus/usb/devices`) and `dev`.
    #[cfg(test)]
    pub(crate) fn rooted(usb_devices: PathBuf, dev: PathBuf) -> Self {
        Self { usb_devices, dev }
    }

    /// The tty of the serial bridge whose USB serial number is `serial`.
    ///
    /// # Errors
    ///
    /// Returns [`OpenError::NotAttached`] when no attached device reporting the serial carries
    /// a tty (a `SocketCAN` adapter carries none), [`OpenError::Ambiguous`] when more than one
    /// tty does, and [`OpenError::Port`] when the kernel's view cannot be read.
    pub fn tty(&self, serial: &str) -> Result<PathBuf, OpenError> {
        let port = |reason: String| OpenError::Port {
            serial: serial.to_owned(),
            reason,
        };
        let entries = fs::read_dir(&self.usb_devices).map_err(|error| port(error.to_string()))?;
        let mut found: Vec<PathBuf> = Vec::new();
        for entry in entries {
            let entry = entry.map_err(|error| port(error.to_string()))?;
            let name = entry.file_name().to_string_lossy().into_owned();
            if name.contains(':') {
                continue;
            }
            if fs::read_to_string(entry.path().join("serial"))
                .is_ok_and(|reported| reported.trim() == serial)
            {
                let ttys =
                    interface_ttys(&entry.path()).map_err(|error| port(error.to_string()))?;
                found.extend(ttys.iter().map(|tty| self.dev.join(tty)));
            }
        }
        match found.as_slice() {
            [] => Err(OpenError::NotAttached(serial.to_owned())),
            [tty] => Ok(tty.clone()),
            _ => Err(OpenError::Ambiguous {
                identity: serial.to_owned(),
                bridges: found.iter().map(|tty| tty.display().to_string()).collect(),
            }),
        }
    }
}

/// The tty names under one USB device's interfaces: `<if>/tty/ttyACMn` for CDC ACM, and
/// `<if>/ttyUSBn` for a USB-serial converter.
fn interface_ttys(device: &Path) -> std::io::Result<Vec<String>> {
    let mut ttys = Vec::new();
    let Some(prefix) = device
        .file_name()
        .map(|name| format!("{}:", name.to_string_lossy()))
    else {
        return Ok(ttys);
    };
    for interface in fs::read_dir(device)? {
        let interface = interface?;
        if !interface.file_name().to_string_lossy().starts_with(&prefix) {
            continue;
        }
        let acm = interface.path().join("tty");
        let children = if acm.is_dir() {
            fs::read_dir(acm)?
        } else {
            fs::read_dir(interface.path())?
        };
        for child in children {
            let name = child?.file_name().to_string_lossy().into_owned();
            if name.starts_with("ttyACM") || name.starts_with("ttyUSB") {
                ttys.push(name);
            }
        }
    }
    ttys.sort();
    Ok(ttys)
}

impl PortSource for UsbSerialPorts {
    fn open(&mut self, serial: &str) -> Result<Port, OpenError> {
        open_tty(&self.tty(serial)?, serial)
    }
}

/// Damiao's USB2CAN bridge on `tty`, commissioned as `serial`.
pub(crate) fn open_tty(tty: &Path, serial: &str) -> Result<Port, OpenError> {
    let stream = serialport::new(tty.to_string_lossy(), USB2CAN_BAUD)
        .timeout(READ_TIMEOUT)
        .open()
        .map_err(|error| OpenError::Port {
            serial: serial.to_owned(),
            reason: error.to_string(),
        })?;
    Ok(Box::new(Usb2CanPort::new(stream)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn device(root: &Path, name: &str, serial: &str, tty: &str, acm: bool) {
        let device = root.join(name);
        fs::create_dir_all(&device).expect("device");
        fs::write(device.join("serial"), format!("{serial}\n")).expect("serial");
        let interface = device.join(format!("{name}:1.0"));
        let parent = if acm {
            interface.join("tty")
        } else {
            interface.clone()
        };
        fs::create_dir_all(parent.join(tty)).expect("tty");
    }

    fn ports(root: &Path) -> UsbSerialPorts {
        UsbSerialPorts::rooted(root.to_path_buf(), PathBuf::from("/dev"))
    }

    #[test]
    fn a_bridge_is_found_by_its_serial_never_its_enumeration_order() {
        let root = tempfile_dir();
        device(&root, "1-1", "B601-ARM", "ttyACM1", true);
        device(&root, "1-2", "B601-GRIP", "ttyUSB0", false);
        let ports = ports(&root);
        assert_eq!(
            ports.tty("B601-ARM").expect("arm"),
            Path::new("/dev/ttyACM1")
        );
        assert_eq!(
            ports.tty("B601-GRIP").expect("gripper"),
            Path::new("/dev/ttyUSB0")
        );
        assert!(matches!(
            ports.tty("absent"),
            Err(OpenError::NotAttached(_))
        ));
        device(&root, "1-3", "B601-ARM", "ttyACM2", true);
        assert!(matches!(
            ports.tty("B601-ARM"),
            Err(OpenError::Ambiguous { .. })
        ));
        let _ = fs::remove_dir_all(root);
    }

    fn tempfile_dir() -> PathBuf {
        let root = std::env::temp_dir().join(format!(
            "sx-usb-serial-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map_or(0, |elapsed| elapsed.as_nanos())
        ));
        fs::create_dir_all(&root).expect("root");
        root
    }
}
