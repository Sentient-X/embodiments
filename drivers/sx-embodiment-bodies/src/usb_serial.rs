//! Commissioned USB serial bridges, found by their USB serial number, on Linux and macOS.
//!
//! A bridge is identified by the serial number its USB descriptor reports, the same kernel fact
//! the station's inventory reports devices by, never by a device path: `/dev/ttyACM0` names
//! whichever bridge enumerated first. On Linux the kernel's `sysfs` tree is read for the USB
//! device whose `serial` is the commissioned one; on macOS the `IOKit` registry, through
//! `serialport`'s enumeration, names the device's callout node (`/dev/cu.*`). Its one tty
//! interface is opened raw. Damiao's
//! USB2CAN bridge is opened at the rate Seeed's controller uses, 921600 baud; a serial-servo
//! adapter (Feetech, Dynamixel) at the rate its body's bus runs at, as a [`ServoLink`].

use std::fs;
use std::io::{self, Read as _, Write as _};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use serialport::ClearBuffer;
use sx_damiao_can::usb2can::Usb2CanPort;
use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{Holder, holder_of};

use crate::{OpenError, Port, PortSource, SerialLink};

/// The rate Seeed's `reBotArm_control_py` opens the USB2CAN bridge at.
pub const USB2CAN_BAUD: u32 = 921_600;
/// How long a read waits for bytes before reporting the buffer empty.
const READ_TIMEOUT: Duration = Duration::from_millis(1);

/// USB serial bridges attached to this host.
pub struct UsbSerialPorts {
    kernel: Kernel,
}

/// Where the host's USB devices and their serial numbers are read.
enum Kernel {
    /// Linux `sysfs`: the USB devices under `usb_devices`, their ttys under `dev`.
    #[cfg_attr(all(target_os = "macos", not(test)), allow(dead_code))]
    Sysfs { usb_devices: PathBuf, dev: PathBuf },
    /// macOS `IOKit`, as `serialport` enumerates it.
    #[cfg(target_os = "macos")]
    IoKit,
}

impl UsbSerialPorts {
    /// This host's kernel view.
    #[must_use]
    pub fn system() -> Self {
        #[cfg(target_os = "macos")]
        let kernel = Kernel::IoKit;
        #[cfg(not(target_os = "macos"))]
        let kernel = Kernel::Sysfs {
            usb_devices: PathBuf::from("/sys/bus/usb/devices"),
            dev: PathBuf::from("/dev"),
        };
        Self { kernel }
    }

    /// The kernel view under `usb_devices` (`/sys/bus/usb/devices`) and `dev`.
    #[cfg(test)]
    pub(crate) fn rooted(usb_devices: PathBuf, dev: PathBuf) -> Self {
        Self {
            kernel: Kernel::Sysfs { usb_devices, dev },
        }
    }

    /// The tty of the serial bridge whose USB serial number is `serial`.
    ///
    /// # Errors
    ///
    /// Returns [`OpenError::NotAttached`] when no attached device reporting the serial carries
    /// a tty (a `SocketCAN` adapter carries none), [`OpenError::Ambiguous`] when more than one
    /// tty does or a tty and another device both report it, and [`OpenError::Port`] when the
    /// kernel's view cannot be read.
    pub fn tty(&self, serial: &str) -> Result<PathBuf, OpenError> {
        match &self.kernel {
            Kernel::Sysfs { usb_devices, dev } => sysfs_tty(usb_devices, dev, serial),
            #[cfg(target_os = "macos")]
            Kernel::IoKit => iokit_tty(serial),
        }
    }
}

/// The callout node of the one USB serial device reporting `serial`, from `IOKit`.
///
/// `IOKit` lists each device twice, as its dial-in `/dev/tty.*` and callout `/dev/cu.*` node; the
/// callout node is the one a host opens without waiting for carrier detect.
#[cfg(target_os = "macos")]
fn iokit_tty(serial: &str) -> Result<PathBuf, OpenError> {
    let found: Vec<PathBuf> = serialport::available_ports()
        .map_err(|error| OpenError::Port {
            serial: serial.to_owned(),
            reason: error.to_string(),
        })?
        .into_iter()
        .filter(|port| {
            port.port_name.starts_with("/dev/cu.")
                && matches!(&port.port_type, serialport::SerialPortType::UsbPort(usb)
                    if usb.serial_number.as_deref().map(str::trim) == Some(serial))
        })
        .map(|port| PathBuf::from(port.port_name))
        .collect();
    match found.as_slice() {
        [] => Err(OpenError::NotAttached(serial.to_owned())),
        [tty] => Ok(tty.clone()),
        _ => Err(OpenError::Ambiguous {
            identity: serial.to_owned(),
            bridges: found
                .iter()
                .map(|path| path.display().to_string())
                .collect(),
        }),
    }
}

/// The tty of the one USB device under `usb_devices` reporting `serial`.
fn sysfs_tty(usb_devices: &Path, dev: &Path, serial: &str) -> Result<PathBuf, OpenError> {
    let port = |reason: String| OpenError::Port {
        serial: serial.to_owned(),
        reason,
    };
    let entries = fs::read_dir(usb_devices).map_err(|error| port(error.to_string()))?;
    let mut found: Vec<PathBuf> = Vec::new();
    // Devices reporting the serial that carry no tty, such as a SocketCAN adapter.
    let mut others: Vec<PathBuf> = Vec::new();
    for entry in entries {
        let entry = entry.map_err(|error| port(error.to_string()))?;
        let name = entry.file_name().to_string_lossy().into_owned();
        if name.contains(':') {
            continue;
        }
        if fs::read_to_string(entry.path().join("serial"))
            .is_ok_and(|reported| reported.trim() == serial)
        {
            let ttys = interface_ttys(&entry.path()).map_err(|error| port(error.to_string()))?;
            if ttys.is_empty() {
                others.push(entry.path());
            }
            found.extend(ttys.iter().map(|tty| dev.join(tty)));
        }
    }
    match (found.as_slice(), others.is_empty()) {
        ([], _) => Err(OpenError::NotAttached(serial.to_owned())),
        ([tty], true) => Ok(tty.clone()),
        // Two ttys, or a tty and another device reporting the same serial: which one is
        // the commissioned bridge cannot be told.
        _ => Err(OpenError::Ambiguous {
            identity: serial.to_owned(),
            bridges: found
                .iter()
                .chain(&others)
                .map(|path| path.display().to_string())
                .collect(),
        }),
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

    fn open_serial(&mut self, serial: &str, baud: u32) -> Result<SerialLink, OpenError> {
        open_servo_tty(&self.tty(serial)?, serial, baud)
    }
}

/// The longest a serial-servo link waits for one status packet. Feetech's `scservo_sdk` waits
/// a packet's transmit time plus 50 ms (`LATENCY_TIMER`, `port_handler.py`); ROBOTIS's
/// `DynamixelSDK` its transmit time plus 34 ms. Every status packet a chain here asks for is a
/// few dozen bytes, under a millisecond at 1 Mbaud, so one fixed 50 ms bounds both.
pub const SERVO_PACKET_TIMEOUT: Duration = Duration::from_millis(50);

/// A serial-servo adapter's tty as the vendor drivers' `PortHandler`.
pub struct ServoLink {
    stream: Box<dyn serialport::SerialPort>,
    deadline: Instant,
}

impl SerialPort for ServoLink {
    fn clear_port(&mut self) -> io::Result<()> {
        self.stream
            .clear(ClearBuffer::Input)
            .map_err(io::Error::other)
    }

    fn write_port(&mut self, packet: &[u8]) -> io::Result<usize> {
        self.stream.write_all(packet)?;
        self.stream.flush()?;
        Ok(packet.len())
    }

    fn read_port(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        let waiting = usize::try_from(self.stream.bytes_to_read().map_err(io::Error::other)?)
            .unwrap_or(usize::MAX);
        let take = buffer.len().min(waiting);
        if take == 0 {
            return Ok(0);
        }
        self.stream.read(&mut buffer[..take])
    }

    fn timeout(&self) -> Duration {
        SERVO_PACKET_TIMEOUT
    }

    fn set_packet_timeout(&mut self, _packet_length: usize) {
        self.deadline = Instant::now() + SERVO_PACKET_TIMEOUT;
    }

    fn is_packet_timeout(&mut self) -> bool {
        Instant::now() > self.deadline
    }
}

/// Why a tty did not open.
#[derive(Debug, thiserror::Error)]
pub enum TtyError {
    /// Another process has it open: `TIOCEXCL` or its exclusive `flock` refused us.
    #[error("{} is in use, {holder}", tty.display())]
    Busy { tty: PathBuf, holder: Holder },
    #[error("{}: {source}", tty.display())]
    Open {
        tty: PathBuf,
        source: serialport::Error,
    },
}

/// Open `tty` raw at `baud`, exclusively: the one way the layer opens a serial bus.
///
/// `serialport` takes `TIOCEXCL` and an exclusive `flock` on the descriptor, so a second
/// opener, in this process or another, is refused while this port lives, and the kernel lets
/// go when the process ends however it ends. That refusal (`EBUSY` from `open`, or the lock
/// held, both reported by `serialport` as `NoDevice` at open) is [`TtyError::Busy`], naming the
/// holder where the host tells it.
///
/// # Errors
///
/// [`TtyError::Busy`] when another opener holds it, [`TtyError::Open`] otherwise.
pub fn open_exclusive(
    tty: &Path,
    baud: u32,
    timeout: Duration,
) -> Result<Box<dyn serialport::SerialPort>, TtyError> {
    serialport::new(tty.to_string_lossy(), baud)
        .timeout(timeout)
        .exclusive(true)
        .open()
        .map_err(|source| busy_or(tty, source))
}

/// A failed open as [`TtyError`]: `NoDevice` is how `serialport` reports a held port at open.
fn busy_or(tty: &Path, source: serialport::Error) -> TtyError {
    if source.kind() == serialport::ErrorKind::NoDevice {
        TtyError::Busy {
            tty: tty.to_path_buf(),
            holder: holder_of(tty),
        }
    } else {
        TtyError::Open {
            tty: tty.to_path_buf(),
            source,
        }
    }
}

impl TtyError {
    /// This failure as the body's [`OpenError`] for the bridge commissioned as `serial`.
    fn for_bridge(self, serial: &str) -> OpenError {
        match self {
            Self::Busy { tty, holder } => OpenError::Busy {
                serial: serial.to_owned(),
                device: tty.display().to_string(),
                holder,
            },
            Self::Open { source, .. } => OpenError::Port {
                serial: serial.to_owned(),
                reason: source.to_string(),
            },
        }
    }
}

/// A serial-servo adapter on `tty`, commissioned as `serial`, at `baud`.
pub(crate) fn open_servo_tty(tty: &Path, serial: &str, baud: u32) -> Result<SerialLink, OpenError> {
    let stream =
        open_exclusive(tty, baud, READ_TIMEOUT).map_err(|error| error.for_bridge(serial))?;
    Ok(Box::new(ServoLink {
        stream,
        deadline: Instant::now(),
    }))
}

/// Damiao's USB2CAN bridge on `tty`, commissioned as `serial`.
pub(crate) fn open_tty(tty: &Path, serial: &str) -> Result<Port, OpenError> {
    let stream = open_exclusive(tty, USB2CAN_BAUD, READ_TIMEOUT)
        .map_err(|error| error.for_bridge(serial))?;
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
        // A serial bridge and a device with no tty (a CAN adapter) reporting one serial.
        let adapter = root.join("1-4");
        fs::create_dir_all(adapter.join("1-4:1.0")).expect("adapter");
        fs::write(adapter.join("serial"), "B601-GRIP\n").expect("serial");
        match ports.tty("B601-GRIP") {
            Err(OpenError::Ambiguous { identity, bridges }) => {
                assert_eq!(identity, "B601-GRIP");
                assert_eq!(bridges.len(), 2);
            }
            other => panic!("expected an ambiguity, got {other:?}"),
        }
        // That device alone is no serial bridge.
        fs::write(adapter.join("serial"), "UB-CAN\n").expect("serial");
        assert!(matches!(
            ports.tty("UB-CAN"),
            Err(OpenError::NotAttached(_))
        ));
        let _ = fs::remove_dir_all(root);
    }

    /// A pseudo-terminal pair: the controller side, which keeps the terminal alive, and the
    /// terminal's path, which a bus opener opens.
    #[cfg(unix)]
    fn pseudo_terminal() -> (serialport::TTYPort, serialport::TTYPort, PathBuf) {
        let (controller, terminal) = serialport::TTYPort::pair().expect("pseudo-terminal");
        let tty = PathBuf::from(
            serialport::SerialPort::name(&terminal).expect("the pseudo-terminal's path"),
        );
        (controller, terminal, tty)
    }

    /// Another process's hold, as the exclusive `flock` every opener in the layer takes.
    #[cfg(unix)]
    #[test]
    fn a_tty_another_opener_holds_is_refused_as_busy_naming_the_holder() {
        let (_controller, _terminal, tty) = pseudo_terminal();
        let hold = fs::File::open(&tty).expect("the holder's descriptor");
        hold.try_lock().expect("the holder's exclusive lock");
        let refused = open_servo_tty(&tty, "B601-GRIP", 1_000_000)
            .err()
            .expect("a second opener is refused");
        let OpenError::Busy {
            serial,
            device,
            holder,
        } = &refused
        else {
            panic!("expected the bus busy, got {refused}");
        };
        assert_eq!(serial, "B601-GRIP");
        assert_eq!(device, &tty.display().to_string());
        // Linux names the holder from /proc: this test process itself.
        if cfg!(target_os = "linux") {
            assert!(
                holder
                    .to_string()
                    .contains(&format!("pid {}", std::process::id())),
                "{holder}"
            );
        }
        assert!(
            refused.to_string().contains("is in use, held by"),
            "{refused}"
        );
        assert!(matches!(
            open_tty(&tty, "B601-ARM"),
            Err(OpenError::Busy { .. })
        ));
    }

    /// The whole exclusive open twice. macOS cannot set a pseudo-terminal's speed
    /// (`IOSSIOSPEED` answers ENOTTY), so the first open only succeeds on Linux.
    #[cfg(target_os = "linux")]
    #[test]
    fn a_second_exclusive_open_is_refused_until_the_first_lets_go() {
        let (_controller, _terminal, tty) = pseudo_terminal();
        let held = open_exclusive(&tty, 1_000_000, READ_TIMEOUT).expect("the first opener");
        assert!(matches!(
            open_exclusive(&tty, 1_000_000, READ_TIMEOUT),
            Err(TtyError::Busy { .. })
        ));
        drop(held);
        open_exclusive(&tty, 1_000_000, READ_TIMEOUT).expect("free once the first lets go");
    }

    #[test]
    fn a_missing_tty_is_not_reported_busy() {
        let missing = tempfile_dir().join("ttyACM9");
        assert!(matches!(
            open_exclusive(&missing, 1_000_000, READ_TIMEOUT),
            Err(TtyError::Open { .. })
        ));
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
