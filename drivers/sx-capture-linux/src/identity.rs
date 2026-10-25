//! Factory identities, read before streaming. Ports and node names are locations only.
use std::collections::BTreeMap;
use std::fs;
#[cfg(target_os = "linux")]
use std::fs::{File, OpenOptions};
use std::path::{Path, PathBuf};
#[cfg(target_os = "linux")]
use std::thread;
use std::time::Duration;
#[cfg(target_os = "linux")]
use std::time::Instant;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum IdentityError {
    #[error("hardware identification: {0}")]
    Io(#[from] std::io::Error),
    #[error(
        "cannot identify supported device {path}: {source}; reconnect the rig and rerun the device check; for permission errors, rerun the pod installer"
    )]
    Device {
        path: PathBuf,
        source: Box<IdentityError>,
    },
    #[cfg(target_os = "linux")]
    #[error("encoder access: {0}")]
    Serial(#[from] serialport::Error),
    #[error(
        "connect the complete registered rig: observed cameras={cameras:?}, encoders={encoders:?}"
    )]
    Incomplete {
        cameras: DevicePaths,
        encoders: DevicePaths,
    },
    #[error("hardware identification refused: {0}")]
    Unsupported(String),
    /// Another holder kept the camera's exclusive lock through the whole wait: the last
    /// capture's teardown or a second identity reader. Transient, never a rig verdict.
    #[error(
        "camera {} is busy: another capture or identity reader held it for {} s; checking again",
        path.display(),
        waited.as_secs()
    )]
    Busy { path: PathBuf, waited: Duration },
}

impl IdentityError {
    /// Whether this failure is only a camera held by someone else, at any depth.
    pub fn is_camera_busy(&self) -> bool {
        match self {
            Self::Busy { .. } => true,
            Self::Device { source, .. } => source.is_camera_busy(),
            _ => false,
        }
    }
}

/// How long an identity reader waits for the camera's exclusive lock. A capture's
/// teardown releases it within milliseconds of its join; a second reader holds it for
/// one identification (a few hundred milliseconds).
#[cfg(any(target_os = "linux", test))]
pub const CAMERA_LOCK_WAIT: Duration = Duration::from_secs(5);
#[cfg(target_os = "linux")]
const CAMERA_LOCK_POLL: Duration = Duration::from_millis(50);

#[cfg(target_os = "linux")]
use sx_yubi::identity::Xu;
use sx_yubi::identity::{CAMERA_XU_GUID as GUID, Refused};

impl From<Refused> for IdentityError {
    fn from(refused: Refused) -> Self {
        Self::Unsupported(refused.0)
    }
}

fn usb_parent(node: &Path, class: &str) -> Result<PathBuf, IdentityError> {
    let node = fs::canonicalize(node)?;
    let name = node
        .file_name()
        .ok_or_else(|| IdentityError::Unsupported("device has no name".into()))?;
    let device = fs::canonicalize(
        Path::new("/sys/class")
            .join(class)
            .join(name)
            .join("device"),
    )?;
    device
        .ancestors()
        .find(|p| p.join("idVendor").exists())
        .map(Path::to_path_buf)
        .ok_or_else(|| IdentityError::Unsupported(format!("{} is not USB", node.display())))
}

fn usb_model(parent: &Path) -> Result<(String, String), IdentityError> {
    Ok((
        fs::read_to_string(parent.join("idVendor"))?
            .trim()
            .to_owned(),
        fs::read_to_string(parent.join("idProduct"))?
            .trim()
            .to_owned(),
    ))
}

pub fn encoder_id(node: &Path) -> Result<String, IdentityError> {
    let parent = usb_parent(node, "tty")?;
    if usb_model(&parent)? != ("303a".into(), "1001".into()) {
        return Err(IdentityError::Unsupported(format!(
            "{} is not a native ESP32-C6",
            node.display()
        )));
    }
    encoder_serial(&parent)
}

fn encoder_serial(parent: &Path) -> Result<String, IdentityError> {
    let serial = fs::read_to_string(parent.join("serial"))?;
    Ok(sx_yubi::identity::encoder_identity(&serial)?)
}

#[cfg(target_os = "linux")]
pub fn camera_uid(node: &Path) -> Result<String, IdentityError> {
    open_camera(node).map(|(_, id)| id)
}

#[cfg(not(target_os = "linux"))]
pub fn camera_uid(_node: &Path) -> Result<String, IdentityError> {
    Err(IdentityError::Unsupported(
        "camera identification requires Linux".into(),
    ))
}

/// Retain both inodes so disconnect/recreation cannot reuse them between identification
/// and open. A pathname or USB device number alone can name a replacement after hotplug.
#[cfg(target_os = "linux")]
struct Connection {
    node: PathBuf,
    node_file: File,
    usb: PathBuf,
    usb_file: File,
}

#[cfg(target_os = "linux")]
impl Connection {
    fn pin(node: &Path, class: &str) -> Result<Self, IdentityError> {
        use std::os::unix::fs::OpenOptionsExt;
        let node_file = OpenOptions::new()
            .read(true)
            .custom_flags(nix::libc::O_PATH)
            .open(node)?;
        let usb = usb_parent(node, class)?;
        let usb_file = File::open(&usb)?;
        Ok(Self {
            node: node.to_path_buf(),
            node_file,
            usb,
            usb_file,
        })
    }

    #[allow(unsafe_code)]
    fn open_serial(&self) -> Result<serialport::TTYPort, IdentityError> {
        use nix::sys::termios::{self, ControlFlags, SetArg};
        use serialport::SerialPort;
        use std::os::fd::{AsRawFd, FromRawFd, IntoRawFd};
        use std::os::unix::fs::OpenOptionsExt;
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(nix::libc::O_NOCTTY | nix::libc::O_NONBLOCK)
            .open(&self.node)?;
        // serialport::open_native configures termios before returning. Verify the
        // raw descriptor first, before any serial configuration or control-line write.
        self.verify(file.as_raw_fd())?;
        // SAFETY: transfer this uniquely owned, verified TTY descriptor exactly once;
        // TTYPort owns its close. The inode pins remain alive until configuration ends.
        let mut serial = unsafe { serialport::TTYPort::from_raw_fd(file.into_raw_fd()) };
        serial.set_exclusive(true)?;
        // Raw 115200 8N1, no hardware flow control; preserve DTR/RTS state.
        // SAFETY: the borrow cannot outlive serial, which remains owned here.
        let fd = unsafe { std::os::fd::BorrowedFd::borrow_raw(serial.as_raw_fd()) };
        let mut settings = termios::tcgetattr(fd).map_err(std::io::Error::from)?;
        termios::cfmakeraw(&mut settings);
        settings
            .control_flags
            .insert(ControlFlags::CREAD | ControlFlags::CLOCAL);
        termios::tcsetattr(fd, SetArg::TCSANOW, &settings).map_err(std::io::Error::from)?;
        nix::fcntl::fcntl(
            fd,
            nix::fcntl::FcntlArg::F_SETFL(nix::fcntl::OFlag::empty()),
        )
        .map_err(std::io::Error::from)?;
        serial.set_baud_rate(115_200)?;
        serial.set_data_bits(serialport::DataBits::Eight)?;
        serial.set_stop_bits(serialport::StopBits::One)?;
        serial.set_parity(serialport::Parity::None)?;
        serial.set_flow_control(serialport::FlowControl::None)?;
        serial.set_timeout(Duration::from_millis(200))?;
        Ok(serial)
    }

    fn verify(&self, fd: std::os::fd::RawFd) -> Result<(), IdentityError> {
        use std::os::unix::fs::MetadataExt;
        let key = |m: std::fs::Metadata| (m.dev(), m.ino(), m.rdev());
        let node = key(self.node_file.metadata()?);
        if key(fs::metadata(&self.node)?) != node
            || key(fs::metadata(format!("/proc/self/fd/{fd}"))?) != node
            || key(fs::metadata(&self.usb)?) != key(self.usb_file.metadata()?)
        {
            return Err(IdentityError::Unsupported(format!(
                "{} reconnected while opening; rerun the device check",
                self.node.display()
            )));
        }
        Ok(())
    }
}

/// An open camera and the exclusive lock that keeps other identity readers and captures
/// off it; both close together.
#[cfg(target_os = "linux")]
pub struct LockedCamera {
    device: v4l::Device,
    _lock: File,
}

#[cfg(target_os = "linux")]
impl std::ops::Deref for LockedCamera {
    type Target = v4l::Device;

    fn deref(&self) -> &v4l::Device {
        &self.device
    }
}

#[cfg(target_os = "linux")]
pub fn open_camera(node: &Path) -> Result<(LockedCamera, String), IdentityError> {
    let connection = Connection::pin(node, "video4linux")?;
    if usb_model(&connection.usb)? != ("32e4".into(), "9230".into())
        || !fs::read(connection.usb.join("descriptors"))?
            .windows(GUID.len())
            .any(|w| w == GUID)
    {
        return Err(IdentityError::Unsupported(
            "not the supported Sonix camera".into(),
        ));
    }
    // Competing identity readers and captures must stop: the lock comes first.
    let lock = lock_camera(node, CAMERA_LOCK_WAIT)?;
    connection.verify(std::os::fd::AsRawFd::as_raw_fd(&lock))?;
    let camera = v4l::Device::with_path(node)?;
    connection.verify(camera.handle().fd())?;
    close_on_exec(camera.handle().fd())?;
    let id = sx_yubi::identity::flash_uid(&XuFd(camera.handle().fd()))?;
    Ok((
        LockedCamera {
            device: camera,
            _lock: lock,
        },
        id,
    ))
}

/// The camera's exclusive lock, on a descriptor of its own opened close-on-exec.
///
/// A flock belongs to the open file description. v4l opens its stream descriptor without
/// `O_CLOEXEC`, and the lock used to be taken on it, so every child spawned while a camera
/// was open — the processor and its validator, which run beside the next recording, adb —
/// shared the lock and kept the camera locked until that child exited, minutes after the
/// capture stopped. This descriptor is close-on-exec from the `open` itself, so no child
/// can hold it, however the spawns interleave.
#[cfg(target_os = "linux")]
fn lock_camera(node: &Path, wait: Duration) -> Result<File, IdentityError> {
    use std::os::unix::fs::OpenOptionsExt;
    let lock = OpenOptions::new()
        .read(true)
        .custom_flags(nix::libc::O_NONBLOCK)
        .open(node)?;
    hold_exclusively(std::os::fd::AsRawFd::as_raw_fd(&lock), node, wait)?;
    Ok(lock)
}

/// Keep a stream descriptor out of spawned children: one a child inherited would keep the
/// device and its buffers open after the capture closed it.
#[cfg(target_os = "linux")]
#[allow(unsafe_code)]
fn close_on_exec(fd: std::os::fd::RawFd) -> Result<(), IdentityError> {
    use nix::fcntl::{FcntlArg, FdFlag, fcntl};
    // SAFETY: the caller owns `fd` open for the whole call; the borrow ends with it.
    let borrowed = unsafe { std::os::fd::BorrowedFd::borrow_raw(fd) };
    fcntl(borrowed, FcntlArg::F_SETFD(FdFlag::FD_CLOEXEC)).map_err(std::io::Error::from)?;
    Ok(())
}

/// Take `fd`'s exclusive lock, waiting up to `wait` for another holder to let go.
#[cfg(target_os = "linux")]
fn hold_exclusively(
    fd: std::os::fd::RawFd,
    node: &Path,
    wait: Duration,
) -> Result<(), IdentityError> {
    use nix::errno::Errno;
    use nix::fcntl::FlockArg;
    let started = Instant::now();
    loop {
        #[allow(deprecated)]
        match nix::fcntl::flock(fd, FlockArg::LockExclusiveNonblock) {
            Ok(()) => return Ok(()),
            Err(Errno::EINTR) => {}
            Err(Errno::EWOULDBLOCK) if started.elapsed() < wait => thread::sleep(CAMERA_LOCK_POLL),
            Err(Errno::EWOULDBLOCK) => {
                return Err(IdentityError::Busy {
                    path: node.to_path_buf(),
                    waited: started.elapsed(),
                });
            }
            Err(error) => return Err(std::io::Error::from(error).into()),
        }
    }
}

#[cfg(target_os = "linux")]
pub fn open_encoder(node: &Path) -> Result<(Encoder, String), IdentityError> {
    let connection = Connection::pin(node, "tty")?;
    let id = encoder_id(node)?;
    let serial = connection.open_serial()?;
    Ok((Encoder(serial), id))
}

/// A serial encoder opened only after its USB identity and connection were verified.
#[cfg(target_os = "linux")]
pub struct Encoder(serialport::TTYPort);
#[cfg(target_os = "linux")]
impl Encoder {
    pub fn set_timeout(&mut self, timeout: Duration) -> Result<(), serialport::Error> {
        serialport::SerialPort::set_timeout(&mut self.0, timeout)
    }
    pub fn clear_input(&self) -> Result<(), serialport::Error> {
        serialport::SerialPort::clear(&self.0, serialport::ClearBuffer::Input)
    }
    /// Reset the encoder's ESP32 through its USB-serial bridge: DTR released, EN held low
    /// through RTS for `pulse`, then released so it boots. Only the verified line is pulsed.
    pub fn pulse_reset(&mut self, pulse: Duration) -> Result<(), serialport::Error> {
        serialport::SerialPort::write_data_terminal_ready(&mut self.0, false)?;
        serialport::SerialPort::write_request_to_send(&mut self.0, true)?;
        thread::sleep(pulse);
        serialport::SerialPort::write_request_to_send(&mut self.0, false)
    }
}
#[cfg(target_os = "linux")]
impl std::io::Read for Encoder {
    fn read(&mut self, bytes: &mut [u8]) -> std::io::Result<usize> {
        std::io::Read::read(&mut self.0, bytes)
    }
}

/// The camera's register-access extension control through `UVCIOC_CTRL_QUERY`.
#[cfg(target_os = "linux")]
struct XuFd(std::os::fd::RawFd);

#[cfg(target_os = "linux")]
impl Xu for XuFd {
    type Error = IdentityError;
    #[allow(unsafe_code)]
    fn query(&self, query: u8, bytes: &mut [u8; 4]) -> Result<(), IdentityError> {
        #[repr(C)]
        struct Query {
            unit: u8,
            selector: u8,
            query: u8,
            size: u16,
            data: *mut u8,
        }
        let mut argument = Query {
            unit: 3,
            selector: 1,
            query,
            size: 4,
            data: bytes.as_mut_ptr(),
        };
        let request = 0xc0000000
            | (std::mem::size_of::<Query>() as nix::libc::c_ulong) << 16
            | (b'u' as nix::libc::c_ulong) << 8
            | 0x21;
        // SAFETY: repr(C) matches uvc_xu_control_query; the live writable four-byte
        // buffer and argument outlive this synchronous ioctl. Kernel receives size=4.
        let result = unsafe { nix::libc::ioctl(self.0, request, &mut argument) };
        if result < 0 {
            return Err(std::io::Error::last_os_error().into());
        }
        Ok(())
    }
}

pub type DevicePaths = BTreeMap<String, PathBuf>;

/// Discover independently of aliases. Enumeration locates components; Factory assigns roles.
pub fn discover() -> Result<(DevicePaths, DevicePaths), IdentityError> {
    discover_at(Path::new("/sys/class"), Path::new("/dev"), camera_uid)
}

fn discover_at(
    classes: &Path,
    devices: &Path,
    read_camera: impl FnMut(&Path) -> Result<String, IdentityError>,
) -> Result<(DevicePaths, DevicePaths), IdentityError> {
    let camera_nodes = candidates(classes, devices, "video4linux", ("32e4", "9230"))?;
    let encoder_nodes = candidates(classes, devices, "tty", ("303a", "1001"))?;
    let (cameras, busy) = identify(camera_nodes.iter().map(PathBuf::as_path), read_camera)?;
    let (encoders, _) = identify(encoder_nodes.iter().map(PathBuf::as_path), |node| {
        let parent = class_usb_parent(&classes.join("tty").join(node.file_name().unwrap()))?
            .ok_or_else(|| {
                IdentityError::Unsupported("encoder disconnected during discovery".into())
            })?;
        encoder_serial(&parent)
    })?;
    // The whole rig is judged before a busy camera is: a missing or extra device is the
    // rig's fault, and a camera that is only busy must never hide it.
    if camera_nodes.len() != 2 || encoders.len() != 2 {
        return Err(IdentityError::Incomplete { cameras, encoders });
    }
    if let Some(busy) = busy {
        return Err(busy);
    }
    if cameras.len() != 2 {
        return Err(IdentityError::Incomplete { cameras, encoders });
    }
    Ok((cameras, encoders))
}

fn class_usb_parent(entry: &Path) -> Result<Option<PathBuf>, IdentityError> {
    let device = match fs::canonicalize(entry.join("device")) {
        Ok(device) => device,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(error) => return Err(error.into()),
    };
    Ok(device
        .ancestors()
        .find(|p| p.join("idVendor").exists())
        .map(Path::to_path_buf))
}

fn candidates(
    classes: &Path,
    devices: &Path,
    class: &str,
    model: (&str, &str),
) -> Result<Vec<PathBuf>, IdentityError> {
    let entries = match fs::read_dir(classes.join(class)) {
        Ok(entries) => entries,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(Vec::new()),
        Err(error) => return Err(error.into()),
    };
    let mut nodes = Vec::new();
    for entry in entries {
        let entry = entry?;
        let Some(parent) = class_usb_parent(&entry.path())? else {
            continue;
        };
        let (vendor, product) = usb_model(&parent)?;
        if (vendor.as_str(), product.as_str()) != model {
            continue;
        }
        // This supported UVC interface exposes video at index 0 and metadata at 1.
        // Filter from sysfs before issuing any vendor request; never probe unrelated USB.
        if class == "video4linux"
            && (fs::read_to_string(entry.path().join("index"))?.trim() != "0"
                || !fs::read(parent.join("descriptors"))?
                    .windows(GUID.len())
                    .any(|w| w == GUID))
        {
            continue;
        }
        nodes.push(devices.join(entry.file_name()));
    }
    nodes.sort();
    Ok(nodes)
}

/// Every node's identity. A node that is only busy is set aside and the rest are still
/// read, so the caller can judge the whole rig first; any other failure stops here.
fn identify<'a>(
    nodes: impl IntoIterator<Item = &'a Path>,
    mut read_id: impl FnMut(&Path) -> Result<String, IdentityError>,
) -> Result<(DevicePaths, Option<IdentityError>), IdentityError> {
    let mut devices = BTreeMap::new();
    let mut busy = None;
    for node in nodes {
        let id = match read_id(node) {
            Ok(id) => id,
            Err(source) => {
                let error = IdentityError::Device {
                    path: node.to_path_buf(),
                    source: Box::new(source),
                };
                if error.is_camera_busy() {
                    busy.get_or_insert(error);
                    continue;
                }
                return Err(error);
            }
        };
        if id.trim().is_empty() || id.trim() != id {
            return Err(IdentityError::Unsupported(
                "device identity must be non-empty and have no surrounding whitespace".into(),
            ));
        }
        if devices.insert(id.clone(), node.to_path_buf()).is_some() {
            return Err(IdentityError::Unsupported(format!(
                "duplicate component {id}"
            )));
        }
    }
    Ok((devices, busy))
}

/// Filesystem-only device inventory for cross-crate transport and binding tests.
#[cfg(all(target_os = "linux", any(test, feature = "test-support")))]
pub mod test_support {
    use super::*;
    /// Fake sysfs/devfs; only the flash transport is replaced, not discovery or roles.
    #[cfg(target_os = "linux")]
    pub struct Inventory {
        pub root: tempfile::TempDir,
    }
    impl Default for Inventory {
        fn default() -> Self {
            Self::new()
        }
    }
    #[cfg(target_os = "linux")]
    impl Inventory {
        pub fn new() -> Self {
            Self {
                root: tempfile::tempdir().unwrap(),
            }
        }
        pub fn add(
            &self,
            class: &str,
            name: &str,
            port: &str,
            model: (&str, &str),
            id: &str,
            index: &str,
        ) {
            use std::os::unix::fs::symlink;
            let root = self.root.path();
            let parent = root.join("usb").join(port);
            let entry = root.join("class").join(class).join(name);
            fs::create_dir_all(&parent).unwrap();
            fs::create_dir_all(&entry).unwrap();
            fs::create_dir_all(root.join("dev")).unwrap();
            fs::write(parent.join("idVendor"), model.0).unwrap();
            fs::write(parent.join("idProduct"), model.1).unwrap();
            fs::write(parent.join("serial"), id).unwrap();
            fs::write(parent.join("descriptors"), GUID).unwrap();
            fs::write(entry.join("index"), index).unwrap();
            symlink(&parent, entry.join("device")).unwrap();
            fs::write(root.join("dev").join(name), id).unwrap();
        }
        pub fn discover(&self) -> Result<(DevicePaths, DevicePaths), IdentityError> {
            discover_at(
                &self.root.path().join("class"),
                &self.root.path().join("dev"),
                |p| Ok(fs::read_to_string(p)?),
            )
        }
        pub fn rig(&self, swapped: bool, c: [&str; 4]) {
            let (a, b) = if swapped { ("8", "2") } else { ("2", "8") };
            for (index, id) in [(a, c[0]), (b, c[1])] {
                self.add(
                    "video4linux",
                    &format!("video{index}"),
                    &format!("camera-port-{index}"),
                    ("32e4", "9230"),
                    id,
                    "0",
                );
            }
            for (index, id) in [(b, c[2]), (a, c[3])] {
                self.add(
                    "tty",
                    &format!("ttyACM{index}"),
                    &format!("encoder-port-{index}"),
                    ("303a", "1001"),
                    id.strip_prefix("usb-serial:").unwrap(),
                    "0",
                );
            }
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    #[cfg(target_os = "linux")]
    fn lock_probe(path: &Path) -> nix::Result<()> {
        use std::os::fd::AsRawFd;
        let file = File::open(path).unwrap();
        #[allow(deprecated)]
        let result = nix::fcntl::flock(
            file.as_raw_fd(),
            nix::fcntl::FlockArg::LockExclusiveNonblock,
        );
        result
    }

    /// The field failure: a stop's teardown still held the camera when the next reader
    /// came. The reader now waits for the holder instead of failing on the first EAGAIN.
    #[cfg(target_os = "linux")]
    #[test]
    fn camera_lock_waits_for_a_holder_that_lets_go() {
        use std::os::fd::AsRawFd;
        let root = tempfile::tempdir().unwrap();
        let node = root.path().join("video2");
        fs::write(&node, b"").unwrap();
        let holder = File::open(&node).unwrap();
        #[allow(deprecated)]
        nix::fcntl::flock(
            holder.as_raw_fd(),
            nix::fcntl::FlockArg::LockExclusiveNonblock,
        )
        .unwrap();
        let release = thread::spawn(move || {
            thread::sleep(Duration::from_millis(300));
            drop(holder);
        });
        let reader = File::open(&node).unwrap();
        let started = Instant::now();
        hold_exclusively(reader.as_raw_fd(), &node, Duration::from_secs(3)).unwrap();
        assert!(started.elapsed() >= Duration::from_millis(250));
        release.join().unwrap();
        assert_eq!(lock_probe(&node), Err(nix::errno::Errno::EWOULDBLOCK));
    }

    /// A holder that never lets go is a typed, transient "camera busy", never an I/O
    /// failure that reads as a broken rig; the lock itself still excludes the reader.
    #[cfg(target_os = "linux")]
    #[test]
    fn camera_lock_held_past_the_wait_is_a_typed_busy() {
        use std::os::fd::AsRawFd;
        let root = tempfile::tempdir().unwrap();
        let node = root.path().join("video2");
        fs::write(&node, b"").unwrap();
        let holder = File::open(&node).unwrap();
        #[allow(deprecated)]
        nix::fcntl::flock(
            holder.as_raw_fd(),
            nix::fcntl::FlockArg::LockExclusiveNonblock,
        )
        .unwrap();
        let reader = File::open(&node).unwrap();
        let error =
            hold_exclusively(reader.as_raw_fd(), &node, Duration::from_millis(200)).unwrap_err();
        assert!(matches!(error, IdentityError::Busy { .. }), "{error}");
        let wrapped = IdentityError::Device {
            path: node.clone(),
            source: Box::new(error),
        };
        assert!(wrapped.is_camera_busy());
        assert!(!IdentityError::Unsupported("x".into()).is_camera_busy());
        drop(holder);
    }

    /// A child spawned while the camera is open must not inherit its lock: when the lock
    /// sat on v4l's descriptor, a validator started mid-recording held the camera for
    /// minutes. Another thread's fork may hold any descriptor until its exec closes it —
    /// a moment the reader's wait absorbs — so the child here outlives that by far.
    #[cfg(target_os = "linux")]
    #[test]
    fn camera_lock_does_not_leak_into_spawned_children() {
        let root = tempfile::tempdir().unwrap();
        let node = root.path().join("video2");
        fs::write(&node, b"").unwrap();
        let lock = lock_camera(&node, Duration::from_secs(2)).unwrap();
        assert_eq!(lock_probe(&node), Err(nix::errno::Errno::EWOULDBLOCK));
        let mut child = std::process::Command::new("sleep")
            .arg("30")
            .spawn()
            .unwrap();
        drop(lock);
        let next = lock_camera(&node, Duration::from_secs(2));
        let _ = child.kill();
        let _ = child.wait();
        assert!(next.is_ok(), "the child kept the camera locked");
    }

    #[test]
    fn identify_reads_only_supplied_candidates() {
        let root = tempfile::tempdir().unwrap();
        let left = root.path().join("yubi_left_camera");
        let right = root.path().join("yubi_right_camera");
        fs::write(&left, "left-factory-id").unwrap();
        fs::write(&right, "right-factory-id").unwrap();
        // Neither another serial peripheral nor a non-USB camera is a rig input.
        fs::write(root.path().join("ttyACM0"), "unrelated").unwrap();
        fs::write(root.path().join("video0"), "unrelated").unwrap();
        let mut observed = Vec::new();
        let (devices, _) = identify([left.as_path(), right.as_path()], |node| {
            observed.push(node.to_path_buf());
            Ok(fs::read_to_string(node)?)
        })
        .unwrap();
        assert_eq!(observed, vec![left.clone(), right.clone()]);
        assert_eq!(
            devices,
            BTreeMap::from([
                ("left-factory-id".into(), left),
                ("right-factory-id".into(), right)
            ])
        );
    }

    #[test]
    fn supported_device_failures_and_duplicate_identities_are_not_skipped() {
        let nodes = [
            Path::new("/dev/yubi_left_esp32c6"),
            Path::new("/dev/yubi_right_esp32c6"),
        ];
        let error = identify(nodes, |_| Ok("same-factory-id".into())).unwrap_err();
        assert!(error.to_string().contains("duplicate component"));
        for malformed in [false, true] {
            let mut calls = 0;
            let error = identify(nodes, |_| {
                calls += 1;
                if calls == 1 {
                    return Ok("valid-factory-id".into());
                }
                if malformed {
                    Err(IdentityError::Unsupported(
                        "malformed configured device".into(),
                    ))
                } else {
                    Err(std::io::Error::from(std::io::ErrorKind::NotFound).into())
                }
            })
            .unwrap_err();
            assert_eq!(calls, 2);
            assert!(error.to_string().contains("/dev/yubi_right_esp32c6"));
            if malformed {
                assert!(matches!(error, IdentityError::Device { source, .. }
                    if matches!(*source, IdentityError::Unsupported(_))));
            } else {
                assert!(matches!(error, IdentityError::Device { source, .. }
                    if matches!(*source, IdentityError::Io(_))));
            }
        }
    }

    /// A camera that is only busy never hides the rest of the rig: a missing encoder is
    /// reported as the incomplete rig it is, and a later camera's real failure as itself.
    /// Busy is the answer only when it is the rig's one problem.
    #[cfg(target_os = "linux")]
    #[test]
    fn a_busy_camera_is_reported_only_when_nothing_else_is_wrong() {
        let busy = |p: &Path| -> Result<String, IdentityError> {
            if p.ends_with("video2") {
                Err(IdentityError::Busy {
                    path: p.to_path_buf(),
                    waited: CAMERA_LOCK_WAIT,
                })
            } else {
                Ok(fs::read_to_string(p)?)
            }
        };
        let c = components();
        let inventory = Inventory::new();
        inventory.rig(
            false,
            [
                &c.left_camera,
                &c.right_camera,
                &c.left_encoder,
                &c.right_encoder,
            ],
        );
        let classes = inventory.root.path().join("class");
        let devices = inventory.root.path().join("dev");
        let error = discover_at(&classes, &devices, busy).unwrap_err();
        assert!(error.is_camera_busy(), "{error}");

        let incomplete = Inventory::new();
        incomplete.rig(
            false,
            [
                &c.left_camera,
                &c.right_camera,
                &c.left_encoder,
                &c.right_encoder,
            ],
        );
        fs::remove_dir_all(incomplete.root.path().join("class/tty/ttyACM8")).unwrap();
        let error = discover_at(
            &incomplete.root.path().join("class"),
            &incomplete.root.path().join("dev"),
            busy,
        )
        .unwrap_err();
        assert!(
            matches!(error, IdentityError::Incomplete { ref encoders, .. } if encoders.len() == 1),
            "{error}"
        );

        let error = discover_at(&classes, &devices, |p| {
            if p.ends_with("video8") {
                Err(IdentityError::Unsupported(
                    "not the supported Sonix camera".into(),
                ))
            } else {
                busy(p)
            }
        })
        .unwrap_err();
        assert!(!error.is_camera_busy(), "{error}");
        assert!(error.to_string().contains("video8"), "{error}");
    }

    #[cfg(target_os = "linux")]
    use super::test_support::Inventory;
    #[cfg(target_os = "linux")]
    fn components() -> Components {
        Components {
            left_camera: "0123456789abcdef0123456789abcdef".into(),
            right_camera: "fedcba9876543210fedcba9876543210".into(),
            left_encoder: "usb-serial:58:E6:C5:14:13:B4".into(),
            right_encoder: "usb-serial:58:E6:C5:13:EE:88".into(),
        }
    }
    #[cfg(target_os = "linux")]
    struct Components {
        left_camera: String,
        right_camera: String,
        left_encoder: String,
        right_encoder: String,
    }
    #[test]
    #[cfg(target_os = "linux")]
    fn discovery_ignores_aliases_metadata_and_unrelated_peripherals_before_probing() {
        let inventory = Inventory::new();
        let c = components();
        inventory.rig(
            false,
            [
                &c.left_camera,
                &c.right_camera,
                &c.left_encoder,
                &c.right_encoder,
            ],
        );
        inventory.add(
            "video4linux",
            "video0",
            "webcam",
            ("1234", "5678"),
            "do not probe",
            "0",
        );
        inventory.add(
            "tty",
            "ttyACM0",
            "serial",
            ("1234", "5678"),
            "do not open",
            "0",
        );
        inventory.add(
            "video4linux",
            "video3",
            "camera-port-2",
            ("32e4", "9230"),
            &c.left_camera,
            "1",
        );
        // A similar USB model without the supported extension unit is unrelated.
        inventory.add(
            "video4linux",
            "video4",
            "other-sonix",
            ("32e4", "9230"),
            "do not probe",
            "0",
        );
        fs::write(
            inventory.root.path().join("usb/other-sonix/descriptors"),
            b"other",
        )
        .unwrap();
        let dev = inventory.root.path().join("dev");
        std::os::unix::fs::symlink(dev.join("missing"), dev.join("yubi_left_camera")).unwrap();
        std::os::unix::fs::symlink(dev.join("video0"), dev.join("yubi_right_camera")).unwrap();
        let mut probed = Vec::new();
        let (cameras, encoders) = discover_at(&inventory.root.path().join("class"), &dev, |p| {
            probed.push(p.file_name().unwrap().to_str().unwrap().to_owned());
            Ok(fs::read_to_string(p)?)
        })
        .unwrap();
        assert_eq!(probed, ["video2", "video8"]);
        assert_eq!(cameras[&c.left_camera], dev.join("video2"));
        assert_eq!(encoders[&c.left_encoder], dev.join("ttyACM8"));
    }

    #[test]
    #[cfg(target_os = "linux")]
    fn discovery_refuses_missing_duplicate_extra_and_unreadable_supported_components() {
        let c = components();
        for fault in ["missing", "duplicate", "extra", "unreadable", "bad-serial"] {
            let inventory = Inventory::new();
            inventory.rig(
                false,
                [
                    &c.left_camera,
                    &c.right_camera,
                    &c.left_encoder,
                    &c.right_encoder,
                ],
            );
            let root = inventory.root.path();
            match fault {
                "missing" => fs::remove_dir_all(root.join("class/video4linux/video8")).unwrap(),
                "duplicate" => fs::write(root.join("dev/video8"), &c.left_camera).unwrap(),
                "extra" => inventory.add(
                    "tty",
                    "ttyACM9",
                    "extra",
                    ("303a", "1001"),
                    "AA:BB:CC:DD:EE:FF",
                    "0",
                ),
                "unreadable" => fs::remove_file(root.join("dev/video8")).unwrap(),
                "bad-serial" => {
                    fs::write(root.join("usb/encoder-port-8/serial"), "shared-or-empty").unwrap()
                }
                _ => unreachable!(),
            }
            assert!(inventory.discover().is_err(), "{fault}");
        }
        assert!(
            Inventory::new()
                .discover()
                .unwrap_err()
                .to_string()
                .contains("complete registered rig")
        );
    }

    #[test]
    #[cfg(target_os = "linux")]
    fn opened_handle_refuses_replaced_node_or_usb_connection_even_at_the_same_path() {
        use std::os::fd::AsRawFd;
        for replacement in ["node", "usb", "opened-other"] {
            let root = tempfile::tempdir().unwrap();
            let node = root.path().join("video0");
            let usb = root.path().join("usb");
            fs::write(&node, "original").unwrap();
            fs::create_dir(&usb).unwrap();
            let original = File::open(&node).unwrap();
            let connection = Connection {
                node: node.clone(),
                node_file: File::open(&node).unwrap(),
                usb: usb.clone(),
                usb_file: File::open(&usb).unwrap(),
            };
            connection.verify(original.as_raw_fd()).unwrap();
            match replacement {
                "node" => {
                    fs::remove_file(&node).unwrap();
                    fs::write(&node, "substitute").unwrap();
                }
                "usb" => {
                    fs::remove_dir(&usb).unwrap();
                    fs::create_dir(&usb).unwrap();
                }
                "opened-other" => {
                    let other = root.path().join("video1");
                    fs::write(&other, "substitute").unwrap();
                    assert!(
                        connection
                            .verify(File::open(other).unwrap().as_raw_fd())
                            .is_err()
                    );
                    continue;
                }
                _ => unreachable!(),
            }
            assert!(
                connection.verify(original.as_raw_fd()).is_err(),
                "{replacement}"
            );
        }
    }

    #[test]
    #[cfg(target_os = "linux")]
    fn substituted_serial_is_refused_before_its_terminal_settings_change() {
        use nix::sys::termios;
        use std::os::unix::fs::symlink;
        let original = nix::pty::openpty(None, None).unwrap();
        let substitute = nix::pty::openpty(None, None).unwrap();
        let root = tempfile::tempdir().unwrap();
        let node = root.path().join("ttyACM0");
        let mut inherited = termios::tcgetattr(&original.slave).unwrap();
        inherited
            .input_flags
            .insert(termios::InputFlags::IXOFF | termios::InputFlags::INPCK);
        inherited.input_flags.remove(termios::InputFlags::IGNPAR);
        termios::tcsetattr(&original.slave, termios::SetArg::TCSANOW, &inherited).unwrap();
        let original_path = nix::unistd::ttyname(&original.slave).unwrap();
        symlink(&original_path, &node).unwrap();
        let connection = Connection {
            node: node.clone(),
            node_file: File::open(&node).unwrap(),
            usb: root.path().to_path_buf(),
            usb_file: File::open(root.path()).unwrap(),
        };
        let before = termios::tcgetattr(&substitute.slave).unwrap();
        fs::remove_file(&node).unwrap();
        symlink(nix::unistd::ttyname(&substitute.slave).unwrap(), &node).unwrap();
        assert!(
            connection
                .open_serial()
                .unwrap_err()
                .to_string()
                .contains("reconnected")
        );
        assert_eq!(termios::tcgetattr(&substitute.slave).unwrap(), before);
        // The verified port is configured and usable by the same production method.
        fs::remove_file(&node).unwrap();
        symlink(original_path, &node).unwrap();
        let serial = connection.open_serial().unwrap();
        let configured = termios::tcgetattr(&original.slave).unwrap();
        assert!(!configured.local_flags.contains(termios::LocalFlags::ICANON));
        assert!(!configured.input_flags.intersects(
            termios::InputFlags::IXON | termios::InputFlags::IXOFF | termios::InputFlags::INPCK
        ));
        assert!(configured.input_flags.contains(termios::InputFlags::IGNPAR));
        assert_eq!(serialport::SerialPort::baud_rate(&serial).unwrap(), 115_200);
    }
}
