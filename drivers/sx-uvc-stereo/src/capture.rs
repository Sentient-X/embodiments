//! Attached devices: listed through `nokhwa`, captured through V4L2 itself on Linux (the `v4l`
//! crate `nokhwa` uses, so a read can wait with a timeout) and through `nokhwa`'s
//! `AVFoundation` on macOS.
//!
//! A device is found by its product's USB vendor and product ids, and by serial where the
//! platform says it. On Linux the serial is read from `sysfs`, so the request's serial must match
//! exactly and only the device's capture node (`index` 0) is opened. A product whose video
//! interface reports a placeholder serial (`Product::serial_product_id`: the ZED 2i says
//! `OV0001`) takes the serial of the device that carries it: the one of that product id on the
//! same host controller whose port chain is the video device's plus one hub port (a USB 3
//! connector's USB 2 half, behind the camera's own hub). On macOS
//! `AVFoundation` names a UVC camera by location, vendor and product id but not by serial, so the
//! one attached device of the product is opened and two are refused as ambiguous.
//!
//! The device is asked for exactly the request's side-by-side raster, YUYV, at its rate; a mode
//! the link cannot carry (any but VGA through USB 2) is the device's refusal, reported as such.
//! On Linux every stream starts on a device just reset on its USB port ([`RESET_WAIT`]).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError, mpsc};
use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime};

use nokhwa::utils::{ApiBackend, CameraIndex, CameraInfo};
use sx_embodiment_drivers::{CaptureError, CapturedAt};
use thiserror::Error;

use crate::source::{CaptureRequest, Device, RawFrame, UvcStereoSource};

/// How long a first-time camera permission request may wait for the person at the machine.
pub const PERMISSION_WAIT: Duration = Duration::from_secs(10);

/// A device that could not be opened for a request.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum OpenError {
    #[error(
        "this process may not use cameras: grant the app that runs it camera access (macOS \
         Privacy & Security > Camera)"
    )]
    Permission,
    #[error("listing cameras failed: {0}")]
    Query(String),
    #[error(
        "no {vendor_id:04x}:{product_id:04x} device with serial {serial} is attached; attached: {attached:?}"
    )]
    NotFound {
        vendor_id: u16,
        product_id: u16,
        serial: String,
        attached: Vec<Attached>,
    },
    #[error(
        "{count} {vendor_id:04x}:{product_id:04x} devices are attached and this platform cannot tell them apart by serial"
    )]
    Ambiguous {
        vendor_id: u16,
        product_id: u16,
        count: usize,
    },
    #[error("the device refused {width}x{height} YUYV at {fps} fps: {message}")]
    Mode {
        width: u32,
        height: u32,
        fps: u32,
        /// Why, as the platform said it: held by another process, forbidden, or the mode itself.
        kind: Unavailable,
        message: String,
    },
    #[error("the device failed to open: {message}")]
    Device { kind: Unavailable, message: String },
}

/// Why a camera is not streaming, typed where the platform says it so no caller reads an
/// error's words.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unavailable {
    /// No such device is attached, or it went away.
    NotAttached,
    /// Another process (or a capture of this one that has not closed it yet) holds it.
    Busy,
    /// This process may not open it.
    PermissionDenied,
    /// It cannot do what was asked: the mode, or a platform that cannot tell units apart.
    Unsupported,
    /// It opened and sends no pictures.
    NoFrames,
    /// Anything else the platform reported.
    Failed,
}

impl Unavailable {
    /// The kind of an operating-system error, or `otherwise` (V4L2 reports one; `AVFoundation`
    /// through `nokhwa` does not).
    #[cfg(any(target_os = "linux", test))]
    fn of(error: &std::io::Error, otherwise: Self) -> Self {
        match error.kind() {
            std::io::ErrorKind::ResourceBusy => Self::Busy,
            std::io::ErrorKind::PermissionDenied => Self::PermissionDenied,
            std::io::ErrorKind::NotFound => Self::NotAttached,
            // ENXIO and ENODEV: the device went away.
            _ if matches!(error.raw_os_error(), Some(6 | 19)) => Self::NotAttached,
            _ => otherwise,
        }
    }
}

impl OpenError {
    /// Why the camera is not streaming.
    #[must_use]
    pub const fn unavailable(&self) -> Unavailable {
        match self {
            Self::Permission => Unavailable::PermissionDenied,
            Self::Query(_) => Unavailable::Failed,
            Self::NotFound { .. } => Unavailable::NotAttached,
            Self::Ambiguous { .. } => Unavailable::Unsupported,
            Self::Mode { kind, .. } | Self::Device { kind, .. } => *kind,
        }
    }

    fn device(kind: Unavailable, message: impl Into<String>) -> Self {
        Self::Device {
            kind,
            message: message.into(),
        }
    }
}

/// One attached camera as the platform names it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Attached {
    pub index: u32,
    pub name: String,
    pub vendor_id: Option<u16>,
    pub product_id: Option<u16>,
    /// The USB serial, where the platform says it.
    pub serial: Option<String>,
    /// Whether this is the device's capture node rather than a metadata one (Linux).
    pub capture: bool,
}

fn permitted() -> bool {
    if nokhwa::nokhwa_check() {
        return true;
    }
    let (sender, receiver) = mpsc::channel();
    nokhwa::nokhwa_initialize(move |granted| {
        // The receiver may have given up; nothing then waits for the answer.
        let _ = sender.send(granted);
    });
    receiver.recv_timeout(PERMISSION_WAIT).unwrap_or(false) && nokhwa::nokhwa_check()
}

#[cfg(target_os = "linux")]
fn identify(info: &CameraInfo, index: u32) -> Attached {
    use std::fs;
    use std::path::Path;

    let node = Path::new("/sys/class/video4linux").join(format!("video{index}"));
    let usb = fs::canonicalize(node.join("device"))
        .ok()
        .and_then(|interface| interface.parent().map(Path::to_path_buf));
    let read = |name: &str| {
        usb.as_ref()
            .and_then(|dir| fs::read_to_string(dir.join(name)).ok())
            .map(|text| text.trim().to_owned())
    };
    let hex = |name: &str| read(name).and_then(|text| u16::from_str_radix(&text, 16).ok());
    let capture = fs::read_to_string(node.join("index"))
        .map(|text| text.trim() == "0")
        .unwrap_or(false);
    let (vendor_id, product_id) = (hex("idVendor"), hex("idProduct"));
    let carrier = crate::PRODUCTS
        .iter()
        .find(|product| {
            Some(product.vendor_id) == vendor_id && Some(product.product_id) == product_id
        })
        .and_then(|product| product.serial_product_id.map(|id| (product.vendor_id, id)));
    let serial = match (carrier, usb.as_deref()) {
        (Some((vendor, product)), Some(video)) => usb_devices()
            .into_iter()
            .find(|(path, ids, _)| *ids == (vendor, product) && serial_carrier(video, path))
            .and_then(|(_, _, serial)| serial),
        _ => read("serial"),
    };
    Attached {
        index,
        name: info.human_name(),
        vendor_id,
        product_id,
        serial,
        capture,
    }
}

/// Every USB device `sysfs` lists: its canonical path, vendor and product ids, and serial.
#[cfg(target_os = "linux")]
fn usb_devices() -> Vec<(std::path::PathBuf, (u16, u16), Option<String>)> {
    use std::fs;

    let Ok(entries) = fs::read_dir("/sys/bus/usb/devices") else {
        return Vec::new();
    };
    entries
        .filter_map(Result::ok)
        .filter_map(|entry| {
            let path = fs::canonicalize(entry.path()).ok()?;
            let read = |name: &str| {
                fs::read_to_string(path.join(name))
                    .ok()
                    .map(|text| text.trim().to_owned())
            };
            let hex = |name: &str| read(name).and_then(|text| u16::from_str_radix(&text, 16).ok());
            let ids = (hex("idVendor")?, hex("idProduct")?);
            let serial = read("serial");
            Some((path, ids, serial))
        })
        .collect()
}

/// A USB device's host controller (the directory above its `usbN` root) and its port chain
/// (`6-1.3` is root port 1, then hub port 3).
#[cfg(any(target_os = "linux", test))]
fn controller_and_ports(device: &std::path::Path) -> Option<(&std::path::Path, Vec<&str>)> {
    let name = device.file_name()?.to_str()?;
    let (_, ports) = name.split_once('-')?;
    if ports.contains(':') {
        return None;
    }
    let root = device
        .ancestors()
        .find(|dir| {
            dir.file_name()
                .and_then(|name| name.to_str())
                .is_some_and(|name| name.starts_with("usb") && !name.contains('-'))
        })?
        .parent()?;
    Some((root, ports.split('.').collect()))
}

/// Whether `carrier` sits on the same connector as `video`: the same host controller, and its
/// port chain the video device's plus the camera hub's one port.
#[cfg(any(target_os = "linux", test))]
fn serial_carrier(video: &std::path::Path, carrier: &std::path::Path) -> bool {
    match (controller_and_ports(video), controller_and_ports(carrier)) {
        (Some((video_root, video_ports)), Some((carrier_root, carrier_ports))) => {
            video_root == carrier_root
                && carrier_ports.len() == video_ports.len() + 1
                && carrier_ports.starts_with(&video_ports)
        }
        _ => false,
    }
}

#[cfg(not(target_os = "linux"))]
fn identify(info: &CameraInfo, index: u32) -> Attached {
    // AVFoundation's unique id for a UVC camera is 0x<location id><vendor id><product id>.
    let unique = u64::from_str_radix(info.misc().trim_start_matches("0x"), 16).ok();
    let field = |shift: u32| unique.and_then(|id| u16::try_from((id >> shift) & 0xffff).ok());
    Attached {
        index,
        name: info.human_name(),
        vendor_id: field(16),
        product_id: field(0),
        serial: None,
        capture: true,
    }
}

/// Every camera attached, as this platform names it.
///
/// # Errors
///
/// Returns [`OpenError::Permission`] when this process may not use cameras, and
/// [`OpenError::Query`] when listing fails.
pub fn devices() -> Result<Vec<Attached>, OpenError> {
    if !permitted() {
        return Err(OpenError::Permission);
    }
    let listed =
        nokhwa::query(ApiBackend::Auto).map_err(|error| OpenError::Query(error.to_string()))?;
    Ok(listed
        .iter()
        .filter_map(|info| match info.index() {
            CameraIndex::Index(index) => Some(identify(info, *index)),
            CameraIndex::String(_) => None,
        })
        .collect())
}

/// The attached device a request names.
fn find(request: &CaptureRequest, attached: Vec<Attached>) -> Result<Attached, OpenError> {
    let product = request.product;
    let candidates: Vec<&Attached> = attached
        .iter()
        .filter(|device| {
            device.capture
                && device.vendor_id == Some(product.vendor_id)
                && device.product_id == Some(product.product_id)
        })
        .collect();
    let by_serial: Vec<&Attached> = candidates
        .iter()
        .copied()
        .filter(|device| device.serial.as_deref() == Some(request.serial.as_str()))
        .collect();
    let unnamed = candidates.iter().all(|device| device.serial.is_none());
    match (by_serial.as_slice(), candidates.as_slice()) {
        ([one], _) => Ok((*one).clone()),
        ([], [one]) if unnamed => Ok((*one).clone()),
        ([], many) if unnamed && many.len() > 1 => Err(OpenError::Ambiguous {
            vendor_id: product.vendor_id,
            product_id: product.product_id,
            count: many.len(),
        }),
        _ => Err(OpenError::NotFound {
            vendor_id: product.vendor_id,
            product_id: product.product_id,
            serial: request.serial.clone(),
            attached,
        }),
    }
}

/// How many frames may wait for a track's source; a track that falls further behind misses
/// frames rather than holding back the other tracks of its device.
const QUEUE: usize = 2;

/// How long [`close_all`] waits for each capture thread to stop its stream and close its device:
/// longer than [`FIRST_FRAME_WAIT`], the longest a read waits, with room to spare.
pub const STOP_WAIT: Duration = Duration::from_secs(4);

/// No frame for this long ends a capture: the device is reported failed, its stream stopped and
/// the device closed. A read that waited for ever held a stalled ZED 2i open on the DGX Spark
/// (10-27): the capture thread never saw it was told to stop, the device stayed open, and every
/// later interval's open was refused as busy.
pub const STALL: Duration = Duration::from_secs(1);

/// How long the first frame after the stream starts may take.
pub const FIRST_FRAME_WAIT: Duration = Duration::from_secs(3);

/// One track's frames from its device's running capture. Dropping it unsubscribes the track;
/// the device keeps streaming for the next one ([`open`]).
pub struct CaptureDevice {
    frames: mpsc::Receiver<Result<Arc<RawFrame>, CaptureError>>,
}

impl Device for CaptureDevice {
    fn wait(&mut self) -> Result<Arc<RawFrame>, CaptureError> {
        self.frames.recv().map_err(|_| CaptureError::Ended)?
    }
}

/// The tracks a running capture hands its frames to.
#[derive(Default)]
struct Subscribers {
    senders: Vec<FrameSender>,
    /// The capture failed or was stopped: it takes no new track, and its device is closed
    /// (or closing) on its thread.
    ended: bool,
}

/// One device opened once and kept streaming for the process's life, on a thread of its own (a
/// `nokhwa` camera may not leave the thread that opened it). Tracks subscribe and unsubscribe
/// as watch and teleoperation intervals come and go; with none, frames are dropped. Stopping
/// and reopening the stream every interval is what hung the DGX Spark's xHCI controller ("xHCI
/// host not responding to stop endpoint command", 10-27) and cost a reconnect each time; now
/// the stream stops only when the device fails (no frame within [`STALL`]) or [`close_all`]
/// stops it as the process ends.
struct Running {
    serial: String,
    mode: ((u32, u32), u32),
    subscribers: Mutex<Subscribers>,
    stop: AtomicBool,
    thread: Mutex<Option<JoinHandle<()>>>,
}

impl Running {
    fn new(serial: String, mode: ((u32, u32), u32)) -> Self {
        Self {
            serial,
            mode,
            subscribers: Mutex::new(Subscribers::default()),
            stop: AtomicBool::new(false),
            thread: Mutex::new(None),
        }
    }

    /// Hand `senders` this capture's frames, unless it has ended.
    fn subscribe(&self, senders: Vec<FrameSender>) -> Result<(), Vec<FrameSender>> {
        let mut subscribers = held(&self.subscribers);
        if subscribers.ended {
            return Err(senders);
        }
        subscribers.senders.extend(senders);
        Ok(())
    }

    /// Every live track gets the frame; a track whose source dropped is unsubscribed, and one
    /// that fell behind misses it.
    fn publish(&self, frame: &Arc<RawFrame>) {
        held(&self.subscribers).senders.retain(|sender| {
            !matches!(
                sender.try_send(Ok(Arc::clone(frame))),
                Err(mpsc::TrySendError::Disconnected(_))
            )
        });
    }

    /// Take no new track, tell every track why (never blocking on one that stopped reading),
    /// and let them go: their sources see the error, then the capture's end.
    fn end(&self, error: Option<&CaptureError>) {
        let mut subscribers = held(&self.subscribers);
        subscribers.ended = true;
        for sender in subscribers.senders.drain(..) {
            if let Some(error) = error {
                let _ = sender.try_send(Err(error.clone()));
            }
        }
    }

    /// Stop the capture and wait (at most [`STOP_WAIT`]) for its thread to close the device:
    /// whether it did. A thread that did not is kept, so it can be waited for again.
    fn stop_and_wait(&self) -> bool {
        self.stop.store(true, Ordering::Release);
        let mut slot = held(&self.thread);
        let Some(thread) = slot.take() else {
            return true;
        };
        let deadline = Instant::now() + STOP_WAIT;
        while !thread.is_finished() {
            if Instant::now() >= deadline {
                eprintln!(
                    "uvc-stereo {}: the capture thread did not stop within {STOP_WAIT:?}; left running",
                    self.serial
                );
                *slot = Some(thread);
                return false;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        let _ = thread.join();
        true
    }
}

/// A lock whose holder panicked is still the state it guards: a capture's tracks and thread
/// must be reachable to end and stop it.
fn held<T>(lock: &Mutex<T>) -> MutexGuard<'_, T> {
    lock.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Ends its capture however its thread leaves, a panic included, so no track waits for ever.
struct EndOnExit(Arc<Running>);

impl Drop for EndOnExit {
    fn drop(&mut self) {
        self.0.end(None);
    }
}

/// Every device this process keeps streaming; once closed ([`close_all`]), nothing opens.
struct Registry {
    captures: Vec<Arc<Running>>,
    closed: bool,
}

static RUNNING: Mutex<Registry> = Mutex::new(Registry {
    captures: Vec::new(),
    closed: false,
});

/// Stop every running capture and wait for each to stop its stream and close its device. A
/// station calls it as it ends, after its robot is stopped and off its control thread, so the
/// stream is never torn down by process exit mid-transfer.
pub fn close_all() {
    let captures = {
        let mut registry = held(&RUNNING);
        registry.closed = true;
        std::mem::take(&mut registry.captures)
    };
    for capture in captures {
        capture.end(Some(&CaptureError::Device(
            "the station is ending".to_owned(),
        )));
        capture.stop_and_wait();
    }
}

fn frame(stream: &mut Stream, width: u32, height: u32) -> Result<RawFrame, CaptureError> {
    let data = stream.frame()?;
    let captured = CapturedAt {
        monotonic: Instant::now(),
        wall: SystemTime::now(),
    };
    let rows = height as usize;
    let (stride, data) = rows_of(data, width as usize, rows)?;
    Ok(RawFrame {
        width,
        height,
        stride,
        data,
        captured,
    })
}

/// The row stride of one raw YUYV frame, and the frame trimmed to its rows. Rows may be padded
/// (a size that divides into whole rows of at least `width × 2` bytes), or the platform may hand
/// over trailing bytes after packed rows: `AVFoundation` delivers the ZED 2i's 1344×376 mode as
/// 1,013,376 bytes, 377 packed rows (measured on macOS, 2026-10-25), so the last row is dropped.
fn rows_of(mut data: Vec<u8>, width: usize, rows: usize) -> Result<(usize, Vec<u8>), CaptureError> {
    let packed = width * 2;
    if rows > 0 && data.len().is_multiple_of(rows) && data.len() / rows >= packed {
        let stride = data.len() / rows;
        return Ok((stride, data));
    }
    if rows > 0 && data.len() >= packed * rows && data.len() - packed * rows < packed * 2 {
        data.truncate(packed * rows);
        return Ok((packed, data));
    }
    Err(CaptureError::Device(format!(
        "a {} byte frame is not {rows} rows of at least {packed} bytes",
        data.len()
    )))
}

/// One track's queue of frames.
type FrameSender = mpsc::SyncSender<Result<Arc<RawFrame>, CaptureError>>;

/// Hand every frame to the capture's tracks until it is stopped or the device fails; then it
/// takes no new track, and the caller closes the device.
fn serve(running: &Running, mut next: impl FnMut() -> Result<RawFrame, CaptureError>) {
    while !running.stop.load(Ordering::Acquire) {
        match next() {
            Ok(frame) => running.publish(&Arc::new(frame)),
            Err(error) => {
                running.end(Some(&error));
                return;
            }
        }
    }
    running.end(None);
}

/// One device streaming at one mode, on the thread that opened it.
#[cfg(target_os = "linux")]
struct Stream {
    // Declared before the device: the stream stops and its buffers are released first.
    buffers: v4l::prelude::MmapStream<'static>,
    device: v4l::Device,
    started: bool,
    /// The bytes of one whole frame: a shorter one is dropped, not handed on.
    frame_bytes: usize,
}

#[cfg(target_os = "linux")]
impl Stream {
    fn open(index: u32, (width, height): (u32, u32), fps: u32) -> Result<Self, OpenError> {
        use v4l::video::Capture;
        let refuse = |kind: Unavailable, message: String| OpenError::Mode {
            width,
            height,
            fps,
            kind,
            message,
        };
        let refused = |error: std::io::Error| {
            refuse(
                Unavailable::of(&error, Unavailable::Unsupported),
                error.to_string(),
            )
        };
        let failed = |error: std::io::Error| {
            OpenError::device(
                Unavailable::of(&error, Unavailable::Failed),
                error.to_string(),
            )
        };
        let device = v4l::Device::new(index as usize).map_err(failed)?;
        let yuyv = v4l::FourCC::new(b"YUYV");
        let mut format = device.format().map_err(failed)?;
        format.width = width;
        format.height = height;
        format.fourcc = yuyv;
        let set = device.set_format(&format).map_err(refused)?;
        if (set.width, set.height, set.fourcc) != (width, height, yuyv) {
            return Err(refuse(
                Unavailable::Unsupported,
                format!("it opened {}x{} {}", set.width, set.height, set.fourcc),
            ));
        }
        let interval = device
            .set_params(&v4l::video::capture::Parameters::with_fps(fps))
            .map_err(refused)?
            .interval;
        if interval.numerator == 0 || interval.denominator != fps * interval.numerator {
            return Err(refuse(
                Unavailable::Unsupported,
                format!(
                    "it opened a frame interval of {}/{} s",
                    interval.numerator, interval.denominator
                ),
            ));
        }
        let mut buffers =
            v4l::prelude::MmapStream::with_buffers(&device, v4l::buffer::Type::VideoCapture, 4)
                .map_err(failed)?;
        buffers.set_timeout(FIRST_FRAME_WAIT);
        Ok(Self {
            buffers,
            device,
            started: false,
            frame_bytes: width as usize * height as usize * 2,
        })
    }

    /// The next whole frame's bytes; no frame within [`STALL`] ([`FIRST_FRAME_WAIT`] for the
    /// first) is the device's failure. A frame the driver flags as damaged, or one shorter than
    /// the mode (a USB hiccup), is dropped and the next one read.
    fn frame(&mut self) -> Result<Vec<u8>, CaptureError> {
        use v4l::io::traits::CaptureStream;
        let wait = if self.started {
            STALL
        } else {
            FIRST_FRAME_WAIT
        };
        let frame = loop {
            let (data, meta) = self.buffers.next().map_err(|error| {
                if error.kind() == std::io::ErrorKind::TimedOut {
                    CaptureError::Device(format!("no frame for {wait:?}"))
                } else {
                    CaptureError::Device(error.to_string())
                }
            })?;
            let used =
                usize::try_from(meta.bytesused).map_or(data.len(), |used| used.min(data.len()));
            if !meta.flags.contains(v4l::buffer::Flags::ERROR) && used >= self.frame_bytes {
                break data[..used].to_vec();
            }
        };
        if !self.started {
            self.started = true;
            self.buffers.set_timeout(STALL);
        }
        Ok(frame)
    }

    /// Stop the stream (`VIDIOC_STREAMOFF`), release its buffers and close the device.
    fn close(self, serial: &str) {
        use v4l::io::traits::Stream as _;
        let Self {
            mut buffers,
            device,
            ..
        } = self;
        if let Err(error) = buffers.stop() {
            eprintln!("uvc-stereo {serial}: stopping the stream failed: {error}");
        }
        // `v4l`'s stream panics when stopping fails as it drops; its buffers and the device's
        // descriptor are released by the unwinding all the same.
        let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || drop(buffers)));
        drop(device);
    }
}

/// One device streaming at one mode, on the thread that opened it.
#[cfg(not(target_os = "linux"))]
struct Stream(nokhwa::Camera);

#[cfg(not(target_os = "linux"))]
impl Stream {
    fn open(index: u32, (width, height): (u32, u32), fps: u32) -> Result<Self, OpenError> {
        use nokhwa::pixel_format::YuyvFormat;
        use nokhwa::utils::{
            CameraFormat, FrameFormat, RequestedFormat, RequestedFormatType, Resolution,
        };
        let format = CameraFormat::new(Resolution::new(width, height), FrameFormat::YUYV, fps);
        // `AVFoundation` through `nokhwa` reports no error code: a refusal is the mode's.
        let refuse = |message: String| OpenError::Mode {
            width,
            height,
            fps,
            kind: Unavailable::Unsupported,
            message,
        };
        let mut camera = nokhwa::Camera::new(
            CameraIndex::Index(index),
            RequestedFormat::new::<YuyvFormat>(RequestedFormatType::Exact(format)),
        )
        .map_err(|error| refuse(error.to_string()))?;
        let delivered = camera.camera_format();
        if delivered != format {
            return Err(refuse(format!("it opened {delivered}")));
        }
        camera
            .open_stream()
            .map_err(|error| OpenError::device(Unavailable::Failed, error.to_string()))?;
        Ok(Self(camera))
    }

    fn frame(&mut self) -> Result<Vec<u8>, CaptureError> {
        Ok(self
            .0
            .frame_raw()
            .map_err(|error| CaptureError::Device(error.to_string()))?
            .into_owned())
    }

    /// Stop the stream and close the device.
    fn close(mut self, serial: &str) {
        if let Err(error) = self.0.stop_stream() {
            eprintln!("uvc-stereo {serial}: stopping the stream failed: {error}");
        }
    }
}

/// What the tracks of one capture share: the device and its mode.
fn shared(request: &CaptureRequest) -> (&str, (u32, u32), u32) {
    (&request.serial, request.frame_raster(), request.fps)
}

/// Open the one device `requests` share ([`CaptureRequest::by_device`]) once, at their mode,
/// and capture it for each: every frame is handed to every track, which cuts its own eyes.
///
/// # Errors
///
/// Returns [`OpenError`] when cameras are not permitted, the device is not attached or cannot
/// be told apart, or it refuses the mode, and for requests that are not one device at one mode.
pub fn open(
    requests: Vec<CaptureRequest>,
) -> Result<Vec<UvcStereoSource<CaptureDevice>>, OpenError> {
    let Some(first) = requests.first() else {
        return Ok(Vec::new());
    };
    if requests
        .iter()
        .any(|request| shared(request) != shared(first))
    {
        return Err(OpenError::device(
            Unavailable::Unsupported,
            "the requests are not one device at one mode",
        ));
    }
    let (width, height) = first.frame_raster();
    let mode = ((width, height), first.fps);
    let (senders, receivers): (Vec<_>, Vec<_>) =
        requests.iter().map(|_| mpsc::sync_channel(QUEUE)).unzip();
    // One open at a time: two intervals asking at once share one capture.
    let mut registry = held(&RUNNING);
    if registry.closed {
        return Err(OpenError::device(
            Unavailable::Failed,
            "the cameras are closed: the station is ending",
        ));
    }
    let mut senders = senders;
    if let Some(index) = registry
        .captures
        .iter()
        .position(|capture| capture.serial == first.serial)
    {
        let capture = Arc::clone(&registry.captures[index]);
        if capture.mode == mode {
            match capture.subscribe(senders) {
                Ok(()) => return Ok(sources(requests, receivers)),
                Err(back) => senders = back,
            }
        } else {
            // Another mode: the tracks still on the old one are told why they end.
            capture.end(Some(&CaptureError::Device(format!(
                "the device was reopened at {width}x{height} {} fps",
                first.fps
            ))));
        }
        // Its device is closed before it is opened again; one that will not close keeps it.
        if !capture.stop_and_wait() {
            return Err(OpenError::device(
                Unavailable::Busy,
                "the previous capture has not closed the device yet",
            ));
        }
        registry.captures.swap_remove(index);
    }
    let capture = Arc::new(Running::new(first.serial.clone(), mode));
    start(first, &capture)?;
    if let Err(senders) = capture.subscribe(senders) {
        drop(senders);
        capture.stop_and_wait();
        return Err(OpenError::device(
            Unavailable::NoFrames,
            "the capture ended as it opened",
        ));
    }
    registry.captures.push(capture);
    Ok(sources(requests, receivers))
}

/// How long a reset device may take to come back as a capture node this process may open
/// (on the DGX Spark: 0.16-0.24 s, then udev's group grant).
pub const RESET_WAIT: Duration = Duration::from_secs(5);

/// `device`, reset on its USB port so its stream starts as after power-on, as it is found again.
///
/// A ZED 2i streams once per enumeration. On the DGX Spark (GB10 xHCI `NVDA8000:02`, Linux
/// 6.17, 10-28) any second stream after a clean `VIDIOC_STREAMOFF` and close (another process,
/// the station restarted, 0.1 s or 10 s later, autosuspend on or off) sends no frame, and
/// stopping that frameless bulk stream hangs the controller ("xHCI host not responding to stop
/// endpoint command", "HC died"): the ZED drops off USB until a reboot. A port reset before each
/// stream gave frames on every one of six reopens and after a `kill -9` mid-stream. A reset the
/// process may not make (no usbfs grant from the host) is reported and the device opened as is.
#[cfg(target_os = "linux")]
fn fresh(request: &CaptureRequest, device: Attached) -> Result<Attached, OpenError> {
    if let Err(error) = reset(device.index) {
        eprintln!(
            "uvc-stereo {}: could not reset the device before streaming ({error}); a second \
             stream since it enumerated may send no frame",
            request.serial
        );
        return Ok(device);
    }
    let deadline = Instant::now() + RESET_WAIT;
    loop {
        match devices().and_then(|attached| find(request, attached)) {
            Ok(back) if openable(back.index) => return Ok(back),
            Err(error @ (OpenError::Permission | OpenError::Ambiguous { .. })) => {
                return Err(error);
            }
            _ if Instant::now() >= deadline => {
                return Err(OpenError::device(
                    Unavailable::NotAttached,
                    format!("the device did not come back within {RESET_WAIT:?} of its reset"),
                ));
            }
            _ => std::thread::sleep(Duration::from_millis(50)),
        }
    }
}

#[cfg(not(target_os = "linux"))]
#[allow(clippy::unnecessary_wraps)]
fn fresh(_request: &CaptureRequest, device: Attached) -> Result<Attached, OpenError> {
    Ok(device)
}

/// Reset the USB device that carries video node `index` (`/dev/bus/usb/BBB/DDD`, usbfs).
#[cfg(target_os = "linux")]
fn reset(index: u32) -> std::io::Result<()> {
    use std::fs;
    use std::os::fd::AsRawFd;

    let interface = fs::canonicalize(format!("/sys/class/video4linux/video{index}/device"))?;
    let usb = interface
        .parent()
        .ok_or_else(|| std::io::Error::other("the video node has no USB device"))?;
    let number = |name: &str| -> std::io::Result<u32> {
        fs::read_to_string(usb.join(name))?
            .trim()
            .parse()
            .map_err(std::io::Error::other)
    };
    let node = format!(
        "/dev/bus/usb/{:03}/{:03}",
        number("busnum")?,
        number("devnum")?
    );
    let file = fs::OpenOptions::new().write(true).open(&node)?;
    sx_capture_usb::usb::usbfs::Device::borrow(file.as_raw_fd()).reset()
}

/// Whether this process may open video node `index` now (udev grants a new node's group
/// shortly after it appears). Opening a node starts nothing.
#[cfg(target_os = "linux")]
fn openable(index: u32) -> bool {
    std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open(format!("/dev/video{index}"))
        .is_ok()
}

/// Open the device `request` names on a thread of its own and start serving `capture` from it.
fn start(request: &CaptureRequest, capture: &Arc<Running>) -> Result<(), OpenError> {
    let device = fresh(request, find(request, devices()?)?)?;
    let ((width, height), fps) = capture.mode;
    let (ready, opened) = mpsc::channel();
    let serving = Arc::clone(capture);
    let thread = std::thread::Builder::new()
        .name(format!("uvc-stereo-{}", capture.serial))
        .spawn(move || {
            let ending = EndOnExit(serving);
            let mut stream = match Stream::open(device.index, (width, height), fps) {
                Ok(stream) => {
                    let _ = ready.send(Ok(()));
                    stream
                }
                Err(error) => {
                    let _ = ready.send(Err(error));
                    return;
                }
            };
            serve(&ending.0, || frame(&mut stream, width, height));
            stream.close(&ending.0.serial);
        })
        .map_err(|error| OpenError::device(Unavailable::Failed, error.to_string()))?;
    *held(&capture.thread) = Some(thread);
    opened.recv().map_err(|_| {
        OpenError::device(
            Unavailable::Failed,
            "the capture thread ended while opening",
        )
    })?
}

fn sources(
    requests: Vec<CaptureRequest>,
    receivers: Vec<mpsc::Receiver<Result<Arc<RawFrame>, CaptureError>>>,
) -> Vec<UvcStereoSource<CaptureDevice>> {
    requests
        .into_iter()
        .zip(receivers)
        .map(|(request, frames)| UvcStereoSource::new(CaptureDevice { frames }, request))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::product::{Eye, PRODUCTS};

    fn request() -> CaptureRequest {
        CaptureRequest {
            track: "head".to_owned(),
            product: &PRODUCTS[0],
            serial: "30371653".to_owned(),
            eye_width: 672,
            eye_height: 376,
            fps: 15,
            cameras: vec!["head_left".to_owned(), "head_right".to_owned()],
            eyes: vec![Eye::Left, Eye::Right],
        }
    }

    fn zed(index: u32, serial: Option<&str>, capture: bool) -> Attached {
        Attached {
            index,
            name: "ZED 2i".to_owned(),
            vendor_id: Some(0x2b03),
            product_id: Some(0xf880),
            serial: serial.map(str::to_owned),
            capture,
        }
    }

    #[test]
    fn a_named_serial_picks_its_capture_node_and_refuses_another_unit() {
        let found = find(
            &request(),
            vec![
                zed(0, Some("1"), true),
                zed(2, Some("30371653"), true),
                zed(3, Some("30371653"), false),
            ],
        )
        .expect("found");
        assert_eq!(found.index, 2);
        assert!(matches!(
            find(&request(), vec![zed(0, Some("1"), true)]),
            Err(OpenError::NotFound { .. })
        ));
    }

    #[test]
    fn without_serials_one_device_is_it_and_two_are_ambiguous() {
        assert_eq!(
            find(&request(), vec![zed(1, None, true)])
                .expect("one")
                .index,
            1
        );
        assert!(matches!(
            find(&request(), vec![zed(0, None, true), zed(1, None, true)]),
            Err(OpenError::Ambiguous { count: 2, .. })
        ));
        assert!(matches!(
            find(&request(), vec![]),
            Err(OpenError::NotFound { .. })
        ));
    }

    #[test]
    fn a_zed_takes_the_serial_of_its_sensor_module_on_the_same_connector() {
        use std::path::Path;
        let video = Path::new("/sys/devices/platform/NVDA8000:02/usb6/6-1");
        let sensor = Path::new("/sys/devices/platform/NVDA8000:02/usb5/5-1/5-1.2");
        assert!(serial_carrier(video, sensor));
        // Another connector, another controller, or not directly behind the camera's hub.
        assert!(!serial_carrier(
            video,
            Path::new("/sys/devices/platform/NVDA8000:02/usb5/5-2/5-2.2")
        ));
        assert!(!serial_carrier(
            video,
            Path::new("/sys/devices/platform/NVDA8000:03/usb5/5-1/5-1.2")
        ));
        assert!(!serial_carrier(
            video,
            Path::new("/sys/devices/platform/NVDA8000:02/usb5/5-1/5-1.2/5-1.2.1")
        ));
        // Behind an external hub, both halves carry the hub's port.
        assert!(serial_carrier(
            Path::new("/sys/devices/pci0000:00/0000:00:14.0/usb2/2-3/2-3.1"),
            Path::new("/sys/devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3.1/1-3.1.2")
        ));
    }

    fn raw() -> RawFrame {
        RawFrame {
            width: 2,
            height: 1,
            stride: 4,
            data: vec![0; 4],
            captured: CapturedAt {
                monotonic: Instant::now(),
                wall: SystemTime::now(),
            },
        }
    }

    /// The device keeps streaming as tracks come and go: a dropped track is unsubscribed, the
    /// next one is served by the same capture, and frames with nobody subscribed are dropped.
    /// A failure ends the capture: its tracks see the error, then its end, and it takes no
    /// new track, so the next open starts a fresh one.
    #[test]
    fn tracks_come_and_go_on_one_running_capture_until_it_fails() {
        let capture = Arc::new(Running::new("test".to_owned(), ((2, 1), 15)));
        let (feed, frames) = mpsc::channel::<Result<RawFrame, CaptureError>>();
        let serving = Arc::clone(&capture);
        let thread = std::thread::spawn(move || {
            serve(&serving, || {
                frames.recv().unwrap_or(Err(CaptureError::Ended))
            });
        });

        let (first, watch) = mpsc::sync_channel(QUEUE);
        capture.subscribe(vec![first]).expect("subscribed");
        feed.send(Ok(raw())).expect("feed");
        let mut watch = CaptureDevice { frames: watch };
        watch.wait().expect("the watch's frame");
        drop(watch);
        // Nobody subscribed: the frame is dropped and the capture keeps running.
        feed.send(Ok(raw())).expect("feed");
        let (second, session) = mpsc::sync_channel(QUEUE);
        capture
            .subscribe(vec![second])
            .expect("the same capture serves the next interval");
        let mut session = CaptureDevice { frames: session };
        feed.send(Ok(raw())).expect("feed");
        session.wait().expect("the session's frame");

        feed.send(Err(CaptureError::Device("no frame for 1s".to_owned())))
            .expect("feed");
        thread.join().expect("served");
        // Frames still queued come first (the unsubscribed one may have raced the subscribe).
        let failure = std::iter::from_fn(|| Some(session.wait()))
            .find_map(Result::err)
            .expect("an error");
        assert_eq!(failure, CaptureError::Device("no frame for 1s".to_owned()));
        assert_eq!(session.wait().err(), Some(CaptureError::Ended));
        let (late, _) = mpsc::sync_channel(QUEUE);
        assert!(
            capture.subscribe(vec![late]).is_err(),
            "an ended capture takes no track"
        );
    }

    /// Stopping a capture ends its thread, which closes the device, before the stop returns.
    #[test]
    fn stopping_waits_for_the_capture_thread() {
        let capture = Arc::new(Running::new("test".to_owned(), ((2, 1), 15)));
        let closed = Arc::new(AtomicBool::new(false));
        let serving = Arc::clone(&capture);
        let device_closed = Arc::clone(&closed);
        let thread = std::thread::spawn(move || {
            serve(&serving, || {
                std::thread::sleep(Duration::from_millis(2));
                Ok(raw())
            });
            std::thread::sleep(Duration::from_millis(20));
            device_closed.store(true, Ordering::Release);
        });
        *capture.thread.lock().expect("slot") = Some(thread);
        assert!(capture.stop_and_wait());
        assert!(
            closed.load(Ordering::Acquire),
            "closed before the stop returned"
        );
        let (late, _) = mpsc::sync_channel(QUEUE);
        assert!(capture.subscribe(vec![late]).is_err());
    }

    /// A capture thread that panics still ends its capture: its tracks see the end rather
    /// than waiting for ever, and it takes no new track.
    #[test]
    fn a_panicking_capture_thread_still_ends_its_tracks() {
        let capture = Arc::new(Running::new("test".to_owned(), ((2, 1), 15)));
        let (sender, frames) = mpsc::sync_channel(QUEUE);
        capture.subscribe(vec![sender]).expect("subscribed");
        let serving = Arc::clone(&capture);
        let thread = std::thread::spawn(move || {
            let ending = EndOnExit(serving);
            serve(&ending.0, || panic!("the driver panicked"));
        });
        assert!(thread.join().is_err());
        let mut track = CaptureDevice { frames };
        assert_eq!(track.wait().err(), Some(CaptureError::Ended));
        let (late, _) = mpsc::sync_channel(QUEUE);
        assert!(capture.subscribe(vec![late]).is_err());
    }

    /// An operating-system error is typed by its code, never its words.
    #[test]
    fn an_os_error_is_typed_by_its_code() {
        let of = |code| {
            Unavailable::of(
                &std::io::Error::from_raw_os_error(code),
                Unavailable::Failed,
            )
        };
        assert_eq!(of(16), Unavailable::Busy);
        assert_eq!(of(13), Unavailable::PermissionDenied);
        assert_eq!(of(1), Unavailable::PermissionDenied);
        assert_eq!(of(2), Unavailable::NotAttached);
        assert_eq!(of(19), Unavailable::NotAttached);
        assert_eq!(of(22), Unavailable::Failed);
        assert_eq!(
            OpenError::device(Unavailable::Busy, "held").unavailable(),
            Unavailable::Busy
        );
        assert_eq!(
            OpenError::Permission.unavailable(),
            Unavailable::PermissionDenied
        );
    }
}
