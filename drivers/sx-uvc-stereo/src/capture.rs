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

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, mpsc};
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
        message: String,
    },
    #[error("the device failed to open: {0}")]
    Device(String),
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

/// How long the last source of a device waits for its capture thread to stop the stream and
/// close the device: longer than [`FIRST_FRAME_WAIT`], the longest a read waits, with room to
/// spare.
pub const STOP_WAIT: Duration = Duration::from_secs(4);

/// No frame for this long ends a capture: the device is reported failed, its stream stopped and
/// the device closed. A read that waited for ever held a stalled ZED 2i open on the DGX Spark
/// (10-27): the capture thread never saw it was told to stop, the device stayed open, and every
/// later interval's open was refused as busy.
pub const STALL: Duration = Duration::from_secs(1);

/// How long the first frame after the stream starts may take.
pub const FIRST_FRAME_WAIT: Duration = Duration::from_secs(3);

/// One opened device's frames, captured on a thread of its own: a `nokhwa` camera may not
/// leave the thread that opened it, and a [`Device`] is `Send`.
pub struct CaptureDevice {
    frames: mpsc::Receiver<Result<Arc<RawFrame>, CaptureError>>,
    _capture: Arc<CaptureThread>,
}

/// The capture thread of one opened device, shared by its tracks' sources. When the last
/// source drops, it is told to stop and waited for (at most [`STOP_WAIT`]); it sees that after
/// the frame it is reading or at most [`STALL`]. The thread stops
/// the stream (`VIDIOC_STREAMOFF`, the buffers released) and closes the device itself, so a
/// process that ends after its sources are dropped never leaves the camera mid-capture. A
/// stream torn down by process exit instead, during a transfer, hung the DGX Spark's xHCI
/// controller until a reboot ("xHCI host not responding to stop endpoint command", 10-27).
struct CaptureThread {
    stop: Arc<AtomicBool>,
    thread: Mutex<Option<JoinHandle<()>>>,
    serial: String,
}

impl Drop for CaptureThread {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        let Some(thread) = self.thread.lock().ok().and_then(|mut thread| thread.take()) else {
            return;
        };
        let deadline = Instant::now() + STOP_WAIT;
        while !thread.is_finished() {
            if Instant::now() >= deadline {
                eprintln!(
                    "uvc-stereo {}: the capture thread did not stop within {STOP_WAIT:?}; left running",
                    self.serial
                );
                return;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        let _ = thread.join();
    }
}

impl Device for CaptureDevice {
    fn wait(&mut self) -> Result<Arc<RawFrame>, CaptureError> {
        self.frames.recv().map_err(|_| CaptureError::Ended)?
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

/// Hand every frame to every track's queue until told to stop, every track's source has
/// dropped, or the device fails.
fn serve(
    stream: &mut Stream,
    mut senders: Vec<FrameSender>,
    (width, height): (u32, u32),
    stopping: &AtomicBool,
) {
    // Told to stop, every track's source dropped, or the device failed: each ends the
    // capture, and the stream is stopped and the device closed here, on this thread.
    while !senders.is_empty() && !stopping.load(Ordering::Acquire) {
        match frame(stream, width, height) {
            Ok(frame) => {
                let frame = Arc::new(frame);
                senders.retain(|sender| {
                    !matches!(
                        sender.try_send(Ok(Arc::clone(&frame))),
                        Err(mpsc::TrySendError::Disconnected(_))
                    )
                });
            }
            Err(error) => {
                for sender in &senders {
                    // Never block on a track that stopped reading: the device closes now.
                    let _ = sender.try_send(Err(error.clone()));
                }
                break;
            }
        }
    }
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
        let refuse = |message: String| OpenError::Mode {
            width,
            height,
            fps,
            message,
        };
        let failed = |error: std::io::Error| OpenError::Device(error.to_string());
        let device = v4l::Device::new(index as usize).map_err(failed)?;
        let yuyv = v4l::FourCC::new(b"YUYV");
        let mut format = device.format().map_err(failed)?;
        format.width = width;
        format.height = height;
        format.fourcc = yuyv;
        let set = device
            .set_format(&format)
            .map_err(|error| refuse(error.to_string()))?;
        if (set.width, set.height, set.fourcc) != (width, height, yuyv) {
            return Err(refuse(format!(
                "it opened {}x{} {}",
                set.width, set.height, set.fourcc
            )));
        }
        let interval = device
            .set_params(&v4l::video::capture::Parameters::with_fps(fps))
            .map_err(|error| refuse(error.to_string()))?
            .interval;
        if interval.numerator == 0 || interval.denominator != fps * interval.numerator {
            return Err(refuse(format!(
                "it opened a frame interval of {}/{} s",
                interval.numerator, interval.denominator
            )));
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
        let refuse = |message: String| OpenError::Mode {
            width,
            height,
            fps,
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
            .map_err(|error| OpenError::Device(error.to_string()))?;
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
        return Err(OpenError::Device(
            "the requests are not one device at one mode".to_owned(),
        ));
    }
    let device = find(first, devices()?)?;
    let (width, height) = first.frame_raster();
    let fps = first.fps;
    let serial = first.serial.clone();
    let (ready, opened) = mpsc::channel();
    let (senders, receivers): (Vec<_>, Vec<_>) =
        requests.iter().map(|_| mpsc::sync_channel(QUEUE)).unzip();
    let stop = Arc::new(AtomicBool::new(false));
    let stopping = Arc::clone(&stop);
    let thread_serial = serial.clone();
    let thread = std::thread::Builder::new()
        .name(format!("uvc-stereo-{serial}"))
        .spawn(move || {
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
            serve(&mut stream, senders, (width, height), &stopping);
            stream.close(&thread_serial);
        })
        .map_err(|error| OpenError::Device(error.to_string()))?;
    let capture = Arc::new(CaptureThread {
        stop,
        thread: Mutex::new(Some(thread)),
        serial,
    });
    opened
        .recv()
        .map_err(|_| OpenError::Device("the capture thread ended while opening".to_owned()))??;
    Ok(requests
        .into_iter()
        .zip(receivers)
        .map(|(request, frames)| {
            UvcStereoSource::new(
                CaptureDevice {
                    frames,
                    _capture: Arc::clone(&capture),
                },
                request,
            )
        })
        .collect())
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

    /// The last source dropping stops the capture thread and waits for it to end, so the
    /// device is closed before the caller goes on; a thread that will not stop is left after
    /// the wait rather than hanging its caller.
    #[test]
    fn the_last_source_stops_and_waits_for_the_capture_thread() {
        let stop = Arc::new(AtomicBool::new(false));
        let seen = Arc::clone(&stop);
        let closed = Arc::new(AtomicBool::new(false));
        let device_closed = Arc::clone(&closed);
        let thread = std::thread::spawn(move || {
            while !seen.load(Ordering::Acquire) {
                std::thread::sleep(Duration::from_millis(1));
            }
            std::thread::sleep(Duration::from_millis(20));
            device_closed.store(true, Ordering::Release);
        });
        let capture = Arc::new(CaptureThread {
            stop,
            thread: Mutex::new(Some(thread)),
            serial: "test".to_owned(),
        });
        let other = Arc::clone(&capture);
        drop(capture);
        assert!(
            !closed.load(Ordering::Acquire),
            "another source still holds it"
        );
        drop(other);
        assert!(
            closed.load(Ordering::Acquire),
            "closed before the last drop returned"
        );
    }
}
