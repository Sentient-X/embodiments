//! Attached devices through `nokhwa`: V4L2 on Linux, `AVFoundation` on macOS.
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

use std::sync::mpsc;
use std::time::{Duration, Instant, SystemTime};

use nokhwa::Camera;
use nokhwa::pixel_format::YuyvFormat;
use nokhwa::utils::{
    ApiBackend, CameraFormat, CameraIndex, CameraInfo, FrameFormat, RequestedFormat,
    RequestedFormatType, Resolution,
};
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

/// How many frames may wait for the source before the capture thread blocks (and the device
/// drops the newest).
const QUEUE: usize = 2;

/// One opened device's frames, captured on a thread of its own: a `nokhwa` camera may not
/// leave the thread that opened it, and a [`Device`] is `Send`.
pub struct NokhwaDevice {
    frames: mpsc::Receiver<Result<RawFrame, CaptureError>>,
}

impl Device for NokhwaDevice {
    fn wait(&mut self) -> Result<RawFrame, CaptureError> {
        self.frames.recv().map_err(|_| CaptureError::Ended)?
    }
}

fn frame(camera: &mut Camera, width: u32, height: u32) -> Result<RawFrame, CaptureError> {
    let data = camera
        .frame_raw()
        .map_err(|error| CaptureError::Device(error.to_string()))?
        .into_owned();
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

/// Open the device `request` names at its mode and capture it.
///
/// # Errors
///
/// Returns [`OpenError`] when cameras are not permitted, the device is not attached or cannot
/// be told apart, or it refuses the mode.
pub fn open(request: CaptureRequest) -> Result<UvcStereoSource<NokhwaDevice>, OpenError> {
    let device = find(&request, devices()?)?;
    let (width, height) = request.frame_raster();
    let fps = request.fps;
    let (ready, opened) = mpsc::channel();
    let (sender, frames) = mpsc::sync_channel(QUEUE);
    std::thread::Builder::new()
        .name(format!("uvc-stereo-{}", request.track))
        .spawn(move || {
            let format = CameraFormat::new(Resolution::new(width, height), FrameFormat::YUYV, fps);
            let refuse = |message: String| OpenError::Mode {
                width,
                height,
                fps,
                message,
            };
            let camera = Camera::new(
                CameraIndex::Index(device.index),
                RequestedFormat::new::<YuyvFormat>(RequestedFormatType::Exact(format)),
            )
            .map_err(|error| refuse(error.to_string()))
            .and_then(|camera| {
                let delivered = camera.camera_format();
                if delivered == format {
                    Ok(camera)
                } else {
                    Err(refuse(format!("it opened {delivered}")))
                }
            })
            .and_then(|mut camera| {
                camera
                    .open_stream()
                    .map(|()| camera)
                    .map_err(|error| OpenError::Device(error.to_string()))
            });
            let mut camera = match camera {
                Ok(camera) => {
                    let _ = ready.send(Ok(()));
                    camera
                }
                Err(error) => {
                    let _ = ready.send(Err(error));
                    return;
                }
            };
            loop {
                let next = frame(&mut camera, width, height);
                let failed = next.is_err();
                // The source was dropped, or the device failed: either ends the capture.
                if sender.send(next).is_err() || failed {
                    break;
                }
            }
        })
        .map_err(|error| OpenError::Device(error.to_string()))?;
    opened
        .recv()
        .map_err(|_| OpenError::Device("the capture thread ended while opening".to_owned()))??;
    Ok(UvcStereoSource::new(NokhwaDevice { frames }, request))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::product::PRODUCTS;

    fn request() -> CaptureRequest {
        CaptureRequest {
            track: "head".to_owned(),
            product: &PRODUCTS[0],
            serial: "30371653".to_owned(),
            eye_width: 672,
            eye_height: 376,
            fps: 15,
            cameras: ["head_left".to_owned(), "head_right".to_owned()],
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
}
