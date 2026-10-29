//! The production [`Port`]: one usbfs device fd handed over by Kotlin.
//!
//! Attach reads the descriptors the kernel cached (no bus traffic) and classifies the device:
//! a wrist camera is 32e4:9230 *and* carries the Sonix extension unit, an encoder is
//! 303a:1001 *and* has CDC-ACM interfaces;
//! anything else is listed and never opened. Interfaces are claimed when first used and
//! released when the port is dropped.

use std::sync::{Arc, Mutex};

use serde_json::{Value, json};
use sx_yubi::camera::ExposureConfig;
use sx_yubi::identity::{CAMERA_MODEL, CAMERA_XU_GUID, ENCODER_MODEL, encoder_identity, flash_uid};

use crate::cdc::{self, AcmInterfaces};
use crate::source::{CameraSource, EncoderSource, ExposureControl, Kind, Port};
use crate::source::{MonotonicClock, SourceFrame};
use crate::usb::Control;
use crate::usb::descriptor::{self, Descriptors};
use crate::usb::usbfs::Device;
use crate::uvc::controls::{self, Entities, ExtensionUnit};
use crate::uvc::descriptors::{FormatKind, UvcCamera};
use crate::uvc::probe;
use crate::uvc::stream::Engine;

/// 640×480 MJPEG at the camera's ~120.1 Hz (interval 513/61612 s in 100 ns units).
pub const SOURCE_INTERVAL_100NS: u32 = 83_263;

pub struct UsbfsPort {
    device: Device,
    clock: MonotonicClock,
    descriptors: Descriptors,
    kind: Kind,
    camera: Option<(UvcCamera, Entities)>,
    acm: Option<AcmInterfaces>,
    /// An encoder's serial link: configured once, its bulk IN read by one reader for as long
    /// as the device is attached.
    link: Option<cdc::CdcLink>,
    claimed: Mutex<Vec<u8>>,
}

impl UsbfsPort {
    /// Classify the device on `fd`; `vendor_id`/`product_id` are what Android reported and
    /// must match the descriptors on the fd.
    pub fn attach(
        fd: i32,
        vendor_id: u16,
        product_id: u16,
        clock: MonotonicClock,
    ) -> Result<Self, String> {
        let device = Device::borrow(fd);
        let bytes = device
            .descriptors()
            .map_err(|e| format!("reading descriptors: {e}"))?;
        let descriptors = descriptor::parse(&bytes).map_err(|e| e.to_string())?;
        let model = (descriptors.device.vendor_id, descriptors.device.product_id);
        if model != (vendor_id, product_id) {
            return Err(format!(
                "fd is {:04x}:{:04x}, not the {vendor_id:04x}:{product_id:04x} it was attached as",
                model.0, model.1
            ));
        }
        descriptors.active().map_err(|e| e.to_string())?;
        let mut port = Self {
            device,
            clock,
            descriptors: descriptors.clone(),
            kind: Kind::Other,
            camera: None,
            acm: None,
            link: None,
            claimed: Mutex::new(Vec::new()),
        };
        match classify(&descriptors)? {
            Classified::Camera(camera, entities) => {
                port.camera = Some((camera, entities));
                port.kind = Kind::Camera;
            }
            Classified::Encoder(acm) => {
                port.acm = Some(acm);
                port.link = Some(cdc::CdcLink::default());
                port.kind = Kind::Encoder;
            }
            Classified::Other => {}
        }
        Ok(port)
    }

    fn claim(&self, interface: u8) -> Result<(), String> {
        let mut claimed = self
            .claimed
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if claimed.contains(&interface) {
            return Ok(());
        }
        self.device
            .claim(interface)
            .map_err(|e| format!("claiming interface {interface}: {e}"))?;
        claimed.push(interface);
        Ok(())
    }
}

/// What a device is to the rig, from its descriptors alone.
pub enum Classified {
    Camera(UvcCamera, Entities),
    Encoder(AcmInterfaces),
    Other,
}

/// Classify a device: the YUBI wrist camera (32e4:9230 with the Sonix extension unit) and
/// the YUBI encoder (303a:1001 with CDC-ACM), nothing else.
pub fn classify(descriptors: &Descriptors) -> Result<Classified, String> {
    let model = (descriptors.device.vendor_id, descriptors.device.product_id);
    let configuration = descriptors.active().map_err(|e| e.to_string())?;
    if model == CAMERA_MODEL
        && let Ok(camera) = UvcCamera::parse(configuration)
        && let Some(extension_unit) = camera.extension_unit(&CAMERA_XU_GUID)
    {
        let entities = Entities {
            interface: camera.control_interface,
            camera_terminal: camera
                .camera_terminal
                .ok_or("camera has no camera terminal")?,
            processing_unit: camera
                .processing_unit
                .ok_or("camera has no processing unit")?,
            extension_unit,
        };
        return Ok(Classified::Camera(camera, entities));
    }
    if model == ENCODER_MODEL
        && let Ok(acm) = AcmInterfaces::find(configuration)
    {
        return Ok(Classified::Encoder(acm));
    }
    Ok(Classified::Other)
}

impl Drop for UsbfsPort {
    fn drop(&mut self) {
        // The link's reader stops (and its last read returns) before the interfaces go.
        drop(self.link.take());
        for interface in self
            .claimed
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .drain(..)
        {
            let _ = self.device.release(interface);
        }
    }
}

fn disconnected(error: &std::io::Error) -> bool {
    matches!(error.raw_os_error(), Some(libc::ENODEV | libc::ESHUTDOWN))
        || error.kind() == std::io::ErrorKind::NotConnected
}

fn source_error(error: std::io::Error) -> crate::source::SourceError {
    if disconnected(&error) {
        crate::source::SourceError::Disconnected(error.to_string())
    } else {
        crate::source::SourceError::Failed(error.to_string())
    }
}

fn io_detail(error: std::io::Error) -> String {
    if disconnected(&error) {
        format!("device disconnected ({error})")
    } else {
        error.to_string()
    }
}

impl Port for UsbfsPort {
    fn kind(&self) -> Kind {
        self.kind
    }

    fn describe(&self) -> Value {
        let device = &self.descriptors.device;
        let interfaces: Vec<Value> = self
            .descriptors
            .configurations
            .first()
            .map(|c| {
                c.alt_settings
                    .iter()
                    .filter(|a| a.alternate == 0)
                    .map(|a| json!({"number": a.number, "class": a.class, "subclass": a.subclass}))
                    .collect()
            })
            .unwrap_or_default();
        json!({
            "vendor_id": format!("{:04x}", device.vendor_id),
            "product_id": format!("{:04x}", device.product_id),
            "usb_version": format!("{:x}.{:02x}", device.usb_version >> 8, device.usb_version & 0xff),
            "class": device.class,
            "interfaces": interfaces,
            "uvc": self.camera.as_ref().map(|(c, _)| json!({
                "version": format!("{:x}.{:02x}", c.uvc_version >> 8, c.uvc_version & 0xff),
                "clock_frequency_hz": c.clock_frequency_hz,
                "iso_alternates": c.iso_alts.len(),
            })),
        })
    }

    fn identify(&self) -> Result<String, String> {
        match self.kind {
            Kind::Camera => {
                let (_, entities) = self.camera.as_ref().ok_or("not a camera")?;
                self.claim(entities.interface)?;
                let unit = ExtensionUnit {
                    control: &self.device,
                    entities: *entities,
                };
                flash_uid(&unit).map_err(|e| e.0)
            }
            Kind::Encoder => {
                let serial = self
                    .device
                    .string_descriptor(self.descriptors.device.serial_index)
                    .map_err(io_detail)?;
                encoder_identity(&serial).map_err(|e| e.0)
            }
            Kind::Other => Err("not a rig component".to_owned()),
        }
    }

    fn port_path(&self) -> Result<String, String> {
        self.device.port_path().map_err(|error| {
            if error.raw_os_error() == Some(libc::ENOTTY) {
                "this kernel lacks USBDEVFS_CONNINFO_EX (Linux 5.5+)".to_owned()
            } else {
                io_detail(error)
            }
        })
    }

    fn open_camera(&self, config: ExposureConfig) -> Result<Box<dyn CameraSource>, String> {
        let (camera, entities) = self.camera.as_ref().ok_or("not a camera")?;
        self.claim(camera.control_interface)?;
        self.claim(camera.streaming_interface)?;
        let (format, frame, interval) = camera
            .mode(FormatKind::Mjpeg, 640, 480, SOURCE_INTERVAL_100NS)
            .ok_or("camera offers no 640x480 MJPEG mode")?;
        let committed = probe::negotiate(
            &self.device,
            camera.streaming_interface,
            camera.uvc_version,
            format,
            frame,
            interval,
        )
        .map_err(|e| format!("probe/commit: {}", io_detail(e)))?;
        let held = controls::apply_exposure(&self.device, entities, config, config.start, true)?;
        if held != config.start {
            return Err(format!(
                "camera holds exposure/gain {}/{} after requesting the start setting",
                held.exposure_100us, held.gain
            ));
        }
        let high_speed = self.device.speed().map_err(io_detail)? >= 3;
        let engine = Engine::start(self.device, camera, &committed, high_speed, self.clock)
            .map_err(|e| format!("streaming: {}", io_detail(e)))?;
        let device = self.device;
        let entities = *entities;
        Ok(Box::new(UsbfsCamera {
            engine: Some(engine),
            exposure: Arc::new(move |setting, manual| {
                controls::apply_exposure(&device, &entities, config, setting, manual)
            }),
            negotiated: json!({
                "format_index": format,
                "frame_index": frame,
                "frame_interval_100ns": committed.frame_interval,
                "max_video_frame_size": committed.max_video_frame_size,
                "max_payload_transfer_size": committed.max_payload_transfer_size,
                "high_speed": high_speed,
            }),
        }))
    }

    fn open_encoder(&self) -> Result<Box<dyn EncoderSource>, String> {
        let acm = self.acm.ok_or("not an encoder")?;
        let link = self.link.as_ref().ok_or("not an encoder")?;
        self.claim(acm.communication)?;
        self.claim(acm.data)?;
        let bulk = Arc::new(DeviceBulk {
            device: self.device,
            endpoint: acm.bulk_in,
        });
        let source = link.open(&self.device, &acm, bulk).map_err(io_detail)?;
        Ok(Box::new(UsbfsEncoder {
            source,
            device: self.device,
            acm,
        }))
    }

    fn release(&self) {
        if let (Some(link), Some(acm)) = (&self.link, &self.acm) {
            link.release(&self.device, acm);
        }
    }
}

struct UsbfsCamera {
    engine: Option<Engine>,
    exposure: ExposureControl,
    negotiated: Value,
}

impl CameraSource for UsbfsCamera {
    fn next(&mut self, out: &mut Vec<SourceFrame>) -> Result<(), crate::source::SourceError> {
        let engine = self
            .engine
            .as_mut()
            .ok_or_else(|| crate::source::SourceError::Failed("camera closed".to_owned()))?;
        engine.pump(100, out).map_err(source_error)
    }

    fn exposure(&self) -> ExposureControl {
        Arc::clone(&self.exposure)
    }

    fn report(&self) -> Value {
        match &self.engine {
            Some(engine) => json!({
                "negotiated": self.negotiated,
                "transfers": engine.decoder.stats,
                "bandwidth": engine.bandwidth,
                "clock": engine.decoder.clock_quality(),
            }),
            None => json!({"negotiated": self.negotiated}),
        }
    }

    fn close(mut self: Box<Self>) {
        if let Some(engine) = self.engine.take() {
            engine.stop();
        }
    }
}

/// An encoder's bulk IN endpoint on its usbfs fd.
struct DeviceBulk {
    device: Device,
    endpoint: u8,
}

impl cdc::BulkIn for DeviceBulk {
    fn read(&self, buffer: &mut [u8], timeout_ms: u32) -> std::io::Result<usize> {
        match self.device.bulk(self.endpoint, buffer, timeout_ms) {
            Ok(count) => Ok(count),
            Err(error) if error.kind() == std::io::ErrorKind::TimedOut => Ok(0),
            Err(error) => Err(error),
        }
    }

    fn clear_halt(&self) -> std::io::Result<()> {
        self.device.clear_halt(self.endpoint)
    }
}

struct UsbfsEncoder {
    source: cdc::LinkSource,
    device: Device,
    acm: AcmInterfaces,
}

impl EncoderSource for UsbfsEncoder {
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, crate::source::SourceError> {
        self.source.read(buffer).map_err(source_error)
    }

    fn diagnostics(&self) -> Option<String> {
        Some(self.source.diagnostics())
    }

    fn reopen(&mut self) -> Result<bool, String> {
        cdc::reopen_lines(&self.device, &self.acm).map_err(io_detail)?;
        Ok(true)
    }

    fn close(self: Box<Self>) {}
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kind_of(bytes: &[u8]) -> &'static str {
        match classify(&descriptor::parse(bytes).unwrap()).unwrap() {
            Classified::Camera(..) => "camera",
            Classified::Encoder(_) => "encoder",
            Classified::Other => "other",
        }
    }

    fn fixture(name: &str) -> Vec<u8> {
        std::fs::read(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("tests/fixtures/descriptors")
                .join(name),
        )
        .unwrap()
    }

    /// The YUBI wrist camera and the YUBI encoder (the real ESP32-C6's descriptors) are
    /// parts; the same encoder under another Espressif product id is not.
    #[test]
    fn only_yubi_parts_attach() {
        assert_eq!(kind_of(&crate::usb::fixtures::yubi_camera()), "camera");
        let encoder = fixture("esp32c6_303a_1001.bin");
        assert_eq!(kind_of(&encoder), "encoder");
        let mut other = encoder.clone();
        // idProduct, little-endian at bytes 10..12 of the device descriptor.
        other[10] ^= 0x01;
        assert_eq!(kind_of(&other), "other");
    }
}
