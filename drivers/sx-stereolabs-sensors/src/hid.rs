//! The sensor module on a real USB HID link, through `hidapi` (feature `hid`).
//!
//! macOS's own HID driver owns the module's interface, so a user-space USB claim (`nusb`) cannot
//! read it there; `hidapi` reads it through `IOHIDManager`, and on Linux through `hidraw`,
//! enumerated by sysfs without libudev. The device is found by its product's USB ids and the
//! serial it reports, as `zed-open-capture`'s `SensorCapture::open` finds it.

use std::sync::Arc;
use std::time::Duration;

use hidapi::{HidApi, HidDevice};
use sx_embodiment_drivers::{CaptureError, InertialSensor};
use thiserror::Error;

use crate::source::{ReportPort, RequestError, SensorRequest, StereolabsInertialSource};

/// A planned sensor that could not be opened.
#[derive(Debug, Error)]
pub enum OpenError {
    #[error(transparent)]
    Request(#[from] RequestError),
    #[error("HID: {0}")]
    Hid(#[from] hidapi::HidError),
    #[error("no {label} sensor module with serial {serial} is attached (attached: {attached:?})")]
    Absent {
        label: &'static str,
        serial: String,
        attached: Vec<String>,
    },
    #[error(transparent)]
    Capture(#[from] CaptureError),
}

/// One attached sensor module.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Attached {
    pub model: &'static str,
    pub serial: String,
    /// Its USB release number, the firmware version.
    pub firmware: u16,
}

/// Every attached sensor module of a listed product.
///
/// # Errors
///
/// [`OpenError::Hid`] when the HID subsystem cannot be read.
pub fn attached() -> Result<Vec<Attached>, OpenError> {
    let api = HidApi::new()?;
    let mut found: Vec<Attached> = api
        .device_list()
        .filter_map(|info| {
            let product = crate::PRODUCTS.iter().find(|product| {
                product.usb_vendor == info.vendor_id() && product.usb_product == info.product_id()
            })?;
            Some(Attached {
                model: product.model,
                serial: info.serial_number()?.to_owned(),
                firmware: info.release_number(),
            })
        })
        .collect();
    found.dedup();
    Ok(found)
}

/// The HID link to one module.
pub struct HidPort(HidDevice);

impl ReportPort for HidPort {
    fn read(&mut self, buffer: &mut [u8], timeout: Duration) -> Result<usize, String> {
        let millis = i32::try_from(timeout.as_millis()).unwrap_or(i32::MAX);
        self.0
            .read_timeout(buffer, millis)
            .map_err(|error| error.to_string())
    }

    fn send_feature(&mut self, report: &[u8]) -> Result<(), String> {
        self.0
            .send_feature_report(report)
            .map_err(|error| error.to_string())
    }
}

/// Open the planned sensor: the module of its product with the serial its unit names.
///
/// # Errors
///
/// [`OpenError`] for a sensor this crate cannot read, a module not attached, or one that
/// refuses to stream.
pub fn open(sensor: &InertialSensor) -> Result<StereolabsInertialSource<HidPort>, OpenError> {
    let request = SensorRequest::for_sensor(sensor)?;
    open_request(&request)
}

/// Open the module `request` names.
///
/// # Errors
///
/// As [`open`].
pub fn open_request(
    request: &SensorRequest,
) -> Result<StereolabsInertialSource<HidPort>, OpenError> {
    let api = HidApi::new()?;
    let product = request.product;
    let matching = |info: &&hidapi::DeviceInfo| {
        info.vendor_id() == product.usb_vendor && info.product_id() == product.usb_product
    };
    let Some(info) = api
        .device_list()
        .filter(matching)
        .find(|info| info.serial_number() == Some(request.serial.as_str()))
    else {
        return Err(OpenError::Absent {
            label: product.label,
            serial: request.serial.clone(),
            attached: api
                .device_list()
                .filter(matching)
                .filter_map(|info| info.serial_number().map(str::to_owned))
                .collect(),
        });
    };
    let firmware = info.release_number();
    let device = info.open_device(&api)?;
    Ok(StereolabsInertialSource::start(
        HidPort(device),
        Arc::clone(&request.frame),
        firmware,
    )?)
}
