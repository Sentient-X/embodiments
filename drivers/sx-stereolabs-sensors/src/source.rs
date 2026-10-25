//! Which device a planned inertial sensor is, and its readings as an [`InertialSource`].

use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use sx_embodiment_drivers::{
    CaptureError, CapturedAt, InertialQuantity, InertialSample, InertialSensor, InertialSource,
};
use thiserror::Error;

use crate::report::{PING_EVERY, REPORT_LEN, RawReport, ping, stream};

/// A Stereolabs sensor module this crate reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Product {
    /// The registry's product id (`sx_embodiments.cameras`).
    pub model: &'static str,
    pub label: &'static str,
    pub usb_vendor: u16,
    /// The sensor module's own USB product id, beside the camera's video one.
    pub usb_product: u16,
    pub measures: &'static [InertialQuantity],
}

/// Every product this crate reads (`SL_USB_PROD_MCU_ZED2i_REVA` in `include/defines.hpp`).
pub const PRODUCTS: &[Product] = &[Product {
    model: "zed_2i",
    label: "Stereolabs ZED 2i",
    usb_vendor: 0x2b03,
    usb_product: 0xf881,
    measures: &[
        InertialQuantity::Acceleration,
        InertialQuantity::AngularVelocity,
        InertialQuantity::MagneticField,
        InertialQuantity::Pressure,
        InertialQuantity::Temperature,
    ],
}];

/// A planned sensor this crate cannot open.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum RequestError {
    #[error("{0} is not a Stereolabs sensor module this crate reads")]
    Product(String),
    #[error("{sensor}: the plan places no unit, so no device serial names it")]
    NoUnit { sensor: String },
    #[error("{sensor}: {unit:?} is not a Stereolabs serial")]
    Serial { sensor: String, unit: String },
    #[error("{sensor}: a {model} does not measure {quantity:?}")]
    Measures {
        sensor: String,
        model: String,
        quantity: InertialQuantity,
    },
}

/// One planned sensor resolved to a device: which product, which serial.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SensorRequest {
    pub product: Product,
    /// The serial the sensor module reports over USB.
    pub serial: String,
    /// The planned sensor's name, the frame its samples are on.
    pub frame: Arc<str>,
}

impl SensorRequest {
    /// The device a planned inertial sensor is.
    ///
    /// # Errors
    ///
    /// [`RequestError`] for a product this crate does not read, a sensor the instance placed no
    /// unit in, a unit that is not a serial, or a quantity the product does not measure.
    pub fn for_sensor(sensor: &InertialSensor) -> Result<Self, RequestError> {
        let product = *PRODUCTS
            .iter()
            .find(|product| product.model == sensor.model)
            .ok_or_else(|| RequestError::Product(sensor.model.clone()))?;
        if let Some(quantity) = sensor
            .measures
            .iter()
            .find(|quantity| !product.measures.contains(quantity))
        {
            return Err(RequestError::Measures {
                sensor: sensor.sensor.clone(),
                model: sensor.model.clone(),
                quantity: *quantity,
            });
        }
        let unit = sensor.unit.clone().ok_or_else(|| RequestError::NoUnit {
            sensor: sensor.sensor.clone(),
        })?;
        if unit.is_empty() || !unit.bytes().all(|byte| byte.is_ascii_digit()) {
            return Err(RequestError::Serial {
                sensor: sensor.sensor.clone(),
                unit,
            });
        }
        Ok(Self {
            product,
            serial: unit,
            frame: Arc::from(sensor.sensor.as_str()),
        })
    }
}

/// The HID link to one sensor module: input reports in, feature reports out.
pub trait ReportPort: Send {
    /// Read one input report into `buffer`, its report id first; `Ok(0)` when none arrived
    /// within `timeout`.
    ///
    /// # Errors
    ///
    /// The link's own failure, as text.
    fn read(&mut self, buffer: &mut [u8], timeout: Duration) -> Result<usize, String>;

    /// Send one feature report, its report id first.
    ///
    /// # Errors
    ///
    /// The link's own failure, as text.
    fn send_feature(&mut self, report: &[u8]) -> Result<(), String>;
}

/// How long the module may stay silent before the source calls the device failed.
pub const SILENCE: Duration = Duration::from_secs(2);

/// A sensor module's readings as an [`InertialSource`]: one sample per new accelerometer and
/// gyroscope reading, the module pinged as the source does, its stream stopped on drop.
pub struct StereolabsInertialSource<P: ReportPort> {
    port: P,
    frame: Arc<str>,
    firmware: u16,
    since_ping: u32,
}

impl<P: ReportPort> StereolabsInertialSource<P> {
    /// Start the module streaming over `port`; `firmware` is its USB release number, which
    /// decides the pressure scale.
    ///
    /// # Errors
    ///
    /// [`CaptureError::Device`] when the module refuses the request to stream.
    pub fn start(mut port: P, frame: Arc<str>, firmware: u16) -> Result<Self, CaptureError> {
        port.send_feature(&stream(true))
            .map_err(|error| CaptureError::Device(format!("start streaming: {error}")))?;
        Ok(Self {
            port,
            frame,
            firmware,
            since_ping: 0,
        })
    }

    /// The module's USB release number.
    #[must_use]
    pub fn firmware(&self) -> u16 {
        self.firmware
    }
}

impl<P: ReportPort> InertialSource for StereolabsInertialSource<P> {
    fn next(&mut self) -> Result<InertialSample, CaptureError> {
        let mut buffer = [0_u8; 64];
        let silent_since = Instant::now();
        loop {
            self.since_ping += 1;
            if self.since_ping >= PING_EVERY {
                self.since_ping = 0;
                self.port
                    .send_feature(&ping())
                    .map_err(|error| CaptureError::Device(format!("ping: {error}")))?;
            }
            let read = self
                .port
                .read(&mut buffer, SILENCE)
                .map_err(CaptureError::Device)?;
            let captured = CapturedAt {
                monotonic: Instant::now(),
                wall: SystemTime::now(),
            };
            let report = if read >= REPORT_LEN {
                RawReport::parse(&buffer[..read]).ok()
            } else {
                None
            };
            let Some(report) = report.filter(RawReport::imu_is_new) else {
                if silent_since.elapsed() >= SILENCE {
                    return Err(CaptureError::Device(format!(
                        "no new inertial reading in {} s",
                        SILENCE.as_secs()
                    )));
                }
                continue;
            };
            return Ok(InertialSample {
                frame: Arc::clone(&self.frame),
                captured,
                device_time: report.device_time(),
                acceleration_mps2: report.acceleration_mps2(),
                angular_velocity_radps: report.angular_velocity_radps(),
                magnetic_field_t: report.magnetic_field_t(),
                pressure_pa: report.pressure_pa(self.firmware),
                temperature_c: report.imu_temperature_c(),
            });
        }
    }
}

impl<P: ReportPort> Drop for StereolabsInertialSource<P> {
    fn drop(&mut self) {
        // Best effort, as the source's `close`: a module left streaming stops on its own once
        // the pings stop.
        let _ = self.port.send_feature(&stream(false));
    }
}
