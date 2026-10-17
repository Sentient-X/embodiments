//! librealsense2 itself, through `realsense-rust` (feature `sdk`).
//!
//! [`open`] finds the device the request names by serial, sets its projector and global time,
//! starts exactly the requested streams, reads each one's factory intrinsics from the active
//! profile, and returns a [`RealSenseSource`] over it. [`devices`] lists what is attached.

use std::collections::HashSet;
use std::ffi::CStr;
use std::time::Duration;

use realsense_rust::base::Rs2Intrinsics;
use realsense_rust::config::Config;
use realsense_rust::context::Context;
use realsense_rust::device::Device as Rs2Device;
use realsense_rust::frame::{
    ColorFrame, CompositeFrame, FrameEx, ImageFrame, InfraredFrame, PixelKind,
};
use realsense_rust::kind::{
    Rs2CameraInfo, Rs2DistortionModel, Rs2Format, Rs2FrameMetadata, Rs2Option, Rs2ProductLine,
    Rs2StreamKind, Rs2TimestampDomain,
};
use realsense_rust::pipeline::{ActivePipeline, InactivePipeline};
use sx_embodiment_drivers::CaptureError;
use thiserror::Error;

use crate::factory::{Distortion, FactoryIntrinsics};
use crate::source::{
    CaptureRequest, Device, Emitter, Pixels, RawFrame, RealSenseSource, TimestampDomain,
};
use crate::unit::{RealSenseUnit, Stream};

/// How long a frameset may take before the device counts as failed.
pub const FRAME_TIMEOUT: Duration = Duration::from_secs(5);

/// The SDK refused to open what a request names.
#[derive(Debug, Error)]
pub enum SdkError {
    #[error("librealsense2: {0}")]
    Sdk(String),
    #[error("no attached RealSense has serial {0}")]
    NotAttached(String),
    #[error("RealSense {serial} cannot {what}")]
    Unsupported { serial: String, what: String },
}

/// One attached device.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DeviceInfo {
    pub serial: String,
    pub product: String,
    pub firmware: String,
}

fn info(device: &Rs2Device, field: Rs2CameraInfo) -> String {
    device
        .info(field)
        .map(CStr::to_string_lossy)
        .map(std::borrow::Cow::into_owned)
        .unwrap_or_default()
}

/// The D400 devices attached now.
///
/// # Errors
///
/// Returns [`SdkError::Sdk`] when librealsense2 cannot start a context.
pub fn devices() -> Result<Vec<DeviceInfo>, SdkError> {
    let context = Context::new().map_err(|error| SdkError::Sdk(error.to_string()))?;
    Ok(context
        .query_devices(HashSet::from([Rs2ProductLine::D400]))
        .iter()
        .map(|device| DeviceInfo {
            serial: info(device, Rs2CameraInfo::SerialNumber),
            product: info(device, Rs2CameraInfo::Name),
            firmware: info(device, Rs2CameraInfo::FirmwareVersion),
        })
        .collect())
}

/// A started device delivering the request's streams.
pub struct SdkDevice {
    pipeline: ActivePipeline,
    streams: Vec<Stream>,
}

/// Open the device `request` names and start its streams.
///
/// Returns the source and, for each requested view in order, its factory intrinsics.
///
/// # Errors
///
/// Returns [`SdkError`] when the device is not attached, refuses the projector or global time,
/// or cannot run the requested profile.
pub fn open(
    request: CaptureRequest,
) -> Result<(RealSenseSource<SdkDevice>, Vec<FactoryIntrinsics>), SdkError> {
    let sdk = |error: &dyn std::fmt::Display| SdkError::Sdk(error.to_string());
    let unsupported = |what: &str| SdkError::Unsupported {
        serial: request.serial.clone(),
        what: what.to_owned(),
    };
    let context = Context::new().map_err(|error| sdk(&error))?;
    let devices = context.query_devices(HashSet::from([Rs2ProductLine::D400]));
    let device = devices
        .iter()
        .find(|device| info(device, Rs2CameraInfo::SerialNumber) == request.serial)
        .ok_or_else(|| SdkError::NotAttached(request.serial.clone()))?;
    let product = info(device, Rs2CameraInfo::Name);
    let firmware = info(device, Rs2CameraInfo::FirmwareVersion);
    for mut sensor in device.sensors() {
        if sensor.supports_option(Rs2Option::GlobalTimeEnabled) {
            sensor
                .set_option(Rs2Option::GlobalTimeEnabled, 1.0)
                .map_err(|_| unsupported("stamp in global time"))?;
        }
        if sensor.supports_option(Rs2Option::EmitterEnabled) {
            let (enabled, on_off) = match request.emitter {
                Emitter::On => (1.0, 0.0),
                Emitter::Off => (0.0, 0.0),
                Emitter::Interleaved => (1.0, 1.0),
            };
            sensor
                .set_option(Rs2Option::EmitterEnabled, enabled)
                .map_err(|_| unsupported("set its projector"))?;
            if sensor.supports_option(Rs2Option::EmitterOnOff) {
                sensor
                    .set_option(Rs2Option::EmitterOnOff, on_off)
                    .map_err(|_| unsupported("interleave its projector"))?;
            } else if request.emitter == Emitter::Interleaved {
                return Err(unsupported("interleave its projector"));
            }
        }
    }
    let mut config = Config::new();
    config
        .enable_device_from_serial(
            &std::ffi::CString::new(request.serial.clone()).map_err(|error| sdk(&error))?,
        )
        .map_err(|error| sdk(&error))?;
    let rate = usize::try_from(request.device_fps).map_err(|error| sdk(&error))?;
    for view in &request.views {
        let (kind, format) = match view.stream {
            Stream::Color => (Rs2StreamKind::Color, Rs2Format::Rgb8),
            Stream::Infrared1 | Stream::Infrared2 => (Rs2StreamKind::Infrared, Rs2Format::Y8),
        };
        config
            .enable_stream(
                kind,
                view.stream.index(),
                usize::try_from(view.width).map_err(|error| sdk(&error))?,
                usize::try_from(view.height).map_err(|error| sdk(&error))?,
                format,
                rate,
            )
            .map_err(|error| sdk(&error))?;
    }
    let pipeline = InactivePipeline::try_from(&context).map_err(|error| sdk(&error))?;
    let pipeline = pipeline.start(Some(config)).map_err(|error| sdk(&*error))?;
    let mut intrinsics = Vec::with_capacity(request.views.len());
    for view in &request.views {
        let profile = pipeline
            .profile()
            .streams()
            .iter()
            .find(|profile| stream_of(profile.kind(), profile.index()) == Some(view.stream))
            .ok_or_else(|| unsupported(&format!("start its {} stream", view.stream)))?;
        let factory = profile.intrinsics().map_err(|error| sdk(&error))?;
        intrinsics.push(factory_intrinsics(
            RealSenseUnit {
                serial: request.serial.clone(),
                stream: view.stream,
            },
            &product,
            &firmware,
            &factory,
        )?);
    }
    let streams = request.views.iter().map(|view| view.stream).collect();
    Ok((
        RealSenseSource::new(SdkDevice { pipeline, streams }, request),
        intrinsics,
    ))
}

fn stream_of(kind: Rs2StreamKind, index: usize) -> Option<Stream> {
    match (kind, index) {
        (Rs2StreamKind::Color, _) => Some(Stream::Color),
        (Rs2StreamKind::Infrared, 1) => Some(Stream::Infrared1),
        (Rs2StreamKind::Infrared, 2) => Some(Stream::Infrared2),
        _ => None,
    }
}

fn factory_intrinsics(
    unit: RealSenseUnit,
    product: &str,
    firmware: &str,
    raw: &Rs2Intrinsics,
) -> Result<FactoryIntrinsics, SdkError> {
    let distortion = raw.distortion();
    let raster =
        |value: usize| u32::try_from(value).map_err(|error| SdkError::Sdk(error.to_string()));
    Ok(FactoryIntrinsics {
        unit,
        product: product.to_owned(),
        firmware: firmware.to_owned(),
        width: raster(raw.width())?,
        height: raster(raw.height())?,
        fx: raw.fx(),
        fy: raw.fy(),
        ppx: raw.ppx(),
        ppy: raw.ppy(),
        distortion: match distortion.model {
            Rs2DistortionModel::None => Distortion::None,
            Rs2DistortionModel::BrownConrady => Distortion::BrownConrady,
            Rs2DistortionModel::BrownConradyModified => Distortion::ModifiedBrownConrady,
            Rs2DistortionModel::BrownConradyInverse => Distortion::InverseBrownConrady,
            Rs2DistortionModel::FThetaFisheye => Distortion::Ftheta,
            Rs2DistortionModel::KannalaBrandt => Distortion::KannalaBrandt4,
        },
        coefficients: distortion.coeffs,
    })
}

fn domain(value: Rs2TimestampDomain) -> TimestampDomain {
    match value {
        Rs2TimestampDomain::GlobalTime => TimestampDomain::GlobalTime,
        Rs2TimestampDomain::HardwareClock => TimestampDomain::HardwareClock,
        Rs2TimestampDomain::SystemTime => TimestampDomain::SystemTime,
    }
}

fn copied<K>(frame: &ImageFrame<K>, stream: Stream) -> Result<RawFrame, CaptureError>
where
    ImageFrame<K>: FrameEx,
{
    let (width, height) = (frame.width(), frame.height());
    let raster = |value: usize| {
        u32::try_from(value).map_err(|error| CaptureError::Device(error.to_string()))
    };
    let mut data = Vec::with_capacity(width * height * if stream.is_infrared() { 1 } else { 3 });
    for row in 0..height {
        for col in 0..width {
            match frame.get_unchecked(col, row) {
                PixelKind::Rgb8 { r, g, b } | PixelKind::Bgr8 { r, g, b } => {
                    data.extend([*r, *g, *b]);
                }
                PixelKind::Y8 { y } => data.push(*y),
                _ => {
                    return Err(CaptureError::Device(format!(
                        "{stream} delivered a pixel format it was not asked for"
                    )));
                }
            }
        }
    }
    let pixels = if stream.is_infrared() {
        Pixels::Y8 {
            stride: width,
            data,
        }
    } else {
        Pixels::Rgb8 {
            stride: width * 3,
            data,
        }
    };
    let lit = frame
        .metadata(Rs2FrameMetadata::FrameLaserPowerMode)
        .map(|mode| mode != 0);
    Ok(RawFrame {
        stream,
        number: frame.frame_number(),
        timestamp_ms: frame.timestamp(),
        domain: domain(frame.timestamp_domain()),
        emitter_lit: lit,
        width: raster(width)?,
        height: raster(height)?,
        pixels,
    })
}

fn frames(composite: &CompositeFrame, wanted: &[Stream]) -> Result<Vec<RawFrame>, CaptureError> {
    let mut out = Vec::with_capacity(wanted.len());
    if wanted.contains(&Stream::Color) {
        for frame in composite.frames_of_type::<ColorFrame>() {
            out.push(copied(&frame, Stream::Color)?);
        }
    }
    for frame in composite.frames_of_type::<InfraredFrame>() {
        let profile = frame.stream_profile();
        if let Some(stream) = stream_of(profile.kind(), profile.index())
            && wanted.contains(&stream)
        {
            out.push(copied(&frame, stream)?);
        }
    }
    Ok(out)
}

impl Device for SdkDevice {
    fn wait(&mut self) -> Result<Vec<RawFrame>, CaptureError> {
        let composite = self
            .pipeline
            .wait(Some(FRAME_TIMEOUT))
            .map_err(|error| CaptureError::Device(error.to_string()))?;
        frames(&composite, &self.streams)
    }
}
