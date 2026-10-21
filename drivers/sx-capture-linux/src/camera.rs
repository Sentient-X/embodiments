//! V4L2 transport on the already verified open camera handle.
use crate::identity::LockedCamera;
use std::collections::BTreeMap;
use std::path::Path;
use std::time::Duration;
use std::{fs, io};
use thiserror::Error;
use v4l::buffer::{Flags, Type};
use v4l::format::{Format, FourCC};
use v4l::io::traits::CaptureStream;
use v4l::prelude::MmapStream;
use v4l::video::Capture;
use v4l::video::capture::Parameters;

#[derive(Debug, Error)]
pub enum CameraError {
    #[error("{0}")]
    Io(#[from] io::Error),
    #[error("{0}")]
    Hardware(String),
}

/// The caller selects the format and rate; returns the actual negotiated rate.
pub fn configure(
    device: &LockedCamera,
    width: u32,
    height: u32,
    fps: u32,
    context: &str,
) -> Result<f64, CameraError> {
    let actual = device.set_format(&Format::new(width, height, FourCC::new(b"MJPG")))?;
    if actual.width != width || actual.height != height || actual.fourcc != FourCC::new(b"MJPG") {
        return Err(CameraError::Hardware(format!(
            "{context}negotiated {actual}, need {width}x{height} MJPG"
        )));
    }
    let parameters = device.set_params(&Parameters::with_fps(fps))?;
    Ok(parameters.interval.denominator as f64 / parameters.interval.numerator.max(1) as f64)
}

pub struct CameraStream<'a>(MmapStream<'a>);
impl<'a> CameraStream<'a> {
    pub fn new(
        device: &'a LockedCamera,
        buffers: u32,
        timeout: Duration,
    ) -> Result<Self, CameraError> {
        let mut stream = MmapStream::with_buffers(device, Type::VideoCapture, buffers)?;
        stream.set_timeout(timeout);
        Ok(Self(stream))
    }
    /// Preview makes no timestamp claim.
    pub fn next_preview(&mut self) -> Result<Vec<u8>, CameraError> {
        Ok(self.0.next()?.0.to_vec())
    }
    /// Do not retry a timeout: v4l already queued the buffer; the owner must exit the stream.
    pub fn next_frame(&mut self) -> Result<(Vec<u8>, u32, i64), CameraError> {
        let (bytes, metadata) = self.0.next()?;
        if !metadata.flags.contains(Flags::TIMESTAMP_MONOTONIC)
            || !metadata.flags.contains(Flags::TSTAMP_SRC_SOE)
        {
            return Err(CameraError::Hardware(
                "camera driver did not provide monotonic start-of-exposure timestamps".into(),
            ));
        }
        let timestamp = metadata.timestamp.sec * 1_000_000_000 + metadata.timestamp.usec * 1_000;
        Ok((bytes.to_vec(), metadata.sequence, timestamp))
    }
}

pub fn require_hardware_timestamps() -> Result<(), CameraError> {
    let path = Path::new("/sys/module/uvcvideo/parameters/hwtimestamps");
    if fs::read_to_string(path)?.trim() != "1" {
        return Err(CameraError::Hardware(format!(
            "{} must be 1 before recording",
            path.display()
        )));
    }
    Ok(())
}

pub fn set_image_controls(
    device: &LockedCamera,
    mut choose: impl FnMut(&v4l::control::Description) -> Option<i64>,
) -> Result<(), CameraError> {
    use v4l::control::{Control, Value as ControlValue};
    for description in device.query_controls()? {
        let Some(value) = choose(&description) else {
            continue;
        };
        let value = if description.typ == v4l::control::Type::Boolean {
            ControlValue::Boolean(value != 0)
        } else {
            ControlValue::Integer(value)
        };
        let shown = format!("{value:?}");
        if let Err(error) = device.set_control(Control {
            id: description.id,
            value,
        }) {
            eprintln!(
                "sx-pod: wrist camera refused {} = {shown}: {error}",
                description.name
            );
        }
    }
    Ok(())
}

pub fn held_image_controls(device: &LockedCamera) -> Result<BTreeMap<String, i64>, CameraError> {
    use v4l::control::{Flags as ControlFlags, Type as ControlType, Value as ControlValue};
    use v4l::v4l_sys::{V4L2_CID_EXPOSURE_ABSOLUTE, V4L2_CID_EXPOSURE_AUTO, V4L2_CID_GAIN};
    let mut held = BTreeMap::new();
    for description in device.query_controls()? {
        if [
            V4L2_CID_EXPOSURE_AUTO,
            V4L2_CID_EXPOSURE_ABSOLUTE,
            V4L2_CID_GAIN,
        ]
        .contains(&description.id)
        {
            continue;
        }
        if !matches!(
            description.typ,
            ControlType::Integer
                | ControlType::Integer64
                | ControlType::Boolean
                | ControlType::Menu
                | ControlType::IntegerMenu
        ) || description
            .flags
            .intersects(ControlFlags::DISABLED | ControlFlags::WRITE_ONLY)
        {
            continue;
        }
        let value = match device.control(description.id).map(|control| control.value) {
            Ok(ControlValue::Integer(value)) => value,
            Ok(ControlValue::Boolean(value)) => i64::from(value),
            Ok(_) => continue,
            Err(error) => {
                eprintln!(
                    "sx-pod: wrist camera control {} unreadable: {error}",
                    description.name
                );
                continue;
            }
        };
        held.insert(description.name, value);
    }
    Ok(held)
}

pub fn apply_exposure(
    device: &LockedCamera,
    exposure_100us: i32,
    gain: i32,
    enter_manual: bool,
) -> Result<(i32, i32), CameraError> {
    use v4l::control::{Control, Value as ControlValue};
    use v4l::v4l_sys::{V4L2_CID_EXPOSURE_ABSOLUTE, V4L2_CID_EXPOSURE_AUTO, V4L2_CID_GAIN};
    // All control ioctls use the verified stream handle, including during capture.
    // Reopening /dev/videoN here could control a substituted camera after hotplug.
    if enter_manual {
        device.set_control(Control {
            id: V4L2_CID_EXPOSURE_AUTO,
            value: ControlValue::Integer(1),
        })?;
    }
    for (id, value) in [
        (V4L2_CID_EXPOSURE_ABSOLUTE, exposure_100us),
        (V4L2_CID_GAIN, gain),
    ] {
        device.set_control(Control {
            id,
            value: ControlValue::Integer(value.into()),
        })?;
    }
    if device.control(V4L2_CID_EXPOSURE_AUTO)?.value != ControlValue::Integer(1) {
        return Err(CameraError::Hardware(
            "camera did not hold manual exposure mode".into(),
        ));
    }
    match (
        device.control(V4L2_CID_EXPOSURE_ABSOLUTE)?.value,
        device.control(V4L2_CID_GAIN)?.value,
    ) {
        (ControlValue::Integer(exposure), ControlValue::Integer(gain)) => Ok((
            i32::try_from(exposure).map_err(|e| CameraError::Hardware(e.to_string()))?,
            i32::try_from(gain).map_err(|e| CameraError::Hardware(e.to_string()))?,
        )),
        _ => Err(CameraError::Hardware(
            "camera exposure readback is incomplete".into(),
        )),
    }
}
