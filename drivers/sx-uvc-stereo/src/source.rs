//! One planned stereo track's capture from one side-by-side UVC device.

use std::time::Instant;

use sx_embodiment_drivers::{CaptureError, CapturedAt, Exposure, FrameSource, Track};
use thiserror::Error;

use crate::product::{Eye, Product, StereoUnit, UnitError};
use crate::split::split;

/// A planned track this crate cannot capture as one side-by-side exposure.
#[derive(Debug, Error, PartialEq)]
pub enum RequestError {
    #[error("track {track}: camera {camera} is a {model}, not a side-by-side stereo camera")]
    NotAStereoCamera {
        track: String,
        camera: String,
        model: String,
    },
    #[error("track {track}: camera {camera} has no unit placed in its slot")]
    Unplaced { track: String, camera: String },
    #[error(transparent)]
    Unit(#[from] UnitError),
    #[error(
        "track {track}: a stereo track is the left then the right eye of one device, as one \
         {modality} exposure"
    )]
    NotOneExposure { track: String, modality: String },
    #[error("track {track}: a {label} has no {width}x{height} eye mode at {fps} fps")]
    Mode {
        track: String,
        label: &'static str,
        width: u32,
        height: u32,
        fps: f64,
    },
}

/// What one track asks of one device.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CaptureRequest {
    pub track: String,
    pub product: &'static Product,
    /// The device's USB serial.
    pub serial: String,
    pub eye_width: u32,
    pub eye_height: u32,
    pub fps: u32,
    /// The left and right views' camera names.
    pub cameras: [String; 2],
}

impl CaptureRequest {
    /// The request that captures `track`, whose two views are the eyes of one stereo device
    /// placed by unit.
    ///
    /// # Errors
    ///
    /// Returns [`RequestError`] for a view that is not a listed stereo product or has no placed
    /// unit; for a track that is not the left then the right colour eye of one device; and for a
    /// raster and rate the product has no mode for.
    pub fn for_track(track: &Track) -> Result<Self, RequestError> {
        let name = || track.track.clone();
        let mut placed = Vec::with_capacity(2);
        for view in &track.views {
            let product =
                Product::of(&view.model).ok_or_else(|| RequestError::NotAStereoCamera {
                    track: name(),
                    camera: view.camera.clone(),
                    model: view.model.clone(),
                })?;
            let unit = view.unit.as_deref().ok_or_else(|| RequestError::Unplaced {
                track: name(),
                camera: view.camera.clone(),
            })?;
            placed.push((view, product, StereoUnit::parse(unit)?));
        }
        let [
            (left, product, left_unit),
            (right, right_product, right_unit),
        ] = placed.as_slice()
        else {
            return Err(RequestError::NotOneExposure {
                track: name(),
                modality: "rgb".to_owned(),
            });
        };
        if product != right_product
            || (left_unit.eye, right_unit.eye) != (Eye::Left, Eye::Right)
            || left_unit.serial != right_unit.serial
            || (left.width, left.height) != (right.width, right.height)
            || left.modality != "rgb"
            || right.modality != "rgb"
        {
            return Err(RequestError::NotOneExposure {
                track: name(),
                modality: "rgb".to_owned(),
            });
        }
        let mode = || RequestError::Mode {
            track: name(),
            label: product.label,
            width: left.width,
            height: left.height,
            fps: track.fps,
        };
        let fps = (1..=1_000_u32)
            .find(|candidate| (f64::from(*candidate) - track.fps).abs() < f64::EPSILON)
            .filter(|fps| product.runs(left.width, left.height, *fps))
            .ok_or_else(mode)?;
        Ok(Self {
            track: name(),
            product,
            serial: left_unit.serial.clone(),
            eye_width: left.width,
            eye_height: left.height,
            fps,
            cameras: [left.camera.clone(), right.camera.clone()],
        })
    }

    /// The raster the device delivers: both eyes side by side.
    #[must_use]
    pub fn frame_raster(&self) -> (u32, u32) {
        (self.eye_width * 2, self.eye_height)
    }
}

/// One side-by-side frame as the device delivered it: YUYV, `stride` bytes a row.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawFrame {
    pub width: u32,
    pub height: u32,
    pub stride: usize,
    pub data: Vec<u8>,
    /// When the host had the frame.
    pub captured: CapturedAt,
}

/// A source of side-by-side frames.
pub trait Device: Send {
    /// Block until the next frame.
    ///
    /// # Errors
    ///
    /// Returns [`CaptureError`] when the device fails or ends.
    fn wait(&mut self) -> Result<RawFrame, CaptureError>;
}

/// One track's [`FrameSource`] over a device.
pub struct UvcStereoSource<D: Device> {
    device: D,
    request: CaptureRequest,
    last: Option<Instant>,
}

impl<D: Device> UvcStereoSource<D> {
    /// Capture `request` from `device`.
    #[must_use]
    pub fn new(device: D, request: CaptureRequest) -> Self {
        Self {
            device,
            request,
            last: None,
        }
    }

    /// The request this source serves.
    #[must_use]
    pub fn request(&self) -> &CaptureRequest {
        &self.request
    }
}

fn device(message: String) -> CaptureError {
    CaptureError::Device(message)
}

impl<D: Device> FrameSource for UvcStereoSource<D> {
    fn next(&mut self) -> Result<Exposure, CaptureError> {
        let frame = self.device.wait()?;
        let planned = self.request.frame_raster();
        if (frame.width, frame.height) != planned {
            return Err(device(format!(
                "the device delivered {}x{}, the plan is {}x{} side by side",
                frame.width, frame.height, planned.0, planned.1
            )));
        }
        if self
            .last
            .is_some_and(|last| frame.captured.monotonic <= last)
        {
            return Err(device("a frame is not after the one before it".to_owned()));
        }
        let [left, right] = split(
            self.request.eye_width,
            self.request.eye_height,
            frame.stride,
            &frame.data,
        )
        .map_err(|error| device(error.to_string()))?;
        self.last = Some(frame.captured.monotonic);
        Ok(Exposure {
            captured: frame.captured,
            views: vec![left, right],
        })
    }
}
