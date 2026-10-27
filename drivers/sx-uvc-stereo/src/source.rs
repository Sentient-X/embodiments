//! One planned stereo track's capture from a side-by-side UVC device it may share with the
//! other eye's track.

use std::sync::Arc;
use std::time::Instant;

use sx_embodiment_drivers::{CaptureError, CapturedAt, Exposure, FrameSource, Track};
use thiserror::Error;

use crate::product::{Eye, Product, StereoUnit, UnitError};
use crate::split::eye;

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
        "track {track}: a stereo track is one rgb eye, or the left then the right eye, of one \
         device at one raster"
    )]
    NotOneExposure { track: String },
    #[error("track {track}: a {label} has no {width}x{height} eye mode at {fps} fps")]
    Mode {
        track: String,
        label: &'static str,
        width: u32,
        height: u32,
        fps: f64,
    },
    #[error(
        "tracks {first} and {second} share device {serial} at different modes; one device runs \
         one mode"
    )]
    SharedMode {
        serial: String,
        first: String,
        second: String,
    },
}

/// What one track asks of one device: one eye, or the left then the right.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CaptureRequest {
    pub track: String,
    pub product: &'static Product,
    /// The device's USB serial.
    pub serial: String,
    pub eye_width: u32,
    pub eye_height: u32,
    pub fps: u32,
    /// The views' camera names, in the track's order.
    pub cameras: Vec<String>,
    /// The eye each view shows, in the track's order.
    pub eyes: Vec<Eye>,
}

impl CaptureRequest {
    /// The request that captures `track`: one view that is an eye of a stereo device placed by
    /// unit, or two views that are its left then its right eye.
    ///
    /// # Errors
    ///
    /// Returns [`RequestError`] for a view that is not a listed stereo product or has no placed
    /// unit; for a track that is not one colour eye, or the left then the right colour eye, of
    /// one device; and for a raster and rate the product has no mode for.
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
        let refuse = || RequestError::NotOneExposure { track: name() };
        let (first, product, first_unit) = placed.first().ok_or_else(refuse)?;
        let eyes: Vec<Eye> = placed.iter().map(|(_, _, unit)| unit.eye).collect();
        let one_device = placed.iter().all(|(view, other, unit)| {
            other == product
                && unit.serial == first_unit.serial
                && (view.width, view.height) == (first.width, first.height)
                && view.modality == "rgb"
        });
        if !one_device || !matches!(eyes.as_slice(), [_] | [Eye::Left, Eye::Right]) {
            return Err(refuse());
        }
        let mode = || RequestError::Mode {
            track: name(),
            label: product.label,
            width: first.width,
            height: first.height,
            fps: track.fps,
        };
        let fps = (1..=1_000_u32)
            .find(|candidate| (f64::from(*candidate) - track.fps).abs() < f64::EPSILON)
            .filter(|fps| product.runs(first.width, first.height, *fps))
            .ok_or_else(mode)?;
        Ok(Self {
            track: name(),
            product,
            serial: first_unit.serial.clone(),
            eye_width: first.width,
            eye_height: first.height,
            fps,
            cameras: placed
                .iter()
                .map(|(view, _, _)| view.camera.clone())
                .collect(),
            eyes,
        })
    }

    /// The raster the device delivers: both eyes side by side.
    #[must_use]
    pub fn frame_raster(&self) -> (u32, u32) {
        (self.eye_width * 2, self.eye_height)
    }

    /// The requests grouped by the device they open, in first-planned order: tracks that are
    /// eyes of one unit share one capture of it, which runs one mode.
    ///
    /// # Errors
    ///
    /// Returns [`RequestError::SharedMode`] for two tracks that ask one device for different
    /// modes.
    pub fn by_device(requests: Vec<Self>) -> Result<Vec<Vec<Self>>, RequestError> {
        let mut devices: Vec<Vec<Self>> = Vec::new();
        for request in requests {
            let Some(device) = devices
                .iter_mut()
                .find(|device| device[0].serial == request.serial)
            else {
                devices.push(vec![request]);
                continue;
            };
            let first = &device[0];
            let mode = |r: &Self| (r.product, r.eye_width, r.eye_height, r.fps);
            if mode(first) != mode(&request) {
                return Err(RequestError::SharedMode {
                    serial: request.serial.clone(),
                    first: first.track.clone(),
                    second: request.track,
                });
            }
            device.push(request);
        }
        Ok(devices)
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
    /// Block until the next frame, shared with every track of the device.
    ///
    /// # Errors
    ///
    /// Returns [`CaptureError`] when the device fails or ends.
    fn wait(&mut self) -> Result<Arc<RawFrame>, CaptureError>;
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
        let request = &self.request;
        let views = request
            .eyes
            .iter()
            .map(|&side| {
                eye(
                    side,
                    request.eye_width,
                    request.eye_height,
                    frame.stride,
                    &frame.data,
                )
            })
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| device(error.to_string()))?;
        self.last = Some(frame.captured.monotonic);
        Ok(Exposure {
            captured: frame.captured,
            views,
        })
    }
}
