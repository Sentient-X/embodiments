//! One planned track's capture from one RealSense device.

use std::collections::BTreeSet;

use sx_embodiment_drivers::{CaptureError, Exposure, FrameSource, I420Image, Track};
use thiserror::Error;

use crate::clock::{self, HostSample};
use crate::image;
use crate::unit::{REALSENSE_MODELS, RealSenseUnit, Stream, UnitError};

/// The D4xx projector during capture.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Emitter {
    On,
    Off,
    /// The device alternates lit and dark frames at twice the planned rate; an infrared track
    /// is published from the dark frames.
    Interleaved,
}

/// A planned track this crate cannot capture as one RealSense exposure.
#[derive(Debug, Error, PartialEq)]
pub enum RequestError {
    #[error("track {track}: camera {camera} is a {model}, not a RealSense")]
    NotARealSense {
        track: String,
        camera: String,
        model: String,
    },
    #[error("track {track}: camera {camera} has no unit placed in its slot")]
    Unplaced { track: String, camera: String },
    #[error(transparent)]
    Unit(#[from] UnitError),
    #[error("track {track}: camera {camera} is a {modality} view of the {stream} imager")]
    Modality {
        track: String,
        camera: String,
        modality: String,
        stream: Stream,
    },
    #[error(
        "track {track}: a RealSense track is one stream, or the infrared pair as infrared 1 left \
         and infrared 2 right, all of one device"
    )]
    NotOneExposure { track: String },
    #[error("track {track}: {fps} fps is not a rate a RealSense runs at")]
    Rate { track: String, fps: f64 },
    #[error("track {track}: an interleaved projector publishes infrared, and this track has none")]
    InterleavedWithoutInfrared { track: String },
}

/// One view of a request: the planned camera and the imager stream that serves it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ViewStream {
    pub camera: String,
    pub stream: Stream,
    pub width: u32,
    pub height: u32,
}

/// What one track asks of one device.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CaptureRequest {
    pub track: String,
    /// The device's serial.
    pub serial: String,
    /// The published rate.
    pub fps: u32,
    /// The rate the device runs at: twice `fps` when an interleaved projector halves it.
    pub device_fps: u32,
    /// In the track's order.
    pub views: Vec<ViewStream>,
    pub emitter: Emitter,
}

impl CaptureRequest {
    /// The request that captures `track`, whose views are RealSense imagers placed by unit.
    ///
    /// # Errors
    ///
    /// Returns [`RequestError`] for a view that is not a RealSense, has no placed unit, or
    /// whose modality is not its stream's; for a track that is not one stream or the infrared
    /// pair (left 1, right 2) of one device; for a non-integral rate; and for an interleaved
    /// projector on a track with no infrared view.
    pub fn for_track(track: &Track, emitter: Emitter) -> Result<Self, RequestError> {
        let name = || track.track.clone();
        let mut views = Vec::with_capacity(track.views.len());
        let mut serials = BTreeSet::new();
        for view in &track.views {
            if !REALSENSE_MODELS.contains(&view.model.as_str()) {
                return Err(RequestError::NotARealSense {
                    track: name(),
                    camera: view.camera.clone(),
                    model: view.model.clone(),
                });
            }
            let unit = view.unit.as_deref().ok_or_else(|| RequestError::Unplaced {
                track: name(),
                camera: view.camera.clone(),
            })?;
            let RealSenseUnit { serial, stream } = RealSenseUnit::parse(unit)?;
            if view.modality != stream.modality() {
                return Err(RequestError::Modality {
                    track: name(),
                    camera: view.camera.clone(),
                    modality: view.modality.clone(),
                    stream,
                });
            }
            serials.insert(serial);
            views.push(ViewStream {
                camera: view.camera.clone(),
                stream,
                width: view.width,
                height: view.height,
            });
        }
        let streams: Vec<Stream> = views.iter().map(|view| view.stream).collect();
        let one_exposure =
            matches!(streams.as_slice(), [_]) || streams == [Stream::Infrared1, Stream::Infrared2];
        let Some(serial) = serials
            .pop_first()
            .filter(|_| serials.is_empty() && one_exposure)
        else {
            return Err(RequestError::NotOneExposure { track: name() });
        };
        let rate = || RequestError::Rate {
            track: name(),
            fps: track.fps,
        };
        let fps = (1..=1_000_u32)
            .find(|candidate| (f64::from(*candidate) - track.fps).abs() < f64::EPSILON)
            .ok_or_else(rate)?;
        let infrared = streams.iter().any(|stream| stream.is_infrared());
        if emitter == Emitter::Interleaved && !infrared {
            return Err(RequestError::InterleavedWithoutInfrared { track: name() });
        }
        Ok(Self {
            track: name(),
            serial,
            fps,
            device_fps: if emitter == Emitter::Interleaved {
                fps * 2
            } else {
                fps
            },
            views,
            emitter,
        })
    }
}

/// Which clock a frame's timestamp is on (`rs2_timestamp_domain`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimestampDomain {
    /// The device clock regressed onto the host's wall clock.
    GlobalTime,
    /// The device clock alone.
    HardwareClock,
    /// Host arrival.
    SystemTime,
}

/// One frame's pixels as the device delivered them, `stride` bytes a row.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Pixels {
    Rgb8 { stride: usize, data: Vec<u8> },
    Y8 { stride: usize, data: Vec<u8> },
}

/// One frame of a frameset, copied out of the device's pool.
#[derive(Clone, Debug, PartialEq)]
pub struct RawFrame {
    pub stream: Stream,
    pub number: u64,
    /// Milliseconds in `domain`.
    pub timestamp_ms: f64,
    pub domain: TimestampDomain,
    /// Whether the projector lit this exposure, when the device's metadata says.
    pub emitter_lit: Option<bool>,
    pub width: u32,
    pub height: u32,
    pub pixels: Pixels,
}

/// A source of framesets: every requested stream's frame of one device instant.
pub trait Device: Send {
    /// Block until the next frameset.
    ///
    /// # Errors
    ///
    /// Returns [`CaptureError`] when the device fails or ends.
    fn wait(&mut self) -> Result<Vec<RawFrame>, CaptureError>;
}

type Clock = Box<dyn FnMut() -> HostSample + Send>;

/// One track's [`FrameSource`] over a device.
pub struct RealSenseSource<D: Device> {
    device: D,
    request: CaptureRequest,
    clock: Clock,
    last: Option<(u64, f64)>,
}

impl<D: Device> RealSenseSource<D> {
    /// Capture `request` from `device`, stamping on this host's clocks.
    #[must_use]
    pub fn new(device: D, request: CaptureRequest) -> Self {
        Self::with_clock(device, request, Box::new(HostSample::now))
    }

    /// Capture with `clock` as the host sample, so a replay is stamped deterministically.
    #[must_use]
    pub fn with_clock(device: D, request: CaptureRequest, clock: Clock) -> Self {
        Self {
            device,
            request,
            clock,
            last: None,
        }
    }

    /// The request this source serves.
    #[must_use]
    pub fn request(&self) -> &CaptureRequest {
        &self.request
    }

    /// The exposure in `frames`, or `None` for a frameset this track does not publish (a lit
    /// frame of an interleaved projector, or one already published).
    fn exposure(
        &mut self,
        frames: &[RawFrame],
        host: HostSample,
    ) -> Result<Option<Exposure>, CaptureError> {
        let taken = self
            .request
            .views
            .iter()
            .map(|view| take(frames, view))
            .collect::<Result<Vec<_>, _>>()?;
        let first = &taken[0];
        if taken.iter().any(|frame| {
            frame.number != first.number || (frame.timestamp_ms - first.timestamp_ms).abs() > 1e-6
        }) {
            return Err(device(
                "the pair's frames are not one exposure: their numbers or times differ".to_owned(),
            ));
        }
        if self.request.emitter == Emitter::Interleaved {
            match first.emitter_lit {
                Some(true) => return Ok(None),
                Some(false) => {}
                None => {
                    return Err(device(
                        "an interleaved projector needs each frame's emitter metadata".to_owned(),
                    ));
                }
            }
        }
        if let Some((number, timestamp_ms)) = self.last {
            if first.number == number {
                return Ok(None);
            }
            if first.timestamp_ms <= timestamp_ms {
                return Err(device(format!(
                    "frame {} at {} ms is not after frame {number} at {timestamp_ms} ms",
                    first.number, first.timestamp_ms
                )));
            }
        }
        let captured = clock::captured_at(first.timestamp_ms, host)
            .map_err(|error| device(error.to_string()))?;
        let views = taken
            .iter()
            .map(convert)
            .collect::<Result<Vec<I420Image>, _>>()?;
        self.last = Some((first.number, first.timestamp_ms));
        Ok(Some(Exposure { captured, views }))
    }
}

fn take(frames: &[RawFrame], view: &ViewStream) -> Result<RawFrame, CaptureError> {
    let frame = frames
        .iter()
        .find(|frame| frame.stream == view.stream)
        .ok_or_else(|| device(format!("the frameset has no {} frame", view.stream)))?;
    if (frame.width, frame.height) != (view.width, view.height) {
        return Err(device(format!(
            "{} delivered {}x{}, the plan is {}x{}",
            view.stream, frame.width, frame.height, view.width, view.height
        )));
    }
    if frame.domain != TimestampDomain::GlobalTime {
        return Err(device(format!(
            "{} is stamped in {:?}, not global time: enable the device's global time and \
             its frame metadata",
            view.stream, frame.domain
        )));
    }
    Ok(frame.clone())
}

fn device(message: String) -> CaptureError {
    CaptureError::Device(message)
}

fn convert(frame: &RawFrame) -> Result<I420Image, CaptureError> {
    let converted = match (&frame.pixels, frame.stream.is_infrared()) {
        (Pixels::Rgb8 { stride, data }, false) => {
            image::rgb8_to_i420(frame.width, frame.height, *stride, data)
        }
        (Pixels::Y8 { stride, data }, true) => {
            image::y8_to_i420(frame.width, frame.height, *stride, data)
        }
        _ => {
            return Err(device(format!(
                "{} delivered pixels of another format",
                frame.stream
            )));
        }
    };
    converted.map_err(|error| device(error.to_string()))
}

impl<D: Device> FrameSource for RealSenseSource<D> {
    fn next(&mut self) -> Result<Exposure, CaptureError> {
        loop {
            let frames = self.device.wait()?;
            let host = (self.clock)();
            if let Some(exposure) = self.exposure(&frames, host)? {
                return Ok(exposure);
            }
        }
    }
}
