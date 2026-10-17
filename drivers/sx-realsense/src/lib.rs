//! Intel RealSense D400 cameras (D405, D435, D435i) as media-plan [`FrameSource`]s.
//!
//! A station publishes what its robot's [`MediaPlan`] names. A planned track whose views are
//! RealSense imagers becomes one [`CaptureRequest`] ([`CaptureRequest::for_track`]): which
//! device, which of its streams, at which raster and rate, with which projector setting. The
//! device and stream come from the camera unit the robot instance placed in each view's slot,
//! spelled `<device serial>/<stream>` ([`RealSenseUnit`]): one RealSense holds one camera unit
//! per imager, each with its own intrinsics, and a robot instance places each unit once.
//!
//! [`RealSenseSource`] turns one device's framesets into [`Exposure`]s for that track, over
//! any [`Device`]: [`sdk::open`] (feature `sdk`) is librealsense2 through `realsense-rust`, and
//! [`recorded::RecordedDevice`] replays framesets, which is how everything here is tested with
//! no hardware.
//!
//! # Capture time
//!
//! A frame is stamped where it was exposed, on this host's clocks ([`clock`]). librealsense's
//! global-time domain is the device's hardware clock regressed onto the host's system clock by
//! the SDK; a frame's global timestamp is therefore a host wall-clock time. The source takes
//! one paired host sample (monotonic and wall) as soon as the frameset is in hand, and subtracts
//! the frame's age, `wall_at_sample - global_timestamp`, from both. Its error is the SDK's
//! regression error plus the half-width of the paired sample (microseconds); a global
//! timestamp up to [`clock::MAX_GLOBAL_TIME_LEAD`] in the future is the regression's own
//! uncertainty and is clamped to the sample, and anything later is refused. A frame stamped
//! in any other domain (system time is host arrival, not exposure; the hardware clock alone has
//! no host relation) is refused, as `sx_drivers.realsense` refuses it.
//!
//! # Projector
//!
//! The D4xx's IR projector paints a dot pattern the infrared imagers see. [`Emitter`] chooses
//! it per device: on, off, or interleaved (the device alternates lit and dark frames at twice
//! the planned rate, and an infrared track is published from the dark frames only, so a viewer
//! sees clean infrared while depth keeps its pattern).
//!
//! # Pixels
//!
//! Colour is requested as RGB8 and infrared as Y8, converted to I420 in BT.601 limited range,
//! the range a WebRTC encoder assumes. (`realsense-rust` 1.3.0 picks a YUYV pixel's luma by
//! row parity rather than column parity, so YUYV is never requested through it.)
//!
//! # Factory intrinsics
//!
//! [`FactoryIntrinsics`] is what a unit's own calibration record says for one active stream
//! profile, rendered as `sx.factory-intrinsics/v1`; `sx_embodiments.factory_intrinsics` reads
//! it into a camera-unit calibration measured by the vendor's factory, labelled as such.

pub mod clock;
pub mod factory;
pub mod image;
pub mod recorded;
#[cfg(feature = "sdk")]
pub mod sdk;
pub mod source;
pub mod unit;

pub use factory::{Distortion, FACTORY_SCHEMA, FactoryIntrinsics};
pub use source::{
    CaptureRequest, Device, Emitter, Pixels, RawFrame, RealSenseSource, RequestError,
    TimestampDomain, ViewStream,
};
pub use sx_embodiment_drivers::{Exposure, FrameSource, MediaPlan};
pub use unit::{REALSENSE_MODELS, RealSenseUnit, Stream, UnitError};
