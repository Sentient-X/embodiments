//! Side-by-side UVC stereo cameras (the Stereolabs ZED 2i) as media-plan [`FrameSource`]s.
//!
//! A ZED exposes both eyes together and delivers them over USB Video Class as one YUYV frame,
//! left eye first, twice an eye's width. A station publishes what its robot's [`MediaPlan`]
//! names: a planned track whose two views are the left and right eyes of one device becomes one
//! [`CaptureRequest`] ([`CaptureRequest::for_track`]), which device by serial, at which eye
//! raster and rate. The device comes from the camera units the robot instance placed in the two
//! views' slots, spelled `<device serial>/left` and `<device serial>/right` ([`StereoUnit`]),
//! and the mode is one the product lists ([`PRODUCTS`]).
//!
//! [`UvcStereoSource`] splits each delivered frame into the two planned views ([`split`]) over
//! any [`Device`]: `capture::open` (feature `capture`) is the device through `nokhwa`, V4L2 on
//! Linux and `AVFoundation` on macOS, and the tests drive it with synthetic frames.
//!
//! # Capture time
//!
//! A UVC frame through `nokhwa` carries no device timestamp, so an exposure is stamped when the
//! host has the frame: later than the exposure by its readout and USB transfer, a bounded but
//! unmeasured lag (about a frame interval). The stamp is monotonic per source.
//!
//! # Pixels
//!
//! The ZED's one UVC format is YUYV 4:2:2. An eye is cut on a macropixel boundary (an eye's
//! width is even in every mode) and its chroma averaged over row pairs into I420, so luma and
//! chroma pass through unconverted, in the camera's own range.
//!
//! Depth, the IMU (the ZED's HID interface) and the ZED SDK's rectification are not here: the
//! camera is two image streams, and a unit's factory calibration is the registry's
//! (`sx_embodiments.factory_intrinsics.zed_factory_calibration`).

#[cfg(feature = "capture")]
pub mod capture;
pub mod product;
pub mod source;
pub mod split;

pub use product::{Eye, Mode, PRODUCTS, Product, StereoUnit, UnitError};
pub use source::{CaptureRequest, Device, RawFrame, RequestError, UvcStereoSource};
pub use sx_embodiment_drivers::{Exposure, FrameSource, MediaPlan};
