//! The Stereolabs ZED 2i's sensor module as an [`InertialSource`].
//!
//! A ZED 2i is two USB devices behind one hub: the stereo camera (`2b03:f880`, USB Video Class,
//! read by `sx-uvc-stereo`) and a sensor module (`2b03:f881`, USB HID) carrying an accelerometer
//! and gyroscope at 400 Hz, a magnetometer, a barometer and temperatures. The camera's video
//! interface reports a placeholder serial; the sensor module reports the unit's real one.
//!
//! A robot's media plan names the sensor generically ([`InertialSensor`]): its product, the unit
//! (serial) the instance placed in its slot, and where its axes sit. [`SensorRequest::for_sensor`]
//! resolves that to a device, [`hid::open`] (feature `hid`) opens it, and
//! [`StereolabsInertialSource`] turns each HID report ([`report`], ported from Stereolabs'
//! `zed-open-capture`) into an [`InertialSample`] on the module's own axes in REP-103 units.
//! The module's axes are Stereolabs' RAW frame (x down, y towards the left eye, z out of the
//! lenses); where they sit on the body is the plan's `mount_from_sensor`, never this crate's.
//!
//! The ZED SDK is not used.

#[cfg(feature = "hid")]
pub mod hid;
pub mod report;
pub mod source;

pub use source::{
    PRODUCTS, Product, ReportPort, RequestError, SILENCE, SensorRequest, StereolabsInertialSource,
};
pub use sx_embodiment_drivers::{InertialSample, InertialSensor, InertialSource, MediaPlan};
