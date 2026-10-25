//! Camera values and bounds supplied by the caller; no capture policy lives here.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Setting {
    pub exposure_100us: i32,
    pub gain: i32,
}

/// A camera's requested initial controls and the caller's permitted adjustment range.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExposureConfig {
    pub start: Setting,
    pub min: Setting,
    pub max: Setting,
}
