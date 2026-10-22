//! Interfaces supplied by capture device drivers. No recording or upload policy.
use serde::Serialize;
use serde_json::Value;
use std::sync::Arc;
use sx_yubi::camera::{ExposureConfig, Setting};

/// The host monotonic clock used to timestamp completed device transfers.
pub type MonotonicClock = fn() -> std::io::Result<i64>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    Camera,
    Encoder,
    /// Any other USB device: enumerated, reported, never opened.
    Other,
}

/// How a device stream ended badly: the device went away, or it misbehaved.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SourceError {
    Disconnected(String),
    Failed(String),
}

impl std::fmt::Display for SourceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Disconnected(detail) => write!(f, "device disconnected: {detail}"),
            Self::Failed(detail) => f.write_str(detail),
        }
    }
}

/// The exposure control of one open camera, callable from its encode thread.
pub type ExposureControl = Arc<dyn Fn(Setting, bool) -> Result<Setting, String> + Send + Sync>;

pub trait CameraSource: Send {
    /// Wait at most ~100 ms for transfers and append the frames they completed.
    fn next(&mut self, out: &mut Vec<SourceFrame>) -> Result<(), SourceError>;
    fn exposure(&self) -> ExposureControl;
    /// Stream statistics and clock quality, for the receipt.
    fn report(&self) -> Value;
    fn close(self: Box<Self>);
}

pub trait EncoderSource: Send {
    /// Wait at most ~200 ms; `Ok(0)` when nothing arrived.
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, SourceError>;
    /// What the link under it has seen, for an error to name.
    fn diagnostics(&self) -> Option<String> {
        None
    }
    /// A link silent through its startup window gets one re-open (both control lines down,
    /// a pause, both up). `Ok(false)` when the source cannot re-open.
    fn reopen(&mut self) -> Result<bool, String> {
        Ok(false)
    }
    fn close(self: Box<Self>);
}

/// One attached USB device.
pub trait Port: Send + Sync {
    fn kind(&self) -> Kind;
    fn describe(&self) -> Value;
    /// Camera flash UID or `usb-serial:<MAC>`.
    fn identify(&self) -> Result<String, String>;
    /// The physical USB port path (`<bus>-<port>.<port>…`), which survives a replug into
    /// the same hub port.
    fn port_path(&self) -> Result<String, String> {
        Err("this device reports no port path".to_owned())
    }
    fn open_camera(&self, config: ExposureConfig) -> Result<Box<dyn CameraSource>, String>;
    fn open_encoder(&self) -> Result<Box<dyn EncoderSource>, String>;
    /// Called while the device's fd is still the rig's (detach, rig close): an encoder's
    /// link lowers its control lines, as `cdc_acm`'s close does.
    fn release(&self) {}
}

/// A source frame as the selector consumes it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceFrame {
    pub jpeg: Vec<u8>,
    pub soe_ns: i64,
    pub sequence: u32,
    pub pts: u32,
}
