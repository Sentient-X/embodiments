//! Verified Linux camera and encoder transports. Recording policy belongs to callers.
#[cfg(target_os = "linux")]
pub mod camera;
pub mod identity;
