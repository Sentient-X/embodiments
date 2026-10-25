//! Capture device transport, independent of recording and Factory policy.
pub mod cdc;
#[cfg(any(target_os = "linux", target_os = "android"))]
pub mod hw;
pub mod source;
pub mod usb;
pub mod uvc;
