//! A user-space UVC client: what Linux `uvcvideo` does for the laptop pod, over usbfs.

pub mod clock;
pub mod controls;
pub mod descriptors;
pub mod payload;
pub mod probe;
pub mod stream;
