//! A device that replays recorded framesets, then ends.

use std::collections::VecDeque;

use sx_embodiment_drivers::CaptureError;

use crate::source::{Device, RawFrame};

/// Replays framesets in order; [`CaptureError::Ended`] once they run out.
#[derive(Clone, Debug, Default)]
pub struct RecordedDevice {
    framesets: VecDeque<Vec<RawFrame>>,
}

impl RecordedDevice {
    #[must_use]
    pub fn new(framesets: impl IntoIterator<Item = Vec<RawFrame>>) -> Self {
        Self {
            framesets: framesets.into_iter().collect(),
        }
    }
}

impl Device for RecordedDevice {
    fn wait(&mut self) -> Result<Vec<RawFrame>, CaptureError> {
        self.framesets.pop_front().ok_or(CaptureError::Ended)
    }
}
