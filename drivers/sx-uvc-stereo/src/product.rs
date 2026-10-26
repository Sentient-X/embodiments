//! The side-by-side stereo products this crate captures, and which eye a camera unit is.

use std::fmt;

use thiserror::Error;

/// One eye raster and the rates the product runs it at.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mode {
    pub eye_width: u32,
    pub eye_height: u32,
    pub fps: &'static [u32],
}

/// One stereo camera product: its registry id, its USB identity and its UVC modes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Product {
    /// The camera-model registry id (`sx_embodiments.cameras`).
    pub model: &'static str,
    pub label: &'static str,
    pub vendor_id: u16,
    /// The video interface's product id.
    pub product_id: u16,
    /// The product id of the device that carries the unit's serial, where the video interface
    /// reports a placeholder instead. A ZED 2i's video interface says `OV0001`; its sensor
    /// module, on the USB 2 half of the same connector, says the unit's serial.
    pub serial_product_id: Option<u16>,
    pub modes: &'static [Mode],
}

impl Product {
    /// The product with registry id `model`.
    #[must_use]
    pub fn of(model: &str) -> Option<&'static Self> {
        PRODUCTS.iter().find(|product| product.model == model)
    }

    /// Whether the product runs eyes of `width` x `height` at `fps`.
    #[must_use]
    pub fn runs(&self, width: u32, height: u32, fps: u32) -> bool {
        self.modes.iter().any(|mode| {
            (mode.eye_width, mode.eye_height) == (width, height) && mode.fps.contains(&fps)
        })
    }
}

/// Every product this crate captures. The ZED 2i's modes are Stereolabs' video-mode table
/// (`docs.stereolabs.com/docs/video/camera-controls`): HD2K, HD1080, HD720 and VGA side by
/// side; through a USB 2 link the camera offers VGA alone.
pub const PRODUCTS: [Product; 1] = [Product {
    model: "zed_2i",
    label: "Stereolabs ZED 2i",
    vendor_id: 0x2b03,
    product_id: 0xf880,
    serial_product_id: Some(0xf881),
    modes: &[
        Mode {
            eye_width: 2208,
            eye_height: 1242,
            fps: &[15],
        },
        Mode {
            eye_width: 1920,
            eye_height: 1080,
            fps: &[30, 15],
        },
        Mode {
            eye_width: 1280,
            eye_height: 720,
            fps: &[60, 30, 15],
        },
        Mode {
            eye_width: 672,
            eye_height: 376,
            fps: &[100, 60, 30, 15],
        },
    ],
}];

/// One eye of a stereo device, seen from behind it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Eye {
    Left,
    Right,
}

impl Eye {
    /// The spelling a unit uses.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Left => "left",
            Self::Right => "right",
        }
    }
}

impl fmt::Display for Eye {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A placed unit that is not one eye of a stereo device.
#[derive(Debug, Error, PartialEq, Eq)]
#[error("{unit:?} is not a stereo eye unit (<device serial>/left|right)")]
pub struct UnitError {
    pub unit: String,
}

/// One camera unit: an eye of the device with this USB serial.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct StereoUnit {
    pub serial: String,
    pub eye: Eye,
}

impl StereoUnit {
    /// Parse `<device serial>/left` or `<device serial>/right`.
    ///
    /// # Errors
    ///
    /// Returns [`UnitError`] for anything else, including a serial that is not alphanumeric.
    pub fn parse(unit: &str) -> Result<Self, UnitError> {
        let refuse = || UnitError {
            unit: unit.to_owned(),
        };
        let (serial, eye) = unit.split_once('/').ok_or_else(refuse)?;
        let eye = match eye {
            "left" => Eye::Left,
            "right" => Eye::Right,
            _ => return Err(refuse()),
        };
        if serial.is_empty() || !serial.bytes().all(|byte| byte.is_ascii_alphanumeric()) {
            return Err(refuse());
        }
        Ok(Self {
            serial: serial.to_owned(),
            eye,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_unit_is_one_eye_of_one_serial() {
        assert_eq!(
            StereoUnit::parse("30371653/right"),
            Ok(StereoUnit {
                serial: "30371653".to_owned(),
                eye: Eye::Right,
            })
        );
        for bad in [
            "30371653",
            "30371653/infrared_1",
            "/left",
            "30-37/left",
            "a/b/left",
        ] {
            assert!(StereoUnit::parse(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn the_zed_2i_runs_the_stereolabs_table_and_nothing_else() {
        let zed = Product::of("zed_2i").expect("zed");
        assert!(zed.runs(1280, 720, 30) && zed.runs(672, 376, 100) && zed.runs(2208, 1242, 15));
        assert!(!zed.runs(1280, 720, 100) && !zed.runs(1344, 376, 15) && !zed.runs(2208, 1242, 30));
        assert!(zed.modes.iter().all(|mode| mode.eye_width % 2 == 0));
    }
}
