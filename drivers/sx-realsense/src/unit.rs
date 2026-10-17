//! Which RealSense imager a camera unit is: `<device serial>/<stream>`.

use std::fmt;

use thiserror::Error;

/// The camera-model registry ids (`sx_embodiments.cameras`) this crate captures.
pub const REALSENSE_MODELS: [&str; 3] = ["realsense_d405", "realsense_d435", "realsense_d435i"];

/// One imager stream of a D400 device.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Stream {
    Color,
    /// The left imager, seen from behind the camera.
    Infrared1,
    /// The right imager, one baseline from the left.
    Infrared2,
}

impl Stream {
    /// The spelling a unit and a factory record use.
    #[must_use]
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Color => "color",
            Self::Infrared1 => "infrared_1",
            Self::Infrared2 => "infrared_2",
        }
    }

    /// The camera modality (`sx_embodiments.parts.CameraModality`) a view of this stream has.
    #[must_use]
    pub fn modality(self) -> &'static str {
        match self {
            Self::Color => "rgb",
            Self::Infrared1 | Self::Infrared2 => "infrared",
        }
    }

    /// librealsense's stream index: 1 and 2 for the imagers, none for colour.
    #[must_use]
    pub fn index(self) -> Option<usize> {
        match self {
            Self::Color => None,
            Self::Infrared1 => Some(1),
            Self::Infrared2 => Some(2),
        }
    }

    /// Whether the projector's pattern reaches this imager.
    #[must_use]
    pub fn is_infrared(self) -> bool {
        !matches!(self, Self::Color)
    }

    fn parse(text: &str) -> Option<Self> {
        match text {
            "color" => Some(Self::Color),
            "infrared_1" => Some(Self::Infrared1),
            "infrared_2" => Some(Self::Infrared2),
            _ => None,
        }
    }
}

impl fmt::Display for Stream {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A placed unit that is not one RealSense imager.
#[derive(Debug, Error, PartialEq, Eq)]
#[error("{unit:?} is not a RealSense imager unit (<device serial>/color|infrared_1|infrared_2)")]
pub struct UnitError {
    pub unit: String,
}

/// One camera unit: an imager of the device with this serial.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct RealSenseUnit {
    /// The device's serial number as librealsense reports it.
    pub serial: String,
    pub stream: Stream,
}

impl RealSenseUnit {
    /// Read a placed unit.
    ///
    /// # Errors
    ///
    /// Returns [`UnitError`] unless the unit is a non-empty serial of ASCII letters and digits,
    /// a slash and a stream.
    pub fn parse(unit: &str) -> Result<Self, UnitError> {
        let refuse = || UnitError {
            unit: unit.to_owned(),
        };
        let (serial, stream) = unit.split_once('/').ok_or_else(refuse)?;
        if serial.is_empty() || !serial.bytes().all(|b| b.is_ascii_alphanumeric()) {
            return Err(refuse());
        }
        Ok(Self {
            serial: serial.to_owned(),
            stream: Stream::parse(stream).ok_or_else(refuse)?,
        })
    }
}

impl fmt::Display for RealSenseUnit {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}", self.serial, self.stream)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_unit_names_one_imager_of_one_device() {
        let unit = RealSenseUnit::parse("923322071108/infrared_2").expect("unit");
        assert_eq!(unit.serial, "923322071108");
        assert_eq!(unit.stream, Stream::Infrared2);
        assert_eq!(unit.to_string(), "923322071108/infrared_2");
        for bad in [
            "923322071108",
            "/color",
            "9233/depth",
            "92 33/color",
            "../x/color",
            "9233/color/x",
        ] {
            assert!(RealSenseUnit::parse(bad).is_err(), "{bad}");
        }
    }
}
