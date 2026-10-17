//! What a unit's own factory calibration says for one active stream profile.
//!
//! librealsense reports each stream profile's intrinsics from the calibration Intel writes into
//! the device. [`FactoryIntrinsics::document`] renders that as `sx.factory-intrinsics/v1`, in
//! librealsense's own terms (the model by its SDK name, five coefficients in the SDK's order);
//! `sx_embodiments.factory_intrinsics` is its one reader, and the one place those terms become
//! the platform's distortion vocabulary.

use serde::Serialize;

use crate::unit::RealSenseUnit;

/// The schema [`FactoryIntrinsics::document`] renders.
pub const FACTORY_SCHEMA: &str = "sx.factory-intrinsics/v1";

/// librealsense's distortion models (`rs2_distortion`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Distortion {
    None,
    BrownConrady,
    ModifiedBrownConrady,
    InverseBrownConrady,
    Ftheta,
    KannalaBrandt4,
}

/// One stream profile's factory intrinsics.
#[derive(Clone, Debug, PartialEq)]
pub struct FactoryIntrinsics {
    pub unit: RealSenseUnit,
    /// The device's product name (`RS2_CAMERA_INFO_NAME`).
    pub product: String,
    /// The firmware that reported the calibration.
    pub firmware: String,
    pub width: u32,
    pub height: u32,
    pub fx: f32,
    pub fy: f32,
    pub ppx: f32,
    pub ppy: f32,
    pub distortion: Distortion,
    pub coefficients: [f32; 5],
}

#[derive(Serialize)]
struct Document<'a> {
    schema: &'static str,
    vendor: &'static str,
    product: &'a str,
    firmware: &'a str,
    unit: String,
    serial: &'a str,
    stream: &'static str,
    width: u32,
    height: u32,
    fx: f64,
    fy: f64,
    ppx: f64,
    ppy: f64,
    distortion: Distortion,
    coefficients: [f64; 5],
}

impl FactoryIntrinsics {
    /// The record as `sx.factory-intrinsics/v1` JSON, every SDK float widened exactly.
    #[must_use]
    pub fn document(&self) -> String {
        let document = Document {
            schema: FACTORY_SCHEMA,
            vendor: "realsense",
            product: &self.product,
            firmware: &self.firmware,
            unit: self.unit.to_string(),
            serial: &self.unit.serial,
            stream: self.unit.stream.as_str(),
            width: self.width,
            height: self.height,
            fx: f64::from(self.fx),
            fy: f64::from(self.fy),
            ppx: f64::from(self.ppx),
            ppy: f64::from(self.ppy),
            distortion: self.distortion,
            coefficients: self.coefficients.map(f64::from),
        };
        serde_json::to_string_pretty(&document).unwrap_or_default() + "\n"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::unit::Stream;

    /// LuViRA's published `rs-enumerate-devices -c` dump of its D435i colour profile at 640x480
    /// (the dump prints each SDK float widened, 606.965881347656 being `606.965_9_f32`)
    /// (`ilaydayaman/LuViRA_Dataset`, `db30d579a24aec5f784de866f1b1a81bb9aaa4e8`,
    /// `vision/camera_factory_cal.txt`, CC-BY-4.0). The dump omits the device serial and
    /// firmware; the fixture's are placeholders.
    pub(crate) fn luvira_colour() -> FactoryIntrinsics {
        FactoryIntrinsics {
            unit: RealSenseUnit {
                serial: "000000000000".to_owned(),
                stream: Stream::Color,
            },
            product: "Intel RealSense D435I".to_owned(),
            firmware: "unrecorded".to_owned(),
            width: 640,
            height: 480,
            fx: 606.965_9,
            fy: 605.677_3,
            ppx: 310.739_65,
            ppy: 242.007_55,
            distortion: Distortion::InverseBrownConrady,
            coefficients: [0.0; 5],
        }
    }

    #[test]
    fn a_real_factory_record_renders_the_fixture_its_reader_is_tested_against() {
        let fixture = include_str!("../tests/fixtures/factory-luvira-d435i-color.json");
        assert_eq!(luvira_colour().document(), fixture);
    }
}
