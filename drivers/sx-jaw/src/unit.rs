//! One gripper unit's measured jaw applied to a take (`sx_embodiments.jaw_unit`'s
//! `JawUnitCalibration`, fitted once from a calipered protocol take): its markers' fixed
//! references, the map from its tags and from its encoder onto the finger angle, and its pads'
//! compression under a squeeze, delivered as the angle the jaw law reads as the pads' gap.

use crate::fusion::{EncoderReads, FusedJaw, FusionModel, TagReads, tag_reads};
use crate::geometry::interp;
use crate::tags::{Frame, JawReference};

/// `finger angle = scale * reading + offset_rad`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinearMap {
    pub scale: f64,
    pub offset_rad: f64,
}

impl LinearMap {
    pub fn apply(&self, reading: f64) -> f64 {
        self.scale * reading + self.offset_rad
    }
}

/// The canonical jaw law sampled from its closed to its open stop: `(angle, gap)` pairs with
/// the gap strictly increasing (`jaw_unit._law`).
#[derive(Clone, Debug, PartialEq)]
pub struct JawLaw {
    pub angles_rad: Vec<f64>,
    pub gaps_m: Vec<f64>,
}

impl JawLaw {
    pub fn new(angles_rad: Vec<f64>, gaps_m: Vec<f64>) -> Result<Self, String> {
        if angles_rad.len() != gaps_m.len() || angles_rad.len() < 2 {
            return Err("the jaw law needs at least two (angle, gap) pairs".to_owned());
        }
        if gaps_m.windows(2).any(|w| w[1] <= w[0]) || angles_rad.windows(2).any(|w| w[1] <= w[0]) {
            return Err("the jaw law's gap must grow with its angle".to_owned());
        }
        Ok(Self { angles_rad, gaps_m })
    }

    fn slope(&self) -> f64 {
        (self.gaps_m[1] - self.gaps_m[0]) / (self.angles_rad[1] - self.angles_rad[0])
    }

    /// The rigid gap at an angle, clamped at the open stop and continued on its closing slope
    /// below the closed stop.
    pub fn gap_m(&self, angle_rad: f64) -> f64 {
        if angle_rad < self.angles_rad[0] {
            return self.gaps_m[0] + self.slope() * (angle_rad - self.angles_rad[0]);
        }
        interp(angle_rad, &self.angles_rad, &self.gaps_m)
    }

    /// The angle at which [`Self::gap_m`] reads `gap_m`.
    pub fn angle_at(&self, gap_m: f64) -> f64 {
        if gap_m < self.gaps_m[0] {
            return self.angles_rad[0] + (gap_m - self.gaps_m[0]) / self.slope();
        }
        interp(gap_m, &self.gaps_m, &self.angles_rad)
    }
}

/// One unit's calibration as the solve applies it.
#[derive(Clone, Debug, PartialEq)]
pub struct JawUnit {
    pub reference: JawReference,
    pub tag_map: LinearMap,
    pub encoder_map: LinearMap,
    pub pad_mm_per_rad: f64,
    pub law: JawLaw,
}

impl JawUnit {
    /// Each frame's finger angle from this unit's tags (NaN where unseen).
    pub fn tags(
        &self,
        t_ns: &[i64],
        frames: &[Frame],
        model: &FusionModel,
    ) -> Result<TagReads, String> {
        let raw = tag_reads(&self.reference, t_ns, frames, model)?;
        let scale = self.tag_map.scale.abs();
        TagReads::new(
            raw.t_ns,
            raw.angle_rad
                .iter()
                .map(|a| self.tag_map.apply(*a))
                .collect(),
            raw.std_rad.iter().map(|s| s * scale).collect(),
            raw.mirror_rad,
        )
    }

    /// The encoder's reads as finger angles in free motion.
    pub fn map_encoder(&self, encoder: &EncoderReads) -> EncoderReads {
        EncoderReads {
            t_ns: encoder.t_ns.clone(),
            angle_rad: encoder
                .angle_rad
                .iter()
                .map(|a| self.encoder_map.apply(*a))
                .collect(),
        }
    }

    /// The pads' gap: the fingers' rigid gap plus the pads' compression the squeeze says.
    pub fn gap_m(&self, fused: &FusedJaw) -> Vec<f64> {
        fused
            .angle_rad
            .iter()
            .zip(&fused.encoder_bias_rad)
            .map(|(angle, bias)| {
                self.law.gap_m(*angle) + self.pad_mm_per_rad * (-bias).max(0.0) / 1000.0
            })
            .collect()
    }

    /// The joint angle the jaw law reads as [`Self::gap_m`], never below closed.
    pub fn delivered_angle(&self, fused: &FusedJaw) -> Vec<f64> {
        self.gap_m(fused)
            .iter()
            .map(|gap| self.law.angle_at(gap.max(0.0)))
            .collect()
    }
}
