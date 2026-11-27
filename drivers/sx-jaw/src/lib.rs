//! The one YUBI jaw solver (sx-embodiments `drivers/sx-jaw`), run on the headset while it
//! records and by the Library afterwards, from the same code:
//!
//! - [`detect`]: ArUco `DICT_4X4_50` finger markers on an 8-bit luma plane, OpenCV's detector
//!   with sub-pixel corners, restricted to a region and the wanted IDs;
//! - [`track`]: the online sighting loop's budget: which frames to analyse and where to look;
//! - [`sightings`]: the per-frame record between detection and solve
//!   (`gripper_{side}.jaw_tags.csv`);
//! - [`lens`], [`tags`]: each marker's pose (IPPE through the fisheye) and its hinge angle;
//! - [`fusion`]: one robust smoother over tags and encoder;
//! - [`solve`], [`unit`]: one side of one take, its reference and its spool rows.
//!
//! Pure computation: no I/O, no threads, no allocation per frame once warm in [`detect`].

#![forbid(unsafe_code)]

pub mod detect;
pub mod dictionary;
pub mod fusion;
pub mod geometry;
pub mod lens;
pub mod sightings;
pub mod solve;
pub mod tags;
pub mod track;
pub mod unit;
