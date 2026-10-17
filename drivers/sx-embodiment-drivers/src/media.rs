//! What one built robot streams, and the surface every camera and microphone adapter implements.
//!
//! `sx_embodiments.media.media_document` renders, from a robot instance resolved against its
//! body, one track per declared camera (a declared stereo pair as one track, left then right)
//! and every fitted microphone, each view with its resolved optics and where it is mounted.
//! [`MediaPlan::parse`] reads that for the instance the caller is configured as and refuses any
//! other, so a publisher never spells a track, raster or lens.
//!
//! A capture adapter (a camera SDK, a microphone) implements [`FrameSource`] or
//! [`AudioSource`] for one planned track: a blocking `next` that returns the next exposure of
//! every view in the track's order, or the next audio chunk, each stamped where the sample was
//! taken. Adapters live beside the body's other drivers in this repository; a publisher only
//! ever sees these traits.

use std::time::{Instant, SystemTime};

use serde::Deserialize;
use thiserror::Error;

/// The schema `media_document` renders.
pub const MEDIA_SCHEMA: &str = "sx.media-plan/v1";
/// The fastest track a plan may declare; a rate outside 1..=this is not a camera.
pub const MAX_FPS: f64 = 1_000.0;

/// Whether `name` is a track or microphone name (`sx_embodiments.media.TRACK_NAME`): it names
/// a published track and the journal file beside it, so it is never a path.
#[must_use]
pub fn is_track_name(name: &str) -> bool {
    let bytes = name.as_bytes();
    (1..=63).contains(&bytes.len())
        && bytes[0].is_ascii_lowercase() | bytes[0].is_ascii_digit()
        && bytes
            .iter()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || *b == b'_' || *b == b'-')
}

/// A rendered media plan that is not one for this robot.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum MediaPlanError {
    #[error("media plan document: {0}")]
    Document(String),
    #[error("media plan resolved for instance {found}, this robot is {expected}")]
    Unit { expected: String, found: String },
}

/// One robot's tracks and microphones.
#[derive(Clone, Debug, PartialEq)]
pub struct MediaPlan {
    /// The body's content id.
    pub embodiment: String,
    /// The instance's content id.
    pub instance: String,
    pub tracks: Vec<Track>,
    pub microphones: Vec<Microphone>,
}

/// One published video track: one camera, or a stereo pair side by side.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Track {
    pub track: String,
    pub fps: f64,
    /// Left to right; one view, or a stereo pair's left and right.
    pub views: Vec<View>,
}

impl Track {
    /// The published raster: every view side by side.
    #[must_use]
    pub fn raster(&self) -> (u32, u32) {
        let width = self.views.iter().map(|view| view.width).sum();
        let height = self.views.first().map_or(0, |view| view.height);
        (width, height)
    }
}

/// One declared camera as the instance resolved it.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct View {
    pub camera: String,
    /// The camera product's registry id (`sx_embodiments.cameras`).
    pub model: String,
    pub modality: String,
    pub width: u32,
    pub height: u32,
    pub fps: f64,
    /// Row-major 3x3, measured when `calibration` is set, nominal otherwise.
    pub image_from_camera: Vec<f64>,
    pub distortion_model: String,
    pub distortion_coefficients: Vec<f64>,
    /// The serial unit placed in this camera's slot.
    pub unit: Option<String>,
    /// The camera-unit calibration the optics came from; `None` when nominal.
    pub calibration: Option<String>,
    /// The description link the camera is mounted on.
    pub mount_frame: String,
    /// The description's root link.
    pub root_frame: String,
    /// The mount link in the root frame, when only fixed joints lie between them.
    pub root_from_mount: Option<Pose>,
    /// The moving joints between the root and the mount, root first; empty for a fixed mount.
    pub moves_with: Vec<String>,
}

/// A rigid pose: translation in metres and a unit quaternion (x, y, z, w).
#[derive(Clone, Copy, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Pose {
    pub translation_m: [f64; 3],
    pub rotation_xyzw: [f64; 4],
}

/// One fitted microphone.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Microphone {
    pub microphone: String,
    pub model: String,
    pub sample_rate_hz: u32,
    pub channels: u32,
    pub mount_frame: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    schema: String,
    embodiment: String,
    instance: String,
    tracks: Vec<Track>,
    microphones: Vec<Microphone>,
}

impl MediaPlan {
    /// Parse a rendered `sx.media-plan/v1` document resolved for `expected`, the content id of
    /// the robot instance the caller is configured as.
    ///
    /// # Errors
    ///
    /// Returns [`MediaPlanError::Document`] for anything but a coherent such document (a track
    /// without views, views of different heights or rates, a pose or lens of the wrong shape),
    /// and [`MediaPlanError::Unit`] for one resolved for another instance.
    pub fn parse(document: &str, expected: &str) -> Result<Self, MediaPlanError> {
        let Document {
            schema,
            embodiment,
            instance,
            tracks,
            microphones,
        } = serde_json::from_str(document)
            .map_err(|error| MediaPlanError::Document(error.to_string()))?;
        if schema != MEDIA_SCHEMA {
            return Err(MediaPlanError::Document(format!(
                "not a {MEDIA_SCHEMA} document"
            )));
        }
        if instance != expected {
            return Err(MediaPlanError::Unit {
                expected: expected.to_owned(),
                found: instance,
            });
        }
        let mut names = std::collections::HashSet::new();
        for track in &tracks {
            let Some(first) = track.views.first() else {
                return Err(MediaPlanError::Document(format!(
                    "track {} has no view",
                    track.track
                )));
            };
            if track.views.len() > 2
                || !(1.0..=MAX_FPS).contains(&track.fps)
                || track.views.iter().any(|view| {
                    view.height != first.height
                        || view.width == 0
                        || view.height == 0
                        || (view.fps - track.fps).abs() > f64::EPSILON
                        || view.image_from_camera.len() != 9
                        || view.root_from_mount.is_none() == view.moves_with.is_empty()
                        || view.root_from_mount.is_some_and(|pose| {
                            let norm: f64 =
                                pose.rotation_xyzw.iter().map(|value| value * value).sum();
                            (norm.sqrt() - 1.0).abs() > 1e-6
                        })
                })
            {
                return Err(MediaPlanError::Document(format!(
                    "track {} is not one exposure of at most two views",
                    track.track
                )));
            }
            if !is_track_name(&track.track) {
                return Err(MediaPlanError::Document(format!(
                    "{:?} is not a track name",
                    track.track
                )));
            }
            if !names.insert(track.track.clone()) {
                return Err(MediaPlanError::Document(format!(
                    "two tracks are named {}",
                    track.track
                )));
            }
        }
        for microphone in &microphones {
            if !is_track_name(&microphone.microphone)
                || !names.insert(microphone.microphone.clone())
                || !matches!(microphone.sample_rate_hz, 16_000 | 24_000 | 32_000 | 48_000)
                || !matches!(microphone.channels, 1 | 2)
            {
                return Err(MediaPlanError::Document(format!(
                    "microphone {} is not a publishable track",
                    microphone.microphone
                )));
            }
        }
        Ok(Self {
            embodiment,
            instance,
            tracks,
            microphones,
        })
    }
}

/// One planar 8-bit 4:2:0 image (I420): `y` is `width * height`, `u` and `v` are each
/// `ceil(width / 2) * ceil(height / 2)`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct I420Image {
    pub width: u32,
    pub height: u32,
    pub y: Vec<u8>,
    pub u: Vec<u8>,
    pub v: Vec<u8>,
}

impl I420Image {
    /// A flat image of one colour.
    #[must_use]
    pub fn filled(width: u32, height: u32, (y, u, v): (u8, u8, u8)) -> Self {
        let luma = width as usize * height as usize;
        let chroma = width.div_ceil(2) as usize * height.div_ceil(2) as usize;
        Self {
            width,
            height,
            y: vec![y; luma],
            u: vec![u; chroma],
            v: vec![v; chroma],
        }
    }

    /// Whether the planes are the sizes the raster says.
    #[must_use]
    pub fn is_well_formed(&self) -> bool {
        let luma = self.width as usize * self.height as usize;
        let chroma = self.width.div_ceil(2) as usize * self.height.div_ceil(2) as usize;
        self.y.len() == luma && self.u.len() == chroma && self.v.len() == chroma
    }
}

/// When one sample was taken: on this host's monotonic clock, and on its wall clock (which
/// the far end of a link can compare against its own, synchronised one).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CapturedAt {
    pub monotonic: Instant,
    pub wall: SystemTime,
}

/// One exposure of every view of a track, in the track's order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Exposure {
    pub captured: CapturedAt,
    pub views: Vec<I420Image>,
}

/// One chunk of interleaved signed 16-bit audio.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AudioChunk {
    pub captured: CapturedAt,
    pub sample_rate_hz: u32,
    pub channels: u32,
    pub samples: Vec<i16>,
}

/// A capture device failed or ended.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum CaptureError {
    #[error("capture device ended")]
    Ended,
    #[error("capture device failed: {0}")]
    Device(String),
}

/// One planned video track's capture: a camera, or a stereo pair exposed together.
pub trait FrameSource: Send {
    /// Block until the next exposure. Every view is the planned raster, in the track's order.
    ///
    /// # Errors
    ///
    /// Returns [`CaptureError`] when the device fails or ends; a publisher stops the track.
    fn next(&mut self) -> Result<Exposure, CaptureError>;
}

/// One planned microphone's capture.
pub trait AudioSource: Send {
    /// Block until the next chunk, at the planned rate and channel count.
    ///
    /// # Errors
    ///
    /// Returns [`CaptureError`] when the device fails or ends; a publisher stops the track.
    fn next(&mut self) -> Result<AudioChunk, CaptureError>;
}

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = include_str!("../tests/fixtures/media-plans.json");

    fn plan(key: &str) -> String {
        let fixtures: serde_json::Value = serde_json::from_str(FIXTURE).expect("fixture");
        fixtures[key].to_string()
    }

    fn instance(document: &str) -> String {
        let value: serde_json::Value = serde_json::from_str(document).expect("plan");
        value["instance"].as_str().expect("instance").to_owned()
    }

    #[test]
    fn a_stereo_pair_is_one_track_twice_as_wide() {
        let document = plan("quest-ego-stereo");
        let parsed = MediaPlan::parse(&document, &instance(&document)).expect("plan");
        assert_eq!(parsed.tracks.len(), 1);
        let track = &parsed.tracks[0];
        assert_eq!(
            track
                .views
                .iter()
                .map(|view| view.camera.as_str())
                .collect::<Vec<_>>(),
            ["head_left", "head_right"]
        );
        assert_eq!(track.raster(), (2560, 960));
        assert_eq!(parsed.microphones[0].sample_rate_hz, 48_000);
    }

    #[test]
    fn each_unpaired_camera_is_its_own_track() {
        let document = plan("quest-ego-mono");
        let parsed = MediaPlan::parse(&document, &instance(&document)).expect("plan");
        assert_eq!(parsed.tracks.len(), 2);
        assert!(parsed.microphones.is_empty());
    }

    #[test]
    fn a_camera_riding_moving_joints_names_them_and_carries_no_fixed_pose() {
        let document = plan("sentient-humanoid-upper-body");
        let parsed = MediaPlan::parse(&document, &instance(&document)).expect("plan");
        let view = &parsed.tracks[0].views[0];
        assert!(view.root_from_mount.is_none());
        assert!(
            view.moves_with
                .iter()
                .any(|joint| joint == "neck_yaw_joint")
        );
    }

    #[test]
    fn a_plan_for_another_robot_or_of_another_shape_is_refused() {
        let document = plan("quest-ego-stereo");
        assert!(matches!(
            MediaPlan::parse(&document, "another"),
            Err(MediaPlanError::Unit { .. })
        ));
        let mut value: serde_json::Value = serde_json::from_str(&document).expect("plan");
        value["tracks"][0]["views"][1]["height"] = serde_json::json!(480);
        assert!(matches!(
            MediaPlan::parse(&value.to_string(), &instance(&document)),
            Err(MediaPlanError::Document(_))
        ));
        let mut value: serde_json::Value = serde_json::from_str(&document).expect("plan");
        value["tracks"][0]["fps"] = serde_json::json!(1e-20);
        assert!(matches!(
            MediaPlan::parse(&value.to_string(), &instance(&document)),
            Err(MediaPlanError::Document(_))
        ));
        for name in ["../escape", "/tmp/x", "Head", ""] {
            let mut value: serde_json::Value = serde_json::from_str(&document).expect("plan");
            value["tracks"][0]["track"] = serde_json::json!(name);
            assert!(matches!(
                MediaPlan::parse(&value.to_string(), &instance(&document)),
                Err(MediaPlanError::Document(_))
            ));
        }
        let mut value: serde_json::Value = serde_json::from_str(&document).expect("plan");
        value["tracks"][0]["views"][0]["lens"] = serde_json::json!("extra");
        assert!(matches!(
            MediaPlan::parse(&value.to_string(), &instance(&document)),
            Err(MediaPlanError::Document(_))
        ));
    }

    #[test]
    fn a_filled_image_is_well_formed_at_an_odd_raster() {
        assert!(I420Image::filled(3, 5, (16, 128, 128)).is_well_formed());
    }
}
