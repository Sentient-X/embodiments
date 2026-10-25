//! A planned ZED track captured from synthetic side-by-side frames, no hardware.

use std::time::{Duration, Instant, SystemTime};

use sx_embodiment_drivers::{CaptureError, CapturedAt, FrameSource, MediaPlan, Track};
use sx_uvc_stereo::{CaptureRequest, Device, RawFrame, RequestError, UvcStereoSource};

const PLANS: &str = include_str!("fixtures/media-plans.json");
const NOMINAL: &str = include_str!("../../sx-embodiment-drivers/tests/fixtures/media-plans.json");

fn track(fixtures: &str, key: &str) -> Track {
    let all: serde_json::Value = serde_json::from_str(fixtures).expect("fixtures");
    let value = &all[key];
    let instance = value["instance"].as_str().expect("instance");
    MediaPlan::parse(&value.to_string(), instance)
        .expect("plan")
        .tracks[0]
        .clone()
}

fn head() -> Track {
    track(PLANS, "static-upper-body-head")
}

#[test]
fn the_static_upper_body_head_is_one_zed_at_hd720_and_30_fps() {
    let track = head();
    assert_eq!(track.track, "head");
    assert_eq!(track.raster(), (2560, 720));
    let request = CaptureRequest::for_track(&track).expect("request");
    assert_eq!(request.serial, "30371653");
    assert_eq!(
        (request.eye_width, request.eye_height, request.fps),
        (1280, 720, 30)
    );
    assert_eq!(request.frame_raster(), (2560, 720));
    assert_eq!(request.cameras, ["head_left", "head_right"]);
    // Each eye carries its factory calibration: the rational model, and its own centre.
    for view in &track.views {
        assert_eq!(view.distortion_model, "rational_polynomial");
        assert_eq!(view.distortion_coefficients.len(), 8);
        assert!(view.calibration.is_some());
        assert!(
            view.moves_with
                .iter()
                .any(|joint| joint == "so101_camera_tilt_joint")
        );
    }
    assert!((track.views[0].image_from_camera[2] - 636.328_160_795_91).abs() < 1e-9);
}

#[test]
fn a_nominal_head_has_no_unit_and_is_refused_until_one_is_placed() {
    let track = track(NOMINAL, "sentient-static-upper-body");
    assert_eq!(track.views[0].model, "zed_2i");
    assert!(matches!(
        CaptureRequest::for_track(&track),
        Err(RequestError::Unplaced { .. })
    ));
}

#[test]
fn a_track_that_is_not_one_zeds_left_then_right_eye_is_refused() {
    let mut swapped = head();
    swapped.views.reverse();
    assert!(matches!(
        CaptureRequest::for_track(&swapped),
        Err(RequestError::NotOneExposure { .. })
    ));
    let mut two = head();
    two.views[1].unit = Some("40000000/right".to_owned());
    assert!(matches!(
        CaptureRequest::for_track(&two),
        Err(RequestError::NotOneExposure { .. })
    ));
    let mut fast = head();
    fast.fps = 100.0;
    assert!(matches!(
        CaptureRequest::for_track(&fast),
        Err(RequestError::Mode { .. })
    ));
    let mut other = head();
    other.views[0].model = "realsense_d435".to_owned();
    assert!(matches!(
        CaptureRequest::for_track(&other),
        Err(RequestError::NotAStereoCamera { .. })
    ));
    let mut mono = head();
    mono.views.pop();
    assert!(matches!(
        CaptureRequest::for_track(&mono),
        Err(RequestError::NotOneExposure { .. })
    ));
}

/// Side-by-side frames whose left eye is luma 50 and right eye luma 200.
struct Synthetic {
    width: u32,
    height: u32,
    start: Instant,
    n: u32,
}

impl Device for Synthetic {
    fn wait(&mut self) -> Result<RawFrame, CaptureError> {
        self.n += 1;
        let half = self.width as usize;
        let mut data = Vec::with_capacity(half * 2 * self.height as usize);
        for _ in 0..self.height {
            for x in 0..half / 2 {
                let y = if x < half / 4 { 50 } else { 200 };
                data.extend([y, 128, y, 128]);
            }
        }
        Ok(RawFrame {
            width: self.width,
            height: self.height,
            stride: half * 2,
            data,
            captured: CapturedAt {
                monotonic: self.start + Duration::from_millis(33 * u64::from(self.n)),
                wall: SystemTime::now(),
            },
        })
    }
}

#[test]
fn each_frame_is_one_exposure_of_both_planned_eyes() {
    let request = CaptureRequest::for_track(&head()).expect("request");
    let (width, height) = request.frame_raster();
    let start = Instant::now();
    let mut source = UvcStereoSource::new(
        Synthetic {
            width,
            height,
            start,
            n: 0,
        },
        request,
    );
    let mut last = None;
    for _ in 0..3 {
        let exposure = source.next().expect("exposure");
        let [left, right] = exposure.views.as_slice() else {
            panic!("two views");
        };
        assert_eq!((left.width, left.height), (1280, 720));
        assert!(left.is_well_formed() && right.is_well_formed());
        assert!(left.y.iter().all(|&y| y == 50) && right.y.iter().all(|&y| y == 200));
        assert!(last.is_none_or(|previous| exposure.captured.monotonic > previous));
        last = Some(exposure.captured.monotonic);
    }
}

#[test]
fn a_frame_of_another_raster_stops_the_track() {
    let request = CaptureRequest::for_track(&head()).expect("request");
    let mut source = UvcStereoSource::new(
        Synthetic {
            width: 1344,
            height: 376,
            start: Instant::now(),
            n: 0,
        },
        request,
    );
    assert!(matches!(source.next(), Err(CaptureError::Device(_))));
}
