//! A planned RealSense track captured from recorded framesets, no hardware.

use std::time::{Duration, Instant, UNIX_EPOCH};

use sx_embodiment_drivers::{CaptureError, FrameSource, MediaPlan, Track};
use sx_realsense::clock::HostSample;
use sx_realsense::recorded::RecordedDevice;
use sx_realsense::{
    CaptureRequest, Emitter, Pixels, RawFrame, RealSenseSource, RequestError, Stream,
    TimestampDomain,
};

const PLANS: &str = include_str!("fixtures/media-plans.json");
const NOMINAL: &str = include_str!("../../sx-embodiment-drivers/tests/fixtures/media-plans.json");
const EPOCH_MS: f64 = 1_760_000_000_000.0;

fn document(fixtures: &str, key: &str) -> serde_json::Value {
    let all: serde_json::Value = serde_json::from_str(fixtures).expect("fixtures");
    all[key].clone()
}

fn track(value: &serde_json::Value) -> Track {
    let text = value.to_string();
    let instance = value["instance"].as_str().expect("instance").to_owned();
    MediaPlan::parse(&text, &instance).expect("plan").tracks[0].clone()
}

fn frame(stream: Stream, number: u64, at_ms: f64, (width, height): (u32, u32)) -> RawFrame {
    let pixels = if stream.is_infrared() {
        Pixels::Y8 {
            stride: width as usize,
            data: vec![200; width as usize * height as usize],
        }
    } else {
        Pixels::Rgb8 {
            stride: width as usize * 3,
            data: vec![255; width as usize * height as usize * 3],
        }
    };
    RawFrame {
        stream,
        number,
        timestamp_ms: at_ms,
        domain: TimestampDomain::GlobalTime,
        emitter_lit: stream.is_infrared().then_some(false),
        width,
        height,
        pixels,
    }
}

/// A host clock that reads 30 ms after each exposure the replay holds.
fn clock(start: Instant) -> Box<dyn FnMut() -> HostSample + Send> {
    let mut n = 0_u32;
    Box::new(move || {
        n += 1;
        HostSample {
            monotonic: start + Duration::from_millis(1_000 + 40 * u64::from(n)),
            wall: UNIX_EPOCH
                + Duration::from_secs_f64((EPOCH_MS + 40.0 * f64::from(n) + 30.0) / 1e3),
        }
    })
}

#[test]
fn the_head_colour_stream_is_one_device_imager_at_its_planned_raster() {
    let request = CaptureRequest::for_track(
        &track(&document(PLANS, "humanoid-head-color")),
        Emitter::Off,
    )
    .expect("request");
    assert_eq!(request.serial, "923322071108");
    assert_eq!((request.fps, request.device_fps), (30, 30));
    assert_eq!(request.views.len(), 1);
    assert_eq!(request.views[0].stream, Stream::Color);
    assert_eq!(
        (request.views[0].width, request.views[0].height),
        (1280, 720)
    );

    let raster = (1280, 720);
    let start = Instant::now();
    let device = RecordedDevice::new([
        vec![frame(Stream::Color, 7, EPOCH_MS + 40.0, raster)],
        // The same frame again is not a new exposure.
        vec![frame(Stream::Color, 7, EPOCH_MS + 40.0, raster)],
        vec![frame(Stream::Color, 8, EPOCH_MS + 120.0, raster)],
    ]);
    let mut source = RealSenseSource::with_clock(device, request, clock(start));
    let first = source.next().expect("first");
    assert_eq!(first.views.len(), 1);
    assert!(first.views[0].is_well_formed());
    assert_eq!(first.views[0].y[0], 235);
    // Sampled 30 ms after the exposure, at monotonic start + 1040 ms.
    let age = (start + Duration::from_millis(1_040)) - first.captured.monotonic;
    assert!(age.abs_diff(Duration::from_millis(30)) < Duration::from_micros(1));
    let second = source.next().expect("second");
    assert!(second.captured.monotonic > first.captured.monotonic);
    assert_eq!(source.next(), Err(CaptureError::Ended));
}

#[test]
fn the_infrared_pair_is_one_exposure_and_interleaving_publishes_the_dark_frames() {
    let plan = document(PLANS, "humanoid-head-infrared");
    let request = CaptureRequest::for_track(&track(&plan), Emitter::Interleaved).expect("pair");
    assert_eq!(
        request
            .views
            .iter()
            .map(|view| view.stream)
            .collect::<Vec<_>>(),
        [Stream::Infrared1, Stream::Infrared2]
    );
    assert_eq!((request.fps, request.device_fps), (60, 120));

    let raster = (848, 480);
    let pair = |number, at, lit: bool| {
        [Stream::Infrared1, Stream::Infrared2]
            .map(|stream| RawFrame {
                emitter_lit: Some(lit),
                ..frame(stream, number, at, raster)
            })
            .to_vec()
    };
    let device = RecordedDevice::new([
        pair(1, EPOCH_MS + 40.0, true),
        pair(2, EPOCH_MS + 48.0, false),
        pair(3, EPOCH_MS + 56.0, true),
    ]);
    let mut source = RealSenseSource::with_clock(device, request.clone(), clock(Instant::now()));
    let exposure = source.next().expect("the dark pair");
    assert_eq!(exposure.views.len(), 2);
    assert!(exposure.views.iter().all(|view| view.y[0] == 188));
    assert_eq!(source.next(), Err(CaptureError::Ended));

    // Two imagers of different instants are not a pair.
    let mut torn = pair(4, EPOCH_MS + 40.0, false);
    torn[1].number = 5;
    let mut source = RealSenseSource::with_clock(
        RecordedDevice::new([torn]),
        request.clone(),
        clock(Instant::now()),
    );
    assert!(matches!(source.next(), Err(CaptureError::Device(_))));

    // Interleaving without the emitter metadata cannot tell lit from dark.
    let mut blind = pair(6, EPOCH_MS + 40.0, false);
    for frame in &mut blind {
        frame.emitter_lit = None;
    }
    let mut source =
        RealSenseSource::with_clock(RecordedDevice::new([blind]), request, clock(Instant::now()));
    assert!(matches!(source.next(), Err(CaptureError::Device(_))));
}

#[test]
fn a_frame_not_in_global_time_or_at_another_raster_is_refused() {
    let request =
        CaptureRequest::for_track(&track(&document(PLANS, "humanoid-head-color")), Emitter::On)
            .expect("request");
    for bad in [
        RawFrame {
            domain: TimestampDomain::SystemTime,
            ..frame(Stream::Color, 1, EPOCH_MS + 40.0, (1280, 720))
        },
        frame(Stream::Color, 1, EPOCH_MS + 40.0, (640, 480)),
        frame(Stream::Infrared1, 1, EPOCH_MS + 40.0, (1280, 720)),
        // Further ahead of the host than the regression's uncertainty.
        frame(Stream::Color, 1, EPOCH_MS + 90.0, (1280, 720)),
    ] {
        let mut source = RealSenseSource::with_clock(
            RecordedDevice::new([vec![bad]]),
            request.clone(),
            clock(Instant::now()),
        );
        assert!(matches!(source.next(), Err(CaptureError::Device(_))));
    }
}

#[test]
fn a_track_that_is_not_one_realsense_exposure_is_refused() {
    // The nominal plan places no unit, so no device can be chosen.
    let nominal = track(&document(NOMINAL, "sentient-humanoid-upper-body"));
    assert!(matches!(
        CaptureRequest::for_track(&nominal, Emitter::Off),
        Err(RequestError::Unplaced { .. })
    ));
    let quest = track(&document(NOMINAL, "quest-ego-stereo"));
    assert!(matches!(
        CaptureRequest::for_track(&quest, Emitter::Off),
        Err(RequestError::NotARealSense { .. })
    ));

    let colour = document(PLANS, "humanoid-head-color");
    let edited = |path: &[&str], value: serde_json::Value| {
        let mut plan = colour.clone();
        let mut at = &mut plan["tracks"][0]["views"][0];
        for key in path {
            at = &mut at[*key];
        }
        *at = value;
        track(&plan)
    };
    for (path, value) in [
        (&["unit"][..], serde_json::json!("923322071108/infrared_1")),
        (&["unit"][..], serde_json::json!("923322071108")),
    ] {
        assert!(CaptureRequest::for_track(&edited(path, value), Emitter::Off).is_err());
    }
    assert!(matches!(
        CaptureRequest::for_track(&track(&colour), Emitter::Interleaved),
        Err(RequestError::InterleavedWithoutInfrared { .. })
    ));

    let mut swapped = document(PLANS, "humanoid-head-infrared");
    let views = swapped["tracks"][0]["views"].as_array_mut().expect("views");
    let left = views[0]["unit"].clone();
    views[0]["unit"] = views[1]["unit"].clone();
    views[1]["unit"] = left;
    assert!(matches!(
        CaptureRequest::for_track(&track(&swapped), Emitter::Off),
        Err(RequestError::NotOneExposure { .. })
    ));
}
