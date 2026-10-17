//! On an attached RealSense: `SX_REALSENSE_SERIAL=<serial> cargo test -p sx-realsense
//! --features sdk --test hardware -- --ignored`. Opens the device's colour stream at 640x480
//! and 30 fps, reads its factory intrinsics, and checks ten exposures arrive in order, each
//! stamped no later than the host and within a second of it.

#![cfg(feature = "sdk")]

use std::time::{Duration, Instant};

use sx_embodiment_drivers::FrameSource;
use sx_realsense::{CaptureRequest, Emitter, Stream, ViewStream, sdk};

#[test]
#[ignore = "needs an attached RealSense named by SX_REALSENSE_SERIAL"]
fn an_attached_device_streams_its_colour_imager_in_global_time() {
    let attached = sdk::devices().expect("librealsense2 context");
    eprintln!("attached: {attached:?}");
    let serial = std::env::var("SX_REALSENSE_SERIAL").expect("SX_REALSENSE_SERIAL");
    let request = CaptureRequest {
        track: "probe".to_owned(),
        serial,
        fps: 30,
        device_fps: 30,
        views: vec![ViewStream {
            camera: "probe".to_owned(),
            stream: Stream::Color,
            width: 640,
            height: 480,
        }],
        emitter: Emitter::On,
    };
    let (mut source, factory) = sdk::open(request).expect("open");
    eprintln!("{}", factory[0].document());
    let mut last = None;
    for _ in 0..10 {
        let exposure = source.next().expect("exposure");
        let now = Instant::now();
        assert!(exposure.captured.monotonic <= now);
        assert!(now - exposure.captured.monotonic < Duration::from_secs(1));
        if let Some(previous) = last {
            assert!(exposure.captured.monotonic > previous);
        }
        last = Some(exposure.captured.monotonic);
    }
}
