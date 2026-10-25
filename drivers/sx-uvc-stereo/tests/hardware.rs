//! On an attached ZED: `cargo test -p sx-uvc-stereo --features capture --test hardware --
//! --ignored --nocapture`, with `SX_UVC_STEREO_SERIAL=<serial>` and `SX_UVC_STEREO_MODE=<eye
//! width>x<eye height>@<fps>` (default `672x376@15`, the one mode a USB 2 link carries). Opens
//! the device, checks ten exposures arrive in order as two eyes of the mode, and with
//! `SX_UVC_STEREO_SAVE=<dir>` writes the last pair as `zed_left.ppm` and `zed_right.ppm`.

#![cfg(feature = "capture")]

use std::fmt::Write as _;
use std::path::Path;
use std::time::{Duration, Instant};

use sx_embodiment_drivers::{FrameSource, I420Image};
use sx_uvc_stereo::{CaptureRequest, PRODUCTS, capture};

/// BT.601 limited range back to RGB, for looking at.
fn ppm(image: &I420Image) -> Vec<u8> {
    let mut header = String::new();
    let _ = write!(header, "P6\n{} {}\n255\n", image.width, image.height);
    let mut out = header.into_bytes();
    let (w, h) = (image.width as usize, image.height as usize);
    let half = w.div_ceil(2);
    let clamp = |v: i32| u8::try_from((v >> 8).clamp(0, 255)).unwrap_or(u8::MAX);
    for y in 0..h {
        for x in 0..w {
            let luma = 298 * (i32::from(image.y[y * w + x]) - 16) + 128;
            let blue = i32::from(image.u[(y / 2) * half + x / 2]) - 128;
            let red = i32::from(image.v[(y / 2) * half + x / 2]) - 128;
            out.extend([
                clamp(luma + 409 * red),
                clamp(luma - 100 * blue - 208 * red),
                clamp(luma + 516 * blue),
            ]);
        }
    }
    out
}

#[test]
#[ignore = "needs an attached ZED; on macOS the running app needs camera access"]
fn an_attached_zed_streams_both_eyes_of_one_mode() {
    eprintln!("attached: {:?}", capture::devices());
    let serial = std::env::var("SX_UVC_STEREO_SERIAL").unwrap_or_default();
    let mode = std::env::var("SX_UVC_STEREO_MODE").unwrap_or_else(|_| "672x376@15".to_owned());
    let (raster, fps) = mode.split_once('@').expect("<w>x<h>@<fps>");
    let (width, height) = raster.split_once('x').expect("<w>x<h>");
    let request = CaptureRequest {
        track: "probe".to_owned(),
        product: &PRODUCTS[0],
        serial,
        eye_width: width.parse().expect("width"),
        eye_height: height.parse().expect("height"),
        fps: fps.parse().expect("fps"),
        cameras: ["left".to_owned(), "right".to_owned()],
    };
    let (eye_width, eye_height) = (request.eye_width, request.eye_height);
    let mut source = capture::open(request).expect("open");
    let mut last = None;
    let mut pair = None;
    for _ in 0..10 {
        let exposure = source.next().expect("exposure");
        let now = Instant::now();
        assert!(exposure.captured.monotonic <= now);
        assert!(now - exposure.captured.monotonic < Duration::from_secs(1));
        assert!(last.is_none_or(|previous| exposure.captured.monotonic > previous));
        last = Some(exposure.captured.monotonic);
        assert!(exposure.views.iter().all(|view| {
            (view.width, view.height) == (eye_width, eye_height) && view.is_well_formed()
        }));
        pair = Some(exposure.views);
    }
    if let (Ok(dir), Some(views)) = (std::env::var("SX_UVC_STEREO_SAVE"), pair) {
        for (name, view) in ["zed_left.ppm", "zed_right.ppm"].iter().zip(&views) {
            std::fs::write(Path::new(&dir).join(name), ppm(view)).expect("write");
        }
    }
}
