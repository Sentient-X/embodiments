//! Open a ZED 2i the way the station does, read frames, then stop its stream and close it:
//! the bench check for a host's USB controller (the DGX Spark's NVDA8000 hung on a stream stop
//! and on a restart, 10-27). Read-only for the robot: camera only.
//!
//! `cargo run --release --features capture --example zed_read -- <serial> <eye width> <eye
//! height> <fps> <frames>`; prints when the first frame came, the rate, and how long the stop
//! took. Run it again to test a restart after a stop.

use std::time::{Duration, Instant};

use sx_embodiment_drivers::FrameSource;
use sx_uvc_stereo::{CaptureRequest, Eye, PRODUCTS, capture};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [serial, width, height, fps, frames] = args.as_slice() else {
        eprintln!("usage: zed_read <serial> <eye width> <eye height> <fps> <frames>");
        std::process::exit(2);
    };
    let number = |text: &str| text.parse::<u32>().expect("a whole number");
    let request = CaptureRequest {
        track: "head".to_owned(),
        product: &PRODUCTS[0],
        serial: serial.clone(),
        eye_width: number(width),
        eye_height: number(height),
        fps: number(fps),
        cameras: vec!["head_left".to_owned(), "head_right".to_owned()],
        eyes: vec![Eye::Left, Eye::Right],
    };
    let opened = Instant::now();
    let mut sources = match capture::open(vec![request]) {
        Ok(sources) => sources,
        Err(error) => {
            println!("open refused ({:?}): {error}", error.unavailable());
            std::process::exit(1);
        }
    };
    let source = &mut sources[0];
    let wanted = number(frames);
    let mut first = None;
    let mut read = 0;
    while read < wanted {
        match source.next() {
            Ok(_) => {
                read += 1;
                first.get_or_insert_with(|| opened.elapsed());
            }
            Err(error) => {
                println!("capture ended after {read} frames: {error}");
                break;
            }
        }
    }
    let streamed = opened.elapsed();
    let first = first.unwrap_or(Duration::ZERO);
    let rate =
        f64::from(read.saturating_sub(1)) / streamed.saturating_sub(first).as_secs_f64().max(1e-9);
    println!("{read} frames; first after {first:?}; {rate:.1} fps");
    drop(sources);
    let stopping = Instant::now();
    capture::close_all();
    println!(
        "stream stopped and device closed in {:?}",
        stopping.elapsed()
    );
}
