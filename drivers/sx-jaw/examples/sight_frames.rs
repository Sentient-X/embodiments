#![allow(unsafe_code)]
//! Sight a recording's finger markers from raw grey frames, as the headset or the Library
//! would, and time it.
//!
//!     cargo run --release -p sx-jaw --example sight_frames -- <frames.bin> <width> <height> \
//!         <side> <stamps.txt> <out.csv> [whole|track] [every]
//!
//! `frames.bin` is the frames' 8-bit luma back to back; `stamps.txt` one exposure (ns) per
//! line. `whole` searches every frame's whole picture (the Library's way); `track` runs the
//! headset's tracker at its cadence (default every frame). Prints the per-frame cost.

use std::io::Write as _;
use std::time::Instant;

/// This thread's CPU time, ms: what the sighting costs, whatever else the machine runs.
fn cpu_ms() -> f64 {
    let mut t = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: a valid out-pointer to a timespec.
    unsafe { libc::clock_gettime(libc::CLOCK_THREAD_CPUTIME_ID, &mut t) };
    t.tv_sec as f64 * 1e3 + t.tv_nsec as f64 / 1e6
}

use sx_jaw::detect::{Detector, Luma, Region};
use sx_jaw::sightings::{Sighting, ended, preamble, rows};
use sx_jaw::tags::Layout;
use sx_jaw::track::{Cadence, Tracker, crop_for};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let frames = std::fs::read(&args[1]).expect("frames");
    let width: usize = args[2].parse().expect("width");
    let height: usize = args[3].parse().expect("height");
    let side = &args[4];
    let stamps: Vec<i64> = std::fs::read_to_string(&args[5])
        .expect("stamps")
        .lines()
        .map(|l| l.trim().parse().expect("stamp"))
        .collect();
    let mode = args.get(7).map_or("whole", String::as_str);
    let every: u32 = args.get(8).map_or(1, |v| v.parse().expect("every"));
    let layout = Layout::yubi(side).expect("side");
    let size = width * height;
    let count = (frames.len() / size).min(stamps.len());
    let mut out = std::io::BufWriter::new(std::fs::File::create(&args[6]).expect("out"));
    out.write_all(preamble(side, layout.ids()).as_bytes())
        .unwrap();
    let mut detector = Detector::new();
    let mut tracker = Tracker::new(
        layout.ids(),
        Cadence {
            every,
            ..Cadence::default()
        },
    );
    let mut times: Vec<f64> = Vec::new();
    let mut sighted = 0;
    let mut analysed = 0;
    let started = Instant::now();
    for i in 0..count {
        let t0 = cpu_ms();
        let luma = Luma::new(&frames[i * size..(i + 1) * size], width, height, width).unwrap();
        let markers = match mode {
            "track" => {
                // As the headset does it: plan, copy each planned region's crop out of the
                // frame, and look in the crops.
                let Some(regions) = tracker.plan(width, height) else {
                    continue;
                };
                let frame = &frames[i * size..(i + 1) * size];
                let mut buffers: Vec<(Vec<u8>, Region)> = Vec::new();
                for region in &regions {
                    let crop = crop_for(*region, width, height);
                    let mut bytes = Vec::with_capacity((crop.x1 - crop.x0) * (crop.y1 - crop.y0));
                    for y in crop.y0..crop.y1 {
                        bytes.extend_from_slice(&frame[y * width + crop.x0..y * width + crop.x1]);
                    }
                    buffers.push((bytes, crop));
                }
                let crops: Vec<(Luma, Region)> = buffers
                    .iter()
                    .zip(&regions)
                    .map(|((bytes, crop), region)| {
                        (
                            Luma::cropped(bytes, crop.x1 - crop.x0, *crop, width, height).unwrap(),
                            *region,
                        )
                    })
                    .collect();
                tracker.look(&crops)
            }
            _ => detector.detect(&luma, Region::whole(width, height), &layout.ids()),
        };
        times.push(cpu_ms() - t0);
        analysed += 1;
        if !markers.is_empty() {
            sighted += 1;
        }
        out.write_all(
            rows(&Sighting {
                t_ns: stamps[i],
                markers,
            })
            .as_bytes(),
        )
        .unwrap();
    }
    out.write_all(ended(count as u64, 0).as_bytes()).unwrap();
    let wall = started.elapsed().as_secs_f64();
    times.sort_by(f64::total_cmp);
    let p = |q: f64| times[((times.len() - 1) as f64 * q) as usize];
    let total: f64 = times.iter().sum();
    eprintln!(
        "{mode} every {every}: {count} frames, {analysed} analysed, {sighted} with a marker; thread CPU per analysed frame mean {:.3} ms p50 {:.3} p95 {:.3} max {:.3}; per recorded frame {:.3} ms; wall {wall:.2} s",
        total / analysed.max(1) as f64,
        p(0.5),
        p(0.95),
        p(1.0),
        total / count as f64,
    );
}
