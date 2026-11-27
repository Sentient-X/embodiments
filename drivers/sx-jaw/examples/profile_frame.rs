//! Repeat the detector on the slowest frames of a recording (for a sampling profiler).
//!     profile_frame <frames.bin> <width> <height> <repeat>
use std::time::Instant;
use sx_jaw::detect::{Detector, Luma, Region};
fn main() {
    let args: Vec<String> = std::env::args().collect();
    let frames = std::fs::read(&args[1]).unwrap();
    let (w, h): (usize, usize) = (args[2].parse().unwrap(), args[3].parse().unwrap());
    let repeat: usize = args[4].parse().unwrap();
    let n = frames.len() / (w * h);
    let mut d = Detector::new();
    let mut times: Vec<(f64, usize)> = (0..n)
        .map(|i| {
            let luma = Luma::new(&frames[i * w * h..(i + 1) * w * h], w, h, w).unwrap();
            let t = Instant::now();
            d.detect(&luma, Region::whole(w, h), &[0, 1]);
            (t.elapsed().as_secs_f64() * 1e3, i)
        })
        .collect();
    times.sort_by(|a, b| b.0.total_cmp(&a.0));
    eprintln!("slowest {:?}", &times[..5]);
    let i = times[0].1;
    let luma = Luma::new(&frames[i * w * h..(i + 1) * w * h], w, h, w).unwrap();
    for _ in 0..repeat {
        d.detect(&luma, Region::whole(w, h), &[0, 1]);
    }
}
