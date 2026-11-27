//! The fusion's parity with the retired Python solver on a real take: the same marker poses
//! (as `sx_embodiments.jaw_tags.sight_tags` located them) and encoder reads, through this
//! crate's reference and smoother; or, with `sightings`, this crate's own detector end to end.
//!
//!     cargo run --release -p sx-jaw --example parity -- <poses.json> <out.csv> [<sightings.csv>]

use std::time::Instant;

use serde_json::Value;
use sx_jaw::fusion::{EncoderReads, FusionModel, solve_jaw};
use sx_jaw::lens::Lens;
use sx_jaw::solve::{coverage, frames_at};
use sx_jaw::tags::{Frame, Layout, TagPose, closed_stop_reference};

fn mat(v: &Value) -> [[f64; 3]; 3] {
    let f: Vec<f64> = v
        .as_array()
        .unwrap()
        .iter()
        .map(|x| x.as_f64().unwrap())
        .collect();
    [[f[0], f[1], f[2]], [f[3], f[4], f[5]], [f[6], f[7], f[8]]]
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let doc: Value = serde_json::from_str(&std::fs::read_to_string(&args[1]).unwrap()).unwrap();
    let t: Vec<i64> = doc["t"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_i64().unwrap())
        .collect();
    let layout = Layout::yubi("right").unwrap();
    let frames: Vec<Frame> = if let Some(path) = args.get(3) {
        let lens_doc = &doc["lens"];
        let k: Vec<f64> = lens_doc["k"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();
        let d: Vec<f64> = lens_doc["d"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_f64().unwrap())
            .collect();
        let lens = Lens::new(
            640,
            480,
            k.try_into().unwrap(),
            lens_doc["distortion_model"].as_str().unwrap(),
            &d,
        )
        .unwrap();
        let (sightings, _) =
            sx_jaw::sightings::parse(&std::fs::read_to_string(path).unwrap(), "right").unwrap();
        let (analysed, sighted) = coverage(&t, &sightings, &layout);
        eprintln!(
            "sightings: {analysed} of {} frames analysed, {sighted} with a marker",
            t.len()
        );
        frames_at(&t, &sightings, &lens, &layout)
    } else {
        doc["poses"]
            .as_array()
            .unwrap()
            .iter()
            .map(|frame| {
                frame
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|p| TagPose {
                        marker_id: p["id"].as_u64().unwrap() as u32,
                        rotation: mat(&p["r"]),
                        center_m: {
                            let c: Vec<f64> = p["c"]
                                .as_array()
                                .unwrap()
                                .iter()
                                .map(|v| v.as_f64().unwrap())
                                .collect();
                            [c[0], c[1], c[2]]
                        },
                        reprojection_px: p["e"].as_f64().unwrap(),
                        alternate: if p["a"].is_null() {
                            None
                        } else {
                            Some(mat(&p["a"]))
                        },
                    })
                    .collect()
            })
            .collect()
    };
    let enc_t: Vec<i64> = doc["enc_t"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_i64().unwrap())
        .collect();
    let enc_a: Vec<f64> = doc["enc_a"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap())
        .collect();
    let encoder = EncoderReads::new(enc_t, enc_a).unwrap();
    let started = Instant::now();
    let reference = closed_stop_reference(&layout, &frames, 0.0).expect("closed-stop reference");
    let (fused, tags) = solve_jaw(
        &reference,
        &t,
        &frames,
        Some(&encoder),
        &FusionModel::default(),
    )
    .unwrap();
    eprintln!(
        "rust solve {:.3} s, lag {} ns",
        started.elapsed().as_secs_f64(),
        fused.camera_lag_ns
    );
    let mut out = String::from("t,angle,std,sensors,tags,bias\n");
    for i in 0..t.len() {
        out.push_str(&format!(
            "{},{:.12},{:.12},{},{:.12},{:.12}\n",
            t[i],
            fused.angle_rad[i],
            fused.std_rad[i],
            fused.sensors[i].as_str(),
            tags.angle_rad[i],
            fused.encoder_bias_rad[i]
        ));
    }
    std::fs::write(&args[2], out).unwrap();
}
