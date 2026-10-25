//! The attached ZED 2i's sensor module, read-only. Ignored unless asked for:
//!
//! ```text
//! cargo test -p sx-stereolabs-sensors --features hid --test hardware -- --ignored --nocapture
//! ```
#![cfg(feature = "hid")]

use sx_embodiment_drivers::{InertialSource, MediaPlan, Stillness};

const PLANS: &str = include_str!("../../sx-uvc-stereo/tests/fixtures/media-plans.json");

#[test]
#[ignore = "needs the static upper body's ZED 2i (serial 30371653) attached"]
fn the_attached_zed_streams_gravity_at_400_hz() {
    let value: serde_json::Value = serde_json::from_str(PLANS).expect("fixtures");
    let head = &value["static-upper-body-head"];
    let plan =
        MediaPlan::parse(&head.to_string(), head["instance"].as_str().expect("id")).expect("plan");
    println!(
        "attached: {:?}",
        sx_stereolabs_sensors::hid::attached().expect("hid")
    );
    let mut source = sx_stereolabs_sensors::hid::open(&plan.inertial[0]).expect("open");
    let samples: Vec<_> = (0..800).map(|_| source.next().expect("sample")).collect();
    let still = Stillness::of(&samples).expect("summary");
    println!("{still:?}");
    let force = still.mean_acceleration_mps2;
    let magnitude = force.iter().map(|v| v * v).sum::<f64>().sqrt();
    assert!((magnitude - 9.8).abs() < 0.5, "|f| {magnitude}");
    assert!((still.rate_hz - 400.0).abs() < 20.0, "{} Hz", still.rate_hz);
}
