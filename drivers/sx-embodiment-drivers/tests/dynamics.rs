//! `Dynamics` against `MuJoCo`'s `mj_inverse` on the static upper body, design and calibrated
//! (`fixtures/dynamics-parity.json`, minted by `tools/mint_dynamics_parity.py`), and a tick
//! that allocates nothing.

use serde_json::Value;
use sx_embodiment_drivers::{Dynamics, DynamicsCalibration, Tree};

const URDF: &str =
    include_str!("../../../assets/sentient_static_upper_body/sentient_static_upper_body.urdf");

fn fixture() -> Value {
    serde_json::from_str(include_str!("fixtures/dynamics-parity.json")).expect("fixture")
}

fn vector(value: &Value) -> Vec<f64> {
    value
        .as_array()
        .expect("array")
        .iter()
        .map(|v| v.as_f64().expect("number"))
        .collect()
}

fn gravity(fixture: &Value) -> [f64; 3] {
    let g = vector(&fixture["gravity"]);
    [g[0], g[1], g[2]]
}

#[test]
fn inverse_dynamics_is_mujocos_within_a_micro_newton_metre() {
    let fixture = fixture();
    let tree = Tree::parse(URDF).expect("tree");
    let axes: Vec<&str> = tree.joints().iter().map(|a| a.name.as_str()).collect();
    let expected_axes: Vec<String> = fixture["axes"]
        .as_array()
        .expect("axes")
        .iter()
        .map(|a| a.as_str().expect("name").to_owned())
        .collect();
    assert_eq!(axes, expected_axes);
    let design = Dynamics::new(tree.clone(), gravity(&fixture));
    let record = fixture["calibration"].to_string();
    let calibration = DynamicsCalibration::parse(&record).expect("calibration");
    let digest = calibration.description_sha256.clone();
    let calibrated = Dynamics::new(tree, gravity(&fixture))
        .calibrated(&calibration, &digest)
        .expect("calibrated");
    let mut worst: f64 = 0.0;
    for sample in fixture["samples"].as_array().expect("samples") {
        let (q, qd, qdd) = (
            vector(&sample["position"]),
            vector(&sample["velocity"]),
            vector(&sample["acceleration"]),
        );
        for (dynamics, key) in [(&design, "design"), (&calibrated, "calibrated")] {
            let torque = dynamics.inverse(&q, &qd, &qdd).expect("torque");
            for (got, want) in torque.iter().zip(vector(&sample[key])) {
                worst = worst.max((got - want).abs());
            }
        }
    }
    assert!(worst < 1e-6, "worst disagreement {worst} N·m");
}

#[test]
fn a_tick_through_scratch_allocates_nothing() {
    let fixture = fixture();
    let dynamics = Dynamics::new(Tree::parse(URDF).expect("tree"), gravity(&fixture));
    let sample = &fixture["samples"][0];
    let (q, qd, qdd) = (
        vector(&sample["position"]),
        vector(&sample["velocity"]),
        vector(&sample["acceleration"]),
    );
    let mut scratch = dynamics.scratch();
    let mut torque = vec![0.0; q.len()];
    let mut held = vec![0.0; q.len()];
    let counted = allocation_counter::measure(|| {
        for _ in 0..100 {
            dynamics
                .inverse_into(&q, &qd, &qdd, &mut scratch, &mut torque)
                .expect("torque");
            dynamics
                .gravity_torque_into(&q, &mut scratch, &mut held)
                .expect("gravity");
        }
    });
    assert_eq!(counted.count_total, 0);
    assert!(torque.iter().all(|t| t.is_finite()));
}

#[test]
fn a_calibration_for_another_description_is_refused() {
    let fixture = fixture();
    let calibration =
        DynamicsCalibration::parse(&fixture["calibration"].to_string()).expect("calibration");
    let dynamics = Dynamics::new(Tree::parse(URDF).expect("tree"), gravity(&fixture));
    assert!(
        dynamics
            .calibrated(&calibration, "0".repeat(64).as_str())
            .is_err()
    );
    let mut document = fixture["calibration"].clone();
    document["links"][0]["mass"] = Value::from(-1.0);
    assert!(DynamicsCalibration::parse(&document.to_string()).is_err());
    let mut document = fixture["calibration"].clone();
    document["extra"] = Value::from(1);
    assert!(DynamicsCalibration::parse(&document.to_string()).is_err());
}

/// Median and worst time of one steady-state call on the static upper body (25 axes, 36 links),
/// printed with `--nocapture`; bounded loosely so a debug build passes.
#[test]
fn a_tick_is_bounded_in_time() {
    let fixture = fixture();
    let dynamics = Dynamics::new(Tree::parse(URDF).expect("tree"), gravity(&fixture));
    let sample = &fixture["samples"][1];
    let (q, qd, qdd) = (
        vector(&sample["position"]),
        vector(&sample["velocity"]),
        vector(&sample["acceleration"]),
    );
    let mut scratch = dynamics.scratch();
    let mut torque = vec![0.0; q.len()];
    let mut times: Vec<std::time::Duration> = Vec::with_capacity(2000);
    for _ in 0..2000 {
        let start = std::time::Instant::now();
        dynamics
            .inverse_into(&q, &qd, &qdd, &mut scratch, &mut torque)
            .expect("torque");
        times.push(start.elapsed());
    }
    times.sort();
    let (median, worst) = (times[times.len() / 2], times[times.len() - 1]);
    eprintln!("inverse_into over 2000 calls: median {median:?}, worst {worst:?}");
    // The work is fixed by the tree; the bound is loose so a debug build on a busy host passes.
    assert!(median < std::time::Duration::from_millis(5));
}
