//! The real handoff's Python gravity controller is the oracle, at identical raw encoders.
//! Regenerate both pinned fixtures with `tools/import_allisa_guidance.py` (sockets refused).

use serde::Deserialize;
use sx_embodiment_drivers::guidance::GuidanceProfile;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Oracle {
    joints: Vec<String>,
    tolerance_nm: f64,
    samples: Vec<Sample>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Sample {
    position: Vec<f64>,
    motor_position: Vec<f64>,
    torque: Vec<f64>,
}

#[test]
fn translated_guidance_matches_handoff_at_identical_motor_positions() {
    let text = include_str!("fixtures/allisa-guidance.json");
    let identity: serde_json::Value = serde_json::from_str(text).unwrap();
    let profile = GuidanceProfile::parse(
        text,
        "1954e1793fc5cafc01c4f98ece2d0ec8bcfc823ed7ea93ce36949d39af4d8a0c",
        identity["drive_sha256"].as_str().unwrap(),
    )
    .unwrap();
    let oracle: Oracle =
        serde_json::from_str(include_str!("fixtures/allisa-guidance-oracle.json")).unwrap();
    assert_eq!(oracle.samples.len(), 256);
    assert_eq!(oracle.joints.len(), 14);
    let order: Vec<_> = profile
        .tree()
        .joints()
        .iter()
        .map(|joint| {
            oracle
                .joints
                .iter()
                .position(|name| *name == joint.name)
                .unwrap()
        })
        .collect();
    let mut scratch = profile.scratch();
    let mut position = vec![0.0; order.len()];
    let mut torque = vec![0.0; order.len()];
    let mut maximum_error = 0.0_f64;
    for (row, sample) in oracle.samples.iter().enumerate() {
        assert_eq!(sample.motor_position.len(), 14);
        for (i, &source) in order.iter().enumerate() {
            position[i] = sample.position[source];
        }
        profile
            .gravity_into(&position, &mut scratch, &mut torque)
            .unwrap();
        for (i, &source) in order.iter().enumerate() {
            let error = (torque[i] - sample.torque[source]).abs();
            maximum_error = maximum_error.max(error);
            assert!(
                error < oracle.tolerance_nm,
                "pose {row}, {}: target {} vs reference {}, error {error}",
                oracle.joints[source],
                torque[i],
                sample.torque[source]
            );
        }
    }
    eprintln!("256 poses / 3584 joint comparisons; maximum error {maximum_error:.12} Nm");
}
