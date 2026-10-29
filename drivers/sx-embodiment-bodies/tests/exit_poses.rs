//! Commissioned targets are explicit data, never an inferred zero or captured start.
use sx_embodiment_bodies::Commissioning;

fn parse(poses: &str) -> Result<Commissioning, sx_embodiment_bodies::OpenError> {
    Commissioning::parse(
        &format!(
            r#"{{"schema":"sx.station-commissioning/v1","instance":"unit","pod_id":"pod","pod_revision":1,"ports":{{"arm":"serial"}},"exit_poses":{poses}}}"#
        ),
        "unit",
    )
}

#[test]
fn missing_poses_are_absent_and_duplicate_or_nonfinite_targets_refuse() {
    let empty = parse("{}").unwrap();
    assert!(empty.exit_poses.ready.is_none());
    assert!(empty.exit_poses.parked.is_none());
    for invalid in [
        r#"{"ready":{"joints":{"joint":0,"joint":1}}}"#,
        r#"{"ready":{"joints":{"joint":1e999}}}"#,
        r#"{"ready":{"joints":{"":0}}}"#,
        r#"{"ready":{"joints":{},"unknown":true}}"#,
        r#"{"parked":{"configuration_sha256":"x","verification_reference":"operator"}}"#,
    ] {
        assert!(parse(invalid).is_err(), "{invalid}");
    }
}

#[test]
fn physical_attestation_is_preserved_separately_from_ready() {
    let parsed = parse(r#"{"ready":{"joints":{"joint":0.1}},"parked":{"pose":{"joints":{"joint":-0.2}},"configuration_sha256":"exact-motion-basis","verification_reference":"physical-check/42"}}"#).unwrap();
    assert_eq!(parsed.exit_poses.ready.unwrap().joints["joint"], 0.1);
    let parked = parsed.exit_poses.parked.unwrap();
    assert_eq!(parked.pose.joints["joint"], -0.2);
    assert_eq!(parked.verification_reference, "physical-check/42");
}
