//! A robot directory's commissioned unit calibrations: `<port>.calibration.json` beside its
//! `commissioning.json`, read for every port no explicit calibration covers.

use sx_embodiment_bodies::{Commissioning, ServoCalibrations, calibration_file};

const INSTANCE: &str = "8d4a416a181d9cecc6494f27466cb2c092292fcb0fa4cf850d087065d16a2ef1";

fn commissioning() -> Commissioning {
    let document = serde_json::json!({
        "instance": INSTANCE,
        "out_of_service": [],
        "pod_id": "bench",
        "pod_revision": 1,
        "ports": {"left_gripper": "5AB0158173", "neck": "5A7C118709",
                  "right_gripper": "5B91028463"},
        "rails": {"*": 12.0},
        "schema": "sx.station-commissioning/v1",
    });
    Commissioning::parse(&document.to_string(), INSTANCE).expect("commissioning")
}

fn lerobot(id: u16, offset: i16) -> String {
    serde_json::json!({
        "joint": {"id": id, "drive_mode": 0, "homing_offset": offset, "range_min": 1000,
                  "range_max": 3000}
    })
    .to_string()
}

#[test]
fn every_commissioned_port_without_an_explicit_calibration_takes_the_directorys() {
    let directory = std::env::temp_dir().join(format!("sx-robot-directory-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&directory);
    std::fs::create_dir_all(&directory).expect("directory");
    std::fs::write(directory.join(calibration_file("neck")), lerobot(6, -42)).expect("neck");
    std::fs::write(
        directory.join(calibration_file("left_gripper")),
        lerobot(3, 7),
    )
    .expect("left");
    // The left gripper is given explicitly: the explicit one wins.
    let explicit = ServoCalibrations::new()
        .with_lerobot("left_gripper", &lerobot(3, 99))
        .expect("explicit");
    let (calibrations, read) = explicit
        .with_robot_directory(&directory, &commissioning())
        .expect("read");
    assert_eq!(read, ["neck"]);
    assert_eq!(
        calibrations.of("neck", 6).map(|c| c.homing_offset),
        Some(-42)
    );
    assert_eq!(
        calibrations.of("left_gripper", 3).map(|c| c.homing_offset),
        Some(99)
    );
    // No file for the right gripper: nothing, as before.
    assert_eq!(calibrations.of("right_gripper", 1), None);
    // A file that is there but broken refuses rather than opening uncalibrated.
    std::fs::write(directory.join(calibration_file("right_gripper")), "{").expect("broken");
    assert!(
        ServoCalibrations::new()
            .with_robot_directory(&directory, &commissioning())
            .is_err()
    );
}
