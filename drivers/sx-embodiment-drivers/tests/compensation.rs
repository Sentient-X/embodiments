//! Independent pendulum mechanics, strict identity, transition and real-time contracts.

use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use sx_embodiment_drivers::compensation::{CompensationError, QuasiStaticCompensation};
use sx_embodiment_drivers::{
    ActuatorBinding, ActuatorBus, ActuatorModel, AxisMotion, BoundAxis, Dynamics,
    DynamicsCalibration, JointFriction, Rest, RestModel, UPRIGHT,
};

const DRIVE: &[u8] = b"exact commissioned drive bytes";
// A 2 kg point mass 0.1 m along X rotates about +Y. Its holding torque is
// -2 * 9.80665 * 0.1 * cos(q), independently of the implementation under test.
const PENDULUM: &str = r#"<robot name="pendulum">
  <link name="base"/>
  <link name="load"><inertial><origin xyz="0.1 0 0"/>
    <mass value="2"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
  </inertial></link>
  <joint name="joint" type="continuous"><parent link="base"/><child link="load"/>
    <axis xyz="0 1 0"/></joint>
  <link name="other_load"/>
  <joint name="other" type="continuous"><parent link="base"/><child link="other_load"/>
    <axis xyz="0 0 1"/></joint>
</robot>"#;

fn source() -> String {
    // The canonical body has an unselected second branch; the source has one independent axis.
    PENDULUM.replace(
        "  <link name=\"other_load\"/>\n  <joint name=\"other\" type=\"continuous\"><parent link=\"base\"/><child link=\"other_load\"/>\n    <axis xyz=\"0 0 1\"/></joint>\n",
        "",
    )
}

fn document() -> Value {
    json!({
        "schema": "sx.quasi-static-compensation/v1",
        "description_sha256": format!("{:x}", Sha256::digest(PENDULUM.as_bytes())),
        "drive_sha256": format!("{:x}", Sha256::digest(DRIVE)),
        "units": ["test-bus"],
        "source_urdf": source(),
        "gravity_scale": 0.95,
        "torque_slew_nm_s": 20.0,
        "joints": [{
            "joint": "joint", "source_joint": "joint", "scale": 1.0, "offset": 0.0,
            "coulomb": 0.22, "velocity_deadband": 0.02, "transition_velocity": 0.03,
            "max_torque": 10.0
        }]
    })
}

fn parse(value: &Value) -> Result<QuasiStaticCompensation, CompensationError> {
    QuasiStaticCompensation::parse(&value.to_string(), PENDULUM, DRIVE, &["other", "joint"])
}

fn gravity_at(q: f64) -> f64 {
    -2.0 * 9.80665 * 0.1 * q.cos() * 0.95
}

fn close(got: f64, expected: f64) {
    assert!(
        (got - expected).abs() < 1e-12,
        "got {got}, expected {expected}"
    );
}

fn rest_model() -> (RestModel, Vec<BoundAxis>) {
    let axes = ["other", "joint"]
        .iter()
        .zip([1, 2])
        .map(|(joint, bus_id)| BoundAxis {
            joint: (*joint).to_owned(),
            port: "can".to_owned(),
            lower: -3.0,
            upper: 3.0,
            binding: ActuatorBinding {
                model: ActuatorModel::new("damiao_dm4340"),
                bus: ActuatorBus::new("damiao_can"),
                bus_id,
                sign: 1,
                reduction: 1.0,
            },
            zero: 0.0,
            direction: 1,
            home: Some(0.0),
            gripper: false,
            motion: AxisMotion {
                speed: 1.0,
                acceleration: 10.0,
                inertia: 1.0,
            },
        })
        .collect::<Vec<_>>();
    // Legacy friction would classify the joint as self-supporting. The selected imported
    // profile must not use that unrelated old fit to release a compensated arm.
    let calibration = DynamicsCalibration {
        description_sha256: format!("{:x}", Sha256::digest(PENDULUM.as_bytes())),
        stiction_velocity: 0.01,
        links: Vec::new(),
        joints: vec![(
            "joint".to_owned(),
            JointFriction {
                coulomb: 100.0,
                ..JointFriction::default()
            },
        )],
    };
    let dynamics = Dynamics::of(PENDULUM, UPRIGHT)
        .unwrap()
        .calibrated(&calibration)
        .unwrap();
    (
        RestModel::new(dynamics, &axes).with_compensation(Some(parse(&document()).unwrap())),
        axes,
    )
}

#[test]
fn compensated_stops_hold_selected_axes_with_imported_gravity_or_no_guessed_geometry() {
    let (mut model, axes) = rest_model();
    let complete = model.rests(&axes, &[Some(0.0), Some(0.0)]);
    let Rest::Hold { gravity } = complete[1] else {
        panic!("legacy friction released a compensated arm")
    };
    close(gravity, gravity_at(0.0));
    let missing = model.rests(&axes, &[Some(0.0), None]);
    let Rest::Hold { gravity } = missing[1] else {
        panic!("missing selected geometry released a compensated arm")
    };
    close(gravity, 0.0);
    let invalid = model.rests(&axes, &[Some(0.0), Some(f64::NAN)]);
    assert!(invalid[1].holds());
}

#[test]
fn compensated_stop_path_allocates_nothing() {
    let (mut model, axes) = rest_model();
    // Actuator ratings are initialized lazily once; measure the steady stop path.
    assert!(model.rests(&axes, &[Some(0.0), Some(0.1)])[1].holds());
    let measured = allocation_counter::measure(|| {
        for _ in 0..100 {
            let rests = model.rests(&axes, &[Some(0.0), Some(0.1)]);
            assert!(rests[1].holds());
        }
    });
    assert_eq!(measured.count_total, 0);
}

#[test]
fn analytical_pendulum_and_partial_canonical_chain() {
    let mut compensation = parse(&document()).unwrap();
    assert_eq!(compensation.units(), &["test-bus"]);
    assert_eq!(compensation.affected_indices().collect::<Vec<_>>(), vec![1]);
    for q in [0.0, 0.2, -0.7, std::f64::consts::FRAC_PI_2] {
        compensation.reset();
        let mut effort = [123.0, 0.0];
        let mut gravity = [456.0, 0.0];
        compensation
            .step(&[0.0, q], &[0.0; 2], 0.01, &mut effort, &mut gravity)
            .unwrap();
        close(effort[1], gravity_at(q));
        close(gravity[1], gravity_at(q));
        assert_eq!(effort[0].to_bits(), 123.0_f64.to_bits());
        assert_eq!(gravity[0].to_bits(), 456.0_f64.to_bits());
    }
}

#[test]
fn affine_source_coordinates_preserve_virtual_work() {
    let mut value = document();
    value["joints"][0]["scale"] = json!(-2.0);
    value["joints"][0]["offset"] = json!(0.4);
    let mut compensation = parse(&value).unwrap();
    let mut gravity = [0.0; 2];
    compensation
        .gravity_into(&[0.0, 0.1], &mut gravity)
        .unwrap();
    close(gravity[1], -2.0 * gravity_at(-2.0 * 0.1 + 0.4));
}

#[test]
fn quantized_standstill_has_no_friction_bias_and_reversal_changes_assist_sign() {
    let mut compensation = parse(&document()).unwrap();
    for speed in [-0.014_652, -0.004_884, 0.0, 0.004_884, 0.014_652] {
        compensation.reset();
        let mut effort = [0.0; 2];
        let mut gravity = [0.0; 2];
        compensation
            .step(&[0.0; 2], &[0.0, speed], 0.02, &mut effort, &mut gravity)
            .unwrap();
        close(effort[1], gravity_at(0.0));
    }
    for speed in [-0.2_f64, 0.2] {
        compensation.reset();
        let mut effort = [0.0; 2];
        let mut gravity = [0.0; 2];
        compensation
            .step(&[0.0; 2], &[0.0, speed], 0.02, &mut effort, &mut gravity)
            .unwrap();
        close(
            effort[1],
            gravity_at(0.0) + 0.22 * ((speed.abs() - 0.02) / 0.03).tanh() * speed.signum(),
        );
        assert!((effort[1] - gravity_at(0.0)) * speed > 0.0);
    }
}

#[test]
fn zero_coulomb_axis_never_gets_velocity_positive_assistance() {
    let mut value = document();
    value["joints"][0]["coulomb"] = json!(0.0);
    let mut compensation = parse(&value).unwrap();
    for speed in [-2.0, -0.01, 0.0, 0.01, 2.0] {
        let mut effort = [0.0; 2];
        let mut gravity = [0.0; 2];
        compensation
            .step(&[0.0; 2], &[0.0, speed], 0.02, &mut effort, &mut gravity)
            .unwrap();
        close(effort[1], gravity_at(0.0));
    }
}

#[test]
fn torque_slew_uses_elapsed_time_at_both_control_rates() {
    for dt in [0.01, 0.02] {
        let mut value = document();
        value["joints"][0]["coulomb"] = json!(5.0);
        let mut compensation = parse(&value).unwrap();
        let mut effort = [0.0; 2];
        let mut gravity = [0.0; 2];
        for tick in 1..=5 {
            compensation
                .step(&[0.0; 2], &[0.0, 2.0], dt, &mut effort, &mut gravity)
                .unwrap();
            close(effort[1], gravity_at(0.0) + f64::from(tick) * 20.0 * dt);
        }
        // Once assistance outweighs gravity, none of the opposite applied effort is gravity.
        if effort[1] > 0.0 {
            close(gravity[1], 0.0);
        }
    }
}

#[test]
fn stop_gravity_ignores_friction_and_does_not_change_slew_history() {
    let mut compensation = parse(&document()).unwrap();
    let mut effort = [0.0; 2];
    let mut gravity = [0.0; 2];
    compensation
        .step(&[0.0; 2], &[0.0; 2], 0.01, &mut effort, &mut gravity)
        .unwrap();
    let mut stop = [77.0; 2];
    compensation.gravity_into(&[0.0, 0.8], &mut stop).unwrap();
    close(stop[1], gravity_at(0.8));
    assert_eq!(stop[0].to_bits(), 77.0_f64.to_bits());
    compensation
        .step(&[0.0, 0.8], &[0.0; 2], 0.01, &mut effort, &mut gravity)
        .unwrap();
    close(effort[1], gravity_at(0.0) + 0.2);
}

#[test]
fn combined_torque_is_clamped_before_slew() {
    let mut value = document();
    value["joints"][0]["max_torque"] = json!(0.75);
    value["joints"][0]["coulomb"] = json!(4.0);
    let mut compensation = parse(&value).unwrap();
    let mut effort = [0.0; 2];
    let mut gravity = [0.0; 2];
    compensation
        .step(&[0.0; 2], &[0.0; 2], 0.02, &mut effort, &mut gravity)
        .unwrap();
    close(effort[1], -0.75);
    compensation
        .step(&[0.0; 2], &[0.0, 1.0], 0.02, &mut effort, &mut gravity)
        .unwrap();
    close(effort[1], -0.35);
    close(gravity[1], -0.35);
}

#[test]
fn identity_unknown_fields_and_bad_mappings_fail_closed() {
    let original = document();
    for (field, bad) in [
        ("schema", json!("other")),
        ("description_sha256", json!("0".repeat(64))),
        ("drive_sha256", json!("0".repeat(64))),
        ("gravity_scale", json!(0.0)),
        ("gravity_scale", json!(1.1)),
        ("torque_slew_nm_s", json!(-1.0)),
        ("extra", json!(true)),
        ("units", json!([])),
        ("units", json!([""])),
        ("units", json!(["   "])),
        ("units", json!(["bus identity"])),
        ("units", json!(["same", "same"])),
        ("units", json!(["z-bus", "a-bus"])),
    ] {
        let mut value = original.clone();
        value[field] = bad;
        assert!(parse(&value).is_err(), "accepted bad {field}");
    }
    for (field, bad) in [
        ("joint", json!("absent")),
        ("source_joint", json!("absent")),
        ("scale", json!(0.0)),
        ("offset", Value::Null),
        ("coulomb", json!(-1.0)),
        ("velocity_deadband", json!(-1.0)),
        ("transition_velocity", json!(0.0)),
        ("max_torque", json!(0.0)),
        ("extra", json!(true)),
    ] {
        let mut value = original.clone();
        value["joints"][0][field] = bad;
        assert!(parse(&value).is_err(), "accepted bad joint {field}");
    }
    let mut missing = original.clone();
    missing.as_object_mut().unwrap().remove("source_urdf");
    assert!(parse(&missing).is_err());
    let mut duplicate = original.clone();
    let entry = duplicate["joints"][0].clone();
    duplicate["joints"].as_array_mut().unwrap().push(entry);
    assert!(parse(&duplicate).is_err());
    let mut incomplete = original;
    incomplete["source_urdf"] = json!(PENDULUM);
    assert!(matches!(
        parse(&incomplete),
        Err(CompensationError::SourceCoverage)
    ));
}

#[test]
fn a_failed_step_preserves_outputs_and_slew_state() {
    let mut compensation = parse(&document()).unwrap();
    let mut effort = [7.0; 2];
    let mut gravity = [8.0; 2];
    for (q, v, dt) in [
        ([0.0, f64::NAN], [0.0; 2], 0.01),
        ([0.0; 2], [0.0, f64::INFINITY], 0.01),
        ([0.0; 2], [0.0; 2], 0.0),
        ([0.0; 2], [0.0; 2], f64::MAX),
    ] {
        assert!(
            compensation
                .step(&q, &v, dt, &mut effort, &mut gravity)
                .is_err()
        );
        assert_eq!(effort.map(f64::to_bits), [7.0_f64.to_bits(); 2]);
        assert_eq!(gravity.map(f64::to_bits), [8.0_f64.to_bits(); 2]);
    }
    assert!(
        compensation
            .step(&[0.0], &[0.0; 2], 0.01, &mut effort, &mut gravity)
            .is_err()
    );
    compensation
        .step(&[0.0; 2], &[0.0; 2], 0.01, &mut effort, &mut gravity)
        .unwrap();
    close(effort[1], gravity_at(0.0));
}

#[test]
fn steady_tracking_and_stop_gravity_allocate_nothing() {
    let mut compensation = parse(&document()).unwrap();
    let mut effort = [0.0; 2];
    let mut gravity = [0.0; 2];
    let measured = allocation_counter::measure(|| {
        for _ in 0..100 {
            compensation
                .step(&[0.0, 0.1], &[0.0, 0.2], 0.01, &mut effort, &mut gravity)
                .unwrap();
            compensation
                .gravity_into(&[0.0, 0.1], &mut gravity)
                .unwrap();
        }
    });
    assert_eq!(measured.count_total, 0);
}

#[test]
fn accepted_left_arm_gravity_matches_independent_python_oracle() {
    let profile = include_str!("fixtures/accepted_compensation/profile.json");
    let drive = include_bytes!("fixtures/accepted_compensation/drive.json");
    let urdf = include_str!("../../../render_packs/sentient-static-upper-body/robot.urdf");
    let document: Value = serde_json::from_str(profile).unwrap();
    let entries = document["joints"].as_array().unwrap();
    let joints: Vec<&str> = entries
        .iter()
        .map(|joint| joint["joint"].as_str().unwrap())
        .collect();
    let limits: Vec<f64> = entries
        .iter()
        .map(|joint| joint["max_torque"].as_f64().unwrap())
        .collect();
    let mut compensation = QuasiStaticCompensation::parse(profile, urdf, drive, &joints).unwrap();
    let oracle: Value =
        serde_json::from_str(include_str!("fixtures/accepted_compensation/gravity.json")).unwrap();
    let tolerance = oracle["absolute_tolerance_nm"].as_f64().unwrap();
    let cases = oracle["cases"].as_array().unwrap();
    let mut worst = 0.0_f64;
    for case in cases {
        let q: Vec<f64> = case["q"]
            .as_array()
            .unwrap()
            .iter()
            .map(|value| value.as_f64().unwrap())
            .collect();
        let expected = case["gravity"].as_array().unwrap();
        let mut gravity = vec![0.0; joints.len()];
        compensation.gravity_into(&q, &mut gravity).unwrap();
        for (index, got) in gravity.iter().enumerate() {
            let want = expected[index]
                .as_f64()
                .unwrap()
                .clamp(-limits[index], limits[index]);
            worst = worst.max((got - want).abs());
        }
    }
    assert!(worst < tolerance, "worst gravity mismatch {worst} Nm");
    println!(
        "{} accepted gravity cases: worst {worst:.9} Nm",
        cases.len()
    );
}
