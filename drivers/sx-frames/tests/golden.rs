//! Every case in the Python kernel's golden file, answered by this crate.
//!
//! `tests/golden/frames.json` is a byte-identical copy of `packages/sx-geometry/golden/frames.json`
//! (the superproject's `packages/sx-geometry/tests/test_golden.py` refuses a drift).

use serde_json::Value;
use sx_frames::{
    Axes, Direction, EulerSequence, Frames, Handedness, Matrix3, NAMED_AXES, QuaternionLayout,
    Source, basis, basis_quaternion, recorded,
};

const TOLERANCE: f64 = 1e-11;

fn golden() -> Value {
    serde_json::from_str(include_str!("golden/frames.json")).expect("golden file is JSON")
}

fn floats(value: &Value) -> Vec<f64> {
    value
        .as_array()
        .expect("array")
        .iter()
        .map(|v| v.as_f64().expect("number"))
        .collect()
}

fn vec3(value: &Value) -> [f64; 3] {
    floats(value).try_into().expect("three numbers")
}

fn vec4(value: &Value) -> [f64; 4] {
    floats(value).try_into().expect("four numbers")
}

fn matrix(value: &Value) -> Matrix3 {
    let rows: Vec<[f64; 3]> = value.as_array().expect("rows").iter().map(vec3).collect();
    rows.try_into().expect("three rows")
}

fn axes(name: &Value) -> Axes {
    let name = name.as_str().expect("axes name");
    NAMED_AXES
        .iter()
        .find(|(known, _)| *known == name)
        .map_or_else(|| panic!("no named axes {name}"), |(_, axes)| *axes)
}

fn close(got: &[f64], want: &[f64], what: &str) {
    assert_eq!(got.len(), want.len(), "{what}");
    for (g, w) in got.iter().zip(want) {
        assert!(
            (g - w).abs() <= TOLERANCE,
            "{what}: got {got:?}, want {want:?}"
        );
    }
}

fn close_matrix(got: &Matrix3, want: &Matrix3, what: &str) {
    close(got.as_flattened(), want.as_flattened(), what);
}

#[test]
fn every_named_axes_value() {
    let cases = golden()["axes"].as_array().expect("axes").clone();
    assert_eq!(cases.len(), NAMED_AXES.len());
    for (case, (name, value)) in cases.iter().zip(NAMED_AXES) {
        assert_eq!(case["name"], name);
        assert_eq!(value.to_string(), name);
        let directions: Vec<&str> = value.directions().iter().map(|d| d.as_str()).collect();
        let want: Vec<&str> = case["directions"]
            .as_array()
            .expect("directions")
            .iter()
            .map(|d| d.as_str().expect("word"))
            .collect();
        assert_eq!(directions, want, "{name}");
        let parsed: Vec<Direction> = want
            .iter()
            .map(|w| Direction::parse(w).expect("word"))
            .collect();
        assert_eq!(parsed, value.directions());
        assert_eq!(Axes::parse(name).expect("spelling"), value);
        assert_eq!(value.handedness().as_str(), case["handedness"], "{name}");
        close_matrix(
            &value.facing_from_axes(),
            &matrix(&case["facing_from_axes"]),
            name,
        );
    }
}

#[test]
fn every_basis_change() {
    let cases = golden()["basis"].as_array().expect("basis").clone();
    assert_eq!(cases.len(), 20);
    for case in &cases {
        let (parent, child) = (axes(&case["parent"]), axes(&case["child"]));
        let what = format!("{parent}<-{child}");
        let got = basis(parent, child).expect("right-handed");
        close_matrix(&got, &matrix(&case["parent_from_child"]), &what);
        let q = basis_quaternion(parent, child).expect("right-handed");
        close(
            &QuaternionLayout::Xyzw.write(q),
            &vec4(&case["rotation_xyzw"]),
            &what,
        );
    }
}

#[test]
fn every_recorded_ingest() {
    let cases = golden()["recorded"].as_array().expect("recorded").clone();
    assert_eq!(cases.len(), 9);
    for case in &cases {
        let mut frames = Frames::new();
        let parent = frames
            .declare("parent", axes(&case["parent"]))
            .expect("parent");
        let child = frames
            .declare("child", axes(&case["child"]))
            .expect("child");
        let source = Source {
            local: axes(&case["local"]),
            ..Source::new(axes(&case["space"]))
        };
        let rotation = QuaternionLayout::Xyzw
            .read(vec4(&case["rotation_xyzw"]))
            .matrix();
        let edge =
            recorded(parent, child, vec3(&case["translation"]), &rotation, source).expect("ingest");
        let what = format!("{case}");
        close(
            &edge.pose().translation_m,
            &vec3(&case["expect_translation_m"]),
            &what,
        );
        let q = edge.pose().rotation.canonical();
        close(
            &QuaternionLayout::Xyzw.write(q),
            &vec4(&case["expect_rotation_xyzw"]),
            &what,
        );
        assert_eq!(edge.parent(), parent);
        assert_eq!(edge.child(), child);
    }
}

#[test]
fn every_euler_sequence() {
    let cases = golden()["euler"].as_array().expect("euler").clone();
    assert_eq!(cases.len(), 4);
    for case in &cases {
        let sequence = EulerSequence::new(
            case["axes"].as_str().expect("axes"),
            case["intrinsic"].as_bool().expect("intrinsic"),
        )
        .expect("sequence");
        let angles = vec3(&case["angles"]);
        let got = sequence.matrix(angles);
        close_matrix(&got, &matrix(&case["matrix"]), &format!("{case}"));
        // And back: these angles are on the principal branch.
        close(
            &sequence.angles(&got).expect("Tait-Bryan"),
            &angles,
            "angles",
        );
    }
}

#[test]
fn unity_is_the_only_left_handed_value() {
    for (name, value) in NAMED_AXES {
        assert_eq!(
            value.handedness() == Handedness::Left,
            name == "RUF",
            "{name}"
        );
    }
}
