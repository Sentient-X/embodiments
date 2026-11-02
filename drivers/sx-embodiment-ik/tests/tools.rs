//! Every committed render pack's tools read, each in a frame of its own tree, and the static
//! upper body's grippers close across their fingers in the solve's own tool frames.

use std::path::Path;

use sx_embodiment_ik::teleop::TeleopConfig;
use sx_embodiment_ik::{Body, Grip, JawKind};

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

#[test]
fn every_pack_states_each_tool_in_its_own_tree() {
    let packs = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../render_packs");
    let mut read = 0;
    for entry in std::fs::read_dir(&packs).expect("render packs") {
        let dir = entry.expect("pack").path();
        let file = |name: &str| std::fs::read_to_string(dir.join(name)).expect(name);
        let body = Body::new(&file("robot.urdf"), &file("collision.json"), &[])
            .and_then(|body| body.with_tools(&file("pack.json")))
            .unwrap_or_else(|e| panic!("{}: {e}", dir.display()));
        assert!(!body.tools().is_empty(), "{}", dir.display());
        for tool in body.tools() {
            let restated = body
                .tool(&tool.frame)
                .expect("a tool rides with its own frame");
            assert_eq!(restated.component, tool.component);
            match (&restated.grip, &tool.grip) {
                (Grip::Jaws(again), Grip::Jaws(jaws)) => {
                    for (a, b) in [
                        (again.closing_axis, jaws.closing_axis),
                        (again.approach_axis, jaws.approach_axis),
                        (again.grasp_point_m, jaws.grasp_point_m),
                    ] {
                        assert!(a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-12));
                    }
                }
                (again, grip) => assert_eq!(again, grip),
            }
            match &tool.grip {
                Grip::Jaws(jaws) => {
                    assert!((dot(jaws.closing_axis, jaws.closing_axis) - 1.0).abs() < 1e-6);
                    assert!(dot(jaws.closing_axis, jaws.approach_axis).abs() < 1e-6);
                }
                Grip::None { detail, .. } => assert!(!detail.is_empty()),
            }
        }
        read += 1;
    }
    assert!(read >= 2, "the committed packs");
}

#[test]
fn the_static_upper_body_closes_across_its_fingers_in_the_solves_tool_frames() {
    let body = Body::static_upper_body();
    for limb in TeleopConfig::static_upper_body_limbs() {
        let Some(approach) = limb.approach else {
            assert!(body.tool(&limb.frame).is_none(), "the camera is no tool");
            continue;
        };
        let tool = body
            .tool(&limb.frame)
            .expect("each gripper's tool point is a tool");
        assert_eq!(tool.frame, limb.frame);
        let Grip::Jaws(jaws) = tool.grip else {
            panic!("{} has jaws", limb.name)
        };
        assert_eq!(jaws.kind, JawKind::Parallel);
        // The registry's approach is the solve's to within a few degrees.
        assert!(dot(jaws.approach_axis, approach) > 5.0_f64.to_radians().cos());
        let closing = jaws
            .closing_normal_to(approach)
            .expect("a line across the approach");
        assert!(dot(closing, approach).abs() < 1e-12);
        for (got, want) in closing.iter().zip([0.0, 1.0, 0.0]) {
            assert!((got - want).abs() < 1e-3, "{}: {closing:?}", limb.name);
        }
    }
}
