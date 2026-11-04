//! The laws the kernel enforces (L3 right-handed inside, L5 checked composition), the codecs,
//! and that nothing past declaration touches the heap.

#![allow(clippy::float_cmp, clippy::many_single_char_names)] // exact layouts; a, b, c edges

use sx_frames::{
    Authority, Axes, Direction, FLU, FrameError, Frames, MUJOCO_EULER, Pose, Quaternion,
    QuaternionLayout, RDF, RFU, RUB, Rigid, Source, Transform, UNITY, URDF_RPY, basis,
    matrix_rotation_vector, recorded, rotation_vector_matrix,
};

fn close(a: &[f64], b: &[f64], tolerance: f64) -> bool {
    a.iter().zip(b).all(|(x, y)| (x - y).abs() <= tolerance)
}

#[test]
fn left_handed_axes_never_name_a_frame() {
    let mut frames = Frames::new();
    assert_eq!(
        frames.declare("unity", UNITY),
        Err(FrameError::LeftHanded(UNITY))
    );
    assert_eq!(basis(FLU, UNITY), Err(FrameError::LeftHanded(UNITY)));
    assert!(matches!(
        frames.declare("a b", FLU),
        Err(FrameError::Name(_))
    ));
    assert!(matches!(
        Axes::new(Direction::Up, Direction::Down, Direction::Left),
        Err(FrameError::AxesRepeatLine)
    ));
    let flu = frames.declare("body", FLU).expect("body");
    assert_eq!(frames.declare("body", FLU), Ok(flu));
    assert!(matches!(
        frames.declare("body", RDF),
        Err(FrameError::Redeclared { .. })
    ));
    assert_eq!(frames.name(flu), "body");
}

#[test]
fn composition_is_checked() {
    let mut frames = Frames::new();
    let world = frames.declare("world", FLU).expect("world");
    let body = frames.declare("body", FLU).expect("body");
    let camera = frames.declare("camera", FLU).expect("camera");
    let pose = Pose {
        translation_m: [1.0, 2.0, 3.0],
        rotation: Quaternion::IDENTITY,
    };
    let world_from_body = Transform::new(world, body, pose, Authority::Design).expect("edge");
    let body_from_camera = Transform::new(body, camera, pose, Authority::Design).expect("edge");
    let world_from_camera = world_from_body.compose(&body_from_camera).expect("meet");
    assert_eq!(
        (world_from_camera.parent(), world_from_camera.child()),
        (world, camera)
    );
    assert_eq!(world_from_camera.pose().translation_m, [2.0, 4.0, 6.0]);
    assert!(matches!(
        body_from_camera.compose(&world_from_body),
        Err(FrameError::Mismatch { .. })
    ));
    assert!(matches!(
        Transform::new(world, world, pose, Authority::Design),
        Err(FrameError::SelfEdge(_))
    ));
    // An edge and its inverse meet, but world<-world carries nothing and is refused.
    assert!(matches!(
        world_from_body.compose(&world_from_body.inverse()),
        Err(FrameError::SelfEdge(_))
    ));
    let body_from_world = world_from_body.inverse();
    assert_eq!(
        (body_from_world.parent(), body_from_world.child()),
        (body, world)
    );
}

#[test]
fn expressed_changes_the_frame_not_the_motion() {
    let mut frames = Frames::new();
    let world = frames.declare("world", FLU).expect("world");
    let camera = frames.declare("camera", RUB).expect("camera");
    let optical = frames.declare("camera_optical", RDF).expect("optical");
    let rotation = QuaternionLayout::Xyzw
        .read([0.1, 0.2, 0.3, 0.9])
        .normalized();
    let edge = Transform::new(
        world,
        camera,
        Pose {
            translation_m: [0.4, 0.5, 0.6],
            rotation,
        },
        Authority::Measured,
    )
    .expect("edge");
    let as_optical = edge.expressed(None, Some(optical)).expect("sibling");
    assert_eq!(as_optical.authority(), Authority::Measured);
    assert_eq!(as_optical.child(), optical);
    // RUB -> RDF is diag(1, -1, -1): the optical z is the camera's -z.
    let point = [0.0, 0.0, 2.0];
    let through_camera = edge.apply([0.0, 0.0, -2.0]);
    assert!(close(&as_optical.apply(point), &through_camera, 1e-12));
}

#[test]
fn unity_ingest_is_the_hand_written_flip() {
    // _unity_lh_to_openxr_rh: p = (x, y, -z), q = (-x, -y, z, w), from Unity RUF into RUB.
    let mut frames = Frames::new();
    let room = frames.declare("room", RUB).expect("room");
    let head = frames.declare("head", RUB).expect("head");
    let q = QuaternionLayout::Xyzw
        .read([0.1, -0.3, 0.2, 0.9])
        .normalized();
    let edge =
        recorded(room, head, [0.5, 1.5, 2.5], &q.matrix(), Source::new(UNITY)).expect("ingest");
    assert!(close(&edge.pose().translation_m, &[0.5, 1.5, -2.5], 1e-12));
    let want = Quaternion {
        x: -q.x,
        y: -q.y,
        z: q.z,
        w: q.w,
    }
    .canonical();
    assert!(close(
        &QuaternionLayout::Xyzw.write(edge.pose().rotation.canonical()),
        &QuaternionLayout::Xyzw.write(want),
        1e-12
    ));
    // Into a Z-up world it is still one call: no swizzle by hand.
    let world = frames.declare("world", RFU).expect("world");
    assert!(recorded(world, head, [0.0; 3], &q.matrix(), Source::new(UNITY)).is_ok());
    let mixed = Source {
        local: FLU,
        ..Source::new(UNITY)
    };
    assert!(matches!(
        recorded(room, head, [0.0; 3], &q.matrix(), mixed),
        Err(FrameError::MixedHandedness { .. })
    ));
}

#[test]
fn layouts_name_their_order() {
    let q = QuaternionLayout::Xyzw.read([1.0, 2.0, 3.0, 4.0]);
    assert_eq!((q.x, q.y, q.z, q.w), (1.0, 2.0, 3.0, 4.0));
    assert_eq!(QuaternionLayout::Wxyz.write(q), [4.0, 1.0, 2.0, 3.0]);
    assert_eq!(QuaternionLayout::Wxyz.read([4.0, 1.0, 2.0, 3.0]), q);
}

#[test]
fn urdf_rpy_and_mujoco_euler_differ() {
    let angles = [0.3, -1.1, 2.4];
    let urdf = URDF_RPY.matrix(angles);
    let mujoco = MUJOCO_EULER.matrix(angles);
    assert!(!close(urdf.as_flattened(), mujoco.as_flattened(), 1e-3));
    // The MJCF->URDF codec pair: the same rotation, two spellings.
    let rpy = URDF_RPY.angles(&mujoco).expect("angles");
    assert!(close(
        URDF_RPY.matrix(rpy).as_flattened(),
        mujoco.as_flattened(),
        1e-12
    ));
}

#[test]
fn rotation_vectors_round_trip() {
    for v in [
        [0.3, -0.2, 0.1],
        [0.0, 0.0, 3.1],
        [1e-9, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ] {
        let back = matrix_rotation_vector(&rotation_vector_matrix(v));
        assert!(close(&back, &v, 1e-9), "{v:?} -> {back:?}");
    }
}

#[test]
fn slerp_is_shortest_and_ends_at_its_ends() {
    let a = Quaternion::IDENTITY;
    let b = Quaternion::from_matrix(&rotation_vector_matrix([0.0, 0.0, 1.0]));
    let half = a.slerp(b, 0.5);
    assert!((a.angle_to(half) - 0.5).abs() < 1e-12);
    assert!(a.slerp(b, 1.0).angle_to(b) < 1e-7);
    let flipped = Quaternion {
        x: -b.x,
        y: -b.y,
        z: -b.z,
        w: -b.w,
    };
    assert!(a.slerp(flipped, 0.5).angle_to(half) < 1e-7);
}

#[test]
fn pose_and_rigid_agree() {
    let pose = Pose {
        translation_m: [0.1, -0.2, 0.3],
        rotation: QuaternionLayout::Xyzw
            .read([0.2, 0.1, -0.4, 0.8])
            .normalized(),
    };
    let other = Pose {
        translation_m: [1.0, 0.5, -0.5],
        rotation: QuaternionLayout::Xyzw
            .read([-0.3, 0.5, 0.1, 0.7])
            .normalized(),
    };
    let point = [0.7, 0.8, 0.9];
    let rigid: Rigid = pose.rigid();
    assert!(close(&pose.apply(point), &rigid.apply(point), 1e-12));
    let composed = pose.compose(&other);
    assert!(close(
        &composed.rigid().apply(point),
        &rigid.then(&other.rigid()).apply(point),
        1e-12
    ));
    assert!(close(
        &pose.compose(&pose.inverse()).translation_m,
        &[0.0; 3],
        1e-12
    ));
    assert!(close(&rigid.then(&rigid.inverse()).t, &[0.0; 3], 1e-12));
    assert_eq!(Rigid::from_row_major(&rigid.to_row_major()), rigid);
    assert!(rigid.pose().rotation.angle_to(pose.rotation) < 1e-7);
}

#[test]
fn pose_document_keeps_its_bytes() {
    let text = r#"{"translation_m":[0.0123,-0.06,0.1],"rotation_xyzw":[-0.5,0.5,-0.5,0.5]}"#;
    let pose: Pose = serde_json::from_str(text).expect("pose");
    assert_eq!(pose.rotation.w, 0.5);
    assert_eq!(serde_json::to_string(&pose).expect("json"), text);
    assert!(
        serde_json::from_str::<Pose>(
            r#"{"translation_m":[0,0,0],"rotation_xyzw":[0,0,0,1],"x":1}"#
        )
        .is_err()
    );
}

#[test]
fn the_tick_never_allocates() {
    let mut frames = Frames::new();
    let world = frames.declare("world", FLU).expect("world");
    let body = frames.declare("body", FLU).expect("body");
    let camera = frames.declare("camera", RDF).expect("camera");
    let pose = Pose {
        translation_m: [0.1, 0.2, 0.3],
        rotation: QuaternionLayout::Xyzw
            .read([0.1, 0.2, 0.3, 0.9])
            .normalized(),
    };
    let a = Transform::new(world, body, pose, Authority::Design).expect("edge");
    let b = Transform::new(body, camera, pose, Authority::Design).expect("edge");
    let counts = allocation_counter::measure(|| {
        for step in 0..1000 {
            let u = f64::from(step) / 1000.0;
            let c = a.compose(&b).expect("meet");
            let q = pose.rotation.slerp(c.pose().rotation, u);
            let r = q.matrix();
            let _ = std::hint::black_box((
                c.inverse().apply([u, 0.0, 0.0]),
                Quaternion::from_matrix(&r),
                pose.rigid().then(&c.pose().rigid()),
                recorded(world, camera, [u; 3], &r, Source::new(UNITY)),
                URDF_RPY.angles(&r),
                matrix_rotation_vector(&r),
                c.expressed(None, Some(body)),
            ));
        }
    });
    assert_eq!(counts.count_total, 0, "{counts:?}");
}

#[test]
fn from_matrix_keeps_w_non_negative_and_canonical_is_the_python_sign() {
    for angle in [0.3, 2.0, 3.0, 3.1, 4.0, 5.5, 6.2] {
        for axis in [[0.0, 0.0, 1.0], [0.6, -0.8, 0.0], [-0.48, 0.6, 0.64]] {
            let r = rotation_vector_matrix(axis.map(|v| v * angle));
            let q = Quaternion::from_matrix(&r);
            assert!(q.w >= 0.0, "{angle} {axis:?}: {q:?}");
            assert!(close(q.matrix().as_flattened(), r.as_flattened(), 1e-12));
        }
    }
    // A half turn has w = 0: the first non-zero of (w, x, y, z) decides, as in the Python Pose.
    let half = Quaternion {
        x: -1.0,
        y: 0.0,
        z: 0.0,
        w: 0.0,
    };
    assert_eq!(half.canonical().x, 1.0);
    let negative = QuaternionLayout::Xyzw.read([0.1, 0.2, 0.3, -0.9]);
    assert_eq!(
        QuaternionLayout::Xyzw.write(negative.canonical()),
        [-0.1, -0.2, -0.3, 0.9]
    );
}

#[test]
fn expressed_parent_is_the_computed_sibling_edge_first() {
    let mut frames = Frames::new();
    let world = frames.declare("world", FLU).expect("world");
    let world_rfu = frames.declare("world_rfu", RFU).expect("world_rfu");
    let head = frames.declare("head", RDF).expect("head");
    let edge = Transform::new(
        world,
        head,
        Pose {
            translation_m: [0.4, -0.2, 1.5],
            rotation: QuaternionLayout::Xyzw
                .read([0.1, 0.2, 0.3, 0.9])
                .normalized(),
        },
        Authority::Recorded,
    )
    .expect("edge");
    let moved = edge.expressed(Some(world_rfu), None).expect("sibling");
    assert_eq!((moved.parent(), moved.child()), (world_rfu, head));
    // RFU from FLU: x_rfu = -y_flu, y_rfu = x_flu.
    let point = [0.3, 0.1, -0.2];
    let flu = edge.apply(point);
    assert!(close(
        &moved.apply(point),
        &[-flu[1], flu[0], flu[2]],
        1e-12
    ));
}

#[test]
fn recorded_scales_its_unit() {
    let mut frames = Frames::new();
    let room = frames.declare("room", FLU).expect("room");
    let tag = frames.declare("tag", FLU).expect("tag");
    let source = Source {
        unit: sx_frames::LengthUnit::Millimetre,
        ..Source::new(FLU)
    };
    let edge = recorded(
        room,
        tag,
        [1000.0, -250.0, 5.0],
        &sx_frames::linalg::I3,
        source,
    )
    .expect("ingest");
    assert!(close(
        &edge.pose().translation_m,
        &[1.0, -0.25, 0.005],
        1e-15
    ));
}

#[test]
fn euler_angles_survive_gimbal_lock() {
    for sequence in [URDF_RPY, MUJOCO_EULER] {
        for pitch in [std::f64::consts::FRAC_PI_2, -std::f64::consts::FRAC_PI_2] {
            let r = sequence.matrix([0.4, pitch, -0.7]);
            let angles = sequence.angles(&r).expect("Tait-Bryan");
            assert!(
                close(
                    sequence.matrix(angles).as_flattened(),
                    r.as_flattened(),
                    1e-9
                ),
                "{sequence:?} {pitch}: {angles:?}"
            );
        }
    }
}

#[test]
fn rotation_vectors_near_a_half_turn_keep_their_axis_sign() {
    let angle = std::f64::consts::PI - 1e-6;
    for axis in [[0.48, -0.6, 0.64], [-0.8, 0.0, 0.6], [0.0, 0.6, -0.8]] {
        let v = axis.map(|a| a * angle);
        let back = matrix_rotation_vector(&rotation_vector_matrix(v));
        assert!(close(&back, &v, 1e-6), "{v:?} -> {back:?}");
    }
}
