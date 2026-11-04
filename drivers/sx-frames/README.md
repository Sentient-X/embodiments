# sx-frames

The frames kernel in Rust, twin of the Python `sx_geometry.frames` (sx
`docs/plans/frames-2026-11.md`). It is the lowest Rust layer: the IK, the drivers' media
plans, the station, the pod and the console WASM all reach it.

```rust
let mut frames = Frames::new();                       // at setup: names interned once
let room = frames.declare("room", RUB)?;              // right-handed only
let head = frames.declare("quest3s_head", RDF)?;
// A Unity (left-handed RUF) sample: reflected once, here, never by hand.
let q = QuaternionLayout::Xyzw.read(sample_xyzw);
let edge = recorded(room, head, sample_p, &q.matrix(), Source::new(UNITY))?;
let room_from_optical = edge.compose(&head_from_optical)?;   // FrameMismatch if they don't meet
```

- **Axes** `FLU RDF RUB RFU LUF` and the left-handed `UNITY` (RUF); `basis(parent, child)` is
  the computed signed permutation.
- **Frame** is a `FrameId` plus axes; **Transform** `parent_from_child` is `Copy`, and
  `compose` is two integer compares. Names stay in `Frames`, so nothing on a tick allocates.
- **Pose** (translation + `Quaternion` with named `x y z w`) is what crosses a boundary; as JSON
  it is `{"translation_m": [..], "rotation_xyzw": [..]}`. **Rigid** (matrix + translation) is
  what a kinematic chain composes per link.
- Codecs: `QuaternionLayout` (xyzw/wxyz), `EulerSequence` (`URDF_RPY` extrinsic xyz,
  `MUJOCO_EULER` intrinsic xyz), rotation vectors, `Quaternion::slerp`.

`cargo test -p sx-frames` runs every case of `tests/golden/frames.json`, a byte-identical copy
of sx's `packages/sx-geometry/golden/frames.json` (sx's `test_golden.py` refuses a drift), plus
the laws and a zero-allocation tick.
