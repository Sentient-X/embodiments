# sx-embodiment-ik

The solver that turns tool and look-at targets into the next joint positions, once per
tick. It solves one constrained QP over a body's render pack (`robot.urdf` plus
`collision.json`). The station runs it as the authority, and the headset links the same
crate to draw its ghost.

```rust
let body = Body::static_upper_body(); // left, right, head
let caps = Caps::declared(&body, 0.25)?.at_most(governor_speed);
let mut ik = Ik::new(body, caps, Settings::default(), Vec::new())?; // + the body's declared table
ik.reset(); // on engage
let step = ik.step(&q_measured, &available, &[left, right, look], dt)?;
// step.q, step.qdot: every axis; step.limbs[i]: residual, reach, binding constraints
```

- **Costs:** tool position, then orientation (0.15 m per rad), a damping term on each
  tick's joint step (damped least squares: the smallest joint motion that closes the error),
  then posture toward the anchor, the measured posture at engage (`Ik::reset`), never a fixed
  rest. Each pose error is closed no faster than the limb can still stop on it along its
  damped least-squares direction, so a step command never overshoots against the
  acceleration caps and wanders the free direction: a 3 cm hand move costs at most about
  1.5x its least-norm joint motion, and holding still moves nothing
  (`a_small_hand_move_is_a_small_joint_move`).
- **Hard constraints:** joint limits with braking, speed and acceleration caps, a 0.01 rad
  per-tick step, and self and table collision. The table is the body's own: its base mount
  declares it in `collision.json` (`support`, the mount frame's z = 0 plane, solid below
  everywhere within reach), and every `Ik` adds it to the scene it is given. Collision is checked on capsule proxies,
  with each pair's approach planned to stop before a 1 cm margin.
- **The camera:** a look-at limb points at its target (teleop's `HeadPolicy::Hands` aims
  halfway between the hands *as the camera sees them*, so both sit the same angle from the
  picture's centre), and turns on its own neck only, never on the torso that carries the
  arms. `LimbReport::hidden_by` names the link standing between the camera and a tool.
  Two more looking rows were tried and removed (October 2026): keeping the picture level and
  stepping the camera around an arm. On recorded operators (`examples/film.rs`) they cut the
  roll from about 20 to 2 degrees and added a few points of clear sight, but this SO-101 neck
  cannot level the ZED inside its tilt range: the tilt sat at its stop up to 74% of the time
  and the parallel pitch joints swung against each other (neck travel 4x main). Today the
  neck moves as main's does (`the_neck_holds_steady_while_both_hands_circle`). A level
  picture is better had by rotating the image from the known camera pose.
- **Unreachable targets:** the solver degrades smoothly and reports
  `Reach::Unreachable` with the residual.
- **Unavailable axes:** `available[axis] = false` holds an axis at its measured position.
  Vectors never shrink.

Real-time shape:

- The QP (`src/qp.rs`, Mehrotra interior point, dense Cholesky) and every buffer are
  sized in `Ik::new`.
- A steady-state tick allocates nothing (`tests/allocation.rs`) and logs nothing.
- A tick runs at most `max_iterations` (40) and stops at `Settings::deadline` (4 ms) with
  `Solve::Held`.

`examples/replay.rs` replays recorded YUBI operator trajectories into the static upper
body.
