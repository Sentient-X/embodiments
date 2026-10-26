# sx-embodiment-ik

The solver that turns tool and look-at targets into the next joint positions, once per
tick. It solves one constrained QP over a body's render pack (`robot.urdf` plus
`collision.json`). The station runs it as the authority, and the headset links the same
crate to draw its ghost.

```rust
let body = Body::static_upper_body(); // left, right, head
let caps = Caps::declared(&body, 0.25)?.at_most(governor_speed);
let mut ik = Ik::new(body, caps, Settings::default(), vec![table])?;
ik.reset(); // on engage
let step = ik.step(&q_measured, &available, &[left, right, look], dt)?;
// step.q, step.qdot: every axis; step.limbs[i]: residual, reach, binding constraints
```

- **Costs:** tool position, then orientation (0.15 m per rad), then posture in the null space.
- **Hard constraints:** joint limits with braking, speed and acceleration caps, a 0.01 rad
  per-tick step, and self and table collision. Collision is checked on capsule proxies,
  with each pair's approach planned to stop before a 1 cm margin.
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
