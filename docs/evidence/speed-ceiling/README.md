# How fast the static upper body can move, and what limits it (2026-10-30)

`python tools/sim_speed_ceiling.py --joints left_,torso --markdown docs/evidence/speed-ceiling/README.md`
(MuJoCo; the right arm mirrors the left). Each joint is moved 1.2 rad (less where its range is
shorter) from the zero pose, where both arms reach straight out and the shoulders and elbows
carry the most, and back, by the governor's own trajectory. Its limits are the registry's
rated motion (`sx_embodiments.calibrate.motion`) held to the gains' ceilings
`DamiaoChain::tracking_ceilings` applies: speed at most `0.2 kp/kd`, `0.1 sqrt(kp/J)` and
`sqrt(0.2 peak/J)`, acceleration at most `0.1 kp/J`; its planned velocity is capped at
`0.1 kp/kd` as `DamiaoChain` caps it. The truth model is `sim_joint_tracking.py`'s with every
link 2.61 times the description's mass, BAM friction per motor model, the prototype MIT gains
of `drivers/sx-damiao-can/src/static_upper_body.rs`, and each motor's law at 1 kHz saturated at
peak torque. "Peak tracking" is the joint's position in the answer to each command against the
position that command just sent; "worst share of bound" is the largest ratio of that error to
the governor's bound at that tick (0.2 rad at rest, widened by three 20 ms steps of the speed
the joint was recently stepped at): over 100 % is a tracking stop.

Fourth run of 10-30, after three refute passes (act-refute-2).

What it shows:

- **The prototype gains, not the motors, set the arms' speed.** The shoulders run at
  1.0-1.26 rad/s (rated 10.47): faster, a host that stalls mid-move leaves momentum the held
  position cannot stop within 0.1 rad (`0.1 sqrt(kp/J)`; at their rated speed they coasted
  0.58 rad in the MIT simulation). The DM4340 joints run at 0.96-1.31 rad/s (rated 3.77): faster,
  their kd/kp of 0.15-0.21 s lags them past the bound. The wrist pitch runs at 3.38 rad/s and the
  roll at 4.0 (rated 12.57). The torso keeps 0.25 rad/s until it is identified and re-tuned.
- **With the planned velocity every joint tracks inside its bound but the elbow on PD alone**,
  which settles 0.21 rad short against gravity the description lacks and trips at rest, at any
  speed: the real left elbow's signature of 2026-10-26 (failures.md). It needs gravity
  feed-forward (`v_des + model` is under the bound in the 1.2 rad move at 50 Hz and 100 Hz in
  earlier runs; measured gravity closes it further). Without the planned velocity the DM4340
  joints trip.
- **A stalled host, at each joint's top speed**, momentum and gravity sag included, until the
  0.75 s watchdog: shoulders 0.02-0.09, elbow 0.19, wrist pitch 0.15, others under 0.1 rad past
  the last position sent (the table after the run). `sx-damiao-can/tests/mit_dynamics.rs`
  holds the same on simulated time without gravity: the elbow within 0.1 rad, the shoulders and
  wrist within 0.2.
- **The watchdog window does not bound the coast; the velocity cap and the momentum ceiling
  do.** The elbow alone at TIMEOUT 0.1, 0.3 and 0.75 s (`watchdog_window`, second table):
  capped, 0.05 rad at 0.75 s; uncapped at 3.77 rad/s, 0.40 rad even at 0.1 s, since cutting
  torque removes the brake, and a loaded arm falls once limp. The TIMEOUT stays 0.75 s: the
  station rides through late ticks for up to 250 ms (`HOLD_HORIZON`), so 0.3 s is the shortest
  safe window, and it buys 0.02 rad.
- **The loop rate is not the ceiling.** 100 Hz tracks no better than 50 Hz under these gains.
- **TCP speed** at the reach pose, every joint at its governed speed: 0.26 m/s along each arm's
  slowest axis direction, 1.28 m/s along its fastest, at share 1; about 0.07 and 0.32 m/s at
  the 0.25 a station opens at until a person raises it.
- **To go faster**, identify each joint and re-tune its gains (robot-bench skill, "Going
  faster"): the elbows and wrists can approach their rated speeds; the shoulders cannot at a
  well-damped ratio within the MIT frame's kd range of 5. The acceleration and inertia here are
  estimates (the description's masses times 2.61, the mass matrix's diagonal, no rotor
  inertia).

## The run

Limits at 100% of the registry's (sx_embodiments.calibrate.motion), truth 2.61x the description's mass, prototype gains.

| Joint | Speed rad/s | Accel rad/s² | Hz | Controller | Peak tracking rad | Worst share of bound | Settled rad | Peak torque |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| torso_joint | 0.25 | 0.3 | 50 | v_des 0 | 0.082 | 38% | 0.024 | 8% |
| torso_joint | 0.25 | 0.3 | 50 | v_des | 0.076 | 35% | 0.024 | 8% |
| torso_joint | 0.25 | 0.3 | 50 | v_des + model | 0.076 | 36% | 0.024 | 8% |
| left_shoulder_pitch_joint | 1.24 | 15.4 | 50 | v_des 0 | 0.124 | 46% | 0.055 | 16% |
| left_shoulder_pitch_joint | 1.24 | 15.4 | 50 | v_des | 0.104 | 38% | 0.056 | 16% |
| left_shoulder_pitch_joint | 1.24 | 15.4 | 50 | v_des + model | 0.085 | 31% | 0.033 | 16% |
| left_shoulder_roll_joint | 1.19 | 14.1 | 50 | v_des 0 | 0.151 | 57% | 0.021 | 20% |
| left_shoulder_roll_joint | 1.19 | 14.1 | 50 | v_des | 0.130 | 49% | 0.015 | 20% |
| left_shoulder_roll_joint | 1.19 | 14.1 | 50 | v_des + model | 0.097 | 36% | 0.028 | 20% |
| left_upper_arm_yaw_joint | 0.96 | 19.3 | 50 | v_des 0 | 0.287 | 112% **trips** | 0.076 | 11% |
| left_upper_arm_yaw_joint | 0.96 | 19.3 | 50 | v_des | 0.187 | 73% | 0.076 | 11% |
| left_upper_arm_yaw_joint | 0.96 | 19.3 | 50 | v_des + model | 0.185 | 72% | 0.074 | 11% |
| left_elbow_pitch_joint | 1.31 | 24.6 | 50 | v_des 0 | 0.399 | 149% **trips** | 0.208 | 25% |
| left_elbow_pitch_joint | 1.31 | 24.6 | 50 | v_des | 0.307 | 114% **trips** | 0.209 | 25% |
| left_elbow_pitch_joint | 1.31 | 24.6 | 50 | v_des + model | 0.263 | 97% | 0.155 | 26% |
| left_forearm_yaw_joint | 1.22 | 37.7 | 50 | v_des 0 | 0.274 | 100% **trips** | 0.060 | 9% |
| left_forearm_yaw_joint | 1.22 | 37.7 | 50 | v_des | 0.175 | 64% | 0.051 | 12% |
| left_forearm_yaw_joint | 1.22 | 37.7 | 50 | v_des + model | 0.175 | 64% | 0.051 | 12% |
| left_wrist_pitch_joint | 3.38 | 99.1 | 50 | v_des 0 | 0.275 | 68% | 0.071 | 34% |
| left_wrist_pitch_joint | 3.38 | 99.1 | 50 | v_des | 0.179 | 44% | 0.056 | 48% |
| left_wrist_pitch_joint | 3.38 | 99.1 | 50 | v_des + model | 0.159 | 40% | 0.035 | 48% |
| left_wrist_roll_joint | 4.00 | 125.7 | 50 | v_des 0 | 0.258 | 59% | 0.004 | 26% |
| left_wrist_roll_joint | 4.00 | 125.7 | 50 | v_des | 0.158 | 36% | 0.004 | 43% |
| left_wrist_roll_joint | 4.00 | 125.7 | 50 | v_des + model | 0.159 | 36% | 0.004 | 43% |
| torso_joint | 0.25 | 0.3 | 100 | v_des 0 | 0.081 | 38% | 0.025 | 8% |
| torso_joint | 0.25 | 0.3 | 100 | v_des | 0.075 | 35% | 0.025 | 8% |
| torso_joint | 0.25 | 0.3 | 100 | v_des + model | 0.075 | 35% | 0.025 | 8% |
| left_shoulder_pitch_joint | 1.24 | 15.4 | 100 | v_des 0 | 0.117 | 44% | 0.053 | 15% |
| left_shoulder_pitch_joint | 1.24 | 15.4 | 100 | v_des | 0.097 | 37% | 0.054 | 15% |
| left_shoulder_pitch_joint | 1.24 | 15.4 | 100 | v_des + model | 0.079 | 29% | 0.030 | 16% |
| left_shoulder_roll_joint | 1.19 | 14.1 | 100 | v_des 0 | 0.145 | 55% | 0.017 | 19% |
| left_shoulder_roll_joint | 1.19 | 14.1 | 100 | v_des | 0.125 | 47% | 0.021 | 19% |
| left_shoulder_roll_joint | 1.19 | 14.1 | 100 | v_des + model | 0.090 | 34% | 0.021 | 19% |
| left_upper_arm_yaw_joint | 0.96 | 19.3 | 100 | v_des 0 | 0.283 | 113% **trips** | 0.076 | 11% |
| left_upper_arm_yaw_joint | 0.96 | 19.3 | 100 | v_des | 0.183 | 72% | 0.083 | 11% |
| left_upper_arm_yaw_joint | 0.96 | 19.3 | 100 | v_des + model | 0.180 | 71% | 0.075 | 11% |
| left_elbow_pitch_joint | 1.31 | 24.6 | 100 | v_des 0 | 0.392 | 148% **trips** | 0.208 | 24% |
| left_elbow_pitch_joint | 1.31 | 24.6 | 100 | v_des | 0.301 | 112% **trips** | 0.209 | 24% |
| left_elbow_pitch_joint | 1.31 | 24.6 | 100 | v_des + model | 0.256 | 95% | 0.156 | 25% |
| left_forearm_yaw_joint | 1.22 | 37.7 | 100 | v_des 0 | 0.268 | 101% **trips** | 0.058 | 8% |
| left_forearm_yaw_joint | 1.22 | 37.7 | 100 | v_des | 0.169 | 63% | 0.051 | 11% |
| left_forearm_yaw_joint | 1.22 | 37.7 | 100 | v_des + model | 0.169 | 63% | 0.051 | 11% |
| left_wrist_pitch_joint | 3.38 | 99.1 | 100 | v_des 0 | 0.259 | 65% | 0.071 | 28% |
| left_wrist_pitch_joint | 3.38 | 99.1 | 100 | v_des | 0.162 | 40% | 0.061 | 37% |
| left_wrist_pitch_joint | 3.38 | 99.1 | 100 | v_des + model | 0.142 | 35% | 0.040 | 38% |
| left_wrist_roll_joint | 4.00 | 125.7 | 100 | v_des 0 | 0.237 | 55% | 0.004 | 14% |
| left_wrist_roll_joint | 4.00 | 125.7 | 100 | v_des | 0.137 | 31% | 0.004 | 22% |
| left_wrist_roll_joint | 4.00 | 125.7 | 100 | v_des + model | 0.137 | 31% | 0.003 | 22% |

If the host stops commanding when the joint moves at its top governed speed (or the fastest its range lets it reach), the v_des controller at 50 Hz: how far past the last position it goes before its 0.75 s watchdog, momentum and gravity included:

| Joint | Speed at the stall rad/s | Coast rad |
| --- | --- | --- |
| torso_joint | 0.25 | 0.022 |
| left_shoulder_pitch_joint | 1.24 | 0.094 |
| left_shoulder_roll_joint | 1.19 | 0.019 |
| left_upper_arm_yaw_joint | 0.96 | 0.027 |
| left_elbow_pitch_joint | 1.31 | 0.189 |
| left_forearm_yaw_joint | 1.22 | 0.063 |
| left_wrist_pitch_joint | 3.38 | 0.154 |
| left_wrist_roll_joint | 4.00 | 0.094 |

How far the elbow goes past its last position after a host stall, by how long its TIMEOUT lets it hold the last frame (single joint, `watchdog_window`), rad:

| Speed rad/s | Planned velocity | 0.1 s | 0.3 s | 0.75 s | 0.1 s, 4.5 N m along |
| --- | --- | --- | --- | --- | --- |
| 1.31 | uncapped | 0.078 | 0.134 | 0.153 | falls to its stop |
| 1.31 | capped | -0.022 | 0.034 | 0.053 | falls to its stop |
| 3.77 | uncapped | 0.401 | 0.477 | 0.528 | falls to its stop |
| 3.77 | capped | -0.074 | 0.002 | 0.052 | falls to its stop |

left arm TCP at the reach pose: 0.26 m/s in its slowest axis direction, 1.28 m/s in its fastest.
right arm TCP at the reach pose: 0.26 m/s in its slowest axis direction, 1.28 m/s in its fastest.
