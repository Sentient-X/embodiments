# Joint tracking with model feed-forward, simulated (slice 3b)

`tools/sim_joint_tracking.py` produced the numbers below. The setup:

- **Truth robot.** The static upper body in MuJoCo, with every URDF mass scaled by a random 0.7–1.3 and each centre of mass moved up to 1 cm.
- **Friction.** BAM-family drive friction per motor: Coulomb, Stribeck, load-dependent, viscous and armature. MuJoCo applies it as `frictionloss` and treats it as stiction. The elbows' breakaway is set to the ±2.5–3 N·m band measured on the robot on 2026-10-26.
- **Motors.** 1 kHz Damiao MIT loops, saturated at peak torque. Backlash and ripple are not modelled; BAM's bench procedure is the next model.
- **Trajectory.** The operator's real 20 s two-arm IK commands from 2026-10-25 (`operator-2026-10-25.csv`), rate-limited to 0.5 rad/s and low-passed at 2 Hz.
- **Gains.** Supraj's per-motor gains for (i) and (ii); a quarter of kp and half of kd for (iii)–(v).

Sanity check: the prototype PD tracking the raw stepping command gives 0.086 rad RMS in simulation; the real robot measured 0.096 rad RMS.

| Controller | Joint RMS (rad) | Joint max (rad) | Tip RMS (mm) | Tip max (mm) |
| --- | --- | --- | --- | --- |
| (i) prototype PD, v_des=0, tau_ff=0 | 0.0516 | 0.181 | 48.2 | 113.4 |
| (ii) prototype PD + planned velocity | 0.0371 | 0.125 | 43.7 | 82.4 |
| (iii) URDF feed-forward + low PD + velocity | 0.1125 | 0.287 | 94.5 | 176.7 |
| (iv) identified feed-forward + low PD + velocity | 0.0426 | 0.249 | 36.2 | 109.1 |
| (v) identified + in-place BAM friction + low PD + velocity | 0.0236 | 0.147 | 29.6 | 74.3 |

Reference: 20.2 s, peak planned speed 0.56 rad/s, both arms.
Prototype PD on the raw stepping command: simulated 0.086 rad RMS, the robot measured 0.096 rad RMS.

Identification in simulation, the runbook's plan as `sx-identify` plays it (PD only, torso held): 20 held poses, each approached from below and above; one 2 x 10 s excitation per arm at <= 0.25 rad/s and 1 rad/s²:
- base parameters seen: 128; column-scaled condition number of the stacked regressor: 884278
- excitation condition numbers: left 136781, right 153179
- worst joint RMS torque residual: holds 0.536 N·m, trajectory 1.175 N·m
- gravity torque error vs the truth at 50 random poses (mean |error| / max): URDF 0.022 / 0.13 N·m, identified 0.063 / 0.51 N·m
- links returned to the design estimate as unphysical: ['left_elbow_pitch_link', 'left_wrist_roll_link', 'left_gripper_left_finger_link', 'left_gripper_right_finger_link']
- run time 135 s

Gains (kp N·m/rad, kd N·m·s/rad):

| Joint | Motor | (i)/(ii) kp | kd | (iii)/(iv) kp | kd |
| --- | --- | --- | --- | --- | --- |
| torso_joint | DM8009 | 40 | 1 | 10 | 0.5 |
| left_shoulder_pitch_joint | DM8009 | 60.984 | 1.091 | 15.246 | 0.5455 |
| left_shoulder_roll_joint | DM8009 | 60.984 | 1.091 | 15.246 | 0.5455 |
| left_upper_arm_yaw_joint | DM4340 | 24 | 5 | 6 | 2.5 |
| left_elbow_pitch_joint | DM4340 | 30.492 | 4.655 | 7.623 | 2.3275 |
| left_forearm_yaw_joint | DM4340 | 29.04 | 4.75 | 7.26 | 2.375 |
| left_wrist_pitch_joint | DM4310 | 20 | 1 | 5 | 0.5 |
| left_wrist_roll_joint | DM4310 | 20 | 1 | 5 | 0.5 |
| right_shoulder_pitch_joint | DM8009 | 60.984 | 1.091 | 15.246 | 0.5455 |
| right_shoulder_roll_joint | DM8009 | 42 | 0.98 | 10.5 | 0.49 |
| right_upper_arm_yaw_joint | DM4340 | 30.492 | 4.655 | 7.623 | 2.3275 |
| right_elbow_pitch_joint | DM4340 | 30.492 | 4.655 | 7.623 | 2.3275 |
| right_forearm_yaw_joint | DM4340 | 30.492 | 4.655 | 7.623 | 2.3275 |
| right_wrist_pitch_joint | DM4310 | 20 | 1 | 5 | 0.5 |
| right_wrist_roll_joint | DM4310 | 20 | 1 | 5 | 0.5 |

Description sha256 e78a6c1603b2e38876c3da6d0c332e37d71cf804a9042358293a845f246ae4fe.

Settled error after a 0.5 rad elbow jog at 0.3 rad/s, prototype gains (target minus measured, degrees; robot 2026-10-26: right +5.4 / -3.7, left +5.9 / -5.3):

| Joint | Controller | From below | From above |
| --- | --- | --- | --- |
| right_elbow_pitch_joint | PD only | +0.8 | -6.0 |
| right_elbow_pitch_joint | PD + velocity + gravity and friction feed-forward | -0.0 | -0.1 |
| left_elbow_pitch_joint | PD only | +6.2 | -0.7 |
| left_elbow_pitch_joint | PD + velocity + gravity and friction feed-forward | +0.2 | +0.2 |


## Reading the result

- **Friction dominates this arm.** The URDF's hull-estimated links are light: holding gravity is under 2 N·m at mid-range. The elbows' stiction band is larger than that.
  - URDF feed-forward with low gains (iii) is worse than the prototype's PD. Low gains cannot push through stiction that the feed-forward ignores.
  - Adding the in-place BAM friction (v) halves the prototype's joint RMS error (0.024 vs 0.052 rad) and cuts tip RMS error from 48 to 30 mm.
- **The elbows' ±5° settling band closes in simulation.** On the bench jog (0.5 rad at 0.3 rad/s) it drops to ±0.2° with friction-plus-velocity feed-forward at the prototype's gains. This is simulation with the friction known exactly. On synthetic sweeps, the in-place fit recovers it within about 2 % (`tests/test_friction.py`, `fit-friction`).
- **Held-pose gravity identification does not pay on this arm yet.**
  - The friction band swamps a gravity signal this small. Averaging the two approach directions leaves 0.5 N·m residuals, and the identified gravity is less accurate than the URDF's (0.063 vs 0.022 N·m mean error).
  - The slow excitation (0.25 rad/s) leaves inertia poorly conditioned.
  - The in-place sweeps already identify each swept joint's gravity curve (`A sin q + B cos q + C`). That, not the held poses, is the gravity to use until the arm carries real payloads.
