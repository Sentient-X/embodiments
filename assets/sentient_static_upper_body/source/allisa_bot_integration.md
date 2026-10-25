# Current Allisa model — 2026-10-25

Hardware implementation is now available through
config/humanoid_allisa_bot_hardware.json and config/allisa_bot_feetech.json.
The preview model/home/limits are unchanged. Hardware adapts the recorded four
wrist coordinates by sign rather than modifying the accepted axes; the supplied
session-model FK matches at the flange after that conversion. See
[hardware_readiness_20261025.md](hardware_readiness_20261025.md) for software
verification and live-operation limits. No APK change or physical test occurred.

The operator rejected the feedback geometry and selected
`/home/sentientx/Documents/allisa_bot/urdf/allisa_bot.urdf` instead. The active
profile is `config/humanoid_allisa_bot_preview.json`, using
`urdf/allisa_bot.urdf`. The feedback model is a joint-limit donor only; its old
profile/APK/baselines are retained as superseded history.

Source URDF SHA256:
`623305bb43845a70386c0f35d42606574f2b2c1bd4e253c14f4509e62a832ade`.
`config/allisa_bot_model_provenance.json` records source/donor/output hashes,
every transferred joint name and original/derived mesh hashes.

## Geometry and limits

All 33 links, 32 joints, 469 visual instances, origins, axes and part placements
come from allisa_bot. All 25 named movable joints receive the exact lower,
upper, velocity and effort fields from allisa_feedback. Continuous torso/arm
joints become bounded revolute joints so those position limits are enforced.
No world transform or old feedback camera frame is imported.

The four wrist pitch/roll axes have the opposite signs to the feedback model.
They remain exactly as supplied in allisa_bot; requested numeric limits are
copied by name without silently negating them. Hardware signs/zeros still need
evidence. Both profile and arm URDF speed ceilings are 0.5 rad/s. The preview
home remains left [0,0,0,-80,0,0,0], right [0,0,0,+80,0,0,0] degrees. It is a
model pose, not a commissioned motor home.

## Grippers and frames

Both RViz and the app now use the supplied native gripper bodies and fingers,
including their mount and TCP frames. The earlier added Feetech assembly and
nominal adapter are not attached to this profile. Handheld controller-to-grasp
transforms, Feetech USB identities/IDs/endpoints and encoder closed confirmation
are reused. Native left/right finger coordinates are animated from those
existing endpoint fractions using the imported finger limits.

The profile's wrist-to-gripper and wrist-to-grasp transforms are derived from
allisa_bot's fixed mount/TCP joints. This is an explicit adoption of the newly
selected model, not a firmware zero write or physical recalibration. These
model dimensions are not physical commissioning evidence.

RViz freezes uncommanded torso/SO101 joints at their model reference pose in its
display description; the app shows their static reference geometry in chest
coordinates. Canonical URDF limits remain intact. There are still only fourteen
arm command coordinates; no torso/SO101/camera motor control is introduced.

## Display meshes and app

The input has about eight million unique STL triangles. Bundled display STLs
are derived from those same meshes with original units and local frames.
All visual origins/materials are retained. The optional offline regeneration
script is scripts/prepare_allisa_bot_model.py, using an isolated NumPy1.26.4 /
fast-simplification0.1.12 environment. It records triangle counts and bounding
box changes; meshes are retained at full resolution if the reduction cannot
meet the 1 mm bounding-box check. This check does not certify every surface
point. Normal builds and operation use the generated files and need no mesh
simplifier or external allisa_bot package.

The app packs repeated meshes once and shares GPU buffers while retaining every
part's transform and material. v0.5-allisa-bot uses the native gripper animation
groups with the existing ten-pose transport. URDF/mount signatures require the
matching app assets. v0.4 contains superseded geometry and must be updated before
testing this profile. Use the [current v0.6 release](releases/pico_humanoid_viewer/README.md);
its startup fix was installed and visible robot rendering confirmed by the
operator. Precision optical alignment and physical commissioning remain unverified.

The native app hides moving finger groups until the first valid jaw sample. Missing encoder input can therefore hide fingers in RViz preview; it does not explain a missing fixed gripper body when the arm model is available. Preview jaw animation uses confirmed `/umi_left/right/jaw_state`; hardware uses measured Feetech fractions. After a valid sample, stale jaw feedback retains the last finger pose with stale status. RViz can show reference fingers without jaw feedback. No missing-input fallback change was implemented.

## Limits and IK comparison

The immediately preceding seven-joint feedback model and current allisa_bot
have identical numeric arm lower/upper/velocity/effort limits. The four wrist
axes differ as documented above; numeric equality is not a physical motor-sign
calibration. Compared with the original five-joint profile:

| Joint/function | Original five-joint left/right (degrees) | Current left (degrees) | Current right (degrees) |
|---|---|---|---|
| Shoulder pitch | Both -180..180 | -151.41..165.58 | -166.95..178.91 |
| Shoulder roll | L -20.45..180; R -180..20.45 | -22.20..68.86 | -69.33..20.32 |
| Upper-arm axial rotation | Both -90..90 (old elbow_yaw) | -293.53..46.24 | -138.72..158.12 |
| Elbow bend | L 0..90; R -90..0 | -170.47..0.05 | -1.67..169.97 |
| Forearm yaw | No separate coordinate | -391.14..107.94 | -344.32..81.71 |
| Wrist pitch | No separate coordinate | -121.93..139.85 | -132.29..129.25 |
| Wrist roll | No separate coordinate | -120.09..120.14 | -121.01..120.25 |

The old wrist_yaw coordinate was -90..90 on both sides; it is not a direct
one-to-one replacement for the new forearm/wrist axes. The table compares
functions and does not establish an anatomical motor mapping.

The original five-joint IK speed ceiling was 1 rad/s. The rejected feedback
model already limited IK to 0.5 through the URDF despite its profile's global
1 rad/s setting. Both URDF and profile now say 0.5, so the gateway agrees too.
Acceleration8 rad/s², control90 Hz, pose_gain8, lead20 degrees, freshness300 ms
and pickup75 mm/20 degrees/200 ms were retained. New arm effort fields are54 Nm
versus the old mostly10 Nm (old right elbow140 Nm); these URDF fields do not
establish hardware current limits or a controller tune.

The solver remains position-priority with subordinate orientation/posture
cost, the same damping/weights and previous-state warm start. In
humanoid_teleop/continuous_ik.py, the initial joint-count adaptation replaced
two fixed five-element zero arrays with zeros_like(q). Integration now accepts
seven named coordinates, loads the selected axes/limits/flange, and controls
the selected native TCP. UrdfKinematics FK/Jacobian mathematics is unchanged.
Those changed model inputs affect IK results without a new solver, PID loop or
gain tuning. The app's v0.6 lifecycle fix changes none of these control inputs.

### Wrist rotation correction — 2026-10-25

Offline analysis reproduced an IK stall with the actual allisa_bot chains/TCPs.
The original primary step cost penalized joint displacement linearly. It could
prefer zero motion with a small position residual still above 1 mm. The
secondary orientation acceptance rule then rejected rotation at that residual.
The interaction could suppress wrist motion even for an FK-generated reachable
target; this is separate from a missing jaw encoder.

For seven-joint chains only, the primary step cost now penalizes squared joint
displacement instead. Position can converge through that dead zone, allowing
the existing secondary orientation solve to run. Five-joint behavior, joint
bounds, speed/acceleration/lead, axes, mounts, home, controller conversion and
the secondary position-preservation checks are retained. No direct controller-
to-wrist joint mapping, new solver, PID change or APK update is introduced.

At 90 Hz over four simulated seconds with immediate simulated feedback, a 30°
local-X orientation request plus 2 mm vertical displacement previously retained
about 30° orientation error and 1.24 mm position error on both arms. After the
correction, orientation errors were about 0.005° left / 0.011° right, with
position errors below 0.001 mm. A reachable right wrist-pitch target generated
by adding 0.3 rad to joint 6 previously retained 10.6° / 1.63 mm error; the
corrected solve reached about 0.001° and below 0.001 mm. These are offline
results, not measured robot accuracy or proof of the operator's exact earlier
failure. No recording of that earlier trial was inspected.

The home Jacobian is full row rank in the offline check but has weak directions;
large pitch/yaw requests can still converge slowly or encounter joint limits.
Orientation is subordinate to position. Rotation about a handheld controller
also moves the requested grasp TCP because its calibrated mount has a nonzero
offset, so coupled arm motion can be needed; the wrist is not a three-axis
spherical wrist. Normal teleoperation must show Following with fresh controller
poses. The separately opened all-zero inspection window is intentionally static
and cannot follow Yubi motion. Jaw encoders affect fingers, not arm orientation.

The host Python module is symlink-installed in this checkout; a new normal
teleoperation process loads the correction. Existing processes retain their
loaded code until the operator relaunches them. Do not restart active processes
or change the configured home automatically. The immutable v0.6 APK/source
archive remains unchanged; its viewer code does not perform the host IK solve.

## Normal preview

Keep existing stock tracking, adapter, pedals and confirmed encoder readers.
Stop only the previous preview launch, then use the rebuilt package:

```bash
cd /home/sentientx/humanoid_pkg
source /opt/ros/humble/setup.bash
source install/yubi_absolute/setup.bash
export PYTHONNOUSERSITE=1
export HUMANOID_PYTHON="$PWD/.venv-humanoid/bin/python"
ros2 launch humanoid_pkg yubi_absolute_rviz.launch.py \
  profile:="$PWD/config/humanoid_allisa_bot_preview.json" use_headset_viewer:=true
```

Preview starts at the model home. Press/release B for the new session, align each
Yubi, then A engages left or C right; press again to disengage. Physical output
remains blocked because the new hardware motor map is empty. This update does
not establish motor identity, zero/direction, gearing or a validated MIT tune.
