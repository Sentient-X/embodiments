# Identifying the static upper body's dynamics

Three experiments, run on the robot by a person at the bench, become one `dynamics` calibration
(`sx_embodiments.calibration.DynamicsCalibration`). The robot instance selects it, and the
station applies it through `sx_embodiment_drivers::Dynamics::of(urdf, UPRIGHT).calibrated(..)`.
The URDF is never edited. Run them in this order:

1. **Friction in place**: constant-velocity sweeps and breakaway ramps, one joint at a time.
2. **Gravity**: twenty held poses.
3. **Inertia**: one excitation trajectory per arm.

Each step's result is useful alone. Friction plus the URDF's gravity already closes most of
the elbows' ±5° band.

## Before any session

- **Who runs it.** A person. Moving the robot stays with a person (CLAUDE.md).
- **The bus is shared.** Message team-lead and wait for an acknowledgement before opening it.
- **Hardware.**
  - The robot is clamped at the table edge and faces outward.
  - The ZLG USBCANFD-200U is on the Mac, serial `A30F115DD0120A949B80`, channel 1.
  - The CAN timeout is 0.75 s: if `sx-identify` dies, the motors disable themselves.
- **Left out.**
  - The torso motor is not in the bench chain.
  - `left_wrist_roll_joint` rests outside its range until its range fix lands. `sx-identify` leaves both out by default.
- **Joint zeros.** The plans hold absolute joint positions, so the drive must carry each unit's zero. Today that is the nominal drive (`drivers/sx-damiao-can/tests/fixtures/drives.json`, `sentient-static-upper-body` / `nominal`), the same one `sx-jog` uses. Re-run the plans once the right forearm calibration lands.
- **Stopping.**
  - Ctrl-C, a refusal, a fault, the governor's 0.2 rad tracking stop, the 80 % peak-torque guard or a suspected contact all disable every motor.
  - A suspected contact is a torque step over 2 N·m, or a tracking-error step over 0.03 rad, between 20 ms ticks.
  - Disabled arms drop. Keep the space under them clear and a hand near the arm being moved.
- **Dry run.** Always run once without `--move`. It prints the duration and segment count and checks every target against the commandable range. Nothing is sent.

Prepare the drive and build the tool once:

```bash
python3 -c "import json; d=json.load(open('drivers/sx-damiao-can/tests/fixtures/drives.json'))['sentient-static-upper-body']['nominal']; json.dump(d, open('/tmp/drive.json','w')); print(d['instance'])"
cargo build --release -p sx-embodiment-commission --bin sx-identify
ID=<the instance printed above>
COMMON="--drive /tmp/drive.json --instance $ID --zlg-serial A30F115DD0120A949B80 --channel 1"
```

## 1. Friction in place (BAM's model family)

**Plan:** `docs/dynamics/static-upper-body-friction-sweeps.json`, from `python tools/identify_dynamics.py sweep-plan`.

Each swept joint has a staging pose. From there its sub-range, up to 1.2 rad, stays at least 5 cm clear of the body and the table along the whole sweep. The plan also checks the path from the ready pose, where every arm joint is 0. Two simplifications make the check conservative:
- Collision uses MuJoCo's convex hulls of the collision meshes, which only overstate the body.
- The table is the whole z = 0 plane of the base, which only overstates it.

**Per joint J** (start with `left_elbow_pitch_joint` and `right_elbow_pitch_joint`):

```bash
target/release/sx-identify --plan docs/dynamics/static-upper-body-friction-sweeps.json \
  --part sweep:J --record sweep-J.csv --speed 0.45 $COMMON            # dry run first
target/release/sx-identify ... --move                                  # ~5 min
target/release/sx-identify --plan docs/dynamics/static-upper-body-friction-sweeps.json \
  --part breakaway:J --record break-J.csv --speed 0.25 $COMMON --move # ~1.5 min
```

**What the sweep run does:**
- Moves from rest to ready to staging at the speed cap.
- Then 3 passes. Each pass goes lower→upper and back at 0.05, 0.1, 0.2, 0.3 and 0.45 rad/s, skipping any speed above `--speed`, with a 0.5 s dwell at each end.
- Returns staging → ready → rest.

**What the breakaway run does:**
- Each direction, 3 times: ramp the target at 0.01 rad/s for 0.08 rad from the middle of the sub-range.
- The onset torque is the torque at the first sample 5 mm-rad (0.005 rad) from the start.

**What to watch:** contact with the table or body. Stop at once with Ctrl-C, report it, and re-run `sweep-plan` with more clearance. Also watch the stop line: `unproven []` means every motor proved it stopped.

**Fit all recorded joints:**

```bash
python tools/identify_dynamics.py fit-friction --recording sweep-L.csv --recording break-L.csv \
  --recording sweep-R.csv --recording break-R.csv --unit A30F115DD0120A949B80:1 --out friction.json
```

**What the fit does:**
- **Trims** 15 % off each end of every sweep, where the target accelerates.
- **Rejects** a segment, with its reason printed, for any of:
  - speed not constant;
  - a torque step over 1.5 N·m, or a tracking step over 0.03 rad, between samples (contact);
  - torque over 80 % of peak;
  - covering less than 90 % of the commanded range (stall).
- **Fits** gravity `A sin q + B cos q + C` together with BAM's models M1 to M4:
  - M1: Coulomb + viscous.
  - M2: + Stribeck.
  - M3: + load.
  - M4: + Stribeck load.
- **Chooses a model** by error on the held-out last pass. A more complex model must win by 5 %.
- **Leaves out M5.** BAM's directional split cannot be separated in place, because during a sweep the drive torque is gravity plus friction.

**Report per joint:** the chosen model, every model's held-out RMS, the forward/backward gravity gap, the parameters, and the segments rejected and why.

**Accept** when:
- the held-out RMS is ≤ 0.3 N·m on the elbows and shoulders, and ≤ 0.1 N·m on the wrists;
- the gravity gap is ≤ 0.3 N·m;
- fewer than a quarter of the segments are rejected.

## 2. Gravity: twenty held poses

**Plan:** `docs/dynamics/static-upper-body-identification-plan.json`, from `python tools/identify_dynamics.py plan`. Its poses are inside every limit less 0.15 rad, collision-free with 2 cm clearance, and chosen to condition gravity's regressor.

```bash
target/release/sx-identify --plan docs/dynamics/static-upper-body-identification-plan.json \
  --part holds --record holds.csv --speed 0.25 $COMMON --move        # ~4 min
```

- Each pose is approached once from 0.05 rad below every joint and once from 0.05 rad above, the final 0.05 rad at 0.1 rad/s.
- Each hold lasts 3 s: 2 s to settle and 1 s recorded.
- The fitter averages the recorded second of each hold, then the two sides. That cancels the friction band (today's elbow error, ±2.5–3 N·m, is this band).
- A joint that moved faster than 0.02 rad/s while recording is printed as a warning.

## 3. Inertia: one excitation per arm

Same plan file:

```bash
target/release/sx-identify --plan ... --part left  --record left.csv  --speed 0.25 $COMMON --move
target/release/sx-identify --plan ... --part right --record right.csv --speed 0.25 $COMMON --move
```

- Each arm swings a 10 s finite Fourier series twice about a held pose, at ≤ 0.25 rad/s and 1 rad/s².
- The other arm holds the plan's posture.
- Each run takes about 40 s.

## Fit and land

```bash
python tools/identify_dynamics.py fit --holds holds.csv --excitation left.csv --excitation right.csv \
  --unit A30F115DD0120A949B80:1 --out dynamics.json
```

`--unit` is required: each is the commissioned identity of a bus the run was recorded on (`<ZLG serial>:<channel>`, as in the robot's `commissioning.json` ports). The station applies a fit only to a robot that has that bus, so a fit from one robot is never fed forward on another.

**What the fit does:**
- Weighted least squares on the base parameters, plus friction.
- Masses and friction are bounded at zero.
- Each link is checked for physical consistency. A link that fails is held to the URDF estimate and named.

**It prints** the base-parameter count, the condition number, and per-joint hold and trajectory residuals against an acceptance bound of 3 % of peak torque (1.2 N·m DM8009, 0.8 N·m DM4340, 0.2 N·m DM4310). A joint marked `OVER` needs its holds re-run.

**To land it:**
1. Merge the friction terms from `friction.json` into `dynamics.json`, one joint each; the sweep fit wins for friction.
2. Add `dynamics.json` to the robot instance's selected calibrations: `RobotInstance.of(body.id, units, (..., calibration))`. `resolve` refuses it for any other description, or for units not placed in the instance.
3. On the station:
   - `Dynamics::of(urdf, UPRIGHT)?.calibrated(&DynamicsCalibration::parse(doc)?)?`
   - then `DamiaoChain::set_feedforward_limits` with each axis' torque bound.
   - then `command_with` each tick.

## Not yet covered

- Gear backlash, torque ripple and thermal drift. BAM's bench procedure identifies them per motor model and is the next model.
- Joint zeros. The plans assume the drive's zeros are true.
