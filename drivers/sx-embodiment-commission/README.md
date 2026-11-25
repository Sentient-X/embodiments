# sx-embodiment-commission

The way a robot is commissioned: find every bridge on the host and what answers on each, then
match each port a body declares to its one bridge. `probe` and `commission` only read. Every
scan speaks through its vendor crate's read-only port, which refuses any other packet or frame
before the link sees a byte. Within `sx-commission`, the subcommand that can write a register or move a joint is
`calibrate` ([below](#calibrating-a-feetech-bus)), and only under `--move`. The
crate also supplies the supervised bench tools, including local `sx-teach` below.

```text
sx-commission probe [--json] [--feetech-ids 1-20] [--damiao-ids 1-32]
sx-commission commission --bindings <file> [--bindings <file>...] \
  [--instance <id> --pod <id> --revision <n>] [--json] [--write <dir>]
```

`probe` lists every USB device with its vendor and product id, serial, tty or `SocketCAN`
interfaces, and whether this host can open it. It then scans each Feetech-capable serial bridge
(WCH, FTDI and Silicon Labs chips, at 1 Mbaud) and each CAN interface. `commission` also
matches the body's ports. By default it prints the `sx.station-commissioning/v1` document and
each Feetech port's `LeRobot` calibration. It writes `commissioning.json` and
`<port>.calibration.json` only with `--write`. It exits 1 when any port is refused.

| Item | Path |
|---|---|
| Host seam | `host::Host` (USB devices, ttys, CAN interfaces, open serial and CAN); `host::System` is this machine: `nusb`, `serialport`, sysfs and `sx-socketcan` on Linux |
| Enumeration | `bridges::enumerate(host)` gives one `Bridge` per non-hub USB device, with a `BridgeKind` (serial bus, CAN bus, camera, input, other) and an `Access` (`Tty`, `SocketCan`, `NoDriver { reason }`, `None`). A ZLG USBCANFD-200U on macOS is `NoDriver` |
| Kernel view | `sysfs::can_interfaces(net)` (type 280 links with their USB adapter's serial) and `sysfs::by_id(dir)` (stable tty aliases) |
| Probe | `probe::probe(host, &Ids)` returns a `Probe` with each `FeetechBus` (`sx_feetech_serial::scan`) and `DamiaoBus` (`sx_damiao_can::scan`) |
| Declared ports | `declared::DeclaredBody::parse(&[documents])` reads the rendered bindings' `port`. Without one, a Damiao axis is on `can` and other axes on a port named for their bus |
| Matching | `matching::commission(&body, &probe)` gives one `PortMatch` per port, or every `PortError` (`Missing`, `Ambiguous`, `WrongModel`, `Unqualified`) naming the candidates |
| Output | `output::commissioning(&Station, &matches)`, `output::calibration(&match)` (`LeRobot` JSON keyed by joint), `output::warnings` (a USB serial carried by several CAN channels), `probe_report`/`probe_json` |

One process owns a bus, and the layer enforces it: every tool here opens a bus through the
embodiment layer's exclusive openers (`TtyPort::open` is `sx_embodiment_bodies::usb_serial::open_exclusive`;
bodies open through `system::SystemPorts`; ZLG adapters through `UsbCanFd::open`), so a bus
another process holds is refused before a byte is sent, as `OpenError::Busy` (or an `io::Error`
of kind `ResourceBusy`, or `ZlgError::Busy`) naming the holding pid where the host tells it.
`probe` records that refusal against the bus instead of scanning it. Checking `ps` before a run
is a courtesy, not the guard.

A bus fits a port when every unit the port declares answers as its model: Feetech by model
number, Damiao by the `PMAX`/`TMAX` row. A bus whose answering ids are exactly the port's wins.
A bus that also carries other ids fits only when no exact bus does, and never when it is
exactly another port's bus. That is how a gripper on ids 1 and 2 is never taken from the neck's
bus, which also answers 1 and 2.

## Known gap

The ZLG USBCANFD-200U shows both of its channels under one USB serial. Commissioning matches
the right channel (`can1`) and records the serial, but `sx-embodiment-bodies`' `SocketCanPorts`
resolves that serial to both interfaces and refuses it as ambiguous. `output::warnings` says so
until a commissioning identity can name a channel.

## Calibrating a Feetech bus

`sx-commission calibrate` replaces one-off scripts for STS3215 buses. Every run is repeatable
and leaves the bus as it found it.

```text
sx-commission calibrate read --bus <tty> [--ids 1-20] [--json]
sx-commission calibrate gripper --bus <tty> --ids 1,2 [--move] [--widen <ticks>] \
  [--closed 1:low,2:high] [--json]
sx-commission calibrate jog --bus <tty> --id <n> --delta <ticks> [--move] [--json]
sx-commission calibrate goto --bus <tty> --id <n> --to <ticks> [--widen <ticks>] [--move]
sx-commission calibrate apply --bus <tty> --calibration <file> --move
sx-commission calibrate --restore <journal>
```

- `read` reports each servo's registers as `probe` does. It adds the control registers
  (`Goal_Position`, `Goal_Speed`, `Acceleration`, `Torque_Limit`, `Lock`), `Present_Load`, and
  the position in `LeRobot`'s degrees and percent. It only reads.
- `gripper` sweeps each finger in turn, slowly, down to its low end and up to its high end.
  Each step sets the goal `--lead` ticks (default 20) past where the servo is. A sweep ends at
  the first of these:
  - `Present_Load` at or above `--load-threshold` (150, out of 1000);
  - no progress of `--min-progress` ticks within `--stall-ms`;
  - the position limit;
  - `--max-travel`.

  At a stall it backs off `--backoff` ticks (30) in the same step, so it never keeps pushing
  against a stop. It then brings the finger back to where it started and turns torque off.
  The result per finger is both ends with why each ended, and a `LeRobot` calibration
  (`range_min`/`range_max` inset `--inset` ticks from the stops, `homing_offset` as the servo
  holds it). `--closed id:low|high` says which end closes; `drive_mode` is then 1 when the high
  end closes, so 0 percent is closed. The calibration is printed, never written to the servo.
  The probed grippers' limits are about 600 ticks wide, so the stops may lie past them:
  `--widen <ticks>` widens the limits for the sweep and restores them after.
- `apply` makes a calibration's `range_min`/`range_max` each servo's own position limits, kept
  across power cycles (`Lock` 0, both words, `Lock` 1), with every servo limp. It takes
  `gripper --json`'s output or a `LeRobot` calibration keyed by joint, prints the limits it
  replaced, and only `Lock` is put back on close. The servo answers late while it stores an
  EEPROM word, so a write whose status packet times out is judged by reading the word back.
  `FeetechBody::open` refuses a servo whose limits disagree with its calibration, so a measured
  gripper range is unusable until it is applied. On 2026-10-26 the static upper body's grippers
  went from about 53 degrees of commandable stroke, past which every finger overloaded, to their
  full measured stroke (about 40 degrees open and back, no fault).
- `jog` powers one joint where it rests and moves it `--delta` ticks (at most 400), then back,
  and turns torque off. It prints each position and the measured ticks per second at the given
  `--goal-speed`. `--hold-seconds <s>` holds it at the target first, printing `HOLDING id N at
  <ticks>` when the hold starts and `RETURNED` once it is back and limp; `--json` gives the hold's
  wall-clock start and end and the positions read during it. `--widen` works as for `gripper`.
  That measurement settled the `Goal_Speed` unit on 2026-10-26: steps per second, with the
  firmware never slower than 50 (the default `--goal-speed 20` runs at 50 steps/s, about
  0.077 rad/s); see `sx-feetech-serial`'s README.
- `range --ids 1-6 --joints <names> [--full-turn <ids>] --out <file>` is a hand-guided
  calibration that writes nothing (every packet through the read-only port, no journal). It
  refuses unless every listed servo reads `Torque_Enable` 0. The person puts every joint at
  its middle and presses Enter: the homing offset that makes that pose read 2047 (`LeRobot`'s
  `int(4095 / 2)`) is computed from `raw = (present + held offset) mod 4096`, not written.
  Then the person moves each joint through its full travel while a live line per joint shows
  now/min/max in ticks and degrees (about 50 Hz), all in the new frame; Enter ends it and the
  `LeRobot` calibration keyed by joint is written to `--out`. `apply --move` then writes each
  entry's `homing_offset` and limits under `Lock` 0 → 1, refusing a servo that holds torque;
  `--protection neck-defaults` also resets `Max_Torque_Limit`/`Protection_Current`/
  `Overload_Torque` (and the live `Torque_Limit`) to 1000/310/80 where a servo differs,
  printing each change first.
- A sweep's end is one of `Load`, `NoProgress` (a stop), `Stuck` (it never moved a lead's
  worth, so it proves no stop), `Limit` (its position limit, or a stall within a lead of it,
  where the goal is clamped) and `MaxTravel`. A move that stops short without load holds where
  it is; only a load stall backs off. A missed status packet is read again up to three times.
  `--trace <file>` writes one JSON line per read, goal, enable and torque-off.
- `goto` powers one servo, moves it to `--to` ticks and leaves it there with torque off;
  its gears hold it. With one servo powered at a time, it parks the opposite finger at its
  closed position, so a sweep's closed end is where the fingers meet. Do not start a sweep
  with the finger resting on a stop: it moves less than a lead and reports `Stuck`.
- Without `--move`, `gripper`, `jog` and `goto` are dry runs. They read through the scan's read-only
  port and print the plan. They create no session and no journal, so they cannot write.

How it keeps a servo from being left powered (`src/calibrate/`):

| Guarantee | How |
|---|---|
| Every change is saved before it is made | `journal::Journal`, one per bus, at `$XDG_STATE_HOME/sx-commission/calibrate/<tty>.json` (`--journal-dir` to override). Each register's value before this run's first change, and each servo it may have powered, is written, synced and renamed into place before the bus sees the change. A run refuses to start while an earlier run's journal is still there |
| Enabling holds still and slow | `Session::enable` writes `Goal_Position` = `Present_Position` (the probed grippers read `Goal_Position` 0). It writes `Goal_Speed`, `Acceleration` and `Torque_Limit` (defaults 20, 5, 250/1000) and reads each back, refusing on any difference, before `Torque_Enable` 1, which is read back too. It refuses a servo resting outside its limits, which the firmware would drive to the nearest limit |
| One servo powered at a time | `Session::enable` refuses a second (`CalibrateError::OnePowered`); there is no override |
| Undone on every way out | `Session::close` on a return or an error, and `Drop` on a panic's unwinding. Each touched servo gets `Torque_Enable` 0, read back. Then each saved register is written back in reverse order and read back. A servo whose torque-off is not proven keeps its lowered `Torque_Limit`; the run prints `DANGER: servo N may still hold torque`, exits 3 and keeps the journal |
| Signals | SIGINT, SIGTERM, SIGHUP and SIGQUIT only set a flag (`signal_hook::flag`). Every motion step checks it and stops; no wait goes more than 20 ms (a packet timeout) or 5 ms (a sleep slice) between checks. SIGKILL cannot be caught. Its journal stays, the next run refuses, and `--restore <journal>` disables and restores what it names |
| A deadline | `--max-seconds` (180) ends the run the same way |
| EEPROM | `--widen` saves `Lock` and both limits, then sets `Lock` 1 before writing them. Feetech's memory table describes a write under `Lock` 1 as not kept across a power cycle; this firmware has not been checked. The journal restores them either way |

Exit codes: 0 done; 1 stopped or refused, with everything undone; 2 bad arguments; 3
something may still be powered or changed (see the printed report and the kept journal).

The registers are `sms_sts.py`'s. Where it does not name one, the source is Feetech's C++
library (`FTServo_Arduino` at `64922cda`, `src/SMS_STS.h`, `src/SMS_STS.cpp`): `Torque_Limit`
is 48, and `Present_Load` (60) is sign-magnitude at bit 10. A stall is judged on the load's
magnitude, so its sign convention does not matter.

**Not yet:** `jog --zed`, ZED frames captured before and after a nudge with the image shift
measured by phase correlation. A jog's `out` and `back` arrivals in `motion::jog` are where
the captures would go. `sx-uvc-stereo`'s `capture` feature opens the camera; phase
correlation needs an FFT this workspace does not carry yet.

Tests: `tests/calibrate.rs` drives each path on `sx-feetech-serial`'s simulated bus (`sim`),
which models hard stops with `Present_Load` and `Torque_Limit`. The paths are a sweep that
stalls, backs off and comes back; limits widened and restored; a deadline and a signal
mid-sweep; a panic and a dropped session; a torque-off that does not read back, then
`--restore`; a `Goal_Speed` that does not read back; one servo at a time; a dry run that sends
only reads; and a stale journal. `tests/calibrate_signal.rs` runs the binary on a
pseudo-terminal whose other end is the simulated servo. It sends SIGTERM, SIGINT and SIGHUP
while the servo is powered and checks that torque is off, the registers are restored, the
journal is gone and the exit code is 1.

## Bench tools

Extend these; never write a new vendor-frame script. A bench tool opens or observes the body
through the embodiment layer (`sx-embodiment-bodies`: `open_with` to drive, `observe` to read
without writing), from the instance's drive and the station's commissioning. A faulty motor is
named in the commissioning's `out_of_service`, never skipped in a tool.

| Binary | For | Through the layer? |
|---|---|---|
| `sx-commission probe` / `commission` / `calibrate` | find bridges and ids read-only, write `commissioning.json` and calibrations, calibrate a Feetech bus | yes (vendor scan modules behind read-only ports; `calibrate` writes only under `--move`) |
| `sx-jog` | supervised relative jog and sequences of any body, `--exclude` joints for one run, `--log` every tick (target, the governor's slewed command, observed, each motor's q/dq/tau/temperatures/state) | yes: `open_with` + `Governor`; the log is written on its own thread from fixed-size records |
| `sx-record` | record every joint, read-only, while a person moves the body by hand; the live twin tails it | yes: `observe`, each bus on its own thread, one JSON line per poll in joint coordinates |
| `sx-can-bench` (`sx-zlg-usbcanfd`) | ZLG USBCANFD link throughput and latency | transport bench, below the layer by design |
| `sx-imu-still` (`sx-stereolabs-sensors`) | ZED IMU stillness check | sensor bench, no actuators |

Static upper body on the bench: the ZLG channel is commissioned as
`A30F115DD0120A949B80:1` (port `can`), the neck CH343 as `5A7C118709` (ids 1-6), the right
gripper as `5B91028463` (ids 1, 2) and the left as `5AB0158173` (ids 3, 4):

```json
{"schema": "sx.station-commissioning/v1", "instance": "<instance id>", "pod_id": "<pod>",
 "pod_revision": 1, "out_of_service": ["torso_joint"],
 "ports": {"can": "A30F115DD0120A949B80:1", "neck": "5A7C118709",
           "right_gripper": "5B91028463", "left_gripper": "5AB0158173"}}
```

`sx-jog` takes the robot's files as a station does:

```text
sx-jog --drive <drive.json> --commissioning <commissioning.json>
       [--calibration <port>=<lerobot.json>]... [--allow-provisional <port>]...
       [--exclude <joint,...>] [--speed 0.2] [--log <ticks.jsonl>]
       [--at-end home|start|release]
       (--joint <joint> --delta-deg <deg> [--hold-seconds <s>] | --sequence <joint:deg|pause:s,...>)
```

When the moves are done the body ends as `--at-end` says (`sx_embodiment_drivers::AtEnd`,
`Governor::end`). `home`, the default, returns every joint to its registry home
(`ChainSpec.home`, rendered into the bindings, where the calibrated zeros put it; a joint whose
part declares none, a gripper's finger, stays where it is) at half the speed cap,
tracking-checked, within a deadline, then disables torque. `start` returns to where the body
rested when it was opened; `release` disables in place. Nothing leaves torque on after the run: a
Damiao motor's armed `TIMEOUT` would drop it 0.75 s later with fault 0xD. A body nothing drove
since it was opened or last stopped is never moved, only stopped. A return refuses to start
while a fault is latched or any joint does not answer a fresh read, and a return that cannot
finish falls back to the stop where the body is; the run then says why and exits non-zero, as it
does for a stop that is not proven. A refusal, a fault or Ctrl-C at any time stops the body
where it is, with no motion.

A step written `&joint:deg` moves with the step after it, so a pose reached by several joints
(an IK answer) is one coordinated move: put the largest move last, since it sets the duration.
`sx-embodiment-ik`'s `look_at` example turns measured neck joints into such a sequence's targets:
the ZED looking ahead, at each gripper and at the table between them.

Its old vendor-frame modes are gone, each covered by the layer: `--scan` is `sx-commission
probe`; `--clear-faults` is the open's own settle phase, which clears a latched 0xD once;
`--disable-all` is `--at-end release`; `--slew-to` and `--release-test` are a move and
a pause in a sequence. `sx-timeout-probe` is gone: the watchdog is armed right before every
enable, and `sx-damiao-can`'s simulated bus holds the phased bring-up to it.

`sx-record` takes the same body flags and only reads:

```text
sx-record --drive <drive.json> --commissioning <commissioning.json>
          [--calibration <port>=<lerobot.json>]... [--allow-provisional <port>]...
          [--exclude <joint,...>] --out <file.jsonl> [--seconds 60] [--rate 50]
```

Each line is one bus's poll: `{"t", "bus", "positions": {joint: value or null}, "missing":
{joint: why}}`, the joint values through the drive and the units' calibration (a serial servo
with none supplied is read with the calibration its registers hold). Observing a provisional
port writes nothing, so `--allow-provisional` needs no supervision there.

`tests/bench_layering.rs` refuses a bench binary that names a vendor crate, a CAN or serial
port or a USB stack. `sx-identify` is the one still listed, until it moves onto `open_with`
and `observe`.
## Local keyboard teaching (`sx-teach`)

`sx-teach` uses local robot files, the embodiment drivers and an attached
terminal on the robot host; an SSH PTY also works. It requires no account or
Fleet session, and direct host operation needs no network. It records measured
joint and gripper trajectories to local JSONL files, without camera video.
In Teach, active grippers release torque for manual guidance while arm gravity
support continues. For keyboard gripper actions during a take, press `h` to hold
while recording continues, use `-`/`+`, then `t` to resume hand guidance.
Other active noncompliant axes use normal governed position control.

Build from the embodiments workspace:

```bash
cargo build --release -p sx-embodiment-commission --bin sx-teach
```

Supply the robot's existing drive, commissioning, URDF and identified dynamics
files. Serial calibrations beside commissioning load automatically, as they do
for the other bench tools. `--dynamics` may be repeated for disjoint calibrations;
`--held joint=rad` explicitly supplies measured, mechanically fixed positions for
unobserved model joints. An out-of-service joint with live encoder feedback can
instead contribute its measured position to gravity compensation without being
commanded; no fixed reference is required. Passive feedback is required unless
the dynamics model proves a root rotation parallel to gravity,
whose angle is irrelevant to static gravity compensation. Such an internal
placeholder is not recorded as a measured joint or used to prove world poses.
Teaching joints must support compliant control. The following paths and joint
names illustrate the invocation; replace them with the exact files/names for the
physical robot:

```bash
./target/release/sx-teach \
  --drive /path/to/robot/drive.json \
  --commissioning /path/to/robot/commissioning.json \
  --urdf /path/to/robot/body.urdf \
  --dynamics /path/to/robot/identified.dynamics.json \
  --joints left_shoulder_pitch_joint,left_shoulder_roll_joint,left_upper_arm_yaw_joint,left_elbow_pitch_joint,left_forearm_yaw_joint,left_wrist_pitch_joint,left_wrist_roll_joint \
  --out-dir /path/to/recordings
```

Without `--move`, startup parses the local files and opens the read-only observer;
no motor is enabled. With an attending operator, supported arms, verified supply
and free workspace, add `--move`. The same startup checks run; no separate
preview session is needed by default. Defaults are `--speed 0.2`, `--share 0.25`,
`--damping 0.5` and `--max-seconds 3600`. The intended control period is 10 ms;
physical full-arm loop timing and gravity compensation still require bench
qualification before claiming sustained 100 Hz operation.

`--speed` caps positioning and approach motion. Playback uses the recorded
timestamps at 1×, with the Governor's validated actuator, share, acceleration,
tracking and device limits still bounding robot commands. Physically infeasible
motion can lag or fault; 1× timing is not a guarantee of perfect tracking.
Observed speed is checked against actuator ratings and device derates.

Replay arrival uses a 0.05 rad joint-position tolerance held for 300 ms. Axes
that report native velocity must also return fresh feedback below 0.05 rad/s;
position-only servos are checked by position, without inferring a velocity from
coarse encoder steps. The approach timeout includes travel at the effective
speed and acceleration limits plus 30 seconds to settle. Arrival tolerance
does not change joint limits or improve the underlying tracking accuracy.

Engagement prepares the motors with zero arm torque and positive damping, then
reads a fresh pose before applying position hold and ramping gravity support.
The one-time activation transaction has a separate one-second budget. Fresh
reads and subsequent control cycles retain their 50 ms deadline. Keep the arms
physically supported through preparation and the gravity ramp.

Pressing `t` in Hold enters hand guidance immediately. It does not wait for
steady readings or require another keypress. An optional support correction is
used if suitable Hold data is already available; otherwise the current gravity
compensation is retained. Calibration files are unchanged. Effort, physical
speed, range and feedback-freshness checks remain active.

| Key | Action |
| --- | --- |
| `e` | Engage a hold at the measured position; keep supporting the arm during the one-second gravity ramp |
| `t` | Enter hand guidance immediately from Hold |
| `r` | Start recording during teaching; press again to hold and save |
| `h` | Hold the current position; recording continues |
| `p` | Approach the selected recording's start; press again when ready to play |
| `-` / `+` | Adjust grippers in hold; record the change if a take is active |
| Space, `q` | Stop into a fault hold; never release torque or home |
| Ctrl-C | Stop, release torque and exit; physically support both arms before pressing |
| `x` | Release torque and exit from hold/fault, after physically supporting arms |

The tool reports when the recording has finished writing and is ready for replay.
During creation/finalization it refuses a new teaching/replay operation. The last
successfully saved recording becomes the selected replay. To replay after a tool
restart, provide `--replay /path/to/recordings/episode-....jsonl` with the same
configuration. Replay follows the recorded timestamps at 1×; governed commands
retain their speed, acceleration and tracking bounds. Replay refuses a different configuration/joint order, incomplete
files and malformed/non-finite measurements. Recordings use unique names and
never replace existing files. An interrupted take is retained without a completion
footer and cannot be replayed.

A bounded writer queue keeps disk operations off the control thread. Queue
saturation or disk failure invalidates the take and stops teaching into hold.
Terminal EOF/disconnection and SIGTERM/SIGHUP/SIGQUIT also stop into hold;
the process keeps refreshing holds. SIGINT/Ctrl-C explicitly stops and releases
torque, so support both arms before using it.
Keyboard Space is a software stop, not a substitute for the physical emergency
stop. On a process crash or power loss, software cannot keep a brakeless arm up.

Before starting, hand bus ownership over from any resident station or bench tool
only after the arms are physically supported or parked. Never kill a station that
is their only support. The local tool refuses a bus already held by another
process. Physical operation remains subject to attending-operator authorization;
a successful compile or simulated test does not qualify hand contact or replay.

The local controller checks joint ranges, speeds, torque, feedback freshness and
control timing. It does not yet check Cartesian workspace clearance or robot
self-collision against a scene. This is a software candidate for attended bench
qualification, not a physically qualified teach mode.

Startup reads the configuration into one immutable snapshot. The digest saved in
recordings covers the same drive, commissioning, serial calibration, dynamics and
URDF bytes used by the hardware opener. Replacing a source file while the operator
reads the instructions does not change that running snapshot; restart to load a
new configuration. Explicit serial calibration files override automatic files for
their ports. Selected joints marked unidentified are refused from this snapshot.

For dynamics identified against an earlier description, startup looks for the
exact earlier URDF at `<URDF directory>/_by_digest/<description digest>/<URDF filename>`.
It verifies those bytes against the calibration's digest, then uses the existing
embodiment dynamics compatibility check before accepting the fit. Missing,
changed or incompatible historical descriptions are refused; no calibration is
converted or re-keyed. Accepted historical URDF bytes also enter the recording's
configuration digest.

Space, Ctrl-C and `q` preempt incomplete terminal escape sequences: pressing Escape
or receiving a partial arrow-key sequence cannot swallow a subsequent stop/exit key.
Ctrl-C (raw terminal byte or SIGINT) is an explicit operator release request from
any mode, including recording and replay. It first runs the existing motion-stop
path, then disables actuators and exits only after release is proven. An active
take interrupted this way remains incomplete. If disabling cannot be confirmed,
the tool reports the failure and stays in Fault for an explicit retry.
Terminal loss, SIGTERM, SIGHUP and SIGQUIT still request hold. Motor faults retain
their existing per-axis safety handling. The launcher uses `exec`,
so this is handled in the controller rather than a shell kill trap.

In Teach, active Feetech grippers have torque disabled so the operator can move
fingers by hand while arm gravity support continues. Fresh measured finger
positions remain part of every recording sample. Hold captures those current
positions before restoring powered gripper control; replay uses the recorded
finger trajectory. Release is supported only for complete gripper-only Feetech
chains; unsupported hardware is refused before engagement. Joint ranges and
feedback freshness still apply, including the calibrated gripper endpoints.
An object held only by motor force may drop when entering Teach.

In Hold, `-` closes and `+` opens both available grippers. Finger angles come from
the embodiment's canonical geometric gripper mapping, including opposite joint
signs on mirrored fingers. Startup selects the embedded render pack only when its
URDF SHA matches the supplied description exactly, maps its finger names into the
hardware drive order, and includes the pack/collision bytes in recording identity.
Without a matching canonical mapping the arm can still be taught, but gripper
keyboard actions are refused; the tool never guesses finger directions.

To investigate a concrete local configuration parse failure without touching any
bus, append `--check-config` to the usual invocation. This optional file-only
check reads, parses and hashes the same immutable input snapshot, including
historical description and calibration compatibility. It returns before opening
an observer, actuator, terminal or recording directory, even if `--move` is also
present. It does not establish live joint feedback, floating stability or motion
readiness. Ordinary startup still runs its usual checks; a separate check-config
session is not required for normal operation.

An optional `--guidance FILE` selects a separate `sx.guidance/v1` hand-guidance
profile. Its affine coordinate mapping, gravity-only model, per-joint gravity
scale, damping, torque cap and slew rate apply only in Teach. Entering and leaving
Teach blends support over two seconds, without a settled-readings gate. Hold,
Approach and Replay keep the original `--dynamics` model and position gains;
the platform station does not read this file. `--damping` remains the legacy
common Teach gain only when no guidance profile is supplied.
The profile's torque cap bounds every Teach support command. If inherited Hold
support is outside that cap, the cap takes precedence over the soft transition
slew bound. Held joints outside the selected set keep receiving updated support
from the original model as the guided joints move.

The profile binds exact physical description and drive bytes; it never rewrites
encoder zeros, signs, limits or the collision model. Its lumped wrist payloads
occur once inside the guidance-only model. Rotational inertias in that model are
numerical regularization for point masses, not identified dynamic parameters;
only static gravity is evaluated. `friction_coulomb` preserves handoff values,
but `friction_scale` must be zero in v1. Enabling friction requires an explicit
controller implementation, rather than making that field appear effective.

For the November 23 Allisa handoff, `tools/import_allisa_guidance.py` in the
embodiments root generates a profile and reference torque fixtures without
opening a socket. It uses the handoff's recommended left J4 multiplier 0.70,
left J2 default 1.00 and tested right baseline, not the optional fitted profiles.
`guidance_handoff` compares 256 matching encoder poses against the supplied
Python controller, including motor signs, payload, gravity scales and caps.

Guidance bytes are part of the recording configuration identity. Recordings made
before this profile was enabled require the old invocation without `--guidance`;
do not rewrite episode headers to force them through a changed configuration.
