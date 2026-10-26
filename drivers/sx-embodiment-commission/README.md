# sx-embodiment-commission

The way a robot is commissioned: find every bridge on the host and what answers on each, then
match each port a body declares to its one bridge. `probe` and `commission` only read. Every
scan speaks through its vendor crate's read-only port, which refuses any other packet or frame
before the link sees a byte. The one part that can write a register or move a joint is
`calibrate` ([below](#calibrating-a-feetech-bus)), and only under `--move`.

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