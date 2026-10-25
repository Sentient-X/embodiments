# sx-embodiment-commission

The way a robot is commissioned: find every bridge on the host and what answers on each, then
match each port a body declares to its one bridge. `probe` and `commission` only read. Every
scan speaks through its vendor crate's read-only port, which refuses any other packet or frame
before the link sees a byte. The one part that can write a register or move a joint is
`calibrate` ([below](#calibrating-a-feetech-bus)), and only under `--move`.

```bash
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

```bash
sx-commission calibrate read --bus <tty> [--ids 1-20] [--json]
sx-commission calibrate gripper --bus <tty> --ids 1,2 [--move] [--widen <ticks>] \
  [--closed 1:low,2:high] [--json]
sx-commission calibrate jog --bus <tty> --id <n> --delta <ticks> [--move] [--json]
sx-commission calibrate goto --bus <tty> --id <n> --to <ticks> [--widen <ticks>] [--move]
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
- `jog` powers one joint where it rests and moves it `--delta` ticks (at most 400), then back,
  and turns torque off. It prints each position and the measured ticks per second at the given
  `--goal-speed`. `--hold-seconds <s>` holds it at the target first, printing `HOLDING id N at
  <ticks>` when the hold starts and `RETURNED` once it is back and limp; `--json` gives the hold's
  wall-clock start and end and the positions read during it. `--widen` works as for `gripper`.
  That measurement settles the `Goal_Speed` unit, which
  `sx-feetech-serial`'s README records as unverified.
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
