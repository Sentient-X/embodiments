# sx-feetech-serial

Feetech STS servos over their half-duplex serial bus, translated from Feetech's own
`scservo_sdk` (<https://github.com/ftservo/FTServo_Python> at `cbcfa646`, `scservo_sdk/`; MIT,
Copyright (c) 2024 ftservo — see `LICENSE-THIRD-PARTY`), and the SO-101 follower's chain of six
STS3215s. Every departure from `scservo_sdk` is listed in the header of the module that makes
it.

Consumer: the `sx` station's `chain::ChainActuator`, over `so101::open_so101`. The station opens
it on an SO-101's serial adapter once a serial device binding exists there.

## The surface the station builds against

| Item | Path |
|---|---|
| The chain trait | `sx_embodiment_drivers::ActuatorChain` (`axes`, `commandable`, `command`, `stop`, `stop_budget`, `clear_faults`); `FeetechChainError` implements `ChainError` |
| A command's evidence | `sx_embodiment_drivers::Receipt<()>`: always `DispatchAttempted`, since the SYNC WRITE has no answer; its `observed` is one SYNC READ of every servo's `Present_Position` right after it (`GroupSyncRead`, `group_sync_read.py`), in native state order, `NaN` for a servo that missed it so a tracking check fails closed. The read covers `Torque_Enable` through `Present_Position`, so a servo found limp, or no longer holding its declared approach limit, is `NaN` too and the next command enables afresh. On the line at 1 Mbaud a command is 184 bytes for six servos (1.84 ms) and 72 for two (720 µs), before adapter latency |
| A stop's proof | `sx_embodiment_drivers::StopReport`: a servo is unproven unless its `Torque_Enable` reads back 0; error flags reported while stopping are `faults` |
| Fault latch | `FeetechChain::faults()`: every flag a stop reported, refusing commands until `ActuatorChain::clear_faults()` |
| The port | `sx_embodiment_drivers::serial::SerialPort`, the shape of the vendor's `PortHandler` |
| A body | `body::FeetechBody { bindings, port, baud, servos }`, each `ServoSetup` a joint's `MotorNormMode` and `ServoConfiguration`; `FeetechBody::open(port, drive, calibration)`. `body::BODIES` is one per bus, named by its port: the SO-101 follower (`so101::SO101`, `serial`) and the static upper body's neck and grippers (`static_upper_body::{NECK, RIGHT_GRIPPER, LEFT_GRIPPER}`, probed read-only on 2026-10-25; which gripper bus is which side, and which id which finger, is provisional); each opens only with its units' calibration |
| The SO-101 chain | `sx_feetech_serial::so101::open_so101(port, calibration)`, composed from `bindings/so101.json` (`so101(calibration)`) |
| Per-unit calibration | `sx_embodiment_drivers::lerobot::MotorCalibration` (`drive_mode`, `homing_offset`, `range_min`, `range_max`; re-exported here), one per servo, as `LeRobot` records it; open refuses a servo whose registers disagree. The wire normalization is `sx_embodiment_drivers::lerobot::unnormalize`, shared with `sx-dynamixel-serial` |
| Configuration | `sx_feetech_serial::ServoConfiguration` (PID, and a `TorqueProtection` for the gripper), written at open as `SOFollower.configure` does |
| Approach speed | Every servo's `Goal_Speed` (46) and `Acceleration` (41) are written at open from its `ServoConfiguration.approach` (`BRING_UP_APPROACH`, 0.25 rad/s and 0.5 rad/s², in the vendor's steps of 0.732 rpm and 8.7 °/s², rounded down) and read back, for the servos whose body declares the limit (the static upper body's; not the SO-101's, which keeps `LeRobot`'s registers); `ActuatorChain::approach_speed_limited(axis)` is then true for those axes. The registers are RAM, lost when a servo power-cycles, so every enable proves them again. Before `Torque_Enable` 1 each servo's goal is set to where it rests |
| Where it rests | `ActuatorChain::last_observed()`: every servo's `Present_Position` as a joint coordinate, read at open and after every stop and cleared by the next command, unclamped to the calibrated range, so a `Governor` starts from where the body rests |
| Stop budget | `ActuatorChain::stop_budget()`: the port's `timeout()` times four exchanges (torque off, unlock, read-back, position) per servo |
| A read-only scan | `sx_feetech_serial::scan::scan(port, ids)`: pings each id and reads model, firmware, the calibration (`MotorCalibration`), mode, PID, torque limit, torque enable, position, voltage and temperature into a `BusScan`; it speaks through `scan::ReadOnlyPort`, which refuses every packet but an addressed PING or READ (`scan::is_read_only`) before the link. The port is opened at `scan::STS3215_BAUD`. `sx-commission` is its consumer |

- **Sequencing** is `LeRobot` 0.6.0's Feetech bus, the driver the SO-101 runs with today:
  at open, the model check, `is_calibrated` and `configure` with torque disabled;
  `enable_torque` (`Torque_Enable` 1, `Lock` 1) before the first command after open or a stop,
  one `Goal_Position` SYNC WRITE per command, `disable_torque` (`Torque_Enable` 0, `Lock` 0)
  on every servo at stop, then the read-back.
- **Open the arm at rest.** Open disables torque on every servo before configuring it, so an
  arm holding a pose slumps under gravity. `LeRobot` assumes the same: `SOFollower.connect`
  expects the arm "in a rest position" when torque is disabled (`so_follower.py:91-96`).
- **Wire values.** The binding's drive map gives the actuator coordinate. That coordinate is
  degrees for the arm and percent of the declared range for the gripper, and the servo's
  calibrated range takes it to ticks as `MotorsBus._unnormalize` does. Targets are admitted
  within the declared bounds, widened to their float32 roundings where those land outside,
  and converted at float32, as the station's earlier
  Python follower (`sx_drivers.feetech.radians_to_wire`) did.

## Parity

`tests/fixtures/scservo/mint.py` runs `scservo_sdk` against a recording port, with `LeRobot`'s
`_unnormalize` executed from its pinned file, and writes `sts3215_so101.json`;
`sources.json` holds the sha256 of every source it ran and of the transcript. The tests
replay each exchange byte for byte.

```bash
cargo test && cargo clippy --all-targets -- -D warnings && cargo fmt --check
```
