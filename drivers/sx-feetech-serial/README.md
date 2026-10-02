# sx-feetech-serial

Feetech STS servos over their half-duplex serial bus, translated from Feetech's own
`scservo_sdk` (<https://github.com/ftservo/FTServo_Python> at `cbcfa646`, `scservo_sdk/`; MIT,
Copyright (c) 2024 ftservo — see `LICENSE-THIRD-PARTY`), and the SO-101 follower's chain of six
STS3215s. Every departure from `scservo_sdk` is listed in the header of the module that makes
it.

Journey: intended consumer, the `sx` station's `FeetechActuator`
(`fleet/station/src/feetech.rs`), which becomes a thin async wrapper over `FeetechChain` in the
lane that lands right after this crate and the `sx` pin advance to it; until then nothing
consumes it. The station opens it on an SO-101's serial adapter once a serial device binding
exists there.

## The surface the station builds against

| Item | Path |
|---|---|
| The chain trait | `sx_embodiment_drivers::ActuatorChain` (`axes`, `commandable`, `command`, `stop`) |
| A command's evidence | `sx_embodiment_drivers::Receipt<()>`: always `DispatchAttempted`, since the SYNC WRITE has no answer |
| A stop's proof | `sx_embodiment_drivers::StopReport`: a servo is unproven unless its `Torque_Enable` reads back 0; error flags reported while stopping are `faults` |
| Fault latch | `FeetechChain::faults()`: every flag a stop reported, refusing commands until `FeetechChain::clear_faults()` |
| The port | `sx_embodiment_drivers::serial::SerialPort`, the shape of the vendor's `PortHandler` |
| The SO-101 chain | `sx_feetech_serial::so101::so101(calibration)` into `sx_feetech_serial::FeetechChain::open(port, servos)` |
| Per-unit calibration | `sx_feetech_serial::MotorCalibration` (`drive_mode`, `homing_offset`, `range_min`, `range_max`), one per servo, as `LeRobot` records it; open refuses a servo whose registers disagree |
| Configuration | `sx_feetech_serial::ServoConfiguration` (PID, and a `TorqueProtection` for the gripper), written at open as `SOFollower.configure` does |
| Stop budget | `FeetechChain::stop_budget()`: the port's `timeout()` times three exchanges (torque off, unlock, read-back) per servo |

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
