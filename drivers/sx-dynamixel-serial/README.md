# sx-dynamixel-serial

Dynamixel X-series servos over their half-duplex serial bus, Protocol 2.0, translated from
ROBOTIS's own DynamixelSDK (<https://github.com/ROBOTIS-GIT/DynamixelSDK> at `a8d35d51`,
`python/src/dynamixel_sdk/`; Apache-2.0, Copyright 2017 ROBOTIS CO., LTD. — see
`LICENSE-THIRD-PARTY`). Every departure from the SDK is listed in the header of the module that
makes it; the one that changes bytes is byte stuffing, which the Python SDK computes and then
drops, and which is applied here as the C++ SDK and the e-manual do.

**Untested on the bench.** The registry's `DYNAMIXEL_XL330_M288` and `DYNAMIXEL_XM430_W350`
models and its `dynamixel_serial` bus were added on 2026-10-25 so the static upper body's
grippers can be bound once that robot is probed; no servo has been driven through this crate. A
model is qualified when one servo of it has been driven on the bench through this chain and has
safe-stopped with its torque read back off. No registered body binds a Dynamixel yet
(`body::BODIES` is empty).

Consumer: `sx-embodiment-bodies`, which opens a body's Dynamixel port beside its other buses.

## The surface

| Item | Path |
|---|---|
| The chain trait | `sx_embodiment_drivers::ActuatorChain`; `DynamixelChainError` implements `ChainError` |
| A command's evidence | `Receipt<()>`: always `DispatchAttempted`, since the `Goal_Position` SYNC WRITE has no answer |
| A stop's proof | `StopReport`: a servo is unproven unless `Torque_Enable` reads back 0; a `Hardware_Error_Status` (or else an error byte) read while stopping is a fault |
| Fault latch | `DynamixelChain::faults()`, refusing commands until `ActuatorChain::clear_faults()` |
| The port | `sx_embodiment_drivers::serial::SerialPort` |
| A body | `body::DynamixelBody { bindings, baud, servos }`, each `ServoSetup` a joint's `MotorNormMode` and `OperatingMode` (position, extended position, current-based position) |
| Per-unit calibration | `sx_embodiment_drivers::lerobot::MotorCalibration`, as `LeRobot` records it; open refuses a servo whose `Homing_Offset`, position limits or `Drive_Mode` disagree |
| Model check | `model_number`: XL330-M288 1200, XM430-W350 1020, from each model's e-manual page; open pings every servo and refuses another number |
| Stop budget | the port's `timeout()` times two exchanges (torque off, read-back) per servo |

- **Sequencing** is `LeRobot` 0.6.0's Dynamixel bus: at open the ping, a refusal of any servo
  reporting a hardware error (it needs a reboot, a person's call), `is_calibrated`, then torque
  off, `Return_Delay_Time` 0 and each servo's `Operating_Mode`; `Torque_Enable` 1 before the
  first command after open or a stop; one `Goal_Position` SYNC WRITE per command; torque off on
  every servo at stop, then one seven-byte read from `Torque_Enable` to `Hardware_Error_Status`.
- **Wire values** are `LeRobot`'s `_unnormalize` with `apply_drive_mode` false (the servo
  reverses itself by its `Drive_Mode`); a goal outside the unit's calibrated range is refused,
  since the extended and current-based modes ignore the servo's position limits.
- **Open at rest.** Open disables torque, so a held pose slumps.

## Parity

`tests/emanual.rs` reproduces the e-manual's Protocol 2.0 worked examples byte for byte (ping,
read, write, sync read, sync write; <https://github.com/ROBOTIS-GIT/emanual> at `0807b1b5`,
`docs/en/dxl/protocol2.md`). `tests/fixtures/dynamixel_sdk/mint.py` runs the Python SDK itself
on a test pair (`bindings.json`, not a registered body) with `LeRobot`'s `_unnormalize`
executed from its pinned file, and writes `xl330_xm430.json`; `sources.json` holds the sha256
of every source and of the transcript, and `tests/transcripts.rs` replays it.

```bash
cargo test -p sx-dynamixel-serial && cargo clippy -p sx-dynamixel-serial --all-targets -- -D warnings
```
