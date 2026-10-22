# sx-damiao-can

Damiao DM-J motors over CAN, translated from Damiao's own `DM_CAN.py`
(<https://gitee.com/kit-miao/motor-sdk> at `fb0e9fc5`, `Python例程/u2can/DM_CAN.py`; Damiao's
copy of cmjang's `DM_Control_Python`, MIT, Copyright (c) 2024 cmjang — see
`LICENSE-THIRD-PARTY`), and the
bodies built of them: the Seeed reBot B601-DM's chain of seven and the Sentient humanoid upper
body's chain of eleven. Every departure from `DM_CAN.py` is listed in the
header of the module that makes it.

Consumer: the `sx` station's `chain::ChainActuator`, over `b601::open_b601` and
`humanoid::open_humanoid_upper_body`.

## The surface the station builds against

| Item | Path |
|---|---|
| The chain trait | `sx_embodiment_drivers::ActuatorChain` (`axes`, `commandable`, `command`, `read_positions`, `stop`, `stop_budget`); `DamiaoChainError` implements `ChainError` |
| A command's evidence | `sx_embodiment_drivers::Receipt<MotorFeedback>`: `evidence`, `observed` (joint coordinates), `feedback` (one `MotorFeedback` per motor) |
| A stop's proof | `sx_embodiment_drivers::StopReport`, and `StopReport::prove()` for a `ProvenStop` |
| The port | `sx_embodiment_drivers::can::{CanFrame, CanPort}` |
| The B601 chain | `sx_damiao_can::b601::open_b601(arm_port, gripper_port, drive)`: a `Composite` of the arm's and gripper's `DamiaoChain`s, composed from `bindings/b601-dm.json` (`b601_arm`, `b601_gripper`) and calibrated by the robot instance's `InstanceDrive` |
| The humanoid upper body | `sx_damiao_can::humanoid::open_humanoid_upper_body(port, drive)`: one `DamiaoChain` of the torso yaw and both arms (DM8009 shoulders and torso, DM4340 elbows, forearms and wrists) on `can0`, answer ids send id + 0x20, composed from `bindings/sentient-humanoid-upper-body.json`; the waist's two-motor linkage is in no chain and its motors are never enabled |
| The USB2CAN bridge | `sx_damiao_can::usb2can::Usb2CanPort::new(serial)` over any `Read + Write` serial stream |
| Per-motor answers | `sx_damiao_can::MotorFeedback`: state (`MotorState` from the first-byte nibble), position, velocity, torque, MOS and rotor temperatures |
| Limits found at open | `DamiaoChain::limits()`: the `PMAX`, `VMAX`, `TMAX` each motor reported and is scaled by |
| Stop budget | `ActuatorChain::stop_budget()`: the longest `stop` takes on this chain, from the crate's own waits, polls and link allowance (122 ms for the B601's six arm motors, 52 ms for its gripper; `open_b601`'s composite stops one after the other, so its budget is their sum, 174 ms; 192 ms for the humanoid upper body's eleven) |

- **Read-only positions.** `read_positions` drains older replies, snapshots feedback counters,
  requests status with the vendor's `0xCC` operation, and requires a fresh, expected-state reply
  from every motor. It applies the existing instance calibration to all coordinates. A missing
  or faulted reply refuses the whole read; it never enables torque or sends a position target.
- **One bridge, two chains.** Seeed ships one USB2CAN bridge for all seven motors
  (`reBotArm_control_py` `config/rebotarm_dm.yaml`: `/dev/ttyACM0`, 921600 baud); open both
  chains on clones of one `sx_embodiment_drivers::can::SharedCanPort` over a `Usb2CanPort`. The
  jaw's tactile stream is a separate source and stays outside the chain.
- **No frame-level API.** Pure frame builders and a per-frame `exchange` are not exposed:
  every bus operation is a `dm_can::MotorControl` call that sends and reads as `DM_CAN.py`
  does, and the chain judges the answers.
- **Commandable ranges.** `commandable()` is each axis's declared bounds intersected with the
  motor range: for the arm, the URDF limits inside the deployed LeRobot driver's soft box; for
  the gripper, 0..0.045 m, what the motor's -5.0 rad open angle reaches of the 0.0715 m stroke.
  A command outside it is refused before any frame is sent; a leader should be mapped into it.
- **Gains** are Seeed's (`reBotArm_control_py` `config/rebotarm_dm.yaml`); the 4340Ps' kd of
  8 is declared as 5, which `DM_CAN.py`'s `controlMIT` clamps it to on the wire.
  The humanoid's are `humanoid-simrealgap`'s single-motor validated gains
  (`configs/training_actuator_params_20260606.csv` at `5c11827f`), not a whole-body approval:
  the DM8009 and the humanoid's whole chain are owed the station's bench qualification.
- **DM4340 velocity range.** Damiao publishes 10 rad/s (`DM_CAN.py`) and 8 rad/s (its C++ SDK).
  Open reads each motor's registers, accepts either, scales that motor by what it reported,
  and records it in `limits()`.

## Parity

`tests/fixtures/dm_can/mint.py` runs `DM_CAN.py` against a recording serial device and writes
`transcripts.json`; `provenance.json` holds the sha256 of every source it ran and of the
transcripts. The tests replay each recorded session byte for byte and wait for wait: four for
the B601-DM and one whole session for the humanoid upper body, plus DM8009 frames and decodes
cross-checked against `sentient_can` and LeRobot. The transcripts' JSON is parsed with
`serde_json`'s `float_roundtrip`, so decoded positions compare bit for bit.

```bash
cargo test && cargo clippy --all-targets -- -D warnings && cargo fmt --check
```
