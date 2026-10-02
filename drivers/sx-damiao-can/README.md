# sx-damiao-can

Damiao DM-J motors over CAN, translated from Damiao's own `DM_CAN.py`
(<https://gitee.com/kit-miao/motor-sdk> at `fb0e9fc5`, `Python例程/u2can/DM_CAN.py`; Damiao's
copy of cmjang's `DM_Control_Python`, MIT, Copyright (c) 2024 cmjang — see
`LICENSE-THIRD-PARTY`), and the
Seeed reBot B601-DM's chain of seven of them. Every departure from `DM_CAN.py` is listed in the
header of the module that makes it.

Intended consumer: the `sx` station's B601 actuator. It switches to this crate in the lane that
lands right after this crate and the `sx` pin advance to it; until then nothing consumes it.

## The surface the station builds against

| Item | Path |
|---|---|
| The chain trait | `sx_embodiment_drivers::ActuatorChain` (`axes`, `commandable`, `command`, `stop`) |
| A command's evidence | `sx_embodiment_drivers::Receipt<MotorFeedback>`: `evidence`, `observed` (joint coordinates), `feedback` (one `MotorFeedback` per motor) |
| A stop's proof | `sx_embodiment_drivers::StopReport`, and `StopReport::prove()` for a `ProvenStop` |
| The port | `sx_embodiment_drivers::can::{CanFrame, CanPort}` |
| The B601 chain | `sx_damiao_can::b601::{b601, b601_arm, b601_gripper}` into `sx_damiao_can::DamiaoChain::open(port, axes)` |
| The USB2CAN bridge | `sx_damiao_can::usb2can::Usb2CanPort::new(serial)` over any `Read + Write` serial stream |
| Per-motor answers | `sx_damiao_can::MotorFeedback`: state (`MotorState` from the first-byte nibble), position, velocity, torque, MOS and rotor temperatures |
| Limits found at open | `DamiaoChain::limits()`: the `PMAX`, `VMAX`, `TMAX` each motor reported and is scaled by |
| Stop budget | `DamiaoChain::stop_budget()`: the longest `stop` takes on this chain, from the crate's own waits, polls and link allowance (136 ms for the B601's seven motors) |

- **One chain on one port.** Seeed ships one USB2CAN bridge for all seven motors
  (`reBotArm_control_py` `config/rebotarm_dm.yaml`: `/dev/ttyACM0`, 921600 baud), so the
  station's arm-and-gripper composite becomes one `DamiaoChain` over `b601()` on one
  `Usb2CanPort`. The jaw's tactile stream is a separate source and stays outside the chain.
- **No frame-level API.** Pure frame builders and a per-frame `exchange` are not exposed:
  every bus operation is a `dm_can::MotorControl` call that sends and reads as `DM_CAN.py`
  does, and the chain judges the answers.
- **Commandable ranges.** `commandable()` is each axis's declared bounds intersected with the
  motor range: for the arm, the URDF limits inside the deployed LeRobot driver's soft box; for
  the gripper, 0..0.045 m, what the motor's -5.0 rad open angle reaches of the 0.0715 m stroke.
  A command outside it is refused before any frame is sent; a leader should be mapped into it.
- **Gains** are Seeed's (`reBotArm_control_py` `config/rebotarm_dm.yaml`); the 4340Ps' kd of
  8 is declared as 5, which `DM_CAN.py`'s `controlMIT` clamps it to on the wire.
- **DM4340 velocity range.** Damiao publishes 10 rad/s (`DM_CAN.py`) and 8 rad/s (its C++ SDK).
  Open reads each motor's registers, accepts either, scales that motor by what it reported,
  and records it in `limits()`.

## Parity

`tests/fixtures/dm_can/mint.py` runs `DM_CAN.py` against a recording serial device and writes
`transcripts.json`; `provenance.json` holds the sha256 of every source it ran and of the
transcripts. The tests replay each recorded session byte for byte and wait for wait.

```bash
cargo test && cargo clippy --all-targets -- -D warnings && cargo fmt --check
```
