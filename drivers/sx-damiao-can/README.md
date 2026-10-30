# sx-damiao-can

Damiao DM-J motors over CAN, translated from Damiao's own `DM_CAN.py`
(<https://gitee.com/kit-miao/motor-sdk> at `fb0e9fc5`, `Python例程/u2can/DM_CAN.py`; Damiao's
copy of cmjang's `DM_Control_Python`, MIT, Copyright (c) 2024 cmjang — see
`LICENSE-THIRD-PARTY`), and the
bodies built of them: the Seeed reBot B601-DM's chain of seven, and the Sentient humanoid upper
body's chain of eleven and static upper body's chain of fifteen, each one `DamiaoBody`. Every departure from `DM_CAN.py` is listed in the
header of the module that makes it.

Consumer: the `sx` station's `chain::ChainActuator`, over `b601::open_b601`,
`humanoid::open_humanoid_upper_body` and `body::DamiaoBody::open`.

## The surface the station builds against

| Item | Path |
|---|---|
| The chain trait | `sx_embodiment_drivers::ActuatorChain` (`axes`, `commandable`, `command`, `read_positions`, `stop`, `stop_budget`); `DamiaoChainError` implements `ChainError` |
| A command's evidence | `sx_embodiment_drivers::Receipt<MotorFeedback>`: `evidence`, `observed` (joint coordinates), `feedback` (one `MotorFeedback` per motor) |
| A stop's proof | `sx_embodiment_drivers::StopReport`, and `StopReport::prove()` for a `ProvenStop` |
| The port | `sx_embodiment_drivers::can::{CanFrame, CanPort}` |
| The B601 chain | `sx_damiao_can::b601::open_b601(arm_port, gripper_port, drive)`: a `Composite` of the arm's and gripper's `DamiaoChain`s, composed from `bindings/b601-dm.json` (`b601_arm`, `b601_gripper`) and calibrated by the robot instance's `InstanceDrive` |
| A body on one bus | `sx_damiao_can::body::DamiaoBody { bindings, master_id_offset, joints }`, a `const` of the rendered bindings, the answer-id offset and each joint's `JointGains { joint, kp, kd }` in the bindings' order: `body(drive)` (the body calibrated, then its Damiao axes alone, refused unless they are on one port and are the joints configured in order; a body's other buses, such as the static upper body's Feetech neck, are other crates'), `axes(drive)` (each motor's range the joint's bounds through its drive map), `open(port, drive)`; `DamiaoBody::of(embodiment_id)` finds one among `body::BODIES` |
| The humanoid upper body | `sx_damiao_can::humanoid::open_humanoid_upper_body(port, drive)`: one `DamiaoChain` of the torso yaw and both arms (DM8009 shoulders and torso, DM4340 elbows, forearms and wrists) on `can0`, answer ids send id + 0x20, composed from `bindings/sentient-humanoid-upper-body.json`; the waist's two-motor linkage is in no chain and its motors are never enabled; `humanoid::HUMANOID_UPPER_BODY` is its `DamiaoBody` |
| The static upper body | `sx_damiao_can::static_upper_body::STATIC_UPPER_BODY.open(port, drive)`: one `DamiaoChain` of the torso yaw (DM-J8009P, 0x1F) and both seven-joint arms (DM-J8009P shoulders, DM4340 upper arm, elbow and forearm, DM4310 wrists; left 0x18..0x1E, right 0x09 then 0x10..0x15) on `can1`, answer ids send id + 0x20, composed from `bindings/sentient-static-upper-body.json` (a development body), whose neck and grippers are `sx-feetech-serial`'s, on ports `neck`, `right_gripper` and `left_gripper` |
| The USB2CAN bridge | `sx_damiao_can::usb2can::Usb2CanPort::new(serial)` over any `Read + Write` serial stream |
| A `SocketCAN` interface | `sx_socketcan::SocketCanPort::open("can1")` (Linux), in its own crate: it is no vendor's; it holds the interface's lock against a second of our processes, and budgets each frame its 10 ms queue wait (`CanPort::frame_allowance`) |
| Where the body rests | `ActuatorChain::last_observed()`: every motor's latest answered position, from open and each stop, which `sx_embodiment_drivers::Governor` starts its slew from |
| Per-motor answers | `sx_damiao_can::MotorFeedback`: state (`MotorState` from the first-byte nibble), position, velocity, torque, MOS and rotor temperatures |
| A read-only scan | `sx_damiao_can::scan::scan(port, ids)`: `read_motor_param` of `MST_ID` for each id and, from each motor answering, `ESC_ID`, `CTRL_MODE`, `PMAX`, `VMAX`, `TMAX`, with the arbitration id its replies arrive on, into a `BusScan`; `MotorScan::fits(model)` checks the ranges against the model's row. No mode switch and no enable; it speaks through `scan::ReadOnlyCanPort`, which refuses every frame but a register read (`0x7FF`, `0x33`) before the link. `sx-commission` is its consumer |
| Limits found at open | `DamiaoChain::limits()`: the `PMAX`, `VMAX`, `TMAX` each motor reported and is scaled by |
| Stop budget | `ActuatorChain::stop_budget()`: the longest `stop` takes on this chain, two phases (disable all, then refresh the silent), each a frame per motor at the port's `frame_allowance`, a 20 ms answer wait and one more allowance (on a USB2CAN bridge, 68 ms for the B601's six arm motors, 48 ms for its gripper; `open_b601`'s composite stops one after the other, so its budget is their sum, 116 ms; 88 ms for the humanoid upper body's eleven, 104 ms for the static upper body's fifteen; on `SocketCAN`, whose sends may wait 10 ms for queue room, the static upper body's is 392 ms) |

- **Read-only positions.** `read_positions` drains older replies, snapshots feedback counters,
  requests status with the vendor's `0xCC` operation, and returns one `AxisReading` per motor in
  native state order, always all of them. A motor that answered since the snapshot is `Fresh`
  (with the instance calibration applied), or `Faulted` with its state code when it reports a
  fault or a state contradicting the chain's; one that did not is `Missing(Stale)` (its newest
  answer predates the poll; `Timeout` if it never answered), and a failed link is
  `Missing(Unavailable)`. It never enables torque or sends a position target.
- **One bridge, two chains.** Seeed ships one USB2CAN bridge for all seven motors
  (`reBotArm_control_py` `config/rebotarm_dm.yaml`: `/dev/ttyACM0`, 921600 baud); open both
  chains on clones of one `sx_embodiment_drivers::can::SharedCanPort` over a `Usb2CanPort`. The
  jaw's tactile stream is a separate source and stays outside the chain.
- **No frame-level API.** Pure frame builders and a per-frame `exchange` are not exposed:
  every frame is one `DM_CAN.py` sends, built by `dm_can::MotorControl`, and the chain judges
  the answers.
- **Phased bring-up: a departure from the vendor's serial order.** `DM_CAN.py` brings one
  motor up at a time, each frame followed by its own wait (0.05 s per register, 0.1 s per
  enable). On the static upper body (ZLG USBCANFD-200U, 2026-10-26) that took over 4 s to open
  and 2.25 s to enable fifteen motors, and the first motor enabled sat silent for 1.4 s against
  its 0.75 s `TIMEOUT` and latched fault 0xD before its first command. Every multi-motor
  operation here is phases: one frame to every motor back to back, then the answers awaited
  once against one deadline. Open: disable all, MIT mode for all, every limit register, disable
  and settle (4 round trips, 90 frames for fifteen, no fixed wait). The first command: write
  every `TIMEOUT`, read every one back, enable all, MIT for all (4 round trips). Stop: disable
  all, then refresh the silent. Each motor's own frames keep the vendor's order, which
  `tests/transcripts.rs` holds motor by motor against the vendor's recordings; only the
  interleaving across motors and the waits differ. A frame refused for a full transmit queue
  (`WouldBlock`) is sent again as the bus drains, within the port's `frame_allowance`.
- **Atomic motion admission.** MIT frames are built before enable or submission, then admitted
  in groups bounded by `CanPort::batch_capacity()`. A refused group accepted no prefix, so only
  that group is retried; a single-frame transport retains its existing sequence. On ZLG a
  fourteen-motor phase fits one transfer instead of a scheduling-dependent split whose suffix
  must wait for an earlier USB acknowledgement. Feedback deadlines and missing readings are
  unchanged: queue admission never proves a motor answered. Bring-up, refresh and stop retain
  their existing per-frame sequencing.
- **Commandable ranges.** `commandable()` is each axis's declared bounds intersected with the
  motor range: for the arm, the URDF limits inside the deployed LeRobot driver's soft box; for
  the gripper, 0..0.045 m, what the motor's -5.0 rad open angle reaches of the 0.0715 m stroke.
  The range limits motion, not state: each command is checked against it widened to where
  the motor last answered (`CommandRange::reaching`: `[min(lower, now), max(upper, now)]`), so
  a joint resting outside (the static upper body's left wrist roll, found at -2.937 rad against
  +/-2.097) can be held or moved toward the range, never further out, and nothing refuses the
  open for where a joint rests. `Governor` steps within the same interval. An axis whose motor
  range spans more than one turn (the static upper body's forearm yaws) is cut to one turn of
  the motor's zero, -pi..pi: Damiao's output encoder is single-turn, so outside that window a
  joint reads as its alias after a power cycle; an aliased joint is commanded from its alias
  reading.
- **Communication timeout.** A `DamiaoAxis` with `can_timeout` has the motor's `TIMEOUT`
  register (0x09, 50 µs per count, Damiao's DM-J4340-2EC manual V1.1 pp. 20 and 30) written in
  RAM right before every enable, confirmed by echo and read-back, and no motor is enabled
  until every motor confirmed; the first command follows the enable at once; never saved to flash. An enabled motor that
  hears no command for that long disables itself and reports fault 0xD until power-cycled or
  cleared with `FF..FB`. Open sends that clear (`DamiaoChain::clear_fault`, refused while
  enabled) once to a motor answering 0xD while disabled and still refuses one whose fault
  persists; it clears no other fault. The static upper body sets 0.75 s, the heartbeat
  of the motor table it was first described with; the B601-DM and humanoid set none, as their sources do.
- **Gains** are Seeed's (`reBotArm_control_py` `config/rebotarm_dm.yaml`); the 4340Ps' kd of
  8 is declared as 5, which `DM_CAN.py`'s `controlMIT` clamps it to on the wire.
  The humanoid's are `humanoid-simrealgap`'s single-motor validated gains
  (`configs/training_actuator_params_20260606.csv` at `5c11827f`), not a whole-body approval:
  the DM8009 and the humanoid's whole chain are owed the station's bench qualification.
  The static upper body's are its hardware profile's
  (`assets/sentient_static_upper_body/source/humanoid_allisa_bot_hardware.json`, `mit_gains`;
  the torso's from the superseded description's motor table), not a qualification either. That driver's 0.25 rad/s speed cap and 0.2 rad tracking bound were its
  bring-up choice and are not kept: `sx_embodiment_drivers::Governor` bounds each joint at its
  rated motion (`AxisLimits::rated`), and sends the speed it steps each axis at as the MIT
  frame's planned velocity.
- **Brake before disable.** A stop of a chain whose motors last answered moving holds each
  motor where it was, at its own `kp` and `kd` with no planned velocity, every 10 ms for up to
  `BRAKE_TIME` (0.3 s) until every joint is slower than `BRAKE_STILL` (0.05 rad/s), then
  disables: a disabled motor gives no torque, so a joint disabled at speed coasted limp (refute
  2026-10-30). A failing link or a motor that does not answer ends the brake at once. A body
  already still gets the vendor's stop, frame for frame (`tests/transcripts.rs`).
- **Stop proof.** A stop drains what is already on the link, then counts only answers to its
  disable frames. Where firmware does not report its enable state (the static upper body's
  answers state 1 enabled or not), a fault-free answer proves the stop weakly and the motor is
  named in `StopReport::unobserved` (`ProvenStop::observed_disabled` is then false).
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
