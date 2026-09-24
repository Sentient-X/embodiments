# StarArm102-LD evidence and promotion gate

`stararm102-ld` is available through `development_embodiments`. The packaged
kinematics are a provisional generic StarArm102 model, not an authoritative LD
hardware revision. Registration in the complete registry is deliberately withheld.

All upstream references below are pinned to
`0896306e40891c3ee4c97228e85dd708d61326de`:

- [Manufacturer specifications and joint mapping](https://github.com/servodevelop/Star-Arm-102/blob/0896306e40891c3ee4c97228e85dd708d61326de/README.md)
  distinguish LD, HD and FL, list seven LD channels with 12-bit magnetic encoders,
  and map physical joints 1–7 to UART IDs 0–6.
- [SDK reader](https://github.com/servodevelop/Star-Arm-102/blob/0896306e40891c3ee4c97228e85dd708d61326de/Python_SDK/stararm102_ro.py)
  defaults to LD, unlocks the leader, polls `send_sync_servo_monitor`, and reads
  `servos[id].angle_monitor`. Hence each provisional coordinate declares a
  `VendorReadout` for its corresponding channel. This is not `ActuatorFeedback`:
  no qualified actuator product, bus binding or drive transform is established.
  Unlocking during this example is runtime behavior, not evidence that the
  hardware is intrinsically passive. Actuation remains explicitly undocumented.
- [Generic URDF](https://github.com/servodevelop/Star-Arm-102/blob/0896306e40891c3ee4c97228e85dd708d61326de/ROS2_HUMBLE/src/stararm102_description/urdf/stararm102_description.urdf)
  is the source of the packaged model. The asset strips visual/collision elements
  and preserves the upstream kinematics and inertials. The parity test verifies
  these packaged bounds and the mimic relation, not LD hardware equivalence.
- [LD CAD inventory](https://github.com/servodevelop/Star-Arm-102/blob/0896306e40891c3ee4c97228e85dd708d61326de/Hardware/cad/README.md)
  identifies separate LD drawings. The generic URDF has not been qualified
  against those drawings or physical LD measurements.

There is concrete ambiguity: the LD table gives joint 1 as ±110 degrees, whereas
this URDF gives ±2.27 radians (about ±130 degrees). The LD table calls the seventh
axis a handle; this URDF describes a pair of coupled fingers. Other joint bounds
also differ. Do not substitute table bounds into the URDF without establishing
coordinate frames, signs and zeros. The SDK additionally scales the handle value
by 1.5 and clamps it before commanding the follower; that teleop mapping is not
physical leader geometry and does not belong in this nominal definition.

Before promotion, obtain an authoritative LD description or a documented
CAD/measurement comparison covering joint frames, limits, handle geometry and
any retained inertials. Reconcile the coordinate convention with the readout,
update the asset identity and parity tests, and then move the definition into
`_ALL_SPECS`. Qualify actuation separately if evidence becomes available. Per-unit
encoder zeros remain session/episode calibration; controller mappings remain
owned by the controller/action layer.
