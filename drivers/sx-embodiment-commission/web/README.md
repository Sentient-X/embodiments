# DGX local teach viewer

A loopback-only viewer for the local keyboard teach tool. It bundles the installed
Three.js and URDFLoader dependencies and reuses the platform's robot render pack.
It needs no cloud service, camera, CDN or Node runtime on the DGX.

Build from the sx workspace (dependencies installed through root pnpm):

```bash
node packages/sx-embodiments/drivers/sx-embodiment-commission/web/build.mjs /tmp/sx-teach-web-dist
```

The example unprivileged `sx-teach-web` service serves http://127.0.0.1:8765.
Set `SX_TEACH_ROBOT=/absolute/path/to/the/commissioned/robot` in
`/etc/sx-teach/viewer.env` before installing that unit.
Its Python host is `.agents/skills/robot-bench/scripts/serve-local-teach.py` in
the sx root. It serves static assets and local diagnostic JSON; it is not a Fleet
API or a control transport. No HTTP endpoint enables, releases or moves motors.

Read sensors explicitly for a bounded snapshot while no station/controller owns
the robot. The existing read-only embodiment recorder supplies all positions.
Wait until the read finishes before launching `~/sx-teach/run left`. The optional
`--status-file` on that launcher publishes controller telemetry at up to10Hz through
a bounded queue; disk and HTTP work remain off the motor control thread.

The model displays received angles without clamping them into URDF limits.
Out-of-range rows and links are highlighted. Missing joints remain unmeasured;
the viewer retains the last drawn coordinate and labels incomplete/reference
poses. Retained data has an age and never becomes live merely by polling.
A world pose is not established when an ancestor is unmeasured. None of this
changes drive signs, zero offsets, physical limits or encoder turn interpretation.

Keyboard engagement/recording/replay remains authoritative. The local launcher
isolates the selected arm and its gripper through embodiment exclusions. Other
CAN motors stay disabled/read-only; fully excluded serial ports are not opened.
Unrelated right-side range errors do not block isolated left teaching. This does
not repair encoder reference mismatches on a side that is selected for control.
