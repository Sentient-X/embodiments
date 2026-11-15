# sx-yubi

The YUBI camera and encoder device contracts shared by capture hosts. `identity` owns
supported USB models, the Sonix flash-UID procedure through `Xu`, and the ESP32-C6
factory-MAC identity. `encoder_line` owns the existing Rust firmware-line parser and
framer. `switch` owns the start/stop-switch firmware's `# {json}` status lines (50 Hz, up to
~540 bytes) and the once-per-hold law: each `(session, event_id)` is a hold once, and a
reader's first frame, or a new session's, is a baseline, so a reconnect never replays an old
hold. What a hold does (which gripper may start an episode) is the host's decision. `camera` supplies exposure/gain values and caller-supplied bounds; it contains
no exposure algorithm or recording defaults.

The crate owns no device handle, worker, recording clock, file, or Factory state. Its
flash-UID transaction retains the existing bounded sleep while polling the device.
There is no pod-package dependency.
`sx-capture-usb` and `sx-capture-linux` implement hardware transport; the pod core
selects exposure policy and the recording runtime decides admission and writes evidence.
Consumers import these contracts directly; there is no copy or reexport in pod-core.

The extraction preserves the Rust parser's existing numeric envelope and first-cut-line
handling exactly. The line bound is 1024 bytes (was 256, which cut every status line and so
lost it); a status line is never an angle sample, so the numeric samples and their timing are
unchanged. The existing Python encoder path has a
4096-byte bound and derives its joint envelope from the registered body, with slightly
different floating-point rounding. Those paths have not been unified or requalified by
this move; changing either acceptance boundary requires explicit evidence.

Run `cargo test -p sx-yubi` from the embodiment workspace. These tests exercise the
identity transactions and parser behavior, and replay two real byte streams from the right
encoder of rig yubi-pod1-20261114 (`tests/fixtures/`, captured 2026-11-15 with a DTR reset:
three 2 s holds, and taps); they do not qualify physical hardware.
