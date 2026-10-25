# sx-yubi

The YUBI camera and encoder device contracts shared by capture hosts. `identity` owns
supported USB models, the Sonix flash-UID procedure through `Xu`, and the ESP32-C6
factory-MAC identity. `encoder_line` owns the existing Rust firmware-line parser and
framer. `camera` supplies exposure/gain values and caller-supplied bounds; it contains
no exposure algorithm or recording defaults.

The crate owns no device handle, worker, recording clock, file, or Factory state. Its
flash-UID transaction retains the existing bounded sleep while polling the device.
There is no pod-package dependency.
`sx-capture-usb` and `sx-capture-linux` implement hardware transport; the pod core
selects exposure policy and the recording runtime decides admission and writes evidence.
Consumers import these contracts directly; there is no copy or reexport in pod-core.

The extraction preserves the Rust parser's existing numeric envelope, first-cut-line
handling, and 256-byte line bound exactly. The existing Python encoder path has a
4096-byte bound and derives its joint envelope from the registered body, with slightly
different floating-point rounding. Those paths have not been unified or requalified by
this move; changing either acceptance boundary requires explicit evidence.

Run `cargo test -p sx-yubi` from the embodiment workspace. These tests exercise the
identity transactions and parser behavior; they do not qualify physical hardware.
