//! A robot's hardware found and commissioned, read-only; and a Feetech bus calibrated, which
//! alone may write, only under an explicit `--move` ([`calibrate`]).
//!
//! Commissioning answers one question for a station: which physical bridge carries each of the
//! body's buses. It is answered by asking the hardware, never by a device path or an
//! enumeration order:
//!
//! 1. [`bridges::enumerate`] lists every USB device this host carries with its vendor and
//!    product id, serial number, tty or `SocketCAN` interface, and whether this host can open it
//!    (a ZLG CAN bridge on macOS is present, with no driver).
//! 2. [`probe::probe`] scans each openable bus with its vendor crate's read-only scan:
//!    `sx_feetech_serial::scan` on serial bridges (PING and READ only) and
//!    `sx_damiao_can::scan` on CAN interfaces (register reads only). Each scan speaks through
//!    its crate's read-only port, which refuses any other packet or frame before the link sees
//!    a byte, so nothing here can write a register, enable torque or move a joint.
//! 3. [`matching::commission`] matches each port a body's bindings declare ([`declared`]) to
//!    the one bridge on which exactly its ids answer as its models, refusing zero or several
//!    candidates by name, and renders the `sx.station-commissioning/v1` document
//!    `sx-embodiment-bodies` opens a body from, with each Feetech port's per-unit calibration
//!    in `LeRobot`'s format ([`output`]).
//!
//! The `sx-commission` binary runs the three: `probe` reports, `commission` dry-runs, and
//! writes files only under an explicit `--write <dir>`.

pub mod bridges;
pub mod calibrate;
pub mod declared;
pub mod host;
pub mod matching;
pub mod output;
pub mod probe;
pub mod sysfs;
pub mod tty;
