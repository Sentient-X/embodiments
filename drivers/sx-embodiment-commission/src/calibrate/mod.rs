//! Calibrating a Feetech STS3215 bus: the one part of this crate that may write a register or
//! move a joint, and only under an explicit `--move`.
//!
//! `sx-commission calibrate` has three modes. `read` dumps each servo's registers and its
//! position in `LeRobot`'s degrees and percent; it speaks through `scan::ReadOnlyPort`, as
//! every probe does. `gripper` sweeps each finger slowly toward both ends until it stalls,
//! backs off at once and records the stops as a `LeRobot` calibration. `jog` nudges one joint
//! by a few ticks and back, and times the move. Without `--move`, `gripper` and `jog` read the
//! servos through the read-only port and print what they would do: no [`session::Session`]
//! exists, so nothing can be written.
//!
//! Every write goes through a [`session::Session`], which makes it impossible to leave a servo
//! powered or a register changed:
//!
//! - Before a register is first changed, its value is read and saved to the run's
//!   [`journal::Journal`] on disk (written, synced and renamed into place), and the session
//!   refuses to start when a journal from an earlier run is still there.
//! - A servo is powered only by [`session::Session::enable`]: its `Goal_Position` is written to
//!   where it rests, its `Goal_Speed`, `Acceleration` and `Torque_Limit` to the run's slow
//!   values, each read back (the enable is refused when any differs), and it is journaled as
//!   powered before `Torque_Enable` 1 goes out. One servo is powered at a time.
//! - On every way out the session undoes what it did: [`session::Session::close`] on a normal
//!   return or an error, and its `Drop` on a panic's unwinding. Each servo it touched is
//!   written `Torque_Enable` 0 and read back, then every saved register is written back in
//!   reverse order and read back. A servo whose torque-off is not proven keeps its saved
//!   registers (a restored full `Torque_Limit` would only make it stronger) and the journal is
//!   kept; [`session::RestoreReport`] names it and the binary exits non-zero.
//! - SIGINT, SIGTERM, SIGHUP and SIGQUIT set a flag (`signal_hook::flag`) instead of killing
//!   the process. Every step of a motion checks it, as it checks the run's deadline
//!   (`--max-seconds`), and returns an error, which closes the session. No wait lasts longer
//!   than one packet timeout (20 ms) or one sleep slice (5 ms) between checks. SIGKILL cannot
//!   be caught: the journal is then left behind, the next run refuses to start, and
//!   `sx-commission calibrate --restore <journal>` disables and restores what it names.
//!
//! The registers are `sms_sts.py`'s (`sx_feetech_serial::scservo_sdk`) and, where it does not
//! name them, Feetech's C++ `SCServo` library (`FTServo_Arduino` at `64922cda`,
//! `src/SMS_STS.h`): `Torque_Limit` is 48, in tenths of a percent, and `Present_Load` (60) is
//! sign-magnitude at bit 10, as `SMS_STS::ReadLoad` decodes it (`src/SMS_STS.cpp:191`).
//! Widened position limits are written with `Lock` (55) at 1, which Feetech's memory table
//! describes as a write the servo does not keep across a power cycle; the journal restores
//! them either way, and every write is read back.

pub mod cli;
pub mod journal;
pub mod motion;
pub mod range;
pub mod read;
pub mod session;

use std::io;
use std::path::PathBuf;
use std::time::Duration;

use sx_feetech_serial::scservo_sdk::CommResult;

/// `Torque_Limit`, RAM, in tenths of a percent of the servo's maximum torque, loaded from
/// `Max_Torque_Limit` (16) at power-on (`SMS_STS_TORQUE_LIMIT_L`, `FTServo_Arduino`
/// `src/SMS_STS.h:38`).
pub const TORQUE_LIMIT: u8 = 48;
/// `Present_Load`'s sign bit (`SMS_STS::ReadLoad`, `FTServo_Arduino` `src/SMS_STS.cpp:191`).
pub const LOAD_SIGN_BIT: u32 = 10;
/// `Present_Position`'s and `Present_Speed`'s sign bit (`sms_sts.py` `ReadPos`, `ReadSpeed`).
pub const POSITION_SIGN_BIT: u32 = 15;
/// `Homing_Offset`'s sign bit (`LeRobot`'s `STS_SMS_SERIES_ENCODINGS_TABLE`).
pub const HOMING_OFFSET_SIGN_BIT: u32 = 11;
/// The STS3215's highest position, in ticks; its encoder has 4096 steps a turn.
pub const MAX_TICK: u16 = 4095;

/// Why a calibration step was refused or stopped.
#[derive(Debug, thiserror::Error)]
pub enum CalibrateError {
    #[error("servo {id}: {result:?} at register {address}")]
    Comm {
        id: u8,
        address: u8,
        result: CommResult,
    },
    #[error("servo {id} reported error flags {flags:#04x} at register {address}")]
    Flags { id: u8, address: u8, flags: u8 },
    #[error("servo {id}: register {address} reads {read} after {wrote} was written")]
    ReadBack {
        id: u8,
        address: u8,
        wrote: u16,
        read: u16,
    },
    #[error("servo {id} cannot be powered while servo {powered} is: one servo at a time")]
    OnePowered { id: u8, powered: u8 },
    #[error("servo {0} is not powered by this run")]
    NotPowered(u8),
    #[error("goal {goal} for servo {id} lies outside its limits {min}..{max}")]
    OutsideLimits {
        id: u8,
        goal: i32,
        min: u16,
        max: u16,
    },
    #[error("stopped by signal {0}")]
    Signal(usize),
    #[error("the run's deadline of {0:?} passed")]
    Deadline(Duration),
    #[error("journal {path}: {source}")]
    Journal { path: PathBuf, source: io::Error },
    #[error(
        "{path} holds an earlier run's changes, not yet undone; run \
         sx-commission calibrate --restore {path}"
    )]
    Unrestored { path: PathBuf },
    #[error("{0}")]
    Refused(String),
}
