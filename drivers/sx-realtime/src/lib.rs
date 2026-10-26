//! The host's real-time policy for the threads a robot's bus traffic passes through.
//!
//! A control tick is only as punctual as the slowest thread between the bus and the loop: the
//! loop itself, an adapter's I/O thread, and a USB stack's completion thread (`nusb-events`,
//! which `nusb` creates on macOS and Linux alike). At default priority an overloaded host parks
//! any of them for tens of milliseconds: on 2026-10-26 a Mac at load 30–100 did, and the static
//! upper body's Damiao chain read answered motors as `Silent(<id>)` (robot-bench skill,
//! failures table). So every such thread asks for the host's real-time policy by its
//! [`Role`], and one crate says what that means on each host:
//!
//! - **macOS**: Mach's `THREAD_TIME_CONSTRAINT_POLICY`, a computation within a constraint each
//!   period (answer path: 250 µs within 1 ms every 1 ms; a loop of period `p`: `p/4` within
//!   `p/2`). Any user may take it.
//! - **Linux**: `SCHED_FIFO` (with `SCHED_RESET_ON_FORK`, so a child never inherits it), the
//!   answer path at [`ANSWER_PATH_PRIORITY`] above control loops at [`CONTROL_LOOP_PRIORITY`],
//!   so a loop that spins can never starve the answers it waits for; the kernel's real-time
//!   throttling (`sched_rt_runtime_us`, 95 % by default) still bounds a runaway. A process needs
//!   `RLIMIT_RTPRIO` of at least [`ANSWER_PATH_PRIORITY`] (systemd `LimitRTPRIO=`, or
//!   `/etc/security/limits.d`) or `CAP_SYS_NICE`: a host need the robot's launcher declares and
//!   applies, never a manual step. Without it every call is refused with `EPERM`
//!   ([`Refused::hint`] says so) and the thread runs as before.
//!
//! The policy alone is not enough on a host whose idle cores sleep deeply. On the DGX Spark
//! (GB10, `acpi_idle`: LPI-2 and LPI-3 take 231 and 433 µs to wake) a thread woken on an idle
//! core paid that on every frame; a 14-motor read cycle's host time on a virtual bus was p50
//! 0.26–0.66 ms, p99 0.6–1.9 ms, and was worse under `SCHED_FIFO` than without (real-time
//! threads are pushed to idle cores). Holding the kernel's CPU wake-latency request at zero
//! while a bus is open ([`hold_wake_latency`]) brought it to p50 40 µs, p99 75 µs, max under
//! 90 µs (2026-10-26, robot-bench skill). It needs write access to `/dev/cpu_dma_latency`
//! (root-only by default): a udev rule the robot's launcher applies, like `RLIMIT_RTPRIO`.
//!
//! A refusal never fails the caller's work: it is returned so the caller reports it once.

use std::fmt;
use std::time::Duration;

#[cfg(target_os = "linux")]
mod linux;
#[cfg(target_os = "macos")]
mod macos;

/// The `SCHED_FIFO` priority of a thread on the answer path (Linux).
pub const ANSWER_PATH_PRIORITY: i32 = 80;
/// The `SCHED_FIFO` priority of a control loop (Linux): below the answer path it waits on.
pub const CONTROL_LOOP_PRIORITY: i32 = 70;

/// What a thread does, which decides the policy it asks for.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Role {
    /// A thread every answer from the bus passes through: an adapter's I/O thread, a USB
    /// stack's completion thread. Each wake does well under 0.25 ms of work.
    AnswerPath,
    /// A control loop ticking every `period`.
    ControlLoop { period: Duration },
}

/// Why the policy was not taken.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Refused {
    /// The host refused it: an `errno` on Linux, a `kern_return_t` on macOS.
    Denied(i32),
    /// This host has no real-time policy this crate sets.
    Unsupported,
}

impl Refused {
    /// What would let the policy be taken, for the caller's one report.
    #[must_use]
    pub fn hint(self) -> &'static str {
        match self {
            #[cfg(target_os = "linux")]
            Self::Denied(libc::EPERM) => {
                "the process may not use SCHED_FIFO: the robot host must grant RLIMIT_RTPRIO >= 80 (systemd LimitRTPRIO=80) or CAP_SYS_NICE"
            }
            #[cfg(target_os = "linux")]
            Self::Denied(libc::EACCES) => {
                "the process may not write /dev/cpu_dma_latency: the robot host must grant it (udev: KERNEL==\"cpu_dma_latency\", GROUP=<the robot's group>, MODE=\"0660\")"
            }
            Self::Denied(_) => "the host refused the real-time policy",
            Self::Unsupported => "this host has no real-time policy here",
        }
    }
}

impl fmt::Display for Refused {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Denied(code) => write!(formatter, "refused ({code}): {}", self.hint()),
            Self::Unsupported => formatter.write_str(self.hint()),
        }
    }
}

impl std::error::Error for Refused {}

/// Put the calling thread under the real-time policy for `role`.
///
/// # Errors
///
/// [`Refused`] when the host refuses the policy or has none.
pub fn elevate(role: Role) -> Result<(), Refused> {
    #[cfg(target_os = "linux")]
    return linux::elevate(0, role);
    #[cfg(target_os = "macos")]
    return macos::elevate(role);
    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    {
        let _ = role;
        Err(Refused::Unsupported)
    }
}

/// [`elevate`] every thread of this process named `name` (as `pthread_setname_np` set it; on
/// Linux the name's first 15 bytes), returning how many took it. For threads a library creates
/// and this process cannot reach otherwise, such as `nusb`'s `nusb-events`.
///
/// # Errors
///
/// [`Refused`] when the threads cannot be listed, or the first refusal of a matching thread.
pub fn elevate_named(name: &str, role: Role) -> Result<usize, Refused> {
    #[cfg(target_os = "linux")]
    return linux::elevate_named(name, role);
    #[cfg(target_os = "macos")]
    return macos::elevate_named(name, role);
    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    {
        let _ = (name, role);
        Err(Refused::Unsupported)
    }
}

/// The kernel's CPU wake-latency request (`/dev/cpu_dma_latency`, a power-management request) held at zero: no core
/// enters an idle state it takes longer than that to wake from, for as long as this lives.
/// Every open bus holds one; the request ends when the last is dropped (the kernel takes the
/// strictest open request).
#[derive(Debug)]
pub struct WakeLatency {
    #[cfg(target_os = "linux")]
    _request: std::fs::File,
}

/// Ask the kernel to keep every core able to wake at once, while the returned guard lives.
///
/// # Errors
///
/// [`Refused::Denied`] with the `errno` when `/dev/cpu_dma_latency` cannot be written (it is
/// root-only unless the host grants it), [`Refused::Unsupported`] off Linux, where Mach's time
/// constraint already bounds the wake.
pub fn hold_wake_latency() -> Result<WakeLatency, Refused> {
    #[cfg(target_os = "linux")]
    {
        use std::io::Write;
        let denied = |error: std::io::Error| Refused::Denied(error.raw_os_error().unwrap_or(0));
        let mut request = std::fs::OpenOptions::new()
            .write(true)
            .open("/dev/cpu_dma_latency")
            .map_err(denied)?;
        request.write_all(&0_i32.to_ne_bytes()).map_err(denied)?;
        Ok(WakeLatency { _request: request })
    }
    #[cfg(not(target_os = "linux"))]
    Err(Refused::Unsupported)
}

/// Whether the calling thread runs under a real-time policy now.
#[must_use]
pub fn is_elevated() -> bool {
    #[cfg(target_os = "linux")]
    return linux::is_elevated();
    #[cfg(target_os = "macos")]
    return macos::is_elevated();
    #[cfg(not(any(target_os = "linux", target_os = "macos")))]
    false
}

/// The Mach time constraint for `role`: `(period, computation, constraint)`.
#[cfg_attr(not(target_os = "macos"), allow(dead_code))]
fn constraint(role: Role) -> (Duration, Duration, Duration) {
    match role {
        Role::AnswerPath => (
            Duration::from_millis(1),
            Duration::from_micros(250),
            Duration::from_millis(1),
        ),
        Role::ControlLoop { period } => (period, period / 4, period / 2),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const _: () = assert!(ANSWER_PATH_PRIORITY > CONTROL_LOOP_PRIORITY);

    #[test]
    fn a_loop_asks_for_a_quarter_of_its_period_within_half() {
        let (period, computation, constraint) = constraint(Role::ControlLoop {
            period: Duration::from_millis(20),
        });
        assert_eq!(
            (period, computation, constraint),
            (
                Duration::from_millis(20),
                Duration::from_millis(5),
                Duration::from_millis(10)
            )
        );
    }

    /// Either the host grants the policy and the thread reports it, or it refuses with a reason
    /// the caller can print; it never half-applies.
    #[test]
    fn elevation_is_taken_or_refused_with_a_reason() {
        std::thread::spawn(|| {
            match elevate(Role::ControlLoop {
                period: Duration::from_millis(20),
            }) {
                Ok(()) => assert!(is_elevated()),
                Err(refused) => {
                    assert!(!is_elevated());
                    assert!(!refused.to_string().is_empty());
                }
            }
        })
        .join()
        .expect("thread");
    }

    #[test]
    fn a_named_thread_is_found_and_no_other() {
        let (tx, rx) = std::sync::mpsc::channel::<()>();
        let (ready, started) = std::sync::mpsc::channel::<()>();
        let worker = std::thread::Builder::new()
            .name("sx-rt-probe".to_owned())
            .spawn(move || {
                // The name is set inside the new thread before this runs.
                let _ = ready.send(());
                let _ = rx.recv();
            })
            .expect("spawn");
        started.recv().expect("started");
        let found = elevate_named("sx-rt-probe", Role::AnswerPath);
        assert!(
            matches!(found, Ok(1) | Err(Refused::Denied(_))),
            "{found:?}"
        );
        assert_eq!(elevate_named("sx-no-such-thread", Role::AnswerPath), Ok(0));
        drop(tx);
        worker.join().expect("join");
    }
}
