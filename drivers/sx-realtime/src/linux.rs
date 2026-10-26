//! `SCHED_FIFO` by thread id: the crate's Linux system calls.

#![allow(unsafe_code)]

use crate::{ANSWER_PATH_PRIORITY, CONTROL_LOOP_PRIORITY, Refused, Role};

/// The longest thread name Linux keeps (`TASK_COMM_LEN` less its NUL).
const COMM_LEN: usize = 15;

fn priority(role: Role) -> i32 {
    match role {
        Role::AnswerPath => ANSWER_PATH_PRIORITY,
        Role::ControlLoop { .. } => CONTROL_LOOP_PRIORITY,
    }
}

/// `tid` (0: the calling thread) under `SCHED_FIFO` at `role`'s priority.
pub fn elevate(tid: libc::pid_t, role: Role) -> Result<(), Refused> {
    let param = libc::sched_param {
        sched_priority: priority(role),
    };
    // SAFETY: `param` is a valid `sched_param` for the whole call; on Linux a thread id names
    // one thread of this process, and 0 the caller.
    let status = unsafe {
        libc::sched_setscheduler(
            tid,
            libc::SCHED_FIFO | libc::SCHED_RESET_ON_FORK,
            &raw const param,
        )
    };
    if status == 0 {
        Ok(())
    } else {
        Err(Refused::Denied(
            std::io::Error::last_os_error().raw_os_error().unwrap_or(0),
        ))
    }
}

/// Whether `comm` (a `/proc/.../comm` line) is the thread named `name`.
fn named(comm: &str, name: &str) -> bool {
    let comm = comm.trim_end_matches('\n');
    let name = &name.as_bytes()[..name.len().min(COMM_LEN)];
    comm.as_bytes() == name
}

pub fn elevate_named(name: &str, role: Role) -> Result<usize, Refused> {
    let tasks = std::fs::read_dir("/proc/self/task")
        .map_err(|error| Refused::Denied(error.raw_os_error().unwrap_or(0)))?;
    let mut taken = 0;
    let mut refused = None;
    for task in tasks.flatten() {
        let Ok(comm) = std::fs::read_to_string(task.path().join("comm")) else {
            continue;
        };
        let Some(tid) = task.file_name().to_str().and_then(|tid| tid.parse().ok()) else {
            continue;
        };
        if named(&comm, name) {
            match elevate(tid, role) {
                Ok(()) => taken += 1,
                Err(error) => refused = refused.or(Some(error)),
            }
        }
    }
    refused.map_or(Ok(taken), Err)
}

pub fn is_elevated() -> bool {
    // SAFETY: no arguments but the calling thread's id.
    let policy = unsafe { libc::sched_getscheduler(0) };
    policy & !libc::SCHED_RESET_ON_FORK == libc::SCHED_FIFO
}

#[cfg(test)]
mod tests {
    use super::named;

    #[test]
    fn a_name_matches_its_first_fifteen_bytes() {
        assert!(named("nusb-events\n", "nusb-events"));
        assert!(named("zlg-usbcanfd-io\n", "zlg-usbcanfd-io"));
        assert!(named("sx-station-benc\n", "sx-station-bench"));
        assert!(!named("nusb-events-2\n", "nusb-events"));
    }
}
