//! One of our processes per interface: an exclusive advisory lock held while a port is open.
//!
//! `SocketCAN` lets any number of sockets bind one interface, so two of our processes could
//! drive one bus at once, each sure it alone commands the motors. Opening a port takes an
//! exclusive, non-blocking `flock` on `/run/lock/sx-socketcan-<interface>.lock` (or under `/tmp`
//! where `/run/lock` cannot be written) and holds it for the port's life; the kernel releases it
//! when the process ends, however it ends. A second open is refused, naming the interface and
//! the lock. It excludes only processes that take this lock: another driver on the same bus,
//! such as the robot's own controller, `candump`, or a process that found `/run/lock` writable
//! where this one fell back to `/tmp`, is not excluded.

#![allow(unsafe_code)]

use std::fs::{File, OpenOptions};
use std::io;
use std::os::fd::AsRawFd;
use std::path::{Path, PathBuf};

use crate::SocketCanError;

/// Where locks are taken, in order of preference.
const LOCK_DIRS: [&str; 2] = ["/run/lock", "/tmp"];

/// The held lock of one interface; dropping it releases the interface.
#[derive(Debug)]
pub struct InterfaceLock {
    _file: File,
    path: PathBuf,
}

impl InterfaceLock {
    /// Take `interface`'s lock.
    ///
    /// # Errors
    ///
    /// [`SocketCanError::Busy`] when another open port holds it, and [`SocketCanError::Io`]
    /// when no lock file can be opened.
    pub fn acquire(interface: &str) -> Result<Self, SocketCanError> {
        let dirs: Vec<&Path> = LOCK_DIRS.iter().map(Path::new).collect();
        Self::acquire_in(&dirs, interface)
    }

    /// The lock file this lock holds.
    #[must_use]
    pub fn path(&self) -> &Path {
        &self.path
    }

    pub(crate) fn acquire_in(dirs: &[&Path], interface: &str) -> Result<Self, SocketCanError> {
        let io = |source: io::Error| SocketCanError::Io {
            interface: interface.to_owned(),
            source,
        };
        let mut last = None;
        for dir in dirs {
            let path = dir.join(format!("sx-socketcan-{interface}.lock"));
            let file = match OpenOptions::new()
                .create(true)
                .truncate(false)
                .write(true)
                .open(&path)
            {
                Ok(file) => file,
                Err(error) => {
                    last = Some(error);
                    continue;
                }
            };
            // SAFETY: `flock` takes a descriptor `file` owns for the whole call, and no pointer.
            let locked = unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) };
            if locked != 0 {
                let error = io::Error::last_os_error();
                return Err(if error.kind() == io::ErrorKind::WouldBlock {
                    SocketCanError::Busy {
                        interface: interface.to_owned(),
                        holder: sx_embodiment_drivers::holder_of(&path),
                        lock: path,
                    }
                } else {
                    io(error)
                });
            }
            return Ok(Self { _file: file, path });
        }
        Err(io(
            last.unwrap_or_else(|| io::Error::other("no lock directory"))
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_second_open_of_one_interface_is_refused_until_the_first_is_dropped() {
        let dir = std::env::temp_dir().join(format!("sx-socketcan-lock-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("dir");
        let missing = dir.join("absent");
        let dirs = [missing.as_path(), dir.as_path()];
        let held = InterfaceLock::acquire_in(&dirs, "can1").expect("the first open");
        assert_eq!(held.path(), dir.join("sx-socketcan-can1.lock"));
        match InterfaceLock::acquire_in(&dirs, "can1") {
            Err(SocketCanError::Busy {
                interface,
                lock,
                holder,
            }) => {
                assert_eq!(interface, "can1");
                assert_eq!(lock, dir.join("sx-socketcan-can1.lock"));
                // Linux names the holder from /proc: here, this test process itself.
                if cfg!(target_os = "linux") {
                    assert!(
                        holder
                            .to_string()
                            .contains(&format!("pid {}", std::process::id())),
                        "{holder}"
                    );
                }
            }
            other => panic!("expected the interface busy, got {other:?}"),
        }
        InterfaceLock::acquire_in(&dirs, "can0").expect("another interface");
        drop(held);
        InterfaceLock::acquire_in(&dirs, "can1").expect("after the first is dropped");
        let _ = std::fs::remove_dir_all(dir);
    }
}
