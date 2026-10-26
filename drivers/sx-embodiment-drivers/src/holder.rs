//! Who holds a bus another process already has open.
//!
//! One process owns a bus. Every opener in the layer opens its device exclusively, so the
//! operating system refuses a second process: a serial tty by `TIOCEXCL` and an exclusive
//! `flock` (the `serialport` crate's default), a ZLG adapter by the USB interface claim, a
//! `SocketCAN` interface by our own lock file. When it refuses, the opener names the holder
//! here: on Linux by finding which processes have the device node (or lock file) open in
//! `/proc/<pid>/fd`; elsewhere, or when those descriptors belong to another user, the holder is
//! [`Holder::Unknown`]. The scan is only run on the refusal path, never while a bus is driven.

use std::fmt;
use std::fs;
use std::path::{Path, PathBuf};

/// One process holding a device.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Process {
    pub pid: u32,
    /// Its command name (`/proc/<pid>/comm`), empty when unreadable.
    pub name: String,
}

impl fmt::Display for Process {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.name.is_empty() {
            write!(formatter, "pid {}", self.pid)
        } else {
            write!(formatter, "pid {} ({})", self.pid, self.name)
        }
    }
}

/// Who holds a device the operating system refused to open for us.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Holder {
    /// The processes this host names as holding it.
    Processes(Vec<Process>),
    /// The host does not say: not Linux, another user's process, or a kernel driver.
    Unknown,
}

impl fmt::Display for Holder {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Processes(processes) => {
                formatter.write_str("held by ")?;
                for (index, process) in processes.iter().enumerate() {
                    if index > 0 {
                        formatter.write_str(", ")?;
                    }
                    write!(formatter, "{process}")?;
                }
                Ok(())
            }
            Self::Unknown => formatter.write_str("held by another process"),
        }
    }
}

impl Holder {
    /// A holder as macOS's I/O Registry records it (`UsbExclusiveOwner`, `pid 1234, name`),
    /// or [`Holder::Unknown`] when the text is not that shape.
    #[must_use]
    pub fn from_exclusive_owner(owner: &str) -> Self {
        let parsed = owner.strip_prefix("pid ").and_then(|rest| {
            let (pid, name) = rest.split_once(',').unwrap_or((rest, ""));
            Some(Process {
                pid: pid.trim().parse().ok()?,
                name: name.trim().to_owned(),
            })
        });
        parsed.map_or(Self::Unknown, |process| Self::Processes(vec![process]))
    }
}

/// Who holds `device` (a device node or lock file), as this host can tell.
#[must_use]
pub fn holder_of(device: &Path) -> Holder {
    if cfg!(target_os = "linux") {
        holder_in(Path::new("/proc"), device)
    } else {
        Holder::Unknown
    }
}

/// [`holder_of`] over a `/proc` rooted at `proc_root`: every process with a descriptor
/// linking to `device` (after resolving its symlinks), in pid order.
#[must_use]
pub fn holder_in(proc_root: &Path, device: &Path) -> Holder {
    let device = fs::canonicalize(device).unwrap_or_else(|_| device.to_path_buf());
    let Ok(entries) = fs::read_dir(proc_root) else {
        return Holder::Unknown;
    };
    let mut processes: Vec<Process> = entries
        .filter_map(Result::ok)
        .filter_map(|entry| {
            let pid: u32 = entry.file_name().to_str()?.parse().ok()?;
            let holds = fs::read_dir(entry.path().join("fd"))
                .ok()?
                .filter_map(Result::ok)
                .any(|fd| fs::read_link(fd.path()).is_ok_and(|target| same(&target, &device)));
            holds.then(|| Process {
                pid,
                name: fs::read_to_string(entry.path().join("comm"))
                    .map(|name| name.trim().to_owned())
                    .unwrap_or_default(),
            })
        })
        .collect();
    if processes.is_empty() {
        return Holder::Unknown;
    }
    processes.sort_by_key(|process| process.pid);
    Holder::Processes(processes)
}

/// Whether a descriptor's link target is `device`, which is already canonical.
fn same(target: &Path, device: &Path) -> bool {
    target == device || fs::canonicalize(target).is_ok_and(|target: PathBuf| target == device)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("sx-holder-{name}-{}", std::process::id()));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).expect("scratch");
        dir
    }

    #[cfg(unix)]
    #[test]
    fn the_processes_with_the_device_open_are_named_in_pid_order() {
        use std::os::unix::fs::symlink;
        let root = scratch("named");
        let device = root.join("ttyACM0");
        fs::write(&device, "").expect("device");
        let other = root.join("ttyACM1");
        fs::write(&other, "").expect("other");
        let proc_root = root.join("proc");
        for (pid, name, target) in [
            ("812", "sx-station\n", &device),
            ("77", "sx-jog\n", &device),
            ("90", "sx-record\n", &other),
        ] {
            let fd = proc_root.join(pid).join("fd");
            fs::create_dir_all(&fd).expect("fd");
            symlink(target, fd.join("5")).expect("link");
            fs::write(proc_root.join(pid).join("comm"), name).expect("comm");
        }
        fs::create_dir_all(proc_root.join("self")).expect("non-pid entry");
        let holder = holder_in(&proc_root, &device);
        assert_eq!(
            holder.to_string(),
            "held by pid 77 (sx-jog), pid 812 (sx-station)"
        );
        assert_eq!(
            holder_in(&proc_root, &root.join("ttyUSB9")),
            Holder::Unknown
        );
        assert_eq!(
            holder_in(&root.join("absent"), &device).to_string(),
            "held by another process"
        );
    }

    #[test]
    fn a_macos_exclusive_owner_names_its_process() {
        assert_eq!(
            Holder::from_exclusive_owner("pid 4321, sx-station-bench").to_string(),
            "held by pid 4321 (sx-station-bench)"
        );
        assert_eq!(
            Holder::from_exclusive_owner("pid 9").to_string(),
            "held by pid 9"
        );
        assert_eq!(Holder::from_exclusive_owner("kernel"), Holder::Unknown);
    }
}
