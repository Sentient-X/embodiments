//! A bus borrowed from the station running on this host, so calibrating never stops it.
//!
//! One process owns a bus. The station lends one to a tool through files in its loans
//! directory, which it makes and owns: the tool writes `<port>.borrow` naming its process; the
//! station, between sessions, proves the body stopped, closes that bus (its axes stay in the
//! robot as unavailable data, and no session is admitted meanwhile) and writes `<port>.lent`.
//! The tool then opens the bus as usual. Dropping the [`Loan`] removes `<port>.borrow`, and
//! the station takes the bus back, with any calibration the run published. A tool that dies
//! without returning it is noticed by its process id.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

/// Where an installed station takes requests for its buses (`fleet/station`, `BusLoans`).
pub const STATION_LOANS: &str = "/var/lib/sx-station/loans";

/// A bus the station lent this process; returned when dropped.
#[derive(Debug)]
pub struct Loan {
    request: PathBuf,
}

impl Loan {
    /// Ask the station for `port`'s bus and wait up to `wait` for it, giving up when `stop`
    /// is set (a signal).
    ///
    /// # Errors
    ///
    /// No station runs here (no loans directory), the request cannot be written, or the station
    /// did not lend the bus in time (it lends only between sessions); nothing is left behind.
    pub fn borrow(
        root: &Path,
        port: &str,
        wait: Duration,
        stop: &AtomicUsize,
    ) -> Result<Self, String> {
        if !root.is_dir() {
            return Err(format!(
                "{} is missing: no station runs here to lend {port}",
                root.display()
            ));
        }
        let request = root.join(format!("{port}.borrow"));
        let lent = root.join(format!("{port}.lent"));
        let staging = root.join(format!(".{port}.borrow.next"));
        std::fs::write(&staging, std::process::id().to_string())
            .and_then(|()| std::fs::rename(&staging, &request))
            .map_err(|error| format!("{}: {error}", request.display()))?;
        let loan = Self { request };
        let started = Instant::now();
        // The answer names the run it lends to, so no run takes an answer meant for another.
        let mine = std::process::id().to_string();
        while std::fs::read_to_string(&lent)
            .ok()
            .as_deref()
            .map(str::trim)
            != Some(mine.as_str())
        {
            if stop.load(Ordering::SeqCst) != 0 {
                return Err(format!(
                    "stopped while waiting for the station to lend {port}"
                ));
            }
            if started.elapsed() > wait {
                return Err(format!(
                    "the station did not lend {port} within {} s: it lends a bus only between \
                     sessions, while no fault holds the robot",
                    wait.as_secs()
                ));
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        Ok(loan)
    }
}

impl Drop for Loan {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.request);
    }
}
