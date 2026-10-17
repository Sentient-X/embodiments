//! A global-time exposure on this host's clocks.
//!
//! librealsense's global timestamp is the device clock regressed onto the host's system
//! (wall) clock. [`HostSample`] pairs the host's monotonic and wall clocks at one instant;
//! [`captured_at`] subtracts a frame's age at that instant from both, so the exposure lands on
//! the monotonic clock the station's control loop runs on without ever comparing the two host
//! clocks directly. The error of the result is the SDK's regression error plus half the
//! sample's width.

use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use sx_embodiment_drivers::CapturedAt;
use thiserror::Error;

/// How far past the host sample a global timestamp may fall and still be the regression's own
/// uncertainty (`sx_drivers.realsense` holds the same bound).
pub const MAX_GLOBAL_TIME_LEAD: Duration = Duration::from_millis(10);

/// The host's two clocks read together.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HostSample {
    pub monotonic: Instant,
    pub wall: SystemTime,
}

impl HostSample {
    /// Read both clocks now: the monotonic reading is the midpoint of two taken around the
    /// wall-clock read.
    #[must_use]
    pub fn now() -> Self {
        let before = Instant::now();
        let wall = SystemTime::now();
        let after = Instant::now();
        Self {
            monotonic: before + (after - before) / 2,
            wall,
        }
    }
}

/// A global timestamp that cannot be an exposure seen at the host sample.
#[derive(Debug, Error, PartialEq)]
pub enum ClockError {
    #[error("global timestamp {timestamp_ms} ms is not a time")]
    NotATime { timestamp_ms: f64 },
    #[error(
        "global timestamp leads the host clock by {lead:?}, past {MAX_GLOBAL_TIME_LEAD:?}: the \
         device clock has not settled onto the host's"
    )]
    Leads { lead: Duration },
    #[error("an exposure {age:?} old predates this host's monotonic clock")]
    TooOld { age: Duration },
}

/// Where an exposure stamped `timestamp_ms` in librealsense's global-time domain (milliseconds
/// since the Unix epoch on the host's wall clock) sits on the host's clocks.
///
/// # Errors
///
/// Returns [`ClockError`] for a timestamp that is not a finite non-negative time, one more than
/// [`MAX_GLOBAL_TIME_LEAD`] after the sample, or one older than the monotonic clock itself.
pub fn captured_at(timestamp_ms: f64, host: HostSample) -> Result<CapturedAt, ClockError> {
    let exposed = Duration::try_from_secs_f64(timestamp_ms / 1_000.0)
        .ok()
        .and_then(|since_epoch| UNIX_EPOCH.checked_add(since_epoch))
        .ok_or(ClockError::NotATime { timestamp_ms })?;
    let age = match host.wall.duration_since(exposed) {
        Ok(age) => age,
        Err(ahead) if ahead.duration() <= MAX_GLOBAL_TIME_LEAD => Duration::ZERO,
        Err(ahead) => {
            return Err(ClockError::Leads {
                lead: ahead.duration(),
            });
        }
    };
    let monotonic = host
        .monotonic
        .checked_sub(age)
        .ok_or(ClockError::TooOld { age })?;
    let wall = host
        .wall
        .checked_sub(age)
        .ok_or(ClockError::TooOld { age })?;
    Ok(CapturedAt { monotonic, wall })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_at(wall_ms: u64) -> HostSample {
        HostSample {
            monotonic: Instant::now() + Duration::from_secs(1),
            wall: UNIX_EPOCH + Duration::from_millis(wall_ms),
        }
    }

    #[test]
    fn an_exposure_is_its_age_before_the_sample_on_both_clocks() {
        let host = sample_at(1_760_000_000_000);
        let at = captured_at(1_760_000_000_000.0 - 21.5, host).expect("captured");
        // An epoch timestamp in f64 milliseconds resolves to a fraction of a microsecond.
        let near =
            |age: Duration| age.abs_diff(Duration::from_micros(21_500)) < Duration::from_micros(1);
        assert!(near(host.monotonic - at.monotonic));
        assert!(near(host.wall.duration_since(at.wall).expect("past")));
        assert_eq!(
            host.monotonic - at.monotonic,
            host.wall.duration_since(at.wall).expect("past")
        );
    }

    #[test]
    fn a_small_lead_is_the_regression_and_a_large_one_is_refused() {
        let host = sample_at(1_760_000_000_000);
        let at = captured_at(1_760_000_000_004.0, host).expect("within the bound");
        assert_eq!((at.monotonic, at.wall), (host.monotonic, host.wall));
        assert!(matches!(
            captured_at(1_760_000_000_011.0, host),
            Err(ClockError::Leads { .. })
        ));
        for bad in [f64::NAN, f64::INFINITY, -1.0, f64::MAX] {
            assert!(matches!(
                captured_at(bad, host),
                Err(ClockError::NotATime { .. })
            ));
        }
    }
}
