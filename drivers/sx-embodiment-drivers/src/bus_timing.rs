//! How long each chain's command takes on its bus, kept without allocating per command.

use std::time::Duration;

/// Histogram resolution: 250 µs.
const BUCKET_US: u64 = 250;
/// Buckets: up to 32 ms resolved; beyond counts in the last.
const BUCKETS: usize = 128;

/// One chain's command times on its bus: how many, their p50 and p99, and the longest. A tick's
/// commands run one chain after another, so the slow chain is the one whose time fills the
/// period (10-28: the Spark's ticks overran by half a period at median).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BusTiming {
    /// The chain's first joint, naming which chain this is.
    pub first_joint_index: usize,
    pub commands: u64,
    pub max: Duration,
    buckets: [u32; BUCKETS],
}

impl BusTiming {
    #[must_use]
    pub const fn new(first_joint_index: usize) -> Self {
        Self {
            first_joint_index,
            commands: 0,
            max: Duration::ZERO,
            buckets: [0; BUCKETS],
        }
    }

    /// Record one command that took `took` on this chain's bus.
    pub fn record(&mut self, took: Duration) {
        let index = usize::try_from(took.as_micros() / u128::from(BUCKET_US))
            .unwrap_or(usize::MAX)
            .min(BUCKETS - 1);
        self.buckets[index] = self.buckets[index].saturating_add(1);
        self.commands = self.commands.saturating_add(1);
        self.max = self.max.max(took);
    }

    /// The command time at quantile `q` (0..=1), to the bucket's upper edge.
    #[must_use]
    pub fn quantile(&self, q: f64) -> Duration {
        if self.commands == 0 {
            return Duration::ZERO;
        }
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            clippy::cast_precision_loss
        )]
        let rank = ((self.commands as f64) * q).ceil().max(1.0) as u64;
        let mut seen = 0_u64;
        for (index, count) in self.buckets.iter().enumerate() {
            seen += u64::from(*count);
            if seen >= rank {
                let upper = Duration::from_micros(BUCKET_US * (index as u64 + 1));
                return upper.min(self.max);
            }
        }
        self.max
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quantiles_come_from_the_buckets_and_the_max_is_exact() {
        let mut timing = BusTiming::new(3);
        for ms in 1..=100 {
            timing.record(Duration::from_micros(ms * 100));
        }
        assert_eq!(timing.commands, 100);
        assert_eq!(timing.max, Duration::from_millis(10));
        let median = timing.quantile(0.5);
        assert!(median >= Duration::from_micros(5000) && median <= Duration::from_micros(5250));
        assert!(timing.quantile(0.99) <= Duration::from_millis(10));
    }
}
