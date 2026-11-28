//! Contact hold for the Spark's commissioned STS3215 fingers.
use std::time::{Duration, Instant};

pub(crate) const RUNTIME_CAP: u8 = 48;
const DETECT_TIME: Duration = Duration::from_millis(100);
const MAX_SAMPLE_GAP: Duration = Duration::from_millis(30);
// STS3215 output duty is in tenths of a percent. A single high sample
// during acceleration does not establish contact.
const CONTACT_LOAD: u16 = 200;
const STILL_TICKS: i32 = 5;
const CLOSE_TICKS: i32 = 8;
const MAX_PRELOAD_TICKS: i32 = 60;
const OPEN_TICKS: i32 = 4;

#[derive(Clone, Copy)]
pub(crate) struct Sample {
    pub at: Instant,
    pub position: i32,
    pub goal: i32,
    pub load: u16,
}

pub(crate) struct Finger {
    close: i32,
    pub baseline_cap: u16,
    hold_cap: u16,
    pub expected_cap: u16,
    pub cap_write_unconfirmed: bool,
    latest: Option<Sample>,
    contact: Option<Sample>,
    hold: Option<(i32, i32)>,
}

impl Finger {
    pub fn new(close: i32, baseline_cap: u16, hold_cap: u16) -> Self {
        Self {
            close,
            baseline_cap,
            hold_cap,
            expected_cap: baseline_cap,
            cap_write_unconfirmed: false,
            latest: None,
            contact: None,
            hold: None,
        }
    }

    pub fn needs_cap_restore(&self) -> bool {
        self.cap_write_unconfirmed || self.expected_cap != self.baseline_cap
    }

    pub fn reset(&mut self) {
        self.latest = None;
        self.contact = None;
        self.hold = None;
    }

    pub fn reenable(&mut self) -> u16 {
        self.observe(None);
        if self.hold.is_some() {
            self.hold_cap
        } else {
            self.baseline_cap
        }
    }

    pub fn observe(&mut self, sample: Option<Sample>) {
        let continuous = self.latest.zip(sample).is_some_and(|(last, next)| {
            next.at.saturating_duration_since(last.at) <= MAX_SAMPLE_GAP
        });
        self.latest = sample;
        if !continuous {
            self.contact = None;
        }
        let Some(sample) = sample else {
            return;
        };
        if sample.load < CONTACT_LOAD || self.close * (sample.goal - sample.position) < CLOSE_TICKS
        {
            self.contact = None;
        } else if self
            .contact
            .is_none_or(|first| (sample.position - first.position).abs() > STILL_TICKS)
        {
            self.contact = Some(sample);
        }
    }

    /// Return the goal and, when transitioning, the cap the driver must acknowledge
    /// before applying it. Restore normal effort after feedback proves an applied
    /// opening goal beyond the contact point, including when the next goal re-grasps.
    pub fn plan(&mut self, requested: i32, now: Instant) -> (i32, Option<u16>) {
        let sample = self
            .latest
            .filter(|sample| now.saturating_duration_since(sample.at) <= MAX_SAMPLE_GAP);
        if let Some((contact, goal)) = self.hold {
            if sample.is_some_and(|sample| {
                self.close * (sample.goal - contact) <= -OPEN_TICKS
                    && self.close * (sample.goal - sample.position) <= 0
            }) {
                self.reset();
                return (requested, Some(self.baseline_cap));
            }
            // Pass through opening. With further closing input the preload stays at
            // the contact location, even if the object or encoder moves a little.
            return (
                self.close * (self.close * requested).min(self.close * goal),
                None,
            );
        }
        if let Some(sample) = sample
            && self.close * (requested - sample.position) >= CLOSE_TICKS
            && self
                .contact
                .is_some_and(|first| sample.at.saturating_duration_since(first.at) >= DETECT_TIME)
        {
            // Keep the previously applied closing demand within the measured contact
            // stroke and current request. The runtime cap limits the resulting effort.
            let goal = self.close
                * (self.close * requested)
                    .min(self.close * sample.goal)
                    .min(self.close * sample.position + MAX_PRELOAD_TICKS);
            self.hold = Some((sample.position, goal));
            return (goal, Some(self.hold_cap));
        }
        // A new opening command cancels pending contact even before its feedback arrives.
        if sample.is_none_or(|sample| self.close * (requested - sample.position) < CLOSE_TICKS) {
            self.contact = None;
        }
        (requested, None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contact_reduces_effort_until_opening_is_confirmed_in_both_directions() {
        for close in [-1, 1] {
            let mut finger = Finger::new(close, 500, 180);
            let start = Instant::now();
            let position = 1800;
            let closing = position + 100 * close;
            let hold = position + 60 * close;
            let mut sample = Sample {
                at: start,
                position,
                goal: closing,
                load: 380,
            };
            for step in 0..=10 {
                sample.at = start + Duration::from_millis(step * 10);
                finger.observe(Some(sample));
                let expected = if step == 10 {
                    (hold, Some(180))
                } else {
                    (closing, None)
                };
                assert_eq!(finger.plan(closing, sample.at), expected);
            }
            sample.at += Duration::from_millis(10);
            sample.position += 3 * close;
            sample.goal = hold;
            finger.observe(Some(sample));
            assert_eq!(finger.plan(closing + 30 * close, sample.at), (hold, None));
            let opening = position - 15 * close;
            assert_eq!(finger.plan(opening, sample.at), (opening, None));
            sample.at += Duration::from_millis(10);
            sample.goal = opening;
            finger.observe(Some(sample));
            assert_eq!(finger.plan(closing, sample.at), (closing, Some(500)));
        }
    }

    #[test]
    fn a_spike_motion_opening_or_a_gap_cannot_build_contact_evidence() {
        let start = Instant::now();
        for scenario in ["spike", "moving", "opening", "gap"] {
            let mut finger = Finger::new(1, 500, 180);
            for step in 0..=20 {
                let at = start + Duration::from_millis(step * 10);
                let position = if scenario == "moving" {
                    1700 + i32::try_from(step).unwrap() * 3
                } else {
                    1700
                };
                let goal = if scenario == "opening" { 1600 } else { 1900 };
                let load = if scenario == "spike" && step != 5 {
                    10
                } else {
                    380
                };
                let sample = (scenario != "gap" || step % 8 != 0).then_some(Sample {
                    at,
                    position,
                    goal,
                    load,
                });
                finger.observe(sample);
                assert_eq!(finger.plan(goal, at), (goal, None), "{scenario} at {step}");
            }
        }
    }
}
