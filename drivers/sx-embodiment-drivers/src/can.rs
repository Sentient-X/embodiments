//! The port a CAN-bus driver speaks through.
//!
//! A port is whatever carries classic CAN frames between the host and one bus: a `SocketCAN`
//! interface, or a vendor's USB-to-CAN bridge. The driver never names the transport.

use std::io;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// What a chain budgets for one frame it sends and the answer it reads when the link names
/// nothing longer: a USB serial bridge's, under 0.6 ms of serial time for the transmit packet
/// and the answer at 921600 baud, and up to a 1 ms USB full-speed frame of latency each way.
pub const FRAME_ALLOWANCE: Duration = Duration::from_millis(2);

/// How many of a link's latest answer times a [`RoundTripWindow`] keeps for its percentiles:
/// a few seconds of a body's ticks, in 4 KiB held inline.
pub const ROUND_TRIP_SAMPLES: usize = 512;

/// How long a link's commands waited for their answers over one period: from the moment the
/// link took the command's frame to the moment the link handed over the answer the motor
/// protocol paired with it, host scheduling included, as `sx-can-bench` measures a round trip.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RoundTrip {
    /// Answers paired with their command in the period.
    pub answers: u64,
    /// The median of the period's latest [`ROUND_TRIP_SAMPLES`] answers.
    pub p50: Duration,
    /// The 99th percentile (nearest rank) of the same answers.
    pub p99: Duration,
    /// The slowest answer of the period, every answer counted.
    pub max: Duration,
}

/// The answer times a link was told of since it was last read ([`CanPort::answered`],
/// [`CanPort::round_trip`], [`CanPort::round_trip_max`]). Fixed size: recording writes one
/// slot of a ring and allocates nothing, so it runs on the control thread, and so does reading
/// the period's slowest ([`RoundTripWindow::take_max`]); the percentiles
/// ([`RoundTripWindow::take`]) sort a 4 KiB copy, for a bench or a tool, never a control loop.
#[derive(Clone, Debug)]
pub struct RoundTripWindow {
    nanos: [u64; ROUND_TRIP_SAMPLES],
    /// The slot the next answer is written to.
    next: usize,
    /// How many slots hold an answer of this period (at most [`ROUND_TRIP_SAMPLES`]).
    held: usize,
    answers: u64,
    max: Duration,
}

impl Default for RoundTripWindow {
    fn default() -> Self {
        Self {
            nanos: [0; ROUND_TRIP_SAMPLES],
            next: 0,
            held: 0,
            answers: 0,
            max: Duration::ZERO,
        }
    }
}

impl RoundTripWindow {
    /// One command answered `round_trip` after it went out; the oldest answer leaves the
    /// percentiles once the ring is full, never the period's slowest.
    pub fn record(&mut self, round_trip: Duration) {
        self.nanos[self.next] = u64::try_from(round_trip.as_nanos()).unwrap_or(u64::MAX);
        self.next = (self.next + 1) % ROUND_TRIP_SAMPLES;
        self.held = (self.held + 1).min(ROUND_TRIP_SAMPLES);
        self.answers = self.answers.saturating_add(1);
        self.max = self.max.max(round_trip);
    }

    /// The period's slowest answer, and a new period begun; `None` when nothing was answered.
    /// Constant time, no copy and no sort: what a control thread reads.
    pub fn take_max(&mut self) -> Option<Duration> {
        let max = (self.answers > 0).then_some(self.max);
        self.restart();
        max
    }

    fn restart(&mut self) {
        self.next = 0;
        self.held = 0;
        self.answers = 0;
        self.max = Duration::ZERO;
    }

    /// The period's answer times, and a new period begun; `None` when nothing was answered.
    /// Sorts a copy of the ring: for a bench or a tool, not a control thread.
    pub fn take(&mut self) -> Option<RoundTrip> {
        if self.answers == 0 {
            return None;
        }
        let mut sorted = self.nanos;
        let held = &mut sorted[..self.held];
        held.sort_unstable();
        let rank = |percent: usize| {
            Duration::from_nanos(held[(held.len() * percent).div_ceil(100).saturating_sub(1)])
        };
        let taken = RoundTrip {
            answers: self.answers,
            p50: rank(50),
            p99: rank(99),
            max: self.max,
        };
        self.restart();
        Some(taken)
    }
}

/// One classic CAN data frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CanFrame {
    /// The arbitration id: 11-bit standard or 29-bit extended, as the bus's protocol uses.
    pub id: u32,
    pub data: [u8; 8],
}

/// What a link knows about its bus, in the words a person at the robot acts on: the device's
/// health line, for a station to report as the device's health. Diagnostic only: nothing may
/// gate motion on it (a chain's own answers decide that). Closed: every transport
/// maps what it sees onto these.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LinkHealth {
    /// Frames go out and answers come back.
    Answering,
    /// Open, and nothing has failed, but nothing has been received yet.
    Quiet,
    /// The adapter works, but nothing on the bus acknowledges what it sends: the cable to the
    /// motors, its termination or the motors' power. Clears by itself when the bus answers.
    NothingAnswering { bus: String },
    /// The controller reports errors on the bus (bus-off, error-passive): wiring, termination
    /// or bitrate. `detail` names what it saw.
    BusErrors { bus: String, detail: String },
    /// The adapter stopped answering its own commands: unplug it and plug it back in.
    AdapterLost { bus: String, detail: String },
    /// This link reports no health.
    Unknown,
}

impl LinkHealth {
    /// Whether the bus can carry a robot's traffic now.
    #[must_use]
    pub const fn usable(&self) -> bool {
        matches!(self, Self::Answering | Self::Quiet)
    }
}

impl std::fmt::Display for LinkHealth {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Answering => f.write_str("Answering"),
            Self::Quiet => f.write_str("Open; nothing received yet"),
            Self::NothingAnswering { bus } => write!(
                f,
                "Nothing on {bus} is answering: check the cable to the motors, its termination and motor power"
            ),
            Self::BusErrors { bus, detail } => write!(
                f,
                "{bus} has bus errors ({detail}): check the wiring, the termination at both ends and the bitrate"
            ),
            Self::AdapterLost { bus, detail } => write!(
                f,
                "The adapter for {bus} stopped answering ({detail}): unplug it and plug it back in"
            ),
            Self::Unknown => f.write_str("No health reported"),
        }
    }
}

/// A link to one CAN bus.
///
/// A refused send carries its reason as an `io::Error` whose kind every transport keeps the
/// same: `WouldBlock`, the transmit queue is full, send again once the bus drains;
/// `Interrupted`, the link recovered itself and dropped what was queued, send again;
/// `NotConnected`, the link needs action outside this process (a power cycle, a bus-off
/// controller, an interface that is down).
pub trait CanPort: Send {
    /// Transmit one frame.
    ///
    /// # Errors
    ///
    /// Returns the link's I/O error.
    fn send(&mut self, frame: &CanFrame) -> io::Result<()>;

    /// The next frame the link has received, or `None` when nothing is pending.
    ///
    /// # Errors
    ///
    /// Returns the link's I/O error.
    fn receive(&mut self) -> io::Result<Option<CanFrame>>;

    /// The next frame received before `deadline`, waiting for one; `None` once it passes.
    /// A link that can block until a frame arrives does, instead of polling; this default polls
    /// [`CanPort::receive`] without sleeping, as a control loop would.
    ///
    /// # Errors
    ///
    /// Returns the link's I/O error.
    fn receive_until(&mut self, deadline: Instant) -> io::Result<Option<CanFrame>> {
        loop {
            if let Some(frame) = self.receive()? {
                return Ok(Some(frame));
            }
            if Instant::now() >= deadline {
                return Ok(None);
            }
            std::hint::spin_loop();
        }
    }

    /// Wait as the vendor driver's command sequencing waits. A link replaying a transcript
    /// records the wait instead of sleeping.
    fn sleep(&mut self, duration: Duration) {
        std::thread::sleep(duration);
    }

    /// The longest one frame and its answer can hold a chain on this link, waits included: a
    /// link whose `send` waits for room in a transmit queue says how long, so a chain's stop
    /// budget stays honest. [`FRAME_ALLOWANCE`] unless the link waits longer.
    fn frame_allowance(&self) -> Duration {
        FRAME_ALLOWANCE
    }

    /// The link's own counts, by name, for a diagnostic report (frames lost, refusals, error
    /// frames, how late the host took answers): what tells a host stall from a bus fault, on
    /// whichever transport. Read between runs, never per tick. Empty where the link keeps none.
    fn counters(&self) -> Vec<(&'static str, u64)> {
        Vec::new()
    }

    /// The bus's health in plain words ([`LinkHealth`]); cheap, never touches the bus.
    fn health(&self) -> LinkHealth {
        LinkHealth::Unknown
    }

    /// One of the link's commands was answered `round_trip` after it went out, as the motor
    /// protocol that paired the answer with its command measured it (the link cannot: only the
    /// protocol knows which frame answers which). Called on the control thread; a link that
    /// keeps answer times records them in a [`RoundTripWindow`], allocating nothing. Ignored
    /// where the link keeps none.
    fn answered(&mut self, round_trip: Duration) {
        let _ = round_trip;
    }

    /// The link's answer times since this or [`CanPort::round_trip_max`] was last called,
    /// which begins a new period: for a bench or a tool to read beside [`CanPort::counters`];
    /// it sorts, so never on a control thread. `None` when nothing was answered or the link
    /// keeps no answer times.
    fn round_trip(&mut self) -> Option<RoundTrip> {
        None
    }

    /// The slowest answer since this or [`CanPort::round_trip`] was last called, which begins
    /// a new period: constant time, no copy and no sort, so a control thread may read it
    /// between two frames. `None` when nothing was answered or the link keeps no answer times.
    fn round_trip_max(&mut self) -> Option<Duration> {
        None
    }
}

/// A port chosen at run time, as a station opens whichever bridge its commissioning names.
impl<P: CanPort + ?Sized> CanPort for Box<P> {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        (**self).send(frame)
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        (**self).receive()
    }

    fn receive_until(&mut self, deadline: Instant) -> io::Result<Option<CanFrame>> {
        (**self).receive_until(deadline)
    }

    fn sleep(&mut self, duration: Duration) {
        (**self).sleep(duration);
    }

    fn frame_allowance(&self) -> Duration {
        (**self).frame_allowance()
    }

    fn counters(&self) -> Vec<(&'static str, u64)> {
        (**self).counters()
    }

    fn health(&self) -> LinkHealth {
        (**self).health()
    }

    fn answered(&mut self, round_trip: Duration) {
        (**self).answered(round_trip);
    }

    fn round_trip(&mut self) -> Option<RoundTrip> {
        (**self).round_trip()
    }

    fn round_trip_max(&mut self) -> Option<Duration> {
        (**self).round_trip_max()
    }
}

/// One CAN link shared by chains that take turns on it, as a body's devices behind one bridge
/// do (the B601-DM's arm and gripper). A chain's reads drop frames from motors it does not
/// drive. `sleep` holds the link while it waits, so a chain's sequencing is never interleaved.
pub struct SharedCanPort<P>(Arc<Mutex<P>>);

impl<P> SharedCanPort<P> {
    #[must_use]
    pub fn new(port: P) -> Self {
        Self(Arc::new(Mutex::new(port)))
    }
}

impl<P> Clone for SharedCanPort<P> {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<P: CanPort> CanPort for SharedCanPort<P> {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        self.0
            .lock()
            .map_err(|_| io::Error::other("the shared CAN port is poisoned"))?
            .send(frame)
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        self.0
            .lock()
            .map_err(|_| io::Error::other("the shared CAN port is poisoned"))?
            .receive()
    }

    /// Holds the link while it waits, as `sleep` does.
    fn receive_until(&mut self, deadline: Instant) -> io::Result<Option<CanFrame>> {
        self.0
            .lock()
            .map_err(|_| io::Error::other("the shared CAN port is poisoned"))?
            .receive_until(deadline)
    }

    fn sleep(&mut self, duration: Duration) {
        match self.0.lock() {
            Ok(mut port) => port.sleep(duration),
            Err(_) => std::thread::sleep(duration),
        }
    }

    fn frame_allowance(&self) -> Duration {
        self.0
            .lock()
            .map_or(FRAME_ALLOWANCE, |port| port.frame_allowance())
    }

    fn counters(&self) -> Vec<(&'static str, u64)> {
        self.0
            .lock()
            .map_or_else(|_| Vec::new(), |port| port.counters())
    }

    /// `Unknown` while another call holds the link (the lock is taken per call, and a
    /// `receive_until` or `sleep` keeps it until its deadline): a health poll never waits on
    /// the bus.
    fn health(&self) -> LinkHealth {
        self.0
            .try_lock()
            .map_or(LinkHealth::Unknown, |port| port.health())
    }

    /// The lock is taken per call, so another chain's `receive_until` may hold the link until
    /// its deadline: the sample is dropped rather than the control thread kept waiting.
    fn answered(&mut self, round_trip: Duration) {
        if let Ok(mut port) = self.0.try_lock() {
            port.answered(round_trip);
        }
    }

    fn round_trip(&mut self) -> Option<RoundTrip> {
        self.0.lock().ok()?.round_trip()
    }

    /// Read on a control thread, so it never waits behind another chain's `receive_until`:
    /// `None` then, and the period runs on until the next read.
    fn round_trip_max(&mut self) -> Option<Duration> {
        self.0.try_lock().ok()?.round_trip_max()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_period_reports_its_slowest_answer_and_its_percentiles() {
        let mut window = RoundTripWindow::default();
        assert_eq!(window.take(), None, "nothing answered: nothing claimed");
        for micros in 1..=100 {
            window.record(Duration::from_micros(micros));
        }
        assert_eq!(
            window.take(),
            Some(RoundTrip {
                answers: 100,
                p50: Duration::from_micros(50),
                p99: Duration::from_micros(99),
                max: Duration::from_micros(100),
            })
        );
        assert_eq!(window.take(), None, "a read begins a new period");
        window.record(Duration::from_micros(479));
        let one = window.take().expect("one answer");
        let rtt = Duration::from_micros(479);
        assert_eq!((one.answers, one.p50, one.p99, one.max), (1, rtt, rtt, rtt));
    }

    #[test]
    fn the_slowest_answer_reads_without_sorting_and_begins_a_new_period() {
        let mut window = RoundTripWindow::default();
        assert_eq!(window.take_max(), None, "nothing answered: nothing claimed");
        for micros in [300, 2_100, 450] {
            window.record(Duration::from_micros(micros));
        }
        assert_eq!(window.take_max(), Some(Duration::from_micros(2_100)));
        assert_eq!(window.take_max(), None, "a read begins a new period");
        assert_eq!(window.take(), None, "for the percentiles too");
    }

    #[test]
    fn a_full_ring_keeps_its_size_and_the_periods_slowest() {
        let mut window = RoundTripWindow::default();
        let size = std::mem::size_of_val(&window);
        window.record(Duration::from_millis(9));
        for _ in 0..ROUND_TRIP_SAMPLES * 3 {
            window.record(Duration::from_micros(400));
        }
        assert_eq!(std::mem::size_of_val(&window), size, "nothing grows");
        assert_eq!(window.held, ROUND_TRIP_SAMPLES);
        let taken = window.take().expect("answered");
        assert_eq!(taken.answers, 1 + 3 * ROUND_TRIP_SAMPLES as u64);
        assert_eq!(
            taken.p99,
            Duration::from_micros(400),
            "the slow one left the ring"
        );
        assert_eq!(
            taken.max,
            Duration::from_millis(9),
            "but not the period's slowest"
        );
    }
}
