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

/// One classic CAN data frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CanFrame {
    /// The arbitration id: 11-bit standard or 29-bit extended, as the bus's protocol uses.
    pub id: u32,
    pub data: [u8; 8],
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
}
