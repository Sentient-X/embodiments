//! The port a CAN-bus driver speaks through.
//!
//! A port is whatever carries classic CAN frames between the host and one bus: a `SocketCAN`
//! interface, or a vendor's USB-to-CAN bridge. The driver never names the transport.

use std::io;
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// One classic CAN data frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CanFrame {
    /// The arbitration id: 11-bit standard or 29-bit extended, as the bus's protocol uses.
    pub id: u32,
    pub data: [u8; 8],
}

/// A link to one CAN bus.
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

    /// Wait as the vendor driver's command sequencing waits. A link replaying a transcript
    /// records the wait instead of sleeping.
    fn sleep(&mut self, duration: Duration) {
        std::thread::sleep(duration);
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

    fn sleep(&mut self, duration: Duration) {
        match self.0.lock() {
            Ok(mut port) => port.sleep(duration),
            Err(_) => std::thread::sleep(duration),
        }
    }
}
