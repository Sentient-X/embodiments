//! The port a CAN-bus driver speaks through.
//!
//! A port is whatever carries classic CAN frames between the host and one bus: a `SocketCAN`
//! interface, or a vendor's USB-to-CAN bridge. The driver never names the transport.

use std::io;
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
