//! The port a half-duplex serial-bus driver speaks through.
//!
//! A port is whatever carries bytes between the host and one daisy chain of servos: a USB
//! serial adapter, or a recording that replays a transcript. Its shape is the vendor
//! `PortHandler` the Feetech SDK drives (`clearPort`, `writePort`, `readPort`,
//! `setPacketTimeout`, `isPacketTimeout`), so a driver keeps its vendor's read loop unchanged.

use std::io;
use std::time::Duration;

/// A link to one serial bus.
pub trait SerialPort: Send {
    /// Discard any unread input, so a late reply from an earlier exchange cannot be taken for
    /// the answer to the next one.
    ///
    /// # Errors
    ///
    /// Returns the link's I/O error.
    fn clear_port(&mut self) -> io::Result<()>;

    /// Transmit one whole packet, returning how many bytes went out.
    ///
    /// # Errors
    ///
    /// Returns the link's I/O error.
    fn write_port(&mut self, packet: &[u8]) -> io::Result<usize>;

    /// The bytes that have arrived, at most `length`, without waiting for more.
    ///
    /// # Errors
    ///
    /// Returns the link's I/O error.
    fn read_port(&mut self, length: usize) -> io::Result<Vec<u8>>;

    /// The longest the link waits for any one status packet; a driver's stop budget is built
    /// from it.
    fn timeout(&self) -> Duration;

    /// Start waiting for a status packet of `packet_length` bytes. A link may wait less for a
    /// short packet, never longer than [`SerialPort::timeout`].
    fn set_packet_timeout(&mut self, packet_length: usize);

    /// Whether the wait started by the last [`SerialPort::set_packet_timeout`] has run out. A
    /// link replaying a transcript runs out as soon as it has nothing more to hand out.
    fn is_packet_timeout(&mut self) -> bool;
}
