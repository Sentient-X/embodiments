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

    /// Copy the bytes that have arrived into `buffer`, at most its length, without waiting for
    /// more, and return how many: `io::Read`'s shape, so a read allocates nothing.
    ///
    /// # Errors
    ///
    /// Returns the link's I/O error.
    fn read_port(&mut self, buffer: &mut [u8]) -> io::Result<usize>;

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

/// A link lent for a while, as a read-only scan borrows the port a caller keeps using.
impl<P: SerialPort + ?Sized> SerialPort for &mut P {
    fn clear_port(&mut self) -> io::Result<()> {
        (**self).clear_port()
    }

    fn write_port(&mut self, packet: &[u8]) -> io::Result<usize> {
        (**self).write_port(packet)
    }

    fn read_port(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        (**self).read_port(buffer)
    }

    fn timeout(&self) -> Duration {
        (**self).timeout()
    }

    fn set_packet_timeout(&mut self, packet_length: usize) {
        (**self).set_packet_timeout(packet_length);
    }

    fn is_packet_timeout(&mut self) -> bool {
        (**self).is_packet_timeout()
    }
}

/// A link chosen at run time, as a station opens whichever bridge its commissioning names.
impl<P: SerialPort + ?Sized> SerialPort for Box<P> {
    fn clear_port(&mut self) -> io::Result<()> {
        (**self).clear_port()
    }

    fn write_port(&mut self, packet: &[u8]) -> io::Result<usize> {
        (**self).write_port(packet)
    }

    fn read_port(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        (**self).read_port(buffer)
    }

    fn timeout(&self) -> Duration {
        (**self).timeout()
    }

    fn set_packet_timeout(&mut self, packet_length: usize) {
        (**self).set_packet_timeout(packet_length);
    }

    fn is_packet_timeout(&mut self) -> bool {
        (**self).is_packet_timeout()
    }
}

/// The longest packet either serial protocol here carries (Dynamixel's `RXPACKET_MAX_LEN`,
/// four times Feetech's), and so the most a [`Packet`] holds.
pub const PACKET_MAX: usize = 1024;

/// One packet's bytes, held inline: what a serial exchange builds and parses without the heap.
#[derive(Clone, Copy)]
pub struct Packet {
    len: usize,
    bytes: [u8; PACKET_MAX],
}

impl Packet {
    /// No bytes.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            len: 0,
            bytes: [0; PACKET_MAX],
        }
    }

    /// `len` zero bytes, at most [`PACKET_MAX`]: a packet to fill in.
    #[must_use]
    pub fn zeroed(len: usize) -> Self {
        Self {
            len: len.min(PACKET_MAX),
            bytes: [0; PACKET_MAX],
        }
    }

    /// `bytes`, as many as fit.
    #[must_use]
    pub fn from_slice(bytes: &[u8]) -> Self {
        let mut packet = Self::new();
        packet.extend_from_slice(bytes);
        packet
    }

    /// Append `bytes`, as many as fit, and return how many did.
    pub fn extend_from_slice(&mut self, bytes: &[u8]) -> usize {
        let taken = bytes.len().min(PACKET_MAX - self.len);
        self.bytes[self.len..self.len + taken].copy_from_slice(&bytes[..taken]);
        self.len += taken;
        taken
    }

    pub fn push(&mut self, byte: u8) {
        self.extend_from_slice(&[byte]);
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// The bytes, as a slice.
    #[must_use]
    pub fn as_slice(&self) -> &[u8] {
        self
    }

    /// Keep the first `len` bytes.
    pub fn truncate(&mut self, len: usize) {
        self.len = self.len.min(len);
    }

    /// Drop the first `count` bytes, keeping the rest in order.
    pub fn drop_front(&mut self, count: usize) {
        let count = count.min(self.len);
        self.bytes.copy_within(count..self.len, 0);
        self.len -= count;
    }

    /// Read from `port` until this packet holds `want` bytes or the port has none waiting,
    /// returning how many arrived.
    ///
    /// # Errors
    ///
    /// The port's I/O error.
    pub fn read_from<P: SerialPort + ?Sized>(
        &mut self,
        port: &mut P,
        want: usize,
    ) -> io::Result<usize> {
        let want = want.min(PACKET_MAX);
        if self.len >= want {
            return Ok(0);
        }
        let read = port.read_port(&mut self.bytes[self.len..want])?;
        self.len += read.min(want - self.len);
        Ok(read)
    }
}

impl Default for Packet {
    fn default() -> Self {
        Self::new()
    }
}

impl std::ops::Deref for Packet {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

impl std::ops::DerefMut for Packet {
    fn deref_mut(&mut self) -> &mut [u8] {
        &mut self.bytes[..self.len]
    }
}

impl std::fmt::Debug for Packet {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        std::fmt::Debug::fmt(&**self, formatter)
    }
}

impl PartialEq for Packet {
    fn eq(&self, other: &Self) -> bool {
        **self == **other
    }
}

impl<const N: usize> PartialEq<[u8; N]> for Packet {
    fn eq(&self, other: &[u8; N]) -> bool {
        **self == *other
    }
}

impl PartialEq<Vec<u8>> for Packet {
    fn eq(&self, other: &Vec<u8>) -> bool {
        **self == **other
    }
}

/// Copy as much of `bytes` as fits into `buffer` and return how many: a port that already
/// holds its input as a slice answers [`SerialPort::read_port`] with it.
#[must_use]
pub fn copy_into(buffer: &mut [u8], bytes: &[u8]) -> usize {
    let count = buffer.len().min(bytes.len());
    buffer[..count].copy_from_slice(&bytes[..count]);
    count
}
