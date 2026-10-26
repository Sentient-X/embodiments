//! A serial device node as the half-duplex bus port the Feetech SDK reads through.

use std::io::{self, Read as _, Write as _};
use std::path::Path;
use std::time::{Duration, Instant};

use sx_embodiment_bodies::usb_serial::{TtyError, open_exclusive};

use sx_embodiment_drivers::serial::SerialPort;

/// The longest a status packet is waited for. A USB serial bridge answers within a
/// millisecond or two at 1 Mbaud; a silent id costs this much.
const PACKET_TIMEOUT: Duration = Duration::from_millis(20);
/// How long a read with nothing buffered rests before reporting so.
const IDLE: Duration = Duration::from_micros(200);

/// One opened tty.
pub struct TtyPort {
    port: Box<dyn serialport::SerialPort>,
    deadline: Instant,
}

impl TtyPort {
    /// Open `path` raw at `baud`, eight data bits, no parity, one stop bit, exclusively, through
    /// the layer's one serial opener.
    ///
    /// # Errors
    ///
    /// Returns why it cannot be opened: of kind [`io::ErrorKind::ResourceBusy`], naming the
    /// holder where the host tells it, when another process has it open.
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        let port =
            open_exclusive(Path::new(path), baud, Duration::from_millis(1)).map_err(|error| {
                match error {
                    TtyError::Busy { .. } => io::Error::new(io::ErrorKind::ResourceBusy, error),
                    TtyError::Open { .. } => io::Error::other(error),
                }
            })?;
        Ok(Self {
            port,
            deadline: Instant::now(),
        })
    }
}

impl SerialPort for TtyPort {
    fn clear_port(&mut self) -> io::Result<()> {
        self.port
            .clear(serialport::ClearBuffer::Input)
            .map_err(io::Error::other)
    }

    fn write_port(&mut self, packet: &[u8]) -> io::Result<usize> {
        self.port.write_all(packet)?;
        self.port.flush()?;
        Ok(packet.len())
    }

    fn read_port(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        let buffered = usize::try_from(self.port.bytes_to_read().map_err(io::Error::other)?)
            .unwrap_or(usize::MAX);
        let take = buffered.min(buffer.len());
        if take == 0 {
            std::thread::sleep(IDLE);
            return Ok(0);
        }
        self.port.read(&mut buffer[..take])
    }

    fn timeout(&self) -> Duration {
        PACKET_TIMEOUT
    }

    fn set_packet_timeout(&mut self, _packet_length: usize) {
        self.deadline = Instant::now() + PACKET_TIMEOUT;
    }

    fn is_packet_timeout(&mut self) -> bool {
        Instant::now() >= self.deadline
    }
}
