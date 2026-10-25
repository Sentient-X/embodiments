//! The raw `CAN_RAW` socket: the crate's one unsafe module, every block a plain system call on
//! a descriptor this module owns.

#![allow(unsafe_code)]

use std::ffi::CString;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::{Duration, Instant};

use sx_embodiment_drivers::can::{CanFrame, CanPort};

use crate::{FRAME_BYTES, InterfaceLock, SocketCanError, decode, encode};

/// The longest a send waits for room in the interface's transmit queue. A full queue answers
/// `ENOBUFS` (or `EAGAIN`) at once; a chain's burst of one frame per motor can outrun the
/// default queue of ten, so the send waits for the bus to drain it rather than dropping the
/// frame, and reports the error only when the bus has not moved in this long.
const SEND_PATIENCE: Duration = Duration::from_millis(10);
const SEND_RETRY: Duration = Duration::from_micros(100);

/// One `SocketCAN` interface, as a non-blocking raw socket bound to it.
#[derive(Debug)]
pub struct SocketCanPort {
    socket: OwnedFd,
    interface: String,
    _lock: InterfaceLock,
}

/// A raw CAN socket bound to `interface`, non-blocking.
fn bind(interface: &str) -> io::Result<OwnedFd> {
    let name = CString::new(interface).map_err(io::Error::other)?;
    // SAFETY: `socket` takes no pointers; a negative return is checked before use.
    let raw = unsafe {
        libc::socket(
            libc::PF_CAN,
            libc::SOCK_RAW | libc::SOCK_NONBLOCK | libc::SOCK_CLOEXEC,
            libc::CAN_RAW,
        )
    };
    if raw < 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: `raw` is a descriptor `socket` just returned and nothing else holds.
    let socket = unsafe { OwnedFd::from_raw_fd(raw) };
    // SAFETY: `name` is a NUL-terminated string that outlives the call.
    let index = unsafe { libc::if_nametoindex(name.as_ptr()) };
    if index == 0 {
        return Err(io::Error::last_os_error());
    }
    // SAFETY: `sockaddr_can` is plain data for which all zero bytes are a valid value.
    let mut address: libc::sockaddr_can = unsafe { std::mem::zeroed() };
    address.can_family = libc::sa_family_t::try_from(libc::AF_CAN).map_err(io::Error::other)?;
    address.can_ifindex = libc::c_int::try_from(index).map_err(io::Error::other)?;
    let length =
        libc::socklen_t::try_from(size_of::<libc::sockaddr_can>()).map_err(io::Error::other)?;
    // SAFETY: `address` is a valid `sockaddr_can` of `length` bytes for the whole call.
    let bound = unsafe {
        libc::bind(
            socket.as_raw_fd(),
            (&raw const address).cast::<libc::sockaddr>(),
            length,
        )
    };
    if bound < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(socket)
}

impl SocketCanPort {
    /// Take `interface`'s lock ([`InterfaceLock`]), then open a raw CAN socket on it (`can1`),
    /// non-blocking.
    ///
    /// # Errors
    ///
    /// Refuses a name the kernel cannot hold, an interface another of our processes holds open,
    /// and returns the system's error when no such interface exists or the socket cannot be
    /// opened or bound.
    pub fn open(interface: &str) -> Result<Self, SocketCanError> {
        if interface.is_empty()
            || interface.len() >= libc::IFNAMSIZ
            || interface.contains(['/', '\0'])
        {
            return Err(SocketCanError::Name(interface.to_owned()));
        }
        let lock = InterfaceLock::acquire(interface)?;
        let socket = bind(interface).map_err(|source| SocketCanError::Io {
            interface: interface.to_owned(),
            source,
        })?;
        Ok(Self {
            socket,
            interface: interface.to_owned(),
            _lock: lock,
        })
    }

    /// The interface this port is bound to.
    #[must_use]
    pub fn interface(&self) -> &str {
        &self.interface
    }

    fn write(&self, bytes: &[u8; FRAME_BYTES]) -> io::Result<usize> {
        // SAFETY: `bytes` is a live buffer of `FRAME_BYTES` bytes for the whole call.
        let written = unsafe {
            libc::write(
                self.socket.as_raw_fd(),
                bytes.as_ptr().cast::<libc::c_void>(),
                FRAME_BYTES,
            )
        };
        usize::try_from(written).map_err(|_| io::Error::last_os_error())
    }

    fn read(&self, bytes: &mut [u8; FRAME_BYTES]) -> io::Result<usize> {
        // SAFETY: `bytes` is a live, writable buffer of `FRAME_BYTES` bytes for the whole call.
        let read = unsafe {
            libc::read(
                self.socket.as_raw_fd(),
                bytes.as_mut_ptr().cast::<libc::c_void>(),
                FRAME_BYTES,
            )
        };
        usize::try_from(read).map_err(|_| io::Error::last_os_error())
    }
}

fn queue_full(error: &io::Error) -> bool {
    error.kind() == io::ErrorKind::WouldBlock || error.raw_os_error() == Some(libc::ENOBUFS)
}

impl CanPort for SocketCanPort {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        let bytes = encode(frame)?;
        let deadline = Instant::now() + SEND_PATIENCE;
        loop {
            match self.write(&bytes) {
                Ok(FRAME_BYTES) => return Ok(()),
                Ok(written) => {
                    return Err(io::Error::other(format!(
                        "{} took {written} of a frame's {FRAME_BYTES} bytes",
                        self.interface
                    )));
                }
                Err(error) if queue_full(&error) && Instant::now() < deadline => {
                    std::thread::sleep(SEND_RETRY);
                }
                Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
                Err(error) => return Err(error),
            }
        }
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        let mut bytes = [0; FRAME_BYTES];
        loop {
            match self.read(&mut bytes) {
                Ok(FRAME_BYTES) => {
                    // A remote request or error frame is not an answer: read past it.
                    if let Some(frame) = decode(&bytes) {
                        return Ok(Some(frame));
                    }
                }
                Ok(read) => {
                    return Err(io::Error::other(format!(
                        "{} gave {read} bytes, not a {FRAME_BYTES}-byte frame",
                        self.interface
                    )));
                }
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => return Ok(None),
                Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
                Err(error) => return Err(error),
            }
        }
    }

    /// A send may wait [`SEND_PATIENCE`] for queue room, then the frame and its answer take
    /// well under a millisecond on a 1 Mbit/s bus.
    fn frame_allowance(&self) -> Duration {
        SEND_PATIENCE + Duration::from_millis(1)
    }
}
