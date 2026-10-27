//! The raw `CAN_RAW` socket: the crate's one unsafe module, every block a plain system call on
//! a descriptor and buffers this module owns.

#![allow(unsafe_code)]

use std::ffi::CString;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use sx_embodiment_drivers::can::{CanFrame, CanPort, RoundTrip, RoundTripWindow};

use crate::{
    BusEvent, FD_FRAME_BYTES, FRAME_BYTES, InterfaceLock, LinkError, Received, SocketCanError,
    decode, encode,
};

/// The longest a send waits for room in the interface's transmit queue. A full queue answers
/// `ENOBUFS` (or `EAGAIN`) at once; a chain's burst of one frame per motor can outrun the
/// default queue of ten, so the send waits for the bus to drain it (ten classic frames take
/// 1.35 ms at 1 Mbit/s) and then refuses with a `WouldBlock` [`LinkError`], which a chain
/// retries, reading answers meanwhile, within [`CanPort::frame_allowance`].
const SEND_PATIENCE: Duration = Duration::from_millis(2);
const SEND_RETRY: Duration = Duration::from_micros(100);

/// Frames one `recvmmsg` can take: more than a 15-motor chain's answers to one phase.
const BATCH: usize = 32;
#[allow(clippy::cast_possible_truncation)]
const BATCH_U32: libc::c_uint = BATCH as libc::c_uint;
/// Control bytes per message: one `SCM_TIMESTAMPING` (three timespecs) and one `SO_RXQ_OVFL`
/// (a `u32`), each with its header, with room to spare.
const CONTROL: usize = 256;

// `linux/can.h`, `linux/can/raw.h`, `asm-generic/socket.h`, `linux/net_tstamp.h`: stable ABI,
// written here so the crate does not depend on which libc release exports them.
const SOL_CAN_RAW: libc::c_int = 101;
const CAN_RAW_ERR_FILTER: libc::c_int = 2;
const CAN_RAW_FD_FRAMES: libc::c_int = 5;
/// Every error class except arbitration loss, transceiver status and the controller's
/// "no error" notes: `TX_TIMEOUT | CRTL | PROT | ACK | BUSOFF | BUSERROR | RESTARTED | CNT`.
const ERROR_CLASSES: u32 = 0x001 | 0x004 | 0x008 | 0x020 | 0x040 | 0x080 | 0x100 | 0x200;
/// `SO_TIMESTAMPING_OLD`: its timespecs are the platform's `struct timespec`.
const SO_TIMESTAMPING: libc::c_int = 37;
const SO_RXQ_OVFL: libc::c_int = 40;
const SOF_TIMESTAMPING_RX_HARDWARE: u32 = 1 << 2;
const SOF_TIMESTAMPING_RX_SOFTWARE: u32 = 1 << 3;
const SOF_TIMESTAMPING_SOFTWARE: u32 = 1 << 4;
const SOF_TIMESTAMPING_RAW_HARDWARE: u32 = 1 << 6;

/// One receive slot: a frame buffer and its control buffer, allocated once at open.
#[repr(C, align(8))]
struct Slot {
    frame: [u8; FD_FRAME_BYTES],
    control: [u8; CONTROL],
}

/// A frame taken from the socket, with the kernel's software receive stamp (`CLOCK_REALTIME`).
#[derive(Clone, Copy)]
struct Taken {
    frame: CanFrame,
    stamp: Option<Duration>,
}

/// What the port counted since it opened ([`CanPort::counters`]).
#[derive(Clone, Copy, Debug, Default)]
struct Counts {
    received: u64,
    read_past_long: u64,
    read_past_remote: u64,
    error_frames: u64,
    bus_off: u64,
    restarted: u64,
    no_ack: u64,
    bus_error: u64,
    tx_timeout: u64,
    controller_overflow: u64,
    error_passive: u64,
    socket_dropped: u64,
    queue_waits: u64,
    queue_full: u64,
    receive_calls: u64,
    malformed: u64,
    hardware_stamped: u64,
    host_delay_max_us: u64,
    host_delay_over_1ms: u64,
}

/// One `SocketCAN` interface, as a non-blocking raw socket bound to it.
pub struct SocketCanPort {
    socket: OwnedFd,
    interface: String,
    slots: Box<[Slot]>,
    taken: [Taken; BATCH],
    /// `taken[next..filled]` are frames received and not yet handed out.
    next: usize,
    filled: usize,
    bus_off: bool,
    /// The controller last reported error-passive (until it reports error-active).
    passive: bool,
    /// A transmit went unacknowledged and no frame has arrived since.
    unanswered: bool,
    counts: Counts,
    /// How long the motor protocol's commands waited for their answers ([`CanPort::answered`]).
    round_trips: RoundTripWindow,
    _lock: Option<InterfaceLock>,
    /// Cores kept able to wake at once while the port is open (`sx_realtime`), where the host
    /// grants it.
    _wake: Option<sx_realtime::WakeLatency>,
}

impl std::fmt::Debug for SocketCanPort {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("SocketCanPort")
            .field("interface", &self.interface)
            .field("bus_off", &self.bus_off)
            .field("counts", &self.counts)
            .finish_non_exhaustive()
    }
}

fn set_option<T>(
    socket: &OwnedFd,
    level: libc::c_int,
    name: libc::c_int,
    value: &T,
) -> io::Result<()> {
    let length = libc::socklen_t::try_from(size_of::<T>()).map_err(io::Error::other)?;
    // SAFETY: `value` is a live `T` of `length` bytes for the whole call.
    let status = unsafe {
        libc::setsockopt(
            socket.as_raw_fd(),
            level,
            name,
            (&raw const *value).cast::<libc::c_void>(),
            length,
        )
    };
    if status < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

/// A raw CAN socket bound to `interface`, non-blocking, reading FD frames and error frames, with
/// receive timestamps and drop counts asked for.
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
    // CAN FD frames on the bus are read (and read past when longer than eight bytes) instead
    // of being invisible; a kernel without FD support refuses, and classic frames still flow.
    let _ = set_option(&socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &1_i32);
    set_option(&socket, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &ERROR_CLASSES)?;
    // Timestamps and drop counts are diagnostics: a driver or kernel without them is no reason
    // to refuse the bus.
    let stamping = SOF_TIMESTAMPING_RX_SOFTWARE
        | SOF_TIMESTAMPING_SOFTWARE
        | SOF_TIMESTAMPING_RX_HARDWARE
        | SOF_TIMESTAMPING_RAW_HARDWARE;
    let _ = set_option(&socket, libc::SOL_SOCKET, SO_TIMESTAMPING, &stamping);
    let _ = set_option(&socket, libc::SOL_SOCKET, SO_RXQ_OVFL, &1_i32);
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

/// What one message's control buffer carried.
#[derive(Default)]
struct Control {
    software: Option<Duration>,
    hardware: bool,
    dropped: Option<u32>,
}

fn duration(spec: &libc::timespec) -> Option<Duration> {
    let seconds = u64::try_from(spec.tv_sec).ok()?;
    let nanos = u32::try_from(spec.tv_nsec).ok()?;
    (seconds != 0 || nanos != 0).then(|| Duration::new(seconds, nanos))
}

/// The timestamps and drop count in `header`'s control buffer.
fn control(header: &libc::msghdr) -> Control {
    let mut found = Control::default();
    // SAFETY: `header` was filled by `recvmmsg`, its control pointer and length naming a live
    // slot's control buffer; the CMSG macros only walk within `msg_controllen`.
    unsafe {
        let mut message = libc::CMSG_FIRSTHDR(header);
        while !message.is_null() {
            let level = (*message).cmsg_level;
            let kind = (*message).cmsg_type;
            let data = libc::CMSG_DATA(message);
            if level == libc::SOL_SOCKET && kind == SO_TIMESTAMPING {
                let stamps = data.cast::<[libc::timespec; 3]>().read_unaligned();
                found.software = duration(&stamps[0]);
                found.hardware = duration(&stamps[2]).is_some();
            } else if level == libc::SOL_SOCKET && kind == SO_RXQ_OVFL {
                found.dropped = Some(data.cast::<u32>().read_unaligned());
            }
            message = libc::CMSG_NXTHDR(header, message);
        }
    }
    found
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
        valid(interface)?;
        let lock = InterfaceLock::acquire(interface)?;
        Self::bound(interface, Some(lock))
    }

    /// Open a virtual interface (named `vcan*` or `vxcan*`, no device behind it in sysfs) without its
    /// lock, so a second process can play the other nodes of a test bus. Refuses any interface
    /// with a device behind it, so it can never share a real bus.
    ///
    /// # Errors
    ///
    /// As [`SocketCanPort::open`], and [`SocketCanError::Io`] for an interface that is not
    /// virtual.
    pub fn open_virtual(interface: &str) -> Result<Self, SocketCanError> {
        valid(interface)?;
        let net = std::path::Path::new("/sys/class/net").join(interface);
        let named_virtual = interface.starts_with("vcan") || interface.starts_with("vxcan");
        if !named_virtual || !net.exists() || net.join("device").exists() {
            return Err(SocketCanError::Io {
                interface: interface.to_owned(),
                source: io::Error::new(
                    io::ErrorKind::PermissionDenied,
                    "not a virtual CAN interface: a device is behind it",
                ),
            });
        }
        Self::bound(interface, None)
    }

    fn bound(interface: &str, lock: Option<InterfaceLock>) -> Result<Self, SocketCanError> {
        let socket = bind(interface).map_err(|source| SocketCanError::Io {
            interface: interface.to_owned(),
            source,
        })?;
        let slots = (0..BATCH)
            .map(|_| Slot {
                frame: [0; FD_FRAME_BYTES],
                control: [0; CONTROL],
            })
            .collect();
        let empty = Taken {
            frame: CanFrame {
                id: 0,
                data: [0; 8],
            },
            stamp: None,
        };
        Ok(Self {
            socket,
            interface: interface.to_owned(),
            slots,
            taken: [empty; BATCH],
            next: 0,
            filled: 0,
            bus_off: false,
            passive: false,
            unanswered: false,
            counts: Counts::default(),
            round_trips: RoundTripWindow::default(),
            _lock: lock,
            _wake: sx_realtime::hold_wake_latency()
                .inspect_err(|refused| {
                    eprintln!("sx-socketcan: the CPU wake-latency request was {refused}");
                })
                .ok(),
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

    /// Take everything pending on the socket, up to [`BATCH`] messages, in one `recvmmsg`;
    /// returns how many messages were read. Nothing is allocated.
    fn fill(&mut self) -> io::Result<usize> {
        self.counts.receive_calls += 1;
        // SAFETY: all-zero `iovec`s and `mmsghdr`s are valid (null pointers, zero lengths)
        // and every one passed to the kernel is filled in below.
        let mut vectors: [libc::iovec; BATCH] = unsafe { std::mem::zeroed() };
        // SAFETY: as above.
        let mut headers: [libc::mmsghdr; BATCH] = unsafe { std::mem::zeroed() };
        for ((slot, vector), header) in self
            .slots
            .iter_mut()
            .zip(vectors.iter_mut())
            .zip(headers.iter_mut())
        {
            vector.iov_base = slot.frame.as_mut_ptr().cast();
            vector.iov_len = FD_FRAME_BYTES;
            header.msg_hdr.msg_iov = &raw mut *vector;
            header.msg_hdr.msg_iovlen = 1;
            header.msg_hdr.msg_control = slot.control.as_mut_ptr().cast();
            header.msg_hdr.msg_controllen = CONTROL as _;
        }
        let count = loop {
            // SAFETY: `headers` holds `BATCH` messages, each naming one live slot's frame and
            // control buffers by pointers that stay valid for the whole call.
            let read = unsafe {
                libc::recvmmsg(
                    self.socket.as_raw_fd(),
                    headers.as_mut_ptr(),
                    BATCH_U32,
                    libc::MSG_DONTWAIT,
                    std::ptr::null_mut(),
                )
            };
            if let Ok(count) = usize::try_from(read) {
                break count;
            }
            let error = io::Error::last_os_error();
            match error.kind() {
                io::ErrorKind::Interrupted => {}
                io::ErrorKind::WouldBlock => return Ok(0),
                _ => return Err(error),
            }
        };
        let now = SystemTime::now().duration_since(UNIX_EPOCH).ok();
        self.next = 0;
        self.filled = 0;
        for (index, header) in headers.iter().enumerate().take(count) {
            let found = control(&header.msg_hdr);
            if let Some(dropped) = found.dropped {
                self.counts.socket_dropped = u64::from(dropped);
            }
            let length = (header.msg_len as usize).min(FD_FRAME_BYTES);
            match decode(&self.slots[index].frame[..length]) {
                // The kernel returns only `can_frame` and `canfd_frame` sizes while CAN XL is
                // off; anything else is counted and read past, never fails the batch around it.
                None => self.counts.malformed += 1,
                Some(Received::Data(frame)) => {
                    self.counts.received += 1;
                    self.unanswered = false;
                    self.counts.hardware_stamped += u64::from(found.hardware);
                    if let (Some(stamp), Some(now)) = (found.software, now) {
                        let delay = u64::try_from(now.saturating_sub(stamp).as_micros())
                            .unwrap_or(u64::MAX);
                        self.counts.host_delay_max_us = self.counts.host_delay_max_us.max(delay);
                        self.counts.host_delay_over_1ms += u64::from(delay > 1000);
                    }
                    self.taken[self.filled] = Taken {
                        frame,
                        stamp: found.software,
                    };
                    self.filled += 1;
                }
                Some(Received::Long) => self.counts.read_past_long += 1,
                Some(Received::Remote) => self.counts.read_past_remote += 1,
                Some(Received::Error(event)) => self.note(event),
            }
        }
        Ok(count)
    }

    fn note(&mut self, event: BusEvent) {
        let counts = &mut self.counts;
        counts.error_frames += 1;
        counts.bus_off += u64::from(event.bus_off);
        counts.restarted += u64::from(event.restarted);
        counts.no_ack += u64::from(event.no_ack);
        counts.bus_error += u64::from(event.bus_error);
        counts.tx_timeout += u64::from(event.tx_timeout);
        counts.controller_overflow += u64::from(event.overflow);
        counts.error_passive += u64::from(event.passive);
        if event.bus_off {
            self.bus_off = true;
        }
        if event.passive {
            self.passive = true;
        }
        // Error counters back under 128 mean error-active again, whether or not the driver
        // sends an "active" note.
        if let Some((tx, rx)) = event.counters
            && tx < 128
            && rx < 128
        {
            self.passive = false;
        }
        if event.no_ack {
            self.unanswered = true;
        }
        if event.restarted || event.active {
            self.bus_off = false;
            self.passive = false;
        }
    }

    /// Wait in `ppoll` until the socket is readable or `deadline` passes.
    fn wait(&self, deadline: Instant) -> io::Result<()> {
        let left = deadline.saturating_duration_since(Instant::now());
        let timeout = libc::timespec {
            tv_sec: libc::time_t::try_from(left.as_secs()).unwrap_or(libc::time_t::MAX),
            tv_nsec: libc::c_long::from(i32::try_from(left.subsec_nanos()).unwrap_or(0)),
        };
        let mut poll = libc::pollfd {
            fd: self.socket.as_raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        // SAFETY: one live `pollfd` and a live `timespec` for the whole call; no signal mask.
        let status = unsafe { libc::ppoll(&raw mut poll, 1, &raw const timeout, std::ptr::null()) };
        if status < 0 {
            let error = io::Error::last_os_error();
            if error.kind() != io::ErrorKind::Interrupted {
                return Err(error);
            }
        }
        Ok(())
    }

    /// The kernel's software receive stamp (`CLOCK_REALTIME`) of the frame `receive` last
    /// handed out, where the driver gives one: the host delay is the time from it to the read.
    #[must_use]
    pub fn last_stamp(&self) -> Option<Duration> {
        self.next
            .checked_sub(1)
            .and_then(|last| self.taken[last].stamp)
    }
}

fn valid(interface: &str) -> Result<(), SocketCanError> {
    if interface.is_empty() || interface.len() >= libc::IFNAMSIZ || interface.contains(['/', '\0'])
    {
        return Err(SocketCanError::Name(interface.to_owned()));
    }
    Ok(())
}

fn queue_full(error: &io::Error) -> bool {
    error.kind() == io::ErrorKind::WouldBlock || error.raw_os_error() == Some(libc::ENOBUFS)
}

impl CanPort for SocketCanPort {
    /// One classic frame onto the interface's transmit queue. When the queue stays full for
    /// [`SEND_PATIENCE`] it is refused as `WouldBlock`: [`LinkError::BusOff`] if the controller
    /// last reported bus-off (it may restart: `restart-ms`), else [`LinkError::QueueFull`].
    /// Nothing is latched: every send is tried, so a stop's disable frames go out the moment
    /// the controller is back. An interface that is down is [`LinkError::Down`]
    /// (`NotConnected`).
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        let bytes = encode(frame)?;
        let started = Instant::now();
        let mut waited = false;
        loop {
            match self.write(&bytes) {
                Ok(FRAME_BYTES) => return Ok(()),
                Ok(written) => {
                    return Err(io::Error::other(format!(
                        "{} took {written} of a frame's {FRAME_BYTES} bytes",
                        self.interface
                    )));
                }
                Err(error) if queue_full(&error) => {
                    if !waited {
                        waited = true;
                        self.counts.queue_waits += 1;
                    }
                    if started.elapsed() >= SEND_PATIENCE {
                        self.counts.queue_full += 1;
                        let interface = self.interface.clone();
                        return Err(if self.bus_off {
                            LinkError::BusOff { interface }
                        } else {
                            LinkError::QueueFull {
                                interface,
                                waited_us: u64::try_from(started.elapsed().as_micros())
                                    .unwrap_or(u64::MAX),
                            }
                        }
                        .into());
                    }
                    std::thread::sleep(SEND_RETRY);
                }
                Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
                Err(error) if error.raw_os_error() == Some(libc::ENETDOWN) => {
                    return Err(LinkError::Down {
                        interface: self.interface.clone(),
                    }
                    .into());
                }
                Err(error) => return Err(error),
            }
        }
    }

    /// The next data frame (classic, or FD of at most eight bytes), or `None` when nothing is
    /// pending. Error frames are counted and decide bus-off; longer FD and remote frames are
    /// read past.
    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        loop {
            if self.next < self.filled {
                self.next += 1;
                return Ok(Some(self.taken[self.next - 1].frame));
            }
            if self.fill()? == 0 {
                return Ok(None);
            }
        }
    }

    /// As `receive`, sleeping in `ppoll` until a frame arrives or `deadline` passes.
    fn receive_until(&mut self, deadline: Instant) -> io::Result<Option<CanFrame>> {
        loop {
            if let Some(frame) = self.receive()? {
                return Ok(Some(frame));
            }
            if Instant::now() >= deadline {
                return Ok(None);
            }
            self.wait(deadline)?;
        }
    }

    /// A chain retries a refused send until this has passed since its first try, and a retry
    /// begun just before then may itself wait [`SEND_PATIENCE`]: two patiences, then the frame
    /// and its answer, well under a millisecond on a 1 Mbit/s bus.
    fn frame_allowance(&self) -> Duration {
        SEND_PATIENCE * 2 + Duration::from_millis(1)
    }

    /// Nothing answering after an unacknowledged transmit with nothing received since; bus
    /// errors while the controller is bus-off or error-passive; answering once a frame arrived.
    fn health(&self) -> sx_embodiment_drivers::can::LinkHealth {
        use sx_embodiment_drivers::can::LinkHealth;
        let bus = self.interface.clone();
        if self.unanswered {
            LinkHealth::NothingAnswering { bus }
        } else if self.bus_off || self.passive {
            let detail = if self.bus_off {
                "bus-off"
            } else {
                "error-passive"
            };
            LinkHealth::BusErrors {
                bus,
                detail: detail.to_owned(),
            }
        } else if self.counts.received > 0 {
            LinkHealth::Answering
        } else {
            LinkHealth::Quiet
        }
    }

    fn counters(&self) -> Vec<(&'static str, u64)> {
        let counts = self.counts;
        vec![
            ("rx_frames", counts.received),
            ("rx_read_past_long_fd", counts.read_past_long),
            ("rx_read_past_remote", counts.read_past_remote),
            ("rx_socket_dropped", counts.socket_dropped),
            ("rx_calls", counts.receive_calls),
            ("rx_malformed", counts.malformed),
            ("rx_hardware_stamped", counts.hardware_stamped),
            ("host_delay_max_us", counts.host_delay_max_us),
            ("host_delay_over_1ms", counts.host_delay_over_1ms),
            ("error_frames", counts.error_frames),
            ("bus_off", counts.bus_off),
            ("restarted", counts.restarted),
            ("no_ack", counts.no_ack),
            ("bus_errors", counts.bus_error),
            ("tx_timeouts", counts.tx_timeout),
            ("controller_overflows", counts.controller_overflow),
            ("error_passive", counts.error_passive),
            ("tx_queue_waits", counts.queue_waits),
            ("tx_queue_full", counts.queue_full),
        ]
    }

    /// Recorded on the control thread into a fixed ring; nothing allocates.
    fn answered(&mut self, round_trip: Duration) {
        self.round_trips.record(round_trip);
    }

    fn round_trip(&mut self) -> Option<RoundTrip> {
        self.round_trips.take()
    }
}
