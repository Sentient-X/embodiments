//! usbfs, spoken directly: the `USBDEVFS_*` ioctls of `linux/usbdevice_fs.h` on a device fd
//! the caller owns (on Android, `UsbDeviceConnection.getFileDescriptor()`).
//!
//! Why not libusb: libusb on Android needs `libusb_wrap_sys_device` plus a cross-compiled
//! C library, its event loop and its own thread model, to do what eleven ioctls do. The rig
//! needs exactly: read descriptors, claim/release (detaching a kernel driver), set an
//! alternate setting, control transfers, and asynchronous URBs — isochronous for the
//! cameras, bulk for the encoders — reaped from a thread that `poll`s the fd. Owning that
//! directly keeps one language, one build, no C dependency, and lets every request be unit
//! tested above this file. The structs here are the kernel ABI on LP64 (aarch64 and x86_64),
//! whose sizes are asserted in tests against the ioctl numbers the kernel headers produce.

use std::alloc::{self, Layout};
use std::ffi::c_void;
use std::io;
use std::mem::size_of;
use std::os::fd::RawFd;
use std::ptr::NonNull;

const IOC_NONE: u32 = 0;
const IOC_WRITE: u32 = 1;
const IOC_READ: u32 = 2;

/// `_IOC(dir, 'U', nr, size)` as asm-generic defines it (arm64 and x86 share it).
const fn ioc(dir: u32, nr: u32, size: usize) -> u64 {
    ((dir as u64) << 30) | ((size as u64) << 16) | ((b'U' as u64) << 8) | nr as u64
}

#[repr(C)]
struct CtrlTransfer {
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
    timeout: u32,
    data: *mut c_void,
}

#[repr(C)]
struct BulkTransfer {
    endpoint: u32,
    length: u32,
    timeout: u32,
    data: *mut c_void,
}

#[repr(C)]
struct SetInterface {
    interface: u32,
    alternate: u32,
}

#[repr(C)]
struct IoctlArgument {
    interface: i32,
    code: i32,
    data: *mut c_void,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct IsoPacket {
    pub length: u32,
    pub actual_length: u32,
    pub status: u32,
}

#[repr(C)]
struct Urb {
    kind: u8,
    endpoint: u8,
    status: i32,
    flags: u32,
    buffer: *mut c_void,
    buffer_length: i32,
    actual_length: i32,
    start_frame: i32,
    packets_or_stream: i32,
    error_count: i32,
    signal: u32,
    context: *mut c_void,
    // followed by `IsoPacket[packets]` for isochronous URBs
}

pub const CONTROL: u64 = ioc(IOC_READ | IOC_WRITE, 0, size_of::<CtrlTransfer>());
pub const BULK: u64 = ioc(IOC_READ | IOC_WRITE, 2, size_of::<BulkTransfer>());
pub const SET_INTERFACE: u64 = ioc(IOC_READ, 4, size_of::<SetInterface>());
pub const SUBMIT_URB: u64 = ioc(IOC_READ, 10, size_of::<Urb>());
pub const DISCARD_URB: u64 = ioc(IOC_NONE, 11, 0);
pub const REAP_URB_NDELAY: u64 = ioc(IOC_WRITE, 13, size_of::<*mut c_void>());
pub const CLAIM_INTERFACE: u64 = ioc(IOC_READ, 15, size_of::<u32>());
pub const RELEASE_INTERFACE: u64 = ioc(IOC_READ, 16, size_of::<u32>());
pub const IOCTL: u64 = ioc(IOC_READ | IOC_WRITE, 18, size_of::<IoctlArgument>());
pub const CLEAR_HALT: u64 = ioc(IOC_READ, 21, size_of::<u32>());
pub const DISCONNECT: u64 = ioc(IOC_NONE, 22, 0);
pub const CONNECT: u64 = ioc(IOC_NONE, 23, 0);
pub const GET_SPEED: u64 = ioc(IOC_NONE, 31, 0);
/// `USBDEVFS_CONNINFO_EX(sizeof(struct usbdevfs_conninfo_ex))` (Linux 5.5+).
pub const CONNINFO_EX: u64 = ioc(IOC_READ, 32, size_of::<ConnInfoEx>());

/// `struct usbdevfs_conninfo_ex`: where the device sits on the bus.
#[repr(C)]
#[derive(Default)]
struct ConnInfoEx {
    size: u32,
    busnum: u32,
    devnum: u32,
    speed: u32,
    num_ports: u8,
    ports: [u8; 7],
}

/// The physical port path of a device, as sysfs names it: `<bus>-<port>.<port>…`. It
/// stays the same when a device is unplugged and plugged back into the same hub port,
/// unlike `/dev/bus/usb/BBB/DDD`, which renumbers.
fn port_path(info: &ConnInfoEx) -> io::Result<String> {
    let count = usize::from(info.num_ports);
    if count == 0 || count > info.ports.len() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("the kernel reported {count} port numbers"),
        ));
    }
    let ports: Vec<String> = info.ports[..count].iter().map(u8::to_string).collect();
    Ok(format!("{}-{}", info.busnum, ports.join(".")))
}

const URB_TYPE_ISO: u8 = 0;
const URB_ISO_ASAP: u32 = 0x02;

/// One ioctl on `fd`; `-1` becomes the errno.
///
/// # Safety
/// `argument` must be what `request` expects, valid for the call.
unsafe fn ioctl(fd: RawFd, request: u64, argument: *mut c_void) -> io::Result<i32> {
    // SAFETY: forwarded from the caller's contract; `request as _` is the platform's
    // ioctl request type (c_int on bionic, c_ulong on glibc), with the same bits.
    let result = unsafe { libc::ioctl(fd, request as _, argument) };
    if result < 0 {
        Err(io::Error::last_os_error())
    } else {
        Ok(result)
    }
}

/// A usbfs device on a borrowed fd: ownership, and the close, stay with whoever opened it
/// (Kotlin's `UsbDeviceConnection`). The rig stops using the fd before it gives it back.
#[derive(Debug, Clone, Copy)]
pub struct Device {
    fd: RawFd,
}

impl Device {
    pub fn borrow(fd: RawFd) -> Self {
        Self { fd }
    }

    pub fn fd(&self) -> RawFd {
        self.fd
    }

    /// The device descriptor and every configuration descriptor, as the kernel cached them
    /// at enumeration: reading a usbfs node from offset 0 returns them without any bus
    /// traffic (this is what `UsbDeviceConnection.getRawDescriptors` does).
    pub fn descriptors(&self) -> io::Result<Vec<u8>> {
        let mut out = Vec::new();
        let mut buffer = [0_u8; 4096];
        loop {
            // SAFETY: the buffer is valid for its length; pread does not move the offset
            // another reader of this fd may depend on.
            let read = unsafe {
                libc::pread(
                    self.fd,
                    buffer.as_mut_ptr().cast(),
                    buffer.len(),
                    out.len() as libc::off_t,
                )
            };
            match read {
                0 => return Ok(out),
                n if n > 0 => out.extend_from_slice(&buffer[..n as usize]),
                _ => return Err(io::Error::last_os_error()),
            }
        }
    }

    /// Detach any kernel driver bound to `interface`, then claim it for this fd. A missing
    /// driver (`ENODATA`) is the normal Android case.
    pub fn claim(&self, interface: u8) -> io::Result<()> {
        let mut argument = IoctlArgument {
            interface: i32::from(interface),
            code: DISCONNECT as i32,
            data: std::ptr::null_mut(),
        };
        // SAFETY: a usbdevfs_ioctl naming DISCONNECT, which takes no data.
        match unsafe { ioctl(self.fd, IOCTL, (&raw mut argument).cast()) } {
            Ok(_) => {}
            Err(error) if error.raw_os_error() == Some(libc::ENODATA) => {}
            Err(error) => return Err(error),
        }
        let mut number = u32::from(interface);
        // SAFETY: CLAIMINTERFACE reads one unsigned int.
        unsafe { ioctl(self.fd, CLAIM_INTERFACE, (&raw mut number).cast()) }.map(drop)
    }

    pub fn release(&self, interface: u8) -> io::Result<()> {
        let mut number = u32::from(interface);
        // SAFETY: RELEASEINTERFACE reads one unsigned int.
        unsafe { ioctl(self.fd, RELEASE_INTERFACE, (&raw mut number).cast()) }.map(drop)
    }

    pub fn set_interface(&self, interface: u8, alternate: u8) -> io::Result<()> {
        let mut argument = SetInterface {
            interface: u32::from(interface),
            alternate: u32::from(alternate),
        };
        // SAFETY: SETINTERFACE reads a usbdevfs_setinterface.
        unsafe { ioctl(self.fd, SET_INTERFACE, (&raw mut argument).cast()) }.map(drop)
    }

    /// The bus speed the device enumerated at (`enum usb_device_speed`: 2 full, 3 high,
    /// 5 super).
    /// The device's physical port path (`USBDEVFS_CONNINFO_EX`); a kernel without the
    /// ioctl answers `ENOTTY`, reported as such.
    pub fn port_path(&self) -> io::Result<String> {
        let mut info = ConnInfoEx::default();
        // SAFETY: CONNINFO_EX writes at most the struct size encoded in its number.
        unsafe { ioctl(self.fd, CONNINFO_EX, (&raw mut info).cast()) }?;
        port_path(&info)
    }

    pub fn speed(&self) -> io::Result<i32> {
        // SAFETY: GET_SPEED takes no argument and returns the speed.
        unsafe { ioctl(self.fd, GET_SPEED, std::ptr::null_mut()) }
    }

    pub fn clear_halt(&self, endpoint: u8) -> io::Result<()> {
        let mut number = u32::from(endpoint);
        // SAFETY: CLEAR_HALT reads one unsigned int.
        unsafe { ioctl(self.fd, CLEAR_HALT, (&raw mut number).cast()) }.map(drop)
    }

    fn control(
        &self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        data: *mut u8,
        length: usize,
    ) -> io::Result<usize> {
        let length = u16::try_from(length)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "control data too long"))?;
        let mut transfer = CtrlTransfer {
            request_type,
            request,
            value,
            index,
            length,
            timeout: 1000,
            data: data.cast(),
        };
        // SAFETY: the transfer points at `length` bytes the caller keeps alive and, for
        // device-to-host requests, writable, for this synchronous call.
        unsafe { ioctl(self.fd, CONTROL, (&raw mut transfer).cast()) }.map(|n| n as usize)
    }

    /// One synchronous bulk transfer; a timeout is `ErrorKind::TimedOut`.
    pub fn bulk(&self, endpoint: u8, data: &mut [u8], timeout_ms: u32) -> io::Result<usize> {
        let mut transfer = BulkTransfer {
            endpoint: u32::from(endpoint),
            length: u32::try_from(data.len())
                .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "bulk too long"))?,
            timeout: timeout_ms,
            data: data.as_mut_ptr().cast(),
        };
        // SAFETY: the transfer points at `data`, alive and writable for this call.
        match unsafe { ioctl(self.fd, BULK, (&raw mut transfer).cast()) } {
            Ok(n) => Ok(n as usize),
            Err(error) if error.raw_os_error() == Some(libc::ETIMEDOUT) => {
                Err(io::Error::new(io::ErrorKind::TimedOut, error))
            }
            Err(error) => Err(error),
        }
    }

    /// Wait up to `timeout_ms` for a completed URB (`POLLOUT`) or a disconnect.
    pub fn wait(&self, timeout_ms: i32) -> io::Result<bool> {
        let mut poll = libc::pollfd {
            fd: self.fd,
            events: libc::POLLOUT | libc::POLLWRNORM,
            revents: 0,
        };
        // SAFETY: one valid pollfd.
        let ready = unsafe { libc::poll(&raw mut poll, 1, timeout_ms) };
        if ready < 0 {
            let error = io::Error::last_os_error();
            return if error.kind() == io::ErrorKind::Interrupted {
                Ok(false)
            } else {
                Err(error)
            };
        }
        if poll.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 {
            return Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "USB device disconnected",
            ));
        }
        Ok(ready > 0)
    }
}

impl super::Control for Device {
    fn control_in(
        &self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        data: &mut [u8],
    ) -> io::Result<usize> {
        self.control(
            request_type | 0x80,
            request,
            value,
            index,
            data.as_mut_ptr(),
            data.len(),
        )
    }

    fn control_out(
        &self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> io::Result<()> {
        let mut copy = data.to_vec();
        let sent = self.control(
            request_type & 0x7f,
            request,
            value,
            index,
            copy.as_mut_ptr(),
            copy.len(),
        )?;
        if sent != data.len() {
            return Err(io::Error::new(
                io::ErrorKind::WriteZero,
                format!("control OUT sent {sent} of {} bytes", data.len()),
            ));
        }
        Ok(())
    }
}

/// One asynchronous transfer the kernel may own between submit and reap: the URB header,
/// its iso packet descriptors behind it, and the data buffer, all at fixed addresses.
pub struct Transfer {
    urb: NonNull<Urb>,
    layout: Layout,
    buffer: Vec<u8>,
    packets: usize,
    packet_size: usize,
}

// SAFETY: a Transfer is plain memory; the kernel touches it only between submit and reap,
// both made by the one thread that owns the pool.
unsafe impl Send for Transfer {}

impl Transfer {
    fn allocate(
        kind: u8,
        endpoint: u8,
        packets: usize,
        packet_size: usize,
        context: usize,
    ) -> Self {
        let size = size_of::<Urb>() + packets * size_of::<IsoPacket>();
        let layout = Layout::from_size_align(size, std::mem::align_of::<Urb>()).expect("URB");
        // SAFETY: a non-zero layout; zeroed memory is a valid Urb and IsoPacket array.
        let raw = unsafe { alloc::alloc_zeroed(layout) }.cast::<Urb>();
        let urb = NonNull::new(raw).unwrap_or_else(|| alloc::handle_alloc_error(layout));
        let mut buffer = vec![0_u8; packets.max(1) * packet_size];
        // SAFETY: freshly allocated, exclusively ours.
        unsafe {
            let urb = urb.as_ptr();
            (*urb).kind = kind;
            (*urb).endpoint = endpoint;
            (*urb).buffer = buffer.as_mut_ptr().cast();
            (*urb).buffer_length = buffer.len() as i32;
            (*urb).context = context as *mut c_void;
            if kind == URB_TYPE_ISO {
                (*urb).flags = URB_ISO_ASAP;
                (*urb).packets_or_stream = packets as i32;
                let descriptors = urb.add(1).cast::<IsoPacket>();
                for index in 0..packets {
                    descriptors.add(index).write(IsoPacket {
                        length: packet_size as u32,
                        actual_length: 0,
                        status: 0,
                    });
                }
            }
        }
        Self {
            urb,
            layout,
            buffer,
            packets,
            packet_size,
        }
    }

    pub fn isochronous(endpoint: u8, packets: usize, packet_size: usize, context: usize) -> Self {
        Self::allocate(URB_TYPE_ISO, endpoint, packets, packet_size, context)
    }

    /// The URB's completion status (0, or a negative errno).
    pub fn status(&self) -> i32 {
        // SAFETY: read after reap, while the kernel no longer owns the URB.
        unsafe { (*self.urb.as_ptr()).status }
    }

    pub fn packets(&self) -> usize {
        self.packets
    }

    /// Packet `index` of a completed isochronous transfer: its status and its bytes. Packets
    /// sit back to back at their requested lengths in the buffer.
    pub fn packet(&self, index: usize) -> (i32, &[u8]) {
        assert!(index < self.packets);
        // SAFETY: the descriptor array follows the header, `packets` long; read after reap.
        let descriptor = unsafe { *self.urb.as_ptr().add(1).cast::<IsoPacket>().add(index) };
        let start = index * self.packet_size;
        let length = (descriptor.actual_length as usize).min(self.packet_size);
        (
            descriptor.status as i32,
            &self.buffer[start..start + length],
        )
    }

    fn reset(&mut self) {
        // SAFETY: called only while the kernel does not own the URB.
        unsafe {
            let urb = self.urb.as_ptr();
            (*urb).status = 0;
            (*urb).actual_length = 0;
            (*urb).error_count = 0;
            (*urb).start_frame = 0;
            let descriptors = urb.add(1).cast::<IsoPacket>();
            for index in 0..self.packets {
                (*descriptors.add(index)).actual_length = 0;
                (*descriptors.add(index)).status = 0;
            }
        }
    }
}

impl Drop for Transfer {
    fn drop(&mut self) {
        // SAFETY: the pool never drops a Transfer the kernel still owns (it discards and
        // reaps them all first); allocated with this layout.
        unsafe { alloc::dealloc(self.urb.as_ptr().cast(), self.layout) };
    }
}

/// A fixed set of in-flight transfers on one endpoint, resubmitted as they complete.
pub struct Pool {
    device: Device,
    transfers: Vec<Transfer>,
    in_flight: Vec<bool>,
}

impl Pool {
    pub fn new(device: Device, transfers: Vec<Transfer>) -> Self {
        let count = transfers.len();
        Self {
            device,
            transfers,
            in_flight: vec![false; count],
        }
    }

    pub fn submit(&mut self, index: usize) -> io::Result<()> {
        let transfer = &mut self.transfers[index];
        transfer.reset();
        // SAFETY: the URB, its packets and buffer stay allocated and untouched by us until
        // REAPURB hands this pointer back.
        unsafe { ioctl(self.device.fd, SUBMIT_URB, transfer.urb.as_ptr().cast()) }?;
        self.in_flight[index] = true;
        Ok(())
    }

    pub fn submit_all(&mut self) -> io::Result<()> {
        for index in 0..self.transfers.len() {
            self.submit(index)?;
        }
        Ok(())
    }

    /// Every transfer that has completed by now, by index; none blocks.
    pub fn reap(&mut self) -> io::Result<Vec<usize>> {
        let mut done = Vec::new();
        loop {
            let mut reaped: *mut c_void = std::ptr::null_mut();
            // SAFETY: REAPURBNDELAY writes one pointer.
            match unsafe { ioctl(self.device.fd, REAP_URB_NDELAY, (&raw mut reaped).cast()) } {
                Ok(_) => {
                    let urb = reaped.cast::<Urb>();
                    // SAFETY: the kernel returns a URB this pool submitted.
                    let index = unsafe { (*urb).context } as usize;
                    if index >= self.transfers.len() || self.transfers[index].urb.as_ptr() != urb {
                        return Err(io::Error::other("reaped a URB this pool never submitted"));
                    }
                    self.in_flight[index] = false;
                    done.push(index);
                }
                Err(error) if error.raw_os_error() == Some(libc::EAGAIN) => return Ok(done),
                Err(error) => return Err(error),
            }
        }
    }

    pub fn transfer(&self, index: usize) -> &Transfer {
        &self.transfers[index]
    }

    /// Discard every in-flight transfer and reap them back, so no memory the kernel may
    /// write is freed. Bounded: a discarded URB completes promptly, and a disconnected
    /// device fails every one at once.
    pub fn cancel(&mut self) {
        for (index, transfer) in self.transfers.iter().enumerate() {
            if self.in_flight[index] {
                // SAFETY: DISCARDURB takes the submitted URB's address.
                let _ = unsafe { ioctl(self.device.fd, DISCARD_URB, transfer.urb.as_ptr().cast()) };
            }
        }
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while self.in_flight.iter().any(|f| *f) && std::time::Instant::now() < deadline {
            match self.reap() {
                Ok(done) if done.is_empty() => {
                    let _ = self.device.wait(20);
                }
                Ok(_) => {}
                Err(_) => break,
            }
        }
    }
}

impl Drop for Pool {
    fn drop(&mut self) {
        self.cancel();
        if self.in_flight.iter().any(|f| *f) {
            // The kernel still owns memory we would free: leak it rather than corrupt.
            for transfer in std::mem::take(&mut self.transfers) {
                std::mem::forget(transfer);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The ioctl numbers `linux/usbdevice_fs.h` yields on LP64; a wrong struct size would
    /// change them and every transfer would fail with ENOTTY.
    #[test]
    fn ioctl_numbers_match_the_kernel_headers_on_lp64() {
        assert_eq!(size_of::<CtrlTransfer>(), 24);
        assert_eq!(size_of::<BulkTransfer>(), 24);
        assert_eq!(size_of::<Urb>(), 56);
        assert_eq!(size_of::<IsoPacket>(), 12);
        assert_eq!(CONTROL, 0xc018_5500);
        assert_eq!(BULK, 0xc018_5502);
        assert_eq!(SET_INTERFACE, 0x8008_5504);
        assert_eq!(SUBMIT_URB, 0x8038_550a);
        assert_eq!(DISCARD_URB, 0x0000_550b);
        assert_eq!(REAP_URB_NDELAY, 0x4008_550d);
        assert_eq!(CLAIM_INTERFACE, 0x8004_550f);
        assert_eq!(RELEASE_INTERFACE, 0x8004_5510);
        assert_eq!(IOCTL, 0xc010_5512);
        assert_eq!(CLEAR_HALT, 0x8004_5515);
        assert_eq!(DISCONNECT, 0x0000_5516);
        assert_eq!(CONNECT, 0x0000_5517);
        assert_eq!(GET_SPEED, 0x0000_551f);
        assert_eq!(size_of::<ConnInfoEx>(), 24);
        assert_eq!(CONNINFO_EX, 0x8018_5520);
    }

    #[test]
    fn a_port_path_is_bus_then_the_hub_ports() {
        let info = ConnInfoEx {
            size: 24,
            busnum: 1,
            devnum: 9,
            speed: 3,
            num_ports: 2,
            ports: [1, 3, 0, 0, 0, 0, 0],
        };
        assert_eq!(port_path(&info).unwrap(), "1-1.3");
        assert!(port_path(&ConnInfoEx::default()).is_err());
    }

    #[test]
    fn iso_transfer_lays_packets_back_to_back() {
        let transfer = Transfer::isochronous(0x81, 4, 3072, 7);
        // SAFETY: test-only read of a transfer never submitted.
        let urb = unsafe { &*transfer.urb.as_ptr() };
        assert_eq!(urb.kind, URB_TYPE_ISO);
        assert_eq!(urb.flags, URB_ISO_ASAP);
        assert_eq!(urb.packets_or_stream, 4);
        assert_eq!(urb.buffer_length, 4 * 3072);
        assert_eq!(urb.context as usize, 7);
        let (status, bytes) = transfer.packet(3);
        assert_eq!((status, bytes.len()), (0, 0));
    }

    /// Reading descriptors from a real usbfs node, when this host has one readable: the
    /// root hubs are world-readable on stock Linux. Proves the read path and the parser
    /// against kernel-produced bytes; it prints what it saw.
    #[test]
    #[cfg(target_os = "linux")]
    fn host_usbfs_nodes_parse() {
        use std::os::fd::AsRawFd;
        let mut parsed = 0;
        for bus in std::fs::read_dir("/dev/bus/usb")
            .into_iter()
            .flatten()
            .flatten()
        {
            for node in std::fs::read_dir(bus.path())
                .into_iter()
                .flatten()
                .flatten()
            {
                let Ok(file) = std::fs::File::open(node.path()) else {
                    continue;
                };
                let bytes = Device::borrow(file.as_raw_fd()).descriptors().unwrap();
                let descriptors = super::super::descriptor::parse(&bytes)
                    .unwrap_or_else(|e| panic!("{}: {e}", node.path().display()));
                eprintln!(
                    "{}: {:04x}:{:04x}, {} configuration(s)",
                    node.path().display(),
                    descriptors.device.vendor_id,
                    descriptors.device.product_id,
                    descriptors.configurations.len()
                );
                parsed += 1;
            }
        }
        eprintln!("parsed {parsed} host usbfs node(s)");
    }
}
