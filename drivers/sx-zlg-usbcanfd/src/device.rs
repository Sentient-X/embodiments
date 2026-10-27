//! One adapter, opened once per process, and its CAN channels as `CanPort`s.
//!
//! [`UsbCanFd::open`] finds the adapter by its USB serial number, claims interface 0 (which
//! the operating system grants to one owner: a second process holding it makes the claim fail
//! with [`ZlgError::Busy`], which names the holding process where the host tells it),
//! probes it as the specification's §1.3 does and starts the I/O thread. Opening the same
//! serial again in this process returns the same adapter, so two channels of one adapter can
//! be opened by two callers. [`UsbCanFd::open_channel`] runs the channel lifecycle of §3.8 and
//! returns a [`Channel`], which closes the channel when dropped.

use std::io;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex, Weak};
use std::task::Context;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use nusb::transfer::{Buffer, Bulk, In, Out};
use nusb::{MaybeFuture, Speed};
use sx_embodiment_drivers::Holder;
use sx_embodiment_drivers::can::{CanFrame, CanPort, RoundTrip, RoundTripWindow};
use thiserror::Error;

use crate::engine::{
    Buffers, ChannelStats, DeviceStats, Engine, EngineConfig, RX_RING, RX_TRANSFER_LEN,
    RX_TRANSFERS, Shared, Stall, ThreadWaker, TxBudget, TxFailure, TxSide, TxState, lock,
};
use crate::timing::{Bitrate, TimingError};
use crate::usb::{BulkPipe, CommandError, Commands, control_in};
use crate::wire::{Command, DeviceInformation, MAX_BATCH, MAX_TX_TRANSFER, TxFrame, init_payload};

/// The adapters this crate drives: (vendor, product).
pub const DEVICES: [(u16, u16); 2] = [(0x04CC, 0x1240), (0x3068, 0x0009)];

/// The device's default transmit timeout (§3.4).
pub const DEFAULT_TRANSMIT_TIMEOUT_MS: u32 = 20;
/// How long the I/O thread sleeps with nothing to do before it looks again.
const IDLE_RECHECK: Duration = Duration::from_millis(100);
/// How much of a receive's wait is spun rather than slept: a sleeping thread wakes late
/// (macOS: about 0.75 ms at p50 for a default thread, README "Waiting on macOS"), so the last
/// stretch before the deadline is polled, and an answer landing just before it is taken.
const SPIN_TAIL: Duration = Duration::from_millis(1);

/// Why an adapter or channel could not be opened or used.
#[derive(Debug, Error)]
pub enum ZlgError {
    #[error("no ZLG USBCANFD adapter{} is attached", .0.as_ref().map(|serial| format!(" with USB serial {serial}")).unwrap_or_default())]
    NotFound(Option<String>),
    /// The operating system refused the interface claim because it is held: one process owns
    /// an adapter, and the holder is named where the host tells it.
    #[error(
        "ZLG adapter {serial} is in use, {holder}: {detail}. One process owns a bus: stop that process first (with no process named on Linux, a driver bound to it: `ls /sys/bus/usb/devices/*/driver` names it)"
    )]
    Busy {
        serial: String,
        detail: String,
        holder: Holder,
    },
    #[error("ZLG adapter {serial}: {detail}")]
    Usb { serial: String, detail: String },
    #[error(transparent)]
    Command(#[from] CommandError),
    #[error(transparent)]
    Timing(#[from] TimingError),
    #[error("channel {index} is not one of the adapter's {channels}")]
    NoChannel { index: u8, channels: u8 },
    #[error("channel {0} is already open in this process")]
    ChannelOpen(u8),
}

/// One attached adapter, as enumeration reports it, before it is opened.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Found {
    pub serial: Option<String>,
    pub vendor_id: u16,
    pub product_id: u16,
    pub speed: Option<String>,
}

/// Every attached adapter.
///
/// # Errors
///
/// Returns the operating system's enumeration failure.
pub fn list() -> Result<Vec<Found>, ZlgError> {
    let devices = nusb::list_devices().wait().map_err(|error| ZlgError::Usb {
        serial: String::new(),
        detail: error.to_string(),
    })?;
    Ok(devices
        .filter(|info| DEVICES.contains(&(info.vendor_id(), info.product_id())))
        .map(|info| Found {
            serial: info.serial_number().map(str::to_owned),
            vendor_id: info.vendor_id(),
            product_id: info.product_id(),
            speed: info.speed().map(speed_name),
        })
        .collect())
}

fn speed_name(speed: Speed) -> String {
    match speed {
        Speed::Low => "low (1.5 Mbit/s)",
        Speed::Full => "full (12 Mbit/s)",
        Speed::High => "high (480 Mbit/s)",
        Speed::Super => "super (5 Gbit/s)",
        _ => "faster than super",
    }
    .to_owned()
}

/// Adapter-wide settings, fixed when it is first opened in this process.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Options {
    /// The device's transmit timeout, ms, 1–4000 (§3.4); re-sent on every channel open.
    pub transmit_timeout_ms: u32,
    /// Transmit transfers outstanding at once: 1 as the reference; 2 is an experiment whose
    /// safety on the device is unknown (§9.2).
    pub in_flight: usize,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            transmit_timeout_ms: DEFAULT_TRANSMIT_TIMEOUT_MS,
            in_flight: 1,
        }
    }
}

/// How a caller waiting for a frame waits.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize)]
pub enum ReceiveWait {
    /// Sleep until the I/O thread delivers a frame, polling only the last millisecond before
    /// the deadline (`SPIN_TAIL`), which a sleeping thread would overshoot.
    #[default]
    Block,
    /// Poll without sleeping, a CPU core per waiting caller, for whatever the wake-up costs.
    /// Opt-in only; the README gives the measured difference.
    Spin,
}

/// How a channel is opened.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChannelOptions {
    pub bitrate: Bitrate,
    /// The channel's 120 Ω termination; the reference enables it (§3.4).
    pub termination: bool,
    /// How [`CanPort::receive_until`] waits.
    pub receive_wait: ReceiveWait,
}

impl Default for ChannelOptions {
    fn default() -> Self {
        Self {
            bitrate: Bitrate::CLASSIC_1M,
            termination: true,
            receive_wait: ReceiveWait::Block,
        }
    }
}

/// How a stalled channel was brought back (§3.8: there is no per-channel reset; stop, then
/// initialise and start again).
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
pub enum Recovery {
    /// Channel stop, initialise, transmit timeout, termination, start.
    ChannelRestart,
    /// Channel stop, device close, device open, then the channel's open again; tried when a
    /// restart did not hold and no other channel is open.
    DeviceReopen,
}

/// What the transmit side reports to a sender, inside the [`io::Error`] its `send` returns
/// (`error.get_ref()` downcasts to it).
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum LinkError {
    /// `WouldBlock`: the queue already holds its bus-time budget; nothing was queued.
    #[error(
        "ZLG channel {channel}: the transmit queue is full: it holds {queued:?} of bus time, its budget being {budget:?} (half the device transmit timeout); the frame was not queued, send it once the bus has drained"
    )]
    QueueFull {
        channel: u8,
        queued: Duration,
        budget: Duration,
    },
    /// `Other`: a batch failed since the last send; reported once.
    #[error("ZLG channel {channel}: {failure}")]
    BatchFailed { channel: u8, failure: TxFailure },
    /// `Interrupted`: the channel had stalled and was recovered without a replug; its queue
    /// was dropped and this frame was not sent.
    #[error(
        "ZLG channel {channel}: {cause}; recovered by {action:?} without a replug, {dropped} queued frames were dropped and this frame was not sent: send again"
    )]
    Recovered {
        channel: u8,
        cause: Stall,
        action: Recovery,
        dropped: u64,
    },
    /// `WouldBlock`: another caller is recovering the channel.
    #[error("ZLG channel {channel}: the channel is being recovered; send again")]
    Recovering { channel: u8 },
    /// `NotConnected`: every batch was refused (the bus did not take it within the transmit
    /// timeout: no node acknowledged) and nothing was received meanwhile. The adapter answers
    /// its commands and the channel was restarted; a replug cannot help an empty bus. The next
    /// send tries again, so the channel works the moment the bus answers.
    #[error(
        "Nothing on CAN{channel} is answering: check the cable to the motors, its termination and motor power"
    )]
    NothingAnswering { channel: u8 },
    /// `NotConnected`: recovery failed or did not hold; only a power cycle helps.
    #[error(
        "ZLG channel {channel}: {cause}, and recovery failed: {detail}. The adapter must be power-cycled: unplug it and plug it back in"
    )]
    PowerCycle {
        channel: u8,
        cause: Stall,
        detail: String,
    },
}

impl LinkError {
    #[must_use]
    pub const fn kind(&self) -> io::ErrorKind {
        match self {
            Self::QueueFull { .. } | Self::Recovering { .. } => io::ErrorKind::WouldBlock,
            Self::BatchFailed { .. } => io::ErrorKind::Other,
            Self::Recovered { .. } => io::ErrorKind::Interrupted,
            Self::PowerCycle { .. } | Self::NothingAnswering { .. } => io::ErrorKind::NotConnected,
        }
    }

    /// The link error inside `error`, if it carries one.
    #[must_use]
    pub fn of(error: &io::Error) -> Option<&Self> {
        error.get_ref()?.downcast_ref()
    }
}

impl From<LinkError> for io::Error {
    fn from(error: LinkError) -> Self {
        Self::new(error.kind(), error)
    }
}

/// What probing found (§1.3), and the USB facts read from descriptors (§9.6).
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize)]
pub struct Probe {
    pub usb_serial: Option<String>,
    /// High byte major, low byte minor; 0 when the request failed.
    pub protocol_version: u16,
    /// The largest transmit transfer the device accepts, bytes.
    pub max_transfer: u32,
    /// Frames per transmit transfer.
    pub batch_limit: usize,
    pub microsecond_timestamps: bool,
    pub hardware_version: u16,
    pub firmware_version: u16,
    pub bootloader_version: u16,
    pub identity: String,
    pub device_serial: String,
    pub can_ports: u8,
    pub lin_ports: u8,
    pub clock_hz: u32,
    pub usb_speed: Option<String>,
    /// wMaxPacketSize of 0x01, 0x81, 0x02, 0x82, 0x83.
    pub packet_sizes: [usize; 5],
    /// Whether this open found the device unconfigured and selected its configuration (on
    /// macOS through `IOUSBHost`, the first open after each plug-in).
    pub configured_on_open: bool,
}

/// The channels' lifecycle state, behind the command lock.
struct Lifecycle {
    commands: Commands,
    open: [bool; 4],
}

/// One opened adapter; dropped when its last handle and channel are.
struct Core {
    shared: Arc<Shared>,
    lifecycle: Mutex<Lifecycle>,
    probe: Probe,
    options: Options,
    io: Mutex<Option<JoinHandle<()>>>,
    /// Cores kept able to wake at once while the adapter is open (`sx_realtime`), where the
    /// host grants it: on the Spark a completion woken on a deeply idle core waited up to
    /// 433 µs.
    _wake: Option<sx_realtime::WakeLatency>,
}

impl Drop for Core {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        self.shared.wake();
        if let Some(thread) = lock(&self.io).take() {
            let _ = thread.join();
        }
    }
}

impl Core {
    fn transmit_timeout(&self) -> Duration {
        Duration::from_millis(u64::from(self.options.transmit_timeout_ms.clamp(1, 4000)))
    }

    fn budget(&self, bitrate: Bitrate) -> TxBudget {
        TxBudget::new(bitrate, self.transmit_timeout(), self.probe.batch_limit)
    }
}

/// The adapters open in this process, by USB serial.
static OPEN: Mutex<Vec<(String, Weak<Core>)>> = Mutex::new(Vec::new());

/// An opened adapter.
#[derive(Clone)]
pub struct UsbCanFd {
    core: Arc<Core>,
}

impl UsbCanFd {
    /// Open the adapter with USB serial `serial`, or the only attached one when `None`, with
    /// default [`Options`].
    ///
    /// # Errors
    ///
    /// As [`UsbCanFd::open_with`].
    pub fn open(serial: Option<&str>) -> Result<Self, ZlgError> {
        Self::open_with(serial, Options::default())
    }

    /// Open the adapter, or return the one this process already opened with that serial (its
    /// first options stand).
    ///
    /// # Errors
    ///
    /// [`ZlgError::NotFound`] when no such adapter is attached or several are and none was
    /// named, [`ZlgError::Busy`] when its interface is held, and any probe failure.
    pub fn open_with(serial: Option<&str>, options: Options) -> Result<Self, ZlgError> {
        let mut open = lock(&OPEN);
        open.retain(|(_, core)| core.strong_count() > 0);
        if let Some(serial) = serial
            && let Some(core) = open
                .iter()
                .find(|(open_serial, _)| open_serial == serial)
                .and_then(|(_, core)| core.upgrade())
        {
            return Ok(Self { core });
        }
        let infos: Vec<nusb::DeviceInfo> = nusb::list_devices()
            .wait()
            .map_err(|error| ZlgError::Usb {
                serial: String::new(),
                detail: error.to_string(),
            })?
            .filter(|info| DEVICES.contains(&(info.vendor_id(), info.product_id())))
            .filter(|info| serial.is_none_or(|serial| info.serial_number() == Some(serial)))
            .collect();
        let [info] = infos.as_slice() else {
            return Err(ZlgError::NotFound(serial.map(str::to_owned)));
        };
        let core = Arc::new(probe(info, options)?);
        if let Some(serial) = &core.probe.usb_serial {
            open.push((serial.clone(), Arc::downgrade(&core)));
        }
        Ok(Self { core })
    }

    #[must_use]
    pub fn probe(&self) -> &Probe {
        &self.core.probe
    }

    #[must_use]
    pub fn options(&self) -> Options {
        self.core.options
    }

    #[must_use]
    pub fn stats(&self) -> DeviceStats {
        self.core.shared.device.snapshot()
    }

    /// Open CAN channel `index` (0-based; the adapter's second channel is 1): device open if no
    /// channel is open, initialise, transmit timeout, termination, start (§3.8).
    ///
    /// # Errors
    ///
    /// Refuses a channel the adapter lacks or one already open, a bit rate its clock cannot
    /// time, and any command the device refuses or USB fails.
    pub fn open_channel(&self, index: u8, options: ChannelOptions) -> Result<Channel, ZlgError> {
        let core = &self.core;
        let channels = core.probe.can_ports.min(4);
        if index >= channels {
            return Err(ZlgError::NoChannel { index, channels });
        }
        options.bitrate.timings(core.probe.clock_hz)?;
        let mut lifecycle = lock(&core.lifecycle);
        let slot = usize::from(index);
        if lifecycle.open[slot] {
            return Err(ZlgError::ChannelOpen(index));
        }
        let first = !lifecycle.open.iter().any(|open| *open);
        if first {
            lifecycle.commands.run(Command::DeviceOpen, 0, &[])?;
        }
        let state = &core.shared.channels[slot];
        lock(&state.rx).ring.clear();
        lock(&state.rx).unreported_loss = 0;
        *lock(&state.tx) = TxSide::new(options.bitrate, core.budget(options.bitrate));
        state.open.store(true, Ordering::Relaxed);
        if let Err(error) = start(&mut lifecycle.commands, core, index, options) {
            state.open.store(false, Ordering::Relaxed);
            let _ = lifecycle.commands.run(Command::ChannelStop, index, &[]);
            if first {
                let _ = lifecycle.commands.run(Command::DeviceClose, 0, &[]);
            }
            return Err(error);
        }
        lifecycle.open[slot] = true;
        Ok(Channel {
            core: core.clone(),
            index,
            options,
            round_trips: RoundTripWindow::default(),
        })
    }
}

/// Initialise and start channel `index` (§3.8 steps 3–6).
fn start(
    commands: &mut Commands,
    core: &Core,
    index: u8,
    options: ChannelOptions,
) -> Result<(), ZlgError> {
    let (nominal, data) = options.bitrate.timings(core.probe.clock_hz)?;
    let timeout = core.options.transmit_timeout_ms.clamp(1, 4000);
    let termination = if options.termination { 0x55 } else { 0xAA };
    let steps: [(Command, u8, &[u8]); 4] = [
        (
            Command::ChannelInit,
            index,
            &init_payload(core.probe.clock_hz, 0, nominal, data),
        ),
        (Command::TransmitTimeout, 0, &timeout.to_le_bytes()),
        (Command::Termination, index, &[termination]),
        (Command::ChannelStart, index, &[]),
    ];
    for (command, argument, payload) in steps {
        commands.run(command, argument, payload)?;
    }
    Ok(())
}

/// Select the device's one configuration when the host left it unconfigured. macOS leaves a
/// vendor-class device that no driver matches unconfigured; the legacy `SetConfiguration` `nusb`
/// would send fails on this adapter and breaks its control pipe until it is replugged, so on
/// macOS `IOUSBHost` selects it ([`crate::macos`]) before `nusb` opens the device, and `nusb`'s
/// `set_configuration` is never called. The adapter is never reset.
#[cfg(target_os = "macos")]
fn configure_before_open(info: &nusb::DeviceInfo) -> Result<bool, String> {
    crate::macos::ensure_configured(info.registry_entry_id())
        .map(|done| done == crate::macos::Configured::Now)
}

#[cfg(not(target_os = "macos"))]
#[allow(clippy::unnecessary_wraps)]
const fn configure_before_open(_info: &nusb::DeviceInfo) -> Result<bool, String> {
    Ok(false)
}

/// Elsewhere (Linux configures a device when it enumerates) select the configuration through
/// `nusb` only if the device is somehow unconfigured.
#[cfg(not(target_os = "macos"))]
fn configure(device: &nusb::Device) -> Result<(), String> {
    if device.active_configuration().is_ok() {
        return Ok(());
    }
    let value = device
        .configurations()
        .next()
        .map(|configuration| configuration.configuration_value())
        .ok_or_else(|| "the device declares no configuration".to_owned())?;
    device
        .set_configuration(value)
        .wait()
        .map_err(|error| format!("set configuration {value}: {error}"))
}

#[cfg(target_os = "macos")]
#[allow(clippy::unnecessary_wraps)]
const fn configure(_device: &nusb::Device) -> Result<(), String> {
    Ok(())
}

/// Who holds the adapter's interface: on Linux the processes with its usbfs node open, on
/// macOS the I/O Registry's `UsbExclusiveOwner` of its interface.
#[cfg(target_os = "linux")]
fn holder(info: &nusb::DeviceInfo) -> Holder {
    sx_embodiment_drivers::holder_of(std::path::Path::new(&format!(
        "/dev/bus/usb/{:03}/{:03}",
        info.busnum(),
        info.device_address()
    )))
}

#[cfg(target_os = "macos")]
fn holder(info: &nusb::DeviceInfo) -> Holder {
    crate::macos::exclusive_owner(info.registry_entry_id()).map_or(Holder::Unknown, |owner| {
        Holder::from_exclusive_owner(&owner)
    })
}

#[cfg(not(any(target_os = "linux", target_os = "macos")))]
fn holder(_info: &nusb::DeviceInfo) -> Holder {
    Holder::Unknown
}

/// Probe an adapter (§1.3), claim its endpoints and start its I/O thread.
fn probe(info: &nusb::DeviceInfo, options: Options) -> Result<Core, ZlgError> {
    let usb_serial = info.serial_number().map(str::to_owned);
    let name = usb_serial
        .clone()
        .unwrap_or_else(|| "(no USB serial)".to_owned());
    let usb = |detail: String| ZlgError::Usb {
        serial: name.clone(),
        detail,
    };
    let configured_on_open = configure_before_open(info).map_err(usb)?;
    let device = info.open().wait().map_err(|error| usb(error.to_string()))?;
    configure(&device).map_err(usb)?;
    let interface = device.claim_interface(0).wait().map_err(|error| {
        if error.kind() == nusb::ErrorKind::Busy {
            ZlgError::Busy {
                serial: name.clone(),
                detail: error.to_string(),
                holder: holder(info),
            }
        } else {
            usb(error.to_string())
        }
    })?;
    let (protocol_version, max_transfer) = transfer_limits(&interface);
    let endpoint = |error: nusb::Error| usb(format!("endpoint: {error}"));
    let command_out = interface.endpoint::<Bulk, Out>(0x01).map_err(endpoint)?;
    let command_in = interface.endpoint::<Bulk, In>(0x81).map_err(endpoint)?;
    let writer = interface.endpoint::<Bulk, Out>(0x02).map_err(endpoint)?;
    let replies = interface.endpoint::<Bulk, In>(0x82).map_err(endpoint)?;
    let rx = interface.endpoint::<Bulk, In>(0x83).map_err(endpoint)?;
    let packet_sizes = [
        command_out.max_packet_size(),
        command_in.max_packet_size(),
        writer.max_packet_size(),
        replies.max_packet_size(),
        rx.max_packet_size(),
    ];
    let mut commands = Commands::new(command_out, command_in);
    let microsecond_timestamps = timestamps(&mut commands, protocol_version)?;
    let batch_limit = (usize::try_from(max_transfer / 1024).unwrap_or(1) * 12).clamp(1, MAX_BATCH);
    let information =
        DeviceInformation::parse(&commands.run(Command::DeviceInformation, 0, &[])?)
            .map_err(|error| usb(format!("device information: {error}")))?;
    let channels = information.channels();
    let probe = Probe {
        usb_serial,
        protocol_version,
        max_transfer,
        batch_limit,
        microsecond_timestamps,
        hardware_version: information.hardware_version,
        firmware_version: information.firmware_version,
        bootloader_version: information.bootloader_version,
        clock_hz: information.clock_hz(),
        identity: information.identity,
        device_serial: information.serial,
        can_ports: channels,
        lin_ports: information.lin_ports.min(2),
        usb_speed: device.speed().map(speed_name),
        packet_sizes,
        configured_on_open,
    };
    let shared = Arc::new(Shared::new(channels, RX_RING));
    let packet = rx.max_packet_size().max(1);
    let rx_request = RX_TRANSFER_LEN.div_ceil(packet) * packet;
    let in_flight = options.in_flight.clamp(1, 2);
    let config = EngineConfig {
        can_ports: channels,
        batch_limit,
        reply_request: replies.max_packet_size().max(8),
        rx_request,
        reply_timeout: Duration::from_millis(
            1000 + u64::from(options.transmit_timeout_ms.clamp(1, 4000)),
        ),
        transmit_timeout: Duration::from_millis(u64::from(
            options.transmit_timeout_ms.clamp(1, 4000),
        )),
        in_flight,
    };
    let buffers = Buffers {
        rx: (0..RX_TRANSFERS).map(|_| rx.allocate(rx_request)).collect(),
        out: (0..in_flight)
            .map(|_| writer.allocate(MAX_TX_TRANSFER))
            .collect(),
        replies: (0..in_flight)
            .map(|_| Buffer::new(config.reply_request))
            .collect(),
    };
    let io = start_io(&shared, config, (rx, writer, replies), buffers).map_err(usb)?;
    Ok(Core {
        shared,
        lifecycle: Mutex::new(Lifecycle {
            commands,
            open: [false; 4],
        }),
        probe,
        options,
        io: Mutex::new(Some(io)),
        _wake: wake_latency(),
    })
}

/// The protocol version (0 when the request fails) and the largest transmit transfer, 1024
/// unless a device of protocol 1.13 or later reports it (§1.2).
fn transfer_limits(interface: &nusb::Interface) -> (u16, u32) {
    let protocol_version = control_in(interface, 0x00, 4)
        .ok()
        .filter(|reply| reply.len() >= 2)
        .map_or(0, |reply| u16::from_le_bytes([reply[0], reply[1]]));
    let max_transfer = if protocol_version >= 0x0113 {
        control_in(interface, 0x01, 4)
            .ok()
            .filter(|reply| reply.len() >= 4)
            .map_or(1024, |reply| {
                u32::from_le_bytes([reply[0], reply[1], reply[2], reply[3]])
            })
    } else {
        1024
    };
    (protocol_version, max_transfer)
}

/// Switch received timestamps to microseconds where the device supports it (§1.3 step 3); a
/// failed read leaves the setting alone. Whether microseconds are in force.
fn timestamps(commands: &mut Commands, protocol_version: u16) -> Result<bool, ZlgError> {
    if protocol_version < 0x0116 {
        return Ok(false);
    }
    let Ok(reply) = commands.run(Command::ReadTransferProperties, 0, &[]) else {
        return Ok(false);
    };
    if reply.first() != Some(&1) {
        commands.run(Command::WriteTransferProperties, 0, &[1])?;
    }
    Ok(true)
}

type DataEndpoints = (
    nusb::Endpoint<Bulk, In>,
    nusb::Endpoint<Bulk, Out>,
    nusb::Endpoint<Bulk, In>,
);

/// Keep the host's cores able to wake at once while the adapter is open, where it is granted;
/// a refusal is reported once and the adapter runs without it.
fn wake_latency() -> Option<sx_realtime::WakeLatency> {
    sx_realtime::hold_wake_latency()
        .inspect_err(|refused| {
            if *refused != sx_realtime::Refused::Unsupported {
                eprintln!("zlg-usbcanfd: the CPU wake-latency request was {refused}");
            }
        })
        .ok()
}

/// Start the I/O thread, returning once it has queued its receive reads.
fn start_io(
    shared: &Arc<Shared>,
    config: EngineConfig,
    (rx, writer, replies): DataEndpoints,
    buffers: Buffers<Buffer, Buffer, Buffer>,
) -> Result<JoinHandle<()>, String> {
    let thread_shared = shared.clone();
    let (ready, started) = std::sync::mpsc::sync_channel(1);
    let io = std::thread::Builder::new()
        .name("zlg-usbcanfd-io".to_owned())
        .spawn(move || {
            io_thread_realtime();
            thread_shared.set_io_thread(std::thread::current());
            let engine = Engine::new(
                thread_shared.clone(),
                config,
                (BulkPipe(rx), BulkPipe(writer), BulkPipe(replies)),
                buffers,
            );
            let _ = ready.send(());
            run(engine, &thread_shared);
        })
        .map_err(|error| format!("I/O thread: {error}"))?;
    let _ = started.recv();
    Ok(io)
}

/// Put the threads every answer passes through under the host's real-time policy
/// (`sx_realtime`, [`sx_realtime::Role::AnswerPath`]): this I/O thread and `nusb`'s
/// `nusb-events`, which delivers every USB completion on macOS and reaps every URB on Linux.
/// On an overloaded host (macOS at load 30–100 parked default threads for 30–100 ms on
/// 2026-10-26) either one at default priority held answers past their phase's deadline, and the
/// chain read a motor as silent (`Silent(<id>)`) though it had answered. Each does under 0.1 ms
/// of work per wake (the I/O thread about 6 % of a core at 250 Hz, README). A refusal leaves a
/// thread at default priority, reported once, with what the host must grant.
fn io_thread_realtime() {
    use sx_realtime::{Role, elevate, elevate_named};
    if let Err(refused) = elevate(Role::AnswerPath) {
        eprintln!("zlg-usbcanfd: the I/O thread's real-time policy was {refused}");
    }
    match elevate_named("nusb-events", Role::AnswerPath) {
        Ok(0) => eprintln!("zlg-usbcanfd: no nusb-events thread to put under the real-time policy"),
        Ok(_) => {}
        Err(refused) => eprintln!("zlg-usbcanfd: nusb-events' real-time policy was {refused}"),
    }
}

/// The I/O thread: step the engine, and park when nothing moved until a completion, a queued
/// frame, the oldest batch's deadline or the idle recheck.
fn run(mut engine: Engine<BulkPipe<In>, BulkPipe<Out>, BulkPipe<In>>, shared: &Shared) {
    let waker = std::task::Waker::from(Arc::new(ThreadWaker(std::thread::current())));
    let mut cx = Context::from_waker(&waker);
    while !shared.stop.load(Ordering::Relaxed) {
        let now = Instant::now();
        let step = engine.step(&mut cx, now);
        if !step.progressed {
            let wait = step
                .deadline
                .map_or(IDLE_RECHECK, |deadline| {
                    deadline.saturating_duration_since(now)
                })
                .min(IDLE_RECHECK);
            std::thread::park_timeout(wait);
        }
    }
    engine.shutdown(Duration::from_secs(1));
}

/// Whether a stall means nothing on the bus answers: the adapter refused every batch of the
/// run (status 1: the bus did not take it within the transmit timeout, which on CAN means no
/// node acknowledged) and not one frame arrived on the channel since the run began.
fn nothing_answering(cause: Stall, received: u64, mark: u64) -> bool {
    matches!(cause, Stall::Refused { .. }) && received == mark
}

/// One open CAN channel; closes it when dropped (§3.8).
pub struct Channel {
    core: Arc<Core>,
    index: u8,
    options: ChannelOptions,
    /// How long the motor protocol's commands waited for their answers ([`CanPort::answered`]).
    round_trips: RoundTripWindow,
}

impl Channel {
    #[must_use]
    pub const fn index(&self) -> u8 {
        self.index
    }

    #[must_use]
    pub const fn options(&self) -> ChannelOptions {
        self.options
    }

    /// The adapter this channel is on.
    #[must_use]
    pub fn adapter(&self) -> UsbCanFd {
        UsbCanFd {
            core: self.core.clone(),
        }
    }

    #[must_use]
    pub fn stats(&self) -> ChannelStats {
        self.state().counters.snapshot()
    }

    /// The latest error record's raw bytes; their layout is undocumented (§5.4).
    #[must_use]
    pub fn last_error_record(&self) -> Option<[u8; 8]> {
        lock(&self.state().rx).last_error
    }

    /// The bus-time budget this channel's transmits are held to.
    #[must_use]
    pub fn budget(&self) -> TxBudget {
        lock(&self.state().tx).budget
    }

    /// Queue `frame` (classic or FD) for transmission.
    ///
    /// # Errors
    ///
    /// As [`CanPort::send`]; every refusal carries a [`LinkError`].
    pub fn send_frame(&mut self, frame: TxFrame) -> io::Result<()> {
        self.usable()?;
        let state = self.state();
        let mut tx = lock(&state.tx);
        match &tx.state {
            TxState::Running | TxState::Held { .. } => {}
            TxState::Stalled(cause) => {
                let cause = *cause;
                drop(tx);
                return Err(self.recover(cause).into());
            }
            TxState::Recovering => {
                return Err(LinkError::Recovering {
                    channel: self.index,
                }
                .into());
            }
            TxState::Dead { cause, detail } => {
                return Err(LinkError::PowerCycle {
                    channel: self.index,
                    cause: *cause,
                    detail: detail.clone(),
                }
                .into());
            }
        }
        if let Some(failure) = tx.failure.take() {
            return Err(LinkError::BatchFailed {
                channel: self.index,
                failure,
            }
            .into());
        }
        if tx.push(frame).is_err() {
            state.counters.tx_queue_full.fetch_add(1, Ordering::Relaxed);
            return Err(LinkError::QueueFull {
                channel: self.index,
                queued: tx.queued,
                budget: tx.budget.queue,
            }
            .into());
        }
        drop(tx);
        state.counters.tx_queued.fetch_add(1, Ordering::Relaxed);
        self.core.shared.wake();
        Ok(())
    }

    /// Mark the channel stalled as if its adapter had stopped replying, so the next send runs
    /// the real recovery against the adapter: a hardware check of [`Recovery`] that sends only
    /// the channel lifecycle's commands.
    #[doc(hidden)]
    pub fn inject_stall(&self) {
        self.state().stall(Stall::NoReply { frames: 0 });
    }

    /// Bring a stalled channel back without a replug (§3.8): first a channel restart; if the
    /// channel stalls again before any batch is accepted, a device close and open (only when no
    /// other channel is open, since it stops them too); after that, or when the adapter does not
    /// answer the commands, the channel is dead until the adapter is power-cycled.
    fn recover(&self, cause: Stall) -> LinkError {
        let core = &self.core;
        let mut lifecycle = lock(&core.lifecycle);
        let state = self.state();
        let attempt = {
            let mut tx = lock(&state.tx);
            if tx.state != TxState::Stalled(cause) {
                return LinkError::Recovering {
                    channel: self.index,
                };
            }
            tx.state = TxState::Recovering;
            let received = state.counters.rx_frames.load(Ordering::Relaxed)
                + state.counters.rx_lost.load(Ordering::Relaxed);
            (
                tx.recoveries_since_success,
                nothing_answering(cause, received, tx.refusal_rx_mark),
            )
        };
        let (attempt, silent) = attempt;
        let alone = lifecycle
            .open
            .iter()
            .enumerate()
            .all(|(slot, open)| !open || slot == usize::from(self.index));
        // The usual ladder: a channel restart, then a device reopen (what a replug would do, in
        // case the adapter's CAN side is stuck). An empty bus is not the adapter's fault, so a
        // silent stall past the ladder restarts the channel again instead of declaring the
        // adapter dead: the channel works the moment the bus answers.
        let action = match attempt {
            0 => Some(Recovery::ChannelRestart),
            1 if alone => Some(Recovery::DeviceReopen),
            _ if silent => Some(Recovery::ChannelRestart),
            _ => None,
        };
        let result = match action {
            None => Err(format!(
                "the channel stalled again after {attempt} recoveries with no batch accepted between them"
            )),
            Some(action) => {
                let commands = &mut lifecycle.commands;
                let mut run = || -> Result<(), ZlgError> {
                    commands.run(Command::ChannelStop, self.index, &[])?;
                    if action == Recovery::DeviceReopen {
                        commands.run(Command::DeviceClose, 0, &[])?;
                        commands.run(Command::DeviceOpen, 0, &[])?;
                    }
                    start(commands, core, self.index, self.options)
                };
                run().map_err(|error| format!("{action:?}: {error}"))
            }
        };
        let mut tx = lock(&state.tx);
        let dropped = tx.clear() as u64;
        state
            .counters
            .tx_dropped
            .fetch_add(dropped, Ordering::Relaxed);
        tx.failure = None;
        tx.refusals = 0;
        match (result, action) {
            (Ok(()), Some(_)) if silent => {
                tx.state = TxState::Running;
                tx.recoveries_since_success += 1;
                state.counters.recoveries.fetch_add(1, Ordering::Relaxed);
                tx.nothing_answering = true;
                LinkError::NothingAnswering {
                    channel: self.index,
                }
            }
            (Ok(()), Some(action)) => {
                tx.state = TxState::Running;
                tx.nothing_answering = false;
                tx.recoveries_since_success += 1;
                state.counters.recoveries.fetch_add(1, Ordering::Relaxed);
                LinkError::Recovered {
                    channel: self.index,
                    cause,
                    action,
                    dropped,
                }
            }
            (result, _) => {
                let detail = result.err().unwrap_or_default();
                tx.state = TxState::Dead {
                    cause,
                    detail: detail.clone(),
                };
                LinkError::PowerCycle {
                    channel: self.index,
                    cause,
                    detail,
                }
            }
        }
    }

    /// The next received frame of any format.
    ///
    /// # Errors
    ///
    /// As [`CanPort::receive`].
    pub fn receive_frame(&mut self) -> io::Result<Option<crate::wire::RxFrame>> {
        self.usable()?;
        let mut rx = lock(&self.state().rx);
        if rx.unreported_loss > 0 {
            let lost = std::mem::take(&mut rx.unreported_loss);
            return Err(io::Error::other(format!(
                "ZLG channel {}: {lost} received frames were lost to a full receive ring",
                self.index
            )));
        }
        Ok(rx.ring.pop())
    }

    /// The next received frame of any format before `deadline`, waiting as the channel's
    /// [`ReceiveWait`] says: asleep until the I/O thread delivers one (polling the last
    /// millisecond before `deadline`), or polling throughout.
    ///
    /// # Errors
    ///
    /// As [`Channel::receive_frame`].
    pub fn receive_frame_until(
        &mut self,
        deadline: Instant,
    ) -> io::Result<Option<crate::wire::RxFrame>> {
        if self.options.receive_wait == ReceiveWait::Spin {
            loop {
                if let Some(frame) = self.receive_frame()? {
                    return Ok(Some(frame));
                }
                if Instant::now() >= deadline {
                    return Ok(None);
                }
                std::hint::spin_loop();
            }
        }
        let state = &self.core.shared.channels[usize::from(self.index)];
        let mut rx = lock(&state.rx);
        loop {
            if let Some(reason) = lock(&self.core.shared.fatal).as_ref() {
                return Err(io::Error::new(io::ErrorKind::NotConnected, reason.clone()));
            }
            if rx.unreported_loss > 0 {
                let lost = std::mem::take(&mut rx.unreported_loss);
                return Err(io::Error::other(format!(
                    "ZLG channel {}: {lost} received frames were lost to a full receive ring",
                    self.index
                )));
            }
            if let Some(frame) = rx.ring.pop() {
                return Ok(Some(frame));
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            if deadline - now <= SPIN_TAIL {
                drop(rx);
                std::hint::spin_loop();
                rx = lock(&state.rx);
                continue;
            }
            rx = state
                .rx_ready
                .wait_timeout(rx, (deadline - now).saturating_sub(SPIN_TAIL))
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .0;
        }
    }

    fn state(&self) -> &crate::engine::ChannelShared {
        &self.core.shared.channels[usize::from(self.index)]
    }

    fn usable(&self) -> io::Result<()> {
        match lock(&self.core.shared.fatal).as_ref() {
            Some(reason) => Err(io::Error::new(io::ErrorKind::NotConnected, reason.clone())),
            None => Ok(()),
        }
    }
}

impl CanPort for Channel {
    /// Queue one classic frame; an identifier above 11 bits is sent extended. Returns at once:
    /// the I/O thread sends it in the next batch. A batch that failed since the last send is
    /// reported here, once.
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        let extended = frame.id > 0x7FF;
        let frame = TxFrame::classic(frame.id, extended, &frame.data)
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
        self.send_frame(frame)
    }

    /// The next classic data frame, or `None` when nothing is pending. CAN FD frames longer
    /// than 8 bytes and remote frames are not classic data and are read past; a full receive
    /// ring's losses are reported once.
    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        loop {
            let Some(frame) = self.receive_frame()? else {
                return Ok(None);
            };
            if frame.remote || frame.len > 8 {
                continue;
            }
            let mut data = [0_u8; 8];
            data.copy_from_slice(&frame.data[..8]);
            return Ok(Some(CanFrame { id: frame.id, data }));
        }
    }

    /// As `receive`, waiting until `deadline` for a classic frame.
    fn receive_until(&mut self, deadline: Instant) -> io::Result<Option<CanFrame>> {
        loop {
            let Some(frame) = self.receive_frame_until(deadline)? else {
                return Ok(None);
            };
            if frame.remote || frame.len > 8 {
                continue;
            }
            let mut data = [0_u8; 8];
            data.copy_from_slice(&frame.data[..8]);
            return Ok(Some(CanFrame { id: frame.id, data }));
        }
    }

    /// A frame may wait behind one outstanding batch, then go in its own: each the device's
    /// transmit timeout at worst (if the device honours it per batch, which the specification
    /// presumes, §9.5) plus a millisecond of USB turnaround.
    fn frame_allowance(&self) -> Duration {
        let batch = Duration::from_millis(u64::from(self.core.options.transmit_timeout_ms) + 1);
        batch * 2
    }

    /// Answering once a frame has arrived; nothing answering after a stall of refused batches
    /// with nothing received; the adapter lost when it needs a power cycle or is disconnected.
    fn health(&self) -> sx_embodiment_drivers::can::LinkHealth {
        use sx_embodiment_drivers::can::LinkHealth;
        let bus = format!("CAN{}", self.index);
        if let Some(reason) = lock(&self.core.shared.fatal).as_ref() {
            return LinkHealth::AdapterLost {
                bus,
                detail: reason.clone(),
            };
        }
        let tx = lock(&self.state().tx);
        if let TxState::Dead { cause, .. } = &tx.state {
            return LinkHealth::AdapterLost {
                bus,
                detail: cause.to_string(),
            };
        }
        if tx.nothing_answering {
            return LinkHealth::NothingAnswering { bus };
        }
        // Refusals or a stall since the last accepted batch: not answering, whatever came
        // before (a cable pulled mid-run), until a batch is accepted or the stall is judged.
        let unsettled =
            tx.refusals > 0 || matches!(tx.state, TxState::Stalled(_) | TxState::Recovering);
        drop(tx);
        if unsettled {
            LinkHealth::Quiet
        } else if self.stats().rx_frames > 0 {
            LinkHealth::Answering
        } else {
            LinkHealth::Quiet
        }
    }

    /// The channel's and the adapter's counts (`stats`, `UsbCanFd::stats`).
    fn counters(&self) -> Vec<(&'static str, u64)> {
        let channel = self.stats();
        let device = self.core.shared.device.snapshot();
        vec![
            ("rx_frames", channel.rx_frames),
            ("rx_lost", channel.rx_lost),
            ("bus_errors", channel.bus_errors),
            ("tx_queued", channel.tx_queued),
            ("tx_queue_full", channel.tx_queue_full),
            ("tx_batches", channel.tx_batches),
            ("tx_partial_batches", channel.tx_partial_batches),
            ("tx_failed_batches", channel.tx_failed_batches),
            ("tx_holds", channel.tx_holds),
            ("tx_stalls", channel.tx_stalls),
            ("tx_dropped", channel.tx_dropped),
            ("recoveries", channel.recoveries),
            ("rx_transfers", device.rx_transfers),
            ("rx_invalid", device.rx_invalid),
            ("rx_unopened", device.rx_unopened),
            ("rx_usb_errors", device.rx_usb_errors),
        ]
    }

    /// Recorded on the control thread into a fixed ring; nothing allocates.
    fn answered(&mut self, round_trip: Duration) {
        self.round_trips.record(round_trip);
    }

    fn round_trip(&mut self) -> Option<RoundTrip> {
        self.round_trips.take()
    }

    fn round_trip_max(&mut self) -> Option<Duration> {
        self.round_trips.take_max()
    }
}

impl Drop for Channel {
    fn drop(&mut self) {
        let mut lifecycle = lock(&self.core.lifecycle);
        let slot = usize::from(self.index);
        let _ = lifecycle
            .commands
            .run(Command::ChannelStop, self.index, &[]);
        self.state().open.store(false, Ordering::Relaxed);
        lifecycle.open[slot] = false;
        if !lifecycle.open.iter().any(|open| *open) {
            let _ = lifecycle.commands.run(Command::DeviceClose, 0, &[]);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An empty bus is told from a wedged adapter by what arrived during the refusals: a run of
    /// status-1 refusals with nothing received is nothing answering, never a power cycle.
    #[test]
    fn refusals_with_nothing_received_are_nothing_answering() {
        let refused = Stall::Refused {
            batches: 3,
            status: 1,
        };
        assert!(nothing_answering(refused, 0, 0));
        assert!(
            nothing_answering(refused, 120, 120),
            "a cable pulled mid-run"
        );
        assert!(
            !nothing_answering(refused, 121, 120),
            "something answered meanwhile"
        );
        assert!(!nothing_answering(Stall::NoReply { frames: 4 }, 0, 0));
        let message = LinkError::NothingAnswering { channel: 1 }.to_string();
        assert_eq!(
            message,
            "Nothing on CAN1 is answering: check the cable to the motors, its termination and motor power"
        );
        assert_eq!(
            io::Error::from(LinkError::NothingAnswering { channel: 1 }).kind(),
            io::ErrorKind::NotConnected
        );
    }
}
