# sx-zlg-usbcanfd

ZLG USBCANFD-200U / 400U CAN-FD adapters (USB `04CC:1240`, `3068:0009`) driven in userspace over
USB through [`nusb`](https://github.com/kevinmehall/nusb) (IOKit on macOS, usbfs on Linux), each
channel a `sx_embodiment_drivers::can::CanPort`. Apache-2.0.

**Clean-room implementation.** This crate was written only from the functional specification in
[`docs/PROTOCOL.md`](docs/PROTOCOL.md), committed here unchanged. Its author never read ZLG's
driver source, headers or libraries. The specification describes the protocol as observed from
the behaviour of ZLG's reference Linux driver (version 2.10) and contains none of its code; what
it marks **[unknown]** stays unknown here and is reported, not guessed (below).

## The surface

| Item | Path |
|---|---|
| Find adapters | `list()`: USB serial, VID/PID and speed of each attached adapter, without opening it |
| Open an adapter | `UsbCanFd::open(Some(serial))` / `open_with(serial, Options { transmit_timeout_ms, in_flight })`: claims interface 0 and probes as §1.3 (protocol version, max transfer, microsecond timestamps, device information, 80 or 60 MHz clock). Opening the same serial again in one process returns the same adapter. `probe()` reports what was found, including each endpoint's wMaxPacketSize and the USB speed read from the descriptors |
| Open a channel | `UsbCanFd::open_channel(index, ChannelOptions { bitrate, termination })`: device open (first channel), initialise, transmit timeout, termination (default on), start (§3.8). `Bitrate::CLASSIC_1M` uses the specification's worked timing; other classic and FD rates are searched within the device's ranges. Dropping the `Channel` stops it, and closes the device after the last channel |
| A channel as a port | `Channel` implements `CanPort`: `send` queues a classic frame and returns at once (an id above 11 bits goes extended); `receive` is non-blocking; `send_frame` / `receive_frame` carry CAN FD frames |
| Counts | `Channel::stats()` (received, lost to a full ring, error records, batches, reported sent counts, partial and failed batches) and `UsbCanFd::stats()` (transfers, invalid transfers, frames for unopened channels) |
| Commissioning | `sx-embodiment-bodies`, feature `zlg`: `system::SystemPorts` resolves an identity `<USB serial>:<channel>` (e.g. `A30F115DD0120A949B80:1`), so two channels under one serial are never ambiguous |
| Benchmark | `sx-can-bench` (below) |

## Performance: what matches the reference (§9)

- **Receive.** Sixteen 16384-byte bulk reads stay queued on 0x83 from probe until close, each
  decoded and re-queued the moment it completes, on one I/O thread that owns the data endpoints.
  Records go to a per-channel ring of 4096 frames allocated once; nothing allocates per frame.
  A full ring counts its losses and the channel's next `receive` reports them.
- **Transmit.** Each channel queues up to 10240 frames (the reference's queue). Whenever no
  transmit is outstanding the I/O thread takes whatever a channel has queued, up to the batch
  limit (`floor(max_transfer / 1024) × 12`, at most 96), and writes it as one transfer at once:
  a single frame is never held back. Nothing more is written until the 8-byte reply on 0x82 or
  its deadline (1000 ms + device transmit timeout). Channels are served in turn. A queued frame
  wakes the parked I/O thread directly.
- **Commands** use 0x01 / 0x81 only, synchronously from the caller, with the reference's 500 ms
  per transfer and three attempts on a USB failure; there is no polling after a channel starts.
- **Experiment, off by default.** `Options { in_flight: 2 }` lets a second transmit go out
  while the first reply is pending. The reference never does this and whether the device
  accepts it is unknown; it stays opt-in until hardware shows it safe and faster.

`frame_allowance()` is twice (device transmit timeout + 1 ms): a frame can wait behind one
outstanding batch and then go in its own, each bounded by the device's transmit timeout if the
device honours it per batch, as the specification presumes (42 ms at the default 20 ms).

## Departures and open questions

- Exclusive access is the operating system's interface claim. On Linux, ZLG's kernel module
  claims the adapter when loaded and exposes its channels as SocketCAN interfaces; then this
  crate's open fails with `ZlgError::Busy`, and the SocketCAN path (`sx-socketcan`) is the one to
  use, or the module must be unloaded.
- The reset control request (0xFF) is never sent: a failed probe is reported and the adapter left
  as found.
- CRCs on what the device sends are not checked, as the reference does not; their presence is
  unverified (§2.3).
- Error records (0xF3) are counted per channel and their raw eight bytes kept
  (`last_error_record`), since their layout, bus-off and error-counter reporting are unknown
  (§5.4); no bus state is inferred, and the receive path does not fail on them.
- A transmit reply's sent count below its batch size is accepted and counted
  (`tx_partial_batches`), since its meaning is unknown (§4.4); a non-zero status or a missing
  reply fails the batch, which is counted, not retried, and reported by the channel's next send.
- CAN FD frames longer than eight bytes and remote frames are read past by the classic
  `CanPort::receive`.

## sx-can-bench

A round-trip benchmark generic over `CanPort`, so the same code measures this crate and, on
Linux, SocketCAN over ZLG's kernel module (`--features socketcan`). It sends only Damiao
register reads of `PMAX`, `VMAX` and `TMAX` (`0x7FF`, `[id_lo, id_hi, 0x33, rid, 0, 0, 0, 0]`)
through a guard that refuses every other frame; it refuses a debug build and a 1-minute load
average of 20 or more.

```bash
cargo build --release -p sx-zlg-usbcanfd --bin sx-can-bench
target/release/sx-can-bench zlg --serial A30F115DD0120A949B80 --channel 1 --json zlg.json
target/release/sx-can-bench socketcan can1 --json socketcan.json   # Linux, feature socketcan
```

It reports single round-trip latency (p50/p90/p99/max over 2000 reads), all-motor cycles at 50,
100, 250 and 500 Hz for 10 s each (cycle p50/p99/max, missed deadlines, lost replies),
saturated replies and frames per second, and process CPU seconds per second for each phase,
which includes the benchmark's own polling. The default motors are the static upper body's:
send ids 0x09, 0x10–0x15, 0x18–0x1F, answering on send id + 0x20.

```bash
cargo test && cargo clippy --all-targets -- -D warnings && cargo fmt --check
```

## macOS

macOS leaves this vendor-class adapter unconfigured when it is plugged in: no macOS driver
matches it, so it has no interface to claim. The legacy `IOUSBDeviceInterface::SetConfiguration`
(what `nusb` and libusb call) fails on it with `kIOReturnNotResponding` (`0xe00002ed`), and after
one failed request the adapter's control pipe stays broken until it is replugged. Apple's
IOUSBHost framework configures it: `IOUSBHostDevice` opened with no options, then
`configureWithValue: 1 matchInterfaces: YES`, which works as a normal user (no root) on a freshly
plugged adapter (USBCANFD-200U, serial A30F115DD0120A949B80, firmware 0x0245, 2026-10-25).

So on macOS `UsbCanFd::open` first looks in the I/O Registry, which sends nothing to the
device, for an `IOUSBHostInterface` below the adapter. If there is none it configures the
adapter through IOUSBHost (`src/macos.rs`, the crate's only unsafe code, on
`objc2-io-usb-host`), releases it, waits for interface 0 to register, and only then opens it
with `nusb`. `nusb`'s `set_configuration` is never called on macOS and the adapter is never
reset (a port reset can leave it off the bus until it is power-cycled). The adapter stays
configured until it is unplugged; `Probe::configured_on_open` says whether this open was the
one that configured it. On Linux the kernel configures the device at enumeration and the
driver opens it over usbfs.
