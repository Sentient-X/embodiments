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
| Open a channel | `UsbCanFd::open_channel(index, ChannelOptions { bitrate, termination, receive_wait })`: device open (first channel), initialise, transmit timeout, termination (default on), start (§3.8). `Bitrate::CLASSIC_1M` uses the specification's worked timing; other classic and FD rates are searched within the device's ranges. Dropping the `Channel` stops it, and closes the device after the last channel |
| A channel as a port | `Channel` implements `CanPort`: `send` queues a classic frame and returns at once (an id above 11 bits goes extended); `receive` is non-blocking; `receive_until(deadline)` sleeps until a frame arrives (`ReceiveWait::Block`, the default) or polls (`ReceiveWait::Spin`, opt-in); `send_frame` / `receive_frame` / `receive_frame_until` carry CAN FD frames |
| Transmit budget and refusals | `Channel::budget()` (below); every send refusal is an `io::Error` carrying a `LinkError` (`LinkError::of(&error)`): `QueueFull` (`WouldBlock`), `BatchFailed` (`Other`, once per failed batch), `Recovered` (`Interrupted`: the channel stalled and was recovered; send again), `Recovering` (`WouldBlock`), `PowerCycle` (`NotConnected`: unplug the adapter and plug it back in) |
| Counts | `Channel::stats()` (received, lost to a full ring, error records, queued, refused as queue-full, batches, reported sent counts, partial and failed batches, holds, stalls, dropped, recoveries) and `UsbCanFd::stats()` (transfers, invalid transfers, frames for unopened channels) |
| Commissioning | `sx-embodiment-bodies`, feature `zlg`: `system::SystemPorts` resolves an identity `<USB serial>:<channel>` (e.g. `A30F115DD0120A949B80:1`), so two channels under one serial are never ambiguous |
| Benchmark | `sx-can-bench` (below) |

## Performance: what matches the reference (§9)

- **Receive.** Sixteen 16384-byte bulk reads stay queued on 0x83 from probe until close, each
  decoded and re-queued the moment it completes, on one I/O thread that owns the data endpoints.
  Records go to a per-channel ring of 4096 frames allocated once; nothing allocates per frame.
  A full ring counts its losses and the channel's next `receive` reports them.
- **Transmit.** Whenever no transmit is outstanding the I/O thread takes whatever a channel has
  queued, up to its batch budget, and writes it as one transfer at once: a single frame is never
  held back. Nothing more is written until the 8-byte reply on 0x82 or its deadline (1000 ms +
  device transmit timeout). Channels are served in turn. A queued frame wakes the parked I/O
  thread directly; with nothing to do it sleeps on USB completions (nusb wakers), never polls.
- **Commands** use 0x01 / 0x81 only, synchronously from the caller, with the reference's 500 ms
  per transfer and three attempts on a USB failure; there is no polling after a channel starts.
- **Experiment, off by default.** `Options { in_flight: 2 }` lets a second transmit go out
  while the first reply is pending. The reference never does this and whether the device
  accepts it is unknown; it stays opt-in until hardware shows it safe and faster.

### Transmit budget (a departure: the reference has none)

The reference queues 10240 frames per channel and writes up to 96 per batch whatever the bus
can carry. On the bench 14-motor cycles at 100–500 Hz did exactly that: the queue filled, the
adapter refused batches with status 1, then stopped answering transmits until it was
power-cycled. So every frame is costed at its worst-case bus time (`timing::frame_time`: an
8-byte standard classic frame is 135 bits with full stuffing, 135 µs at 1 Mbit/s; FD frames
count their data phase at the data rate), and per channel (`Channel::budget()`, a `TxBudget`):

- a **batch** holds at most half the device transmit timeout of bus time, so the device can
  put it on the bus within its timeout with the other half left for the answers that share
  the bus: 10 ms, 74 classic frames, at 1 Mbit/s and the default 20 ms;
- the **queue** holds one more such batch behind the outstanding one (10 ms, 74 frames), which
  keeps `frame_allowance()` honest; a frame beyond it is refused at once with
  `LinkError::QueueFull` (`WouldBlock`) instead of growing a queue the bus cannot drain. A
  longer transmit timeout (`Options::transmit_timeout_ms`) buys a longer queue.

A **refused batch** (non-zero status: the bus did not take it within the timeout) is counted,
reported once, and holds its channel until a frame arrives on it (the bus is moving) or one
transmit timeout passes. Three refusals in a row, a reply that never comes, or a failed
transfer **stall** the channel: it sends nothing and its queue is dropped. Its next send
**recovers** it without a replug and returns `LinkError::Recovered` once: first a channel
restart (stop, initialise, transmit timeout, termination, start: §3.8 has no other reset); if
it stalls again before any batch is accepted, a device close and open (only when no other
channel is open); after that, or if the adapter does not answer those commands, every send
returns `LinkError::PowerCycle`, which says the adapter must be unplugged and plugged back in.
`sx-can-bench --exercise-recovery` checks both recoveries on the adapter.

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
100, 200 and 250 Hz for 10 s each (cycle p50/p99/max from first request to last reply, missed
deadlines, skipped cycles, lost replies), saturated replies and frames per second, the link's
refusals per phase (queue-full, failed batches, recoveries), and process CPU seconds per second
for each phase. The default motors are the static upper body's that answer: send ids 0x09,
0x10–0x15, 0x18–0x1E, answering on send id + 0x20; the torso, 0x1F, is known faulty and left out.

Each cycle's bus time is `2 × motors` worst-case frames (3780 µs for 14 motors at 1 Mbit/s,
printed per rate). A rate whose cycle needs more bus time than its period (500 Hz for 14
motors) is refused before anything is sent, and a cycle that has less of its period left than
its bus time when it comes due is skipped and counted missed, never queued late. Cycles are
paced by sleeping to 1 ms before they are due and spinning the rest, because macOS oversleeps
by about 0.75 ms; replies are awaited with `receive_until`, which sleeps (`--spin` to poll).
A link that needs a power cycle stops the run with that message.

### Measured, 2026-10-25

USBCANFD-200U A30F115DD0120A949B80 (firmware 0x0245) on the bench Mac, channel 1 at 1 Mbit/s,
the 14 answering motors, read-only register reads, `--release`, 1-minute load 7–14. "Before"
is the previous driver (unbounded 10240-frame queue, the bench polling); it was run at 50 Hz
only, since at 100–500 Hz it flooded the adapter until it needed a power cycle.

| | before | after, blocking | after, `--spin` | after, blocking + `--realtime` |
|---|---|---|---|---|
| round trip p50 / p99 | 507 / 589 µs | 524 / 2646 µs | 544 / 698 µs | 571 / 719 µs |
| round trip CPU | 1.09 s/s | 0.14 s/s | 1.12 s/s | 0.14 s/s |
| 50 Hz cycle p50 / p99, missed | 3804 / 12929 µs, 0 | 3820 / 13426 µs, 1 | 3807 / 7129 µs, 0 | 3812 / 5929 µs, 0 |
| 100 Hz p50 / p99, missed / lost | (flooded at 100 Hz) | 3773 / 5724 µs, 17 / 150 | 3790 / 4396 µs, 8 / 87 | 3780 / 5780 µs, 4 / 24 |
| 200 Hz p50 / p99, missed / lost | — | 3751 / 4361 µs, 57 / 221 | 3774 / 3954 µs, 43 / 409 | 3767 / 4026 µs, 55 / 485 |
| 250 Hz p50 / p99, missed / lost | — | 3744 / 3891 µs, 149 / 490 | 3760 / 3842 µs, 57 / 401 | 3767 / 3855 µs, 121 / 1199 |
| cycle CPU 50 / 100 / 200 / 250 Hz | 1.02 at 50 Hz | 0.05 / 0.08 / 0.30 / 0.21 | 0.23 / 0.45 / 1.05 / 1.11 | 0.05 / 0.09 / 0.15 / 0.19 |
| throughput replies/s, CPU | 3693, 1.09 s/s | 3745, 0.16 s/s | 3772, 1.12 s/s | 3764, 0.17 s/s |
| queue-full, failed batches, recoveries | — | 0, 0, 0 at every rate | 0, 0, 0 | 0, 0, 0 |

A cycle of 14 motors takes 3.75–3.8 ms at p50 against a 3780 µs worst-case bus budget: the
bus, not the host, sets the cycle, so 250 Hz (4 ms) has almost no slack and misses whenever
anything is late. The replies lost at 100–250 Hz never arrived before their cycle's deadline,
with no refusal from the driver or adapter. The adapter stayed answering throughout and
needed no replug. The I/O thread used about 6 % of a core in every variant (per-thread `ps -M`).
`--exercise-recovery`: channel restart 97 ms, device close and open 153 ms, then a read
answered in 1.1 ms. Opening a freshly plugged adapter configured it through IOUSBHost
(`configured_on_open: true`, normal user), and a 200-read check then gave p50 543 µs, p99
713 µs.

### Waiting on macOS

macOS coalesces a default thread's timers: `thread::sleep(3 ms)` and a 3 ms condvar timeout
wake 0.76–0.77 ms late at p50 (p99 3.2–5.8 ms), and a thread blocked on a condvar is woken
late enough to put 2.6 ms on the round trip's p99. Under Mach's
`THREAD_TIME_CONSTRAINT_POLICY` (`macos::time_constraint`) the same sleeps wake 9 µs late at
p50 (p99 44 µs), and the blocking receive has the spinning one's tail at an eighth of the CPU
(table). So a blocking receive is the default, and the robot's real-time layer puts its loop
threads under the time-constraint policy (the design doc covers it); `--realtime` does the same
for the bench thread, and `ReceiveWait::Spin` stays an opt-in for a thread that cannot have it.

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
