//! The data path: one I/O thread owning the receive and transmit endpoints (§9 of
//! `docs/PROTOCOL.md`).
//!
//! - **Receive.** [`RX_TRANSFERS`] reads of [`RX_TRANSFER_LEN`] bytes stay queued on 0x83; each
//!   is decoded and re-queued the moment it completes. Records go to their channel's receive
//!   ring, allocated once; a full ring counts what it could not hold, and the channel reports
//!   the loss on its next read rather than dropping it unseen.
//! - **Transmit.** Each channel has a queue of [`TX_QUEUE`] frames. Whenever no transmit is
//!   outstanding, the channels are served in turn: whatever one has queued, up to the batch
//!   limit, goes out as one 0xF2 transfer at once — a single frame is never held back to fill a
//!   batch — and nothing else is written until the device's 8-byte reply on 0x82 arrives or its
//!   deadline passes. That one-outstanding rule is the reference's only flow control.
//!   `in_flight = 2` lets a second batch go out while the first reply is pending: a measured
//!   experiment, not the default.
//! - **Failures.** A refused or timed-out batch is not retried (the reference does not). It is
//!   counted, and the channel's next send reports it once.
//!
//! The engine is written against [`Pipe`], one endpoint's queue of transfers, so the flow
//! control runs the same over a fake device in tests as over USB.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard, OnceLock};
use std::task::{Context, Poll};
use std::thread::Thread;
use std::time::{Duration, Instant};

use crate::ring::Ring;
use crate::wire::{self, MAX_BATCH, MAX_TX_TRANSFER, Record, RxFrame, TxFrame, TxReply};

/// Receive reads kept queued on 0x83, as the reference keeps.
pub const RX_TRANSFERS: usize = 16;
/// Each receive read's buffer, as the reference sizes it.
pub const RX_TRANSFER_LEN: usize = 16384;
/// Frames each channel's receive ring holds before it counts losses.
pub const RX_RING: usize = 4096;
/// Frames each channel's transmit queue holds, as the reference's per-channel queue.
pub const TX_QUEUE: usize = 10240;

/// Why a transfer did not complete.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransferFailure {
    Cancelled,
    Disconnected,
    Stall,
    Fault,
    Other,
}

/// One completed transfer: its buffer back, the bytes moved, and how it ended.
pub struct Done<B> {
    pub buf: B,
    pub len: usize,
    pub status: Result<(), TransferFailure>,
}

/// A transfer buffer the engine fills and reads.
pub trait Buf {
    /// The first `len` bytes a completed IN transfer received.
    fn received(&self, len: usize) -> &[u8];
    /// Make this an OUT transfer of exactly `bytes`.
    fn load(&mut self, bytes: &[u8]);
    /// Make this an IN transfer asking for `len` bytes.
    fn request(&mut self, len: usize);
}

/// One endpoint's queue of transfers, completed in order.
pub trait Pipe {
    type Buf: Buf;
    fn submit(&mut self, buf: Self::Buf);
    fn pending(&self) -> usize;
    /// The oldest transfer once it completes; only called while [`Pipe::pending`] is non-zero.
    fn poll_complete(&mut self, cx: &mut Context<'_>) -> Poll<Done<Self::Buf>>;
    fn cancel_all(&mut self);
}

/// Monotonic counts per channel; read with [`Counters::snapshot`].
#[derive(Debug, Default)]
pub struct Counters {
    pub rx_frames: AtomicU64,
    pub rx_lost: AtomicU64,
    pub bus_errors: AtomicU64,
    pub tx_queued: AtomicU64,
    pub tx_batches: AtomicU64,
    pub tx_reported_sent: AtomicU64,
    pub tx_partial_batches: AtomicU64,
    pub tx_failed_batches: AtomicU64,
    pub tx_failed_frames: AtomicU64,
}

/// One channel's counts at one moment.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize)]
pub struct ChannelStats {
    /// Data frames put in the receive ring.
    pub rx_frames: u64,
    /// Data frames the full receive ring could not hold.
    pub rx_lost: u64,
    /// Error records (0xF3) the device pushed for this channel; their content is undocumented.
    pub bus_errors: u64,
    pub tx_queued: u64,
    pub tx_batches: u64,
    /// The sum of the replies' sent counts (their meaning is unverified, §4.4).
    pub tx_reported_sent: u64,
    /// Accepted replies whose sent count was below the batch size.
    pub tx_partial_batches: u64,
    pub tx_failed_batches: u64,
    pub tx_failed_frames: u64,
}

impl Counters {
    #[must_use]
    pub fn snapshot(&self) -> ChannelStats {
        let read = |counter: &AtomicU64| counter.load(Ordering::Relaxed);
        ChannelStats {
            rx_frames: read(&self.rx_frames),
            rx_lost: read(&self.rx_lost),
            bus_errors: read(&self.bus_errors),
            tx_queued: read(&self.tx_queued),
            tx_batches: read(&self.tx_batches),
            tx_reported_sent: read(&self.tx_reported_sent),
            tx_partial_batches: read(&self.tx_partial_batches),
            tx_failed_batches: read(&self.tx_failed_batches),
            tx_failed_frames: read(&self.tx_failed_frames),
        }
    }
}

/// Device-wide receive counts.
#[derive(Debug, Default)]
pub struct DeviceCounters {
    pub rx_transfers: AtomicU64,
    pub rx_invalid: AtomicU64,
    pub rx_foreign_channel: AtomicU64,
    pub rx_malformed: AtomicU64,
    pub rx_unopened: AtomicU64,
    pub rx_usb_errors: AtomicU64,
}

/// The device-wide counts at one moment.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize)]
pub struct DeviceStats {
    pub rx_transfers: u64,
    /// Transfers refused whole (bad sync, type, length or count).
    pub rx_invalid: u64,
    pub rx_foreign_channel: u64,
    pub rx_malformed: u64,
    /// Frames for a channel this process has not opened.
    pub rx_unopened: u64,
    pub rx_usb_errors: u64,
}

impl DeviceCounters {
    #[must_use]
    pub fn snapshot(&self) -> DeviceStats {
        let read = |counter: &AtomicU64| counter.load(Ordering::Relaxed);
        DeviceStats {
            rx_transfers: read(&self.rx_transfers),
            rx_invalid: read(&self.rx_invalid),
            rx_foreign_channel: read(&self.rx_foreign_channel),
            rx_malformed: read(&self.rx_malformed),
            rx_unopened: read(&self.rx_unopened),
            rx_usb_errors: read(&self.rx_usb_errors),
        }
    }
}

/// A transmit batch that did not go out as asked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxFailure {
    /// The device's reply carried a non-zero status (meaning undocumented).
    Refused { status: u8, frames: usize },
    /// No reply came before the deadline.
    Timeout { frames: usize },
    /// The USB transfer itself failed.
    Usb {
        failure: TransferFailure,
        frames: usize,
    },
    /// The reply could not be parsed.
    Reply { frames: usize },
}

/// What a receive ring holds, and what it could not.
pub struct RxSide {
    pub ring: Ring<RxFrame>,
    /// Frames lost since the channel last reported a loss.
    pub unreported_loss: u64,
    /// The latest error record's raw bytes (undocumented layout, §5.4).
    pub last_error: Option<[u8; 8]>,
}

/// One channel's queues and counts, shared between its users and the I/O thread.
pub struct ChannelShared {
    pub rx: Mutex<RxSide>,
    pub tx: Mutex<Ring<TxFrame>>,
    pub open: AtomicBool,
    pub counters: Counters,
    pub tx_failure: Mutex<Option<TxFailure>>,
}

impl ChannelShared {
    fn new(rx_ring: usize, tx_queue: usize) -> Self {
        Self {
            rx: Mutex::new(RxSide {
                ring: Ring::new(rx_ring, RxFrame::EMPTY),
                unreported_loss: 0,
                last_error: None,
            }),
            tx: Mutex::new(Ring::new(tx_queue, EMPTY_TX)),
            open: AtomicBool::new(false),
            counters: Counters::default(),
            tx_failure: Mutex::new(None),
        }
    }

    fn fail(&self, failure: TxFailure, frames: usize) {
        self.counters
            .tx_failed_batches
            .fetch_add(1, Ordering::Relaxed);
        self.counters
            .tx_failed_frames
            .fetch_add(frames as u64, Ordering::Relaxed);
        let mut slot = lock(&self.tx_failure);
        if slot.is_none() {
            *slot = Some(failure);
        }
    }
}

const EMPTY_TX: TxFrame = TxFrame {
    id: 0,
    extended: false,
    fd: false,
    brs: false,
    len: 0,
    data: [0; 64],
};

/// Lock, recovering the data if a holder panicked: every value here stays consistent per
/// operation.
pub fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner)
}

/// What the device's users and its I/O thread share.
pub struct Shared {
    pub channels: Box<[ChannelShared]>,
    pub device: DeviceCounters,
    pub stop: AtomicBool,
    /// Why the device can no longer be used (disconnected, every receive read failed).
    pub fatal: Mutex<Option<String>>,
    io_thread: OnceLock<Thread>,
}

impl Shared {
    #[must_use]
    pub fn new(channels: u8, rx_ring: usize, tx_queue: usize) -> Self {
        Self {
            channels: (0..channels)
                .map(|_| ChannelShared::new(rx_ring, tx_queue))
                .collect(),
            device: DeviceCounters::default(),
            stop: AtomicBool::new(false),
            fatal: Mutex::new(None),
            io_thread: OnceLock::new(),
        }
    }

    /// Record the thread that serves this device, so queued frames can wake it.
    pub fn set_io_thread(&self, thread: Thread) {
        let _ = self.io_thread.set(thread);
    }

    /// Wake the I/O thread: a frame was queued or the device is closing.
    pub fn wake(&self) {
        if let Some(thread) = self.io_thread.get() {
            thread.unpark();
        }
    }

    fn set_fatal(&self, reason: String) {
        let mut slot = lock(&self.fatal);
        if slot.is_none() {
            *slot = Some(reason);
        }
    }
}

/// One transmit batch awaiting its reply.
struct InFlight {
    channel: u8,
    frames: usize,
    deadline: Instant,
}

/// The engine's settings.
#[derive(Clone, Copy, Debug)]
pub struct EngineConfig {
    pub can_ports: u8,
    /// Frames per transmit transfer (§1.3 step 4).
    pub batch_limit: usize,
    /// Bytes each reply read asks for: the 0x82 endpoint's packet size.
    pub reply_request: usize,
    /// Bytes each receive read asks for.
    pub rx_request: usize,
    /// How long a batch may wait for its reply: 1000 ms plus the device transmit timeout.
    pub reply_timeout: Duration,
    /// Transmit transfers outstanding at once: 1 as the reference, 2 as an experiment.
    pub in_flight: usize,
}

/// The buffers an engine is built with, each allocated once: the receive reads, and one
/// transmit and one reply buffer per batch it may have outstanding.
pub struct Buffers<RB, WB, AB> {
    pub rx: Vec<RB>,
    pub out: Vec<WB>,
    pub replies: Vec<AB>,
}

/// What one step did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Step {
    pub progressed: bool,
    /// When the oldest outstanding batch times out, if any is outstanding.
    pub deadline: Option<Instant>,
}

/// The data path over three pipes: receive (0x83), transmit write (0x02), transmit reply (0x82).
pub struct Engine<R: Pipe, W: Pipe, A: Pipe> {
    shared: std::sync::Arc<Shared>,
    config: EngineConfig,
    rx: R,
    writer: W,
    replies: A,
    out_pool: Vec<W::Buf>,
    reply_pool: Vec<A::Buf>,
    in_flight: VecDeque<InFlight>,
    draining: bool,
    next_channel: usize,
    batch: Box<[TxFrame; MAX_BATCH]>,
    encoded: Box<[u8; MAX_TX_TRANSFER]>,
}

impl<R: Pipe, W: Pipe, A: Pipe> Engine<R, W, A> {
    /// An engine with its receive reads queued and one transmit and reply buffer per allowed
    /// outstanding batch.
    pub fn new(
        shared: std::sync::Arc<Shared>,
        config: EngineConfig,
        (mut rx, writer, replies): (R, W, A),
        buffers: Buffers<R::Buf, W::Buf, A::Buf>,
    ) -> Self {
        let Buffers {
            rx: rx_buffers,
            out: out_pool,
            replies: reply_pool,
        } = buffers;
        for mut buf in rx_buffers {
            buf.request(config.rx_request);
            rx.submit(buf);
        }
        let in_flight = config
            .in_flight
            .clamp(1, out_pool.len().min(reply_pool.len()).max(1));
        Self {
            shared,
            config: EngineConfig {
                in_flight,
                batch_limit: config.batch_limit.clamp(1, MAX_BATCH),
                ..config
            },
            rx,
            writer,
            replies,
            out_pool,
            reply_pool,
            in_flight: VecDeque::with_capacity(in_flight),
            draining: false,
            next_channel: 0,
            batch: Box::new([EMPTY_TX; MAX_BATCH]),
            encoded: Box::new([0; MAX_TX_TRANSFER]),
        }
    }

    /// Receive reads currently queued.
    pub fn rx_pending(&self) -> usize {
        self.rx.pending()
    }

    /// Transmit batches awaiting their reply.
    pub fn outstanding(&self) -> usize {
        self.in_flight.len()
    }

    /// Do everything that can be done now without waiting.
    pub fn step(&mut self, cx: &mut Context<'_>, now: Instant) -> Step {
        let mut progressed = self.receive(cx);
        progressed |= self.collect_writes(cx);
        progressed |= self.collect_replies(cx);
        self.expire(now);
        if self.draining && self.writer.pending() == 0 && self.replies.pending() == 0 {
            self.draining = false;
            progressed = true;
        }
        while !self.draining && self.in_flight.len() < self.config.in_flight && self.send(now) {
            progressed = true;
        }
        Step {
            progressed,
            deadline: self.in_flight.front().map(|batch| batch.deadline),
        }
    }

    fn receive(&mut self, cx: &mut Context<'_>) -> bool {
        let mut progressed = false;
        while self.rx.pending() > 0 {
            let Poll::Ready(mut done) = self.rx.poll_complete(cx) else {
                break;
            };
            progressed = true;
            match done.status {
                Ok(()) => self.deliver(done.buf.received(done.len)),
                Err(TransferFailure::Cancelled) => continue,
                Err(TransferFailure::Disconnected) => {
                    self.shared
                        .set_fatal("the device was disconnected".to_owned());
                    continue;
                }
                Err(_) => {
                    self.shared
                        .device
                        .rx_usb_errors
                        .fetch_add(1, Ordering::Relaxed);
                }
            }
            if !self.shared.stop.load(Ordering::Relaxed) {
                done.buf.request(self.config.rx_request);
                self.rx.submit(done.buf);
            }
        }
        if self.rx.pending() == 0 && !self.shared.stop.load(Ordering::Relaxed) {
            self.shared
                .set_fatal("no receive read is queued on the device".to_owned());
        }
        progressed
    }

    /// Hand one received transfer's records to their channels, locking each channel once.
    fn deliver(&self, bytes: &[u8]) {
        let shared = &self.shared;
        shared.device.rx_transfers.fetch_add(1, Ordering::Relaxed);
        let mut guards: [Option<MutexGuard<'_, RxSide>>; 4] = [None, None, None, None];
        let mut unopened = 0_u64;
        let decoded = wire::decode_receive(
            bytes,
            self.config.rx_request,
            self.config.can_ports,
            |record| {
                let channel = match record {
                    Record::Frame(frame) => frame.channel,
                    Record::Error { channel, .. } => channel,
                };
                let index = usize::from(channel);
                let Some(state) = shared.channels.get(index) else {
                    return;
                };
                if !state.open.load(Ordering::Relaxed) {
                    unopened += 1;
                    return;
                }
                let side = guards[index.min(3)].get_or_insert_with(|| lock(&state.rx));
                match record {
                    Record::Frame(frame) => {
                        if side.ring.push(frame).is_ok() {
                            state.counters.rx_frames.fetch_add(1, Ordering::Relaxed);
                        } else {
                            side.unreported_loss += 1;
                            state.counters.rx_lost.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    Record::Error { data, .. } => {
                        side.last_error = Some(data);
                        state.counters.bus_errors.fetch_add(1, Ordering::Relaxed);
                    }
                }
            },
        );
        drop(guards);
        let device = &shared.device;
        device.rx_unopened.fetch_add(unopened, Ordering::Relaxed);
        match decoded {
            Ok(decoded) => {
                device
                    .rx_foreign_channel
                    .fetch_add(decoded.foreign_channel as u64, Ordering::Relaxed);
                device
                    .rx_malformed
                    .fetch_add(decoded.malformed as u64, Ordering::Relaxed);
            }
            Err(_) => {
                device.rx_invalid.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    fn collect_writes(&mut self, cx: &mut Context<'_>) -> bool {
        let mut progressed = false;
        while self.writer.pending() > 0 {
            let Poll::Ready(done) = self.writer.poll_complete(cx) else {
                break;
            };
            progressed = true;
            self.out_pool.push(done.buf);
            if let (Err(failure), false) = (done.status, self.draining) {
                self.abandon(|frames| TxFailure::Usb { failure, frames });
            }
        }
        progressed
    }

    fn collect_replies(&mut self, cx: &mut Context<'_>) -> bool {
        let mut progressed = false;
        while self.replies.pending() > 0 {
            let Poll::Ready(done) = self.replies.poll_complete(cx) else {
                break;
            };
            progressed = true;
            let reply = match done.status {
                Ok(()) => Ok(TxReply::parse(done.buf.received(done.len))),
                Err(failure) => Err(failure),
            };
            self.reply_pool.push(done.buf);
            if self.draining {
                continue;
            }
            let Some(batch) = self.in_flight.pop_front() else {
                continue;
            };
            let Some(channel) = self.shared.channels.get(usize::from(batch.channel)) else {
                continue;
            };
            match reply {
                Ok(Ok(reply)) if reply.status == 0 => {
                    let counters = &channel.counters;
                    counters.tx_batches.fetch_add(1, Ordering::Relaxed);
                    counters
                        .tx_reported_sent
                        .fetch_add(u64::from(reply.sent), Ordering::Relaxed);
                    if usize::from(reply.sent) < batch.frames {
                        counters.tx_partial_batches.fetch_add(1, Ordering::Relaxed);
                    }
                }
                Ok(Ok(reply)) => channel.fail(
                    TxFailure::Refused {
                        status: reply.status,
                        frames: batch.frames,
                    },
                    batch.frames,
                ),
                Ok(Err(_)) => channel.fail(
                    TxFailure::Reply {
                        frames: batch.frames,
                    },
                    batch.frames,
                ),
                Err(failure) => {
                    channel.fail(
                        TxFailure::Usb {
                            failure,
                            frames: batch.frames,
                        },
                        batch.frames,
                    );
                    self.abandon(|frames| TxFailure::Usb { failure, frames });
                }
            }
        }
        progressed
    }

    /// Time out the oldest batch when its deadline has passed.
    fn expire(&mut self, now: Instant) {
        if !self.draining
            && self
                .in_flight
                .front()
                .is_some_and(|batch| now >= batch.deadline)
        {
            self.abandon(|frames| TxFailure::Timeout { frames });
        }
    }

    /// Fail every outstanding batch, cancel both transmit pipes and wait for their transfers
    /// to come back before sending again.
    fn abandon(&mut self, failure: impl Fn(usize) -> TxFailure) {
        for batch in self.in_flight.drain(..) {
            if let Some(channel) = self.shared.channels.get(usize::from(batch.channel)) {
                channel.fail(failure(batch.frames), batch.frames);
            }
        }
        self.writer.cancel_all();
        self.replies.cancel_all();
        self.draining = true;
    }

    /// Send the next channel's queued frames, if any, as one batch.
    fn send(&mut self, now: Instant) -> bool {
        if self.out_pool.is_empty() || self.reply_pool.is_empty() {
            return false;
        }
        let channels = self.shared.channels.len();
        for turn in 0..channels {
            let index = (self.next_channel + turn) % channels;
            let channel = &self.shared.channels[index];
            let count = {
                let mut queue = lock(&channel.tx);
                if queue.is_empty() {
                    continue;
                }
                queue.drain_into(&mut self.batch[..self.config.batch_limit])
            };
            self.next_channel = (index + 1) % channels;
            let channel_index = u8::try_from(index).unwrap_or(0);
            let Ok(len) =
                wire::encode_transmit(channel_index, &self.batch[..count], &mut self.encoded[..])
            else {
                channel.fail(TxFailure::Reply { frames: count }, count);
                return true;
            };
            let (Some(mut out), Some(mut reply)) = (self.out_pool.pop(), self.reply_pool.pop())
            else {
                return false;
            };
            out.load(&self.encoded[..len]);
            reply.request(self.config.reply_request);
            self.writer.submit(out);
            self.replies.submit(reply);
            self.in_flight.push_back(InFlight {
                channel: channel_index,
                frames: count,
                deadline: now + self.config.reply_timeout,
            });
            return true;
        }
        false
    }

    /// Cancel everything and wait, up to `limit`, for the transfers to come back.
    pub fn shutdown(&mut self, limit: Duration) {
        self.shared.stop.store(true, Ordering::Relaxed);
        self.rx.cancel_all();
        self.writer.cancel_all();
        self.replies.cancel_all();
        let waker =
            std::task::Waker::from(std::sync::Arc::new(ThreadWaker(std::thread::current())));
        let mut cx = Context::from_waker(&waker);
        let deadline = Instant::now() + limit;
        while (self.rx.pending() + self.writer.pending() + self.replies.pending()) > 0
            && Instant::now() < deadline
        {
            let mut moved = false;
            for _ in 0..self.rx.pending() {
                if self.rx.poll_complete(&mut cx).is_ready() {
                    moved = true;
                }
            }
            if self.writer.pending() > 0 && self.writer.poll_complete(&mut cx).is_ready() {
                moved = true;
            }
            if self.replies.pending() > 0 && self.replies.poll_complete(&mut cx).is_ready() {
                moved = true;
            }
            if !moved {
                std::thread::park_timeout(Duration::from_millis(10));
            }
        }
    }
}

/// Wakes a parked thread: the I/O thread's waker.
pub struct ThreadWaker(pub Thread);

impl std::task::Wake for ThreadWaker {
    fn wake(self: std::sync::Arc<Self>) {
        self.0.unpark();
    }

    fn wake_by_ref(self: &std::sync::Arc<Self>) {
        self.0.unpark();
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use super::*;
    use crate::wire::{FD_RECORD, SHORT_RECORD, TYPE_CLASSIC, TYPE_ERROR, request_header};

    impl Buf for Vec<u8> {
        fn received(&self, len: usize) -> &[u8] {
            &self[..len.min(self.len())]
        }

        fn load(&mut self, bytes: &[u8]) {
            self.clear();
            self.extend_from_slice(bytes);
        }

        fn request(&mut self, len: usize) {
            self.clear();
            self.resize(len, 0);
        }
    }

    /// A pipe whose transfers complete only when the test completes them.
    #[derive(Default)]
    struct Fake {
        queued: VecDeque<Vec<u8>>,
        done: VecDeque<Done<Vec<u8>>>,
        submitted: Vec<Vec<u8>>,
        cancels: usize,
    }

    impl Fake {
        /// Complete the oldest queued transfer with `bytes` received.
        fn complete(&mut self, bytes: &[u8]) {
            let mut buf = self.queued.pop_front().expect("a queued transfer");
            buf[..bytes.len()].copy_from_slice(bytes);
            self.done.push_back(Done {
                buf,
                len: bytes.len(),
                status: Ok(()),
            });
        }

        fn complete_out(&mut self) {
            let buf = self.queued.pop_front().expect("a queued transfer");
            let len = buf.len();
            self.done.push_back(Done {
                buf,
                len,
                status: Ok(()),
            });
        }
    }

    impl Pipe for Fake {
        type Buf = Vec<u8>;

        fn submit(&mut self, buf: Vec<u8>) {
            self.submitted.push(buf.clone());
            self.queued.push_back(buf);
        }

        fn pending(&self) -> usize {
            self.queued.len() + self.done.len()
        }

        fn poll_complete(&mut self, _cx: &mut Context<'_>) -> Poll<Done<Vec<u8>>> {
            self.done.pop_front().map_or(Poll::Pending, Poll::Ready)
        }

        fn cancel_all(&mut self) {
            self.cancels += 1;
            for buf in self.queued.drain(..) {
                self.done.push_back(Done {
                    buf,
                    len: 0,
                    status: Err(TransferFailure::Cancelled),
                });
            }
        }
    }

    const CONFIG: EngineConfig = EngineConfig {
        can_ports: 2,
        batch_limit: 12,
        reply_request: 512,
        rx_request: RX_TRANSFER_LEN,
        reply_timeout: Duration::from_millis(1020),
        in_flight: 1,
    };

    fn engine(in_flight: usize) -> (Engine<Fake, Fake, Fake>, Arc<Shared>) {
        let shared = Arc::new(Shared::new(2, 64, 1024));
        for channel in &*shared.channels {
            channel.open.store(true, Ordering::Relaxed);
        }
        let engine = Engine::new(
            shared.clone(),
            EngineConfig {
                in_flight,
                ..CONFIG
            },
            (Fake::default(), Fake::default(), Fake::default()),
            Buffers {
                rx: vec![Vec::new(); RX_TRANSFERS],
                out: vec![Vec::new(); 2],
                replies: vec![Vec::new(); 2],
            },
        );
        (engine, shared)
    }

    fn cx() -> Context<'static> {
        Context::from_waker(std::task::Waker::noop())
    }

    fn queue(shared: &Shared, channel: usize, count: usize) {
        let mut queue = lock(&shared.channels[channel].tx);
        for index in 0..count {
            let id = u32::try_from(index).expect("fits");
            queue
                .push(TxFrame::classic(0x7FF, false, &[id.to_le_bytes()[0]; 8]).expect("frame"))
                .expect("room");
        }
    }

    fn reply(channel: u8, status: u8, sent: u8) -> [u8; 8] {
        [0x55, 0xF2, channel, 0, status, sent, 0, 0]
    }

    fn count_of(transfer: &[u8]) -> usize {
        usize::from(u16::from_le_bytes([transfer[8], transfer[9]]))
    }

    #[test]
    fn a_single_queued_frame_goes_out_at_once_and_nothing_follows_until_its_reply() {
        let (mut engine, shared) = engine(1);
        let now = Instant::now();
        assert_eq!(engine.rx_pending(), RX_TRANSFERS);
        queue(&shared, 0, 1);
        assert!(engine.step(&mut cx(), now).progressed);
        assert_eq!(engine.writer.submitted.len(), 1);
        assert_eq!(
            count_of(&engine.writer.submitted[0]),
            1,
            "not held back to fill a batch"
        );
        assert_eq!(engine.replies.submitted.len(), 1);
        // Frames pile up while the reply is pending; nothing more is written.
        queue(&shared, 0, 30);
        engine.writer.complete_out();
        engine.step(&mut cx(), now);
        assert_eq!(engine.writer.submitted.len(), 1, "one outstanding transmit");
        assert_eq!(engine.outstanding(), 1);
        // The reply releases the next batch: the 30 that piled up go as 12, 12, 6.
        engine.replies.complete(&reply(0, 0, 1));
        engine.step(&mut cx(), now);
        assert_eq!(engine.writer.submitted.len(), 2);
        assert_eq!(count_of(&engine.writer.submitted[1]), 12);
        for expected in [12, 6] {
            engine.writer.complete_out();
            engine.replies.complete(&reply(0, 0, 12));
            engine.step(&mut cx(), now);
            assert_eq!(
                count_of(engine.writer.submitted.last().expect("one")),
                expected
            );
        }
        // A reply counting fewer frames than its batch is accepted and counted (§4.4 leaves
        // its meaning open).
        engine.writer.complete_out();
        engine.replies.complete(&reply(0, 0, 2));
        engine.step(&mut cx(), now);
        let stats = shared.channels[0].counters.snapshot();
        assert_eq!(stats.tx_batches, 4);
        assert_eq!(stats.tx_partial_batches, 1);
        assert_eq!(stats.tx_reported_sent, 1 + 12 + 12 + 2);
        assert_eq!(engine.outstanding(), 0);
    }

    #[test]
    fn channels_are_served_in_turn_one_channel_per_transfer() {
        let (mut engine, shared) = engine(1);
        let now = Instant::now();
        queue(&shared, 0, 20);
        queue(&shared, 1, 3);
        engine.step(&mut cx(), now);
        let first = engine.writer.submitted[0].clone();
        assert_eq!((first[2], count_of(&first)), (0, 12));
        engine.writer.complete_out();
        engine.replies.complete(&reply(0, 0, 12));
        engine.step(&mut cx(), now);
        let second = engine.writer.submitted[1].clone();
        assert_eq!(
            (second[2], count_of(&second)),
            (1, 3),
            "channel 1 is not starved"
        );
        assert!(
            second[12..]
                .chunks(FD_RECORD)
                .take(3)
                .all(|record| record[14] == 1)
        );
    }

    #[test]
    fn a_refused_or_late_batch_is_counted_not_retried_and_reported_once() {
        let (mut engine, shared) = engine(1);
        let now = Instant::now();
        queue(&shared, 0, 2);
        engine.step(&mut cx(), now);
        engine.writer.complete_out();
        engine.replies.complete(&reply(0, 7, 0));
        engine.step(&mut cx(), now);
        assert_eq!(
            *lock(&shared.channels[0].tx_failure),
            Some(TxFailure::Refused {
                status: 7,
                frames: 2
            })
        );
        assert_eq!(engine.writer.submitted.len(), 1, "not retried");
        // A batch whose reply never comes times out, both pipes are cancelled, and sending
        // resumes once their transfers are back.
        *lock(&shared.channels[0].tx_failure) = None;
        queue(&shared, 0, 1);
        engine.step(&mut cx(), now);
        assert_eq!(engine.writer.submitted.len(), 2);
        engine.writer.complete_out();
        let late = now + CONFIG.reply_timeout;
        queue(&shared, 0, 1);
        engine.step(&mut cx(), late);
        assert_eq!(
            *lock(&shared.channels[0].tx_failure),
            Some(TxFailure::Timeout { frames: 1 })
        );
        assert_eq!(engine.replies.cancels, 1);
        engine.step(&mut cx(), late);
        assert_eq!(
            engine.writer.submitted.len(),
            3,
            "sending resumes after the drain"
        );
        let stats = shared.channels[0].counters.snapshot();
        assert_eq!((stats.tx_failed_batches, stats.tx_failed_frames), (2, 3));
    }

    #[test]
    fn two_outstanding_transmits_are_an_opt_in_experiment() {
        let (mut engine, shared) = engine(2);
        let now = Instant::now();
        queue(&shared, 0, 1);
        engine.step(&mut cx(), now);
        queue(&shared, 0, 1);
        engine.step(&mut cx(), now);
        assert_eq!(engine.outstanding(), 2);
        queue(&shared, 0, 1);
        engine.step(&mut cx(), now);
        assert_eq!(engine.writer.submitted.len(), 2, "never a third");
        engine.writer.complete_out();
        engine.replies.complete(&reply(0, 0, 1));
        engine.step(&mut cx(), now);
        assert_eq!(engine.writer.submitted.len(), 3);
    }

    fn classic(channel: u8, id: u32) -> Vec<u8> {
        let mut record = vec![0_u8; SHORT_RECORD];
        record[4..8].copy_from_slice(&id.to_le_bytes());
        record[14] = channel;
        record[15] = 8;
        record
    }

    fn transfer(kind: u8, records: &[Vec<u8>]) -> Vec<u8> {
        let size = records.first().map_or(0, Vec::len);
        let mut bytes = request_header(
            kind,
            0,
            u32::try_from(4 + records.len() * size).expect("fits"),
        )
        .to_vec();
        bytes.extend(u16::try_from(records.len()).expect("fits").to_le_bytes());
        bytes.extend([0, 0]);
        for record in records {
            bytes.extend(record);
        }
        bytes.extend([0, 0]);
        bytes
    }

    #[test]
    fn receive_reads_stay_queued_and_a_full_ring_counts_its_losses() {
        let (mut engine, shared) = engine(1);
        let now = Instant::now();
        // 682 classic records fill one 16384-byte read; three of them overflow a 64-frame ring.
        let records: Vec<Vec<u8>> = (0..682).map(|id| classic(1, id)).collect();
        let full = transfer(TYPE_CLASSIC, &records);
        assert!(full.len() <= RX_TRANSFER_LEN);
        for _ in 0..3 {
            engine.rx.complete(&full);
        }
        engine.rx.complete(&transfer(TYPE_ERROR, &[classic(1, 0)]));
        engine.rx.complete(&[0x56; 20]);
        engine.step(&mut cx(), now);
        assert_eq!(
            engine.rx_pending(),
            RX_TRANSFERS,
            "every read re-queued at once"
        );
        assert_eq!(engine.rx.submitted.len(), RX_TRANSFERS + 5);
        let stats = shared.channels[1].counters.snapshot();
        assert_eq!(stats.rx_frames, 64);
        assert_eq!(stats.rx_lost, 3 * 682 - 64);
        assert_eq!(stats.bus_errors, 1);
        assert_eq!(lock(&shared.channels[1].rx).unreported_loss, 3 * 682 - 64);
        let device = shared.device.snapshot();
        assert_eq!((device.rx_transfers, device.rx_invalid), (5, 1));
        // Frames for a channel nobody opened are counted, not queued.
        shared.channels[0].open.store(false, Ordering::Relaxed);
        engine
            .rx
            .complete(&transfer(TYPE_CLASSIC, &[classic(0, 5)]));
        engine.step(&mut cx(), now);
        assert_eq!(shared.device.snapshot().rx_unopened, 1);
        assert!(lock(&shared.channels[0].rx).ring.is_empty());
    }

    #[test]
    fn a_disconnect_is_fatal_and_shutdown_drains_every_transfer() {
        let (mut engine, shared) = engine(1);
        let buf = engine.rx.queued.pop_front().expect("queued");
        engine.rx.done.push_back(Done {
            buf,
            len: 0,
            status: Err(TransferFailure::Disconnected),
        });
        engine.step(&mut cx(), Instant::now());
        assert!(lock(&shared.fatal).is_some());
        assert_eq!(engine.rx_pending(), RX_TRANSFERS - 1);
        engine.shutdown(Duration::from_millis(100));
        assert_eq!(engine.rx_pending(), 0);
    }
}
