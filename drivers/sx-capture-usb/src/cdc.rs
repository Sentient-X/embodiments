//! ESP32-C6 CDC-ACM link transport: line coding, control lines, one buffered reader.
use std::io;

use crate::usb::descriptor::{Configuration, TransferType};
use crate::usb::{CLASS_INTERFACE_OUT, Control};

pub const CDC_COMMUNICATION: u8 = 0x02;
pub const CDC_DATA: u8 = 0x0a;
pub const SET_LINE_CODING: u8 = 0x20;
pub const SET_CONTROL_LINE_STATE: u8 = 0x22;
pub const BAUD: u32 = 115_200;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AcmInterfaces {
    pub communication: u8,
    pub data: u8,
    pub bulk_in: u8,
    pub bulk_in_size: usize,
}

impl AcmInterfaces {
    pub fn find(configuration: &Configuration) -> Result<Self, String> {
        let communication = configuration
            .alt_settings
            .iter()
            .find(|a| a.class == CDC_COMMUNICATION && a.subclass == 0x02)
            .ok_or("no CDC-ACM communication interface")?
            .number;
        let data = configuration
            .alt_settings
            .iter()
            .find(|a| a.class == CDC_DATA)
            .ok_or("no CDC data interface")?;
        let bulk_in = data
            .endpoints
            .iter()
            .find(|e| e.is_in() && e.transfer_type() == TransferType::Bulk)
            .ok_or("CDC data interface has no bulk IN endpoint")?;
        Ok(Self {
            communication,
            data: data.number,
            bulk_in: bulk_in.address,
            bulk_in_size: usize::from(bulk_in.max_packet_size & 0x07ff).max(64),
        })
    }
}

/// DTR and RTS, always set together.
pub const LINES_UP: u16 = 0x0003;
pub const LINES_DOWN: u16 = 0x0000;
/// How long both lines stay down when a link is re-opened.
pub const REOPEN_PAUSE: std::time::Duration = std::time::Duration::from_millis(100);

/// Both control lines at once: down (`cdc_acm`'s close) or up (its open).
pub fn set_lines(control: &impl Control, interfaces: &AcmInterfaces, lines: u16) -> io::Result<()> {
    control.control_out(
        CLASS_INTERFACE_OUT,
        SET_CONTROL_LINE_STATE,
        lines,
        u16::from(interfaces.communication),
        &[],
    )
}

/// `cdc_acm`'s close then open: 115200 8N1, both lines down, both lines up — a DTR edge
/// whatever state an earlier host left the device in.
pub fn configure(control: &impl Control, interfaces: &AcmInterfaces) -> io::Result<()> {
    let mut coding = BAUD.to_le_bytes().to_vec();
    coding.extend_from_slice(&[0, 0, 8]);
    let index = u16::from(interfaces.communication);
    control.control_out(CLASS_INTERFACE_OUT, SET_LINE_CODING, 0, index, &coding)?;
    set_lines(control, interfaces, LINES_DOWN)?;
    set_lines(control, interfaces, LINES_UP)
}

/// A silent link's one recovery: both lines down, a pause, both up — a fresh `cdc_acm`
/// close and open.
pub fn reopen_lines(control: &impl Control, interfaces: &AcmInterfaces) -> io::Result<()> {
    set_lines(control, interfaces, LINES_DOWN)?;
    std::thread::sleep(REOPEN_PAUSE);
    set_lines(control, interfaces, LINES_UP)
}

/// The bulk IN endpoint of a CDC data interface, read with a timeout.
pub trait BulkIn: Send + Sync + 'static {
    /// Up to `buffer.len()` bytes within `timeout_ms`; `Ok(0)` when nothing came.
    fn read(&self, buffer: &mut [u8], timeout_ms: u32) -> io::Result<usize>;
    /// Clear a halt (stall) on the endpoint so it can be read again.
    fn clear_halt(&self) -> io::Result<()> {
        Ok(())
    }
}

/// Transfer errors after which Linux's `cdc_acm` resubmits its read rather than closing the
/// tty: a stall (`EPIPE`, cleared first) and a bus-level transaction error (`EPROTO`,
/// `EILSEQ`, `ETIME`, `EOVERFLOW`). The laptop pod's encoders never see them; over usbfs
/// they ended the episode (pod5-quest's left encoder: 33 episodes in four days). A device
/// that is gone answers `ENODEV`/`ESHUTDOWN`, which stays fatal.
fn transient(error: &io::Error) -> bool {
    matches!(
        error.raw_os_error(),
        Some(libc::EPIPE | libc::EPROTO | libc::EILSEQ | libc::ETIME | libc::EOVERFLOW)
    )
}

/// Transient errors in a row, with no transfer between them, after which the link fails.
pub const MAX_TRANSIENT_IN_ROW: u32 = 5;
/// The pause before re-reading after a transient error.
const TRANSIENT_PAUSE: std::time::Duration = std::time::Duration::from_millis(10);
/// What a recovered error leaves in the listener's bytes: a NUL on each side of a newline,
/// so the line cut by the error and the line resumed after it both fail to parse as a
/// sample and are dropped, and no two halves of different lines are ever joined into a
/// plausible angle.
pub const DISCONTINUITY: &[u8] = b"\0\n\0";

/// Bytes the link keeps for its listener before the oldest go.
pub const LINK_BUFFER: usize = 1 << 16;
/// How long one read waits for bytes.
const READ_TIMEOUT_MS: u32 = 200;

#[derive(Default)]
struct LinkState {
    buffer: std::collections::VecDeque<u8>,
    /// The open source's generation, or `None` while nobody listens.
    listener: Option<u64>,
    generation: u64,
    failure: Option<(io::ErrorKind, String)>,
    /// Reads completed, reads that timed out empty, and bytes, since the link opened.
    reads: u64,
    empty: u64,
    bytes: u64,
    /// Transient transfer errors read past, and the last one's detail.
    recovered: u64,
    last_recovered: Option<String>,
}

struct LinkShared {
    state: std::sync::Mutex<LinkState>,
    arrived: std::sync::Condvar,
    stop: std::sync::atomic::AtomicBool,
}

impl LinkShared {
    fn lock(&self) -> std::sync::MutexGuard<'_, LinkState> {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

/// One encoder's serial link, held as Linux `cdc_acm` holds a tty: configured once per
/// attach ([`configure`]: line coding, both lines down, both up — the lines only ever move
/// together, since on the ESP32-C6's USB Serial/JTAG unit one alone is the reset or
/// download-mode input), lowered on [`CdcLink::release`], and its bulk IN endpoint read by exactly one reader from
/// the first open until the device goes, so the firmware never sees its host stop polling.
/// An episode's source takes the bytes that arrive while it is open; between episodes they
/// are read and dropped, as `cdc_acm` drops them with no reader.
pub struct CdcLink {
    configured: std::sync::atomic::AtomicBool,
    shared: std::sync::Arc<LinkShared>,
    reader: std::sync::Mutex<Option<std::thread::JoinHandle<()>>>,
}

impl Default for CdcLink {
    fn default() -> Self {
        Self {
            configured: std::sync::atomic::AtomicBool::new(false),
            shared: std::sync::Arc::new(LinkShared {
                state: std::sync::Mutex::new(LinkState::default()),
                arrived: std::sync::Condvar::new(),
                stop: std::sync::atomic::AtomicBool::new(false),
            }),
            reader: std::sync::Mutex::new(None),
        }
    }
}

impl CdcLink {
    /// Open a source on the link: the first open configures the device and starts the one
    /// reader; a later open supersedes an earlier source (it then reads an error).
    pub fn open(
        &self,
        control: &impl Control,
        interfaces: &AcmInterfaces,
        bulk: std::sync::Arc<dyn BulkIn>,
    ) -> io::Result<LinkSource> {
        use std::sync::atomic::Ordering;
        if !self.configured.swap(true, Ordering::SeqCst)
            && let Err(error) = configure(control, interfaces)
        {
            self.configured.store(false, Ordering::SeqCst);
            return Err(error);
        }
        let generation = {
            let mut state = self.shared.lock();
            if let Some((kind, detail)) = state.failure.clone() {
                return Err(io::Error::new(kind, detail));
            }
            state.generation += 1;
            state.listener = Some(state.generation);
            state.buffer.clear();
            state.generation
        };
        self.shared.arrived.notify_all();
        let mut reader = self
            .reader
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if reader.is_none() {
            let shared = std::sync::Arc::clone(&self.shared);
            let size = interfaces.bulk_in_size.max(64) * 8;
            *reader = Some(
                std::thread::Builder::new()
                    .name("sx-rig-cdc".to_owned())
                    .spawn(move || read_link(&shared, bulk.as_ref(), size))?,
            );
        }
        Ok(LinkSource {
            shared: std::sync::Arc::clone(&self.shared),
            generation,
        })
    }
}

impl CdcLink {
    /// `cdc_acm`'s close: both lines down, once, if this link raised them. Called while the
    /// device's fd is still the rig's (detach, rig close); a device already gone is no error.
    pub fn release(&self, control: &impl Control, interfaces: &AcmInterfaces) {
        if self
            .configured
            .swap(false, std::sync::atomic::Ordering::SeqCst)
        {
            let _ = set_lines(control, interfaces, LINES_DOWN);
        }
    }
}

impl Drop for CdcLink {
    fn drop(&mut self) {
        self.shared
            .stop
            .store(true, std::sync::atomic::Ordering::SeqCst);
        self.shared.arrived.notify_all();
        let handle = self
            .reader
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .take();
        if let Some(handle) = handle {
            let _ = handle.join();
        }
    }
}

/// The link's one reader: every byte the endpoint yields, to the listener or dropped.
fn read_link(shared: &LinkShared, bulk: &dyn BulkIn, size: usize) {
    use std::sync::atomic::Ordering;
    let mut buffer = vec![0_u8; size];
    let mut in_row = 0_u32;
    while !shared.stop.load(Ordering::SeqCst) {
        match bulk.read(&mut buffer, READ_TIMEOUT_MS) {
            Ok(0) => {
                in_row = 0;
                let mut state = shared.lock();
                state.reads += 1;
                state.empty += 1;
            }
            Err(error) if transient(&error) && in_row < MAX_TRANSIENT_IN_ROW => {
                in_row += 1;
                let cleared = if error.raw_os_error() == Some(libc::EPIPE) {
                    bulk.clear_halt()
                } else {
                    Ok(())
                };
                let mut state = shared.lock();
                if let Err(clear) = cleared {
                    state.failure = Some((
                        clear.kind(),
                        format!("{error}; clearing the stalled endpoint failed: {clear}"),
                    ));
                    drop(state);
                    shared.arrived.notify_all();
                    return;
                }
                state.recovered += 1;
                state.last_recovered = Some(error.to_string());
                if state.listener.is_some() {
                    state.buffer.extend(DISCONTINUITY);
                }
                drop(state);
                std::thread::sleep(TRANSIENT_PAUSE);
            }
            Ok(count) => {
                in_row = 0;
                let mut state = shared.lock();
                state.reads += 1;
                state.bytes += count as u64;
                if state.listener.is_some() {
                    state.buffer.extend(&buffer[..count]);
                    let excess = state.buffer.len().saturating_sub(LINK_BUFFER);
                    state.buffer.drain(..excess);
                    drop(state);
                    shared.arrived.notify_all();
                }
            }
            Err(error) => {
                shared.lock().failure = Some((error.kind(), error.to_string()));
                shared.arrived.notify_all();
                return;
            }
        }
    }
}

/// An episode's view of a link: the bytes that arrived since it opened.
pub struct LinkSource {
    shared: std::sync::Arc<LinkShared>,
    generation: u64,
}

impl LinkSource {
    /// The link's counters since it opened: what a silent encoder's error names.
    pub fn diagnostics(&self) -> String {
        let state = self.shared.lock();
        format!(
            "link: {} bulk reads, {} empty, {} bytes{}{}",
            state.reads,
            state.empty,
            state.bytes,
            state
                .last_recovered
                .as_ref()
                .map_or_else(String::new, |last| format!(
                    ", {} transfer errors read past (last: {last})",
                    state.recovered
                )),
            state
                .failure
                .as_ref()
                .map_or_else(String::new, |(_, detail)| format!(", failed: {detail}"))
        )
    }

    /// Wait at most ~200 ms; `Ok(0)` when nothing arrived.
    pub fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        let deadline = std::time::Instant::now()
            + std::time::Duration::from_millis(u64::from(READ_TIMEOUT_MS));
        let mut state = self.shared.lock();
        loop {
            if state.listener != Some(self.generation) {
                return Err(io::Error::other("another reader took this encoder's link"));
            }
            if !state.buffer.is_empty() {
                let count = out.len().min(state.buffer.len());
                for (slot, byte) in out.iter_mut().zip(state.buffer.drain(..count)) {
                    *slot = byte;
                }
                return Ok(count);
            }
            if let Some((kind, detail)) = state.failure.clone() {
                return Err(io::Error::new(kind, detail));
            }
            let left = deadline.saturating_duration_since(std::time::Instant::now());
            if left.is_zero() {
                return Ok(0);
            }
            state = self
                .shared
                .arrived
                .wait_timeout(state, left)
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .0;
        }
    }
}

impl Drop for LinkSource {
    fn drop(&mut self) {
        let mut state = self.shared.lock();
        if state.listener == Some(self.generation) {
            state.listener = None;
            state.buffer.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::uvc::controls::tests::Controls;

    /// A scripted bulk IN endpoint that counts how many reads run at once.
    #[derive(Default)]
    struct ScriptedBulk {
        chunks: std::sync::Mutex<std::collections::VecDeque<Vec<u8>>>,
        reading: std::sync::atomic::AtomicUsize,
        most: std::sync::atomic::AtomicUsize,
    }

    impl BulkIn for ScriptedBulk {
        fn read(&self, buffer: &mut [u8], _timeout_ms: u32) -> io::Result<usize> {
            use std::sync::atomic::Ordering;
            let now = self.reading.fetch_add(1, Ordering::SeqCst) + 1;
            self.most.fetch_max(now, Ordering::SeqCst);
            std::thread::sleep(std::time::Duration::from_millis(5));
            let chunk = self.chunks.lock().unwrap().pop_front();
            self.reading.fetch_sub(1, Ordering::SeqCst);
            Ok(chunk.map_or(0, |chunk| {
                buffer[..chunk.len()].copy_from_slice(&chunk);
                chunk.len()
            }))
        }
    }

    fn interfaces() -> AcmInterfaces {
        AcmInterfaces {
            communication: 0,
            data: 1,
            bulk_in: 0x81,
            bulk_in_size: 64,
        }
    }

    fn read_some(source: &mut LinkSource) -> Vec<u8> {
        let mut got = Vec::new();
        let mut buffer = [0_u8; 64];
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while got.is_empty() && std::time::Instant::now() < deadline {
            let count = source.read(&mut buffer).unwrap();
            got.extend_from_slice(&buffer[..count]);
        }
        got
    }

    /// The device is configured once — line coding, then DTR and RTS together — however
    /// many episodes open it; the control lines are never sent again.
    #[test]
    fn a_link_configures_its_device_once_across_episodes() {
        let controls = Controls::default();
        let link = CdcLink::default();
        let bulk = std::sync::Arc::new(ScriptedBulk::default());
        for _ in 0..3 {
            let source = link.open(&controls, &interfaces(), bulk.clone()).unwrap();
            drop(source);
        }
        let log = controls.log.borrow();
        assert_eq!(log.len(), 3, "{log:?}");
        assert_eq!(log[0].1, SET_LINE_CODING);
        assert_eq!((log[1].1, log[1].2), (SET_CONTROL_LINE_STATE, LINES_DOWN));
        assert_eq!((log[2].1, log[2].2), (SET_CONTROL_LINE_STATE, LINES_UP));
    }

    /// An ESP32-C6 as the DTR-edge hypothesis has it: it streams only after it sees DTR go
    /// 0→1, and it may still hold DTR high from a host process that exited without closing.
    struct EdgeEsp {
        dtr: std::sync::atomic::AtomicBool,
        streaming: std::sync::atomic::AtomicBool,
        lines: std::sync::Mutex<Vec<u16>>,
    }

    impl EdgeEsp {
        fn left_high() -> Self {
            Self {
                dtr: true.into(),
                streaming: false.into(),
                lines: std::sync::Mutex::default(),
            }
        }
    }

    impl Control for EdgeEsp {
        fn control_in(&self, _: u8, _: u8, _: u16, _: u16, _: &mut [u8]) -> io::Result<usize> {
            Ok(0)
        }
        fn control_out(&self, _: u8, request: u8, value: u16, _: u16, _: &[u8]) -> io::Result<()> {
            use std::sync::atomic::Ordering;
            if request == SET_CONTROL_LINE_STATE {
                self.lines.lock().unwrap().push(value);
                let dtr = value & 1 == 1;
                if dtr && !self.dtr.swap(dtr, Ordering::SeqCst) {
                    self.streaming.store(true, Ordering::SeqCst);
                }
                self.dtr.store(dtr, Ordering::SeqCst);
                if !dtr {
                    self.streaming.store(false, Ordering::SeqCst);
                }
            }
            Ok(())
        }
    }

    impl BulkIn for EdgeEsp {
        fn read(&self, buffer: &mut [u8], _timeout_ms: u32) -> io::Result<usize> {
            std::thread::sleep(std::time::Duration::from_millis(5));
            if !self.streaming.load(std::sync::atomic::Ordering::SeqCst) {
                return Ok(0);
            }
            let line = b"enc-L,0.1\n";
            buffer[..line.len()].copy_from_slice(line);
            Ok(line.len())
        }
    }

    /// A device left with DTR high (the app force-stopped, the device never told) streams
    /// once the link opens, because the open lowers both lines before raising them; release
    /// lowers them again, and the lines never move one at a time.
    #[test]
    fn an_open_is_a_dtr_edge_even_when_the_last_host_left_dtr_high() {
        let esp = std::sync::Arc::new(EdgeEsp::left_high());
        let link = CdcLink::default();
        let mut source = link.open(esp.as_ref(), &interfaces(), esp.clone()).unwrap();
        assert_eq!(read_some(&mut source), b"enc-L,0.1\n");
        drop(source);
        link.release(esp.as_ref(), &interfaces());
        link.release(esp.as_ref(), &interfaces());
        assert_eq!(
            *esp.lines.lock().unwrap(),
            [LINES_DOWN, LINES_UP, LINES_DOWN]
        );
        // A re-open is the same edge.
        let reopened = EdgeEsp::left_high();
        reopen_lines(&reopened, &interfaces()).unwrap();
        assert!(reopened.streaming.load(std::sync::atomic::Ordering::SeqCst));
        drop(link);
    }

    /// One reader per endpoint: a second source supersedes the first, which then reads an
    /// error, never half the bytes; the endpoint is never read by two threads at once, and
    /// bytes between episodes are drained, not handed to the next episode.
    #[test]
    fn one_reader_holds_the_endpoint_and_a_new_source_supersedes_the_old() {
        use std::sync::atomic::Ordering;
        let controls = Controls::default();
        let link = CdcLink::default();
        let bulk = std::sync::Arc::new(ScriptedBulk::default());
        let mut first = link.open(&controls, &interfaces(), bulk.clone()).unwrap();
        bulk.chunks
            .lock()
            .unwrap()
            .push_back(b"enc-L,0.1\n".to_vec());
        assert_eq!(read_some(&mut first), b"enc-L,0.1\n");
        let mut second = link.open(&controls, &interfaces(), bulk.clone()).unwrap();
        assert!(
            first.read(&mut [0; 8]).is_err(),
            "the superseded source is told"
        );
        bulk.chunks
            .lock()
            .unwrap()
            .push_back(b"enc-L,0.2\n".to_vec());
        assert_eq!(read_some(&mut second), b"enc-L,0.2\n");
        drop(second);
        // Read while nobody listens, then dropped.
        bulk.chunks.lock().unwrap().push_back(b"stale\n".to_vec());
        std::thread::sleep(std::time::Duration::from_millis(100));
        let mut third = link.open(&controls, &interfaces(), bulk.clone()).unwrap();
        bulk.chunks.lock().unwrap().push_back(b"fresh\n".to_vec());
        assert_eq!(read_some(&mut third), b"fresh\n");
        assert_eq!(bulk.most.load(Ordering::SeqCst), 1, "one reader at a time");
        drop(third);
        drop(link);
    }

    /// A bulk IN endpoint that answers from a script of results, then nothing.
    #[derive(Default)]
    struct FlakyBulk {
        script: std::sync::Mutex<std::collections::VecDeque<io::Result<Vec<u8>>>>,
        cleared: std::sync::atomic::AtomicUsize,
    }

    impl FlakyBulk {
        fn new(script: Vec<io::Result<Vec<u8>>>) -> Self {
            Self {
                script: std::sync::Mutex::new(script.into()),
                cleared: 0.into(),
            }
        }
    }

    impl BulkIn for FlakyBulk {
        fn read(&self, buffer: &mut [u8], _timeout_ms: u32) -> io::Result<usize> {
            std::thread::sleep(std::time::Duration::from_millis(1));
            match self.script.lock().unwrap().pop_front() {
                Some(Ok(bytes)) => {
                    buffer[..bytes.len()].copy_from_slice(&bytes);
                    Ok(bytes.len())
                }
                Some(Err(error)) => Err(error),
                None => {
                    std::thread::sleep(std::time::Duration::from_millis(20));
                    Ok(0)
                }
            }
        }

        fn clear_halt(&self) -> io::Result<()> {
            self.cleared
                .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            Ok(())
        }
    }

    fn os(errno: i32) -> io::Result<Vec<u8>> {
        Err(io::Error::from_raw_os_error(errno))
    }

    fn read_until_quiet(source: &mut LinkSource) -> io::Result<Vec<u8>> {
        let mut got = Vec::new();
        let mut buffer = [0_u8; 64];
        let mut quiet = 0;
        while quiet < 3 {
            let count = source.read(&mut buffer)?;
            quiet = if count == 0 { quiet + 1 } else { 0 };
            got.extend_from_slice(&buffer[..count]);
        }
        Ok(got)
    }

    /// A stall and a transaction error mid-stream are read past as `cdc_acm` reads past
    /// them: the stall is cleared, the stream goes on, and the line each error cut is
    /// poisoned so no halves of two lines are joined into an angle.
    #[test]
    fn a_stall_or_transaction_error_mid_stream_is_read_past_and_marks_the_cut() {
        let controls = Controls::default();
        let link = CdcLink::default();
        let bulk = std::sync::Arc::new(FlakyBulk::new(vec![
            Ok(b"0.1\n0.2".to_vec()),
            os(libc::EPIPE),
            Ok(b"5\n0.3\n".to_vec()),
            os(libc::EPROTO),
            Ok(b"0.4\n".to_vec()),
        ]));
        let mut source = link.open(&controls, &interfaces(), bulk.clone()).unwrap();
        let got = read_until_quiet(&mut source).unwrap();
        assert_eq!(got, b"0.1\n0.2\x00\n\x005\n0.3\n\x00\n\x000.4\n");
        assert_eq!(bulk.cleared.load(std::sync::atomic::Ordering::SeqCst), 1);
        let mut lines = sx_yubi::encoder_line::Lines::default();
        let mut angles = Vec::new();
        lines
            .feed(&[b"x\n".as_slice(), &got].concat(), |line| {
                if let Some((_, angle)) = sx_yubi::encoder_line::parse(line)? {
                    angles.push(angle);
                }
                Ok::<(), String>(())
            })
            .unwrap();
        // The line after a cut may be a tail, so it goes too, even when (as the 0.4 here)
        // the cut fell between lines: a lost sample, which the seal's drop count judges.
        assert_eq!(
            angles.len(),
            2,
            "the cut lines are dropped, never joined: {angles:?}"
        );
        assert!((angles[0] - 0.1).abs() < 1e-9 && (angles[1] - 0.3).abs() < 1e-9);
        assert!(
            source.diagnostics().contains("2 transfer errors read past"),
            "{}",
            source.diagnostics()
        );
        drop(source);
        drop(link);
    }

    /// An endpoint that keeps failing is not retried forever: past the bound the link fails
    /// with the device's own error, and a gone device fails it at once.
    #[test]
    fn errors_in_a_row_past_the_bound_or_a_gone_device_fail_the_link() {
        let controls = Controls::default();
        let link = CdcLink::default();
        let script = (0..=MAX_TRANSIENT_IN_ROW)
            .map(|_| os(libc::EPROTO))
            .collect();
        let bulk = std::sync::Arc::new(FlakyBulk::new(script));
        let mut source = link.open(&controls, &interfaces(), bulk).unwrap();
        let error = read_until_quiet(&mut source).unwrap_err();
        assert_eq!(error.raw_os_error(), None);
        assert!(error.to_string().contains("Protocol error"), "{error}");
        drop(source);
        drop(link);

        let link = CdcLink::default();
        let bulk = std::sync::Arc::new(FlakyBulk::new(vec![os(libc::ENODEV)]));
        let mut source = link.open(&controls, &interfaces(), bulk).unwrap();
        let error = read_until_quiet(&mut source).unwrap_err();
        assert!(error.to_string().contains("No such device"), "{error}");
        drop(source);
        drop(link);
    }

    #[test]
    fn line_coding_and_control_lines_match_cdc_acm_open() {
        let controls = Controls::default();
        let interfaces = AcmInterfaces {
            communication: 0,
            data: 1,
            bulk_in: 0x81,
            bulk_in_size: 64,
        };
        configure(&controls, &interfaces).unwrap();
        let log = controls.log.borrow();
        assert_eq!(
            log[0],
            (0x21, 0x20, 0, 0, vec![0x00, 0xc2, 0x01, 0x00, 0, 0, 8])
        );
        assert_eq!(log[1], (0x21, 0x22, 0x0000, 0, vec![]));
        assert_eq!(log[2], (0x21, 0x22, 0x0003, 0, vec![]));
        assert_eq!(log.len(), 3, "the lines never move one alone");
    }
}
