//! From completed USB transfers to timestamped source frames.
//!
//! [`Decoder`] is pure: it takes the packets of each completed isochronous URB (or each bulk
//! payload) with the host time it was reaped at, reassembles frames, feeds every SCR to the
//! [`Clock`](super::clock::Clock) and stamps each frame's PTS as start of exposure on
//! CLOCK_MONOTONIC. [`Engine`] (Linux/Android) owns the usbfs side: claim, alternate setting,
//! a ring of in-flight URBs, `poll` + reap, resubmit, and a clean stop.

use serde::Serialize;

use super::clock::{Clock, Quality};
use super::payload::{self, Assembler, Dropped, Event};

use crate::source::SourceFrame;

/// The streaming bandwidth an engine selected: the isochronous alternate setting, its bytes
/// per service interval, and the interval.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub struct Bandwidth {
    pub alternate: u8,
    pub bytes_per_interval: usize,
    pub service_interval_ns: i64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub struct StreamStats {
    pub transfers: u64,
    pub payloads: u64,
    pub packet_errors: u64,
    pub header_errors: u64,
    pub frames: u64,
    pub dropped_device_error: u64,
    /// JPEGs missing a marker mid-stream: a payload lost on the bus.
    pub dropped_truncated: u64,
    /// The stream's first frame cut short because streaming began part-way through it
    /// (every stream-on joins a frame already leaving the camera): not a loss.
    pub truncated_at_start: u64,
    pub dropped_oversized: u64,
    /// Frames before the clock locked, or without PTS: never delivered.
    pub untimed: u64,
    pub payloads_without_scr: u64,
}

pub struct Decoder {
    assembler: Assembler,
    clock: Clock,
    service_interval_ns: i64,
    poison_next: bool,
    pub stats: StreamStats,
}

impl Decoder {
    pub fn new(clock_frequency_hz: u32, max_frame_size: usize, service_interval_ns: i64) -> Self {
        Self {
            assembler: Assembler::new(max_frame_size, true),
            clock: Clock::new(clock_frequency_hz),
            service_interval_ns,
            poison_next: false,
            stats: StreamStats::default(),
        }
    }

    pub fn clock_quality(&self) -> Quality {
        self.clock.quality()
    }

    /// The packets of one completed isochronous URB, in order, reaped at `reap_ns`.
    pub fn on_iso<'p>(
        &mut self,
        packets: impl ExactSizeIterator<Item = (i32, &'p [u8])>,
        reap_ns: i64,
        out: &mut Vec<SourceFrame>,
    ) {
        self.stats.transfers += 1;
        let count = packets.len() as i64;
        for (index, (status, bytes)) in packets.enumerate() {
            // Packet k sat (count-1-k) service intervals before the URB's last one.
            let host_ns = reap_ns - (count - 1 - index as i64) * self.service_interval_ns;
            if status != 0 {
                self.stats.packet_errors += 1;
                self.poison_next = true;
                continue;
            }
            self.on_payload(bytes, host_ns, out);
        }
    }

    fn on_payload(&mut self, packet: &[u8], host_ns: i64, out: &mut Vec<SourceFrame>) {
        let (header, data) = match payload::parse(packet) {
            Ok(Some(parsed)) => parsed,
            Ok(None) => return,
            Err(_) => {
                self.stats.header_errors += 1;
                self.poison_next = true;
                return;
            }
        };
        self.stats.payloads += 1;
        match header.scr {
            Some((stc, sof)) => self.clock.observe(stc, sof, host_ns),
            None => self.stats.payloads_without_scr += 1,
        }
        let mut header = header;
        if std::mem::take(&mut self.poison_next) {
            // A packet before this one was lost: the frame it belonged to is not evidence.
            header.error = true;
        }
        let mut events = Vec::new();
        self.assembler.push(&header, data, host_ns, &mut events);
        for event in events {
            match event {
                Event::Frame(frame) => {
                    let soe = frame
                        .pts
                        .and_then(|pts| Some((pts, self.clock.soe_ns(pts)?)));
                    match soe {
                        Some((pts, soe_ns)) => {
                            self.stats.frames += 1;
                            out.push(SourceFrame {
                                jpeg: frame.bytes,
                                soe_ns,
                                sequence: frame.sequence,
                                pts,
                            });
                        }
                        None => self.stats.untimed += 1,
                    }
                }
                Event::Dropped { why, sequence } => match why {
                    Dropped::DeviceError => self.stats.dropped_device_error += 1,
                    Dropped::Truncated if sequence == 0 => self.stats.truncated_at_start += 1,
                    Dropped::Truncated => self.stats.dropped_truncated += 1,
                    Dropped::Oversized => self.stats.dropped_oversized += 1,
                },
            }
        }
    }
}

#[cfg(any(target_os = "linux", target_os = "android"))]
pub use engine::Engine;

#[cfg(any(target_os = "linux", target_os = "android"))]
mod engine {
    use super::*;
    use crate::usb::usbfs::{Device, Pool, Transfer};
    use crate::uvc::descriptors::UvcCamera;
    use crate::uvc::probe::Probe;
    use std::io;

    /// In-flight URBs and packets per URB: ~32 ms of high-speed microframes in flight, as
    /// Linux uvcvideo keeps (UVC_URBS × UVC_MAX_PACKETS).
    pub const URBS: usize = 8;
    pub const PACKETS: usize = 32;

    pub struct Engine {
        device: Device,
        clock: crate::source::MonotonicClock,
        camera: UvcCamera,
        pool: Pool,
        pub decoder: Decoder,
        pub bandwidth: Bandwidth,
    }

    impl Engine {
        /// Select the bandwidth `probe` committed and start streaming.
        pub fn start(
            device: Device,
            camera: &UvcCamera,
            probe: &Probe,
            high_speed: bool,
            clock: crate::source::MonotonicClock,
        ) -> io::Result<Self> {
            let frequency = if camera.clock_frequency_hz != 0 {
                camera.clock_frequency_hz
            } else {
                probe.clock_frequency
            };
            if frequency == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::Unsupported,
                    "camera declares no device clock frequency; PTS cannot be mapped",
                ));
            }
            let max_frame = probe.max_video_frame_size as usize;
            let Some(alt) = camera.alt_for(probe.max_payload_transfer_size) else {
                return Err(io::Error::new(
                    io::ErrorKind::Unsupported,
                    format!(
                        "no isochronous bandwidth carries {} bytes per interval",
                        probe.max_payload_transfer_size
                    ),
                ));
            };
            device.set_interface(camera.streaming_interface, alt.alternate)?;
            let size = alt.endpoint.bytes_per_interval() as usize;
            // bInterval is 2^(n-1) frames (full speed: 1 ms) or microframes (high speed).
            let unit = if high_speed { 125_000 } else { 1_000_000 };
            let service_interval_ns = unit << alt.endpoint.interval.saturating_sub(1).min(15);
            let bandwidth = Bandwidth {
                alternate: alt.alternate,
                bytes_per_interval: size,
                service_interval_ns,
            };
            let transfers = (0..URBS)
                .map(|i| Transfer::isochronous(alt.endpoint.address, PACKETS, size, i))
                .collect();
            let pool = Pool::new(device, transfers);
            let mut engine = Self {
                device,
                clock,
                camera: camera.clone(),
                pool,
                decoder: Decoder::new(frequency, max_frame, service_interval_ns),
                bandwidth,
            };
            engine.pool.submit_all()?;
            Ok(engine)
        }

        /// Wait up to `timeout_ms` and decode whatever completed; frames go to `out`.
        pub fn pump(&mut self, timeout_ms: i32, out: &mut Vec<SourceFrame>) -> io::Result<()> {
            if !self.device.wait(timeout_ms)? {
                return Ok(());
            }
            let reap_ns = (self.clock)()?;
            for index in self.pool.reap()? {
                let transfer = self.pool.transfer(index);
                match transfer.status() {
                    0 => {}
                    status if status == -libc::ENODEV || status == -libc::ESHUTDOWN => {
                        return Err(io::Error::new(
                            io::ErrorKind::NotConnected,
                            "camera disconnected",
                        ));
                    }
                    // A whole URB failed: count it as lost packets and keep streaming.
                    _ => self.decoder.stats.packet_errors += 1,
                }
                let packets: Vec<(i32, Vec<u8>)> = (0..transfer.packets())
                    .map(|k| {
                        let (status, bytes) = transfer.packet(k);
                        (status, bytes.to_vec())
                    })
                    .collect();
                self.decoder.on_iso(
                    packets.iter().map(|(s, b)| (*s, b.as_slice())),
                    reap_ns,
                    out,
                );
                self.pool.submit(index)?;
            }
            Ok(())
        }

        /// Stop streaming: every URB back from the kernel, bandwidth released.
        pub fn stop(mut self) {
            self.pool.cancel();
            let _ = self
                .device
                .set_interface(self.camera.streaming_interface, 0);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::uvc::clock::simulation::Bus;
    use crate::uvc::payload::header_bytes;

    /// A synthetic 120 Hz MJPEG camera on a simulated bus, packetized into 32-packet URBs of
    /// 3 KiB high-speed microframes: every frame comes out once, in order, with its start of
    /// exposure within 250 µs, and a lost packet costs exactly the frame it was in.
    /// Streaming joins the frame already leaving the camera: that first frame lacks its
    /// SOI, and is counted apart from a frame truncated mid-stream (a lost payload).
    #[test]
    fn the_partial_frame_at_stream_on_is_not_counted_as_loss() {
        let mut decoder = Decoder::new(48_000_000, 614_400, 125_000);
        let mut out = Vec::new();
        let payload = |fid: bool, eof: bool, data: &[u8]| {
            [header_bytes(fid, eof, Some(1), Some((1, 1))), data.to_vec()].concat()
        };
        // The tail of a frame: no SOI.
        decoder.on_iso(
            [(0, payload(false, true, &[7, 7, 0xff, 0xd9]).as_slice())].into_iter(),
            1,
            &mut out,
        );
        // A whole frame.
        decoder.on_iso(
            [(
                0,
                payload(true, true, &[0xff, 0xd8, 7, 0xff, 0xd9]).as_slice(),
            )]
            .into_iter(),
            2,
            &mut out,
        );
        assert_eq!(decoder.stats.truncated_at_start, 1);
        assert_eq!(decoder.stats.dropped_truncated, 0);
        // A frame whose last payload was lost: no EOI.
        decoder.on_iso(
            [(0, payload(false, true, &[0xff, 0xd8, 7, 7]).as_slice())].into_iter(),
            3,
            &mut out,
        );
        assert_eq!(decoder.stats.dropped_truncated, 1);
        assert_eq!(decoder.stats.truncated_at_start, 1);
    }

    #[test]
    fn iso_urbs_become_timestamped_frames() {
        let mut bus = Bus::new(48_000_000);
        let mut decoder = Decoder::new(48_000_000, 614_400, 125_000);
        let jpeg = |n: u32| {
            let mut bytes = vec![0xff, 0xd8];
            bytes.extend(std::iter::repeat_n(
                (n % 251) as u8,
                9_000 + (n as usize % 7) * 100,
            ));
            bytes.extend_from_slice(&[0xff, 0xd9]);
            bytes
        };
        let period = 1e9 * 513.0 / 61_612.0;
        // Pre-packetize: each frame's bytes leave over consecutive microframes starting 4 ms
        // after its exposure began, 3060 data bytes per packet.
        let mut slots: Vec<(f64, Vec<u8>)> = Vec::new();
        let mut truth = Vec::new();
        for n in 0..1_200_u32 {
            let start = f64::from(n) * period + 1e6;
            let pts = bus.pts(start);
            truth.push(bus.host_ns(start));
            let bytes = jpeg(n);
            let chunks: Vec<&[u8]> = bytes.chunks(3_060).collect();
            let first_slot = ((start + 4e6) / 125_000.0).ceil() * 125_000.0;
            for (k, chunk) in chunks.iter().enumerate() {
                let t = first_slot + k as f64 * 125_000.0;
                let scr = bus.scr(t);
                let eof = k + 1 == chunks.len();
                let packet = [
                    header_bytes(n % 2 == 1, eof, Some(pts), Some(scr)),
                    chunk.to_vec(),
                ]
                .concat();
                slots.push((t, packet));
            }
        }
        slots.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut frames = Vec::new();
        let lost_frame = 600;
        let mut lost_one = false;
        let mut t = 0.0;
        let mut cursor = 0;
        while cursor < slots.len() {
            let mut packets: Vec<(i32, Vec<u8>)> = Vec::new();
            for k in 0..32 {
                let slot = t + f64::from(k) * 125_000.0;
                if cursor < slots.len() && (slots[cursor].0 - slot).abs() < 1.0 {
                    let mut status = 0;
                    // Lose one mid-frame packet of frame 600.
                    if !lost_one && slot > truth_slot(lost_frame, period) {
                        status = -71;
                        lost_one = true;
                    }
                    packets.push((status, slots[cursor].1.clone()));
                    cursor += 1;
                } else {
                    packets.push((0, Vec::new()));
                }
            }
            let reap = bus.host_ns(t + 31.0 * 125_000.0) + bus.latency_ns();
            decoder.on_iso(
                packets.iter().map(|(s, b)| (*s, b.as_slice())),
                reap.round() as i64,
                &mut frames,
            );
            t += 32.0 * 125_000.0;
        }
        assert_eq!(decoder.stats.packet_errors, 1);
        assert_eq!(decoder.stats.dropped_device_error, 1);
        let stats = decoder.stats;
        assert_eq!(
            stats.frames + stats.untimed + stats.dropped_device_error,
            1_200,
            "{stats:?}"
        );
        assert!(stats.untimed < 80, "{stats:?}");
        let mut previous = None;
        for frame in &frames {
            let n = frame.sequence as usize;
            assert_ne!(n, lost_frame);
            assert_eq!(frame.jpeg, jpeg(n as u32));
            // Sparse SCRs here (only the ~4 data packets of each frame carry one, where the
            // camera's 30-60 KB frames carry ten times more) leave the envelope looser than
            // the dense case `clock` tests to 100 µs; the bench bound is 1 ms.
            let error = (frame.soe_ns as f64 - truth[n]).abs();
            assert!(error < 250_000.0, "frame {n}: {error} ns");
            if let Some(p) = previous {
                assert!(n > p);
            }
            previous = Some(n);
        }
    }

    fn truth_slot(frame: usize, period: f64) -> f64 {
        frame as f64 * period + 1e6 + 4e6 + 250_000.0
    }
}
