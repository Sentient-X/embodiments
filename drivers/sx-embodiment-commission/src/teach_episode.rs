//! Local teach recordings: bounded nonblocking capture, durable completion and strict replay.
//!
//! A recording without its completion footer is never a replayable episode. Queue saturation
//! or writer failure invalidates the whole take, rather than silently losing motor samples.

use std::collections::BTreeSet;
use std::fs::{File, OpenOptions};
use std::io::{self, BufRead, BufReader, BufWriter, Read, Write};
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{SyncSender, TrySendError, sync_channel};
use std::thread::JoinHandle;

use serde::{Deserialize, Serialize};
use thiserror::Error;

pub const SCHEMA: &str = "sx.local-teach-episode/v1";
pub const MAX_AXES: usize = 32;
/// One hour at the supported maximum recording rate, with a bounded file and line size.
pub const MAX_SAMPLES: usize = 360_000;
pub const MAX_DURATION_S: f64 = 3600.0;
const MAX_BYTES: u64 = 256 * 1024 * 1024;
const MAX_LINE: u64 = 32 * 1024;

#[derive(Debug, Error)]
pub enum EpisodeError {
    #[error("episode I/O: {0}")]
    Io(#[from] io::Error),
    #[error("episode JSON: {0}")]
    Json(#[from] serde_json::Error),
    #[error("invalid episode: {0}")]
    Invalid(String),
    #[error("recording invalidated: writer failed or its bounded queue filled")]
    WriterFailed,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Header {
    pub schema: String,
    pub configuration_sha256: String,
    pub joints: Vec<String>,
    pub period_s: f64,
}

impl Header {
    /// The exact local robot configuration and native joint order this recording belongs to.
    ///
    /// # Errors
    /// Rejects a malformed digest, duplicate/empty joints or unsupported period.
    pub fn new(
        configuration_sha256: String,
        joints: Vec<String>,
        period_s: f64,
    ) -> Result<Self, EpisodeError> {
        let header = Self {
            schema: SCHEMA.to_owned(),
            configuration_sha256,
            joints,
            period_s,
        };
        header.validate()?;
        Ok(header)
    }

    fn validate(&self) -> Result<(), EpisodeError> {
        if self.schema != SCHEMA
            || self.configuration_sha256.len() != 64
            || !self
                .configuration_sha256
                .bytes()
                .all(|b| b.is_ascii_hexdigit())
            || self.joints.is_empty()
            || self.joints.len() > MAX_AXES
            || self.joints.iter().any(|joint| joint.trim().is_empty())
            || self.joints.iter().collect::<BTreeSet<_>>().len() != self.joints.len()
            || !self.period_s.is_finite()
            || !(0.01..=1.0).contains(&self.period_s)
        {
            return invalid("header schema, configuration, joint order or period");
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sample {
    pub t_s: f64,
    pub position: Vec<f64>,
    pub velocity: Vec<f64>,
}

#[derive(Serialize, Deserialize)]
#[serde(
    tag = "type",
    content = "data",
    rename_all = "snake_case",
    deny_unknown_fields
)]
enum Line {
    Header(Header),
    Sample(Sample),
    Complete(Complete),
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Complete {
    samples: usize,
    duration_s: f64,
}

#[derive(Clone, Copy)]
struct QueuedSample {
    t_s: f64,
    position: [f64; MAX_AXES],
    velocity: [f64; MAX_AXES],
}

#[allow(clippy::large_enum_variant)] // Inline samples keep control-tick enqueue allocation-free.
enum Message {
    Sample(QueuedSample),
    Finish,
}

/// One take's recorder. Dropping it without `finish` leaves an incomplete file.
pub struct EpisodeWriter {
    sender: Option<SyncSender<Message>>,
    worker: Option<JoinHandle<Result<usize, EpisodeError>>>,
    failed: Arc<AtomicBool>,
    width: usize,
    count: usize,
    previous: Option<f64>,
}

impl EpisodeWriter {
    /// Create a new file without replacing a previous take.
    ///
    /// # Errors
    /// Invalid header, zero queue capacity, or failure to create the file.
    pub fn create(path: &Path, header: Header, capacity: usize) -> Result<Self, EpisodeError> {
        header.validate()?;
        if capacity == 0 {
            return invalid("recording queue must have capacity");
        }
        let file = OpenOptions::new().write(true).create_new(true).open(path)?;
        Self::spawn(file, header, capacity)
    }

    fn spawn(file: File, header: Header, capacity: usize) -> Result<Self, EpisodeError> {
        let width = header.joints.len();
        let (sender, receiver) = sync_channel(capacity);
        let failed = Arc::new(AtomicBool::new(false));
        let worker_failed = Arc::clone(&failed);
        let worker = std::thread::Builder::new()
            .name("sx-teach-record".to_owned())
            .spawn(move || {
                let outcome = (|| {
                    let mut out = BufWriter::new(file);
                    write_line(&mut out, &Line::Header(header))?;
                    let mut count = 0;
                    let mut duration_s = 0.0;
                    while let Ok(message) = receiver.recv() {
                        if worker_failed.load(Ordering::Acquire) {
                            return Err(EpisodeError::WriterFailed);
                        }
                        match message {
                            Message::Sample(sample) => {
                                write_line(
                                    &mut out,
                                    &Line::Sample(Sample {
                                        t_s: sample.t_s,
                                        position: sample.position[..width].to_vec(),
                                        velocity: sample.velocity[..width].to_vec(),
                                    }),
                                )?;
                                // Surface disk failures during capture rather than only at finish.
                                out.flush()?;
                                if out.get_ref().metadata()?.len() > MAX_BYTES - MAX_LINE {
                                    return invalid("recording exceeds file size limit");
                                }
                                count += 1;
                                duration_s = sample.t_s;
                            }
                            Message::Finish => {
                                if count < 2 {
                                    return invalid("at least two samples are needed");
                                }
                                write_line(
                                    &mut out,
                                    &Line::Complete(Complete {
                                        samples: count,
                                        duration_s,
                                    }),
                                )?;
                                out.flush()?;
                                out.get_ref().sync_all()?;
                                return Ok(count);
                            }
                        }
                    }
                    invalid("recording ended without completion")
                })();
                if outcome.is_err() {
                    worker_failed.store(true, Ordering::Release);
                }
                outcome
            })?;
        Ok(Self {
            sender: Some(sender),
            worker: Some(worker),
            failed,
            width,
            count: 0,
            previous: None,
        })
    }

    /// Check the disk-writer state without blocking the control loop.
    ///
    /// # Errors
    /// Returns an invalidation immediately after the worker reports its failure.
    pub fn check(&self) -> Result<(), EpisodeError> {
        if self.failed.load(Ordering::Acquire) {
            Err(EpisodeError::WriterFailed)
        } else {
            Ok(())
        }
    }

    /// Copy measured joints into the bounded queue without allocation or waiting.
    ///
    /// # Errors
    /// Invalid sample, a full/disconnected queue, or a prior writer failure invalidates the take.
    pub fn record(
        &mut self,
        t_s: f64,
        position: &[f64],
        velocity: &[f64],
    ) -> Result<(), EpisodeError> {
        self.check()?;
        if let Err(error) = validate_sample(
            self.width,
            self.previous,
            self.count,
            t_s,
            position,
            velocity,
        ) {
            self.failed.store(true, Ordering::Release);
            return Err(error);
        }
        let mut sample = QueuedSample {
            t_s,
            position: [0.0; MAX_AXES],
            velocity: [0.0; MAX_AXES],
        };
        sample.position[..self.width].copy_from_slice(position);
        sample.velocity[..self.width].copy_from_slice(velocity);
        let sender = self.sender.as_ref().ok_or(EpisodeError::WriterFailed)?;
        match sender.try_send(Message::Sample(sample)) {
            Ok(()) => {
                self.previous = Some(t_s);
                self.count += 1;
                Ok(())
            }
            Err(TrySendError::Full(_) | TrySendError::Disconnected(_)) => {
                self.failed.store(true, Ordering::Release);
                Err(EpisodeError::WriterFailed)
            }
        }
    }

    /// Drain, complete, flush and sync the take. Call outside the motor control thread.
    ///
    /// # Errors
    /// An invalidated take, fewer than two samples, writer failure or panic.
    pub fn finish(mut self) -> Result<usize, EpisodeError> {
        let healthy = self.check();
        if healthy.is_ok()
            && let Some(sender) = self.sender.take()
            && sender.send(Message::Finish).is_err()
        {
            self.failed.store(true, Ordering::Release);
        }
        self.sender = None;
        let outcome = self
            .worker
            .take()
            .ok_or(EpisodeError::WriterFailed)?
            .join()
            .map_err(|_| EpisodeError::WriterFailed)?;
        healthy?;
        outcome
    }
}

fn write_line(out: &mut impl Write, line: &Line) -> Result<(), EpisodeError> {
    serde_json::to_writer(&mut *out, line)?;
    out.write_all(b"\n")?;
    Ok(())
}

fn invalid<T>(reason: &str) -> Result<T, EpisodeError> {
    Err(EpisodeError::Invalid(reason.to_owned()))
}

fn validate_sample(
    width: usize,
    previous: Option<f64>,
    count: usize,
    t_s: f64,
    position: &[f64],
    velocity: &[f64],
) -> Result<(), EpisodeError> {
    if count >= MAX_SAMPLES
        || !t_s.is_finite()
        || !(0.0..=MAX_DURATION_S).contains(&t_s)
        || previous.is_none() && t_s != 0.0
        || previous.is_some_and(|previous| t_s <= previous)
        || position.len() != width
        || velocity.len() != width
        || position
            .iter()
            .chain(velocity)
            .any(|value| !value.is_finite())
    {
        return invalid("sample count, timing, width or finite measurements");
    }
    Ok(())
}

/// A completed, configuration-matched recording, ready for governed replay.
#[derive(Debug)]
pub struct Episode {
    pub header: Header,
    pub samples: Vec<Sample>,
}

impl Episode {
    /// Load a bounded completed take matching the exact configuration and joint order.
    ///
    /// # Errors
    /// Refuses partial, malformed, oversized or mismatched recordings.
    pub fn load(path: &Path, expected: &Header) -> Result<Self, EpisodeError> {
        expected.validate()?;
        let file = File::open(path)?;
        if file.metadata()?.len() > MAX_BYTES {
            return invalid("episode exceeds file size limit");
        }
        Self::read(BufReader::new(file), expected)
    }

    fn read(mut reader: impl BufRead, expected: &Header) -> Result<Self, EpisodeError> {
        let mut bytes = Vec::new();
        let mut total = 0_u64;
        let mut header = None;
        let mut samples: Vec<Sample> = Vec::new();
        let mut complete = false;
        loop {
            bytes.clear();
            let read = reader
                .by_ref()
                .take(MAX_LINE + 1)
                .read_until(b'\n', &mut bytes)?;
            if read == 0 {
                break;
            }
            total += u64::try_from(read).map_err(|_| EpisodeError::Invalid("byte count".into()))?;
            if bytes.len() > usize::try_from(MAX_LINE).expect("line limit fits usize")
                || total > MAX_BYTES
            {
                return invalid("episode exceeds byte limits");
            }
            if complete || bytes.last() != Some(&b'\n') {
                return invalid("trailing data or incomplete line");
            }
            match serde_json::from_slice::<Line>(&bytes)? {
                Line::Header(found) if header.is_none() && samples.is_empty() => {
                    found.validate()?;
                    if found.configuration_sha256 != expected.configuration_sha256
                        || found.joints != expected.joints
                    {
                        return invalid("configuration identity or native joint order differs");
                    }
                    header = Some(found);
                }
                Line::Sample(sample) if header.is_some() => {
                    validate_sample(
                        expected.joints.len(),
                        samples.last().map(|s| s.t_s),
                        samples.len(),
                        sample.t_s,
                        &sample.position,
                        &sample.velocity,
                    )?;
                    samples.push(sample);
                }
                Line::Complete(footer) if header.is_some() && samples.len() >= 2 => {
                    if footer.samples != samples.len()
                        || Some(footer.duration_s) != samples.last().map(|sample| sample.t_s)
                    {
                        return invalid("completion count or duration differs");
                    }
                    complete = true;
                }
                _ => return invalid("unexpected episode record order"),
            }
        }
        if !complete {
            return invalid("missing completion footer");
        }
        Ok(Self {
            header: header.ok_or_else(|| EpisodeError::Invalid("missing header".into()))?,
            samples,
        })
    }

    /// Interpolate recorded positions at local elapsed time, slowed by `speed` (0 < speed <= 1).
    /// Returns true at/after the last sample, filling the last position for the final hold.
    ///
    /// # Errors
    /// Invalid elapsed time, speed or output width.
    pub fn interpolate(
        &self,
        elapsed_s: f64,
        speed: f64,
        out: &mut [f64],
    ) -> Result<bool, EpisodeError> {
        if !(elapsed_s.is_finite()
            && elapsed_s >= 0.0
            && speed.is_finite()
            && 0.0 < speed
            && speed <= 1.0)
            || out.len() != self.header.joints.len()
        {
            return invalid("replay elapsed time, speed or output width");
        }
        let time = elapsed_s * speed;
        let last = self
            .samples
            .last()
            .ok_or_else(|| EpisodeError::Invalid("no samples".into()))?;
        if time >= last.t_s {
            out.copy_from_slice(&last.position);
            return Ok(true);
        }
        let upper = self.samples.partition_point(|sample| sample.t_s <= time);
        let right = &self.samples[upper];
        let left = &self.samples[upper.saturating_sub(1)];
        let mix = (time - left.t_s) / (right.t_s - left.t_s);
        for ((target, a), b) in out.iter_mut().zip(&left.position).zip(&right.position) {
            *target = a * (1.0 - mix) + b * mix;
        }
        Ok(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicU64;

    static NEXT: AtomicU64 = AtomicU64::new(0);
    fn path() -> std::path::PathBuf {
        std::env::temp_dir().join(format!(
            "sx-teach-episode-{}-{}.jsonl",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ))
    }
    fn header() -> Header {
        Header::new(
            "ab".repeat(32),
            vec!["elbow".into(), "gripper".into()],
            0.01,
        )
        .unwrap()
    }
    fn serialized() -> Vec<u8> {
        let mut bytes = Vec::new();
        write_line(&mut bytes, &Line::Header(header())).unwrap();
        write_line(
            &mut bytes,
            &Line::Sample(Sample {
                t_s: 0.0,
                position: vec![0.0, 0.2],
                velocity: vec![0.0, 0.0],
            }),
        )
        .unwrap();
        write_line(
            &mut bytes,
            &Line::Sample(Sample {
                t_s: 2.0,
                position: vec![1.0, 0.8],
                velocity: vec![0.0, 0.0],
            }),
        )
        .unwrap();
        write_line(
            &mut bytes,
            &Line::Complete(Complete {
                samples: 2,
                duration_s: 2.0,
            }),
        )
        .unwrap();
        bytes
    }

    #[test]
    fn durable_take_reopens_and_never_overwrites() {
        let path = path();
        let mut recorder = EpisodeWriter::create(&path, header(), 16).unwrap();
        assert!(EpisodeWriter::create(&path, header(), 16).is_err());
        recorder.record(0.0, &[0.0, 0.2], &[0.0, 0.0]).unwrap();
        recorder.record(2.0, &[1.0, 0.8], &[0.0, 0.0]).unwrap();
        assert_eq!(recorder.finish().unwrap(), 2);
        let episode = Episode::load(&path, &header()).unwrap();
        let mut out = [0.0; 2];
        assert!(!episode.interpolate(2.0, 0.5, &mut out).unwrap());
        assert!(out.iter().all(|value| (*value - 0.5).abs() < 1e-12));
        assert!(episode.interpolate(4.0, 0.5, &mut out).unwrap());
        assert!((out[0] - 1.0).abs() < 1e-12 && (out[1] - 0.8).abs() < 1e-12);
        assert!(episode.interpolate(1.0, 1.01, &mut out).is_err());
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn refuses_truncation_wrong_identity_order_counts_and_timing() {
        let bytes = serialized();
        let text = String::from_utf8(bytes.clone()).unwrap();
        let lines: Vec<&str> = text.split_inclusive('\n').collect();
        for end in 0..bytes.len() {
            assert!(
                Episode::read(&bytes[..end], &header()).is_err(),
                "accepted truncation at {end}"
            );
        }
        let mut wrong = header();
        wrong.configuration_sha256 = "cd".repeat(32);
        assert!(Episode::read(bytes.as_slice(), &wrong).is_err());
        wrong = header();
        wrong.joints.reverse();
        assert!(Episode::read(bytes.as_slice(), &wrong).is_err());
        for malformed in [
            format!("{}{}{}{}", lines[0], lines[2], lines[1], lines[3]),
            text.replace("\"samples\":2", "\"samples\":3"),
            text.replace("\"t_s\":2.0", "\"t_s\":0.0"),
            text.replace("\"t_s\":2.0", "\"unknown\":1,\"t_s\":2.0"),
            format!("{text}{}", lines[1]),
        ] {
            assert!(Episode::read(malformed.as_bytes(), &header()).is_err());
        }
    }

    #[test]
    fn invalid_measurement_invalidates_entire_take() {
        let path = path();
        let mut recorder = EpisodeWriter::create(&path, header(), 16).unwrap();
        recorder.record(0.0, &[0.0, 0.0], &[0.0, 0.0]).unwrap();
        assert!(recorder.record(1.0, &[f64::NAN, 0.0], &[0.0, 0.0]).is_err());
        assert!(recorder.check().is_err());
        assert!(recorder.finish().is_err());
        assert!(Episode::load(&path, &header()).is_err());
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn saturated_queue_is_reported_and_cannot_finish() {
        // A deliberately held receiver reproduces a disk writer that cannot drain its queue.
        let (sender, _receiver) = sync_channel(1);
        let mut recorder = EpisodeWriter {
            sender: Some(sender),
            worker: None,
            failed: Arc::new(AtomicBool::new(false)),
            width: 2,
            count: 0,
            previous: None,
        };
        recorder.record(0.0, &[0.0, 0.0], &[0.0, 0.0]).unwrap();
        assert!(matches!(
            recorder.record(0.01, &[0.0, 0.0], &[0.0, 0.0]),
            Err(EpisodeError::WriterFailed)
        ));
        assert!(recorder.check().is_err());
        assert!(recorder.finish().is_err());
    }

    #[test]
    #[cfg(target_os = "linux")]
    fn disk_write_failure_surfaces_during_recording() {
        let file = OpenOptions::new().write(true).open("/dev/full").unwrap();
        let mut recorder = EpisodeWriter::spawn(file, header(), 16).unwrap();
        recorder.record(0.0, &[0.0, 0.0], &[0.0, 0.0]).unwrap();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
        while recorder.check().is_ok() && std::time::Instant::now() < deadline {
            std::thread::yield_now();
        }
        assert!(matches!(recorder.check(), Err(EpisodeError::WriterFailed)));
        assert!(recorder.finish().is_err());
    }
}
