//! Record where every axis of a body is, read-only, through the embodiment layer's observer.
//!
//! Each bus is read on its own thread at the requested rate: the buses are separate links, and
//! a slow serial bus never holds back the CAN bus. A reading is one fixed-size [`Sample`]
//! copied into a preallocated bounded queue, never a wait (a full queue drops the sample and
//! counts it); one writer thread formats and writes the JSON lines. Nothing here can command,
//! enable or configure an actuator: an observer speaks through a port that refuses any request
//! but a read.
//!
//! Each line is one bus's poll, in joint coordinates through the instance's drive and the
//! units' calibration:
//!
//! ```text
//! {"t": seconds, "bus": port, "positions": {joint: value or null}, "missing": {joint: why}}
//! ```
//!
//! `missing` names each axis without a fresh reading this poll (`timeout`, `stale`, `corrupt`,
//! `unavailable` for a joint out of service, or `fault 0x..`), and is left out when every axis
//! answered.

use std::io::{self, BufWriter, Write};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, TrySendError, sync_channel};
use std::time::{Duration, Instant};

use serde_json::{Map, Value, json};
use sx_embodiment_bodies::BusObserver;
use sx_embodiment_drivers::{Axes, AxisReading, MissingReason, Observe};

/// One bus's poll.
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    /// Seconds since the recording began.
    pub t: f64,
    /// The bus's index among the recorded buses.
    pub bus: usize,
    pub readings: Axes<AxisReading>,
}

/// What a recording wrote.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Recorded {
    pub lines: u64,
    /// Samples a full queue dropped.
    pub dropped: u64,
}

/// Samples queued for the writer: ten seconds of four buses at 50 Hz.
const QUEUE: usize = 2000;

/// Read every bus every `period` until `lasts` has passed or `stop` is set, and write each
/// poll to `out` as a JSON line.
///
/// # Errors
///
/// The writer's I/O failure.
pub fn record(
    buses: Vec<BusObserver>,
    out: Box<dyn Write + Send>,
    period: Duration,
    lasts: Duration,
    stop: &AtomicBool,
) -> io::Result<Recorded> {
    let names: Vec<(String, Vec<String>)> = buses
        .iter()
        .map(|bus| {
            let axes = bus.axes();
            (
                axes.first()
                    .map(|axis| axis.port.clone())
                    .unwrap_or_default(),
                axes.iter().map(|axis| axis.joint.clone()).collect(),
            )
        })
        .collect();
    let (sender, receiver) = sync_channel::<Sample>(QUEUE);
    let dropped = AtomicU64::new(0);
    let began = Instant::now();
    let end = began + lasts;
    std::thread::scope(|scope| {
        let names = &names;
        let writer = scope.spawn(move || write_lines(out, names, &receiver));
        for (index, mut bus) in buses.into_iter().enumerate() {
            let sender = sender.clone();
            let dropped = &dropped;
            scope.spawn(move || {
                let mut next = Instant::now();
                while !stop.load(Ordering::SeqCst) && Instant::now() < end {
                    let readings = bus.read_positions();
                    let sample = Sample {
                        t: began.elapsed().as_secs_f64(),
                        bus: index,
                        readings,
                    };
                    if let Err(TrySendError::Full(_) | TrySendError::Disconnected(_)) =
                        sender.try_send(sample)
                    {
                        dropped.fetch_add(1, Ordering::Relaxed);
                    }
                    next += period;
                    match next.checked_duration_since(Instant::now()) {
                        Some(wait) => std::thread::sleep(
                            wait.min(end.saturating_duration_since(Instant::now())),
                        ),
                        None => next = Instant::now(),
                    }
                }
            });
        }
        drop(sender);
        let lines = writer
            .join()
            .map_err(|_| io::Error::other("the recording writer panicked"))??;
        Ok(Recorded {
            lines,
            dropped: dropped.load(Ordering::Relaxed),
        })
    })
}

fn why(reading: AxisReading) -> Option<String> {
    match reading {
        AxisReading::Fresh { .. } => None,
        AxisReading::Missing(reason) => Some(
            match reason {
                MissingReason::Timeout => "timeout",
                MissingReason::Corrupt => "corrupt",
                MissingReason::Stale => "stale",
                MissingReason::Unavailable => "unavailable",
                MissingReason::OutOfService => "out_of_service",
            }
            .to_owned(),
        ),
        AxisReading::Faulted(code) => Some(format!("fault {code:#x}")),
    }
}

fn write_lines(
    out: Box<dyn Write + Send>,
    names: &[(String, Vec<String>)],
    receiver: &Receiver<Sample>,
) -> io::Result<u64> {
    let mut out = BufWriter::new(out);
    let mut lines = 0;
    for sample in receiver {
        let Some((bus, joints)) = names.get(sample.bus) else {
            continue;
        };
        let mut positions = Map::new();
        let mut missing = Map::new();
        for (joint, reading) in joints.iter().zip(sample.readings.iter()) {
            positions.insert(joint.clone(), json!(reading.position()));
            if let Some(why) = why(*reading) {
                missing.insert(joint.clone(), Value::String(why));
            }
        }
        let mut line = json!({"t": sample.t, "bus": bus, "positions": positions});
        if !missing.is_empty() {
            line["missing"] = Value::Object(missing);
        }
        serde_json::to_writer(&mut out, &line)?;
        out.write_all(b"\n")?;
        out.flush()?;
        lines += 1;
    }
    Ok(lines)
}
