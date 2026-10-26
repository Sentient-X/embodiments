//! `sx-record`: record where every joint of a body is while a person moves it by hand, without
//! moving anything.
//!
//! ```text
//! sx-record --drive <drive.json> --commissioning <commissioning.json>
//!           [--calibration <port>=<lerobot.json>]... [--allow-provisional <port>]...
//!           [--exclude <joint,...>] --out <file.jsonl> [--seconds 60] [--rate 50]
//! ```
//!
//! The body is observed through the embodiment layer (`sx_embodiment_bodies::observe`) on the
//! bridges the commissioning names: Damiao motors by register reads and status refreshes,
//! Feetech and Dynamixel servos by `PING` and `READ`, each through a port that refuses any other
//! request, so nothing here can enable or move an actuator. Observing a provisional port writes
//! nothing, so `--allow-provisional` needs no supervision. Each bus is read on its own thread;
//! each poll is one JSON line in joint coordinates (`sx_embodiment_commission::record`). A
//! serial port opens to one process: stop the recorder (or leave a port out) before a tool that
//! drives it.

use std::fs::File;
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use sx_embodiment_commission::bench::BodyFlags;
use sx_embodiment_commission::record::record;

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("sx-record: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let mut body = BodyFlags::default();
    let (mut out, mut seconds, mut rate) = (None::<String>, 60.0_f64, 50.0_f64);
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} needs a value"));
        if body.take(&arg, &mut value)? {
            continue;
        }
        match arg.as_str() {
            "--out" => out = Some(value()?),
            "--seconds" => seconds = value()?.parse().map_err(|_| "bad --seconds")?,
            "--rate" => rate = value()?.parse().map_err(|_| "bad --rate")?,
            other => return Err(format!("unknown argument {other}")),
        }
    }
    let out = out.ok_or("--out is required")?;
    if !(rate > 0.0 && rate <= 500.0) {
        return Err("--rate is above 0 and at most 500 Hz".into());
    }
    let lasts = Duration::try_from_secs_f64(seconds).map_err(|_| "bad --seconds")?;
    let observer = body.observe()?;
    let file = File::create(&out).map_err(|error| format!("{out}: {error}"))?;
    let stop = Arc::new(AtomicBool::new(false));
    {
        let stop = Arc::clone(&stop);
        ctrlc::set_handler(move || stop.store(true, Ordering::SeqCst))
            .map_err(|error| error.to_string())?;
    }
    let recorded = record(
        observer.into_buses(),
        Box::new(file),
        Duration::from_secs_f64(1.0 / rate),
        lasts,
        &stop,
    )
    .map_err(|error| error.to_string())?;
    eprintln!(
        "sx-record: {} polls written to {out}, {} dropped",
        recorded.lines, recorded.dropped
    );
    Ok(())
}
