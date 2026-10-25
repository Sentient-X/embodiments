//! `sx-record`: record where every joint of the static upper body is while a person moves it by
//! hand, without moving anything.
//!
//! ```text
//! sx-record --out <file.jsonl> [--seconds 60] [--rate 50]
//!           [--zlg-serial <usb serial>] [--channel 1]
//!           [--feetech <name>=<tty>:<id,id,...>]...
//! ```
//!
//! Damiao motors are asked for a feedback frame with `refresh_motor_status` (0x7FF, 0xCC), which
//! never enables a motor; every frame leaving the CAN port is checked to be that or a register
//! read. Feetech servos are read with single `READ`s of `Present_Position` through
//! [`ReadOnlyPort`], which refuses anything but `PING` and `READ`. Each bus runs on its own
//! thread at the requested rate and every sample is one JSON line:
//! `{"t": seconds, "bus": name, "positions": {joint or id: value}}`, Damiao in radians as the
//! motor reports them, Feetech in raw ticks (calibration is applied when the recording is shown).

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::process::ExitCode;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use serde_json::{Map, Value, json};
use sx_damiao_can::dm_can::{DmMotorType, Motor, MotorControl};
use sx_embodiment_commission::tty::TtyPort;
use sx_embodiment_drivers::body::BodyBindings;
use sx_embodiment_drivers::can::{CanFrame, CanPort};
use sx_feetech_serial::scan::ReadOnlyPort;
use sx_feetech_serial::scservo_sdk::{
    CommResult, ProtocolPacketHandler, SMS_STS_PRESENT_POSITION_L,
};
use sx_zlg_usbcanfd::{ChannelOptions, UsbCanFd};

/// The register-access id every Damiao motor listens on (`DM_CAN.py`'s 0x7FF).
const PARAM_ID: u32 = 0x7FF;
/// How long the refresh replies of one sweep are given before they are read.
const REPLY_WINDOW: Duration = Duration::from_millis(5);

/// A CAN port that only lets out refresh requests and register reads.
struct RefreshOnly<P>(P);

impl<P: CanPort> CanPort for RefreshOnly<P> {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        let asks = frame.id == PARAM_ID && matches!(frame.data[2], 0xCC | 0x33);
        if !asks {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "sx-record sends only refresh requests and register reads",
            ));
        }
        self.0.send(frame)
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        self.0.receive()
    }
}

fn motor_type(model: &str) -> Option<DmMotorType> {
    match model {
        "damiao_dm4310" => Some(DmMotorType::DM4310),
        "damiao_dm4340" => Some(DmMotorType::DM4340),
        "damiao_dm8009" => Some(DmMotorType::DM8009),
        _ => None,
    }
}

struct Sample {
    t: f64,
    bus: String,
    positions: Map<String, Value>,
}

fn damiao(
    serial: Option<&str>,
    channel: u8,
    rate: f64,
    until: Instant,
    start: Instant,
    out: &mpsc::Sender<Sample>,
) -> Result<(), String> {
    let body = BodyBindings::parse(sx_damiao_can::static_upper_body::BINDINGS)
        .map_err(|error| error.to_string())?;
    let adapter = UsbCanFd::open(serial).map_err(|error| error.to_string())?;
    let port = adapter
        .open_channel(channel, ChannelOptions::default())
        .map_err(|error| error.to_string())?;
    let mut control = MotorControl::new(RefreshOnly(port));
    let mut joints = Vec::new();
    for axis in &body.axes {
        // The torso motor (0x1F) is known faulty; it is asked like the rest and shows as null.
        let model = motor_type(axis.binding.model.wire())
            .ok_or_else(|| format!("{}: not a Damiao model", axis.joint))?;
        control.add_motor(Motor::new(
            model,
            axis.binding.bus_id,
            axis.binding.bus_id + 0x20,
        ));
        joints.push((axis.joint.clone(), axis.binding.bus_id));
    }
    let period = Duration::from_secs_f64(1.0 / rate);
    let mut next = Instant::now();
    let mut seen: Vec<u64> = vec![0; joints.len()];
    while Instant::now() < until {
        for (_, id) in &joints {
            control
                .refresh_motor_status(*id)
                .map_err(|error| error.to_string())?;
        }
        thread::sleep(REPLY_WINDOW);
        control.recv().map_err(|error| error.to_string())?;
        let mut positions = Map::new();
        for (index, (joint, id)) in joints.iter().enumerate() {
            let motor = control.motor(*id).map_err(|error| error.to_string())?;
            let fresh = motor.feedback_count != seen[index];
            seen[index] = motor.feedback_count;
            positions.insert(
                joint.clone(),
                if fresh {
                    json!(motor.get_position())
                } else {
                    Value::Null
                },
            );
        }
        let _ = out.send(Sample {
            t: start.elapsed().as_secs_f64(),
            bus: "can".to_owned(),
            positions,
        });
        next += period;
        if let Some(wait) = next.checked_duration_since(Instant::now()) {
            thread::sleep(wait);
        } else {
            next = Instant::now();
        }
    }
    Ok(())
}

fn feetech(
    name: &str,
    tty: &str,
    ids: &[u8],
    rate: f64,
    until: Instant,
    start: Instant,
    out: &mpsc::Sender<Sample>,
) -> Result<(), String> {
    let port = TtyPort::open(tty, 1_000_000).map_err(|error| format!("{tty}: {error}"))?;
    let mut handler = ProtocolPacketHandler::new(ReadOnlyPort::new(port), 0);
    let period = Duration::from_secs_f64(1.0 / rate);
    let mut next = Instant::now();
    while Instant::now() < until {
        let mut positions = Map::new();
        for id in ids {
            let (ticks, result, _) = handler.read2_byte_tx_rx(*id, SMS_STS_PRESENT_POSITION_L);
            positions.insert(
                id.to_string(),
                if result == CommResult::Success {
                    json!(ticks)
                } else {
                    Value::Null
                },
            );
        }
        let _ = out.send(Sample {
            t: start.elapsed().as_secs_f64(),
            bus: name.to_owned(),
            positions,
        });
        next += period;
        if let Some(wait) = next.checked_duration_since(Instant::now()) {
            thread::sleep(wait);
        } else {
            next = Instant::now();
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    let mut args = std::env::args().skip(1);
    let mut out_path = None;
    let mut seconds = 60.0;
    let mut rate = 50.0;
    let mut serial = None;
    let mut channel = 1;
    let mut buses: Vec<(String, String, Vec<u8>)> = Vec::new();
    while let Some(arg) = args.next() {
        let mut value = || args.next().unwrap_or_default();
        match arg.as_str() {
            "--out" => out_path = Some(value()),
            "--seconds" => seconds = value().parse().unwrap_or(seconds),
            "--rate" => rate = value().parse().unwrap_or(rate),
            "--zlg-serial" => serial = Some(value()),
            "--channel" => channel = value().parse().unwrap_or(channel),
            "--feetech" => {
                // name=tty:1,2,3
                let spec = value();
                let Some((name, rest)) = spec.split_once('=') else {
                    eprintln!("--feetech wants name=tty:id,id");
                    return ExitCode::FAILURE;
                };
                let Some((tty, ids)) = rest.rsplit_once(':') else {
                    eprintln!("--feetech wants name=tty:id,id");
                    return ExitCode::FAILURE;
                };
                let ids = ids.split(',').filter_map(|id| id.parse().ok()).collect();
                buses.push((name.to_owned(), tty.to_owned(), ids));
            }
            other => {
                eprintln!("unknown argument {other}");
                return ExitCode::FAILURE;
            }
        }
    }
    let Some(out_path) = out_path else {
        eprintln!("--out <file.jsonl> is required");
        return ExitCode::FAILURE;
    };
    let start = Instant::now();
    let until = start + Duration::from_secs_f64(seconds);
    let (sender, receiver) = mpsc::channel::<Sample>();
    let mut workers = Vec::new();
    {
        let sender = sender.clone();
        workers.push(thread::spawn(move || {
            damiao(serial.as_deref(), channel, rate, until, start, &sender)
                .map_err(|e| format!("can: {e}"))
        }));
    }
    for (name, tty, ids) in buses {
        let sender = sender.clone();
        workers.push(thread::spawn(move || {
            feetech(&name, &tty, &ids, rate, until, start, &sender)
                .map_err(|e| format!("{name}: {e}"))
        }));
    }
    drop(sender);
    let file = match File::create(&out_path) {
        Ok(file) => file,
        Err(error) => {
            eprintln!("{out_path}: {error}");
            return ExitCode::FAILURE;
        }
    };
    let mut writer = BufWriter::new(file);
    let mut counts = std::collections::BTreeMap::<String, usize>::new();
    let mut last_report = Instant::now();
    for sample in receiver {
        *counts.entry(sample.bus.clone()).or_default() += 1;
        let line = json!({"t": sample.t, "bus": sample.bus, "positions": sample.positions});
        let _ = writeln!(writer, "{line}");
        if last_report.elapsed() > Duration::from_secs(5) {
            eprintln!(
                "{:5.1} s  samples {counts:?}",
                start.elapsed().as_secs_f64()
            );
            last_report = Instant::now();
        }
    }
    let _ = writer.flush();
    let mut failed = false;
    for worker in workers {
        if let Ok(Err(error)) = worker.join() {
            eprintln!("{error}");
            failed = true;
        }
    }
    eprintln!("wrote {out_path}: {counts:?}");
    if failed {
        ExitCode::FAILURE
    } else {
        ExitCode::SUCCESS
    }
}
