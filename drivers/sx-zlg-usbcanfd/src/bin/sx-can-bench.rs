//! Measure a CAN link with read-only Damiao register reads; see `sx_zlg_usbcanfd::bench`.
//!
//!     sx-can-bench zlg [--serial S] [--channel 1] [--in-flight 1|2] [options]
//!     sx-can-bench socketcan <interface> [options]        (Linux, feature `socketcan`)
//!
//! Options: --motors 0x09,0x10,...  (master = send + 0x20; default the static upper body's
//! fifteen) --samples N --rates 50,100,250,500 --seconds S --throughput-seconds S
//! --timeout-ms N --json PATH --max-load L (default 20).
//!
//! Refuses a debug build, and a machine whose 1-minute load average is at or above the limit,
//! since either would measure the host, not the link.

use std::process::ExitCode;

use sx_zlg_usbcanfd::bench::{self, Config, ReadOnly};
use sx_zlg_usbcanfd::{ChannelOptions, Options, UsbCanFd};

fn main() -> ExitCode {
    match run(std::env::args().skip(1).collect()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("sx-can-bench: {error}");
            ExitCode::FAILURE
        }
    }
}

fn number(text: &str) -> Result<u64, String> {
    let parsed = text
        .strip_prefix("0x")
        .map_or_else(|| text.parse(), |hex| u64::from_str_radix(hex, 16));
    parsed.map_err(|_| format!("{text:?} is not a number"))
}

#[allow(clippy::too_many_lines)]
fn run(args: Vec<String>) -> Result<(), String> {
    let mut args = args.into_iter();
    let transport = args.next().ok_or("name a transport: zlg or socketcan")?;
    let mut config = Config::static_upper_body();
    let (mut serial, mut channel, mut in_flight) = (None, 1_u8, 1_usize);
    let (mut interface, mut json, mut max_load) = (None, None, 20.0_f64);
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} needs a value"));
        match arg.as_str() {
            "--serial" => serial = Some(value()?),
            "--channel" => channel = u8::try_from(number(&value()?)?).map_err(|e| e.to_string())?,
            "--in-flight" => {
                in_flight = usize::try_from(number(&value()?)?).map_err(|e| e.to_string())?;
            }
            "--motors" => {
                config.motors = value()?
                    .split(',')
                    .map(|id| {
                        let id = u16::try_from(number(id)?).map_err(|e| e.to_string())?;
                        Ok((id, id + 0x20))
                    })
                    .collect::<Result<_, String>>()?;
            }
            "--samples" => {
                config.latency_samples =
                    usize::try_from(number(&value()?)?).map_err(|e| e.to_string())?;
            }
            "--rates" => {
                config.rates_hz = value()?
                    .split(',')
                    .map(|rate| u32::try_from(number(rate)?).map_err(|e| e.to_string()))
                    .collect::<Result<_, String>>()?;
            }
            "--seconds" => config.cycle_seconds = value()?.parse().map_err(|_| "bad --seconds")?,
            "--throughput-seconds" => {
                config.throughput_seconds =
                    value()?.parse().map_err(|_| "bad --throughput-seconds")?;
            }
            "--timeout-ms" => config.reply_timeout_ms = number(&value()?)?,
            "--json" => json = Some(value()?),
            "--max-load" => max_load = value()?.parse().map_err(|_| "bad --max-load")?,
            other if interface.is_none() && !other.starts_with("--") => {
                interface = Some(other.to_owned());
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    if cfg!(debug_assertions) {
        return Err(
            "build with --release: a debug build measures the compiler, not the link".to_owned(),
        );
    }
    match bench::load_average() {
        Some(load) if load >= max_load => {
            return Err(format!(
                "the 1-minute load average is {load:.1}, at or above {max_load}; wait for a quiet machine"
            ));
        }
        Some(_) => {}
        None => eprintln!("sx-can-bench: the load average cannot be read here; measuring anyway"),
    }
    let (report, extra) = match transport.as_str() {
        "zlg" => {
            let adapter = UsbCanFd::open_with(
                serial.as_deref(),
                Options {
                    in_flight,
                    ..Options::default()
                },
            )
            .map_err(|error| error.to_string())?;
            eprintln!(
                "probe: {}",
                serde_json::to_string(adapter.probe()).unwrap_or_default()
            );
            let port = adapter
                .open_channel(channel, ChannelOptions::default())
                .map_err(|error| error.to_string())?;
            let mut port = ReadOnly::new(port);
            let name =
                format!("zlg userspace, channel {channel}, {in_flight} transmit(s) in flight");
            let report = bench::run(&mut port, &name, &config, bench::process_cpu)
                .map_err(|error| error.to_string())?;
            let extra = serde_json::json!({
                "probe": adapter.probe(),
                "channel": port.inner().stats(),
                "device": adapter.stats(),
            });
            (report, extra)
        }
        #[cfg(all(feature = "socketcan", target_os = "linux"))]
        "socketcan" => {
            let interface = interface.ok_or("name the SocketCAN interface, e.g. can1")?;
            let port =
                sx_socketcan::SocketCanPort::open(&interface).map_err(|error| error.to_string())?;
            let mut port = ReadOnly::new(port);
            let report = bench::run(
                &mut port,
                &format!("socketcan {interface}"),
                &config,
                bench::process_cpu,
            )
            .map_err(|error| error.to_string())?;
            (report, serde_json::Value::Null)
        }
        other => {
            return Err(format!(
                "unknown transport {other} (zlg, or socketcan on Linux with the feature)"
            ));
        }
    };
    print!("{}", bench::table(&report));
    let document = serde_json::json!({ "report": report, "transport_detail": extra });
    let text = serde_json::to_string_pretty(&document).map_err(|error| error.to_string())?;
    match json {
        Some(path) => std::fs::write(&path, text).map_err(|error| format!("{path}: {error}"))?,
        None => println!("{text}"),
    }
    Ok(())
}
