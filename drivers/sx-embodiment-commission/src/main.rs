//! `sx-commission`: find a robot's bridges and what answers on them, and commission its ports.
//!
//! ```text
//! sx-commission probe [--json] [--feetech-ids 1-20] [--damiao-ids 1-32]
//! sx-commission commission --bindings <file>... [--instance <id> --pod <id> --revision <n>]
//!                          [--json] [--write <dir>]
//! sx-commission calibrate read|gripper|jog --bus <tty> ... [--move]
//! sx-commission calibrate --restore <journal>
//! ```
//!
//! `probe` and `commission` send a bus only PINGs, READs and register reads; see the library
//! header. `commission` prints what it would record and writes only under `--write`.
//! `calibrate` writes registers and moves servos only under `--move`
//! (`sx_embodiment_commission::calibrate`).

use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::AtomicUsize;

use serde_json::json;
use sx_embodiment_commission::declared::{BusKind, DeclaredBody};
use sx_embodiment_commission::host::System;
use sx_embodiment_commission::matching::commission;
use sx_embodiment_commission::output::{
    Station, calibration, commission_report, commissioning, probe_json, probe_report, warnings,
};
use sx_embodiment_commission::probe::{Ids, probe};
use sx_embodiment_commission::tty::TtyPort;

const USAGE: &str = "usage:
  sx-commission probe [--json] [--feetech-ids 1-20] [--damiao-ids 1-32]
  sx-commission commission --bindings <file> [--bindings <file>...]
                           [--instance <id> --pod <id> --revision <n>] [--json] [--write <dir>]
  sx-commission calibrate ... (sx-commission calibrate --help)";

/// `calibrate`: every terminating signal a person or a supervisor sends sets `stop` instead of
/// killing the process, so the run undoes what it did before it exits.
fn calibrate(words: &[String]) -> ExitCode {
    use sx_embodiment_commission::calibrate::cli;
    if words.iter().any(|word| word == "--help") {
        println!("{}", cli::USAGE);
        return ExitCode::SUCCESS;
    }
    let args = match cli::parse(words) {
        Ok(args) => args,
        Err(error) => {
            eprintln!("{error}\n{}", cli::USAGE);
            return ExitCode::from(2);
        }
    };
    let stop = Arc::new(AtomicUsize::new(0));
    for signal in [
        signal_hook::consts::SIGINT,
        signal_hook::consts::SIGTERM,
        signal_hook::consts::SIGHUP,
        signal_hook::consts::SIGQUIT,
    ] {
        let number = usize::try_from(signal).unwrap_or(usize::MAX);
        if let Err(error) = signal_hook::flag::register_usize(signal, Arc::clone(&stop), number) {
            eprintln!("cannot catch signal {signal}: {error}; refusing to run");
            return ExitCode::from(2);
        }
    }
    let code = cli::run(
        &args,
        TtyPort::open,
        &stop,
        &mut std::io::stdout(),
        &mut std::io::stderr(),
    );
    ExitCode::from(code)
}

#[derive(Default)]
struct Args {
    command: String,
    json: bool,
    bindings: Vec<PathBuf>,
    instance: Option<String>,
    pod: Option<String>,
    revision: Option<u32>,
    write: Option<PathBuf>,
    feetech_ids: Option<Vec<u16>>,
    damiao_ids: Option<Vec<u16>>,
}

/// `1-20`, `9,16-21` or `0x09,0x10-0x15`.
fn ids(text: &str) -> Result<Vec<u16>, String> {
    let number = |part: &str| {
        let part = part.trim();
        part.strip_prefix("0x")
            .map_or_else(|| part.parse(), |hex| u16::from_str_radix(hex, 16))
            .map_err(|_| format!("{part} is not an id"))
    };
    let mut found = Vec::new();
    for part in text.split(',') {
        if let Some((low, high)) = part.split_once('-') {
            found.extend(number(low)?..=number(high)?);
        } else {
            found.push(number(part)?);
        }
    }
    Ok(found)
}

fn parse(mut words: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut args = Args {
        command: words.next().ok_or("no command")?,
        ..Args::default()
    };
    while let Some(flag) = words.next() {
        let mut value = || words.next().ok_or(format!("{flag} needs a value"));
        match flag.as_str() {
            "--json" => args.json = true,
            "--bindings" => args.bindings.push(PathBuf::from(value()?)),
            "--instance" => args.instance = Some(value()?),
            "--pod" => args.pod = Some(value()?),
            "--revision" => {
                args.revision = Some(value()?.parse().map_err(|_| "--revision is a number")?);
            }
            "--write" => args.write = Some(PathBuf::from(value()?)),
            "--feetech-ids" => args.feetech_ids = Some(ids(&value()?)?),
            "--damiao-ids" => args.damiao_ids = Some(ids(&value()?)?),
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(args)
}

fn scan_ids(args: &Args, body: Option<&DeclaredBody>) -> Ids {
    let mut ids = Ids::default();
    if let Some(feetech) = &args.feetech_ids {
        ids.feetech = feetech
            .iter()
            .filter_map(|id| u8::try_from(*id).ok())
            .collect();
    }
    if let Some(damiao) = &args.damiao_ids {
        ids.damiao.clone_from(damiao);
    }
    for port in body.iter().flat_map(|body| &body.ports) {
        for unit in &port.units {
            match port.bus {
                BusKind::Feetech => {
                    if let Ok(id) = u8::try_from(unit.bus_id)
                        && !ids.feetech.contains(&id)
                    {
                        ids.feetech.push(id);
                    }
                }
                BusKind::Damiao => {
                    if !ids.damiao.contains(&unit.bus_id) {
                        ids.damiao.push(unit.bus_id);
                    }
                }
            }
        }
    }
    ids
}

fn run(args: &Args) -> Result<bool, String> {
    let mut host = System::default();
    match args.command.as_str() {
        "probe" => {
            let found = probe(&mut host, &scan_ids(args, None)).map_err(|e| e.to_string())?;
            if args.json {
                println!("{:#}", probe_json(&found));
            } else {
                print!("{}", probe_report(&found));
            }
            Ok(true)
        }
        "commission" => run_commission(args, &mut host),
        other => Err(format!("unknown command {other}")),
    }
}

fn run_commission(args: &Args, host: &mut System) -> Result<bool, String> {
    if args.bindings.is_empty() {
        return Err("commission needs --bindings".to_owned());
    }
    let documents: Vec<String> = args
        .bindings
        .iter()
        .map(|path| std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display())))
        .collect::<Result<_, _>>()?;
    let documents: Vec<&str> = documents.iter().map(String::as_str).collect();
    let body = DeclaredBody::parse(&documents).map_err(|e| e.to_string())?;
    let station = match (&args.instance, &args.pod, args.revision) {
        (Some(instance), Some(pod_id), Some(pod_revision)) => Some(Station {
            instance: instance.clone(),
            pod_id: pod_id.clone(),
            pod_revision,
        }),
        (None, None, None) => None,
        _ => return Err("--instance, --pod and --revision go together".to_owned()),
    };
    if args.write.is_some() && station.is_none() {
        return Err("--write needs --instance, --pod and --revision".to_owned());
    }
    let found = probe(host, &scan_ids(args, Some(&body))).map_err(|e| e.to_string())?;
    let result = commission(&body, &found);
    let matched = result.as_ref().ok();
    let document = station
        .as_ref()
        .zip(matched)
        .map(|(station, matched)| commissioning(station, matched));
    let calibrations: Vec<(String, serde_json::Value)> = matched
        .into_iter()
        .flatten()
        .filter_map(|port| calibration(port).map(|value| (port.port.clone(), value)))
        .collect();
    let notes = matched
        .map(|matched| warnings(&found, matched))
        .unwrap_or_default();
    if args.json {
        println!(
            "{:#}",
            json!({
                "embodiment": body.embodiment,
                "probe": probe_json(&found),
                "matched": matched.map(|matched| matched.iter().map(|port| json!({
                    "port": port.port, "identity": port.identity, "channel": port.channel,
                })).collect::<Vec<_>>()),
                "refused": result.as_ref().err().map(|errors| errors.iter().map(ToString::to_string).collect::<Vec<_>>()),
                "commissioning": document,
                "calibration": calibrations.iter().cloned().collect::<serde_json::Map<_, _>>(),
                "warnings": notes,
            })
        );
    } else {
        print!("{}", probe_report(&found));
        println!("\nCommissioning {}:", body.embodiment);
        print!("{}", commission_report(&result));
        for note in &notes {
            println!("  warning: {note}");
        }
        if let Some(document) = &document {
            println!("\ncommissioning.json:\n{document:#}");
        } else if matched.is_some() {
            println!("\n(pass --instance, --pod and --revision for the commissioning document)");
        }
        for (port, value) in &calibrations {
            println!("\n{port}.calibration.json:\n{value:#}");
        }
    }
    if let (Some(dir), Some(document)) = (&args.write, &document) {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        let write = |name: String, value: &serde_json::Value| {
            let path = dir.join(name);
            std::fs::write(&path, format!("{value:#}\n"))
                .map_err(|e| format!("{}: {e}", path.display()))?;
            eprintln!("wrote {}", path.display());
            Ok::<(), String>(())
        };
        write("commissioning.json".to_owned(), document)?;
        for (port, value) in &calibrations {
            write(format!("{port}.calibration.json"), value)?;
        }
    } else if args.write.is_none() && matched.is_some() {
        eprintln!("dry run: nothing written (pass --write <dir>)");
    }
    Ok(result.is_ok())
}

fn main() -> ExitCode {
    let words: Vec<String> = std::env::args().skip(1).collect();
    if words.first().is_some_and(|word| word == "calibrate") {
        return calibrate(&words[1..]);
    }
    let args = match parse(words.into_iter()) {
        Ok(args) => args,
        Err(error) => {
            eprintln!("{error}\n{USAGE}");
            return ExitCode::from(2);
        }
    };
    match run(&args) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::from(2)
        }
    }
}
