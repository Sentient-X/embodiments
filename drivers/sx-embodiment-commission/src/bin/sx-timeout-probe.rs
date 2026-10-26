//! `sx-timeout-probe`: measure one Damiao motor's CAN watchdog on the bench.
//!
//! ```text
//! sx-timeout-probe --joint left_elbow_pitch_joint --counts 15000 --silence-ms 500,1000
//!                  [--while-disabled] [--zlg-serial <serial>] [--channel 1]
//! ```
//!
//! For each silence: disable, clear any fault, write TIMEOUT `counts` (RAM) and read it back,
//! enable and hold where the motor is at kp 5 for 0.3 s (no motion), then send nothing for the
//! silence, then ask the motor's state: fault 0xD means the watchdog tripped within the silence.
//! The motor is disabled and its fault cleared after every trial, and TIMEOUT is put back to
//! the value it held before.

use std::process::ExitCode;
use std::thread;
use std::time::{Duration, Instant};

use sx_damiao_can::dm_can::{DmVariable, Motor, MotorControl};
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_drivers::InstanceDrive;
use sx_zlg_usbcanfd::{ChannelOptions, UsbCanFd};

const DRIVES: &str = include_str!("../../../sx-damiao-can/tests/fixtures/drives.json");

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("sx-timeout-probe: {error}");
            ExitCode::FAILURE
        }
    }
}

// A bench probe: one flat sequence of trials, its joint named `axis` beside the body's `axes`.
#[allow(clippy::too_many_lines, clippy::similar_names)]
fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let (mut joint, mut counts, mut silences) = (String::new(), 15000.0_f64, vec![500_u64]);
    let (mut serial, mut channel, mut disabled) = (None::<String>, 1_u8, false);
    while let Some(flag) = args.next() {
        let mut value = || args.next().ok_or(format!("{flag} takes a value"));
        match flag.as_str() {
            "--joint" => joint = value()?,
            "--counts" => counts = value()?.parse().map_err(|_| "bad --counts")?,
            "--silence-ms" => {
                silences = value()?
                    .split(',')
                    .map(|s| s.parse().map_err(|_| "bad --silence-ms"))
                    .collect::<Result<_, _>>()?;
            }
            "--while-disabled" => disabled = true,
            "--zlg-serial" => serial = Some(value()?),
            "--channel" => channel = value()?.parse().map_err(|_| "bad --channel")?,
            other => return Err(format!("unknown argument {other}")),
        }
    }
    let drives: serde_json::Value = serde_json::from_str(DRIVES).map_err(|e| e.to_string())?;
    let nominal = drives["sentient-static-upper-body"]["nominal"].to_string();
    let instance = drives["sentient-static-upper-body"]["nominal"]["instance"]
        .as_str()
        .ok_or("drives.json has no instance")?;
    let drive = InstanceDrive::parse(&nominal, instance).map_err(|e| e.to_string())?;
    let axes = STATIC_UPPER_BODY.axes(&drive).map_err(|e| e.to_string())?;
    let axis = axes
        .iter()
        .find(|a| a.axis.joint == joint)
        .ok_or("unknown joint")?;
    let id = axis.axis.binding.bus_id;
    let model = sx_damiao_can::chain::motor_type(axis.axis.binding.model).ok_or("model")?;
    let adapter = UsbCanFd::open(serial.as_deref()).map_err(|e| e.to_string())?;
    let port = adapter
        .open_channel(channel, ChannelOptions::default())
        .map_err(|e| e.to_string())?;
    let mut c = MotorControl::new(port);
    c.add_motor(Motor::new(model, id, axis.master_id));
    let state = |c: &mut MotorControl<_>| -> Result<(u8, f32), String> {
        c.refresh_motor_status(id).map_err(|e| e.to_string())?;
        thread::sleep(Duration::from_millis(20));
        c.recv().map_err(|e| e.to_string())?;
        let m = c.motor(id).map_err(|e| e.to_string())?;
        Ok((m.get_error(), m.get_position()))
    };
    let flash = c
        .read_motor_param(id, DmVariable::TIMEOUT)
        .map_err(|e| e.to_string())?;
    println!("{joint} (0x{id:02X}): TIMEOUT before {flash:?}");
    for silence in silences {
        c.disable(id).map_err(|e| e.to_string())?;
        c.clear_error(id).map_err(|e| e.to_string())?;
        thread::sleep(Duration::from_millis(30));
        let ok = c
            .change_motor_param(id, DmVariable::TIMEOUT, counts)
            .map_err(|e| e.to_string())?;
        let back = c
            .read_motor_param(id, DmVariable::TIMEOUT)
            .map_err(|e| e.to_string())?;
        let (before, q) = state(&mut c)?;
        if disabled {
            // Never enabled: does the watchdog count while the motor is disabled?
            thread::sleep(Duration::from_millis(silence));
            let (after, _) = state(&mut c)?;
            c.enable(id).map_err(|e| e.to_string())?;
            let (enabled, _) = state(&mut c)?;
            c.disable(id).map_err(|e| e.to_string())?;
            c.clear_error(id).map_err(|e| e.to_string())?;
            println!(
                "counts {counts} readback {back:?} | disabled {silence} ms: state {after:#x}, then enable {enabled:#x}{}",
                if enabled == 0xD {
                    "  FAULTED AT ENABLE"
                } else {
                    ""
                }
            );
            continue;
        }
        c.enable(id).map_err(|e| e.to_string())?;
        let (at_enable, _) = state(&mut c)?;
        let hold = Instant::now();
        while hold.elapsed() < Duration::from_millis(300) {
            c.control_mit(id, 5.0, 0.3, f64::from(q), 0.0, 0.0)
                .map_err(|e| e.to_string())?;
            thread::sleep(Duration::from_millis(10));
        }
        let (holding, _) = state(&mut c)?;
        thread::sleep(Duration::from_millis(silence));
        let (after, _) = state(&mut c)?;
        c.disable(id).map_err(|e| e.to_string())?;
        c.clear_error(id).map_err(|e| e.to_string())?;
        println!(
            "counts {counts} write {ok} readback {back:?} | state before {before:#x} enable {at_enable:#x} holding {holding:#x} after {silence} ms silence {after:#x}{}",
            if after == 0xD { "  TRIPPED" } else { "" }
        );
    }
    let restore = match flash {
        Some(sx_damiao_can::dm_can::ParamValue::Uint32(v)) => f64::from(v),
        _ => 0.0,
    };
    c.change_motor_param(id, DmVariable::TIMEOUT, restore)
        .map_err(|e| e.to_string())?;
    c.disable(id).map_err(|e| e.to_string())?;
    println!("{joint}: TIMEOUT back to {restore}, disabled");
    Ok(())
}
