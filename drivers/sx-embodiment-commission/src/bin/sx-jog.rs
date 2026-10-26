//! `sx-jog`: move one joint of the static upper body's arms by a relative amount, supervised.
//!
//! ```text
//! sx-jog --joint left_elbow_pitch_joint --delta-deg 10 [--speed 0.2] [--hold-seconds 0]
//!        [--zlg-serial <serial>] [--channel 1]
//! ```
//!
//! Relative to where the joint is, so it needs no zero calibration. The 14 working Damiao arm
//! motors are opened through `DamiaoChain` (each motor's CAN timeout written at open, so a dead
//! process leaves them disabled within 0.75 s), wrapped in the `Governor` (speed limit, tracking
//! error stop). Every other arm joint is held where it is while the named one moves. At the end the
//! chain is stopped (motors disabled) after an optional hold; Ctrl-C stops it at once.

use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use sx_damiao_can::chain::DamiaoChain;
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_drivers::governor::{AxisLimits, Governor};
use sx_embodiment_drivers::{ActuatorChain, InstanceDrive};
use sx_zlg_usbcanfd::{ChannelOptions, UsbCanFd};

const DRIVES: &str = include_str!("../../../sx-damiao-can/tests/fixtures/drives.json");
const PERIOD: Duration = Duration::from_millis(20);

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("sx-jog: {error}");
            ExitCode::FAILURE
        }
    }
}

// A bench tool: one flat argument dispatch whose modes each name the joint they act on `axis`
// beside the chain's `axes`.
#[allow(clippy::too_many_lines, clippy::similar_names)]
fn run() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let (mut joint, mut delta_deg, mut speed, mut hold) =
        (String::new(), 0.0_f64, 0.2_f64, 0.0_f64);
    let (mut serial, mut channel, mut scan_only, mut disable_all) =
        (None::<String>, 1_u8, false, false);
    let mut listen_only = false;
    let mut release_test = false;
    let mut clear_faults = false;
    let mut sequence = String::new();
    let mut slew_to: Option<f64> = None;
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} needs a value"));
        match arg.as_str() {
            "--joint" => joint = value()?,
            "--delta-deg" => delta_deg = value()?.parse().map_err(|_| "bad --delta-deg")?,
            "--speed" => speed = value()?.parse().map_err(|_| "bad --speed")?,
            "--hold-seconds" => hold = value()?.parse().map_err(|_| "bad --hold-seconds")?,
            "--zlg-serial" => serial = Some(value()?),
            "--channel" => channel = value()?.parse().map_err(|_| "bad --channel")?,
            "--scan" => scan_only = true,
            "--disable-all" => disable_all = true,
            "--listen" => {
                listen_only = true;
            }
            "--release-test" => {
                release_test = true;
            }
            "--clear-faults" => {
                clear_faults = true;
            }
            "--sequence" => {
                sequence = value()?;
            }
            "--slew-to" => {
                slew_to = Some(value()?.parse::<f64>().map_err(|_| "bad --slew-to")?);
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    if !(0.0..=0.5).contains(&speed) {
        return Err("--speed is capped at 0.5 rad/s".into());
    }
    if delta_deg.abs() > 45.0 {
        return Err("--delta-deg is capped at 45 degrees per jog".into());
    }
    let drives: serde_json::Value = serde_json::from_str(DRIVES).map_err(|e| e.to_string())?;
    let nominal = drives["sentient-static-upper-body"]["nominal"].to_string();
    let instance = drives["sentient-static-upper-body"]["nominal"]["instance"]
        .as_str()
        .ok_or("drives.json has no instance")?;
    let drive = InstanceDrive::parse(&nominal, instance).map_err(|e| e.to_string())?;
    // The torso motor (0x1F) is known faulty and stays out of the chain.
    let axes: Vec<_> = STATIC_UPPER_BODY
        .axes(&drive)
        .map_err(|e| e.to_string())?
        .into_iter()
        .filter(|axis| axis.axis.joint != "torso_joint")
        .collect();
    if clear_faults {
        // Damiao's clear-error frame (FF FF FF FF FF FF FF FB to the motor's id), then disable,
        // then read each motor's state back.
        use sx_embodiment_drivers::can::{CanFrame, CanPort};
        let adapter = UsbCanFd::open(serial.as_deref()).map_err(|e| e.to_string())?;
        let mut port = adapter
            .open_channel(channel, ChannelOptions::default())
            .map_err(|e| e.to_string())?;
        for axis in &axes {
            let id = u32::from(axis.axis.binding.bus_id);
            for cmd in [0xFB_u8, 0xFD_u8] {
                port.send(&CanFrame {
                    id,
                    data: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd],
                })
                .map_err(|e| e.to_string())?;
                thread::sleep(Duration::from_millis(5));
            }
        }
        thread::sleep(Duration::from_millis(50));
        while port.receive().map_err(|e| e.to_string())?.is_some() {}
        let mut control = sx_damiao_can::dm_can::MotorControl::new(port);
        for axis in &axes {
            let model = sx_damiao_can::chain::motor_type(axis.axis.binding.model).ok_or("model")?;
            control.add_motor(sx_damiao_can::dm_can::Motor::new(
                model,
                axis.axis.binding.bus_id,
                axis.master_id,
            ));
        }
        // Back to no communication timeout (the motors' state before today's open; RAM only).
        for axis in &axes {
            let id = axis.axis.binding.bus_id;
            let ok = control
                .change_motor_param(id, sx_damiao_can::dm_can::DmVariable::TIMEOUT, 0.0)
                .map_err(|e| e.to_string())?;
            println!("{:28} TIMEOUT -> 0: {ok}", axis.axis.joint);
        }
        for axis in &axes {
            let id = u32::from(axis.axis.binding.bus_id);
            control
                .port_mut()
                .send(&CanFrame {
                    id,
                    data: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB],
                })
                .map_err(|e| e.to_string())?;
            thread::sleep(Duration::from_millis(5));
        }
        thread::sleep(Duration::from_millis(30));
        for axis in &axes {
            control
                .refresh_motor_status(axis.axis.binding.bus_id)
                .map_err(|e| e.to_string())?;
        }
        thread::sleep(Duration::from_millis(30));
        control.recv().map_err(|e| e.to_string())?;
        for axis in &axes {
            let m = control
                .motor(axis.axis.binding.bus_id)
                .map_err(|e| e.to_string())?;
            println!(
                "{:28} id {:#04x} state {:#x} position {:+.3}",
                axis.axis.joint,
                axis.axis.binding.bus_id,
                m.get_error(),
                m.get_position()
            );
        }
        return Ok(());
    }
    if let Some(goal) = slew_to {
        // One motor: enable, low-stiffness MIT hold that slews from where it is to `goal` (rad)
        // at `speed`, hold 0.5 s, disable. Used to bring a joint back into range.
        let axis = axes
            .iter()
            .find(|a| a.axis.joint == joint)
            .ok_or("unknown joint")?;
        if goal < axis.axis.lower || goal > axis.axis.upper {
            return Err(format!(
                "goal {goal} outside {}..{}",
                axis.axis.lower, axis.axis.upper
            ));
        }
        let id = axis.axis.binding.bus_id;
        let adapter = UsbCanFd::open(serial.as_deref()).map_err(|e| e.to_string())?;
        let port = adapter
            .open_channel(channel, ChannelOptions::default())
            .map_err(|e| e.to_string())?;
        let mut control = sx_damiao_can::dm_can::MotorControl::new(port);
        let model = sx_damiao_can::chain::motor_type(axis.axis.binding.model).ok_or("model")?;
        control.add_motor(sx_damiao_can::dm_can::Motor::new(model, id, axis.master_id));
        control
            .refresh_motor_status(id)
            .map_err(|e| e.to_string())?;
        thread::sleep(Duration::from_millis(20));
        control.recv().map_err(|e| e.to_string())?;
        let start = f64::from(control.motor(id).map_err(|e| e.to_string())?.get_position());
        println!("{joint}: from {start:+.3} to {goal:+.3} rad at {speed} rad/s");
        let stop = Arc::new(AtomicBool::new(false));
        {
            let stop = Arc::clone(&stop);
            let _ = ctrlc::set_handler(move || stop.store(true, Ordering::SeqCst));
        }
        control.enable(id).map_err(|e| e.to_string())?;
        let duration = (goal - start).abs() / speed.max(0.05);
        let t0 = Instant::now();
        let mut printed = Instant::now();
        let mut result = Ok(());
        while t0.elapsed().as_secs_f64() < duration + 0.5 && !stop.load(Ordering::SeqCst) {
            let f = (t0.elapsed().as_secs_f64() / duration.max(1e-3)).min(1.0);
            let target = start + (goal - start) * f;
            if let Err(e) = control.control_mit(id, 8.0, 0.5, target, 0.0, 0.0) {
                result = Err(e.to_string());
                break;
            }
            let m = control.motor(id).map_err(|e| e.to_string())?;
            if (f64::from(m.get_position()) - target).abs() > 0.4 {
                result = Err(format!(
                    "tracking error at {:+.3} vs {target:+.3}",
                    m.get_position()
                ));
                break;
            }
            if printed.elapsed() > Duration::from_millis(500) {
                println!(
                    "  target {target:+.3}  at {:+.3}  torque {:+.2}",
                    m.get_position(),
                    m.get_torque()
                );
                printed = Instant::now();
            }
            thread::sleep(Duration::from_millis(10));
        }
        for _ in 0..3 {
            let _ = control.disable(id);
            thread::sleep(Duration::from_millis(10));
        }
        control
            .refresh_motor_status(id)
            .map_err(|e| e.to_string())?;
        thread::sleep(Duration::from_millis(20));
        control.recv().map_err(|e| e.to_string())?;
        println!(
            "released at {:+.3} rad",
            control.motor(id).map_err(|e| e.to_string())?.get_position()
        );
        return result;
    }
    if release_test {
        // One motor: enable, hold where it is at kp 5 for 5 s, disable; print its state each step.
        let axis = axes
            .iter()
            .find(|a| a.axis.joint == joint)
            .ok_or("unknown joint")?;
        let id = axis.axis.binding.bus_id;
        let adapter = UsbCanFd::open(serial.as_deref()).map_err(|e| e.to_string())?;
        let port = adapter
            .open_channel(channel, ChannelOptions::default())
            .map_err(|e| e.to_string())?;
        let mut control = sx_damiao_can::dm_can::MotorControl::new(port);
        let model = sx_damiao_can::chain::motor_type(axis.axis.binding.model).ok_or("model")?;
        control.add_motor(sx_damiao_can::dm_can::Motor::new(model, id, axis.master_id));
        let state =
            |c: &mut sx_damiao_can::dm_can::MotorControl<_>, label: &str| -> Result<(), String> {
                c.refresh_motor_status(id).map_err(|e| e.to_string())?;
                thread::sleep(Duration::from_millis(20));
                c.recv().map_err(|e| e.to_string())?;
                let m = c.motor(id).map_err(|e| e.to_string())?;
                println!(
                    "{label:16} state {:#x} position {:+.3} torque {:+.3}",
                    m.get_error(),
                    m.get_position(),
                    m.get_torque()
                );
                Ok(())
            };
        state(&mut control, "before")?;
        let q = control.motor(id).map_err(|e| e.to_string())?.get_position();
        control.enable(id).map_err(|e| e.to_string())?;
        thread::sleep(Duration::from_millis(20));
        state(&mut control, "after enable")?;
        let hold = Instant::now();
        let mut printed = Instant::now();
        while hold.elapsed() < Duration::from_secs(15) {
            control
                .control_mit(id, 5.0, 0.3, f64::from(q), 0.0, 0.0)
                .map_err(|e| e.to_string())?;
            if printed.elapsed() > Duration::from_millis(500) {
                let m = control.motor(id).map_err(|e| e.to_string())?;
                println!(
                    "holding          state {:#x} position {:+.3} torque {:+.3}",
                    m.get_error(),
                    m.get_position(),
                    m.get_torque()
                );
                printed = Instant::now();
            }
            thread::sleep(Duration::from_millis(10));
        }
        for _ in 0..3 {
            control.disable(id).map_err(|e| e.to_string())?;
            thread::sleep(Duration::from_millis(10));
        }
        state(&mut control, "after disable")?;
        thread::sleep(Duration::from_millis(500));
        state(&mut control, "0.5 s later")?;
        return Ok(());
    }
    if listen_only {
        use sx_embodiment_drivers::can::CanPort;
        let adapter = UsbCanFd::open(serial.as_deref()).map_err(|e| e.to_string())?;
        let mut port = adapter
            .open_channel(channel, ChannelOptions::default())
            .map_err(|e| e.to_string())?;
        let mut seen = std::collections::BTreeMap::<u32, (usize, [u8; 8])>::new();
        let end = Instant::now() + Duration::from_secs(2);
        while Instant::now() < end {
            while let Some(frame) = port.receive().map_err(|e| e.to_string())? {
                let entry = seen.entry(frame.id).or_insert((0, frame.data));
                entry.0 += 1;
                entry.1 = frame.data;
            }
            thread::sleep(Duration::from_millis(1));
        }
        println!("frames heard in 2 s without sending anything:");
        for (id, (count, data)) in &seen {
            println!("  id {id:#05x}  {count:5} frames  last {data:02x?}");
        }
        if seen.is_empty() {
            println!("  none: nobody else is talking on this bus");
        }
        return Ok(());
    }
    if disable_all {
        let adapter = UsbCanFd::open(serial.as_deref()).map_err(|e| e.to_string())?;
        let port = adapter
            .open_channel(channel, ChannelOptions::default())
            .map_err(|e| e.to_string())?;
        let mut control = sx_damiao_can::dm_can::MotorControl::new(port);
        for axis in &axes {
            let id = axis.axis.binding.bus_id;
            let model = sx_damiao_can::chain::motor_type(axis.axis.binding.model).ok_or("model")?;
            control.add_motor(sx_damiao_can::dm_can::Motor::new(model, id, axis.master_id));
        }
        for _ in 0..3 {
            for axis in &axes {
                control
                    .disable(axis.axis.binding.bus_id)
                    .map_err(|e| e.to_string())?;
            }
            thread::sleep(Duration::from_millis(20));
        }
        for axis in &axes {
            control
                .refresh_motor_status(axis.axis.binding.bus_id)
                .map_err(|e| e.to_string())?;
        }
        thread::sleep(Duration::from_millis(30));
        control.recv().map_err(|e| e.to_string())?;
        for axis in &axes {
            let motor = control
                .motor(axis.axis.binding.bus_id)
                .map_err(|e| e.to_string())?;
            println!(
                "{:28} id {:#04x} state {:#x} position {:+.3} feedback {}",
                axis.axis.joint,
                axis.axis.binding.bus_id,
                motor.get_error(),
                motor.get_position(),
                motor.feedback_count
            );
        }
        return Ok(());
    }
    if scan_only {
        let adapter = UsbCanFd::open(serial.as_deref()).map_err(|e| e.to_string())?;
        let port = adapter
            .open_channel(channel, ChannelOptions::default())
            .map_err(|e| e.to_string())?;
        let ids: Vec<u16> = axes.iter().map(|a| a.axis.binding.bus_id).collect();
        let scan = sx_damiao_can::scan::scan(port, ids).map_err(|e| e.to_string())?;
        for (motor, axis) in scan.motors.iter().zip(&axes) {
            println!(
                "{:28} id {:#04x} master {:#04x} mode {:?} PMAX {:?} VMAX {:?} TMAX {:?}",
                axis.axis.joint,
                motor.slave_id,
                motor.master_id,
                motor.control_mode,
                motor.p_max,
                motor.v_max,
                motor.t_max
            );
        }
        println!("silent: {:?}", scan.silent);
        return Ok(());
    }
    let index = if sequence.is_empty() {
        axes.iter()
            .position(|axis| axis.axis.joint == joint)
            .ok_or(format!("{joint} is not a working arm joint"))?
    } else {
        0
    };
    let adapter = UsbCanFd::open(serial.as_deref()).map_err(|e| e.to_string())?;
    let port = adapter
        .open_channel(channel, ChannelOptions::default())
        .map_err(|e| e.to_string())?;
    let chain = DamiaoChain::open(port, axes).map_err(|e| e.to_string())?;
    let width = chain.axes().len();
    let mut governor = Governor::new(
        chain,
        vec![
            AxisLimits {
                max_speed: speed,
                max_tracking_error: 0.2,
            };
            width
        ],
    )
    .map_err(|e| format!("{e:?}"))?;
    let stop = Arc::new(AtomicBool::new(false));
    {
        let stop = Arc::clone(&stop);
        let _ = ctrlc::set_handler(move || stop.store(true, Ordering::SeqCst));
    }
    let start = governor
        .chain()
        .last_observed()
        .ok_or("the chain reports no rest position")?;
    if !sequence.is_empty() {
        // "joint:deg,joint:deg,...": each step moves one joint by deg relative to where the
        // sequence has put it, holding every joint enabled throughout; "pause:seconds" holds.
        let names: Vec<String> = governor
            .chain()
            .axes()
            .iter()
            .map(|a| a.joint.clone())
            .collect();
        let mut target = start.clone();
        let mut outcome = Ok(());
        'steps: for step in sequence.split(',') {
            let (name, amount) = step.split_once(':').ok_or(format!("bad step {step}"))?;
            let amount: f64 = amount.parse().map_err(|_| format!("bad step {step}"))?;
            let seconds = if name == "pause" {
                amount
            } else {
                let i = names
                    .iter()
                    .position(|n| n == name)
                    .ok_or(format!("unknown joint {name}"))?;
                target[i] += amount.to_radians();
                eprintln!("{name}: to {:.1} deg", target[i].to_degrees());
                amount.to_radians().abs() / speed + 0.4
            };
            let begin = Instant::now();
            let mut next = Instant::now();
            while begin.elapsed().as_secs_f64() < seconds {
                if stop.load(Ordering::SeqCst) {
                    break 'steps;
                }
                if let Err(error) = governor.command(&target) {
                    outcome = Err(format!("command refused or failed: {error:?}"));
                    break 'steps;
                }
                next += PERIOD;
                if let Some(wait) = next.checked_duration_since(Instant::now()) {
                    thread::sleep(wait);
                } else {
                    next = Instant::now();
                }
            }
        }
        let report = governor.stop();
        eprintln!(
            "stopped: unproven {:?} faults {:?} answered disable, enable state unobservable {:?}",
            report.unproven, report.faults, report.unobserved
        );
        return outcome;
    }
    let mut target = start.clone();
    target[index] = start[index] + delta_deg.to_radians();
    eprintln!(
        "{joint}: at {:.1} deg, moving to {:.1} deg at {speed} rad/s",
        start[index].to_degrees(),
        target[index].to_degrees()
    );
    let travel =
        Duration::from_secs_f64(delta_deg.to_radians().abs() / speed.max(1e-3) + 1.0 + hold);
    let begin = Instant::now();
    let mut next = Instant::now();
    let mut last_print = Instant::now();
    let mut outcome = Ok(());
    while begin.elapsed() < travel && !stop.load(Ordering::SeqCst) {
        match governor.command(&target) {
            Ok(receipt) => {
                if last_print.elapsed() > Duration::from_millis(250) {
                    if let Some(q) = receipt.observed.get(index) {
                        eprintln!(
                            "  {:5.2} s  {joint} {:7.1} deg",
                            begin.elapsed().as_secs_f64(),
                            q.to_degrees()
                        );
                    }
                    last_print = Instant::now();
                }
            }
            Err(error) => {
                outcome = Err(format!("command refused or failed: {error:?}"));
                break;
            }
        }
        next += PERIOD;
        if let Some(wait) = next.checked_duration_since(Instant::now()) {
            thread::sleep(wait);
        } else {
            next = Instant::now();
        }
    }
    let report = governor.stop();
    eprintln!(
        "stopped: unproven {:?} faults {:?} answered disable, enable state unobservable {:?}",
        report.unproven, report.faults, report.unobserved
    );
    outcome
}
