//! What commissioning emits: the station's commissioning document, each Feetech port's
//! calibration, and the reports a person and a machine read.
//!
//! The commissioning document is `sx.station-commissioning/v1`, the one
//! `sx_embodiment_bodies::Commissioning::parse` reads. A Feetech port's calibration is
//! `LeRobot`'s calibration file (`MotorsBus.write_calibration`'s JSON: each motor by name, its
//! `id`, `drive_mode`, `homing_offset`, `range_min` and `range_max`), keyed by the body's
//! joint names, which is what the station hands its Feetech chain as `MotorCalibration`s.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use serde_json::{Value, json};
use sx_embodiment_drivers::rail::Rails;
use sx_feetech_serial::scan::ServoScan;

use crate::bridges::{Access, Bridge};
use crate::matching::{PortError, PortMatch};
use crate::probe::Probe;

/// The schema `sx-embodiment-bodies` parses (`COMMISSIONING_SCHEMA`).
pub const COMMISSIONING_SCHEMA: &str = "sx.station-commissioning/v1";

/// Who the commissioned robot is: its instance and the Fleet pod revision recording it, and
/// each power rail's voltage as measured on it.
#[derive(Clone, Debug)]
pub struct Station {
    pub instance: String,
    pub pod_id: String,
    pub pod_revision: u32,
    /// Measured volts by selector (`*`, a port, a model, `<port>/<model>`): an axis over its
    /// model's supported supply opens derated, or refused far over it.
    pub rails: Rails,
}

/// The commissioning document for `matched`.
#[must_use]
pub fn commissioning(station: &Station, matched: &[PortMatch]) -> Value {
    let ports: BTreeMap<&str, &str> = matched
        .iter()
        .map(|port| (port.port.as_str(), port.identity.as_str()))
        .collect();
    let mut document = json!({
        "schema": COMMISSIONING_SCHEMA,
        "instance": station.instance,
        "pod_id": station.pod_id,
        "pod_revision": station.pod_revision,
        "ports": ports,
    });
    if !station.rails.volts().is_empty() {
        document["rails"] = json!(station.rails.volts());
    }
    if let Some(decided) = station.rails.derate_decided() {
        document["derate"] = json!({
            "max_speed": station.rails.derate().max_speed,
            "decided": decided,
        });
    }
    document
}

/// A Feetech port's units as `LeRobot`'s calibration file; `None` for a Damiao port.
#[must_use]
pub fn calibration(port: &PortMatch) -> Option<Value> {
    if port.servos.is_empty() {
        return None;
    }
    let motors: serde_json::Map<String, Value> = port
        .servos
        .iter()
        .map(|(joint, servo)| {
            let calibration = servo.calibration;
            (
                joint.clone(),
                json!({
                    "id": servo.id,
                    "drive_mode": u8::from(calibration.drive_mode),
                    "homing_offset": calibration.homing_offset,
                    "range_min": calibration.range_min,
                    "range_max": calibration.range_max,
                }),
            )
        })
        .collect();
    Some(Value::Object(motors))
}

/// Identities `sx-embodiment-bodies` cannot open by, as found: a USB serial carried by several
/// CAN channels (the ZLG's two) resolves to every one of them and is refused as ambiguous.
#[must_use]
pub fn warnings(probe: &Probe, matched: &[PortMatch]) -> Vec<String> {
    matched
        .iter()
        .filter_map(|port| {
            let channels: Vec<&str> = probe
                .damiao
                .iter()
                .filter(|bus| bus.identity == port.identity)
                .map(|bus| bus.interface.as_str())
                .collect();
            (channels.len() > 1).then(|| {
                format!(
                    "port {}: identity {} carries {} CAN interfaces ({}); the units answered \
                     on {}, but a station opening by this identity refuses it as ambiguous \
                     until a channel can be named",
                    port.port,
                    port.identity,
                    channels.len(),
                    channels.join(", "),
                    port.channel
                )
            })
        })
        .collect()
}

fn servo_json(servo: &ServoScan) -> Value {
    json!({
        "id": servo.id,
        "model": servo.model,
        "firmware": format!("{}.{}", servo.firmware_major, servo.firmware_minor),
        "range_min": servo.calibration.range_min,
        "range_max": servo.calibration.range_max,
        "homing_offset": servo.calibration.homing_offset,
        "operating_mode": servo.operating_mode,
        "p_coefficient": servo.p_coefficient,
        "d_coefficient": servo.d_coefficient,
        "i_coefficient": servo.i_coefficient,
        "max_torque_limit": servo.max_torque_limit,
        "torque_enabled": servo.torque_enabled,
        "present_position": servo.present_position,
        "voltage": f64::from(servo.voltage_decivolts) / 10.0,
        "temperature_celsius": servo.temperature_celsius,
        "status": servo.status,
    })
}

/// The probe as JSON.
#[must_use]
pub fn probe_json(probe: &Probe) -> Value {
    let feetech: Vec<Value> = probe
        .feetech
        .iter()
        .map(|bus| match &bus.scan {
            Ok(scan) => json!({
                "identity": bus.identity,
                "tty": bus.tty,
                "servos": scan.servos.iter().map(servo_json).collect::<Vec<_>>(),
                "failed": scan.failed.iter().map(|failure| json!({
                    "id": failure.id,
                    "address": failure.address,
                    "result": failure.result.code(),
                })).collect::<Vec<_>>(),
            }),
            Err(error) => json!({"identity": bus.identity, "tty": bus.tty, "error": error}),
        })
        .collect();
    let damiao: Vec<Value> = probe
        .damiao
        .iter()
        .map(|bus| match &bus.scan {
            Ok(scan) => json!({
                "identity": bus.identity,
                "interface": bus.interface,
                "motors": scan.motors.iter().map(|motor| json!({
                    "id": motor.slave_id,
                    "reply_id": motor.reply_id,
                    "master_id": motor.master_id,
                    "control_mode": motor.control_mode,
                    "p_max": motor.p_max,
                    "v_max": motor.v_max,
                    "t_max": motor.t_max,
                })).collect::<Vec<_>>(),
            }),
            Err(error) => {
                json!({"identity": bus.identity, "interface": bus.interface, "error": error})
            }
        })
        .collect();
    json!({"bridges": probe.bridges, "feetech": feetech, "damiao": damiao})
}

fn access(bridge: &Bridge) -> String {
    match &bridge.access {
        Access::Tty { path } => path.clone(),
        Access::SocketCan { interfaces } => format!("SocketCAN {}", interfaces.join(", ")),
        Access::NoDriver { reason } => format!("present, unusable here: {reason}"),
        Access::None => String::new(),
    }
}

/// The probe as a person reads it.
#[must_use]
pub fn probe_report(probe: &Probe) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "Bridges ({}):", probe.bridges.len());
    for bridge in &probe.bridges {
        let usb = &bridge.usb;
        let _ = writeln!(
            out,
            "  {:04x}:{:04x}  {:<11} {:<32} serial {:<22} {}",
            usb.vendor_id,
            usb.product_id,
            format!("{:?}", bridge.kind),
            bridge.name,
            usb.serial.as_deref().unwrap_or("-"),
            access(bridge)
        );
    }
    for bus in &probe.feetech {
        let _ = writeln!(out, "\nFeetech bus {} on {}:", bus.identity, bus.tty);
        match &bus.scan {
            Err(error) => {
                let _ = writeln!(out, "  could not scan: {error}");
            }
            Ok(scan) if scan.servos.is_empty() && scan.failed.is_empty() => {
                let _ = writeln!(out, "  nothing answered");
            }
            Ok(scan) => {
                for servo in &scan.servos {
                    let calibration = servo.calibration;
                    let _ = writeln!(
                        out,
                        "  id {:>2}  model {}  fw {}.{}  range {}..{}  offset {:>5}  mode {}  \
                         PDI {}/{}/{}  max_torque {}  torque {}  position {}  {:.1} V  {} C{}",
                        servo.id,
                        servo.model,
                        servo.firmware_major,
                        servo.firmware_minor,
                        calibration.range_min,
                        calibration.range_max,
                        calibration.homing_offset,
                        servo.operating_mode,
                        servo.p_coefficient,
                        servo.d_coefficient,
                        servo.i_coefficient,
                        servo.max_torque_limit,
                        if servo.torque_enabled { "on" } else { "off" },
                        servo.present_position,
                        f64::from(servo.voltage_decivolts) / 10.0,
                        servo.temperature_celsius,
                        if servo.status == 0 {
                            String::new()
                        } else {
                            format!("  status {:#04x}", servo.status)
                        }
                    );
                }
                for failure in &scan.failed {
                    let _ = writeln!(
                        out,
                        "  id {:>2}  answered its ping, then failed reading address {} ({:?})",
                        failure.id, failure.address, failure.result
                    );
                }
            }
        }
    }
    for bus in &probe.damiao {
        let _ = writeln!(out, "\nDamiao bus {} on {}:", bus.identity, bus.interface);
        match &bus.scan {
            Err(error) => {
                let _ = writeln!(out, "  could not scan: {error}");
            }
            Ok(scan) if scan.motors.is_empty() => {
                let _ = writeln!(out, "  nothing answered");
            }
            Ok(scan) => {
                for motor in &scan.motors {
                    let _ = writeln!(
                        out,
                        "  id {:#04x}  answers on {:#04x}  MST_ID {:#04x}  mode {:?}  PMAX {:?} \
                         VMAX {:?} TMAX {:?}",
                        motor.slave_id,
                        motor.reply_id,
                        motor.master_id,
                        motor.control_mode,
                        motor.p_max,
                        motor.v_max,
                        motor.t_max
                    );
                }
            }
        }
    }
    if probe.feetech.is_empty() && probe.damiao.is_empty() {
        let _ = writeln!(out, "\nNo bus this host can open was found.");
    }
    out
}

/// The match as a person reads it.
#[must_use]
pub fn commission_report(result: &Result<Vec<PortMatch>, Vec<PortError>>) -> String {
    let mut out = String::new();
    match result {
        Ok(matched) => {
            let _ = writeln!(out, "Every port matched:");
            for port in matched {
                let ids: Vec<u8> = port.servos.iter().map(|(_, servo)| servo.id).collect();
                let _ = writeln!(
                    out,
                    "  {:<16} -> {} ({}){}",
                    port.port,
                    port.identity,
                    port.channel,
                    if ids.is_empty() {
                        String::new()
                    } else {
                        format!("  ids {ids:?}")
                    }
                );
            }
        }
        Err(errors) => {
            let _ = writeln!(out, "Refused; nothing written:");
            for error in errors {
                let _ = writeln!(out, "  {error}");
            }
        }
    }
    out
}
