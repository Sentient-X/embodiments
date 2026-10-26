//! What a servo's registers say, read through any packet handler: the read-only scan's port
//! for `calibrate read` and a dry run, the session's for a motion.

use serde_json::{Value, json};
use sx_embodiment_drivers::lerobot::{MotorCalibration, MotorNormMode, normalize};
use sx_embodiment_drivers::serial::SerialPort;
use sx_feetech_serial::scan::{BusScan, ServoScan, scan};
use sx_feetech_serial::scservo_sdk::{
    CommResult, ProtocolPacketHandler, SMS_STS_MAX_ANGLE_LIMIT_L, SMS_STS_MIN_ANGLE_LIMIT_L,
    SMS_STS_OFS_L, SMS_STS_PRESENT_POSITION_L, SMS_STS_TORQUE_ENABLE,
};

use super::{CalibrateError, HOMING_OFFSET_SIGN_BIT, LOAD_SIGN_BIT, POSITION_SIGN_BIT};

/// A servo's motion as one read of `Present_Position` (56) through `Present_Load` (61).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ServoState {
    /// Ticks, decoded at bit 15.
    pub position: i32,
    /// Ticks a second as the servo reports it, decoded at bit 15.
    pub speed: i32,
    /// Tenths of a percent of its maximum torque, decoded at bit 10.
    pub load: i32,
}

/// A servo's RAM control registers, as one read of `Torque_Enable` (40) through `Lock` (55).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ControlRegisters {
    pub torque_enable: u8,
    pub acceleration: u8,
    pub goal_position: u16,
    pub goal_speed: u16,
    pub torque_limit: u16,
    pub lock: u8,
}

fn decode(raw: u16, sign_bit: u32) -> i32 {
    ProtocolPacketHandler::<()>::scs_tohost(raw, sign_bit)
}

/// `length` bytes from `address`, refusing a failed exchange or a servo's error flags.
///
/// # Errors
///
/// The exchange's failure, or the servo's error flags.
pub fn bytes<P: SerialPort>(
    ph: &mut ProtocolPacketHandler<P>,
    id: u8,
    address: u8,
    length: u8,
) -> Result<Vec<u8>, CalibrateError> {
    let (data, result, flags) = ph.read_tx_rx(id, address, length);
    if result != CommResult::Success {
        return Err(CalibrateError::Comm {
            id,
            address,
            result,
        });
    }
    if flags != 0 {
        return Err(CalibrateError::Flags { id, address, flags });
    }
    if data.len() != usize::from(length) {
        return Err(CalibrateError::Comm {
            id,
            address,
            result: CommResult::RxCorrupt,
        });
    }
    Ok(data.to_vec())
}

fn word(data: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([data[at], data[at + 1]])
}

/// A register of `width` bytes (1 or 2) as a number.
///
/// # Errors
///
/// As [`bytes`].
pub fn register<P: SerialPort>(
    ph: &mut ProtocolPacketHandler<P>,
    id: u8,
    address: u8,
    width: u8,
) -> Result<u16, CalibrateError> {
    let data = bytes(ph, id, address, width)?;
    Ok(if width == 1 {
        u16::from(data[0])
    } else {
        word(&data, 0)
    })
}

/// # Errors
///
/// As [`bytes`].
pub fn state<P: SerialPort>(
    ph: &mut ProtocolPacketHandler<P>,
    id: u8,
) -> Result<ServoState, CalibrateError> {
    let data = bytes(ph, id, SMS_STS_PRESENT_POSITION_L, 6)?;
    Ok(ServoState {
        position: decode(word(&data, 0), POSITION_SIGN_BIT),
        speed: decode(word(&data, 2), POSITION_SIGN_BIT),
        load: decode(word(&data, 4), LOAD_SIGN_BIT),
    })
}

/// # Errors
///
/// As [`bytes`].
pub fn control<P: SerialPort>(
    ph: &mut ProtocolPacketHandler<P>,
    id: u8,
) -> Result<ControlRegisters, CalibrateError> {
    let data = bytes(ph, id, SMS_STS_TORQUE_ENABLE, 16)?;
    Ok(ControlRegisters {
        torque_enable: data[0],
        acceleration: data[1],
        goal_position: word(&data, 2),
        goal_speed: word(&data, 6),
        torque_limit: word(&data, 8),
        lock: data[15],
    })
}

/// `Min_Angle_Limit` and `Max_Angle_Limit`, as one read of 9..=12.
///
/// # Errors
///
/// As [`bytes`].
pub fn limits<P: SerialPort>(
    ph: &mut ProtocolPacketHandler<P>,
    id: u8,
) -> Result<(u16, u16), CalibrateError> {
    let data = bytes(ph, id, SMS_STS_MIN_ANGLE_LIMIT_L, 4)?;
    debug_assert_eq!(SMS_STS_MAX_ANGLE_LIMIT_L, SMS_STS_MIN_ANGLE_LIMIT_L + 2);
    Ok((word(&data, 0), word(&data, 2)))
}

/// `Homing_Offset`, decoded at bit 11.
///
/// # Errors
///
/// As [`bytes`].
pub fn homing_offset<P: SerialPort>(
    ph: &mut ProtocolPacketHandler<P>,
    id: u8,
) -> Result<i16, CalibrateError> {
    let raw = register(ph, id, SMS_STS_OFS_L, 2)?;
    Ok(i16::try_from(decode(raw & 0x0FFF, HOMING_OFFSET_SIGN_BIT)).unwrap_or_default())
}

/// The STS3215's encoder steps a turn (`LeRobot`'s `MODEL_RESOLUTION`).
const RESOLUTION: f64 = 4096.0;

/// `raw` ticks in `LeRobot`'s degrees about the middle of `calibration`'s range, and in its
/// percent of the range (`MotorsBus._normalize`, drive mode applied as Feetech's bus does).
#[must_use]
pub fn lerobot_position(calibration: &MotorCalibration, raw: i32) -> (f64, f64) {
    let degrees = normalize(
        MotorNormMode::Degrees,
        calibration,
        true,
        RESOLUTION,
        f64::from(raw),
        (0.0, 0.0),
    )
    .to_degrees();
    let percent = normalize(
        MotorNormMode::Range0100,
        calibration,
        true,
        RESOLUTION,
        f64::from(raw),
        (0.0, 100.0),
    );
    (degrees, percent)
}

/// One servo as `calibrate read` reports it.
#[derive(Clone, Debug)]
pub struct ServoReading {
    pub scan: ServoScan,
    pub control: Result<ControlRegisters, String>,
    pub state: Result<ServoState, String>,
}

impl ServoReading {
    #[must_use]
    pub fn json(&self) -> Value {
        let calibration = self.scan.calibration;
        let (degrees, percent) =
            lerobot_position(&calibration, i32::from(self.scan.present_position));
        json!({
            "id": self.scan.id,
            "model": self.scan.model,
            "firmware": format!("{}.{}", self.scan.firmware_major, self.scan.firmware_minor),
            "range_min": calibration.range_min,
            "range_max": calibration.range_max,
            "homing_offset": calibration.homing_offset,
            "operating_mode": self.scan.operating_mode,
            "pdi": [self.scan.p_coefficient, self.scan.d_coefficient, self.scan.i_coefficient],
            "max_torque_limit": self.scan.max_torque_limit,
            "torque_enabled": self.scan.torque_enabled,
            "present_position": self.scan.present_position,
            "lerobot_degrees": degrees,
            "lerobot_percent": percent,
            "voltage": f64::from(self.scan.voltage_decivolts) / 10.0,
            "temperature_celsius": self.scan.temperature_celsius,
            "status": self.scan.status,
            "control": self.control.as_ref().map_or_else(|e| json!({ "error": e }), |c| json!({
                "acceleration": c.acceleration,
                "goal_position": c.goal_position,
                "goal_speed": c.goal_speed,
                "torque_limit": c.torque_limit,
                "lock": c.lock,
            })),
            "state": self.state.as_ref().map_or_else(|e| json!({ "error": e }), |s| json!({
                "speed": s.speed,
                "load": s.load,
            })),
        })
    }

    #[must_use]
    pub fn line(&self) -> String {
        let s = &self.scan;
        let c = s.calibration;
        let (degrees, percent) = lerobot_position(&c, i32::from(s.present_position));
        let control = self.control.as_ref().map_or_else(
            |e| format!("control unread ({e})"),
            |c| {
                format!(
                    "goal {} speed {} acc {} torque_limit {} lock {}",
                    c.goal_position, c.goal_speed, c.acceleration, c.torque_limit, c.lock
                )
            },
        );
        let load = self.state.as_ref().map_or_else(
            |e| format!("load unread ({e})"),
            |s| format!("load {}", s.load),
        );
        format!(
            "id {:2}  model {}  fw {}.{}  range {}..{}  offset {:5}  mode {}  max_torque {}  \
             torque {}  position {} ({degrees:+.1} deg, {percent:.1}%)  {load}  {control}  \
             {:.1} V  {} C",
            s.id,
            s.model,
            s.firmware_major,
            s.firmware_minor,
            c.range_min,
            c.range_max,
            c.homing_offset,
            s.operating_mode,
            s.max_torque_limit,
            if s.torque_enabled { "ON" } else { "off" },
            s.present_position,
            f64::from(s.voltage_decivolts) / 10.0,
            s.temperature_celsius,
        )
    }
}

/// Scan `ids` on `port` read-only and read each answering servo's control registers and load
/// too, through the same read-only port.
pub fn read_bus<P: SerialPort>(
    port: &mut P,
    ids: impl IntoIterator<Item = u8>,
) -> (BusScan, Vec<ServoReading>) {
    let found = scan(&mut *port, ids);
    let mut ph = ProtocolPacketHandler::new(sx_feetech_serial::scan::ReadOnlyPort::new(port), 0);
    let readings = found
        .servos
        .iter()
        .map(|servo| ServoReading {
            scan: *servo,
            control: control(&mut ph, servo.id).map_err(|e| e.to_string()),
            state: state(&mut ph, servo.id).map_err(|e| e.to_string()),
        })
        .collect();
    (found, readings)
}
