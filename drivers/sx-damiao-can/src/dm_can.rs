//! Damiao's own Python driver, `DM_CAN.py`, translated.
//!
//! Reference: <https://gitee.com/kit-miao/motor-sdk> at
//! `fb0e9fc5455ecb02ed13cc1f43de58078be61b07`, file `Python例程/u2can/DM_CAN.py` (the motor SDK
//! submodule of Damiao's official repository <https://gitee.com/kit-miao/damiao>, `SDK/电机SDK`),
//! Damiao's modified copy of cmjang's `DM_Control_Python`
//! (<https://github.com/cmjang/DM_Control_Python> at `7da93877ba844d9587149d6f3a6385453aa8379f`).
//!
//! Vendor-derived: Copyright (c) 2024 cmjang, MIT License. The full permission notice is in
//! `LICENSE-THIRD-PARTY` beside this crate's `Cargo.toml`; Sentient-X's changes are Apache-2.0.
//!
//! Same structure, names and arithmetic: [`Motor`] is its `Motor`, [`MotorControl`] its
//! `MotorControl`, and the free functions its module helpers. Every command keeps the vendor's
//! frame, ids, waits and order: `enable` sends `FF..FC` to the motor's `SlaveID`, waits 0.1 s and
//! reads; `disable` sends `FF..FD` and waits 0.01 s without reading; `controlMIT` packs and
//! clamps exactly as `float_to_uint` does and reads once; register reads and writes go to
//! `0x7FF`. `tests/fixtures/dm_can/` pins this file to transcripts recorded by running
//! `DM_CAN.py` itself.
//!
//! Where this translation departs from `DM_CAN.py`, and why:
//!
//! - The USB2CAN serial framing of `__send_data` and `__extract_packets` lives in
//!   [`crate::usb2can`], behind the bus-agnostic `CanPort`, so the same driver runs over
//!   `SocketCAN`. The serial packet's command byte (`CMD == 0x11`, a received CAN frame) is
//!   checked there; here every frame is already a received CAN frame.
//! - The transport keeps an unparsed remainder across every read; `recv_set_param_data` in
//!   `DM_CAN.py` reads without prepending it.
//! - `recv` and `recv_set_param_data` read until the link has nothing pending; `DM_CAN.py`
//!   reads `serial_.read_all()` once per call.
//! - A method naming an unregistered motor returns [`DmCanError::MotorNotFound`] where the
//!   vendor prints and returns. That includes `enable`, `disable`, `set_zero_position` and
//!   `refresh_motor_status`, which `DM_CAN.py` sends whether or not the motor is registered.
//! - A feedback frame is taken only on the motor's `MasterID` with the motor's id in its first
//!   byte's low nibble; `DM_CAN.py` takes any registered id and, on id 0, looks the motor up by
//!   that nibble. A frame failing the check is dropped, so the motor reads as silent.
//! - [`Motor`] also keeps the feedback's MOS and rotor temperatures (bytes 6 and 7), which
//!   `DM_CAN.py` does not decode, and counts its feedback frames (`feedback_count`) so a caller
//!   can tell a fresh answer from a cached one.
//! - Each [`Motor`] carries its own limit row, and [`MotorControl::change_limit_param`] changes
//!   one motor's, as Damiao's `SocketCAN` routine does (`Motor.limit_param` and
//!   `changeMotorLimit` in `SocketCan控制例程/Python例程/damiao_socketcan.py`,
//!   <https://gitee.com/kit-miao/motor-control-routine> at
//!   `fa8f511cab00ac424a10fb94d4ad2c11e270f5b2`); `DM_CAN.py`'s `change_limit_param` mutates
//!   the row every motor of that model shares.
//! - Translated only where a consumer drives it: MIT control, enable, disable, set zero,
//!   refresh, register read and write, control-mode switch, limit change. The position,
//!   velocity, force-position and CSP modes, `enable_old`, `control_delay`,
//!   `change_motor_param` and `save_motor_param` (a flash write) are left out; `Pd`, `Vd` and
//!   `isEnable`, which `DM_CAN.py` never reads, too.
//! - Kept from `DM_CAN.py` as it is: `recv` decodes a register reply arriving on a `MasterID`
//!   as feedback, as the vendor's `__process_packet` does; replies are consumed by
//!   `recv_set_param_data` first in every sequence this crate runs.

// The vendor's names (`kp_uint`/`kd_uint`, `recv_q`/`recv_dq`) are kept.
#![allow(clippy::similar_names)]

use std::collections::BTreeMap;
use std::io;
use std::time::Duration;

use sx_embodiment_drivers::can::{CanFrame, CanPort};
use thiserror::Error;

/// `DM_Motor_Type`, the index into [`LIMIT_PARAM`].
#[allow(non_camel_case_types)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum DmMotorType {
    DM4310 = 0,
    DM4310_48V = 1,
    DM4340 = 2,
    DM4340_48V = 3,
    DM6006 = 4,
    DM8006 = 5,
    DM8009 = 6,
    DM10010L = 7,
    DM10010 = 8,
    DMH3510 = 9,
    DMH6215 = 10,
    DMG6220 = 11,
    DMJH11 = 12,
    DM6248P = 13,
    DM3507 = 14,
}

/// `MotorControl.Limit_Param`: `[Q_MAX, DQ_MAX, TAU_MAX]` per [`DmMotorType`].
pub const LIMIT_PARAM: [[f64; 3]; 15] = [
    [12.5, 30.0, 10.0],    // DM4310
    [12.5, 50.0, 10.0],    // DM4310_48
    [12.5, 10.0, 28.0],    // DM4340
    [12.5, 10.0, 28.0],    // DM4340_48
    [12.5, 45.0, 20.0],    // DM6006
    [12.5, 45.0, 40.0],    // DM8006
    [12.5, 45.0, 54.0],    // DM8009
    [12.5, 25.0, 200.0],   // DM10010L
    [12.5, 20.0, 200.0],   // DM10010
    [12.5, 280.0, 1.0],    // DMH3510
    [12.5, 45.0, 10.0],    // DMG6215
    [12.5, 45.0, 10.0],    // DMH6220
    [12.5, 10.0, 12.0],    // DMJH11
    [12.566, 20.0, 120.0], // DM6248P
    [12.566, 50.0, 5.0],   // DM3507
];

impl DmMotorType {
    /// This model's `[Q_MAX, DQ_MAX, TAU_MAX]` row.
    #[must_use]
    pub const fn limit_param(self) -> [f64; 3] {
        LIMIT_PARAM[self as usize]
    }
}

/// `DM_variable`, the motor's register ids (`RID`).
#[allow(non_camel_case_types)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum DmVariable {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    LS = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GREF = 30,
    Deta = 31,
    V_BW = 32,
    IQ_c1 = 33,
    VL_c1 = 34,
    can_br = 35,
    sub_ver = 36,
    u_off = 50,
    v_off = 51,
    k1 = 52,
    k2 = 53,
    m_off = 54,
    dir = 55,
    p_m = 80,
    xout = 81,
}

/// `Control_Type`, the values of the `CTRL_MODE` register.
#[allow(non_camel_case_types)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ControlType {
    MIT = 1,
    POS_VEL = 2,
    VEL = 3,
    Torque_Pos = 4,
    POS_VEL_CSP = 5,
    VEL_CSP = 6,
}

/// One register value as `temp_param_dict` holds it: `uint32` or `float` by [`is_in_ranges`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParamValue {
    Uint32(u32),
    Float(f32),
}

/// `Motor`: one motor's ids, model and the state its latest feedback frame reported.
#[derive(Clone, Debug, PartialEq)]
pub struct Motor {
    pub state_q: f32,
    pub state_dq: f32,
    pub state_tau: f32,
    pub state_err: u8,
    /// MOS and rotor temperatures, degrees Celsius (not in `DM_CAN.py`).
    pub state_t_mos: u8,
    pub state_t_rotor: u8,
    pub slave_id: u16,
    pub master_id: u16,
    pub motor_type: DmMotorType,
    /// `[Q_MAX, DQ_MAX, TAU_MAX]` this motor's frames are scaled by.
    pub limit_param: [f64; 3],
    pub now_control_mode: ControlType,
    pub temp_param_dict: BTreeMap<u8, ParamValue>,
    /// Feedback frames decoded so far (not in `DM_CAN.py`; see the module header).
    pub feedback_count: u64,
}

impl Motor {
    /// `Motor(MotorType, SlaveID, MasterID)`. Damiao advises against a master id of 0.
    #[must_use]
    pub const fn new(motor_type: DmMotorType, slave_id: u16, master_id: u16) -> Self {
        Self {
            state_q: 0.0,
            state_dq: 0.0,
            state_tau: 0.0,
            state_err: 0,
            state_t_mos: 0,
            state_t_rotor: 0,
            slave_id,
            master_id,
            motor_type,
            limit_param: motor_type.limit_param(),
            now_control_mode: ControlType::MIT,
            temp_param_dict: BTreeMap::new(),
            feedback_count: 0,
        }
    }

    pub fn recv_data(&mut self, q: f32, dq: f32, tau: f32, err: u8) {
        self.state_q = q;
        self.state_dq = dq;
        self.state_tau = tau;
        self.state_err = err;
        self.feedback_count += 1;
    }

    /// The cached position: Damiao motors answer one frame per frame sent, so this is fresh only
    /// after a control frame or `refresh_motor_status`.
    #[must_use]
    pub const fn get_position(&self) -> f32 {
        self.state_q
    }

    #[must_use]
    pub const fn get_velocity(&self) -> f32 {
        self.state_dq
    }

    #[must_use]
    pub const fn get_torque(&self) -> f32 {
        self.state_tau
    }

    /// The feedback's first-byte high nibble; [`crate::chain::MotorState`] names its values.
    #[must_use]
    pub const fn get_error(&self) -> u8 {
        self.state_err
    }

    /// `getParam`: a register value read earlier, if any.
    #[must_use]
    pub fn get_param(&self, rid: DmVariable) -> Option<ParamValue> {
        self.temp_param_dict.get(&(rid as u8)).copied()
    }
}

#[derive(Debug, Error)]
pub enum DmCanError {
    #[error("Motor ID {0:#x} not found")]
    MotorNotFound(u16),
    #[error("cannot convert float NaN to integer")]
    NotANumber,
    #[error("Value must be an integer within the range of uint32")]
    ParamOutOfRange,
    #[error("the CAN link failed")]
    Io(#[from] io::Error),
}

const ENABLE_WAIT: Duration = Duration::from_millis(100);
pub(crate) const DISABLE_WAIT: Duration = Duration::from_millis(10);
const SET_ZERO_WAIT: Duration = Duration::from_millis(100);
const PARAM_RETRY_INTERVAL: Duration = Duration::from_millis(50);
const SWITCH_MODE_RETRIES: usize = 10;
const READ_PARAM_RETRIES: usize = 20;
const PARAM_ID: u16 = 0x7FF;

/// `MotorControl`: the motors on one CAN link, addressed by `SlaveID`.
pub struct MotorControl<P> {
    port: P,
    motors: Vec<Motor>,
    /// `motors_map`: `SlaveID` and (when not 0) `MasterID`, each to its motor.
    motors_map: BTreeMap<u32, usize>,
}

impl<P: CanPort> MotorControl<P> {
    #[must_use]
    pub const fn new(port: P) -> Self {
        Self {
            port,
            motors: Vec::new(),
            motors_map: BTreeMap::new(),
        }
    }

    /// `controlMIT`: one MIT frame to the motor's `SlaveID`, then read.
    ///
    /// # Errors
    ///
    /// Returns an unregistered motor or the link's failure.
    pub fn control_mit(
        &mut self,
        slave_id: u16,
        kp: f64,
        kd: f64,
        q: f64,
        dq: f64,
        tau: f64,
    ) -> Result<(), DmCanError> {
        let index = self.index(slave_id)?;
        let kp_uint = float_to_uint(kp, 0.0, 500.0, 12)?;
        let kd_uint = float_to_uint(kd, 0.0, 5.0, 12)?;
        let [q_max, dq_max, tau_max] = self.motors[index].limit_param;
        let q_uint = float_to_uint(q, -q_max, q_max, 16)?;
        let dq_uint = float_to_uint(dq, -dq_max, dq_max, 12)?;
        let tau_uint = float_to_uint(tau, -tau_max, tau_max, 12)?;
        let data_buf = [
            low_byte(q_uint >> 8),
            low_byte(q_uint),
            low_byte(dq_uint >> 4),
            low_byte(((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf)),
            low_byte(kp_uint),
            low_byte(kd_uint >> 4),
            low_byte(((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf)),
            low_byte(tau_uint),
        ];
        self.send_data(slave_id, data_buf)?;
        self.recv()
    }

    /// `enable`: `FF..FC`, wait 0.1 s, read. Damiao advises enabling a few seconds after power-up.
    ///
    /// # Errors
    ///
    /// Returns an unregistered motor or the link's failure.
    pub fn enable(&mut self, slave_id: u16) -> Result<(), DmCanError> {
        self.control_cmd(slave_id, 0xFC)?;
        self.port.sleep(ENABLE_WAIT);
        self.recv()
    }

    /// `disable`: `FF..FD`, wait 0.01 s. The vendor does not read the answer here.
    ///
    /// # Errors
    ///
    /// Returns an unregistered motor or the link's failure.
    pub fn disable(&mut self, slave_id: u16) -> Result<(), DmCanError> {
        self.control_cmd(slave_id, 0xFD)?;
        self.port.sleep(DISABLE_WAIT);
        Ok(())
    }

    /// `set_zero_position`: `FF..FE`, wait 0.1 s, read.
    ///
    /// # Errors
    ///
    /// Returns an unregistered motor or the link's failure.
    pub fn set_zero_position(&mut self, slave_id: u16) -> Result<(), DmCanError> {
        self.control_cmd(slave_id, 0xFE)?;
        self.port.sleep(SET_ZERO_WAIT);
        self.recv()
    }

    /// `recv`: decode every received frame into its motor's state.
    ///
    /// # Errors
    ///
    /// Returns the link's failure.
    pub fn recv(&mut self) -> Result<(), DmCanError> {
        while let Some(frame) = self.port.receive()? {
            self.process_packet(frame.data, frame.id);
        }
        Ok(())
    }

    /// `recv_set_param_data`: take register replies; every other frame is dropped.
    ///
    /// # Errors
    ///
    /// Returns the link's failure.
    pub fn recv_set_param_data(&mut self) -> Result<(), DmCanError> {
        while let Some(frame) = self.port.receive()? {
            self.process_set_param_packet(frame.data, frame.id);
        }
        Ok(())
    }

    fn process_packet(&mut self, data: [u8; 8], can_id: u32) {
        let Some(&index) = self.motors_map.get(&can_id) else {
            return;
        };
        let motor = &self.motors[index];
        if u32::from(motor.master_id) != can_id
            || u16::from(data[0] & 0x0f) != motor.slave_id & 0x0f
        {
            return;
        }
        let err_int = (data[0] >> 4) & 0x0f;
        let q_uint = (u16::from(data[1]) << 8) | u16::from(data[2]);
        let dq_uint = (u16::from(data[3]) << 4) | (u16::from(data[4]) >> 4);
        let tau_uint = ((u16::from(data[4]) & 0xf) << 8) | u16::from(data[5]);
        let motor = &mut self.motors[index];
        let [q_max, dq_max, tau_max] = motor.limit_param;
        let recv_q = uint_to_float(q_uint, -q_max, q_max, 16);
        let recv_dq = uint_to_float(dq_uint, -dq_max, dq_max, 12);
        let recv_tau = uint_to_float(tau_uint, -tau_max, tau_max, 12);
        motor.state_t_mos = data[6];
        motor.state_t_rotor = data[7];
        motor.recv_data(recv_q, recv_dq, recv_tau, err_int);
    }

    fn process_set_param_packet(&mut self, data: [u8; 8], can_id: u32) {
        if data[2] != 0x33 && data[2] != 0x55 {
            return;
        }
        let mut master_id = can_id;
        let slave_id = (u32::from(data[1]) << 8) | u32::from(data[0]);
        if can_id == 0x00 {
            master_id = slave_id;
        }
        if !self.motors_map.contains_key(&master_id) {
            if !self.motors_map.contains_key(&slave_id) {
                return;
            }
            master_id = slave_id;
        }
        let rid = data[3];
        let num = if is_in_ranges(rid) {
            ParamValue::Uint32(uint8s_to_uint32(data[4], data[5], data[6], data[7]))
        } else {
            ParamValue::Float(uint8s_to_float(data[4], data[5], data[6], data[7]))
        };
        let index = self.motors_map[&master_id];
        self.motors[index].temp_param_dict.insert(rid, num);
    }

    /// `addMotor`: register under `SlaveID` and, when not 0, `MasterID`.
    pub fn add_motor(&mut self, motor: Motor) -> bool {
        let index = self.motors.len();
        self.motors_map.insert(u32::from(motor.slave_id), index);
        if motor.master_id != 0 {
            self.motors_map.insert(u32::from(motor.master_id), index);
        }
        self.motors.push(motor);
        true
    }

    fn control_cmd(&mut self, slave_id: u16, cmd: u8) -> Result<(), DmCanError> {
        self.index(slave_id)?;
        let data_buf = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd];
        self.send_data(slave_id, data_buf)
    }

    fn send_data(&mut self, motor_id: u16, data: [u8; 8]) -> Result<(), DmCanError> {
        self.port.send(&CanFrame {
            id: u32::from(motor_id),
            data,
        })?;
        Ok(())
    }

    fn read_rid_param(&mut self, slave_id: u16, rid: DmVariable) -> Result<(), DmCanError> {
        let [can_id_l, can_id_h] = slave_id.to_le_bytes();
        let data_buf = [can_id_l, can_id_h, 0x33, rid as u8, 0x00, 0x00, 0x00, 0x00];
        self.send_data(PARAM_ID, data_buf)
    }

    /// `__write_motor_param`: the value's wire type follows the register, by [`is_in_ranges`].
    fn write_motor_param(
        &mut self,
        slave_id: u16,
        rid: DmVariable,
        data: f64,
    ) -> Result<(), DmCanError> {
        let [can_id_l, can_id_h] = slave_id.to_le_bytes();
        let mut data_buf = [can_id_l, can_id_h, 0x55, rid as u8, 0x00, 0x00, 0x00, 0x00];
        data_buf[4..8].copy_from_slice(&if is_in_ranges(rid as u8) {
            // `int(data)` truncates toward zero; `data_to_uint8s` refuses what uint32 cannot hold.
            let truncated = data.trunc();
            if !(0.0..=f64::from(u32::MAX)).contains(&truncated) {
                return Err(DmCanError::ParamOutOfRange);
            }
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            let value = truncated as u32;
            data_to_uint8s(value)
        } else {
            #[allow(clippy::cast_possible_truncation)]
            let value = data as f32;
            float_to_uint8s(value)
        });
        self.send_data(PARAM_ID, data_buf)
    }

    /// `switchControlMode`: write `CTRL_MODE`, then poll every 0.05 s, ten times, for the
    /// motor's echo. `true` only when the echoed mode is the one asked for.
    ///
    /// # Errors
    ///
    /// Returns an unregistered motor or the link's failure.
    pub fn switch_control_mode(
        &mut self,
        slave_id: u16,
        control_mode: ControlType,
    ) -> Result<bool, DmCanError> {
        let index = self.index(slave_id)?;
        let rid = DmVariable::CTRL_MODE;
        self.write_motor_param(slave_id, rid, f64::from(control_mode as u8))?;
        for _ in 0..SWITCH_MODE_RETRIES {
            self.port.sleep(PARAM_RETRY_INTERVAL);
            self.recv_set_param_data()?;
            if let Some(value) = self.motors[index].get_param(rid) {
                return Ok(value == ParamValue::Uint32(u32::from(control_mode as u8)));
            }
        }
        Ok(false)
    }

    /// `changeMotorLimit` of Damiao's `SocketCAN` routine: rescale one motor's frames to the
    /// `PMAX`, `VMAX` and `TMAX` its registers hold.
    ///
    /// # Errors
    ///
    /// Returns an unregistered motor.
    pub fn change_limit_param(
        &mut self,
        slave_id: u16,
        p_max: f64,
        v_max: f64,
        t_max: f64,
    ) -> Result<(), DmCanError> {
        let index = self.index(slave_id)?;
        self.motors[index].limit_param = [p_max, v_max, t_max];
        Ok(())
    }

    /// `refresh_motor_status`: ask the motor for a feedback frame, then read.
    ///
    /// # Errors
    ///
    /// Returns an unregistered motor or the link's failure.
    pub fn refresh_motor_status(&mut self, slave_id: u16) -> Result<(), DmCanError> {
        self.index(slave_id)?;
        let [can_id_l, can_id_h] = slave_id.to_le_bytes();
        let data_buf = [can_id_l, can_id_h, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00];
        self.send_data(PARAM_ID, data_buf)?;
        self.recv()
    }

    /// `read_motor_param`: request a register, wait 0.05 s, take replies, return the value held.
    ///
    /// As in `DM_CAN.py` the poll loop returns after its first wait whether or not the reply
    /// arrived, and a value read earlier answers a later read whose reply is lost.
    ///
    /// # Errors
    ///
    /// Returns an unregistered motor or the link's failure.
    pub fn read_motor_param(
        &mut self,
        slave_id: u16,
        rid: DmVariable,
    ) -> Result<Option<ParamValue>, DmCanError> {
        let index = self.index(slave_id)?;
        self.read_rid_param(slave_id, rid)?;
        for _ in 0..READ_PARAM_RETRIES {
            self.port.sleep(PARAM_RETRY_INTERVAL);
            self.recv_set_param_data()?;
            if self.motors_map.contains_key(&u32::from(slave_id)) {
                return Ok(self.motors[index].get_param(rid));
            }
        }
        Ok(None)
    }

    /// The registered motor addressed by `slave_id`.
    ///
    /// # Errors
    ///
    /// Returns an unregistered motor.
    pub fn motor(&self, slave_id: u16) -> Result<&Motor, DmCanError> {
        Ok(&self.motors[self.index(slave_id)?])
    }

    #[must_use]
    pub const fn port(&self) -> &P {
        &self.port
    }

    pub const fn port_mut(&mut self) -> &mut P {
        &mut self.port
    }

    fn index(&self, slave_id: u16) -> Result<usize, DmCanError> {
        self.motors_map
            .get(&u32::from(slave_id))
            .copied()
            .ok_or(DmCanError::MotorNotFound(slave_id))
    }
}

/// `LIMIT_MIN_MAX`.
#[must_use]
pub fn limit_min_max(x: f64, min: f64, max: f64) -> f64 {
    if x <= min {
        min
    } else if x > max {
        max
    } else {
        x
    }
}

/// `float_to_uint`: clamp, normalise, scale to `bits`, truncate as `np.uint16` does.
///
/// # Errors
///
/// A NaN passes `LIMIT_MIN_MAX` unclamped and `np.uint16` refuses it; so does this.
pub fn float_to_uint(x: f64, x_min: f64, x_max: f64, bits: u32) -> Result<u16, DmCanError> {
    let x = limit_min_max(x, x_min, x_max);
    let span = x_max - x_min;
    let data_norm = (x - x_min) / span;
    let scaled = data_norm * f64::from((1_u32 << bits) - 1);
    if scaled.is_nan() {
        return Err(DmCanError::NotANumber);
    }
    // In 0..=(1 << bits) - 1 by the clamp.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let packed = scaled as u16;
    Ok(packed)
}

/// `uint_to_float`: computed in double precision and returned as `np.float32`.
#[must_use]
pub fn uint_to_float(x: u16, min: f64, max: f64, bits: u32) -> f32 {
    let span = max - min;
    let data_norm = f64::from(x) / f64::from((1_u32 << bits) - 1);
    let temp = data_norm * span + min;
    #[allow(clippy::cast_possible_truncation)]
    let narrowed = temp as f32;
    narrowed
}

/// `float_to_uint8s`: the `float32`'s bytes (`pack('f')`, little-endian on every station host).
#[must_use]
pub const fn float_to_uint8s(value: f32) -> [u8; 4] {
    value.to_le_bytes()
}

/// `data_to_uint8s`: the `uint32`'s bytes.
#[must_use]
pub const fn data_to_uint8s(value: u32) -> [u8; 4] {
    value.to_le_bytes()
}

/// `is_in_ranges`: whether register `number` holds a `uint32` rather than a `float`.
#[must_use]
pub const fn is_in_ranges(number: u8) -> bool {
    matches!(number, 7..=10 | 13..=16 | 35..=36)
}

#[must_use]
pub const fn uint8s_to_uint32(byte1: u8, byte2: u8, byte3: u8, byte4: u8) -> u32 {
    u32::from_le_bytes([byte1, byte2, byte3, byte4])
}

#[must_use]
pub const fn uint8s_to_float(byte1: u8, byte2: u8, byte3: u8, byte4: u8) -> f32 {
    f32::from_le_bytes([byte1, byte2, byte3, byte4])
}

const fn low_byte(value: u16) -> u8 {
    value.to_le_bytes()[0]
}
