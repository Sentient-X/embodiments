//! A chain of bound Damiao motors as one `ActuatorChain`, driven through [`crate::dm_can`].
//!
//! Every bus operation is a `DM_CAN.py` call; the order they are called in is this chain's.
//! Open follows the vendor demo's bring-up for each motor (`DM_Motor_Test.py:14-26`, same
//! commit: `switchControlMode`, then `read_motor_param` of `PMAX`, `VMAX` and `TMAX`, which the
//! demo prints), then `disable`s it. Two deviations from the demo: it switches to `POS_VEL`
//! where the chain switches to `MIT`, and it refuses nothing where the chain refuses a register
//! value (below). Command `enable`s each motor when the chain is disabled, then sends one `controlMIT` per
//! motor holding its target with zero velocity and feed-forward torque. Stop `disable`s every
//! motor.
//!
//! What this chain adds over the vendor's calls, and why:
//!
//! - It refuses before any frame: a target outside the axis's commandable range (the declared
//!   bounds intersected with the motor's configured range), ids or gains the protocol cannot
//!   carry, a model or bus this crate is not qualified for. `DM_CAN.py` clamps silently.
//! - It reads each motor's `PMAX`, `VMAX` and `TMAX` registers at open and scales that motor's
//!   frames by them, refusing values no Damiao table carries for its model, since a wrong row
//!   rescales every position, velocity and torque on the wire. Damiao's own tables disagree on
//!   the DM4340's velocity range — 10 rad/s in `DM_CAN.py`, 8 rad/s in its C++ SDK
//!   (`C++例程/u2can/include/damiao.h`, same commit) — so either is accepted for that model, and
//!   [`DamiaoChain::limits`] records which the motor reported. The vendor's own answer to a
//!   motor on another table is `DM_CAN.py`'s `change_limit_param`, which rewrites the row every
//!   motor of that model shares; this crate rescales the one motor instead
//!   ([`crate::dm_can::MotorControl::change_limit_param`]).
//! - After the vendor's own read it waits briefly for each motor's answer, and judges it by the
//!   feedback's first-byte high nibble ([`MotorState`]). `DM_CAN.py` stores the nibble unread.
//! - A stop is proven motor by motor: a fresh answer reporting disabled. A silent motor is asked
//!   once more with `refresh_motor_status`. After an unproven stop every command is refused
//!   until a stop is proven.

use std::time::Duration;

use sx_embodiment_drivers::can::CanPort;
use sx_embodiment_drivers::{
    ActuatorBus, ActuatorChain, ActuatorModel, BindingError, BoundAxis, CommandRange, Evidence,
    Receipt, StopReport, TargetError, check_targets, validate_chain,
};
use thiserror::Error;

use crate::dm_can::{
    ControlType, DmCanError, DmMotorType, DmVariable, Motor, MotorControl, ParamValue,
};

/// `ActuatorModel.DAMIAO_DM4310` in the registry.
pub const DAMIAO_DM4310: ActuatorModel = ActuatorModel::new("damiao_dm4310");
/// `ActuatorModel.DAMIAO_DM4340` in the registry.
pub const DAMIAO_DM4340: ActuatorModel = ActuatorModel::new("damiao_dm4340");
/// `ActuatorBus.DAMIAO_CAN` in the registry.
pub const DAMIAO_CAN: ActuatorBus = ActuatorBus::new("damiao_can");

const KP_MAX: f64 = 500.0;
const KD_MAX: f64 = 5.0;
/// `0x7FF` addresses register requests; no motor may sit on it.
const MAX_MOTOR_ID: u16 = 0x7FE;
const ANSWER_POLLS: usize = 20;
const ANSWER_POLL_INTERVAL: Duration = Duration::from_millis(1);
/// The DM4340 velocity ranges Damiao publishes: `DM_CAN.py`'s row, then `damiao.h`'s.
const DM4340_VMAX: [f64; 2] = [10.0, 8.0];

/// The `DM_CAN.py` limit row a qualified registry model is driven with.
#[must_use]
pub fn motor_type(model: ActuatorModel) -> Option<DmMotorType> {
    if model == DAMIAO_DM4310 {
        Some(DmMotorType::DM4310)
    } else if model == DAMIAO_DM4340 {
        Some(DmMotorType::DM4340)
    } else {
        None
    }
}

/// The `VMAX` values a Damiao table carries for `model`.
fn accepted_vmax(model: DmMotorType) -> Vec<f64> {
    match model {
        DmMotorType::DM4340 => DM4340_VMAX.to_vec(),
        other => vec![other.limit_param()[1]],
    }
}

/// One bound axis with what its Damiao motor needs beyond the registry's binding.
#[derive(Clone, Debug, PartialEq)]
pub struct DamiaoAxis {
    pub axis: BoundAxis,
    /// The id the motor answers on (`MasterID`).
    pub master_id: u16,
    /// MIT position gain (0..=500) and damping (0..=5).
    pub kp: f64,
    pub kd: f64,
    /// The motor's own commandable range, radians, inside its model's `Q_MAX`.
    pub motor_lower: f64,
    pub motor_upper: f64,
}

impl DamiaoAxis {
    /// The joint range this axis is commanded within: the declared bounds intersected with the
    /// motor range mapped back through the binding.
    #[must_use]
    pub fn commandable(&self) -> CommandRange {
        let binding = self.axis.binding;
        let ends = [
            binding.joint_from_actuator(self.motor_lower),
            binding.joint_from_actuator(self.motor_upper),
        ];
        CommandRange {
            lower: self.axis.lower.max(ends[0].min(ends[1])),
            upper: self.axis.upper.min(ends[0].max(ends[1])),
        }
    }
}

/// The state a feedback frame's first-byte high nibble reports.
///
/// The codes are Damiao's: "DM-J4310-2EC V1.2 Geared Motor User Manual V1.4 2026-09-14" and
/// "DM-J4340P-2EC V1.1 Geared Motor User Manual V1.4 2026-09-14", chapter "CAN Communication",
/// subsection "Feedback Frames" (p. 14-15; gitee `kit-miao/DM-J4310-2EC` at `dc15860d`,
/// `kit-miao/DM-J4340P-2EC` at `0f93050a`, `说明书/`), and `status_name` in
/// `motorbridge/motorbridge` at `e8b3ac66f73a4a5a82e79c48a1707935ca4c8b96`,
/// `motor_vendors/damiao/src/protocol.rs:29-41`, the driver Seeed's controller runs: 0 disabled
/// (the power-on state), 1 enabled, 8 over-voltage, 9 under-voltage, 0xA over-current, 0xB MOSFET
/// over-temperature, 0xC motor coil over-temperature, 0xD communication lost, 0xE overload. The
/// Chinese editions of both manuals (subsection "反馈帧") also list 3 output-shaft calibration
/// error, 4 sensor output error and 5 motor encoder calibration error. Every code other than 0
/// and 1 is a [`MotorState::Fault`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotorState {
    Disabled,
    Enabled,
    Fault(u8),
}

impl MotorState {
    #[must_use]
    pub const fn from_err(err: u8) -> Self {
        match err {
            0 => Self::Disabled,
            1 => Self::Enabled,
            code => Self::Fault(code),
        }
    }
}

/// One motor's latest answer, in the motor's own units (rad, rad/s, N·m, °C).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorFeedback {
    pub bus_id: u16,
    pub state: MotorState,
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub mos_celsius: u8,
    pub rotor_celsius: u8,
}

/// The limit row a motor reported at open, and its frames are scaled by.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorLimits {
    pub bus_id: u16,
    pub p_max: f32,
    pub v_max: f32,
    pub t_max: f32,
}

#[derive(Debug, Error)]
pub enum DamiaoChainError {
    #[error(transparent)]
    Binding(#[from] BindingError),
    #[error(transparent)]
    Target(#[from] TargetError),
    #[error(
        "bus_id {0} is not on the damiao_can bus or binds a model this crate is not qualified for"
    )]
    Unqualified(u16),
    #[error("bus_id {0} or its master id is 0, 0x7FF or above, or shared with another motor")]
    Ids(u16),
    #[error("bus_id {0} has MIT gains outside kp 0..=500, kd 0..=5")]
    Gains(u16),
    #[error("bus_id {0} has a motor range that is not finite lower < upper inside its Q_MAX")]
    MotorRange(u16),
    #[error("joint {joint} target {value} is outside its commandable range")]
    Uncommandable { joint: String, value: f64 },
    #[error("bus_id {0} did not confirm MIT control mode")]
    ModeNotConfirmed(u16),
    #[error(
        "bus_id {bus_id} reports {register:?} = {motor:?}, which no Damiao table carries for its model"
    )]
    LimitMismatch {
        bus_id: u16,
        register: DmVariable,
        motor: Option<ParamValue>,
    },
    #[error("bus_id {0} did not answer")]
    Silent(u16),
    #[error("bus_id {bus_id} reports {state:?} where {expected:?} was required")]
    UnexpectedState {
        bus_id: u16,
        state: MotorState,
        expected: MotorState,
    },
    #[error("an earlier stop is unproven; motion is refused until a stop is proven")]
    StopPending,
    #[error(transparent)]
    DmCan(#[from] DmCanError),
}

/// Bound Damiao motors on one CAN link, in native state order.
pub struct DamiaoChain<P> {
    control: MotorControl<P>,
    motors: Vec<DamiaoAxis>,
    axes: Vec<BoundAxis>,
    commandable: Vec<CommandRange>,
    limits: Vec<MotorLimits>,
    enabled: bool,
    stop_pending: bool,
}

impl<P: CanPort> DamiaoChain<P> {
    /// Validate every axis, bring each motor up as the vendor does, scale each by its own limit
    /// registers, and leave the chain disabled with every motor's disabled answer in hand.
    ///
    /// # Errors
    ///
    /// Refuses an invalid or unqualified axis, a motor that does not confirm MIT mode or whose
    /// limit registers no Damiao table carries, and a motor that does not answer disabled.
    // A register holds the exact float32 of a table literal; equality is the comparison meant.
    #[allow(clippy::float_cmp)]
    pub fn open(port: P, motors: Vec<DamiaoAxis>) -> Result<Self, DamiaoChainError> {
        let axes: Vec<BoundAxis> = motors.iter().map(|motor| motor.axis.clone()).collect();
        validate_chain(&axes)?;
        let mut ids: Vec<u16> = Vec::new();
        for motor in &motors {
            validate(motor, &mut ids)?;
        }
        let mut control = MotorControl::new(port);
        for motor in &motors {
            let binding = motor.axis.binding;
            let model =
                motor_type(binding.model).ok_or(DamiaoChainError::Unqualified(binding.bus_id))?;
            control.add_motor(Motor::new(model, binding.bus_id, motor.master_id));
        }
        let mut limits = Vec::with_capacity(motors.len());
        for motor in &motors {
            let bus_id = motor.axis.binding.bus_id;
            if !control.switch_control_mode(bus_id, ControlType::MIT)? {
                return Err(DamiaoChainError::ModeNotConfirmed(bus_id));
            }
            let model = control.motor(bus_id)?.motor_type;
            let row = model.limit_param();
            let mut found = [0.0_f32; 3];
            for (index, register) in [DmVariable::PMAX, DmVariable::VMAX, DmVariable::TMAX]
                .into_iter()
                .enumerate()
            {
                let accepted = if register == DmVariable::VMAX {
                    accepted_vmax(model)
                } else {
                    vec![row[index]]
                };
                let value = control.read_motor_param(bus_id, register)?;
                match value {
                    Some(ParamValue::Float(reported))
                        if accepted.iter().any(|table| reported == narrow(*table)) =>
                    {
                        found[index] = reported;
                    }
                    _ => {
                        return Err(DamiaoChainError::LimitMismatch {
                            bus_id,
                            register,
                            motor: value,
                        });
                    }
                }
            }
            let reported = found.map(f64::from);
            if reported != row.map(|table| f64::from(narrow(table))) {
                control.change_limit_param(bus_id, reported[0], reported[1], reported[2])?;
            }
            limits.push(MotorLimits {
                bus_id,
                p_max: found[0],
                v_max: found[1],
                t_max: found[2],
            });
        }
        let commandable = motors.iter().map(DamiaoAxis::commandable).collect();
        let mut chain = Self {
            control,
            motors,
            axes,
            commandable,
            limits,
            enabled: false,
            stop_pending: false,
        };
        let before = chain.feedback_counts()?;
        for index in 0..chain.motors.len() {
            chain.control.disable(chain.bus_id(index))?;
        }
        chain.await_answers(&before)?;
        chain.expect_all(&before, MotorState::Disabled)?;
        Ok(chain)
    }

    /// The limit row each motor reported at open, in native state order.
    #[must_use]
    pub fn limits(&self) -> &[MotorLimits] {
        &self.limits
    }

    /// Each motor's latest answer, as cached from its last feedback frame.
    ///
    /// # Errors
    ///
    /// Never for a chain that opened; the lookup is by the chain's own ids.
    pub fn feedback(&self) -> Result<Vec<MotorFeedback>, DamiaoChainError> {
        self.motors
            .iter()
            .map(|motor| {
                let bus_id = motor.axis.binding.bus_id;
                let cached = self.control.motor(bus_id)?;
                Ok(MotorFeedback {
                    bus_id,
                    state: MotorState::from_err(cached.get_error()),
                    position: cached.get_position(),
                    velocity: cached.get_velocity(),
                    torque: cached.get_torque(),
                    mos_celsius: cached.state_t_mos,
                    rotor_celsius: cached.state_t_rotor,
                })
            })
            .collect()
    }

    #[must_use]
    pub const fn port(&self) -> &P {
        self.control.port()
    }

    fn bus_id(&self, index: usize) -> u16 {
        self.motors[index].axis.binding.bus_id
    }

    fn feedback_counts(&self) -> Result<Vec<u64>, DmCanError> {
        self.motors
            .iter()
            .map(|motor| {
                Ok(self
                    .control
                    .motor(motor.axis.binding.bus_id)?
                    .feedback_count)
            })
            .collect()
    }

    fn fresh(&self, before: &[u64]) -> Result<Vec<bool>, DmCanError> {
        Ok(self
            .feedback_counts()?
            .iter()
            .zip(before)
            .map(|(now, then)| now > then)
            .collect())
    }

    /// Read until every motor has answered since `before`, polling a few milliseconds at most.
    fn await_answers(&mut self, before: &[u64]) -> Result<(), DmCanError> {
        for attempt in 0..ANSWER_POLLS {
            if self.fresh(before)?.iter().all(|fresh| *fresh) {
                return Ok(());
            }
            if attempt > 0 {
                self.control.port_mut().sleep(ANSWER_POLL_INTERVAL);
            }
            self.control.recv()?;
        }
        Ok(())
    }

    fn expect_all(&self, before: &[u64], expected: MotorState) -> Result<(), DamiaoChainError> {
        for (index, fresh) in self.fresh(before)?.into_iter().enumerate() {
            let bus_id = self.bus_id(index);
            if !fresh {
                return Err(DamiaoChainError::Silent(bus_id));
            }
            let state = MotorState::from_err(self.control.motor(bus_id)?.get_error());
            if state != expected {
                return Err(DamiaoChainError::UnexpectedState {
                    bus_id,
                    state,
                    expected,
                });
            }
        }
        Ok(())
    }
}

#[allow(clippy::cast_possible_truncation)]
fn narrow(value: f64) -> f32 {
    value as f32
}

fn validate(motor: &DamiaoAxis, ids: &mut Vec<u16>) -> Result<(), DamiaoChainError> {
    let binding = motor.axis.binding;
    let bus_id = binding.bus_id;
    let Some(model) = motor_type(binding.model).filter(|_| binding.bus == DAMIAO_CAN) else {
        return Err(DamiaoChainError::Unqualified(bus_id));
    };
    for id in [bus_id, motor.master_id] {
        if id == 0 || id > MAX_MOTOR_ID || ids.contains(&id) {
            return Err(DamiaoChainError::Ids(bus_id));
        }
        ids.push(id);
    }
    if !((0.0..=KP_MAX).contains(&motor.kp) && (0.0..=KD_MAX).contains(&motor.kd)) {
        return Err(DamiaoChainError::Gains(bus_id));
    }
    let q_max = model.limit_param()[0];
    let range = motor.commandable();
    if !(motor.motor_lower.is_finite()
        && motor.motor_upper.is_finite()
        && -q_max <= motor.motor_lower
        && motor.motor_lower < motor.motor_upper
        && motor.motor_upper <= q_max
        && range.lower < range.upper)
    {
        return Err(DamiaoChainError::MotorRange(bus_id));
    }
    Ok(())
}

impl<P: CanPort> ActuatorChain for DamiaoChain<P> {
    type Error = DamiaoChainError;
    type Feedback = MotorFeedback;

    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    fn commandable(&self) -> &[CommandRange] {
        &self.commandable
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt<MotorFeedback>, DamiaoChainError> {
        if self.stop_pending {
            return Err(DamiaoChainError::StopPending);
        }
        check_targets(&self.axes, joints)?;
        if let Some((motor, joint)) = self
            .motors
            .iter()
            .zip(&self.commandable)
            .zip(joints)
            .find(|((_, range), joint)| !range.admits(**joint))
            .map(|((motor, _), joint)| (motor, joint))
        {
            return Err(DamiaoChainError::Uncommandable {
                joint: motor.axis.joint.clone(),
                value: *joint,
            });
        }
        let targets: Vec<f64> = self
            .motors
            .iter()
            .zip(joints)
            .map(|(motor, joint)| motor.axis.binding.actuator_from_joint(*joint))
            .collect();
        if !self.enabled {
            let before = self.feedback_counts()?;
            for index in 0..self.motors.len() {
                self.control.enable(self.bus_id(index))?;
            }
            self.await_answers(&before)?;
            self.expect_all(&before, MotorState::Enabled)?;
            self.enabled = true;
        }
        let before = self.feedback_counts()?;
        for (index, target) in targets.into_iter().enumerate() {
            let DamiaoAxis { kp, kd, .. } = self.motors[index];
            self.control
                .control_mit(self.bus_id(index), kp, kd, target, 0.0, 0.0)?;
        }
        self.await_answers(&before)?;
        self.expect_all(&before, MotorState::Enabled)?;
        let feedback = self.feedback()?;
        let observed = self
            .motors
            .iter()
            .zip(&feedback)
            .map(|(motor, answer)| {
                motor
                    .axis
                    .binding
                    .joint_from_actuator(f64::from(answer.position))
            })
            .collect();
        Ok(Receipt {
            evidence: Evidence::BusObservedComplete,
            observed,
            feedback,
        })
    }

    fn stop(&mut self) -> StopReport {
        self.enabled = false;
        let Ok(before) = self.feedback_counts() else {
            self.stop_pending = true;
            return StopReport {
                unproven: self.motors.iter().map(|m| m.axis.binding.bus_id).collect(),
                faults: Vec::new(),
            };
        };
        for index in 0..self.motors.len() {
            let _ = self.control.disable(self.bus_id(index));
        }
        let _ = self.await_answers(&before);
        let silent: Vec<u16> = self
            .fresh(&before)
            .unwrap_or_default()
            .into_iter()
            .enumerate()
            .filter(|(_, fresh)| !fresh)
            .map(|(index, _)| self.bus_id(index))
            .collect();
        if !silent.is_empty() {
            for bus_id in silent {
                let _ = self.control.refresh_motor_status(bus_id);
            }
            let _ = self.await_answers(&before);
        }
        let fresh = self.fresh(&before).unwrap_or_default();
        let mut report = StopReport::default();
        for index in 0..self.motors.len() {
            let bus_id = self.bus_id(index);
            let state = self
                .control
                .motor(bus_id)
                .map(|motor| MotorState::from_err(motor.get_error()));
            match (fresh.get(index).copied().unwrap_or(false), state) {
                (true, Ok(MotorState::Disabled)) => {}
                (true, Ok(MotorState::Fault(code))) => {
                    report.faults.push((bus_id, code));
                    report.unproven.push(bus_id);
                }
                _ => report.unproven.push(bus_id),
            }
        }
        self.stop_pending = !report.is_proven();
        report
    }
}
