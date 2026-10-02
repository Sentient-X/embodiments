//! A chain of bound Damiao motors as one `ActuatorChain`, driven through [`crate::dm_can`].
//!
//! Every bus operation is a `DM_CAN.py` call in the vendor's own order. Open runs the vendor
//! test's bring-up for each motor (`DM_Motor_Test.py`: `switchControlMode`, then
//! `read_motor_param` of `PMAX`, `VMAX` and `TMAX`), then `disable`s it. Command `enable`s each
//! motor when the chain is disabled, then sends one `controlMIT` per motor holding its target with
//! zero velocity and feed-forward torque. Stop `disable`s every motor.
//!
//! What this chain adds over the vendor's calls, and why:
//!
//! - It refuses before any frame: a target outside the joint's declared bounds or the motor's
//!   range, ids or gains the protocol cannot carry, a model or bus this crate is not qualified
//!   for. `DM_CAN.py` clamps silently.
//! - It refuses a motor whose `PMAX`, `VMAX` or `TMAX` register differs from the limit row its
//!   frames are scaled by, since a mismatch rescales every position, velocity and torque on the
//!   wire. Damiao's tables disagree for the DM4340 (`DM_CAN.py` 10 rad/s, Damiao's C++ SDK
//!   8 rad/s); the motor's own register decides.
//! - After the vendor's own read it waits briefly for each motor's answer, and judges it: the
//!   feedback's first-byte high nibble is the motor's state (0 disabled, 1 enabled, 8 over-voltage,
//!   9 under-voltage, 0xA over-current, 0xB MOS over-temperature, 0xC rotor over-temperature,
//!   0xD communication lost, 0xE overload; Damiao's manual, and `status_name` in the
//!   `motorbridge` driver Seeed's controller runs). `DM_CAN.py` stores the nibble unread.
//! - A stop is proven motor by motor: a fresh answer reporting disabled. A silent motor is asked
//!   once more with `refresh_motor_status`. After an unproven stop every command is refused
//!   until a stop is proven.

use std::time::Duration;

use sx_embodiment_drivers::can::CanPort;
use sx_embodiment_drivers::{
    ActuatorBus, ActuatorChain, ActuatorModel, BindingError, BoundAxis, Evidence, Receipt,
    StopReport, TargetError, check_targets, validate_chain,
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

/// The state a feedback frame's first-byte high nibble reports.
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
    #[error("joint {joint} target maps to motor angle {value} rad, outside the motor's range")]
    MotorOutOfRange { joint: String, value: f64 },
    #[error("bus_id {0} did not confirm MIT control mode")]
    ModeNotConfirmed(u16),
    #[error("bus_id {bus_id} reports {register:?} = {motor:?}; its frames are scaled by {table}")]
    LimitMismatch {
        bus_id: u16,
        register: DmVariable,
        motor: Option<ParamValue>,
        table: f32,
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
    enabled: bool,
    stop_pending: bool,
}

impl<P: CanPort> DamiaoChain<P> {
    /// Validate every axis, bring each motor up as the vendor does, and leave the chain
    /// disabled with every motor's disabled answer in hand.
    ///
    /// # Errors
    ///
    /// Refuses an invalid or unqualified axis, a motor that does not confirm MIT mode or whose
    /// limit registers differ from its table, and a motor that does not answer disabled.
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
        for motor in &motors {
            let bus_id = motor.axis.binding.bus_id;
            if !control.switch_control_mode(bus_id, ControlType::MIT)? {
                return Err(DamiaoChainError::ModeNotConfirmed(bus_id));
            }
            let row = control.motor(bus_id)?.motor_type.limit_param();
            for (register, table) in [DmVariable::PMAX, DmVariable::VMAX, DmVariable::TMAX]
                .into_iter()
                .zip(row)
            {
                #[allow(clippy::cast_possible_truncation)]
                let table = table as f32;
                let value = control.read_motor_param(bus_id, register)?;
                if value != Some(ParamValue::Float(table)) {
                    return Err(DamiaoChainError::LimitMismatch {
                        bus_id,
                        register,
                        motor: value,
                        table,
                    });
                }
            }
        }
        let mut chain = Self {
            control,
            motors,
            axes,
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

    /// The motors' latest states and joint coordinates, as cached from their last answers.
    ///
    /// # Errors
    ///
    /// Never for a chain that opened; the lookup is by the chain's own ids.
    pub fn states(&self) -> Result<Vec<(MotorState, f64)>, DamiaoChainError> {
        self.motors
            .iter()
            .map(|motor| {
                let cached = self.control.motor(motor.axis.binding.bus_id)?;
                Ok((
                    MotorState::from_err(cached.get_error()),
                    motor
                        .axis
                        .binding
                        .joint_from_actuator(f64::from(cached.get_position())),
                ))
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
    if !(motor.motor_lower.is_finite()
        && motor.motor_upper.is_finite()
        && -q_max <= motor.motor_lower
        && motor.motor_lower < motor.motor_upper
        && motor.motor_upper <= q_max)
    {
        return Err(DamiaoChainError::MotorRange(bus_id));
    }
    Ok(())
}

impl<P: CanPort> ActuatorChain for DamiaoChain<P> {
    type Error = DamiaoChainError;

    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt, DamiaoChainError> {
        if self.stop_pending {
            return Err(DamiaoChainError::StopPending);
        }
        check_targets(&self.axes, joints)?;
        let targets = self
            .motors
            .iter()
            .zip(joints)
            .map(|(motor, joint)| {
                let value = motor.axis.binding.actuator_from_joint(*joint);
                if (motor.motor_lower..=motor.motor_upper).contains(&value) {
                    Ok(value)
                } else {
                    Err(DamiaoChainError::MotorOutOfRange {
                        joint: motor.axis.joint.clone(),
                        value,
                    })
                }
            })
            .collect::<Result<Vec<f64>, _>>()?;
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
        Ok(Receipt {
            evidence: Evidence::BusObservedComplete,
            observed: self.states()?.into_iter().map(|(_, joint)| joint).collect(),
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
