//! Damiao motors read and never commanded: a [`DamiaoObserver`] polls the motors a
//! [`DamiaoChain`](crate::chain::DamiaoChain) would open, in the same order and joint
//! coordinates, without writing a byte that changes what a motor holds.
//!
//! What it sends, and only that, through [`AskOnlyCanPort`], which refuses anything else before
//! the link sees it:
//!
//! - at open, each motor's `PMAX`, `VMAX` and `TMAX` registers, read (`0x7FF`, `0x33`) as the
//!   chain's open reads them, so each motor's feedback is scaled by what it reports, exactly as
//!   the chain scales it ([`crate::chain`]'s header: one static upper-body shoulder reports a
//!   `PMAX` beside its table's row);
//! - each poll, one status refresh (`0x7FF`, `0xCC`) per motor, the chain's own
//!   [`ActuatorChain::read_positions`] request, decoded by the same code.
//!
//! No mode switch, `TIMEOUT` write, enable, disable, clear or MIT frame is sent, so a motor
//! stays as it was: limp, or held by whatever else drives it. Since nothing is held about a
//! motor's enable state, a fresh answer reporting enabled or disabled is fresh either way; a
//! fault code is faulted.
//!
//! A motor that does not answer its limit registers at open is not refused: it reads missing
//! (corrupt: its scale is unknown) until it answers a poll, when its registers are read again.
//! A motor whose registers hold a row no Damiao table carries reads corrupt for the observer's
//! life, where the chain would refuse to open.
//!
//! [`ActuatorChain::read_positions`]: sx_embodiment_drivers::ActuatorChain::read_positions

use std::io;
use std::time::{Duration, Instant};

use sx_embodiment_drivers::can::{CanFrame, CanPort};
use sx_embodiment_drivers::{Axes, AxisReading, BoundAxis, MissingReason, Observe, validate_chain};

use crate::chain::{
    Awaits, DamiaoAxis, DamiaoChainError, LIMIT_REGISTERS, MotorState, Phase, REGISTER_WAIT,
    Request, Wait, every, held_limits, motor_type, phase, phase_into, validate,
};
use crate::dm_can::{Motor, MotorControl};
use crate::scan;

/// The id register requests and status refreshes are sent to.
const PARAM_ID: u32 = 0x7FF;
/// A status refresh's command byte: ask for one feedback frame.
const REFRESH: u8 = 0xCC;

/// Whether `frame` only asks: a register read ([`scan::is_read_only`]) or a status refresh
/// (`0x7FF`, `0xCC`).
#[must_use]
pub const fn only_asks(frame: &CanFrame) -> bool {
    scan::is_read_only(frame) || (frame.id == PARAM_ID && frame.data[2] == REFRESH)
}

/// A port that carries only [`only_asks`] frames; any other frame is refused before the link
/// sees it.
pub struct AskOnlyCanPort<P>(P);

impl<P> AskOnlyCanPort<P> {
    pub const fn new(port: P) -> Self {
        Self(port)
    }
}

impl<P: CanPort> CanPort for AskOnlyCanPort<P> {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        if !only_asks(frame) {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                "an observer sends only register reads and status refreshes",
            ));
        }
        self.0.send(frame)
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        self.0.receive()
    }

    fn receive_until(&mut self, deadline: Instant) -> io::Result<Option<CanFrame>> {
        self.0.receive_until(deadline)
    }

    fn sleep(&mut self, duration: Duration) {
        self.0.sleep(duration);
    }

    fn frame_allowance(&self) -> Duration {
        self.0.frame_allowance()
    }

    fn counters(&self) -> Vec<(&'static str, u64)> {
        self.0.counters()
    }

    fn health(&self) -> sx_embodiment_drivers::can::LinkHealth {
        self.0.health()
    }

    fn answered(&mut self, round_trip: Duration) {
        self.0.answered(round_trip);
    }

    fn round_trip(&mut self) -> Option<sx_embodiment_drivers::can::RoundTrip> {
        self.0.round_trip()
    }
}

/// How a motor's feedback is scaled.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Scale {
    /// By the limit row it reported.
    Reported,
    /// Not yet known: it did not answer its limit registers.
    Unknown,
    /// Never: it reported a row no Damiao table carries.
    Refused,
}

/// Damiao motors on one CAN link, read only, in native state order.
pub struct DamiaoObserver<P> {
    control: MotorControl<AskOnlyCanPort<P>>,
    motors: Vec<DamiaoAxis>,
    axes: Vec<BoundAxis>,
    scale: Vec<Scale>,
    /// One status refresh per motor, the shape of every poll.
    every_feedback: Vec<Request>,
    /// The poll's buffers, allocated at open and reused every read.
    tick: Phase,
}

impl<P: CanPort> DamiaoObserver<P> {
    /// Validate `motors` as [`DamiaoChain::open`](crate::chain::DamiaoChain::open) does and
    /// read each motor's limit registers.
    ///
    /// # Errors
    ///
    /// Refuses what [`DamiaoChain::open`](crate::chain::DamiaoChain::open) refuses before a
    /// frame (an invalid or unqualified axis, ids or a range the protocol cannot carry), and the link's failure.
    pub fn open(port: P, motors: Vec<DamiaoAxis>) -> Result<Self, DamiaoChainError> {
        let axes: Vec<BoundAxis> = motors.iter().map(|motor| motor.axis.clone()).collect();
        validate_chain(&axes)?;
        let mut ids = Vec::new();
        for motor in &motors {
            validate(motor, &mut ids)?;
        }
        let mut control = MotorControl::new(AskOnlyCanPort::new(port));
        for motor in &motors {
            let binding = motor.axis.binding;
            let model =
                motor_type(binding.model).ok_or(DamiaoChainError::Unqualified(binding.bus_id))?;
            control.add_motor(Motor::new(model, binding.bus_id, motor.master_id));
        }
        let every_feedback = every(
            &motors
                .iter()
                .map(|motor| motor.axis.binding.bus_id)
                .collect::<Vec<_>>(),
            Awaits::Feedback,
        );
        let mut observer = Self {
            control,
            tick: Phase::with_capacity(motors.len()),
            every_feedback,
            scale: vec![Scale::Unknown; motors.len()],
            motors,
            axes,
        };
        for index in 0..observer.motors.len() {
            observer.scale(index)?;
        }
        Ok(observer)
    }

    /// Read motor `index`'s limit registers and scale it by them.
    fn scale(&mut self, index: usize) -> Result<(), DamiaoChainError> {
        let bus_id = self.motors[index].axis.binding.bus_id;
        let reads: Vec<Request> = LIMIT_REGISTERS
            .into_iter()
            .map(|register| Request {
                bus_id,
                awaits: Awaits::Register(register),
            })
            .collect();
        phase(
            &mut self.control,
            &reads,
            REGISTER_WAIT,
            |control, at, request| control.send_read_param(request.bus_id, LIMIT_REGISTERS[at]),
        )
        .checked()?;
        self.scale[index] = match held_limits(&mut self.control, bus_id) {
            Ok(_) => Scale::Reported,
            Err(DamiaoChainError::LimitMismatch { motor: None, .. }) => Scale::Unknown,
            Err(DamiaoChainError::LimitMismatch { .. }) => Scale::Refused,
            Err(error) => return Err(error),
        };
        Ok(())
    }

    /// One refresh phase, decoded as the chain decodes it, either enable state accepted.
    fn poll(&mut self) -> Axes<AxisReading> {
        phase_into(
            &mut self.control,
            &self.every_feedback,
            Wait::Tick,
            |control, _, request| control.send_refresh(request.bus_id),
            &mut self.tick,
        );
        let asked = &self.tick;
        let link_failed = asked.failure.is_some();
        (0..self.every_feedback.len())
            .map(|index| {
                let Ok(cached) = self.control.motor(self.every_feedback[index].bus_id) else {
                    return AxisReading::Missing(MissingReason::Unavailable);
                };
                let Some(at) = asked.received[index] else {
                    return AxisReading::Missing(if asked.sent[index].is_none() || link_failed {
                        MissingReason::Unavailable
                    } else if asked.before[index] > 0 {
                        MissingReason::Stale
                    } else {
                        MissingReason::Timeout
                    });
                };
                let code = cached.get_error();
                if let MotorState::Fault(_) = MotorState::from_err(code) {
                    return AxisReading::Faulted(code);
                }
                AxisReading::Fresh {
                    position: self.motors[index]
                        .axis
                        .joint_from_actuator(f64::from(cached.get_position())),
                    received: at,
                }
            })
            .collect()
    }

    #[must_use]
    pub const fn port(&self) -> &P {
        &self.control.port().0
    }
}

impl<P: CanPort> Observe for DamiaoObserver<P> {
    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    /// One status refresh per motor, decoded as [`DamiaoChain`](crate::chain::DamiaoChain)
    /// decodes it; a motor whose scale is unknown or refused is missing (corrupt). A motor of unknown scale that answers
    /// has its limit registers read, and once scaled is polled again before this returns.
    fn read_positions(&mut self) -> Axes<AxisReading> {
        let mut readings = self.poll();
        let mut rescaled = false;
        for (index, reading) in readings.iter_mut().enumerate() {
            if self.scale[index] == Scale::Unknown && !matches!(reading, AxisReading::Missing(_)) {
                if self.scale(index).is_err() {
                    *reading = AxisReading::Missing(MissingReason::Unavailable);
                    continue;
                }
                rescaled |= self.scale[index] == Scale::Reported;
            }
            if self.scale[index] != Scale::Reported && reading.is_fresh() {
                *reading = AxisReading::Missing(MissingReason::Corrupt);
            }
        }
        if rescaled {
            return self.read_positions();
        }
        readings
    }
}
