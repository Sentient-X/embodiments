//! The one surface every embodiment actuator driver implements.
//!
//! A driver drives a chain of qualified actuators on one bus. Each actuator is bound to one
//! joint axis by the registry's `ActuatorBinding` (`sx_embodiments/layout.py`): a model on a
//! bus at a chain address, and the drive map `actuator = sign * (joint - zero_offset) *
//! reduction`. The chain's axes are listed in the embodiment's native state order, so a
//! command vector and an observed vector are indexed exactly as `robot.state.coordinates`.
//!
//! [`ActuatorChain`] is that surface: command the joint vector, stop with a report of what
//! each actuator's answer proves. A [`ProvenStop`] exists only when every actuator proved it
//! stopped. Port traits are per bus kind ([`can::CanPort`]); one crate per bus implements the
//! chain over its port, translating its vendor's own driver.
//!
//! Synchronous and allocation-light on purpose: the station wraps a chain in its own async
//! actuator, and nothing here owns a thread or a runtime.

pub mod binding;
pub mod can;
pub mod chain;

pub use binding::{
    ActuatorBinding, ActuatorBus, ActuatorModel, BindingError, BoundAxis, validate_chain,
};
pub use chain::{
    ActuatorChain, CommandRange, Evidence, ProvenStop, Receipt, StopReport, TargetError,
    check_targets,
};
