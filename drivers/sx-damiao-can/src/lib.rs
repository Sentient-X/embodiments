//! Damiao DM-J motors over CAN, and the bodies built of them: the reBot B601-DM, and the Sentient
//! humanoid and static upper bodies.
//!
//! [`dm_can`] is Damiao's own `DM_CAN.py` translated; [`usb2can`] its USB2CAN serial framing as
//! a `CanPort`; [`chain::DamiaoChain`] drives bound motors through it as one
//! `sx_embodiment_drivers::ActuatorChain`; [`b601`] composes the B601-DM's seven motors, and a
//! [`body::DamiaoBody`] any body on one bus from its rendered bindings and gains: [`humanoid`]'s
//! eleven and [`static_upper_body`]'s fifteen, each calibrated from a robot instance's drive.
//!
//! Consumer: the sx station's `ChainActuator`, over [`b601::open_b601`] and
//! [`humanoid::open_humanoid_upper_body`] and [`body::DamiaoBody::open`]. See `README.md` beside
//! this crate for the surface the station builds against.

pub mod b601;
pub mod body;
pub mod chain;
pub mod dm_can;
pub mod humanoid;
pub mod observe;
pub mod scan;
#[cfg(any(test, feature = "sim"))]
pub mod sim;
pub mod static_upper_body;
pub mod usb2can;

pub use chain::{
    DAMIAO_CAN, DAMIAO_DM4310, DAMIAO_DM4340, DAMIAO_DM8009, DamiaoAxis, DamiaoChain,
    DamiaoChainError, MotorFeedback, MotorLimits, MotorState,
};
