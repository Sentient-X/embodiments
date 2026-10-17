//! Damiao DM-J motors over CAN, and the bodies built of them: the reBot B601-DM and the Sentient
//! humanoid upper body.
//!
//! [`dm_can`] is Damiao's own `DM_CAN.py` translated; [`usb2can`] its USB2CAN serial framing as
//! a `CanPort`; [`chain::DamiaoChain`] drives bound motors through it as one
//! `sx_embodiment_drivers::ActuatorChain`; [`b601`] composes the B601-DM's seven motors and
//! [`humanoid`] the humanoid upper body's eleven, each calibrated from a robot instance's drive.
//!
//! Consumer: the sx station's `ChainActuator`, over [`b601::open_b601`] and
//! [`humanoid::open_humanoid_upper_body`]. See `README.md` beside
//! this crate for the surface the station builds against.

pub mod b601;
pub mod chain;
pub mod dm_can;
pub mod humanoid;
#[cfg(any(test, feature = "sim"))]
pub mod sim;
pub mod usb2can;

pub use chain::{
    DAMIAO_CAN, DAMIAO_DM4310, DAMIAO_DM4340, DAMIAO_DM8009, DamiaoAxis, DamiaoChain,
    DamiaoChainError, MotorFeedback, MotorLimits, MotorState,
};
