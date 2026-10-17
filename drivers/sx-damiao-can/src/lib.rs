//! Damiao DM-J motors over CAN, and the reBot B601-DM's chain of them.
//!
//! [`dm_can`] is Damiao's own `DM_CAN.py` translated; [`usb2can`] its USB2CAN serial framing as
//! a `CanPort`; [`chain::DamiaoChain`] drives bound motors through it as one
//! `sx_embodiment_drivers::ActuatorChain`; [`b601`] composes the B601-DM's seven motors.
//!
//! Consumer: the sx station's `ChainActuator`, over [`b601::open_b601`]. See `README.md` beside
//! this crate for the surface the station builds against.

pub mod b601;
pub mod chain;
pub mod dm_can;
pub mod usb2can;

pub use chain::{
    DAMIAO_CAN, DAMIAO_DM4310, DAMIAO_DM4340, DamiaoAxis, DamiaoChain, DamiaoChainError,
    MotorFeedback, MotorLimits, MotorState,
};
