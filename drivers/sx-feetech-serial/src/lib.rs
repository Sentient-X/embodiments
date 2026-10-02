//! Feetech STS servos over their half-duplex serial bus, and the SO-101's chain of them.
//!
//! [`scservo_sdk`] is Feetech's own `scservo_sdk` translated; [`chain::FeetechChain`] drives
//! bound STS3215 servos through it as one `sx_embodiment_drivers::ActuatorChain`; [`so101`]
//! composes the SO-101 follower's six servos.
//!
//! Consumer: the sx station's `FeetechActuator` (`fleet/station/src/feetech.rs`), a thin async
//! wrapper over [`chain::FeetechChain`]. See `README.md` beside this crate for the surface it
//! builds against.

pub mod chain;
pub mod scservo_sdk;
pub mod so101;

pub use chain::{
    FEETECH_SERIAL, FEETECH_STS3215, FeetechChain, FeetechChainError, FeetechServo,
    MotorCalibration, MotorNormMode, STS3215_MODEL_NUMBER,
};
