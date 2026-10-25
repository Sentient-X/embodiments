//! Feetech STS servos over their half-duplex serial bus, and the SO-101's chain of them.
//!
//! [`scservo_sdk`] is Feetech's own `scservo_sdk` translated; [`chain::FeetechChain`] drives
//! bound STS3215 servos through it as one `sx_embodiment_drivers::ActuatorChain`; a
//! [`body::FeetechBody`] composes any body's servos on one bus from its rendered bindings and
//! setups: [`so101`]'s follower and [`static_upper_body`]'s neck.
//!
//! Consumer: the sx station's `ChainActuator`, over [`so101::open_so101`]. See `README.md` beside
//! this crate for the surface it builds against.

pub mod body;
pub mod chain;
pub mod scan;
pub mod scservo_sdk;
#[cfg(any(test, feature = "sim"))]
pub mod sim;
pub mod so101;
pub mod static_upper_body;

pub use chain::{
    ApproachLimit, BRING_UP_APPROACH, FEETECH_SERIAL, FEETECH_STS3215, FeetechChain,
    FeetechChainError, FeetechServo, MotorCalibration, MotorNormMode, STS3215_MODEL_NUMBER,
    ServoConfiguration, TorqueProtection,
};
