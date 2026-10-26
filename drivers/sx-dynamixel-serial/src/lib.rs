//! Dynamixel X-series servos over their half-duplex serial bus, Protocol 2.0.
//!
//! [`dynamixel_sdk`] is ROBOTIS's own `DynamixelSDK` packet handler translated;
//! [`chain::DynamixelChain`] drives bound servos through it as one
//! `sx_embodiment_drivers::ActuatorChain`; a [`body::DynamixelBody`] composes any body's servos
//! on one bus from its rendered bindings and setups. **Untested on the bench**: see
//! [`chain`] for what qualifies a model.
//!
//! Consumer: `sx-embodiment-bodies`, which opens a body's Dynamixel bus beside its others. See
//! `README.md` beside this crate for the surface it builds against.

pub mod body;
pub mod chain;
pub mod dynamixel_sdk;
pub mod observe;

pub use chain::{
    DYNAMIXEL_SERIAL, DYNAMIXEL_XL330_M288, DYNAMIXEL_XM430_W350, DynamixelChain,
    DynamixelChainError, DynamixelServo, MotorCalibration, MotorNormMode, OperatingMode,
    model_number,
};
