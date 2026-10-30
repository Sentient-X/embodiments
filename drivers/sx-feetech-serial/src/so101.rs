//! The SO-101 follower's six Feetech STS3215 servos as one chain.
//!
//! The axes and their bindings are the registry's (`sx_embodiments/known/so101.py`), read from
//! `bindings/so101.json`, which `tools/render_driver_bindings.py` renders; no bound, address or
//! drive map is spelled here. What is added here is
//! each servo's wire normalization, from `huggingface/lerobot` at `v0.6.0`
//! (`30da8e687a6dfc617fcd94afc367ac7071c376ce`),
//! `src/lerobot/robots/so_follower/so_follower.py:50-59`: ids 1 to 5 in `DEGREES` (its
//! `use_degrees` default), the gripper on id 6 in `RANGE_0_100`; and each servo's
//! configuration, from `SOFollower.configure` (`so_follower.py:159-173`): P 16, I 0, D 32 on
//! every servo, and on the gripper a torque limit of 500, a protection current of 250 and an
//! overload torque of 25. The calibration is per unit, so the caller supplies it, in native
//! state order. The body is one [`FeetechBody`], [`SO101`].

use sx_embodiment_drivers::BodyBindings;
use sx_embodiment_drivers::serial::SerialPort;

use crate::body::{FeetechBody, ServoSetup};
use crate::chain::{
    FeetechChain, FeetechChainError, FeetechServo, MotorCalibration, MotorNormMode,
    ServoConfiguration, TorqueProtection,
};

/// The registry's SO-101 axes, rendered.
pub const BINDINGS: &str = include_str!("../bindings/so101.json");

/// `FeetechMotorsBus.default_baudrate` (`src/lerobot/motors/feetech/feetech.py:44`).
pub const LEROBOT_BAUD: u32 = 1_000_000;

/// The position loop on every servo: P lowered from 32 "to avoid shakiness", I and D at their
/// defaults.
pub const POSITION_LOOP: ServoConfiguration = ServoConfiguration {
    p_coefficient: 16,
    i_coefficient: 0,
    d_coefficient: 32,
    protection: None,
    approach: None,
};
/// The gripper's limits: half of its torque and current, and 25 % torque when overloaded.
pub const GRIPPER_PROTECTION: TorqueProtection = TorqueProtection {
    max_torque_limit: 500,
    protection_current: 250,
    overload_torque: 25,
};

const fn arm(joint: &'static str) -> ServoSetup {
    ServoSetup {
        joint,
        norm_mode: MotorNormMode::Degrees,
        configuration: POSITION_LOOP,
    }
}

/// `SOFollower`'s motor table, in native state order.
pub const SO101: FeetechBody = FeetechBody {
    embodiment: "so101",
    bindings: BINDINGS,
    port: "serial",
    baud: LEROBOT_BAUD,
    registry_approach: false,
    servos: &[
        arm("shoulder_pan"),
        arm("shoulder_lift"),
        arm("elbow_flex"),
        arm("wrist_flex"),
        arm("wrist_roll"),
        ServoSetup {
            joint: "gripper",
            norm_mode: MotorNormMode::Range0100,
            configuration: ServoConfiguration {
                protection: Some(GRIPPER_PROTECTION),
                ..POSITION_LOOP
            },
        },
    ],
};

/// The SO-101's arm and jaw, calibrated by `calibration` in native state order.
///
/// # Errors
///
/// Refuses rendered bindings whose joints are not `SOFollower`'s six.
pub fn so101(calibration: [MotorCalibration; 6]) -> Result<Vec<FeetechServo>, FeetechChainError> {
    SO101.servos(SO101.axes(BodyBindings::parse(BINDINGS)?)?, &calibration)
}

/// Open the SO-101 follower's chain on `port`, calibrated by `calibration`.
///
/// # Errors
///
/// Returns why the chain did not open.
pub fn open_so101<P: SerialPort>(
    port: P,
    calibration: [MotorCalibration; 6],
) -> Result<FeetechChain<P>, FeetechChainError> {
    FeetechChain::open(port, so101(calibration)?)
}
