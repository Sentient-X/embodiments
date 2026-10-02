//! The SO-101 follower's six Feetech STS3215 servos as one chain.
//!
//! The axes and their bindings are the registry's (`sx_embodiments/known/so101.py`), held equal
//! to `tests/fixtures/so101_bindings.json`, which the registry renders. What is added here is
//! each servo's wire normalization, from `huggingface/lerobot` at `v0.6.0`
//! (`30da8e687a6dfc617fcd94afc367ac7071c376ce`),
//! `src/lerobot/robots/so_follower/so_follower.py:50-59`: ids 1 to 5 in `DEGREES` (its
//! `use_degrees` default), the gripper on id 6 in `RANGE_0_100`; and each servo's
//! configuration, from `SOFollower.configure` (`so_follower.py:159-173`): P 16, I 0, D 32 on
//! every servo, and on the gripper a torque limit of 500, a protection current of 250 and an
//! overload torque of 25. The calibration is per unit, so the caller supplies it, in native
//! state order.

use sx_embodiment_drivers::{ActuatorBinding, BoundAxis};

use crate::chain::{
    FEETECH_SERIAL, FEETECH_STS3215, FeetechServo, MotorCalibration, MotorNormMode,
    ServoConfiguration, TorqueProtection,
};

/// The position loop on every servo: P lowered from 32 "to avoid shakiness", I and D at their
/// defaults.
const POSITION_LOOP: ServoConfiguration = ServoConfiguration {
    p_coefficient: 16,
    i_coefficient: 0,
    d_coefficient: 32,
    protection: None,
};
/// The gripper's limits: half of its torque and current, and 25 % torque when overloaded.
const GRIPPER_PROTECTION: TorqueProtection = TorqueProtection {
    max_torque_limit: 500,
    protection_current: 250,
    overload_torque: 25,
};

/// (joint, lower, upper, bus id, normalization), in native state order. The bounds are
/// spelled as `known/so101.py` spells them.
#[allow(clippy::unreadable_literal)]
const SO101: [(&str, f64, f64, u16, MotorNormMode); 6] = [
    ("shoulder_pan", -1.91986, 1.91986, 1, MotorNormMode::Degrees),
    (
        "shoulder_lift",
        -1.74533,
        1.74533,
        2,
        MotorNormMode::Degrees,
    ),
    ("elbow_flex", -1.69, 1.69, 3, MotorNormMode::Degrees),
    ("wrist_flex", -1.65806, 1.65806, 4, MotorNormMode::Degrees),
    ("wrist_roll", -2.74385, 2.84121, 5, MotorNormMode::Degrees),
    ("gripper", -0.174533, 2.0944, 6, MotorNormMode::Range0100),
];

/// The SO-101's arm and jaw, calibrated by `calibration` in native state order.
#[must_use]
pub fn so101(calibration: [MotorCalibration; 6]) -> Vec<FeetechServo> {
    SO101
        .iter()
        .zip(calibration)
        .map(
            |(&(joint, lower, upper, bus_id, norm_mode), calibration)| FeetechServo {
                axis: BoundAxis {
                    joint: joint.to_owned(),
                    lower,
                    upper,
                    binding: ActuatorBinding {
                        model: FEETECH_STS3215,
                        bus: FEETECH_SERIAL,
                        bus_id,
                        sign: 1,
                        zero_offset: 0.0,
                        reduction: 1.0,
                    },
                },
                norm_mode,
                calibration,
                configuration: ServoConfiguration {
                    protection: (norm_mode == MotorNormMode::Range0100)
                        .then_some(GRIPPER_PROTECTION),
                    ..POSITION_LOOP
                },
            },
        )
        .collect()
}
