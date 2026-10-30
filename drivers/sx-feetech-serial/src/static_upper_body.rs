//! The Sentient static upper body's Feetech buses: the SO-101 neck and the two grippers.
//!
//! The axes and their bindings are the registry's (`sx_embodiments/known/static_upper_body.py`,
//! a development body), read from `bindings/sentient-static-upper-body.json`, the same document
//! `sx-damiao-can` composes the body's Damiao bus from, which `tools/render_driver_bindings.py`
//! renders. A read-only probe on 2026-10-25 found each bus on its own CH343 USB serial adapter at
//! 1,000,000 baud, every servo an STS3215:
//!
//! - [`NECK`], port `neck`: ids 1..6 in SO-101 order, the sixth the camera tilt where the
//!   SO-101's gripper servo was. Each servo is set up as [`crate::so101`]'s arm joints are:
//!   `LeRobot`'s `DEGREES` normalization and `SOFollower.configure`'s position loop, the camera
//!   tilt included, since it is a revolute joint and not a jaw; and, beyond the SO-101, an
//!   approach limit on the servo itself, the registry's for the port (the bindings' `approach`,
//!   `STATIC_UPPER_BODY_SERVO_APPROACH` in `sx_embodiments/known/static_upper_body.py`).
//! - [`RIGHT_GRIPPER`] and [`LEFT_GRIPPER`], ports `right_gripper` (ids 1 and 2) and
//!   `left_gripper` (ids 3 and 4), one servo per finger, each set up as the SO-101's gripper is:
//!   `RANGE_0_100` over the finger's declared range, and `SOFollower.configure`'s torque and
//!   current limits, and their port's approach limit from the registry.
//!
//! **Provisional.** Which gripper bus is which side, which id is which finger, that each neck
//! joint's zero is its servo's mid-range, and that each finger's declared range is its
//! servo's calibrated range are not yet checked on the robot. Every servo's `LeRobot`
//! calibration is the unit's: nothing opens these buses without one, [`FeetechBody::open`]
//! takes it per servo and refuses any servo whose registers disagree. The probe found neck ids
//! 2, 3, 4 and 6 resting outside their calibrated ranges: each chain reports where its servos
//! rest ([`sx_embodiment_drivers::ActuatorChain::last_observed`]) so the first command can be
//! slewed from there.

use crate::body::{FeetechBody, ServoSetup};
use crate::chain::{MotorNormMode, ServoConfiguration};
use crate::so101::{GRIPPER_PROTECTION, LEROBOT_BAUD, POSITION_LOOP};

/// The registry's static upper-body axes, rendered.
pub const BINDINGS: &str = include_str!("../bindings/sentient-static-upper-body.json");

const fn degrees(joint: &'static str) -> ServoSetup {
    ServoSetup {
        joint,
        norm_mode: MotorNormMode::Degrees,
        configuration: POSITION_LOOP,
    }
}

const fn finger(joint: &'static str) -> ServoSetup {
    ServoSetup {
        joint,
        norm_mode: MotorNormMode::Range0100,
        configuration: ServoConfiguration {
            protection: Some(GRIPPER_PROTECTION),
            ..POSITION_LOOP
        },
    }
}

/// The neck's six servos on their own bus.
pub const NECK: FeetechBody = FeetechBody {
    embodiment: "sentient-static-upper-body",
    bindings: BINDINGS,
    port: "neck",
    baud: LEROBOT_BAUD,
    registry_approach: true,
    servos: &[
        degrees("so101_shoulder_pan_joint"),
        degrees("so101_shoulder_lift_joint"),
        degrees("so101_elbow_flex_joint"),
        degrees("so101_wrist_flex_joint"),
        degrees("so101_wrist_roll_joint"),
        degrees("so101_camera_tilt_joint"),
    ],
};

/// The right gripper's two fingers on their own bus. Provisional, as the module says.
pub const RIGHT_GRIPPER: FeetechBody = FeetechBody {
    embodiment: "sentient-static-upper-body",
    bindings: BINDINGS,
    port: "right_gripper",
    baud: LEROBOT_BAUD,
    registry_approach: true,
    servos: &[
        finger("right_gripper_left_finger_joint"),
        finger("right_gripper_right_finger_joint"),
    ],
};

/// The left gripper's two fingers on their own bus. Provisional, as the module says.
pub const LEFT_GRIPPER: FeetechBody = FeetechBody {
    embodiment: "sentient-static-upper-body",
    bindings: BINDINGS,
    port: "left_gripper",
    baud: LEROBOT_BAUD,
    registry_approach: true,
    servos: &[
        finger("left_gripper_left_finger_joint"),
        finger("left_gripper_right_finger_joint"),
    ],
};
