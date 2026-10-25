//! The Sentient static upper body's neck: an SO-101 on the chest, six Feetech STS3215s.
//!
//! **Provisional until the robot is probed.** The axes and their bindings are the registry's
//! (`sx_embodiments/known/static_upper_body.py`, a development body), read from
//! `bindings/sentient-static-upper-body.json`, the same document `sx-damiao-can` composes the
//! body's Damiao bus from, which `tools/render_driver_bindings.py` renders. The registry binds
//! the neck as an SO-101 is wired, ids 1..6 on a bus of its own named `neck`, its sixth joint
//! the camera tilt where the SO-101's gripper servo was; no probe has read those ids. Each
//! servo is set up as [`crate::so101`]'s arm joints are: `LeRobot`'s `DEGREES` normalization and
//! `SOFollower.configure`'s position loop, the camera tilt included, since it is a revolute
//! joint and not a jaw. Each servo's `LeRobot` calibration is unknown, and nothing opens the
//! neck without one: [`FeetechBody::open`] takes it per servo and refuses any servo whose
//! registers disagree. Whether each joint's zero is its servo's mid-range, as the degree
//! normalization assumes, is for that calibration to settle.

use crate::body::{FeetechBody, ServoSetup};
use crate::chain::MotorNormMode;
use crate::so101::POSITION_LOOP;

/// The registry's static upper-body axes, rendered; the neck's are its Feetech axes.
pub const BINDINGS: &str = include_str!("../bindings/sentient-static-upper-body.json");

const fn degrees(joint: &'static str) -> ServoSetup {
    ServoSetup {
        joint,
        norm_mode: MotorNormMode::Degrees,
        configuration: POSITION_LOOP,
    }
}

/// The neck's six servos on their own bus.
pub const NECK: FeetechBody = FeetechBody {
    bindings: BINDINGS,
    servos: &[
        degrees("so101_shoulder_pan_joint"),
        degrees("so101_shoulder_lift_joint"),
        degrees("so101_elbow_flex_joint"),
        degrees("so101_wrist_flex_joint"),
        degrees("so101_wrist_roll_joint"),
        degrees("so101_camera_tilt_joint"),
    ],
};
