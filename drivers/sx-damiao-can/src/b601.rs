//! The Seeed Studio reBot B601-DM's seven Damiao motors as one chain.
//!
//! References:
//!
//! - `Seeed-Projects/reBotArm_control_py` at `6415d43130d1e143c70dc106096a857ac5556f81`,
//!   `config/rebotarm_dm.yaml`: `joint1`..`joint6` on `motor_id` 0x01..0x06 answering on
//!   `feedback_id` 0x11..0x16, the gripper on 0x07/0x17; models `4340P` (joints 1..3) and `4310`
//!   (joints 4..6, gripper); every motor's MIT gains (kp 120 / kd 8 on the 4340Ps, 18 / 2 on the
//!   arm's 4310s, 8 / 1 on the gripper) — the one source of gains here.
//! - `Seeed-Projects/reBotArmController_ROS2` at `a61efe4fa223ca50cd721ef8ebe4a60e90f28bfd`:
//!   `src/rebotarm_bringup/config/rebotarm_hardware.yaml` (the gripper motor's
//!   `position_limits`, open -5.0 rad, close 0.0 rad), and
//!   `src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py:7-17`
//!   (`_gripper_motor_to_joint_position`: the finger joint is the motor's fraction of its open
//!   angle times half of `_GRIPPER_MAX_WIDTH = 0.09` m, saturating there; the arm joints are the
//!   motor angles unchanged).
//! - `huggingface/lerobot` at `ff71cae1ae2d09fd035553c35da65888ed6c8304`,
//!   `src/lerobot/robots/rebot_b601_follower/motor_family.py`, `DM_PROFILE.joint_limits`: the
//!   deployed driver's soft box, in degrees.
//!
//! The axes and their bindings are the registry's (`sx_embodiments/known/b601.py`), held equal
//! to `tests/fixtures/b601_bindings.json`, which the registry renders. What is added here is
//! controller configuration, not hardware identity: master ids, gains and each motor's
//! commandable range. The DM4340 joints' kd of 8 is outside the MIT range; `DM_CAN.py`'s
//! `controlMIT` clamps it to 5 on the wire (`float_to_uint(kd, 0, 5, 12)`), so 5 is declared.
//!
//! Each arm motor's range is the conservative intersection `known/b601.py` asks a live
//! consumer to take, taken here once: the URDF's limits intersected with the soft box. The
//! gripper motor's is its open angle, which the transmission maps to 0..0.045 m of the URDF's
//! 0.0715 m finger stroke. [`DamiaoAxis::commandable`] gives both as joint ranges.

use sx_embodiment_drivers::{ActuatorBinding, ActuatorModel, BoundAxis};

use crate::chain::{DAMIAO_CAN, DAMIAO_DM4310, DAMIAO_DM4340, DamiaoAxis};

/// The vendor's answer-id convention: `feedback_id = motor_id + 0x10`.
pub const MASTER_ID_OFFSET: u16 = 0x10;
/// The gripper motor's fully open angle, radians (`position_limits.open`).
pub const GRIPPER_MOTOR_OPEN_RAD: f64 = -5.0;
/// One finger's travel at the motor's open angle: half of `_GRIPPER_MAX_WIDTH`.
pub const GRIPPER_FINGER_OPEN_M: f64 = 0.09 / 2.0;

fn direct(model: ActuatorModel, bus_id: u16) -> ActuatorBinding {
    ActuatorBinding {
        model,
        bus: DAMIAO_CAN,
        bus_id,
        sign: 1,
        zero_offset: 0.0,
        reduction: 1.0,
    }
}

/// One arm joint: name, motor id, URDF limits (rad), soft box (deg), kp, kd.
type Joint = (&'static str, u16, (f64, f64), (f64, f64), f64, f64);

// 3.14 is the URDF's own limit, not an approximation of pi.
#[allow(clippy::approx_constant)]
const ARM: [Joint; 6] = [
    ("joint1", 1, (-2.8, 2.8), (-150.0, 150.0), 120.0, 5.0),
    ("joint2", 2, (-3.14, 0.0), (-200.0, 1.0), 120.0, 5.0),
    ("joint3", 3, (-3.14, 0.0), (-200.0, 1.0), 120.0, 5.0),
    ("joint4", 4, (-1.87, 1.57), (-80.0, 90.0), 18.0, 2.0),
    ("joint5", 5, (-1.57, 1.57), (-90.0, 90.0), 18.0, 2.0),
    ("joint6", 6, (-3.14, 3.14), (-90.0, 90.0), 18.0, 2.0),
];

fn joint(&(name, bus_id, (lower, upper), (soft_lower, soft_upper), kp, kd): &Joint) -> DamiaoAxis {
    let model = if bus_id <= 3 {
        DAMIAO_DM4340
    } else {
        DAMIAO_DM4310
    };
    DamiaoAxis {
        axis: BoundAxis {
            joint: name.to_owned(),
            lower,
            upper,
            binding: direct(model, bus_id),
        },
        master_id: bus_id + MASTER_ID_OFFSET,
        kp,
        kd,
        // The arm's drive map is identity, so the motor's range is the joint's.
        motor_lower: lower.max(soft_lower.to_radians()),
        motor_upper: upper.min(soft_upper.to_radians()),
    }
}

/// `joint1`..`joint6`, in native state order, with the URDF's limits.
#[must_use]
pub fn b601_arm() -> Vec<DamiaoAxis> {
    ARM.iter().map(joint).collect()
}

/// `gripper_joint1`, one finger's travel in metres, driven by the 0x07 motor through the
/// vendor's linear transmission: open angle over finger travel at that angle.
#[must_use]
pub fn b601_gripper() -> DamiaoAxis {
    DamiaoAxis {
        axis: BoundAxis {
            joint: "gripper_joint1".to_owned(),
            lower: 0.0,
            upper: 0.0715,
            binding: ActuatorBinding {
                sign: -1,
                reduction: -GRIPPER_MOTOR_OPEN_RAD / GRIPPER_FINGER_OPEN_M,
                ..direct(DAMIAO_DM4310, 7)
            },
        },
        master_id: 7 + MASTER_ID_OFFSET,
        kp: 8.0,
        kd: 1.0,
        motor_lower: GRIPPER_MOTOR_OPEN_RAD,
        motor_upper: 0.0,
    }
}

/// The single follower's whole chain: the arm, then the gripper, as `b601-dm`'s state order.
#[must_use]
pub fn b601() -> Vec<DamiaoAxis> {
    let mut chain = b601_arm();
    chain.push(b601_gripper());
    chain
}
