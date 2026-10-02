//! The Seeed Studio reBot B601-DM's seven Damiao motors as one chain.
//!
//! References:
//!
//! - `Seeed-Projects/reBotArm_control_py` at `6415d43130d1e143c70dc106096a857ac5556f81`,
//!   `config/rebotarm_dm.yaml`: `joint1`..`joint6` on `motor_id` 0x01..0x06 answering on
//!   `feedback_id` 0x11..0x16, the gripper on 0x07/0x17; models `4340P` (joints 1..3) and `4310`
//!   (joints 4..6, gripper); the gripper's MIT gains (kp 8, kd 1).
//! - `Seeed-Projects/reBotArmController_ROS2` at `a61efe4fa223ca50cd721ef8ebe4a60e90f28bfd`:
//!   `src/rebotarm_bringup/config/rebotarm_hardware.yaml` (the arm's MIT gains, kp
//!   `[120, 120, 120, 18, 18, 18]`, kd `[8, 8, 8, 2, 2, 2]`, and the gripper motor's
//!   `position_limits`, open -5.0 rad, close 0.0 rad), and
//!   `src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py`
//!   (`_gripper_motor_to_joint_position`: the finger joint is the motor's fraction of its open
//!   angle times half of `_GRIPPER_MAX_WIDTH = 0.09` m, and the arm joints are the motor angles
//!   unchanged).
//!
//! The axes and their bindings are the registry's (`sx_embodiments/known/b601.py`), held equal
//! to `tests/fixtures/b601_bindings.json`, which the registry renders. What is added here is
//! controller configuration, not hardware identity: master ids, gains and the gripper motor's
//! commandable range. The DM4340 joints' kd of 8 is outside the MIT range; `DM_CAN.py` clamps it
//! to 5 on the wire, so 5 is declared.

use sx_embodiment_drivers::{ActuatorBinding, BoundAxis};

use crate::chain::{DAMIAO_CAN, DAMIAO_DM4310, DAMIAO_DM4340, DamiaoAxis};

/// The vendor's answer-id convention: `feedback_id = motor_id + 0x10`.
pub const MASTER_ID_OFFSET: u16 = 0x10;
/// The gripper motor's fully open angle, radians (`position_limits.open`).
pub const GRIPPER_MOTOR_OPEN_RAD: f64 = -5.0;
/// One finger's travel at the motor's open angle: half of `_GRIPPER_MAX_WIDTH`.
pub const GRIPPER_FINGER_OPEN_M: f64 = 0.09 / 2.0;

fn axis(joint: &str, lower: f64, upper: f64, binding: ActuatorBinding) -> BoundAxis {
    BoundAxis {
        joint: joint.to_owned(),
        lower,
        upper,
        binding,
    }
}

fn direct(model: sx_embodiment_drivers::ActuatorModel, bus_id: u16) -> ActuatorBinding {
    ActuatorBinding {
        model,
        bus: DAMIAO_CAN,
        bus_id,
        sign: 1,
        zero_offset: 0.0,
        reduction: 1.0,
    }
}

fn joint(name: &str, bus_id: u16, lower: f64, upper: f64, kp: f64, kd: f64) -> DamiaoAxis {
    let model = if bus_id <= 3 {
        DAMIAO_DM4340
    } else {
        DAMIAO_DM4310
    };
    DamiaoAxis {
        axis: axis(name, lower, upper, direct(model, bus_id)),
        master_id: bus_id + MASTER_ID_OFFSET,
        kp,
        kd,
        // The arm's drive map is identity, so the motor's range is the joint's.
        motor_lower: lower,
        motor_upper: upper,
    }
}

/// `joint1`..`joint6`, in native state order, with the URDF's limits.
#[must_use]
// 3.14 is the URDF's own limit, not an approximation of pi.
#[allow(clippy::approx_constant)]
pub fn b601_arm() -> Vec<DamiaoAxis> {
    vec![
        joint("joint1", 1, -2.8, 2.8, 120.0, 5.0),
        joint("joint2", 2, -3.14, 0.0, 120.0, 5.0),
        joint("joint3", 3, -3.14, 0.0, 120.0, 5.0),
        joint("joint4", 4, -1.87, 1.57, 18.0, 2.0),
        joint("joint5", 5, -1.57, 1.57, 18.0, 2.0),
        joint("joint6", 6, -3.14, 3.14, 18.0, 2.0),
    ]
}

/// `gripper_joint1`, one finger's travel in metres, driven by the 0x07 motor through the
/// vendor's linear transmission: open angle over finger travel at that angle.
#[must_use]
pub fn b601_gripper() -> DamiaoAxis {
    DamiaoAxis {
        axis: axis(
            "gripper_joint1",
            0.0,
            0.0715,
            ActuatorBinding {
                sign: -1,
                reduction: -GRIPPER_MOTOR_OPEN_RAD / GRIPPER_FINGER_OPEN_M,
                ..direct(DAMIAO_DM4310, 7)
            },
        ),
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
