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
//! The axes and their bindings are the registry's (`sx_embodiments/known/b601.py`), read from
//! `bindings/b601-dm.json`, which `tools/render_driver_bindings.py` renders; no bound, address,
//! model, sign or reduction is spelled here, the gripper's transmission included. What is added
//! here is controller configuration, not hardware identity: master ids, gains and each motor's
//! commandable range, keyed by joint name. The DM4340 joints' kd of 8 is outside the MIT range; `DM_CAN.py`'s
//! `controlMIT` clamps it to 5 on the wire (`float_to_uint(kd, 0, 5, 12)`), so 5 is declared.
//!
//! Each arm motor's range is the conservative intersection `known/b601.py` asks a live
//! consumer to take, taken here once: the URDF's limits intersected with the soft box. The
//! gripper motor's is its open angle, which the transmission maps to 0..0.045 m of the URDF's
//! 0.0715 m finger stroke. [`DamiaoAxis::commandable`] gives both as joint ranges.
//!
//! [`open_b601`] opens the follower as one [`Composite`]: the arm's chain, then the gripper's,
//! each on its own `CanPort` (on the stock B601-DM, clones of one
//! `sx_embodiment_drivers::can::SharedCanPort` behind its one USB2CAN bridge), calibrated from
//! one built robot's [`InstanceDrive`].

use sx_embodiment_drivers::can::CanPort;
use sx_embodiment_drivers::{BodyBindings, BodyError, BoundAxis, Composite, InstanceDrive};

use crate::chain::{DamiaoAxis, DamiaoChain, DamiaoChainError};

/// The registry's B601-DM axes, rendered.
pub const BINDINGS: &str = include_str!("../bindings/b601-dm.json");

/// The registry name of the body these bindings are.
pub const EMBODIMENT: &str = "b601-dm";

/// The vendor's answer-id convention: `feedback_id = motor_id + 0x10`.
pub const MASTER_ID_OFFSET: u16 = 0x10;
/// The gripper motor's fully open angle, radians (`position_limits.open`).
pub const GRIPPER_MOTOR_OPEN_RAD: f64 = -5.0;
/// One finger's travel at the motor's open angle: half of `_GRIPPER_MAX_WIDTH`. The registry's
/// gripper reduction is this transmission; the bindings test holds the two equal.
pub const GRIPPER_FINGER_OPEN_M: f64 = 0.09 / 2.0;
/// The finger joint the gripper motor drives.
pub const GRIPPER_JOINT: &str = "gripper_joint1";

/// One arm joint's controller configuration: name, soft box (deg), kp, kd.
type Joint = (&'static str, (f64, f64), f64, f64);

const ARM: [Joint; 6] = [
    ("joint1", (-150.0, 150.0), 120.0, 5.0),
    ("joint2", (-200.0, 1.0), 120.0, 5.0),
    ("joint3", (-200.0, 1.0), 120.0, 5.0),
    ("joint4", (-80.0, 90.0), 18.0, 2.0),
    ("joint5", (-90.0, 90.0), 18.0, 2.0),
    ("joint6", (-90.0, 90.0), 18.0, 2.0),
];

fn master_id(axis: &BoundAxis) -> u16 {
    axis.binding.bus_id + MASTER_ID_OFFSET
}

fn joint(axis: BoundAxis, &(_, (soft_lower, soft_upper), kp, kd): &Joint) -> DamiaoAxis {
    // The soft box is the vendor's, in joint coordinates. It becomes the motor's range through
    // this unit's drive map, so a calibrated zero or direction carries it along with the joint;
    // uncalibrated, the arm's map is identity and the motor's range is the joint's.
    let ends = [
        axis.actuator_from_joint(axis.lower.max(soft_lower.to_radians())),
        axis.actuator_from_joint(axis.upper.min(soft_upper.to_radians())),
    ];
    let (motor_lower, motor_upper) = (ends[0].min(ends[1]), ends[0].max(ends[1]));
    DamiaoAxis {
        master_id: master_id(&axis),
        axis,
        kp,
        kd,
        motor_lower,
        motor_upper,
        // Seeed's controller leaves the motors' TIMEOUT register as it finds it.
        can_timeout: None,
        reports_enable_state: true,
    }
}

/// The rendered body, refused unless its joints are the B601-DM's, then calibrated by `drive`.
///
/// # Errors
///
/// Refuses other joints and a drive of another body.
pub fn body(drive: &InstanceDrive) -> Result<BodyBindings, BodyError> {
    let body = BodyBindings::parse(BINDINGS)?;
    let mut joints: Vec<&str> = ARM.iter().map(|(name, ..)| *name).collect();
    joints.push(GRIPPER_JOINT);
    body.require_joints(&joints)?;
    body.calibrated(drive)
}

/// `joint1`..`joint6`, in native state order, with the registry's bounds, as `drive`'s robot
/// drives them.
///
/// # Errors
///
/// Refuses rendered bindings whose joints are not the B601-DM's, and a drive of another body.
pub fn b601_arm(drive: &InstanceDrive) -> Result<Vec<DamiaoAxis>, BodyError> {
    let body = body(drive)?;
    Ok(body
        .axes
        .into_iter()
        .zip(&ARM)
        .map(|(axis, configuration)| joint(axis, configuration))
        .collect())
}

/// `gripper_joint1`, one finger's travel in metres, driven by its motor through the
/// registry's transmission: open angle over finger travel at that angle.
///
/// # Errors
///
/// Refuses rendered bindings whose joints are not the B601-DM's, and a drive of another body.
pub fn b601_gripper(drive: &InstanceDrive) -> Result<DamiaoAxis, BodyError> {
    let axis = body(drive)?.axis(GRIPPER_JOINT)?.clone();
    Ok(DamiaoAxis {
        master_id: master_id(&axis),
        axis,
        kp: 8.0,
        kd: 1.0,
        motor_lower: GRIPPER_MOTOR_OPEN_RAD,
        motor_upper: 0.0,
        can_timeout: None,
        reports_enable_state: true,
    })
}

/// The single follower's whole chain: the arm, then the gripper, as `b601-dm`'s state order.
///
/// # Errors
///
/// Refuses rendered bindings whose joints are not the B601-DM's, and a drive of another body.
pub fn b601(drive: &InstanceDrive) -> Result<Vec<DamiaoAxis>, BodyError> {
    let mut chain = b601_arm(drive)?;
    chain.push(b601_gripper(drive)?);
    Ok(chain)
}

/// The B601-DM follower: the arm's chain on `arm_port`, then the gripper's on `gripper_port`,
/// each brought up as the vendor does and left disabled.
pub type B601<P, Q> = Composite<DamiaoChain<P>, DamiaoChain<Q>>;

/// Open the B601-DM follower as one chain.
///
/// # Errors
///
/// Returns the chain that did not open and why.
pub fn open_b601<P: CanPort, Q: CanPort>(
    arm_port: P,
    gripper_port: Q,
    drive: &InstanceDrive,
) -> Result<B601<P, Q>, DamiaoChainError> {
    let arm = DamiaoChain::open(arm_port, b601_arm(drive)?)?;
    let gripper = DamiaoChain::open(gripper_port, vec![b601_gripper(drive)?])?;
    Ok(Composite::new(arm, gripper))
}

#[cfg(test)]
mod tests {
    use super::*;

    const DRIVES: &str = include_str!("../tests/fixtures/drives.json");

    fn drive(which: &str) -> InstanceDrive {
        let drives: serde_json::Value = serde_json::from_str(DRIVES).expect("drives");
        InstanceDrive::parse(
            &drives["b601-dm"][which].to_string(),
            drives["b601-dm"][which]["instance"]
                .as_str()
                .expect("an instance"),
        )
        .expect("a drive")
    }

    #[test]
    fn a_calibrated_arm_joint_keeps_its_soft_box_in_joint_coordinates() {
        let body = body(&drive("nominal")).expect("the rendered body");
        for (axis, configuration) in body.axes.into_iter().zip(&ARM) {
            let nominal = joint(axis.clone(), configuration).commandable();
            for direction in [1, -1] {
                let unit = axis.clone().calibrated(0.1, direction).expect("a unit");
                let calibrated = joint(unit, configuration).commandable();
                assert!(
                    (calibrated.lower - nominal.lower).abs() < 1e-12,
                    "{}",
                    configuration.0
                );
                assert!(
                    (calibrated.upper - nominal.upper).abs() < 1e-12,
                    "{}",
                    configuration.0
                );
            }
        }
    }

    #[test]
    fn an_instances_measured_joint_reaches_its_motor() {
        let chain = b601(&drive("measured")).expect("composes");
        assert!((chain[0].axis.zero - 0.1).abs() < f64::EPSILON);
        assert_eq!(chain[0].axis.direction, -1);
        assert!(chain[1..].iter().all(|motor| motor.axis.direction == 1));
    }
}
