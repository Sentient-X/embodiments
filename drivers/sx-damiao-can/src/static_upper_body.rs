//! The Sentient static upper body's fifteen directly driven Damiao motors as one chain.
//!
//! Reference: `assets/sentient_static_upper_body/source/humanoid_allisa_bot_hardware.json`, the
//! hardware profile `humanoid_pkg` ships with the selected `allisa_bot` description: each arm
//! joint's send id, answer id (send id + 0x20), model, sign and MIT `kp` and `kd`, on `can0`.
//! Those gains are that driver's, not a qualification: whole-body qualification is the
//! station's bench work. The torso has no entry there; its id and gains, and the timeouts below,
//! are the superseded `allisa_feedback` description's motor table (`source/received.urdf`
//! before 2026-10-26, in git history), which agrees with the profile on every arm motor. That
//! table's driver also capped speed at 0.25 rad/s and tracking error at 0.2 rad, with a
//! 0.5 s feedback and a 0.75 s heartbeat timeout. The heartbeat is enforced here: every motor's
//! `TIMEOUT` register is written to 0.75 s at open ([`crate::chain`]), so an enabled motor that
//! hears no command for that long disables itself. The 0.25 rad/s was that driver's bring-up
//! choice, not the hardware's, and is not kept: speed, acceleration and tracking error are
//! `sx_embodiment_drivers::Governor`'s, from each joint's rated motion in the bindings
//! (`AxisLimits::rated`: the motor's rated speed, the acceleration its torque leaves over the
//! arm). Nothing applies the 0.5 s feedback timeout.
//! Both forearm yaws span more than one turn, so each is commanded only within one turn of its
//! motor's zero (`DamiaoAxis::motor_window`).
//!
//! The axes and their bindings are the registry's (`sx_embodiments/known/static_upper_body.py`,
//! a development body), read from `bindings/sentient-static-upper-body.json`, which
//! `tools/render_driver_bindings.py` renders: the torso yaw and both seven-joint arms, identity
//! sign except both wrists (sign -1: `allisa_bot` turns those axes against their motors), every
//! commandable range the confirmed limits. The grippers and the SO-101 neck are
//! Feetech servos on serial buses of their own, the same document's other ports, composed by
//! `sx-feetech-serial`, not here. No bound, address, model, sign or reduction is spelled
//! here; what is added is controller configuration, keyed by joint name.

use std::time::Duration;

use crate::body::{DamiaoBody, JointGains};

/// The registry's static upper-body axes, rendered.
pub const BINDINGS: &str = include_str!("../bindings/sentient-static-upper-body.json");

/// The motor table's answer ids: `recv_id = send_id + 0x20`.
pub const MASTER_ID_OFFSET: u16 = 0x20;

/// The torso yaw and both arms on one bus.
pub const STATIC_UPPER_BODY: DamiaoBody = DamiaoBody {
    embodiment: "sentient-static-upper-body",
    bindings: BINDINGS,
    master_id_offset: MASTER_ID_OFFSET,
    // The received driver's heartbeat_timeout, written to every motor's TIMEOUT register.
    // Measured 2026-10-26 with sx-timeout-probe: both arms count 50 us, so 15000 trips between
    // 0.7 and 0.8 s of silence (left elbow between 0.7 and 1.0 s), only while enabled. The left
    // arm's 0xD at enable came from enabling one motor per 0.1 s; the chain enables them together.
    can_timeout: Some(Duration::from_millis(750)),
    reports_enable_state: false,
    joints: &[
        JointGains::new("torso_joint", 40.0, 1.0),
        JointGains::new("left_shoulder_pitch_joint", 60.984, 1.091),
        JointGains::new("left_shoulder_roll_joint", 60.984, 1.091),
        JointGains::new("left_upper_arm_yaw_joint", 24.0, 5.0),
        JointGains::new("left_elbow_pitch_joint", 30.492, 4.655),
        JointGains::new("left_forearm_yaw_joint", 29.04, 4.75),
        JointGains::new("left_wrist_pitch_joint", 20.0, 1.0),
        JointGains::new("left_wrist_roll_joint", 20.0, 1.0),
        JointGains::new("right_shoulder_pitch_joint", 60.984, 1.091),
        JointGains::new("right_shoulder_roll_joint", 42.0, 0.98),
        JointGains::new("right_upper_arm_yaw_joint", 30.492, 4.655),
        JointGains::new("right_elbow_pitch_joint", 30.492, 4.655),
        JointGains::new("right_forearm_yaw_joint", 30.492, 4.655),
        JointGains::new("right_wrist_pitch_joint", 20.0, 1.0),
        JointGains::new("right_wrist_roll_joint", 20.0, 1.0),
    ],
};

/// PROPOSED, not commissioned: [`STATIC_UPPER_BODY`] with its torso and shoulders damped toward
/// a damping ratio of 0.5. No station opens a robot with it; only `sx-identify --gains damped
/// --part step:<joint> --move`, a person's run at the robot, drives it, and fits each step's
/// natural frequency and damping ratio, so the arm shows them before they replace
/// [`STATIC_UPPER_BODY`]'s.
///
/// Why. The commissioned shoulders follow at zeta 0.10-0.12 and the torso at 0.063 (their gains
/// over the registry inertia; a step on act-sim's motor model reads them back within 1 % in
/// omega and 7 % in zeta, `sx-embodiment-commission` `tests/damped_gains.rs`). They ring. With
/// the Spark's 26-63 ms transport delay and the caps the station applies, main's solve brought
/// the left elbow 4.5 mm inside its 10 mm margin at 90 Hz, and a part 0.49 mm into a keep-out
/// box at 50 Hz (embodiments `tests/spark_followers.rs` `table_gains`, 2026-11-03). With these
/// gains: 7.98 / 7.94 mm at 90 / 50 Hz, no part inside a box over the 0.8-1.25x inertia band.
/// The desk path tracks at 17.6 mm rms (commissioned 13.8), and act-sim passes as it does today.
/// They do not bring a tool speed within its cap (the command 19-24x, the real tool 15-18x at
/// 0.05 m/s): that overspeed is the rows'.
///
/// How. The MIT frame carries kd only up to 5 ([`crate::dm_can`]). So each joint's kd is
/// `2 zeta sqrt(kp J)` (J the joint's inertia from the bindings, kp unchanged) where that fits.
/// Where it does not, kp is lowered to `(5 / (2 zeta))^2 / J`: the left shoulder pitch to 60.0
/// and roll to 55.0, the torso to 15.8. The torso's acceleration ceiling `0.1 kp / J` drops from
/// 2.5 to 1.0 rad/s^2 and the left roll's by 10 %; the shoulders' speed ceilings stay within 5 %.
/// Peak torque is unchanged: kd 5 at the shoulders' 1.2 rad/s ceiling is 6 N m of the DM8009P's
/// 40. The planned-velocity cap `MAX_COAST kp / kd` now equals the shoulders' top speed, so a
/// stalled host's coast doubles, 0.05 to 0.10 rad (within the chain's `2 MAX_COAST` bound). At
/// kd 5 the MIT frame's truncated velocity (a planned zero arrives as -0.011 rad/s on a DM8009)
/// holds the torso 3.5 mrad off. Zeta 0.7 would need kp about halved (shoulders 28-32, torso
/// 8.1): the same clearance, desk tracking at 25 mm rms, the ceilings 29 % lower, and act-sim's
/// station-clock chunks test fails.
pub const STATIC_UPPER_BODY_DAMPED: DamiaoBody = DamiaoBody {
    joints: &[
        JointGains::new("torso_joint", 15.8, 5.0),
        JointGains::new("left_shoulder_pitch_joint", 60.0, 5.0),
        JointGains::new("left_shoulder_roll_joint", 55.0, 5.0),
        JointGains::new("left_upper_arm_yaw_joint", 24.0, 5.0),
        JointGains::new("left_elbow_pitch_joint", 30.492, 4.655),
        JointGains::new("left_forearm_yaw_joint", 29.04, 4.75),
        JointGains::new("left_wrist_pitch_joint", 20.0, 1.0),
        JointGains::new("left_wrist_roll_joint", 20.0, 1.0),
        JointGains::new("right_shoulder_pitch_joint", 60.984, 4.95),
        JointGains::new("right_shoulder_roll_joint", 42.0, 4.23),
        JointGains::new("right_upper_arm_yaw_joint", 30.492, 4.655),
        JointGains::new("right_elbow_pitch_joint", 30.492, 4.655),
        JointGains::new("right_forearm_yaw_joint", 30.492, 4.655),
        JointGains::new("right_wrist_pitch_joint", 20.0, 1.0),
        JointGains::new("right_wrist_roll_joint", 20.0, 1.0),
    ],
    ..STATIC_UPPER_BODY
};
