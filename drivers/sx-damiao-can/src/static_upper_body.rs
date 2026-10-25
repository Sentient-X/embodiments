//! The Sentient static upper body's fifteen directly driven Damiao motors as one chain.
//!
//! Reference: `assets/sentient_static_upper_body/source/received.urdf` in this repository, the
//! description as received on 2026-10-25 (maintainer `suprajs`). Its `<motor_configuration>`
//! puts every motor on `can1` (Linux `SocketCAN`) at 50 Hz, and its `<motor_parameters>` give
//! each joint's send id, answer id (send id + 0x20) and MIT `kp` and `kd`. Those gains are that
//! file's driver's, not a qualification: whole-body qualification is the station's bench work.
//! The same file's driver also caps speed at 0.25 rad/s and tracking error at 0.2 rad, with a
//! 0.5 s feedback and a 0.75 s heartbeat timeout. The heartbeat is enforced here: every motor's
//! `TIMEOUT` register is written to 0.75 s at open ([`crate::chain`]), so an enabled motor that
//! hears no command for that long disables itself. Speed and tracking error are not enforced by
//! this chain: they are `sx_embodiment_drivers::Governor`'s, which bounds them only when the
//! caller wraps the chain in it with those limits. Nothing applies the 0.5 s feedback timeout.
//! Both forearm yaws span more than one turn, so each is commanded only within one turn of its
//! motor's zero (`DamiaoAxis::motor_window`).
//!
//! The axes and their bindings are the registry's (`sx_embodiments/known/static_upper_body.py`,
//! a development body), read from `bindings/sentient-static-upper-body.json`, which
//! `tools/render_driver_bindings.py` renders: the torso yaw and both seven-joint arms, identity
//! sign, every commandable range the confirmed limits. The grippers and the SO-101 neck are
//! Feetech servos on serial buses of their own, the same document's other ports, composed by
//! `sx-feetech-serial`, not here. No bound, address, model, sign or reduction is spelled
//! here; what is added is controller configuration, keyed by joint name.

use std::time::Duration;

use crate::body::{DamiaoBody, JointGains};

/// The registry's static upper-body axes, rendered.
pub const BINDINGS: &str = include_str!("../bindings/sentient-static-upper-body.json");

/// The received motor table's answer ids: `recv_id = send_id + 0x20`.
pub const MASTER_ID_OFFSET: u16 = 0x20;

/// The torso yaw and both arms on one bus.
pub const STATIC_UPPER_BODY: DamiaoBody = DamiaoBody {
    bindings: BINDINGS,
    master_id_offset: MASTER_ID_OFFSET,
    // The received driver's heartbeat_timeout, written to every motor's TIMEOUT register.
    can_timeout: Some(Duration::from_millis(750)),
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
