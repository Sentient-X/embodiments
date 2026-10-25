//! The Sentient humanoid upper body's eleven directly driven Damiao motors as one chain.
//!
//! References:
//!
//! - `Sentient-X/sentient_can` at `3837d224327d89ed30b6fdee3577769d8dae5f94`,
//!   `examples/capture_readings_27motors_mt_direct.cpp`: every motor on `can0`, each answering
//!   on its send id + 0x20 (`recv_can_ids`).
//! - `Sentient-X/humanoid-simrealgap` at `5c11827f9ffeede4529cc1cdd114e3b57e75f4b3`,
//!   `configs/training_actuator_params_20260606.csv`: each motor's MIT `kp` and `kd`, its
//!   `protocol_capped_kd` already at the protocol's maximum of 5. That export calls them
//!   "single-motor validated gains, not whole-body deployment approval"; they are the one
//!   source of gains here, and whole-body qualification is the station's bench work.
//!
//! The axes and their bindings are the registry's (`sx_embodiments/known/humanoid.py`), read
//! from `bindings/sentient-humanoid-upper-body.json`, which `tools/render_driver_bindings.py`
//! renders: the torso yaw and both five-joint arms. The waist's roll and pitch are driven
//! together by two motors through a linkage the registry leaves undocumented, so they are in no
//! chain and their motors are never enabled. No bound, address, model, sign or reduction is
//! spelled here; what is added is controller configuration, keyed by joint name. Every
//! commandable range is the URDF's: the vendor states no soft box.
//!
//! The body is one [`DamiaoBody`], [`HUMANOID_UPPER_BODY`]; [`open_humanoid_upper_body`] opens
//! the whole chain on one `CanPort`, calibrated from one built robot's [`InstanceDrive`].

use sx_embodiment_drivers::can::CanPort;
use sx_embodiment_drivers::{BodyBindings, BodyError, InstanceDrive};

use crate::body::{DamiaoBody, JointGains};
use crate::chain::{DamiaoAxis, DamiaoChain, DamiaoChainError};

/// The registry's humanoid upper-body axes, rendered.
pub const BINDINGS: &str = include_str!("../bindings/sentient-humanoid-upper-body.json");

/// `sentient_can`'s answer-id convention for the humanoid: `recv_id = send_id + 0x20`.
pub const MASTER_ID_OFFSET: u16 = 0x20;

/// The torso yaw and both arms on `can0`.
pub const HUMANOID_UPPER_BODY: DamiaoBody = DamiaoBody {
    bindings: BINDINGS,
    master_id_offset: MASTER_ID_OFFSET,
    // `sentient_can` leaves the motors' TIMEOUT register as it finds it.
    can_timeout: None,
    joints: &[
        JointGains::new("torso_joint", 350.0, 5.0),
        JointGains::new("right_shoulder_pitch_joint", 100.0, 5.0),
        JointGains::new("right_shoulder_roll_joint", 40.0, 2.8),
        JointGains::new("right_elbow_yaw_joint", 54.772, 5.0),
        JointGains::new("right_elbow_forearm_yaw_joint", 54.772, 5.0),
        JointGains::new("right_wrist_yaw_joint", 54.772, 5.0),
        JointGains::new("left_shoulder_pitch_joint", 40.0, 2.6595),
        JointGains::new("left_shoulder_roll_joint", 100.0, 5.0),
        JointGains::new("left_elbow_yaw_joint", 54.772, 4.861),
        JointGains::new("left_elbow_forearm_yaw_joint", 110.0, 5.0),
        JointGains::new("left_wrist_yaw_joint", 54.772, 4.075_301_53),
    ],
};

/// The rendered body, refused unless its driven joints are the ones configured here, then
/// calibrated by `drive`.
///
/// # Errors
///
/// Refuses other joints and a drive of another body.
pub fn body(drive: &InstanceDrive) -> Result<BodyBindings, BodyError> {
    HUMANOID_UPPER_BODY.body(drive)
}

/// The torso yaw and both arms, in native state order, as `drive`'s robot drives them.
///
/// # Errors
///
/// Refuses rendered bindings whose joints are not the upper body's, and a drive of another body.
pub fn humanoid_upper_body(drive: &InstanceDrive) -> Result<Vec<DamiaoAxis>, BodyError> {
    HUMANOID_UPPER_BODY.axes(drive)
}

/// Open the humanoid upper body's motors on `port`, each brought up as the vendor does and
/// left disabled.
///
/// # Errors
///
/// Returns why the chain did not open.
pub fn open_humanoid_upper_body<P: CanPort>(
    port: P,
    drive: &InstanceDrive,
) -> Result<DamiaoChain<P>, DamiaoChainError> {
    HUMANOID_UPPER_BODY.open(port, drive)
}
