//! The one surface every embodiment actuator driver implements.
//!
//! A driver drives a chain of qualified actuators on one bus. Each actuator is bound to one
//! joint axis by the registry's `ActuatorBinding` (`sx_embodiments/layout.py`): a model on a
//! bus at a chain address, and the drive map
//! `actuator = sign * direction * (joint - zero) * reduction`, the zero and direction being
//! the unit's joint calibration. The chain's axes are listed in the
//! embodiment's native state order, so a command vector and an observed vector are indexed
//! exactly as `robot.state.coordinates`. A driver reads those facts from the registry's
//! rendered [`BodyBindings`], never spells them, and calibrates them from one built robot's
//! [`InstanceDrive`], which the registry resolves from its robot instance.
//!
//! [`ActuatorChain`] is that surface: command the joint vector, stop with a report of what
//! each actuator's answer proves, each failure classified as a [`ChainFailure`]. A
//! [`ProvenStop`] exists only when every actuator proved it stopped. Port traits are per bus
//! kind ([`can::CanPort`], [`serial::SerialPort`]); one crate per bus implements the chain over
//! its port, translating its vendor's own driver. A body spanning two chains (two devices, or
//! two buses) is one [`Composite`].
//!
//! [`MediaPlan`] is the same rule for what a robot streams: its declared cameras and
//! microphones as the registry resolves them for one instance, captured through
//! [`FrameSource`] and [`AudioSource`]. A camera riding moving joints is placed at capture by
//! its view's [`kinematics::Chain`], the description's own joints from root to mount.
//!
//! Synchronous and allocation-light on purpose: the station wraps a chain in its own async
//! actuator, and nothing here owns a thread or a runtime.

pub mod binding;
pub mod body;
pub mod can;
pub mod chain;
pub mod composite;
pub mod drive;
pub mod kinematics;
pub mod layout;
pub mod media;
pub mod serial;

pub use binding::{
    ActuatorBinding, ActuatorBus, ActuatorModel, BindingError, BoundAxis, UNCALIBRATED_DIRECTION,
    UNCALIBRATED_ZERO, validate_chain,
};
pub use body::{BodyBindings, BodyError};
pub use chain::{
    ActuatorChain, ChainError, ChainFailure, CommandRange, Evidence, ProvenStop, Receipt,
    StopReport, TargetError, check_targets,
};
pub use composite::{Composite, CompositeError};
pub use drive::{DRIVE_SCHEMA, InstanceDrive, JointDrive};
pub use kinematics::{Chain, ChainStep, PlacementError};
pub use layout::{LAYOUT_SCHEMA, LayoutControl, LayoutError, RobotLayout};
pub use media::{
    AudioChunk, AudioSource, CaptureError, CapturedAt, Exposure, FrameSource, I420Image,
    MEDIA_SCHEMA, MediaPlan, MediaPlanError, Microphone, Pose, Track, View, is_track_name,
};
