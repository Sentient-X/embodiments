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
//! two buses) is one [`Composite`]; a body whose buses are read from its bindings, one chain
//! per port ([`BodyBindings::buses`]), is one [`Chains`]. [`Governor`] wraps any chain with a
//! speed and tracking-error bound per axis, stopping and latching a fault when the body does
//! not follow; [`Governor::end`] ends a body's operation as its [`AtEnd`] says, by default
//! returning it home under those same bounds before it is stopped.
//!
//! [`MediaPlan`] is the same rule for what a robot streams: its declared cameras and
//! microphones as the registry resolves them for one instance, captured through
//! [`FrameSource`] and [`AudioSource`]. A camera riding moving joints is placed at capture by
//! its view's [`kinematics::Chain`], the description's own joints from root to mount. The plan's
//! inertial sensors are captured through [`InertialSource`], each sample on the sensor's own
//! axes, which the plan places in its mount link.
//!
//! Synchronous and allocation-light on purpose: the station wraps a chain in its own async
//! actuator, and nothing here owns a thread or a runtime.

pub mod approach;
pub mod axes;
pub mod binding;
pub mod body;
pub mod bus_timing;
pub mod can;
pub mod chain;
pub mod chains;
pub mod composite;
pub mod drive;
pub mod dynamics;
pub mod ending;
pub mod governor;
pub mod holder;
pub mod host_needs;
pub mod inertial;
pub mod kinematics;
pub mod layout;
pub mod lerobot;
pub mod masked;
pub mod media;
pub mod rail;
pub mod serial;

pub use approach::{ApproachLimit, BRING_UP_APPROACH};
pub use axes::{Axes, MAX_AXES};
pub use binding::{
    ActuatorBinding, ActuatorBus, ActuatorModel, ActuatorRating, AxisMotion, BindingError,
    BoundAxis, CommandAnswer, UNCALIBRATED_DIRECTION, UNCALIBRATED_ZERO, is_port_name,
    validate_chain,
};
pub use body::{BodyBindings, BodyError, Bus, Twin};
pub use bus_timing::BusTiming;
pub use chain::{
    ActuatorChain, AxisReading, ChainError, ChainFailure, CommandRange, Evidence, Feedforward,
    FeedforwardLimit, MissingReason, MotorTemperature, Observe, ProvenStop, Receipt, Reports,
    StopReport, TargetError, TrackingCeiling, check_reach, check_targets, fresh_positions,
};
pub use chains::{Chains, ChainsError};
pub use composite::{Composite, CompositeError};
pub use drive::{DRIVE_SCHEMA, InstanceDrive, JointDrive};
pub use dynamics::{
    Dynamics, DynamicsCalibration, DynamicsError, JointFriction, LinkInertia, STANDARD_GRAVITY,
    Scratch, UPRIGHT,
};
pub use ending::{
    AtEnd, Ending, FeedforwardFn, MAX_RETURN, Pace, Progress, RETURN_MARGIN, RETURN_SETTLE,
    RETURN_SLACK, RETURN_SPEED_SHARE, RETURN_TOLERANCE, Return, ReturnFailure,
};
pub use governor::{
    AxisLimits, Clock, Governor, GovernorError, LimitsError, MAX_MISSED_IN_A_ROW,
    MAX_STEP_INTERVAL, MonotonicClock,
};
pub use holder::{Holder, Process, holder_of};
pub use host_needs::{HOST_NEEDS_SCHEMA, HostNeed, HostNeeds};
pub use inertial::{
    InertialQuantity, InertialSample, InertialSensor, InertialSource, MAX_INERTIAL_HZ, Stillness,
    Tilt,
};
pub use kinematics::{Axis, Chain, ChainStep, PlacementError, Tree, TreeError, TreeJoint};
pub use layout::{LAYOUT_SCHEMA, LayoutControl, LayoutError, RobotLayout, ServedInterface};
pub use masked::{Masked, check_available};
pub use media::{
    AudioChunk, AudioSource, CaptureError, CapturedAt, Exposure, FrameSource, I420Image,
    MEDIA_SCHEMA, MediaPlan, MediaPlanError, Microphone, Pose, Track, View, is_track_name,
};
pub use rail::{
    AxisDerate, DerateLimits, OVER_RAIL, OverRail, REFUSE_ABOVE, RailCheck, Rails, RailsError,
    derates,
};
