//! The registry's per-axis drive facts, as a driver receives them.
//!
//! Field for field this is `sx_embodiments.layout.ActuatorBinding` and the axis it binds; the
//! validation is that class's `__post_init__` and `JointLayout`'s unique-address law, so a
//! binding the registry would refuse is refused here too. A driver never spells these facts:
//! it reads them from the document `tools/render_driver_bindings.py` renders from the registry
//! ([`crate::body::BodyBindings`]).
//!
//! The drive map is `actuator = sign * direction * (joint - zero) * reduction`. Sign and
//! reduction are the design's; the zero and the direction are measured per motor, so they are
//! the unit's joint calibration (`sx_embodiments.calibration.JointUnitCalibration`) and sit on
//! [`BoundAxis`], never on the binding. [`BoundAxis::calibrated`] is the one place a
//! calibration is applied; until a unit's calibration is supplied, every axis carries
//! [`UNCALIBRATED_ZERO`] and [`UNCALIBRATED_DIRECTION`].

use std::collections::HashMap;
use std::sync::OnceLock;

use serde::Deserialize;
use thiserror::Error;

/// One member of the registry's closed `ActuatorModel` vocabulary, by its wire value.
///
/// [`ActuatorModel::ALL`] is the vocabulary, held equal to the Python enum by the rendered
/// `vocabulary.json`; a driver crate names the members it is qualified for as constants.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct ActuatorModel(&'static str);

impl ActuatorModel {
    /// Every member of the registry's `ActuatorModel`, by wire value.
    pub const ALL: [Self; 7] = [
        Self("feetech_sts3215"),
        Self("damiao_dm4310"),
        Self("damiao_dm4340"),
        Self("damiao_dm8009"),
        Self("damiao_dm8009p"),
        Self("dynamixel_xl330_m288"),
        Self("dynamixel_xm430_w350"),
    ];

    #[must_use]
    pub const fn new(wire: &'static str) -> Self {
        Self(wire)
    }

    /// The member spelled `wire`, or `None` when the registry has none.
    #[must_use]
    pub fn parse(wire: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|model| model.0 == wire)
    }

    #[must_use]
    pub const fn wire(self) -> &'static str {
        self.0
    }

    /// The registry's `ACTUATOR_RATINGS` entry for this model, as `vocabulary.json` carries it.
    ///
    /// # Panics
    ///
    /// Never for a member of [`Self::ALL`]: the rendered vocabulary rates every one, which
    /// `tests/vocabulary.rs` holds.
    #[must_use]
    pub fn rating(self) -> ActuatorRating {
        *ratings()
            .get(self.0)
            .unwrap_or_else(|| panic!("the registry rates no actuator model {:?}", self.0))
    }
}

/// A tested product's ratings: `sx_embodiments.layout.ActuatorRating`.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq)]
pub struct ActuatorRating {
    /// What torque guards are a share of, N·m.
    pub peak_torque_nm: f64,
    /// The supply the vendor specifies, `[min, max]` V inclusive, or `None` where none is
    /// recorded; the bring-up compares the measured rail against it.
    pub supply_volts: Option<[f64; 2]>,
    /// The vendor's rated output speed at its rated supply, rad/s, or `None` where none is
    /// recorded: what a description declares as the joint's `<limit velocity>`.
    pub rated_speed_rad_s: Option<f64>,
}

impl ActuatorRating {
    /// Whether `volts` lies in the specified supply range; `None` when none is recorded.
    #[must_use]
    pub fn admits_supply(&self, volts: f64) -> Option<bool> {
        self.supply_volts
            .map(|[low, high]| (low..=high).contains(&volts))
    }
}

fn ratings() -> &'static HashMap<String, ActuatorRating> {
    #[derive(Deserialize)]
    struct Vocabulary {
        ratings: HashMap<String, ActuatorRating>,
    }
    static RATINGS: OnceLock<HashMap<String, ActuatorRating>> = OnceLock::new();
    RATINGS.get_or_init(|| {
        serde_json::from_str::<Vocabulary>(include_str!("../bindings/vocabulary.json"))
            .expect("the rendered vocabulary parses")
            .ratings
    })
}

/// One member of the registry's closed `ActuatorBus` vocabulary, by its wire value.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct ActuatorBus(&'static str);

impl ActuatorBus {
    /// Every member of the registry's `ActuatorBus`, by wire value.
    pub const ALL: [Self; 3] = [
        Self("feetech_serial"),
        Self("damiao_can"),
        Self("dynamixel_serial"),
    ];

    #[must_use]
    pub const fn new(wire: &'static str) -> Self {
        Self(wire)
    }

    /// The member spelled `wire`, or `None` when the registry has none.
    #[must_use]
    pub fn parse(wire: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|bus| bus.0 == wire)
    }

    #[must_use]
    pub const fn wire(self) -> &'static str {
        self.0
    }
}

/// How one joint axis is physically driven: a qualified product at an address on a bus.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorBinding {
    pub model: ActuatorModel,
    pub bus: ActuatorBus,
    /// The actuator's address on its chain.
    pub bus_id: u16,
    pub sign: i8,
    pub reduction: f64,
}

/// The zero of a joint whose unit has no joint calibration yet: the design's own zero.
pub const UNCALIBRATED_ZERO: f64 = 0.0;

/// The direction of a unit with no joint calibration yet: it turns as the design says.
pub const UNCALIBRATED_DIRECTION: i8 = 1;

impl ActuatorBinding {
    /// Refuse what `ActuatorBinding.__post_init__` refuses.
    ///
    /// # Errors
    ///
    /// Returns the first invalid field.
    pub fn validate(&self) -> Result<(), BindingError> {
        if self.bus_id < 1 {
            return Err(BindingError::BusId(self.bus_id));
        }
        if !matches!(self.sign, -1 | 1) {
            return Err(BindingError::Sign(self.bus_id));
        }
        if !self.reduction.is_finite() || self.reduction <= 0.0 {
            return Err(BindingError::Reduction(self.bus_id));
        }
        Ok(())
    }
}

/// The fastest one joint moves, in its own unit per second and per second squared: what its
/// actuator is rated for and the arm it carries allows (`sx_embodiments.calibrate.motion.JointMotion`,
/// rendered into the bindings). Every motion path's governor limits come from it
/// ([`crate::AxisLimits::rated`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisMotion {
    pub speed: f64,
    pub acceleration: f64,
    /// The most inertia the joint moves, kg m² at the joint (the registry's estimate, at its
    /// body's mass margin); 0 where none is known.
    pub inertia: f64,
}

/// One coordinate of the native state order with the actuator that drives it.
#[derive(Clone, Debug, PartialEq)]
pub struct BoundAxis {
    /// The registry's joint name (`JointAxis.name`).
    pub joint: String,
    /// The physical bus the actuator is wired to, by the name commissioning records its
    /// bridge under (`ActuatorBinding.bus_port`): `can` for a body's one Damiao bus, `neck`
    /// for the static upper body's Feetech neck.
    pub port: String,
    /// The joint's declared bounds, in its own unit.
    pub lower: f64,
    pub upper: f64,
    pub binding: ActuatorBinding,
    /// The joint coordinate at the actuator's zero: this unit's joint calibration, or
    /// [`UNCALIBRATED_ZERO`].
    pub zero: f64,
    /// `-1` when this unit turns against its design sign: its joint calibration, or
    /// [`UNCALIBRATED_DIRECTION`].
    pub direction: i8,
    /// Where the joint goes when an operation ends ([`crate::ending`]), in its own unit: its
    /// chain's declared `home` (`ChainSpec.home`); `None` where its part declares none (a
    /// gripper's finger), and such a joint stays where it is.
    pub home: Option<f64>,
    /// How fast the hardware moves this joint.
    pub motion: AxisMotion,
}

impl BoundAxis {
    /// This axis with a unit's measured zero and direction applied: the one seam a joint
    /// calibration (`JointUnitCalibration.zero_offset` and `.direction`) enters.
    ///
    /// # Errors
    ///
    /// Refuses a non-finite zero and a direction other than 1 or -1.
    pub fn calibrated(self, zero: f64, direction: i8) -> Result<Self, BindingError> {
        if !zero.is_finite() {
            return Err(BindingError::Zero(self.joint));
        }
        if !matches!(direction, -1 | 1) {
            return Err(BindingError::Direction(self.joint));
        }
        Ok(Self {
            zero,
            direction,
            ..self
        })
    }

    fn signed(&self) -> f64 {
        f64::from(self.binding.sign) * f64::from(self.direction)
    }

    /// `actuator = sign * direction * (joint - zero) * reduction`.
    #[must_use]
    pub fn actuator_from_joint(&self, joint: f64) -> f64 {
        self.signed() * (joint - self.zero) * self.binding.reduction
    }

    /// The inverse map, from the actuator's own coordinate to the joint's.
    #[must_use]
    pub fn joint_from_actuator(&self, actuator: f64) -> f64 {
        actuator / (self.signed() * self.binding.reduction) + self.zero
    }

    /// Refuse empty bounds and an invalid binding.
    ///
    /// # Errors
    ///
    /// Returns the first invalid fact.
    pub fn validate(&self) -> Result<(), BindingError> {
        if !(self.lower.is_finite() && self.upper.is_finite() && self.lower < self.upper) {
            return Err(BindingError::Bounds(self.joint.clone()));
        }
        if !is_port_name(&self.port) {
            return Err(BindingError::Port(self.joint.clone()));
        }
        if !self.zero.is_finite() {
            return Err(BindingError::Zero(self.joint.clone()));
        }
        if !matches!(self.direction, -1 | 1) {
            return Err(BindingError::Direction(self.joint.clone()));
        }
        if self.home.is_some_and(|home| !self.admits(home)) {
            return Err(BindingError::Home(self.joint.clone()));
        }
        self.binding.validate()
    }

    /// Whether `joint` is a finite value inside the declared bounds.
    #[must_use]
    pub fn admits(&self, joint: f64) -> bool {
        joint.is_finite() && (self.lower..=self.upper).contains(&joint)
    }
}

/// Validate every axis and `JointLayout`'s law: one address per bus within a chain, each
/// port being its own bus.
///
/// # Errors
///
/// Returns the first invalid axis, or the first repeated address.
pub fn validate_chain(axes: &[BoundAxis]) -> Result<(), BindingError> {
    if axes.is_empty() {
        return Err(BindingError::EmptyChain);
    }
    if axes.len() > crate::axes::MAX_AXES {
        return Err(BindingError::TooWide(axes.len()));
    }
    let address = |axis: &BoundAxis| (axis.binding.bus, axis.port.clone(), axis.binding.bus_id);
    for (index, axis) in axes.iter().enumerate() {
        axis.validate()?;
        if axes[..index]
            .iter()
            .any(|earlier| address(earlier) == address(axis))
        {
            return Err(BindingError::DuplicateAddress(axis.binding.bus_id));
        }
    }
    Ok(())
}

/// Whether `port` is spelled as the registry's `BusPort` is: lowercase letters, digits, `_`
/// and `-`, starting with a letter.
#[must_use]
pub fn is_port_name(port: &str) -> bool {
    port.starts_with(|first: char| first.is_ascii_lowercase())
        && port
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || matches!(c, '_' | '-'))
}

#[derive(Debug, Error, PartialEq)]
pub enum BindingError {
    #[error("the chain binds no axis")]
    EmptyChain,
    #[error("bus_id {0} must be a positive integer")]
    BusId(u16),
    #[error("the actuator at bus_id {0} has a sign other than +1 or -1")]
    Sign(u16),
    #[error("joint {0} has a non-finite zero")]
    Zero(String),
    #[error("joint {0} has a direction other than 1 or -1")]
    Direction(String),
    #[error("the actuator at bus_id {0} has a reduction that is not a positive finite ratio")]
    Reduction(u16),
    #[error("joint {0} has bounds that are not finite lower < upper")]
    Bounds(String),
    #[error("joint {0} has a home that is not a finite value inside its bounds")]
    Home(String),
    #[error("joint {0} names a port that is not a lowercase identifier")]
    Port(String),
    #[error("the chain binds {0} axes, more than a control tick carries")]
    TooWide(usize),
    #[error("bus_id {0} appears twice on one bus within the chain")]
    DuplicateAddress(u16),
}

#[cfg(test)]
mod tests {
    use super::*;

    const MODEL: ActuatorModel = ActuatorModel::new("model");
    const BUS: ActuatorBus = ActuatorBus::new("bus");

    fn axis(bus_id: u16) -> BoundAxis {
        BoundAxis {
            joint: format!("joint{bus_id}"),
            port: "can".to_owned(),
            lower: -1.0,
            upper: 1.0,
            binding: ActuatorBinding {
                model: MODEL,
                bus: BUS,
                bus_id,
                sign: -1,
                reduction: 4.0,
            },
            zero: 0.25,
            direction: UNCALIBRATED_DIRECTION,
            home: Some(0.0),
            motion: AxisMotion {
                speed: 1.0,
                acceleration: 10.0,
                inertia: 0.0,
            },
        }
    }

    #[test]
    fn the_drive_map_round_trips() {
        let bound = axis(1);
        let actuator = bound.actuator_from_joint(0.75);
        assert!((actuator - -2.0).abs() < 1e-12);
        assert!((bound.joint_from_actuator(actuator) - 0.75).abs() < 1e-12);
        let recalibrated = bound.clone().calibrated(0.5, 1).expect("a finite zero");
        assert!((recalibrated.actuator_from_joint(0.75) - -1.0).abs() < 1e-12);
        let reversed = bound
            .calibrated(0.5, -1)
            .expect("a unit turning against its design");
        assert!((reversed.actuator_from_joint(0.75) - 1.0).abs() < 1e-12);
        assert!((reversed.joint_from_actuator(1.0) - 0.75).abs() < 1e-12);
    }

    #[test]
    fn what_the_registry_refuses_is_refused() {
        let mut bad = axis(1);
        bad.binding.bus_id = 0;
        assert_eq!(bad.validate(), Err(BindingError::BusId(0)));
        let mut bad = axis(1);
        bad.binding.sign = 0;
        assert_eq!(bad.validate(), Err(BindingError::Sign(1)));
        let mut bad = axis(1);
        bad.binding.reduction = 0.0;
        assert_eq!(bad.validate(), Err(BindingError::Reduction(1)));
        let mut bad = axis(1);
        bad.zero = f64::NAN;
        assert!(matches!(bad.validate(), Err(BindingError::Zero(_))));
        assert!(matches!(
            axis(1).calibrated(f64::INFINITY, 1),
            Err(BindingError::Zero(_))
        ));
        assert!(matches!(
            axis(1).calibrated(0.0, 0),
            Err(BindingError::Direction(_))
        ));
        let mut bad = axis(1);
        bad.home = Some(1.5);
        assert!(matches!(bad.validate(), Err(BindingError::Home(_))));
        bad.home = None;
        assert_eq!(bad.validate(), Ok(()));
        let mut bad = axis(1);
        bad.upper = bad.lower;
        assert!(matches!(bad.validate(), Err(BindingError::Bounds(_))));
        assert_eq!(validate_chain(&[]), Err(BindingError::EmptyChain));
        assert_eq!(
            validate_chain(&[axis(1), axis(2), axis(1)]),
            Err(BindingError::DuplicateAddress(1))
        );
        assert_eq!(validate_chain(&[axis(1), axis(2)]), Ok(()));
        let mut elsewhere = axis(1);
        elsewhere.port = "neck".to_owned();
        assert_eq!(validate_chain(&[axis(1), elsewhere]), Ok(()));
        for spelled in ["", "Neck", "1can", "neck bus"] {
            let mut bad = axis(1);
            bad.port = spelled.to_owned();
            assert!(matches!(bad.validate(), Err(BindingError::Port(_))));
        }
    }

    #[test]
    fn the_vocabulary_parses_only_its_own_members() {
        assert_eq!(
            ActuatorModel::parse("damiao_dm4310"),
            Some(ActuatorModel::ALL[1])
        );
        assert_eq!(ActuatorModel::parse("dynamixel_xl330"), None);
        assert!(ActuatorModel::parse("dynamixel_xl330_m288").is_some());
        assert!(ActuatorBus::parse("dynamixel_serial").is_some());
        assert_eq!(ActuatorBus::parse("damiao_can"), Some(ActuatorBus::ALL[1]));
        assert_eq!(ActuatorBus::parse("can"), None);
    }
}
