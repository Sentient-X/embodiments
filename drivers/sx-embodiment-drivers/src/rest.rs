//! What a stop leaves each joint in: held under torque where it would fall, released where it
//! would not.
//!
//! Until 2026-11-03 every stop disabled every actuator, and on a body without brakes (the
//! static upper body's Damiao arms) a disabled arm dropped under gravity. A stop is now a
//! [`Rest`] per axis, which [`crate::ActuatorChain::stop_holding`] establishes:
//!
//! - [`Rest::Hold`]: torque stays on, the position target is where the joint is once the
//!   stop's brake has ended, with zero planned velocity and the gravity torque the model
//!   gives there as feed-forward. No motion: the target is the measured pose, re-sent by the
//!   caller ([`crate::ActuatorChain::keep_holding`]) so a motor watchdog never drops it.
//! - [`Rest::Release`]: torque off, as every stop did before.
//!
//! Which one is derived, never listed per body: a joint **falls when released** when its
//! actuator has no holding brake ([`crate::ActuatorRating::holding_brake`]) and the gravity
//! torque the description gives at the measured pose is more than its identified static
//! sliding friction can hold with margin ([`falls`]). Without a dynamics model a brakeless
//! joint that was under torque is held ([`unmodelled`]): no gravity is known, so none is
//! assumed absent. A joint with no reading at all is held too, at its last commanded position
//! with no feed-forward (it may still answer: one missed answer is no reason to drop it); a
//! caller releases it only once it stays silent (the station's `HOLD_SILENCE`).

use crate::axes::Axes;
use crate::binding::{ActuatorRating, BoundAxis};
use crate::dynamics::{Dynamics, JointFriction, Scratch};

/// What one axis is left in at a stop.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Rest {
    /// Torque off.
    #[default]
    Release,
    /// Torque on at the pose the joint stopped at, zero velocity, `gravity` (N·m at the
    /// joint, the torque that holds it against gravity there) as feed-forward.
    Hold { gravity: f64 },
}

impl Rest {
    #[must_use]
    pub const fn holds(self) -> bool {
        matches!(self, Self::Hold { .. })
    }
}

/// How many times over a joint's static friction must outweigh gravity's torque on it before
/// it is released: identified friction is a fit, it varies with temperature and wear, and a
/// joint that creeps once released cannot be caught again. Twice over leaves a joint whose
/// friction was overestimated by half still standing.
pub const RELEASE_MARGIN: f64 = 2.0;

/// The gravity torque below which nothing falls, N·m: a joint whose axis is vertical (a wrist
/// roll, a neck yaw) carries numerically zero, and a joint needs no friction to stay put there.
pub const GRAVITY_FLOOR: f64 = 1e-3;

/// Whether a joint falls when its torque is released: its actuator has no holding brake and
/// `gravity` (the torque that holds it at its pose, N·m) is above [`GRAVITY_FLOOR`] and more
/// than `friction`'s sliding friction there divided by [`RELEASE_MARGIN`]. Released, the drive
/// delivers nothing and the load is gravity's (`external = -gravity`). Sliding, not the
/// breakaway (Stribeck) friction a joint shows at rest: a knock starts it moving, and then
/// only sliding friction holds it (the refute of 2026-11-03).
#[must_use]
pub fn falls(rating: &ActuatorRating, gravity: f64, friction: &JointFriction) -> bool {
    if rating.holding_brake {
        return false;
    }
    if !gravity.is_finite() {
        // A non-finite model value says nothing: treat it as a joint that would fall.
        return true;
    }
    let load = gravity.abs();
    load > GRAVITY_FLOOR && load * RELEASE_MARGIN > friction.sliding(0.0, -gravity)
}

/// Each axis's rest without a dynamics model: every brakeless axis is held with no
/// feed-forward (its gravity unknown), read or not; a braked one is released.
#[must_use]
pub fn unmodelled(axes: &[BoundAxis], measured: &[Option<f64>]) -> Axes<Rest> {
    let _ = measured;
    axes.iter()
        .map(|axis| {
            if axis.binding.model.rating().holding_brake {
                Rest::Release
            } else {
                Rest::Hold { gravity: 0.0 }
            }
        })
        .collect()
}

/// The fall rule over one body: its description's dynamics (with any identified calibration
/// already applied), mapped to a chain's axes by joint name, with buffers sized once.
#[derive(Clone, Debug)]
pub struct RestModel {
    dynamics: Dynamics,
    scratch: Scratch,
    /// For each description joint ([`crate::Tree::joints`]), the chain axis it is.
    axis_of: Vec<Option<usize>>,
    q: Vec<f64>,
    torque: Vec<f64>,
}

impl RestModel {
    /// The model `dynamics` over a chain whose axes are named `axes`.
    #[must_use]
    pub fn new(dynamics: Dynamics, axes: &[BoundAxis]) -> Self {
        let axis_of: Vec<Option<usize>> = dynamics
            .tree()
            .joints()
            .iter()
            .map(|joint| axes.iter().position(|axis| axis.joint == joint.name))
            .collect();
        let width = axis_of.len();
        Self {
            scratch: dynamics.scratch(),
            dynamics,
            axis_of,
            q: vec![0.0; width],
            torque: vec![0.0; width],
        }
    }

    /// Each axis's rest at `measured` (one per axis, `None` where it has no reading at all): a
    /// joint the description carries is held with its gravity torque where it [`falls`], else
    /// released; one the description does not carry (its gravity unknown), and one with no
    /// reading, is held without feed-forward unless braked. A description joint with no
    /// reading is placed at zero for the others' gravity.
    pub fn rests(&mut self, axes: &[BoundAxis], measured: &[Option<f64>]) -> Axes<Rest> {
        for (q, axis) in self.q.iter_mut().zip(&self.axis_of) {
            *q = axis
                .and_then(|index| measured.get(index).copied().flatten())
                .unwrap_or(0.0);
        }
        let modelled = self
            .dynamics
            .gravity_torque_into(&self.q, &mut self.scratch, &mut self.torque)
            .is_ok();
        let mut rests = unmodelled(axes, measured);
        if !modelled {
            return rests;
        }
        let friction = self.dynamics.friction();
        for (joint, axis) in self.axis_of.iter().enumerate() {
            let Some(index) = *axis else {
                continue;
            };
            // Unread: held as it is, no feed-forward, until its silence releases it.
            if !rests[index].holds() || measured.get(index).copied().flatten().is_none() {
                continue;
            }
            let gravity = self.torque[joint];
            let rating = axes[index].binding.model.rating();
            rests[index] = if falls(&rating, gravity, &friction[joint]) {
                Rest::Hold { gravity }
            } else {
                Rest::Release
            };
        }
        rests
    }
}

/// How hot a held motor runs against its rated limit
/// ([`crate::ActuatorRating::thermal_limit_celsius`]).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Heat {
    Within,
    /// Within [`THERMAL_WARN_MARGIN`] of the limit: said once, in one line.
    Near,
    /// At or past it: the joint counts as faulted and is released.
    AtLimit,
}

/// How close to its thermal limit a held motor runs before it is said, °C: a motor holding a
/// loaded arm warms a degree or two a minute on the static upper body's rated torque, so ten
/// degrees leaves a person minutes to Park it before it is released.
pub const THERMAL_WARN_MARGIN: f64 = 10.0;

/// How hot `hottest` (°C, the hotter of a motor's reported temperatures) is for `rating`; a
/// model with no recorded limit is always within it.
#[must_use]
pub fn heat(rating: &ActuatorRating, hottest: f64) -> Heat {
    match rating.thermal_limit_celsius {
        Some(limit) if hottest >= limit => Heat::AtLimit,
        Some(limit) if hottest >= limit - THERMAL_WARN_MARGIN => Heat::Near,
        _ => Heat::Within,
    }
}

/// How hard a held motor works against its ratings.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Effort {
    Within,
    /// Past its rated (continuous) torque: it heats while it holds. Said once.
    OverRated,
    /// At its peak torque: it is not holding, it is saturated. The joint counts as faulted
    /// (over-current) and is released.
    AtPeak,
}

/// How hard `torque` (N·m at the motor, as its answer reports it) is for `rating`.
#[must_use]
pub fn effort(rating: &ActuatorRating, torque: f64) -> Effort {
    let torque = torque.abs();
    if !torque.is_finite() {
        return Effort::Within;
    }
    if torque >= rating.peak_torque_nm {
        Effort::AtPeak
    } else if rating.rated_torque_nm.is_some_and(|rated| torque > rated) {
        Effort::OverRated
    } else {
        Effort::Within
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::binding::{ActuatorBinding, ActuatorBus, ActuatorModel, AxisMotion};
    use crate::kinematics::Tree;

    const DM4340: ActuatorModel = ActuatorModel::new("damiao_dm4340");

    fn rating(brake: bool) -> ActuatorRating {
        ActuatorRating {
            holding_brake: brake,
            ..DM4340.rating()
        }
    }

    fn friction(coulomb: f64) -> JointFriction {
        JointFriction {
            coulomb,
            ..JointFriction::default()
        }
    }

    #[test]
    fn a_brakeless_joint_falls_where_gravity_outweighs_half_its_friction() {
        let brakeless = rating(false);
        assert!(falls(&brakeless, 2.0, &JointFriction::default()));
        assert!(falls(&brakeless, -2.0, &friction(3.9)));
        // Friction twice the load or more holds it.
        assert!(!falls(&brakeless, 2.0, &friction(4.0)));
        // A vertical axis carries nothing.
        assert!(!falls(&brakeless, 1e-4, &JointFriction::default()));
        // A model that gives nothing finite says nothing: held.
        assert!(falls(&brakeless, f64::NAN, &friction(100.0)));
    }

    #[test]
    fn a_braked_joint_never_falls() {
        assert!(!falls(&rating(true), 30.0, &JointFriction::default()));
        assert!(!falls(&rating(true), f64::NAN, &JointFriction::default()));
    }

    #[test]
    fn load_on_the_gearbox_adds_to_what_holds_it() {
        let loaded = JointFriction {
            load_external: 1.0,
            ..JointFriction::default()
        };
        // Budget: |e * -g| = 2, half the margin's 4: falls; at e = 2, budget 4: held.
        assert!(falls(&rating(false), 2.0, &loaded));
        let stiff = JointFriction {
            load_external: 2.0,
            ..JointFriction::default()
        };
        assert!(!falls(&rating(false), 2.0, &stiff));
    }

    fn axis(joint: &str) -> BoundAxis {
        BoundAxis {
            joint: joint.into(),
            port: "can".into(),
            lower: -3.0,
            upper: 3.0,
            binding: ActuatorBinding {
                model: DM4340,
                bus: ActuatorBus::new("damiao_can"),
                bus_id: 1,
                sign: 1,
                reduction: 1.0,
            },
            zero: 0.0,
            direction: 1,
            home: None,
            motion: AxisMotion {
                speed: 1.0,
                acceleration: 1.0,
                inertia: 0.0,
            },
        }
    }

    /// A one-link pendulum about y: 1 kg at 0.3 m along x, so gravity's torque is
    /// `m g l cos q` (2.94 N·m level, none hanging straight down at q = -pi/2).
    const PENDULUM: &str = r#"<robot name="p">
      <link name="base"/>
      <link name="arm"><inertial><origin xyz="0.3 0 0"/><mass value="1"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>
      <joint name="shoulder" type="revolute"><parent link="base"/><child link="arm"/>
        <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
    </robot>"#;

    #[test]
    fn the_model_holds_a_level_arm_and_releases_a_hanging_one() {
        let dynamics = Dynamics::new(Tree::parse(PENDULUM).expect("tree"), crate::UPRIGHT);
        let axes = [axis("shoulder"), axis("gripper")];
        let mut model = RestModel::new(dynamics, &axes);
        let level = model.rests(&axes, &[Some(0.0), Some(0.01)]);
        let Rest::Hold { gravity } = level[0] else {
            panic!("a level arm is held: {level:?}");
        };
        assert!((gravity.abs() - 0.3 * crate::STANDARD_GRAVITY).abs() < 1e-6);
        // The gripper is not in the description: its gravity unknown, held without it.
        assert_eq!(level[1], Rest::Hold { gravity: 0.0 });
        let hanging = model.rests(&axes, &[Some(-std::f64::consts::FRAC_PI_2), None]);
        assert_eq!(hanging[0], Rest::Release, "{hanging:?}");
        // No reading: held without feed-forward, whatever the model says; its silence alone
        // releases it later.
        assert_eq!(hanging[1], Rest::Hold { gravity: 0.0 });
        let silent = model.rests(&axes, &[None, Some(0.0)]);
        assert_eq!(silent[0], Rest::Hold { gravity: 0.0 });
    }

    #[test]
    fn without_a_model_every_brakeless_axis_is_held_read_or_not() {
        let axes = [axis("a"), axis("b")];
        assert_eq!(
            unmodelled(&axes, &[Some(0.2), None]).as_slice(),
            &[Rest::Hold { gravity: 0.0 }, Rest::Hold { gravity: 0.0 }]
        );
    }

    /// The refute of 2026-11-03: breakaway friction a joint shows at rest is not what holds it
    /// once it slides. 1 N·m of gravity on 0.5 N·m of Coulomb friction falls, however large the
    /// Stribeck term at standstill.
    #[test]
    fn breakaway_friction_does_not_hold_a_joint_up() {
        let breakaway = JointFriction {
            coulomb: 0.5,
            stribeck: 2.0,
            stribeck_velocity: 0.01,
            stribeck_exponent: 1.0,
            ..JointFriction::default()
        };
        assert!(falls(&DM4340.rating(), 1.0, &breakaway));
    }

    #[test]
    fn heat_and_effort_read_against_the_rating() {
        let rated = DM4340.rating();
        assert_eq!(rated.thermal_limit_celsius, Some(100.0));
        assert_eq!(heat(&rated, 89.0), Heat::Within);
        assert_eq!(heat(&rated, 90.0), Heat::Near);
        assert_eq!(heat(&rated, 100.0), Heat::AtLimit);
        let unrated = ActuatorModel::new("feetech_sts3215").rating();
        assert_eq!(heat(&unrated, 150.0), Heat::Within);
        assert_eq!(effort(&rated, 8.0), Effort::Within);
        assert_eq!(effort(&rated, -9.5), Effort::OverRated);
        assert_eq!(effort(&rated, 27.0), Effort::AtPeak);
    }

    #[test]
    fn no_rated_product_carries_a_brake_today() {
        for model in ActuatorModel::ALL {
            assert!(!model.rating().holding_brake, "{}", model.wire());
        }
    }
}
