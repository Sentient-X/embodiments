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
//! caller releases it only once it stays silent (the station's `HOLD_SILENCE`). The gravity
//! of every joint above or below it then depends on an angle nobody knows, so the model
//! searches the unread joint's whole range for each such joint's lowest and highest gravity
//! torque ([`RestModel::gravity_bounds`]). Held, the joint feeds forward the middle of that
//! interval; it is released only where it would not fall anywhere in it. A silent gripper
//! finger moves an arm's gravity by hundredths of a newton-metre, so the arm keeps its full
//! feed-forward; a silent shoulder leaves the elbow an interval of newton-metres, held. Before
//! 2026-11-14 an unread joint was placed at zero (a guessed angle could release an arm), and
//! then for a day its relatives were held with no feed-forward at all (a silent finger sagged
//! a whole arm; the refute of the reference-model merge).

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
    /// For each description joint, how its position enters gravity: exactly as
    /// `C + A cos q + B sin q`, linearly, or neither (it leads mimic joints that turn at
    /// another rate), which decides how its range is searched ([`RestModel::gravity_bounds`]).
    shape: Vec<Shape>,
    /// For each description joint, the range an unread position is searched over: its
    /// limits, or a whole turn.
    range: Vec<[f64; 2]>,
    q: Vec<f64>,
    torque: Vec<f64>,
    low: Vec<f64>,
    high: Vec<f64>,
}

/// How one independent joint's position enters every gravity torque.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Shape {
    /// One revolute or continuous tree joint turns with it, at its own rate: each gravity
    /// torque is `C + A cos q + B sin q` in it (a rotation is linear in its sine and cosine).
    Turning,
    /// Only prismatic tree joints slide with it, at fixed rates: each torque is affine in it.
    Sliding,
    /// Anything else (a leader whose mimics turn at other rates): searched densely.
    Mixed,
}

/// Where a mixed joint's range is searched, and how finely.
const MIXED_SAMPLES: u32 = 129;

fn shapes(tree: &crate::Tree) -> Vec<Shape> {
    use crate::kinematics::JointType;
    tree.joints()
        .iter()
        .map(|axis| {
            let moving: Vec<_> = tree
                .joints
                .iter()
                .filter(|joint| joint.kind != JointType::Fixed && joint.leader == axis.name)
                .collect();
            if moving
                .iter()
                .all(|joint| joint.kind == JointType::Prismatic)
            {
                Shape::Sliding
            } else if let [joint] = moving.as_slice()
                && (joint.multiplier - 1.0).abs() < 1e-12
            {
                Shape::Turning
            } else {
                Shape::Mixed
            }
        })
        .collect()
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
            shape: shapes(dynamics.tree()),
            range: dynamics
                .tree()
                .joints()
                .iter()
                .map(|axis| {
                    axis.limits
                        .unwrap_or([-std::f64::consts::PI, std::f64::consts::PI])
                })
                .collect(),
            scratch: dynamics.scratch(),
            dynamics,
            axis_of,
            q: vec![0.0; width],
            torque: vec![0.0; width],
            low: vec![0.0; width],
            high: vec![0.0; width],
        }
    }

    /// Gravity's torque on each description joint over every position its unread joints may
    /// have, into `self.low` and `self.high`: exact for one unread joint, its intervals summed
    /// over several. A joint the chain does not drive is placed at zero, as the description
    /// stands. False when the model gives nothing finite.
    fn bound(&mut self, measured: &[Option<f64>]) -> bool {
        let mut unread = Vec::new();
        for (k, (q, axis)) in self.q.iter_mut().zip(&self.axis_of).enumerate() {
            *q = match axis.map(|index| measured.get(index).copied().flatten()) {
                Some(Some(value)) => value,
                Some(None) => {
                    unread.push(k);
                    let [lower, upper] = self.range[k];
                    0.5 * (lower + upper)
                }
                None => 0.0,
            };
        }
        let Some(nominal) = self.gravity_with(0, None) else {
            return false;
        };
        self.low.copy_from_slice(&nominal);
        self.high.copy_from_slice(&nominal);
        for k in unread {
            let [lower, upper] = self.range[k];
            // Each torque's lowest and highest over the range, less its nominal value.
            let mut span = vec![[0.0_f64, 0.0_f64]; nominal.len()];
            let take = |span: &mut [[f64; 2]], torque: &[f64]| {
                for ((slot, value), centre) in span.iter_mut().zip(torque).zip(&nominal) {
                    slot[0] = slot[0].min(value - centre);
                    slot[1] = slot[1].max(value - centre);
                }
            };
            match self.shape[k] {
                Shape::Sliding | Shape::Mixed => {
                    let samples = if self.shape[k] == Shape::Sliding {
                        2
                    } else {
                        MIXED_SAMPLES
                    };
                    for step in 0..samples {
                        let share = f64::from(step) / f64::from(samples - 1);
                        let Some(torque) =
                            self.gravity_with(k, Some(lower + share * (upper - lower)))
                        else {
                            return false;
                        };
                        take(&mut span, &torque);
                    }
                }
                Shape::Turning => {
                    // C + A cos q + B sin q from q = 0, pi/2, pi; its extremes over the range
                    // are at the ends or where its derivative vanishes.
                    use std::f64::consts::{FRAC_PI_2, PI, TAU};
                    let (Some(zero), Some(quarter), Some(opposite)) = (
                        self.gravity_with(k, Some(0.0)),
                        self.gravity_with(k, Some(FRAC_PI_2)),
                        self.gravity_with(k, Some(PI)),
                    ) else {
                        return false;
                    };
                    for (j, slot) in span.iter_mut().enumerate() {
                        let c = 0.5 * (zero[j] + opposite[j]);
                        let a = 0.5 * (zero[j] - opposite[j]);
                        let b = quarter[j] - c;
                        let phase = b.atan2(a);
                        let turns =
                            [phase, phase + PI].map(|q| lower + (q - lower).rem_euclid(TAU));
                        for q in [lower, upper]
                            .into_iter()
                            .chain(turns.into_iter().filter(|q| *q <= upper))
                        {
                            let value = c + a * q.cos() + b * q.sin() - nominal[j];
                            slot[0] = slot[0].min(value);
                            slot[1] = slot[1].max(value);
                        }
                    }
                }
            }
            for ((low, high), [down, up]) in self.low.iter_mut().zip(&mut self.high).zip(span) {
                *low += down;
                *high += up;
            }
        }
        self.low
            .iter()
            .chain(&self.high)
            .all(|value| value.is_finite())
    }

    /// Gravity's torque on every description joint at `self.q`, with joint `k` at `at` when
    /// given; `self.q` is left as it was. `None` when the model gives nothing finite.
    fn gravity_with(&mut self, k: usize, at: Option<f64>) -> Option<Vec<f64>> {
        let kept = self.q[k];
        if let Some(q) = at {
            self.q[k] = q;
        }
        let ok = self
            .dynamics
            .gravity_torque_into(&self.q, &mut self.scratch, &mut self.torque)
            .is_ok();
        self.q[k] = kept;
        ok.then(|| self.torque.clone())
    }

    /// The gravity torque each axis may carry at `measured` (one per axis, `None` where it has
    /// no reading at all), lowest and highest: one number twice when every joint it depends on
    /// is read, an interval when one is not (each unread joint searched over its whole range),
    /// `None` where the description does not carry the axis or it has no reading itself.
    pub fn gravity_bounds(
        &mut self,
        axes: &[BoundAxis],
        measured: &[Option<f64>],
    ) -> Axes<Option<[f64; 2]>> {
        let mut out: Axes<Option<[f64; 2]>> = axes.iter().map(|_| None).collect();
        if !self.bound(measured) {
            return out;
        }
        for (joint, axis) in self.axis_of.iter().enumerate() {
            if let Some(index) = *axis
                && measured.get(index).copied().flatten().is_some()
            {
                out[index] = Some([self.low[joint], self.high[joint]]);
            }
        }
        out
    }

    /// Each axis's rest at `measured` (one per axis, `None` where it has no reading at all).
    /// A joint the description carries is released only where it would not fall
    /// ([`falls`]) anywhere in its gravity interval ([`RestModel::gravity_bounds`]); held, its
    /// feed-forward is the interval's middle, the value nearest every torque it may need. One
    /// the description does not carry (its gravity unknown), and one with no reading, is held
    /// without feed-forward unless braked.
    pub fn rests(&mut self, axes: &[BoundAxis], measured: &[Option<f64>]) -> Axes<Rest> {
        let bounds = self.gravity_bounds(axes, measured);
        let mut rests = unmodelled(axes, measured);
        let friction = self.dynamics.friction();
        for (joint, axis) in self.axis_of.iter().enumerate() {
            let Some(index) = *axis else {
                continue;
            };
            let Some([low, high]) = bounds[index] else {
                continue;
            };
            if !rests[index].holds() {
                continue;
            }
            let rating = axes[index].binding.model.rating();
            rests[index] = if falls(&rating, low, &friction[joint])
                || falls(&rating, high, &friction[joint])
            {
                Rest::Hold {
                    gravity: 0.5 * (low + high),
                }
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
            gripper: false,
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

    /// A shoulder carrying an elbow, and a neck on another branch of the same base.
    const BRANCHES: &str = r#"<robot name="b">
      <link name="base"/>
      <link name="upper"><inertial><origin xyz="0.3 0 0"/><mass value="1"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>
      <link name="fore"><inertial><origin xyz="0.2 0 0"/><mass value="0.5"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>
      <link name="head"><inertial><origin xyz="0.1 0 0"/><mass value="0.5"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>
      <joint name="shoulder" type="revolute"><parent link="base"/><child link="upper"/>
        <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
      <joint name="elbow" type="revolute"><parent link="upper"/><child link="fore"/>
        <origin xyz="0.3 0 0"/>
        <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
      <joint name="neck" type="revolute"><parent link="base"/><child link="head"/>
        <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
    </robot>"#;

    /// An unread joint's angle is unknown, so every joint on its branch takes the gravity
    /// interval over its whole range: released only where nothing in it falls, held with the
    /// interval's middle. A joint on another branch keeps its own exact rule.
    #[test]
    fn an_unread_joint_bounds_the_gravity_of_its_branch() {
        let dynamics = Dynamics::new(Tree::parse(BRANCHES).expect("tree"), crate::UPRIGHT);
        let axes = [axis("shoulder"), axis("elbow"), axis("neck")];
        let mut model = RestModel::new(dynamics, &axes);
        let g = crate::STANDARD_GRAVITY;
        // Read throughout, the hanging elbow (q_shoulder + q_elbow = -pi/2) is released.
        let hanging = -std::f64::consts::FRAC_PI_2;
        let read = model.rests(&axes, &[Some(0.0), Some(hanging), Some(hanging)]);
        assert_eq!(read[1], Rest::Release, "{read:?}");
        assert_eq!(read[2], Rest::Release, "{read:?}");
        // The shoulder unread over [-3, 3]: the elbow's 0.1 N·m-per-g arm may point anywhere
        // through q_s + q_e, so its interval reaches 0.5 * 0.2 * g either way and it is held
        // with feed-forward near zero; the neck is on another branch, still released.
        let silent = model.rests(&axes, &[None, Some(hanging), Some(hanging)]);
        assert_eq!(silent[0], Rest::Hold { gravity: 0.0 });
        let Rest::Hold { gravity } = silent[1] else {
            panic!("the elbow is held: {silent:?}");
        };
        assert!(gravity.abs() < 0.5 * 0.2 * g, "{gravity}");
        let bounds = model.gravity_bounds(&axes, &[None, Some(hanging), Some(hanging)]);
        let [low, high] = bounds[1].expect("the elbow is bounded");
        assert!((high - 0.5 * 0.2 * g).abs() < 1e-9 && (low + 0.5 * 0.2 * g).abs() < 1e-9);
        assert_eq!(bounds[0], None);
        assert_eq!(silent[2], Rest::Release, "{silent:?}");
        // The elbow unread with the upper arm level: the shoulder carries 0.3 * g of its own
        // and 0.5 * (0.3 + 0.2 cos q_e) * g of the forearm, held at the middle of that, never
        // at zero.
        let below = model.gravity_bounds(&axes, &[Some(0.0), None, Some(0.0)]);
        let [low, high] = below[0].expect("the shoulder is bounded");
        let own = 0.3 * g + 0.5 * 0.3 * g;
        assert!((high.abs().max(low.abs()) - (own + 0.5 * 0.2 * g)).abs() < 1e-9);
        assert!(
            (high.abs().min(low.abs()) - (own - 0.5 * 0.2 * 3.0_f64.cos().abs() * g)).abs() < 1e-9
        );
        let rests = model.rests(&axes, &[Some(0.0), None, Some(0.0)]);
        assert_eq!(
            rests[0],
            Rest::Hold {
                gravity: 0.5 * (low + high)
            }
        );
        assert!(matches!(rests[2], Rest::Hold { gravity } if gravity.abs() > 0.4));
    }

    /// The refute of the reference-model merge (2026-11-14): a gripper finger out of service
    /// left all seven left-arm joints held with no feed-forward, and the arm sagged 4–6°. A
    /// finger moves the arm's gravity by hundredths of a newton-metre over its stroke, so the
    /// arm keeps its feed-forward, within that of the value with the finger read.
    #[test]
    fn refute_a_silent_gripper_finger_unfeeds_the_whole_arm() {
        const URDF: &str =
            include_str!("../../../render_packs/sentient-static-upper-body/robot.urdf");
        let dynamics = Dynamics::of(URDF, crate::UPRIGHT).expect("description");
        let names: Vec<String> = dynamics
            .tree()
            .joints()
            .iter()
            .map(|joint| joint.name.clone())
            .collect();
        let axes: Vec<BoundAxis> = names.iter().map(|name| axis(name)).collect();
        let at = |name: &str| names.iter().position(|n| n == name).expect(name);
        let arm = [
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_upper_arm_yaw_joint",
            "left_elbow_pitch_joint",
            "left_forearm_yaw_joint",
            "left_wrist_pitch_joint",
            "left_wrist_roll_joint",
        ];
        let finger = names
            .iter()
            .position(|n| n.starts_with("left") && n.contains("finger"))
            .expect("a left finger joint");
        let mut measured: Vec<Option<f64>> = vec![Some(0.0); names.len()];
        measured[at("left_shoulder_roll_joint")] = Some(1.2);
        measured[at("left_elbow_pitch_joint")] = Some(-1.2);
        let mut model = RestModel::new(dynamics, &axes);
        let read = model.rests(&axes, &measured);
        measured[finger] = None;
        let silent = model.rests(&axes, &measured);
        assert_eq!(silent[finger], Rest::Hold { gravity: 0.0 });
        for name in arm {
            let index = at(name);
            match (read[index], silent[index]) {
                (Rest::Release, Rest::Release) => {}
                (Rest::Hold { gravity: want }, Rest::Hold { gravity: got }) => {
                    assert!((got - want).abs() < 0.05, "{name}: {got} against {want}");
                }
                (Rest::Release, Rest::Hold { gravity }) => {
                    assert!(gravity.abs() < 0.05, "{name}: held at {gravity}");
                }
                (want, got) => panic!("{name}: {got:?} against {want:?}"),
            }
        }
        // The arm raised sideways: its shoulder roll carries newton-metres, fed forward.
        let Rest::Hold { gravity } = silent[at("left_shoulder_roll_joint")] else {
            panic!("the shoulder roll is held: {silent:?}");
        };
        assert!(gravity.abs() > 2.0, "{gravity}");
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
