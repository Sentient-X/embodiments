//! Power rails, as commissioning measured them, against each actuator model's rated supply.
//!
//! An actuator run above its model's supported maximum (the static upper body's DM-J8009P
//! shoulders and torso on a 54 V battery, rated 24–48 V, until a 48 V converter is fitted) is
//! **derated**, not refused: its axis moves slower, accelerates and brakes gentler (regenerated
//! energy lifts the rail further, toward the motor's own over-voltage trip) and is sent less
//! feed-forward torque ([`OVER_RAIL`]). A rail more than [`REFUSE_ABOVE`] times the maximum is
//! refused outright: the body does not open with that axis in service. Both follow from data,
//! so a derate lifts on its own once commissioning records a rail in range.
//!
//! The derate is [`OVER_RAIL`] unless commissioning declares another speed for this robot
//! ([`Rails::with_derate`], its `derate` entry): a person's recorded decision, who and why in
//! its `decided` text (the owner's 0.6 rad/s for the J8009P on the 54 V battery, 2026-10-26),
//! data like the rails, never a code constant or a run flag. Its acceleration keeps
//! [`OVER_RAIL`]'s ratio to its speed (a gentle brake regenerates less), its feed-forward
//! torque share is [`OVER_RAIL`]'s, and no axis is derated faster than its model's rated speed.
//!
//! [`Rails`] is commissioning's `rails` entry: measured volts by selector, where a selector is
//! `<port>/<model>`, a model's wire value (`damiao_dm8009p`), a port name (`can`), or `*`; the
//! most specific selector that matches an axis is its rail. [`RailCheck`] compares one axis's
//! rail against its model's [`crate::ActuatorRating`]; [`AxisDerate`] is a derated axis as data,
//! reported by [`crate::ActuatorChain::derated`] and applied by [`crate::Governor`].

use std::collections::BTreeMap;
use std::fmt;

use crate::binding::{ActuatorModel, BoundAxis, is_port_name};

/// How far over its model's supported maximum a rail may be and still be driven, derated.
pub const REFUSE_ABOVE: f64 = 1.15;

/// What a derated axis is held to.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DerateLimits {
    /// The fastest a command moves the axis, unit/s.
    pub max_speed: f64,
    /// The fastest the commanded speed changes, unit/s²: a gentle brake regenerates less.
    pub max_acceleration: f64,
    /// The share of the model's peak torque feed-forward may send.
    pub torque_share: f64,
}

/// What a declared derate's decision says when it is faster than [`OVER_RAIL`]: lifting the
/// over-rail protection is recorded as such, never as a side effect of a number.
pub const LIFTS_DERATE: &str = "lifts the derate";

/// The derate of an axis whose rail is over its model's supported maximum.
pub const OVER_RAIL: DerateLimits = DerateLimits {
    max_speed: 0.3,
    max_acceleration: 1.5,
    torque_share: 0.5,
};

/// A rail over its model's supported maximum, V.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OverRail {
    pub rail_volts: f64,
    pub max_volts: f64,
}

impl fmt::Display for OverRail {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "over-rail {} V > {} V", self.rail_volts, self.max_volts)
    }
}

/// One axis's rail against its model's rated supply.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RailCheck {
    /// No rail recorded for the axis, or its model publishes no supply range.
    Unchecked,
    /// Within the model's supported range.
    Within,
    /// Over the maximum by at most [`REFUSE_ABOVE`]: driven, [`OVER_RAIL`] derated.
    Derated(OverRail),
    /// Over the maximum by more than [`REFUSE_ABOVE`]: not enabled.
    Refused(OverRail),
}

impl RailCheck {
    /// `model` on a rail of `rail_volts`, or [`RailCheck::Unchecked`] with none.
    #[must_use]
    pub fn of(model: ActuatorModel, rail_volts: Option<f64>) -> Self {
        let (Some(rail_volts), Some([_, max_volts])) = (rail_volts, model.rating().supply_volts)
        else {
            return Self::Unchecked;
        };
        let over = OverRail {
            rail_volts,
            max_volts,
        };
        if rail_volts <= max_volts {
            Self::Within
        } else if rail_volts <= max_volts * REFUSE_ABOVE {
            Self::Derated(over)
        } else {
            Self::Refused(over)
        }
    }
}

/// A derated axis, by its index in its chain's native state order.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisDerate {
    pub axis: usize,
    pub over: OverRail,
    pub limits: DerateLimits,
}

impl fmt::Display for AxisDerate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "derated: {}, held to {:.2} rad/s and {:.1} rad/s², feed-forward at most {:.0} % of peak",
            self.over,
            self.limits.max_speed,
            self.limits.max_acceleration,
            self.limits.torque_share * 100.0
        )
    }
}

/// The derated axes of `axes` on `rails`, by index.
#[must_use]
pub fn derates(axes: &[BoundAxis], rails: &Rails) -> Vec<AxisDerate> {
    axes.iter()
        .enumerate()
        .filter_map(|(axis, bound)| match rails.check(bound) {
            RailCheck::Derated(over) => {
                let mut limits = rails.derate();
                // No faster than the model's rated speed; a model with none recorded is held to
                // the automatic derate at most, never lifted on a guess.
                let ceiling = bound
                    .binding
                    .model
                    .rating()
                    .rated_speed_rad_s
                    .unwrap_or(OVER_RAIL.max_speed);
                if limits.max_speed > ceiling {
                    limits.max_acceleration *= ceiling / limits.max_speed;
                    limits.max_speed = ceiling;
                }
                Some(AxisDerate { axis, over, limits })
            }
            _ => None,
        })
        .collect()
}

/// A rails entry commissioning cannot record.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RailsError {
    #[error("rail selector {0:?} is not *, a port, a model or <port>/<model>")]
    Selector(String),
    #[error("rail {0} needs a finite, positive voltage")]
    Volts(String),
    #[error("a declared derate needs a finite, positive speed and the decision it records")]
    Derate,
    #[error(
        "a declared derate faster than the automatic one must say \"lifts the derate\" in its \
         decision to be recorded"
    )]
    Lifts,
}

/// Commissioning's measured rails: volts by selector (see the module docs). A selector that is
/// no registry model reads as a port; the body's opener refuses one naming no port it has.
#[derive(Clone, Debug, PartialEq)]
pub struct Rails {
    volts: BTreeMap<String, f64>,
    derate: DerateLimits,
    /// Who decided a declared derate, and why; `None` for [`OVER_RAIL`].
    decided: Option<String>,
}

impl Default for Rails {
    fn default() -> Self {
        Self {
            volts: BTreeMap::new(),
            derate: OVER_RAIL,
            decided: None,
        }
    }
}

/// Every voltage is finite, so equality is total.
impl Eq for Rails {}

impl Rails {
    /// `volts` by selector, each selector and voltage checked.
    ///
    /// # Errors
    ///
    /// A selector that is not `*`, a port name, a registry model or `<port>/<model>`, and a
    /// voltage that is not finite and positive.
    pub fn new(volts: BTreeMap<String, f64>) -> Result<Self, RailsError> {
        for (selector, value) in &volts {
            let valid = selector == "*"
                || ActuatorModel::parse(selector).is_some()
                || is_port_name(selector)
                || selector.split_once('/').is_some_and(|(port, model)| {
                    is_port_name(port) && ActuatorModel::parse(model).is_some()
                });
            if !valid {
                return Err(RailsError::Selector(selector.clone()));
            }
            if !(value.is_finite() && *value > 0.0) {
                return Err(RailsError::Volts(selector.clone()));
            }
        }
        Ok(Self {
            volts,
            ..Self::default()
        })
    }

    /// These rails with the derate commissioning declares for this robot, `decided` naming
    /// who decided it and why: an over-rail axis is held to `max_speed` (no faster than its
    /// model's rated speed) and that speed's [`OVER_RAIL`] acceleration instead of
    /// [`OVER_RAIL`]'s own.
    ///
    /// # Errors
    ///
    /// [`RailsError::Derate`] for a speed that is not finite and positive, and an empty
    /// decision.
    pub fn with_derate(mut self, max_speed: f64, decided: &str) -> Result<Self, RailsError> {
        if !(max_speed.is_finite() && max_speed > 0.0) || decided.trim().is_empty() {
            return Err(RailsError::Derate);
        }
        // Lifting the over-rail protection is a decision that says so: a derate faster than the
        // automatic one names it in its record, so a typo (6 for 0.6) is refused, not obeyed.
        if max_speed > OVER_RAIL.max_speed && !decided.contains(LIFTS_DERATE) {
            return Err(RailsError::Lifts);
        }
        self.derate = DerateLimits {
            max_speed,
            max_acceleration: max_speed * OVER_RAIL.max_acceleration / OVER_RAIL.max_speed,
            ..OVER_RAIL
        };
        self.decided = Some(decided.trim().to_owned());
        Ok(self)
    }

    /// Volts by selector, as recorded.
    #[must_use]
    pub const fn volts(&self) -> &BTreeMap<String, f64> {
        &self.volts
    }

    /// The derate an over-rail axis is held to: the declared one, else [`OVER_RAIL`].
    #[must_use]
    pub const fn derate(&self) -> DerateLimits {
        self.derate
    }

    /// Who decided a declared derate and why (else it is [`OVER_RAIL`]).
    #[must_use]
    pub fn derate_decided(&self) -> Option<&str> {
        self.decided.as_deref()
    }

    /// The rail of `axis`: its `<port>/<model>`, else its model's, else its port's, else `*`.
    #[must_use]
    pub fn of(&self, axis: &BoundAxis) -> Option<f64> {
        let model = axis.binding.model.wire();
        [
            format!("{}/{model}", axis.port),
            model.to_owned(),
            axis.port.clone(),
            "*".to_owned(),
        ]
        .iter()
        .find_map(|selector| self.volts.get(selector).copied())
    }

    /// `axis`'s rail against its model's rated supply.
    #[must_use]
    pub fn check(&self, axis: &BoundAxis) -> RailCheck {
        RailCheck::of(axis.binding.model, self.of(axis))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::binding::{ActuatorBinding, ActuatorBus};

    fn axis(model: &'static str, port: &str) -> BoundAxis {
        BoundAxis {
            joint: "j".to_owned(),
            port: port.to_owned(),
            lower: -1.0,
            upper: 1.0,
            binding: ActuatorBinding {
                model: ActuatorModel::new(model),
                bus: ActuatorBus::new("damiao_can"),
                bus_id: 1,
                sign: 1,
                reduction: 1.0,
            },
            zero: 0.0,
            direction: 1,
            home: None,
            motion: crate::binding::AxisMotion {
                speed: 1.0,
                acceleration: 10.0,
                inertia: 0.0,
            },
        }
    }

    fn rails(entries: &[(&str, f64)]) -> Rails {
        Rails::new(entries.iter().map(|(k, v)| ((*k).to_owned(), *v)).collect()).expect("rails")
    }

    #[test]
    fn the_most_specific_selector_is_the_rail() {
        let rails = rails(&[
            ("*", 24.0),
            ("damiao_dm8009p", 54.0),
            ("can/damiao_dm8009p", 47.0),
        ]);
        assert_eq!(rails.of(&axis("damiao_dm8009p", "can")), Some(47.0));
        assert_eq!(rails.of(&axis("damiao_dm8009p", "can2")), Some(54.0));
        assert_eq!(rails.of(&axis("damiao_dm4340", "can")), Some(24.0));
        assert_eq!(Rails::default().of(&axis("damiao_dm4340", "can")), None);
    }

    #[test]
    fn a_rail_over_the_maximum_derates_then_refuses() {
        let j8009p = ActuatorModel::new("damiao_dm8009p");
        assert_eq!(RailCheck::of(j8009p, Some(48.0)), RailCheck::Within);
        let over = |rail_volts| OverRail {
            rail_volts,
            max_volts: 48.0,
        };
        assert_eq!(
            RailCheck::of(j8009p, Some(54.0)),
            RailCheck::Derated(over(54.0))
        );
        assert_eq!(
            RailCheck::of(j8009p, Some(55.1)),
            RailCheck::Derated(over(55.1))
        );
        assert_eq!(
            RailCheck::of(j8009p, Some(56.0)),
            RailCheck::Refused(over(56.0))
        );
        assert_eq!(RailCheck::of(j8009p, None), RailCheck::Unchecked);
        // The DM8009 publishes no supply range: nothing to check against.
        let dm8009 = ActuatorModel::new("damiao_dm8009");
        assert_eq!(RailCheck::of(dm8009, Some(80.0)), RailCheck::Unchecked);
        let derate = AxisDerate {
            axis: 0,
            over: over(54.0),
            limits: OVER_RAIL,
        };
        assert_eq!(
            derate.to_string(),
            "derated: over-rail 54 V > 48 V, held to 0.30 rad/s and 1.5 rad/s², feed-forward at most 50 % of peak"
        );
    }

    #[test]
    fn rails_refuse_unknown_selectors_and_bad_volts() {
        let one = |k: &str, v: f64| Rails::new(BTreeMap::from([(k.to_owned(), v)]));
        assert!(matches!(one("Can", 24.0), Err(RailsError::Selector(_))));
        assert!(matches!(
            one("can/damiao_dm9999", 24.0),
            Err(RailsError::Selector(_))
        ));
        assert!(matches!(one("*", 0.0), Err(RailsError::Volts(_))));
        assert!(matches!(one("*", f64::NAN), Err(RailsError::Volts(_))));
    }

    #[test]
    fn a_declared_derate_replaces_the_automatic_one_and_a_bad_one_is_refused() {
        let rails = rails(&[("damiao_dm8009p", 54.0)]);
        let axes = [axis("damiao_dm8009p", "can")];
        assert_eq!(derates(&axes, &rails)[0].limits, OVER_RAIL);
        assert_eq!(rails.derate_decided(), None);
        let declared = rails
            .clone()
            .with_derate(0.6, "owner, 2026-10-26: lifts the derate")
            .expect("declared");
        assert_eq!(
            declared.derate_decided(),
            Some("owner, 2026-10-26: lifts the derate")
        );
        let derate = derates(&axes, &declared)[0];
        // The automatic ratio of acceleration to speed (5 s^-1), the automatic torque share.
        assert_eq!(
            (
                derate.limits.max_speed,
                derate.limits.max_acceleration,
                derate.limits.torque_share
            ),
            (0.6, 3.0, OVER_RAIL.torque_share)
        );
        // Never faster than the model's rated speed (100 rpm).
        let wild = rails
            .clone()
            .with_derate(100.0, "a typo that lifts the derate")
            .expect("declared");
        let rated = ActuatorModel::parse("damiao_dm8009p")
            .expect("a model")
            .rating()
            .rated_speed_rad_s
            .expect("rated");
        assert!((derates(&axes, &wild)[0].limits.max_speed - rated).abs() < 1e-12);
        // Faster than the automatic derate without saying so: refused, typed.
        assert!(matches!(
            rails.clone().with_derate(6.0, "owner"),
            Err(RailsError::Lifts)
        ));
        // Slower than the automatic derate needs no such words.
        assert!(rails.clone().with_derate(0.2, "owner: gentler").is_ok());
        // A model with no rated speed is never derated faster than the automatic derate.
        let unrated = [axis("dynamixel_xm430_w350", "serial")];
        let lifted = Rails::new(BTreeMap::from([("dynamixel_xm430_w350".to_owned(), 16.0)]))
            .expect("rails")
            .with_derate(5.0, "lifts the derate")
            .expect("declared");
        assert!(
            (derates(&unrated, &lifted)[0].limits.max_speed - OVER_RAIL.max_speed).abs() < 1e-12
        );
        for (speed, decided) in [(0.0, "x"), (f64::NAN, "x"), (-1.0, "x"), (0.6, "  ")] {
            assert_eq!(
                rails.clone().with_derate(speed, decided),
                Err(RailsError::Derate)
            );
        }
    }
}
