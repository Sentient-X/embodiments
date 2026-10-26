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
        write!(f, "derated: {}", self.over)
    }
}

/// The derated axes of `axes` on `rails`, by index.
#[must_use]
pub fn derates(axes: &[BoundAxis], rails: &Rails) -> Vec<AxisDerate> {
    axes.iter()
        .enumerate()
        .filter_map(|(axis, bound)| match rails.check(bound) {
            RailCheck::Derated(over) => Some(AxisDerate {
                axis,
                over,
                limits: OVER_RAIL,
            }),
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
}

/// Commissioning's measured rails: volts by selector (see the module docs). A selector that is
/// no registry model reads as a port; the body's opener refuses one naming no port it has.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Rails(BTreeMap<String, f64>);

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
        Ok(Self(volts))
    }

    /// Volts by selector, as recorded.
    #[must_use]
    pub const fn volts(&self) -> &BTreeMap<String, f64> {
        &self.0
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
        .find_map(|selector| self.0.get(selector).copied())
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
        assert_eq!(derate.to_string(), "derated: over-rail 54 V > 48 V");
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
}
