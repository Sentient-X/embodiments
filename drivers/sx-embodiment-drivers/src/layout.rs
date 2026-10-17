//! What one built robot is commanded with and observed by, as the platform derives it.
//!
//! The platform derives a robot's action and observation interfaces in Python
//! (`sx_actions.robot_layout.robot_layout_document`): an ordinary `ActionInterface` over the
//! coordinates the body binds to an actuator, and the `StateInterface` of the same coordinates,
//! each serialized whole with its content id. This crate never re-derives either: [`RobotLayout`]
//! is a parser of that rendering (`sx.robot-action-layout/v1`), refusing one for another robot
//! instance ([`RobotLayout::parse`]) and one whose command coordinates are not, in order, the
//! joints a drive drives ([`RobotLayout::check_drive`]). Every pod that commands or observes a
//! robot (a station's control core, a teleoperation session's controls, a served policy's
//! handshake) reads the same document through it.

use serde::Deserialize;
use thiserror::Error;

use crate::drive::InstanceDrive;

/// The schema `robot_layout_document` renders.
pub const LAYOUT_SCHEMA: &str = "sx.robot-action-layout/v1";

/// One command coordinate, in the interface's vector order.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LayoutControl {
    /// The native joint the coordinate commands.
    pub joint: String,
    pub component_id: String,
    pub coordinate_id: String,
    pub lower: f64,
    pub upper: f64,
}

/// One robot instance's rendered action and observation interfaces.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotLayout {
    schema: String,
    /// The body's content id.
    pub embodiment: String,
    /// The robot instance's content id.
    pub instance: String,
    /// The action interface's content id, as `sx_actions` computes it.
    pub action_interface_id: String,
    /// The state interface's content id, as `sx_actions` computes it.
    pub state_interface_id: String,
    /// The action interface's exact control period.
    pub period_ns: u64,
    pub controls: Vec<LayoutControl>,
    /// The action interface document itself, carried for any reader that needs more than the
    /// ids and coordinates; never re-derived here.
    pub action_interface: serde_json::Value,
    /// The state interface document itself.
    pub state_interface: serde_json::Value,
}

/// A rendering that is not this robot's layout.
#[derive(Debug, Error, PartialEq)]
pub enum LayoutError {
    #[error("the action layout does not parse: {0}")]
    Document(String),
    #[error("the action layout was rendered for robot instance {found}, not {expected}")]
    Instance { expected: String, found: String },
    #[error("the action layout is of body {layout}, and the drive of body {drive}")]
    Embodiment { layout: String, drive: String },
    #[error("the action layout commands joints {layout:?}, and the drive drives {drive:?}")]
    Joints {
        layout: Vec<String>,
        drive: Vec<String>,
    },
}

fn content_id(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
}

impl RobotLayout {
    /// Parse a rendered layout for `expected`, the robot instance the caller is configured as.
    ///
    /// # Errors
    ///
    /// Refuses anything but this schema, a rendering for another instance, an interface id that
    /// is not a content id or differs from its document's, a zero period, and a coordinate with a
    /// blank identity or bounds no actuator could hold.
    pub fn parse(document: &str, expected: &str) -> Result<Self, LayoutError> {
        let layout: Self = serde_json::from_str(document)
            .map_err(|error| LayoutError::Document(error.to_string()))?;
        if layout.schema != LAYOUT_SCHEMA {
            return Err(LayoutError::Document(format!(
                "schema {} is not {LAYOUT_SCHEMA}",
                layout.schema
            )));
        }
        if layout.instance != expected {
            return Err(LayoutError::Instance {
                expected: expected.to_owned(),
                found: layout.instance,
            });
        }
        let refuse = |reason: &str| Err(LayoutError::Document(reason.to_owned()));
        if !content_id(&layout.action_interface_id) || !content_id(&layout.state_interface_id) {
            return refuse("an interface id is a lowercase SHA-256");
        }
        if layout.action_interface["id"] != layout.action_interface_id.as_str() {
            return refuse("the action interface id is not its document's");
        }
        if layout.period_ns == 0 || layout.controls.is_empty() {
            return refuse("a layout has a period and at least one coordinate");
        }
        layout.check_documents().map_err(LayoutError::Document)?;
        let blank = |value: &str| value.is_empty() || value.chars().any(char::is_whitespace);
        if layout.controls.iter().any(|control| {
            blank(&control.joint)
                || blank(&control.component_id)
                || blank(&control.coordinate_id)
                || !control.lower.is_finite()
                || !control.upper.is_finite()
                || control.lower > control.upper
        }) {
            return refuse("every coordinate names its joint and has finite, ordered bounds");
        }
        Ok(layout)
    }

    /// The coordinates, bounds, rate and body the controls and ids state are the ones the
    /// carried interface documents state, in order. Their content ids are `sx_actions`'
    /// (golden-held in `packages/sx-actions/tests/test_robot_layout.py`); this check refuses a
    /// rendering whose summary disagrees with the documents it was rendered from.
    fn check_documents(&self) -> Result<(), String> {
        let action = &self.action_interface;
        let state = &self.state_interface;
        if action["embodiment_id"] != self.embodiment.as_str()
            || state["embodiment_id"] != self.embodiment.as_str()
        {
            return Err("an interface document is of another body".to_owned());
        }
        let hz = action["control_hz"]
            .as_f64()
            .ok_or("the action interface has no rate")?;
        #[allow(clippy::cast_precision_loss)]
        let period = 1e9 / self.period_ns as f64;
        if (hz - period).abs() > 1e-9 * hz.max(1.0) {
            return Err("the period is not the action interface's rate".to_owned());
        }
        let commands: Vec<&serde_json::Value> = action["groups"]
            .as_array()
            .ok_or("the action interface has no groups")?
            .iter()
            .flat_map(|group| group["coordinates"].as_array().into_iter().flatten())
            .collect();
        let fields = state["fields"]
            .as_array()
            .ok_or("the state interface has no fields")?;
        if commands.len() != self.controls.len() || fields.len() != self.controls.len() {
            return Err("the controls are not the interfaces' coordinates".to_owned());
        }
        for ((control, command), field) in self.controls.iter().zip(commands).zip(fields) {
            let same = |value: &serde_json::Value| {
                value["component_id"] == control.component_id.as_str()
                    && value["coordinate_id"] == control.coordinate_id.as_str()
            };
            if !same(command)
                || !same(field)
                || command["bounds"]["lower"].as_f64() != Some(control.lower)
                || command["bounds"]["upper"].as_f64() != Some(control.upper)
            {
                return Err(format!(
                    "control {} is not its interfaces' coordinate, in order",
                    control.joint
                ));
            }
        }
        Ok(())
    }

    /// Refuse a layout of another body than `drive`'s, or of other joints than it drives.
    ///
    /// # Errors
    ///
    /// [`LayoutError::Embodiment`] or [`LayoutError::Joints`].
    pub fn check_drive(&self, drive: &InstanceDrive) -> Result<(), LayoutError> {
        if self.embodiment != drive.embodiment {
            return Err(LayoutError::Embodiment {
                layout: self.embodiment.clone(),
                drive: drive.embodiment.clone(),
            });
        }
        let layout: Vec<String> = self.controls.iter().map(|c| c.joint.clone()).collect();
        let driven: Vec<String> = drive.joints.iter().map(|j| j.joint.clone()).collect();
        if layout != driven {
            return Err(LayoutError::Joints {
                layout,
                drive: driven,
            });
        }
        Ok(())
    }

    /// The control period.
    #[must_use]
    pub const fn period(&self) -> std::time::Duration {
        std::time::Duration::from_nanos(self.period_ns)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn document(instance: &str) -> serde_json::Value {
        let id = "a".repeat(64);
        serde_json::json!({
            "schema": LAYOUT_SCHEMA,
            "embodiment": "b".repeat(64),
            "instance": instance,
            "action_interface_id": id,
            "state_interface_id": "c".repeat(64),
            "period_ns": 20_000_000_u64,
            "controls": [{"joint": "j1", "component_id": "arm", "coordinate_id": "j1",
                          "lower": -1.0, "upper": 1.0}],
            "action_interface": {"id": id, "embodiment_id": "b".repeat(64), "control_hz": 50.0,
                "groups": [{"coordinates": [{"component_id": "arm", "coordinate_id": "j1",
                    "bounds": {"lower": -1.0, "upper": 1.0}}]}]},
            "state_interface": {"embodiment_id": "b".repeat(64),
                "fields": [{"component_id": "arm", "coordinate_id": "j1"}]},
        })
    }

    #[test]
    fn a_layout_parses_for_its_instance_only() {
        let layout = RobotLayout::parse(&document("unit").to_string(), "unit").expect("layout");
        assert_eq!(layout.period(), std::time::Duration::from_millis(20));
        assert!(matches!(
            RobotLayout::parse(&document("unit").to_string(), "other"),
            Err(LayoutError::Instance { .. })
        ));
    }

    #[test]
    fn an_id_its_document_does_not_carry_and_unordered_bounds_are_refused() {
        let mut wrong = document("unit");
        wrong["action_interface"]["id"] = serde_json::json!("d".repeat(64));
        assert!(RobotLayout::parse(&wrong.to_string(), "unit").is_err());
        let mut bounds = document("unit");
        bounds["controls"][0]["lower"] = serde_json::json!(2.0);
        assert!(RobotLayout::parse(&bounds.to_string(), "unit").is_err());
        let mut divergent = document("unit");
        divergent["action_interface"]["groups"][0]["coordinates"][0]["bounds"]["upper"] =
            serde_json::json!(0.5);
        assert!(RobotLayout::parse(&divergent.to_string(), "unit").is_err());
        let mut rate = document("unit");
        rate["action_interface"]["control_hz"] = serde_json::json!(30.0);
        assert!(RobotLayout::parse(&rate.to_string(), "unit").is_err());
        let mut unknown = document("unit");
        unknown["extra"] = serde_json::json!(1);
        assert!(RobotLayout::parse(&unknown.to_string(), "unit").is_err());
    }
}
