//! One built robot's joint calibrations, as the registry resolves them for a driver.
//!
//! A robot instance (`sx_embodiments.instance.RobotInstance`) places serial units into a
//! body's slots and selects their calibrations; `sx_embodiments.instance.resolve` binds the two
//! and `drive_document` renders, for every driven joint in native state order, the zero and
//! direction its unit was measured at, or the nominal ones. That rendering is the one source
//! here: a driver never binds an instance to its body itself. [`InstanceDrive::parse`] takes the
//! robot instance the caller is configured as and refuses a document resolved for any other, so
//! a drive belongs to one physical robot; [`BodyBindings::calibrated`] refuses one of another
//! body and any joint missing, extra or out of order, then applies each through
//! [`BoundAxis::calibrated`](crate::BoundAxis::calibrated), the one seam a joint calibration
//! enters.

use serde::Deserialize;

use crate::body::{BodyBindings, BodyError};

/// The schema `drive_document` renders.
pub const DRIVE_SCHEMA: &str = "sx.instance-drive/v1";

/// One instance's resolved drive: each driven joint's zero and direction.
#[derive(Clone, Debug, PartialEq)]
pub struct InstanceDrive {
    /// The body's content id.
    pub embodiment: String,
    /// The instance's content id.
    pub instance: String,
    pub joints: Vec<JointDrive>,
}

/// One driven joint as its unit was measured, or nominal.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct JointDrive {
    pub joint: String,
    pub zero: f64,
    pub direction: i8,
    /// The calibration id the zero and direction came from; `None` when nominal.
    pub calibration: Option<String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    schema: String,
    embodiment: String,
    instance: String,
    joints: Vec<JointDrive>,
}

impl InstanceDrive {
    /// Parse a rendered `sx.instance-drive/v1` document resolved for `expected`, the content id
    /// of the robot instance the caller is configured as: another unit of the same body has
    /// other zeros, so its drive is refused.
    ///
    /// # Errors
    ///
    /// Returns [`BodyError::Document`] for anything but such a document, and
    /// [`BodyError::Unit`] for one resolved for another instance.
    pub fn parse(document: &str, expected: &str) -> Result<Self, BodyError> {
        let Document {
            schema,
            embodiment,
            instance,
            joints,
        } = serde_json::from_str(document)
            .map_err(|error| BodyError::Document(error.to_string()))?;
        if schema != DRIVE_SCHEMA {
            return Err(BodyError::Document(format!(
                "not a {DRIVE_SCHEMA} document"
            )));
        }
        if instance != expected {
            return Err(BodyError::Unit {
                expected: expected.to_owned(),
                found: instance,
            });
        }
        Ok(Self {
            embodiment,
            instance,
            joints,
        })
    }
}

impl BodyBindings {
    /// These axes as one built robot drives them: `drive`'s zero and direction on each.
    ///
    /// # Errors
    ///
    /// Refuses a drive of another body (one assigned an id this body supersedes is this body),
    /// joints that are not exactly this body's driven joints
    /// in state order, and any zero or direction a calibration cannot carry.
    pub fn calibrated(self, drive: &InstanceDrive) -> Result<Self, BodyError> {
        if !self.is(&drive.embodiment) {
            return Err(BodyError::Instance {
                embodiment: self.embodiment,
                found: drive.embodiment.clone(),
            });
        }
        let joints: Vec<&str> = drive.joints.iter().map(|j| j.joint.as_str()).collect();
        self.require_joints(&joints)?;
        let axes = self
            .axes
            .into_iter()
            .zip(&drive.joints)
            .map(|(axis, joint)| axis.calibrated(joint.zero, joint.direction))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self { axes, ..self })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::BindingError;

    const BODY: &str = r#"{"embodiment": "b", "id": "0", "axes": [
        {"joint": "j1", "lower": -1.0, "upper": 1.0, "model": "damiao_dm4310",
         "bus": "damiao_can", "port": "can", "bus_id": 1, "sign": 1, "reduction": 1.0, "speed": 1.0, "acceleration": 10.0},
        {"joint": "j2", "lower": -1.0, "upper": 1.0, "model": "damiao_dm4310",
         "bus": "damiao_can", "port": "can", "bus_id": 2, "sign": 1, "reduction": 1.0, "speed": 1.0, "acceleration": 10.0}]}"#;
    const DRIVE: &str = r#"{"schema": "sx.instance-drive/v1", "embodiment": "0",
        "instance": "i", "joints": [
        {"joint": "j1", "zero": 0.1, "direction": -1, "calibration": "c"},
        {"joint": "j2", "zero": 0.0, "direction": 1, "calibration": null}]}"#;

    fn body() -> BodyBindings {
        BodyBindings::parse(BODY).expect("parses")
    }

    fn drive(document: &str) -> InstanceDrive {
        InstanceDrive::parse(document, "i").expect("parses")
    }

    #[test]
    fn a_drive_calibrates_each_joint_once() {
        let body = body().calibrated(&drive(DRIVE)).expect("calibrates");
        let j1 = body.axis("j1").expect("j1");
        assert!((j1.zero - 0.1).abs() < f64::EPSILON);
        assert_eq!(j1.direction, -1);
        assert_eq!(body.axis("j2").expect("j2").direction, 1);
    }

    #[test]
    fn a_drive_of_another_robot_body_or_joints_is_refused() {
        assert!(matches!(
            InstanceDrive::parse(DRIVE, "another unit of the same body"),
            Err(BodyError::Unit { .. })
        ));
        let other = drive(&DRIVE.replace(r#""embodiment": "0""#, r#""embodiment": "1""#));
        assert!(matches!(
            body().calibrated(&other),
            Err(BodyError::Instance { .. })
        ));
        let swapped = drive(
            &DRIVE
                .replace(r#""joint": "j1""#, r#""joint": "jx""#)
                .replace(r#""joint": "j2""#, r#""joint": "j1""#)
                .replace(r#""joint": "jx""#, r#""joint": "j2""#),
        );
        assert!(matches!(
            body().calibrated(&swapped),
            Err(BodyError::Joints { .. })
        ));
        let reversed_twice = drive(&DRIVE.replace(r#""direction": -1"#, r#""direction": 2"#));
        assert_eq!(
            body().calibrated(&reversed_twice),
            Err(BodyError::Binding(BindingError::Direction("j1".to_owned())))
        );
        assert!(
            InstanceDrive::parse(
                &DRIVE.replace("instance-drive/v1", "instance-drive/v2"),
                "i"
            )
            .is_err()
        );
        assert!(
            InstanceDrive::parse(
                &DRIVE.replace(r#""calibration": null"#, r#""calibration": null, "x": 1"#),
                "i"
            )
            .is_err()
        );
    }
}
