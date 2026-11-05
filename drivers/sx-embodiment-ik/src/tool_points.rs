//! Whether a body's tool points were checked on its hardware, as its render pack's `pack.json`
//! records it (`sx_embodiments.tool_points`, embodiments #242): nothing drives a tool point a
//! person has not checked on the real robot.
//!
//! A check names the embodiment id (one revision's content digest) and the link checked on each
//! side. It holds for that revision and those links only: a pack whose `tcp_frames` are not the
//! checked points, or a robot opened as another revision, is unchecked, and a new description
//! or a moved tool point asks for a new check. Absence is never a check.

use std::collections::BTreeMap;
use std::fmt;

use serde::Deserialize;

/// A body's tool points, checked on its hardware for one revision.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CheckedToolPoints {
    /// The embodiment id the check was of.
    pub embodiment: String,
    /// The link checked on each side: `{"left": link, "right": link}`.
    pub points: BTreeMap<String, String>,
}

impl CheckedToolPoints {
    /// Whether `frame` is one of the checked tool points.
    #[must_use]
    pub fn drives(&self, frame: &str) -> bool {
        self.points.values().any(|point| point == frame)
    }
}

/// Why a body's tool points are not driven.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ToolPointsUnchecked {
    /// The pack records no check: nobody has checked this body's tool points on its hardware.
    NoRecord,
    /// The check was of another revision than the robot opened.
    OtherRevision { checked: String, opened: String },
    /// The pack's tool points are not the links the check names.
    PointsMoved,
    /// The pack does not read as a render pack.
    Malformed(String),
}

impl fmt::Display for ToolPointsUnchecked {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("tool point not checked on this robot yet: ")?;
        match self {
            Self::NoRecord => f.write_str(
                "nobody has checked where its grippers close on the real robot (robot-bench)",
            ),
            Self::OtherRevision { checked, opened } => write!(
                f,
                "the check was of revision {}, and this robot runs {}",
                short(checked),
                short(opened)
            ),
            Self::PointsMoved => f.write_str("its tool points moved since they were checked"),
            Self::Malformed(error) => write!(f, "its render pack does not read ({error})"),
        }
    }
}

impl std::error::Error for ToolPointsUnchecked {}

fn short(id: &str) -> &str {
    id.get(..12).unwrap_or(id)
}

#[derive(Deserialize)]
struct Pack {
    #[serde(default)]
    tcp_frames: BTreeMap<String, String>,
    #[serde(default)]
    tool_points_checked: Option<Record>,
}

#[derive(Deserialize)]
struct Record {
    embodiment: String,
    points: BTreeMap<String, String>,
}

/// The tool points `pack_json` records as checked for the robot opened as `embodiment`.
///
/// # Errors
///
/// [`ToolPointsUnchecked`] when the pack records no check, the check was of another revision,
/// its points are not the pack's `tcp_frames`, or the pack does not read.
pub fn checked_tool_points(
    pack_json: &str,
    embodiment: &str,
) -> Result<CheckedToolPoints, ToolPointsUnchecked> {
    let pack: Pack = serde_json::from_str(pack_json)
        .map_err(|error| ToolPointsUnchecked::Malformed(error.to_string()))?;
    let record = pack
        .tool_points_checked
        .ok_or(ToolPointsUnchecked::NoRecord)?;
    if record.embodiment != embodiment {
        return Err(ToolPointsUnchecked::OtherRevision {
            checked: record.embodiment,
            opened: embodiment.to_owned(),
        });
    }
    if record.points.is_empty() || record.points != pack.tcp_frames {
        return Err(ToolPointsUnchecked::PointsMoved);
    }
    Ok(CheckedToolPoints {
        embodiment: record.embodiment,
        points: record.points,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const STATIC_ID: &str = "51db79c80289f76c9f20f2043c114836dfcb80baca88e8f5094762c925a37402";

    #[test]
    fn the_static_upper_body_is_checked_for_its_revision() {
        let checked = checked_tool_points(crate::STATIC_UPPER_BODY_PACK, STATIC_ID)
            .expect("the static upper body carries a check");
        assert!(checked.drives("left_gripper_tcp") && checked.drives("right_gripper_tcp"));
        assert!(!checked.drives("ZED_camera"));
    }

    #[test]
    fn another_revision_or_no_record_or_moved_points_drive_nothing() {
        let other = checked_tool_points(crate::STATIC_UPPER_BODY_PACK, "b601");
        assert!(matches!(
            other,
            Err(ToolPointsUnchecked::OtherRevision { .. })
        ));
        let unchecked = r#"{"tcp_candidates": {"left": "left_gripper_link"}, "tcp_frames": {}}"#;
        assert_eq!(
            checked_tool_points(unchecked, STATIC_ID),
            Err(ToolPointsUnchecked::NoRecord)
        );
        let moved = format!(
            r#"{{"tcp_frames": {{"left": "left_flange"}},
                "tool_points_checked": {{"embodiment": "{STATIC_ID}",
                                         "points": {{"left": "left_gripper_tcp"}}}}}}"#
        );
        assert_eq!(
            checked_tool_points(&moved, STATIC_ID),
            Err(ToolPointsUnchecked::PointsMoved)
        );
        assert!(matches!(
            checked_tool_points("not json", STATIC_ID),
            Err(ToolPointsUnchecked::Malformed(_))
        ));
        let words = ToolPointsUnchecked::NoRecord.to_string();
        assert!(
            words.starts_with("tool point not checked on this robot yet"),
            "{words}"
        );
    }
}
