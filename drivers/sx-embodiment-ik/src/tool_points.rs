//! Whether a body's tool points were checked on its hardware, as its render pack's `pack.json`
//! records it (`sx_embodiments.tool_points`, embodiments #242): nothing drives a tool point a
//! person has not checked on the real robot.
//!
//! A check names the link checked on each side and the digest of the kinematic chain from the
//! description's root to them (`sx_embodiments.tool_points.tool_chain`). It holds for those links
//! and that chain only: a pack whose `tcp_frames` are not the checked points, or a robot whose
//! assigned revision has another chain (`BodyBindings::tool_chain`), is unchecked; a revision
//! that changes only what is off the tool path keeps it. Absence is never a check.

use std::collections::BTreeMap;
use std::fmt;

use serde::Deserialize;

/// A body's tool points, checked on its hardware for one revision.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CheckedToolPoints {
    /// The chain digest the check was of.
    pub chain: String,
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
    /// The robot's revision has no chain to its tool points the registry renders (an id the
    /// body does not answer for, or one whose description lacks a checked link).
    NoChain,
    /// The check was of another chain than the robot's revision has: a tool point or a joint on
    /// its chain moved since.
    OtherChain { checked: String, opened: String },
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
            Self::NoChain => f.write_str("its revision's tool points have no checked chain"),
            Self::OtherChain { checked, opened } => write!(
                f,
                "its tool points' chain changed since the check (checked {}, this robot's {})",
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
    chain: String,
    points: BTreeMap<String, String>,
}

/// The tool points `pack_json` records as checked for a robot whose assigned revision has the
/// tool chain `chain` (`BodyBindings::tool_chain`; `None` when it has none).
///
/// # Errors
///
/// [`ToolPointsUnchecked`] when the pack records no check, the robot's revision has no chain or
/// another one, the check's points are not the pack's `tcp_frames`, or the pack does not read.
pub fn checked_tool_points(
    pack_json: &str,
    chain: Option<&str>,
) -> Result<CheckedToolPoints, ToolPointsUnchecked> {
    let pack: Pack = serde_json::from_str(pack_json)
        .map_err(|error| ToolPointsUnchecked::Malformed(error.to_string()))?;
    let record = pack
        .tool_points_checked
        .ok_or(ToolPointsUnchecked::NoRecord)?;
    let chain = chain.ok_or(ToolPointsUnchecked::NoChain)?;
    if record.chain != chain {
        return Err(ToolPointsUnchecked::OtherChain {
            checked: record.chain,
            opened: chain.to_owned(),
        });
    }
    if record.points.is_empty() || record.points != pack.tcp_frames {
        return Err(ToolPointsUnchecked::PointsMoved);
    }
    Ok(CheckedToolPoints {
        chain: record.chain,
        points: record.points,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn static_chain() -> String {
        let pack: serde_json::Value = serde_json::from_str(
            crate::registered("sentient-static-upper-body")
                .expect("committed")
                .pack,
        )
        .expect("pack");
        pack["tool_points_checked"]["chain"]
            .as_str()
            .expect("a chain")
            .to_owned()
    }

    #[test]
    fn the_static_upper_body_is_checked_for_its_chain() {
        let chain = static_chain();
        let checked = checked_tool_points(
            crate::registered("sentient-static-upper-body")
                .expect("committed")
                .pack,
            Some(&chain),
        )
        .expect("the static upper body carries a check");
        assert!(checked.drives("left_gripper_tcp") && checked.drives("right_gripper_tcp"));
        assert!(!checked.drives("ZED_camera"));
    }

    #[test]
    fn another_chain_or_none_or_no_record_or_moved_points_drive_nothing() {
        let chain = static_chain();
        assert!(matches!(
            checked_tool_points(
                crate::registered("sentient-static-upper-body")
                    .expect("committed")
                    .pack,
                Some(&"0".repeat(64))
            ),
            Err(ToolPointsUnchecked::OtherChain { .. })
        ));
        assert_eq!(
            checked_tool_points(
                crate::registered("sentient-static-upper-body")
                    .expect("committed")
                    .pack,
                None
            ),
            Err(ToolPointsUnchecked::NoChain)
        );
        let unchecked = r#"{"tcp_candidates": {"left": "left_gripper_link"}, "tcp_frames": {}}"#;
        assert_eq!(
            checked_tool_points(unchecked, Some(&chain)),
            Err(ToolPointsUnchecked::NoRecord)
        );
        let moved = format!(
            r#"{{"tcp_frames": {{"left": "left_flange"}},
                "tool_points_checked": {{"chain": "{chain}",
                                         "points": {{"left": "left_gripper_tcp"}}}}}}"#
        );
        assert_eq!(
            checked_tool_points(&moved, Some(&chain)),
            Err(ToolPointsUnchecked::PointsMoved)
        );
        assert!(matches!(
            checked_tool_points("not json", Some(&chain)),
            Err(ToolPointsUnchecked::Malformed(_))
        ));
        let words = ToolPointsUnchecked::NoRecord.to_string();
        assert!(
            words.starts_with("tool point not checked on this robot yet"),
            "{words}"
        );
    }
}
