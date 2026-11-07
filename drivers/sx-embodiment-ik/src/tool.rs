//! A body's tools and how each grips, as its render pack's `pack.json` carries them
//! (`sx_embodiments.tool_frames`): the frame a tool is stated in, and its jaws' closing line,
//! approach and grasp point, or the reason it has none. The registry derives them from the
//! description by one rule for every body; a consumer reads them here, never re-measures them.

use serde::Deserialize;

use crate::IkError;
use sx_frames::linalg::{dot, mat_vec, norm, scale, sub, transpose};

/// How a gripper's jaws meet.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JawKind {
    /// Two jaws move toward each other.
    Parallel,
    /// One jaw moves against a fixed one.
    SingleMoving,
}

/// Why a tool has no closing line.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NoJawsReason {
    /// A hand of more than two fingers: no one line.
    MultiFingerHand,
    /// An arm the registry mounts no gripper on.
    BareFlange,
    /// A device on an arm whose jaw no source describes.
    DeviceUndescribed,
    /// The description draws no geometry on the moving jaws.
    JawsUndescribed,
    /// The gripper's joints move none of its jaws.
    NoJawMotion,
}

/// A tool whose jaws close on one line; every vector in [`ToolFrame::frame`].
#[derive(Clone, Copy, Debug, PartialEq, Deserialize)]
pub struct Jaws {
    #[serde(rename = "jaws")]
    pub kind: JawKind,
    /// Unit, normal to `approach_axis`: parallel jaws' signed so its largest component is
    /// positive, a single moving jaw's from the fixed jaw to the moving one.
    pub closing_axis: [f64; 3],
    /// Unit: from the gripper's mount toward the grasp point.
    pub approach_axis: [f64; 3],
    pub grasp_point_m: [f64; 3],
}

impl Jaws {
    /// The closing line made normal to `approach` (a solve's own approach for this tool), its
    /// sign kept. `None` when `approach` is no direction or runs along the line.
    #[must_use]
    pub fn closing_normal_to(&self, approach: [f64; 3]) -> Option<[f64; 3]> {
        let length = norm(approach);
        if length < 1e-9 {
            return None;
        }
        let approach = scale(approach, 1.0 / length);
        let normal = sub(
            self.closing_axis,
            scale(approach, dot(self.closing_axis, approach)),
        );
        let length = norm(normal);
        (length > 1e-6).then(|| scale(normal, 1.0 / length))
    }
}

/// A tool's grip: jaws, or the stated reason it has none.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(untagged)]
pub enum Grip {
    Jaws(Jaws),
    None {
        #[serde(rename = "none")]
        reason: NoJawsReason,
        detail: String,
    },
}

/// One tool of a body: its component, the description frame its grip is stated in, and the
/// grip.
#[derive(Clone, Debug, PartialEq, Deserialize)]
pub struct ToolFrame {
    pub component: String,
    pub frame: String,
    pub grip: Grip,
    /// The side of the body the tool is on (`left`, `right`), where the body places one
    /// (`sx_embodiments.frames.sides`): the hand an operator drives it with.
    #[serde(default)]
    pub side: Option<String>,
}

#[derive(Deserialize)]
struct Pack {
    tools: Vec<ToolFrame>,
}

/// The tools a render pack's `pack.json` carries.
///
/// # Errors
///
/// [`IkError::Tools`] for a document with no `tools` or one this crate cannot read.
pub fn parse_tools(pack_json: &str) -> Result<Vec<ToolFrame>, IkError> {
    serde_json::from_str::<Pack>(pack_json)
        .map(|pack| pack.tools)
        .map_err(|e| IkError::Tools(e.to_string()))
}

impl ToolFrame {
    /// This tool stated in `frame` instead, given both frames' rotations and positions in a
    /// common frame (`r_*`, `t_*`): the two must ride rigidly together.
    #[must_use]
    pub(crate) fn restated(
        &self,
        frame: &str,
        (r_own, t_own): (&[[f64; 3]; 3], [f64; 3]),
        (r_new, t_new): (&[[f64; 3]; 3], [f64; 3]),
    ) -> Self {
        let into = transpose(r_new);
        let turn = |v: [f64; 3]| mat_vec(&into, mat_vec(r_own, v));
        let grip = match &self.grip {
            Grip::Jaws(jaws) => Grip::Jaws(Jaws {
                kind: jaws.kind,
                closing_axis: turn(jaws.closing_axis),
                approach_axis: turn(jaws.approach_axis),
                grasp_point_m: mat_vec(
                    &into,
                    sub(
                        sx_frames::linalg::add(mat_vec(r_own, jaws.grasp_point_m), t_own),
                        t_new,
                    ),
                ),
            }),
            none @ Grip::None { .. } => none.clone(),
        };
        Self {
            component: self.component.clone(),
            frame: frame.to_owned(),
            grip,
            side: self.side.clone(),
        }
    }
}
