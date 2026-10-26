//! Collision proxies (`collision.json`, `sx.collision-proxies/v3`) and the distances between
//! them and the scene.

use serde::Deserialize;

use crate::IkError;
use crate::math::{V3, add, dot, mat_vec, norm, scale, sub};
use crate::model::Model;

pub const PROXIES_SCHEMA: &str = "sx.collision-proxies/v3";

/// How far a declared support reaches from its frame, m: past any limb's reach.
const SUPPORT_REACH_M: f64 = 3.0;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Document {
    schema: String,
    #[allow(dead_code)]
    embodiment_id: String,
    #[allow(dead_code)]
    urdf_sha256: String,
    capsules: Vec<CapsuleDocument>,
    ignored_pairs: Ignored,
    #[allow(dead_code)]
    adjacent_joints: u32,
    #[allow(dead_code)]
    generated_by: String,
    support: Option<SupportDocument>,
}

/// The surface the body stands on: the `z = 0` plane of `frame`, solid below up to `edge`.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SupportDocument {
    frame: String,
    #[allow(dead_code)]
    kind: String,
    edge: Option<EdgeDocument>,
}

/// Where the support ends: the line where the `frame` coordinate along `out` is `at_m`;
/// nothing of the support lies beyond it.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct EdgeDocument {
    out: Direction,
    at_m: f64,
}

/// An axis direction in the support frame's xy plane.
#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Direction {
    PosX,
    NegX,
    PosY,
    NegY,
}

impl Direction {
    const fn unit(self) -> V3 {
        match self {
            Self::PosX => [1.0, 0.0, 0.0],
            Self::NegX => [-1.0, 0.0, 0.0],
            Self::PosY => [0.0, 1.0, 0.0],
            Self::NegY => [0.0, -1.0, 0.0],
        }
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CapsuleDocument {
    link: String,
    a: V3,
    b: V3,
    radius: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Ignored {
    adjacent: Vec<[String; 2]>,
    touching_at_zero: Vec<[String; 2]>,
}

/// A segment `a`-`b` swept by a ball of `radius`, in its link's frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Capsule {
    pub link: usize,
    pub a: V3,
    pub b: V3,
    pub radius: f64,
}

/// An axis-aligned box in the root frame: a table, a wall, a machine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Obstacle {
    pub min: V3,
    pub max: V3,
}

/// The body's proxies and the capsule pairs a self-collision check compares.
#[derive(Clone, Debug)]
pub struct Proxies {
    pub capsules: Vec<Capsule>,
    /// Capsule index pairs on links that are neither ignored nor rigidly joined.
    pub pairs: Vec<(usize, usize)>,
    /// The surface the body stands on, as its embodiment declares it (its base mount): the
    /// mount frame's `z = 0` plane, solid below up to the support's declared edge, or
    /// everywhere within reach when it declares none. A link below the top and behind the
    /// edge is in collision; one in front of a clamp-edge table's edge may hang below its top.
    pub support: Option<Obstacle>,
}

impl Proxies {
    /// Read `collision.json` against the tree it was fitted to.
    ///
    /// # Errors
    ///
    /// [`IkError::Proxies`] when the document is not `sx.collision-proxies/v3`, names a link
    /// the tree lacks, or declares a support on a frame that moves or is not level with the
    /// root.
    pub fn parse(json: &str, model: &Model) -> Result<Self, IkError> {
        let document: Document =
            serde_json::from_str(json).map_err(|e| IkError::Proxies(e.to_string()))?;
        if document.schema != PROXIES_SCHEMA {
            return Err(IkError::Proxies(format!(
                "schema {:?} is not {PROXIES_SCHEMA}",
                document.schema
            )));
        }
        let link = |name: &str| {
            model
                .link(name)
                .ok_or_else(|| IkError::Proxies(format!("{name:?} is not a link of the tree")))
        };
        let capsules = document
            .capsules
            .iter()
            .map(|c| {
                Ok(Capsule {
                    link: link(&c.link)?,
                    a: c.a,
                    b: c.b,
                    radius: c.radius,
                })
            })
            .collect::<Result<Vec<_>, IkError>>()?;
        let mut ignored = Vec::new();
        for [first, second] in document
            .ignored_pairs
            .adjacent
            .iter()
            .chain(&document.ignored_pairs.touching_at_zero)
        {
            ignored.push((link(first)?, link(second)?));
        }
        let mut pairs = Vec::new();
        for (i, p) in capsules.iter().enumerate() {
            for (j, q) in capsules.iter().enumerate().skip(i + 1) {
                let skip = p.link == q.link
                    || ignored.contains(&(p.link, q.link))
                    || ignored.contains(&(q.link, p.link))
                    || rigid(model, p.link, q.link);
                if !skip {
                    pairs.push((i, j));
                }
            }
        }
        let support = match &document.support {
            None => None,
            Some(declared) => Some(support(
                model,
                link(&declared.frame)?,
                &declared.frame,
                declared.edge.as_ref(),
            )?),
        };
        Ok(Self {
            capsules,
            pairs,
            support,
        })
    }
}

/// The support under `frame` as a box in the root frame: the frame must be fixed to the root
/// and level with it, so its `z = 0` plane is a root-frame height. Without an edge the box
/// reaches past any limb on every side; an edge cuts it at the edge line, which must run
/// along a root axis so the box stays axis-aligned.
fn support(
    model: &Model,
    frame: usize,
    name: &str,
    edge: Option<&EdgeDocument>,
) -> Result<Obstacle, IkError> {
    if !model.above(frame).is_empty() {
        return Err(IkError::Proxies(format!(
            "support frame {name:?} moves with a joint"
        )));
    }
    let zero = vec![0.0; model.axes().len()];
    let pose = model.poses(&zero).links[frame];
    if (pose.r[2][2] - 1.0).abs() > 1e-9 {
        return Err(IkError::Proxies(format!(
            "support frame {name:?} is not level with the root"
        )));
    }
    let [x, y, z] = pose.t;
    let reach = SUPPORT_REACH_M;
    let mut obstacle = Obstacle {
        min: [x - reach, y - reach, z - reach],
        max: [x + reach, y + reach, z],
    };
    if let Some(edge) = edge {
        if !edge.at_m.is_finite() {
            return Err(IkError::Proxies(format!(
                "support frame {name:?} edge at_m is not finite"
            )));
        }
        let out = mat_vec(&pose.r, edge.out.unit());
        let Some(axis) = (0..2).find(|&k| (out[k].abs() - 1.0).abs() < 1e-9) else {
            return Err(IkError::Proxies(format!(
                "support frame {name:?} edge does not run along a root axis"
            )));
        };
        let line = pose.apply(scale(edge.out.unit(), edge.at_m))[axis];
        if out[axis] > 0.0 {
            obstacle.max[axis] = line;
        } else {
            obstacle.min[axis] = line;
        }
    }
    Ok(obstacle)
}

/// Whether no moving joint lies between two links: nothing can change their distance.
fn rigid(model: &Model, a: usize, b: usize) -> bool {
    let (above_a, above_b) = (model.above(a), model.above(b));
    above_a.len() == above_b.len() && above_a.iter().zip(above_b).all(|(x, y)| x.joint == y.joint)
}

/// The closest points of segments `p0`-`p1` and `q0`-`q1` (Ericson, *Real-Time Collision
/// Detection* 5.1.9).
#[must_use]
pub fn closest_segments(p0: V3, p1: V3, q0: V3, q1: V3) -> (V3, V3) {
    let (d1, d2, r) = (sub(p1, p0), sub(q1, q0), sub(p0, q0));
    let (a, e, f) = (dot(d1, d1), dot(d2, d2), dot(d2, r));
    let tiny = 1e-18;
    let (s, t) = if a <= tiny && e <= tiny {
        (0.0, 0.0)
    } else if a <= tiny {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = dot(d1, r);
        if e <= tiny {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = dot(d1, d2);
            let denominator = a.mul_add(e, -(b * b));
            let s = if denominator > tiny {
                (b.mul_add(f, -(c * e)) / denominator).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let t = b.mul_add(s, f) / e;
            if t < 0.0 {
                ((-c / a).clamp(0.0, 1.0), 0.0)
            } else if t > 1.0 {
                (((b - c) / a).clamp(0.0, 1.0), 1.0)
            } else {
                (s, t)
            }
        }
    };
    (add(p0, scale(d1, s)), add(q0, scale(d2, t)))
}

/// The signed distance from `p` to `obstacle` (negative inside) and the outward direction.
#[must_use]
pub fn box_distance(obstacle: &Obstacle, p: V3) -> (f64, V3) {
    let clamped = [
        p[0].clamp(obstacle.min[0], obstacle.max[0]),
        p[1].clamp(obstacle.min[1], obstacle.max[1]),
        p[2].clamp(obstacle.min[2], obstacle.max[2]),
    ];
    let outside = sub(p, clamped);
    let distance = norm(outside);
    if distance > 1e-12 {
        return (distance, scale(outside, 1.0 / distance));
    }
    // Inside: out through the nearest face.
    let mut best = (f64::INFINITY, [0.0, 0.0, 1.0]);
    for k in 0..3 {
        for (depth, sign) in [
            (obstacle.max[k] - p[k], 1.0),
            (p[k] - obstacle.min[k], -1.0),
        ] {
            if depth < best.0 {
                let mut n = [0.0; 3];
                n[k] = sign;
                best = (depth, n);
            }
        }
    }
    (-best.0, best.1)
}

/// The point of segment `a`-`b` nearest `obstacle` (signed distance is convex along a
/// segment, so a fixed golden-section search finds it), its signed distance and direction.
#[must_use]
pub fn segment_box(obstacle: &Obstacle, a: V3, b: V3) -> (V3, f64, V3) {
    const STEPS: usize = 40;
    let ratio = (5.0_f64.sqrt() - 1.0) / 2.0;
    let at = |s: f64| add(a, scale(sub(b, a), s));
    let f = |s: f64| box_distance(obstacle, at(s)).0;
    let (mut low, mut high) = (0.0_f64, 1.0_f64);
    let mut x1 = high - ratio * (high - low);
    let mut x2 = low + ratio * (high - low);
    let (mut f1, mut f2) = (f(x1), f(x2));
    for _ in 0..STEPS {
        if f1 <= f2 {
            high = x2;
            x2 = x1;
            f2 = f1;
            x1 = high - ratio * (high - low);
            f1 = f(x1);
        } else {
            low = x1;
            x1 = x2;
            f1 = f2;
            x2 = low + ratio * (high - low);
            f2 = f(x2);
        }
    }
    let candidates = [0.0, 1.0, f64::midpoint(low, high)];
    let s = candidates
        .into_iter()
        .min_by(|&x, &y| f(x).total_cmp(&f(y)))
        .unwrap_or(0.0);
    let point = at(s);
    let (distance, normal) = box_distance(obstacle, point);
    (point, distance, normal)
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // Exact comparisons of values the support copies through.
mod tests {
    use super::*;
    use sx_embodiment_drivers::kinematics::Tree;

    /// A root `world` with the mount frame fixed `yaw` about z and lifted 0.8 m.
    fn model(yaw: f64) -> Model {
        let urdf = format!(
            r#"<robot name="r"><link name="world"/><link name="base"/>
            <joint name="mount" type="fixed"><parent link="world"/><child link="base"/>
            <origin xyz="0.1 0.2 0.8" rpy="0 0 {yaw}"/></joint></robot>"#
        );
        Model::new(&Tree::parse(&urdf).expect("tree"))
    }

    fn proxies(model: &Model, edge: &str) -> Result<Proxies, IkError> {
        let json = format!(
            r#"{{"schema":"{PROXIES_SCHEMA}","embodiment_id":"r","urdf_sha256":"0",
            "capsules":[],"ignored_pairs":{{"adjacent":[],"touching_at_zero":[]}},
            "adjacent_joints":2,"generated_by":"test",
            "support":{{"frame":"base","kind":"clamp_edge","edge":{edge}}}}}"#
        );
        Proxies::parse(&json, model)
    }

    fn support(yaw: f64, edge: &str) -> Obstacle {
        proxies(&model(yaw), edge)
            .expect("proxies")
            .support
            .expect("support")
    }

    #[test]
    fn a_support_without_an_edge_is_the_whole_slab() {
        let slab = support(0.0, "null");
        assert_eq!(
            slab.max,
            [0.1 + SUPPORT_REACH_M, 0.2 + SUPPORT_REACH_M, 0.8]
        );
        assert_eq!(
            slab.min,
            [
                0.1 - SUPPORT_REACH_M,
                0.2 - SUPPORT_REACH_M,
                0.8 - SUPPORT_REACH_M
            ]
        );
    }

    #[test]
    fn a_clamp_edge_support_ends_at_its_edge_and_lies_behind_it() {
        let table = support(0.0, r#"{"out":"pos_x","at_m":0.0549}"#);
        assert!((table.max[0] - (0.1 + 0.0549)).abs() < 1e-12, "{table:?}");
        assert_eq!(table.min[0], 0.1 - SUPPORT_REACH_M);
        assert_eq!(table.max[2], 0.8);
        // In front of the edge, below the top, is free; behind it is the table.
        let below = |x: f64| box_distance(&table, [x, 0.2, 0.6]).0;
        assert!(below(0.1 + 0.0549 + 0.05) > 0.0);
        assert!(below(0.1) < 0.0);
        let back = support(0.0, r#"{"out":"neg_x","at_m":0.03}"#);
        assert!((back.min[0] - (0.1 - 0.03)).abs() < 1e-12, "{back:?}");
        assert_eq!(back.max[0], 0.1 + SUPPORT_REACH_M);
    }

    #[test]
    fn the_edge_turns_with_its_frame() {
        // The mount frame yawed a quarter turn: its +x is the root's +y.
        let table = support(
            std::f64::consts::FRAC_PI_2,
            r#"{"out":"pos_x","at_m":0.05}"#,
        );
        assert!((table.max[1] - 0.25).abs() < 1e-9, "{table:?}");
        assert_eq!(table.max[0], 0.1 + SUPPORT_REACH_M);
        let side = support(std::f64::consts::PI, r#"{"out":"neg_y","at_m":0.05}"#);
        assert!((side.max[1] - 0.25).abs() < 1e-9, "{side:?}");
    }

    #[test]
    fn an_edge_off_a_root_axis_or_unknown_is_refused() {
        assert!(proxies(&model(0.3), r#"{"out":"pos_x","at_m":0.05}"#).is_err());
        assert!(proxies(&model(0.0), r#"{"out":"pos_z","at_m":0.05}"#).is_err());
        assert!(proxies(&model(0.0), r#"{"out":"pos_x"}"#).is_err());
    }
}
