//! Collision proxies (`collision.json`, `sx.collision-proxies/v1`) and the distances between
//! them and the scene.

use serde::Deserialize;

use crate::IkError;
use crate::math::{V3, add, dot, norm, scale, sub};
use crate::model::Model;

pub const PROXIES_SCHEMA: &str = "sx.collision-proxies/v1";

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
}

impl Proxies {
    /// Read `collision.json` against the tree it was fitted to.
    ///
    /// # Errors
    ///
    /// [`IkError::Proxies`] when the document is not `sx.collision-proxies/v1` or names a link
    /// the tree lacks.
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
        Ok(Self { capsules, pairs })
    }
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
