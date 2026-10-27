//! Each link's collision geometry as convex pieces (`collision.json`'s `hulls`), and the
//! distance between two links' pieces by GJK: the second pass after the capsules. A capsule
//! contains its whole link, so two capsules overlap where the parts are a few millimetres
//! apart inside a bracket's corner (the static upper body's folded neck rests 1.5 mm clear);
//! the pieces follow the part's concavities, so only a contact they cannot rule out stands.

use crate::math::{Iso, V3, add, dot, mat_vec, norm, scale, sub, transpose};

/// One convex piece of a link, in the link's frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Piece {
    pub link: usize,
    pub points: Vec<V3>,
    /// A ball containing every point: pieces too far apart are never compared.
    pub centre: V3,
    pub radius: f64,
}

impl Piece {
    #[must_use]
    pub fn new(link: usize, points: Vec<V3>) -> Self {
        let count = points.len().max(1);
        #[allow(clippy::cast_precision_loss)]
        let centre = scale(
            points.iter().fold([0.0; 3], |sum, p| add(sum, *p)),
            1.0 / count as f64,
        );
        let radius = points
            .iter()
            .map(|p| norm(sub(*p, centre)))
            .fold(0.0, f64::max);
        Self {
            link,
            points,
            centre,
            radius,
        }
    }

    /// The piece's farthest point along `direction` (root frame), placed by `pose`.
    fn support(&self, pose: &Iso, direction: V3) -> V3 {
        let local = mat_vec(&transpose(&pose.r), direction);
        let mut best = self.points[0];
        let mut reach = dot(best, local);
        for &point in &self.points[1..] {
            let along = dot(point, local);
            if along > reach {
                reach = along;
                best = point;
            }
        }
        pose.apply(best)
    }
}

/// The closest approach of two pieces placed by their poses: the distance and a closest point
/// on each, or `None` when they touch or overlap (no separation to measure).
#[must_use]
pub fn piece_distance(a: &Piece, pose_a: &Iso, b: &Piece, pose_b: &Iso) -> Option<(f64, V3, V3)> {
    match gjk(a, pose_a, b, pose_b, f64::INFINITY) {
        Gjk::Apart(found) => Some(found),
        Gjk::Overlap | Gjk::Beyond => None,
    }
}

/// What GJK found between two pieces.
enum Gjk {
    /// They touch or overlap.
    Overlap,
    /// They are at least the limit apart (proved by a separating direction).
    Beyond,
    /// Their distance and a closest point on each.
    Apart((f64, V3, V3)),
}

/// GJK distance between two placed pieces, stopping as soon as a separating direction proves
/// them at least `limit` apart.
fn gjk(a: &Piece, pose_a: &Iso, b: &Piece, pose_b: &Iso, limit: f64) -> Gjk {
    // Simplex vertices of the difference a - b, each with the points it came from.
    let mut simplex: [(V3, V3, V3); 4] = [([0.0; 3], [0.0; 3], [0.0; 3]); 4];
    let mut size = 0;
    let first = sub(pose_a.apply(a.centre), pose_b.apply(b.centre));
    let start = if norm(first) > 1e-12 {
        first
    } else {
        [1.0, 0.0, 0.0]
    };
    let (pa, pb) = (
        a.support(pose_a, scale(start, -1.0)),
        b.support(pose_b, start),
    );
    let mut v = sub(pa, pb);
    let mut weights = [1.0, 0.0, 0.0, 0.0];
    simplex[0] = (v, pa, pb);
    size += 1;
    for _ in 0..64 {
        let length = dot(v, v);
        if length < 1e-18 {
            return Gjk::Overlap;
        }
        let (pa, pb) = (a.support(pose_a, scale(v, -1.0)), b.support(pose_b, v));
        let w = sub(pa, pb);
        // Every point of a - b lies at least v.w / |v| along v: that far apart at least.
        if dot(v, w) >= limit * length.sqrt() {
            return Gjk::Beyond;
        }
        // No support point gets closer to the origin than v by more than rounding: v is it.
        if length - dot(v, w) <= 1e-10 * length.max(1e-12) {
            break;
        }
        if simplex[..size]
            .iter()
            .any(|(s, _, _)| norm(sub(*s, w)) < 1e-14)
        {
            break;
        }
        simplex[size] = (w, pa, pb);
        size += 1;
        let Some((closest, kept, lambda)) = closest_on_simplex(&simplex[..size]) else {
            return Gjk::Overlap;
        };
        // Keep only the vertices the closest point needs.
        let mut reduced = [([0.0; 3], [0.0; 3], [0.0; 3]); 4];
        let mut count = 0;
        for index in 0..size {
            if kept & (1 << index) != 0 {
                reduced[count] = simplex[index];
                weights[count] = lambda[index];
                count += 1;
            }
        }
        simplex = reduced;
        size = count;
        v = closest;
        if size == 4 {
            return Gjk::Overlap; // the origin is inside
        }
    }
    let distance = norm(v);
    if distance < 1e-9 {
        return Gjk::Overlap;
    }
    let mut on_a = [0.0; 3];
    let mut on_b = [0.0; 3];
    for (index, (_, pa, pb)) in simplex[..size].iter().enumerate() {
        on_a = add(on_a, scale(*pa, weights[index]));
        on_b = add(on_b, scale(*pb, weights[index]));
    }
    Gjk::Apart((distance, on_a, on_b))
}

/// The point of the simplex closest to the origin, which vertices it is a combination of (a
/// bit mask) and their weights; every face of up to four vertices is tried. No allocation:
/// it runs inside a control tick.
fn closest_on_simplex(simplex: &[(V3, V3, V3)]) -> Option<(V3, u8, [f64; 4])> {
    let n = simplex.len();
    let mut best: Option<(f64, V3, u8, [f64; 4])> = None;
    for mask in 1_u8..(1 << n) {
        let mut indices = [0_usize; 4];
        let mut count = 0;
        for i in 0..n {
            if mask & (1 << i) != 0 {
                indices[count] = i;
                count += 1;
            }
        }
        let Some(lambda) = affine_weights(simplex, &indices[..count]) else {
            continue;
        };
        if lambda[..count].iter().any(|&l| l < -1e-12) {
            continue;
        }
        let mut point = [0.0; 3];
        let mut weights = [0.0; 4];
        for (k, &index) in indices[..count].iter().enumerate() {
            point = add(point, scale(simplex[index].0, lambda[k]));
            weights[index] = lambda[k];
        }
        let length = dot(point, point);
        if best.is_none_or(|(b, ..)| length < b - 1e-18) {
            best = Some((length, point, mask, weights));
        }
    }
    best.map(|(_, point, mask, weights)| (point, mask, weights))
}

/// The weights (summing to one) of the point of the affine hull of `indices`' vertices
/// closest to the origin, or `None` for a degenerate face.
fn affine_weights(simplex: &[(V3, V3, V3)], indices: &[usize]) -> Option<[f64; 4]> {
    let origin = simplex[indices[0]].0;
    let k = indices.len() - 1;
    let mut edges = [[0.0; 3]; 3];
    for (slot, &i) in indices[1..].iter().enumerate() {
        edges[slot] = sub(simplex[i].0, origin);
    }
    let mut lambda = [0.0; 4];
    if k == 0 {
        lambda[0] = 1.0;
        return Some(lambda);
    }
    // (E^T E) mu = -E^T origin
    let mut m = [[0.0; 3]; 3];
    let mut rhs = [0.0; 3];
    for r in 0..k {
        for c in 0..k {
            m[r][c] = dot(edges[r], edges[c]);
        }
        rhs[r] = -dot(edges[r], origin);
    }
    let mu = solve(&m, &rhs, k)?;
    lambda[0] = 1.0 - mu[..k].iter().sum::<f64>();
    lambda[1..=k].copy_from_slice(&mu[..k]);
    Some(lambda)
}

/// Solve the leading `k` by `k` system by Gaussian elimination with pivoting.
fn solve(m: &[[f64; 3]; 3], rhs: &[f64; 3], k: usize) -> Option<[f64; 3]> {
    let mut a = *m;
    let mut b = *rhs;
    let scale_of = (0..k)
        .map(|i| a[i][i].abs())
        .fold(0.0, f64::max)
        .max(1e-300);
    for col in 0..k {
        let pivot = (col..k).max_by(|&x, &y| a[x][col].abs().total_cmp(&a[y][col].abs()))?;
        if a[pivot][col].abs() < 1e-12 * scale_of {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..k {
            let f = a[row][col] / a[col][col];
            for c in col..k {
                a[row][c] -= f * a[col][c];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = [0.0; 3];
    for row in (0..k).rev() {
        let mut sum = b[row];
        for c in row + 1..k {
            sum -= a[row][c] * x[c];
        }
        x[row] = sum / a[row][row];
    }
    Some(x)
}

/// The closest approach of two links' pieces, placed by their poses: distance and a closest
/// point on each, or `None` when a pair of pieces touches or a link has no pieces.
#[must_use]
pub fn link_distance(
    pieces: &[Piece],
    of_a: &[usize],
    pose_a: &Iso,
    of_b: &[usize],
    pose_b: &Iso,
) -> Option<(f64, V3, V3)> {
    if of_a.is_empty() || of_b.is_empty() {
        return None;
    }
    let apart = |i: usize, j: usize| {
        let (a, b) = (&pieces[i], &pieces[j]);
        norm(sub(pose_a.apply(a.centre), pose_b.apply(b.centre))) - a.radius - b.radius
    };
    // First the pair whose bounding balls come closest, for a bound that rules most out.
    let mut nearest = (f64::INFINITY, of_a[0], of_b[0]);
    for &i in of_a {
        for &j in of_b {
            let bound = apart(i, j);
            if bound < nearest.0 {
                nearest = (bound, i, j);
            }
        }
    }
    let mut best = piece_distance(&pieces[nearest.1], pose_a, &pieces[nearest.2], pose_b)?;
    for &i in of_a {
        for &j in of_b {
            if (i, j) == (nearest.1, nearest.2) || apart(i, j) >= best.0 {
                continue;
            }
            match gjk(&pieces[i], pose_a, &pieces[j], pose_b, best.0) {
                Gjk::Overlap => return None,
                Gjk::Beyond => {}
                Gjk::Apart(found) => {
                    if found.0 < best.0 {
                        best = found;
                    }
                }
            }
        }
    }
    Some(best)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube(link: usize, centre: V3, half: f64) -> Piece {
        let mut points = Vec::new();
        for x in [-half, half] {
            for y in [-half, half] {
                for z in [-half, half] {
                    points.push(add(centre, [x, y, z]));
                }
            }
        }
        Piece::new(link, points)
    }

    #[test]
    fn two_cubes_are_as_far_apart_as_their_faces() {
        let a = cube(0, [0.0; 3], 0.5);
        let b = cube(1, [2.0, 0.3, 0.0], 0.5);
        let (d, on_a, on_b) =
            piece_distance(&a, &Iso::IDENTITY, &b, &Iso::IDENTITY).expect("apart");
        assert!((d - 1.0).abs() < 1e-9, "{d}");
        assert!((on_a[0] - 0.5).abs() < 1e-9 && (on_b[0] - 1.5).abs() < 1e-9);
        let overlapping = cube(1, [0.9, 0.0, 0.0], 0.5);
        assert!(piece_distance(&a, &Iso::IDENTITY, &overlapping, &Iso::IDENTITY).is_none());
    }

    #[test]
    fn an_edge_to_edge_gap_is_found_in_any_pose() {
        let a = cube(0, [0.0; 3], 0.5);
        let b = cube(1, [0.0; 3], 0.5);
        // b turned 45 deg about z and moved along x: its edge faces a's face.
        let turn = crate::math::about([0.0, 0.0, 1.0], std::f64::consts::FRAC_PI_4);
        let pose_b = Iso {
            r: turn,
            t: [0.5 + 0.5 * 2.0_f64.sqrt() + 0.25, 0.0, 0.0],
        };
        let (d, ..) = piece_distance(&a, &Iso::IDENTITY, &b, &pose_b).expect("apart");
        assert!((d - 0.25).abs() < 1e-9, "{d}");
        let pieces = vec![a.clone(), cube(0, [5.0, 5.0, 5.0], 0.1), b];
        let (d, ..) =
            link_distance(&pieces, &[0, 1], &Iso::IDENTITY, &[2], &pose_b).expect("apart");
        assert!((d - 0.25).abs() < 1e-9, "{d}");
    }
}
