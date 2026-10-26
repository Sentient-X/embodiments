//! Where a robot stands in an operator's room: "I am the robot".
//!
//! A teleoperation headset draws the robot in the operator's own room, and the operator's body
//! is the robot's: the operator stands inside it, eyes at its camera, hands at its grippers.
//! The placement is one gravity-aligned rigid transform `W` from the robot's base frame to the
//! room (a yaw about the vertical, then a translation; never a tilt, never a scale), solved by
//! weighted least squares from correspondences between robot sites and operator sites:
//!
//! - **points**: a robot site in the base frame (the camera, each tool frame) and where the
//!   operator holds it in the room (the eyes, each hand-held tool tip), with a weight;
//! - **headings**: a robot site's forward direction and the operator's (gaze, pointing), both
//!   on the floor plane, with a weight.
//!
//! The yaw is the weighted 2-D Procrustes rotation of the centred floor points and the headings;
//! the horizontal translation carries the weighted robot centroid onto the operator centroid.
//! The height is the robot's own when its mount declares it (a body clamped to a table stands
//! at the table's height, whatever the operator's), else the weighted mean height difference.
//! Both frames are Z-up and right-handed; the residual is reported, never absorbed.

/// A robot site and where the operator holds it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointPair {
    /// In the robot's base frame, metres.
    pub robot: [f64; 3],
    /// In the room, metres.
    pub room: [f64; 3],
    pub weight: f64,
}

/// A robot site's forward direction and the operator's, each projected on the floor (x, y);
/// they need not be unit length (a short projection weighs less).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeadingPair {
    pub robot: [f64; 2],
    pub room: [f64; 2],
    pub weight: f64,
}

/// `room = Rz(yaw) · base + translation`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placement {
    pub yaw: f64,
    pub translation: [f64; 3],
}

impl Placement {
    /// A base-frame point in the room.
    #[must_use]
    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let (s, c) = self.yaw.sin_cos();
        [
            c * p[0] - s * p[1] + self.translation[0],
            s * p[0] + c * p[1] + self.translation[1],
            p[2] + self.translation[2],
        ]
    }
}

/// The weighted least-squares gravity-aligned placement; `base_height` fixes the base frame's
/// height in the room (`translation[2]`) when the body's mount declares it. `None` when no
/// pair carries weight.
#[must_use]
pub fn place(
    points: &[PointPair],
    headings: &[HeadingPair],
    base_height: Option<f64>,
) -> Option<Placement> {
    let total: f64 = points.iter().map(|p| p.weight.max(0.0)).sum();
    if !(total > 0.0) {
        return None;
    }
    let mut cr = [0.0; 3];
    let mut co = [0.0; 3];
    for p in points {
        let w = p.weight.max(0.0) / total;
        for k in 0..3 {
            cr[k] += w * p.robot[k];
            co[k] += w * p.room[k];
        }
    }
    let (mut dot, mut cross) = (0.0, 0.0);
    for p in points {
        let w = p.weight.max(0.0);
        let a = [p.robot[0] - cr[0], p.robot[1] - cr[1]];
        let b = [p.room[0] - co[0], p.room[1] - co[1]];
        dot += w * (a[0] * b[0] + a[1] * b[1]);
        cross += w * (a[0] * b[1] - a[1] * b[0]);
    }
    for h in headings {
        let w = h.weight.max(0.0);
        dot += w * (h.robot[0] * h.room[0] + h.robot[1] * h.room[1]);
        cross += w * (h.robot[0] * h.room[1] - h.robot[1] * h.room[0]);
    }
    let yaw = if dot == 0.0 && cross == 0.0 {
        0.0
    } else {
        cross.atan2(dot)
    };
    let (s, c) = yaw.sin_cos();
    let rotated = [c * cr[0] - s * cr[1], s * cr[0] + c * cr[1]];
    let z = base_height.unwrap_or(co[2] - cr[2]);
    let placement = Placement {
        yaw,
        translation: [co[0] - rotated[0], co[1] - rotated[1], z],
    };
    placement
        .translation
        .iter()
        .all(|v| v.is_finite())
        .then_some(placement)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pair(robot: [f64; 3], w: &Placement, weight: f64) -> PointPair {
        PointPair {
            robot,
            room: w.apply(robot),
            weight,
        }
    }

    #[test]
    fn recovers_a_placement_from_head_and_hands() {
        let truth = Placement {
            yaw: 2.4,
            translation: [1.5, -0.7, 0.797],
        };
        let points = [
            pair([0.32, 0.0, 0.75], &truth, 4.0),
            pair([0.44, 0.21, 0.22], &truth, 1.0),
            pair([0.44, -0.21, 0.22], &truth, 1.0),
        ];
        let got = place(&points, &[], None).expect("placed");
        assert!((got.yaw - truth.yaw).abs() < 1e-12);
        for k in 0..3 {
            assert!((got.translation[k] - truth.translation[k]).abs() < 1e-12);
        }
    }

    #[test]
    fn a_declared_mount_height_wins_over_the_operator() {
        let truth = Placement {
            yaw: -0.3,
            translation: [0.2, 0.1, 0.9],
        };
        let points = [pair([0.32, 0.0, 0.75], &truth, 1.0)];
        let heading = HeadingPair {
            robot: [1.0, 0.0],
            room: [(-0.3f64).cos(), (-0.3f64).sin()],
            weight: 1.0,
        };
        let got = place(&points, &[heading], Some(0.797)).expect("placed");
        assert!((got.yaw + 0.3).abs() < 1e-12);
        assert!((got.translation[2] - 0.797).abs() < 1e-12);
        // Horizontally the camera still lands on the eyes.
        let eyes = got.apply([0.32, 0.0, 0.75]);
        let want = truth.apply([0.32, 0.0, 0.75]);
        assert!((eyes[0] - want[0]).abs() < 1e-12 && (eyes[1] - want[1]).abs() < 1e-12);
    }

    #[test]
    fn nothing_weighted_places_nothing() {
        assert_eq!(place(&[], &[], None), None);
    }
}
