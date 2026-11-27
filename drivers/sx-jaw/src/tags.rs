//! The jaw joint angle from the two finger markers (formerly `sx_embodiments.jaw_tags`).
//!
//! Each finger carries one 15 mm `DICT_4X4_50` marker that turns with the finger. Each finger
//! link pivots on its own hinge, and the fingers are a mimic pair, so the markers turn by equal
//! and opposite amounts (yubi-0006, 2026-11-25: 1.02 and 0.98 times the encoder angle over 43
//! degrees).
//!
//! 1. Pose: undistort a marker's corners through the lens, then IPPE-square with the printed
//!    side. The rotation is precise; the depth is not used.
//! 2. Reference: a marker's pose at a known joint angle and its hinge axis are fixed by where
//!    it is glued. [`fit_reference`] learns them from frames where the encoder worked;
//!    [`closed_stop_reference`] takes the narrowest pose seen as the joint's lower stop.
//! 3. Reading: each visible marker's rotation from its reference about its hinge is the joint
//!    angle; two must agree within 6 degrees (the mirror check), and they are averaged.

use crate::geometry::{
    self, Mat3, Vec3, apply, column, correlation, dot, log, median, norm, percentile,
    principal_axis, ptp, relative, scale, sub,
};
use crate::lens::Lens;

/// The printed side of a YUBI finger marker (the outer black square), metres.
pub const YUBI_MARKER_SIDE_M: f64 = 0.015;
/// A marker whose corners reproject worse than this is not a reliable pose.
pub const MAX_REPROJECTION_PX: f64 = 1.5;
/// A face-on marker's two IPPE tilts fit almost equally; the right one may fit a little worse.
pub const MAX_ALTERNATE_REPROJECTION_PX: f64 = 4.0;
/// Two markers whose joint angles differ by more than this disagree: a bad frame or a broken
/// gripper (a sound unit's spread is 2.76 degrees at p95).
pub const MAX_MIRROR_DISAGREEMENT_RAD: f64 = 6.0 * std::f64::consts::PI / 180.0;
/// A reference needs this many sightings of a marker over this much travel.
pub const MIN_REFERENCE_SIGHTINGS: usize = 60;
pub const MIN_REFERENCE_TRAVEL_RAD: f64 = 15.0 * std::f64::consts::PI / 180.0;
/// Frames to look at before deciding a recording has no jaw markers at all.
pub const NO_TAGS_PROBE_FRAMES: usize = 300;
const AXIS_ROUNDS: usize = 4;
const MAX_OFF_AXIS_RAD: f64 = 3.0 * std::f64::consts::PI / 180.0;

/// Which markers are on the two fingers, and how big they are printed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Layout {
    pub first_id: u32,
    pub second_id: u32,
    pub marker_side_m: f64,
}

impl Layout {
    pub fn new(first_id: u32, second_id: u32, marker_side_m: f64) -> Result<Self, String> {
        if first_id == second_id {
            return Err("the two finger markers need distinct IDs".to_owned());
        }
        if crate::dictionary::code(first_id).is_none()
            || crate::dictionary::code(second_id).is_none()
        {
            return Err("the finger markers are DICT_4X4_50 IDs 0 to 49".to_owned());
        }
        if !(marker_side_m.is_finite() && (0.002..=0.1).contains(&marker_side_m)) {
            return Err(format!(
                "marker side {marker_side_m} m is not a printed tag size"
            ));
        }
        Ok(Self {
            first_id,
            second_id,
            marker_side_m,
        })
    }

    /// YUBI's layout: the right wrist sees IDs 0 and 1, the left 2 and 3.
    pub fn yubi(side: &str) -> Result<Self, String> {
        match side {
            "right" => Self::new(0, 1, YUBI_MARKER_SIDE_M),
            "left" => Self::new(2, 3, YUBI_MARKER_SIDE_M),
            other => Err(format!("YUBI has no {other:?} hand")),
        }
    }

    pub fn ids(&self) -> [u32; 2] {
        [self.first_id, self.second_id]
    }

    /// The marker's corners in its own frame, in OpenCV's detection order.
    pub fn corners_m(&self) -> [Vec3; 4] {
        let h = self.marker_side_m / 2.0;
        [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]]
    }
}

/// One finger marker's pose in the camera frame.
#[derive(Clone, Debug, PartialEq)]
pub struct TagPose {
    pub marker_id: u32,
    pub rotation: Mat3,
    pub center_m: Vec3,
    pub reprojection_px: f64,
    /// IPPE's other solution (a mirrored tilt) when it reprojects within 4 px.
    pub alternate: Option<Mat3>,
}

impl TagPose {
    pub fn candidates(&self) -> Vec<Mat3> {
        match self.alternate {
            Some(alternate) => vec![self.rotation, alternate],
            None => vec![self.rotation],
        }
    }

    /// The rotation (of the two) whose face normal is closest to `normal`.
    pub fn facing(&self, normal: Vec3) -> Mat3 {
        match self.alternate {
            Some(alternate)
                if dot(column(&alternate, 2), normal) > dot(column(&self.rotation, 2), normal) =>
            {
                alternate
            }
            _ => self.rotation,
        }
    }
}

/// One frame's sighted markers: at most one pose per ID.
pub type Frame = Vec<TagPose>;

pub fn pose_of(frame: &Frame, id: u32) -> Option<&TagPose> {
    frame.iter().find(|pose| pose.marker_id == id)
}

/// One marker from its four detected corners (pixels, OpenCV order), or `None` if poor.
pub fn locate_tag(
    marker_id: u32,
    pixels: &[[f64; 2]; 4],
    lens: &Lens,
    layout: &Layout,
) -> Option<TagPose> {
    let mut ideal = [[0.0; 2]; 4];
    for (out, pixel) in ideal.iter_mut().zip(pixels) {
        *out = lens.normalized(*pixel)?;
    }
    let object = layout.corners_m();
    let mut solutions: Vec<(f64, Mat3, Vec3)> = Vec::new();
    for rotation in ippe_square(&ideal, layout.marker_side_m)? {
        let center = translation(&rotation, &object, &ideal)?;
        let mut sum = 0.0;
        let mut finite = true;
        for (corner, pixel) in object.iter().zip(pixels) {
            let p = apply(&rotation, *corner);
            match lens.project([p[0] + center[0], p[1] + center[1], p[2] + center[2]]) {
                Some(projected) => {
                    sum += (projected[0] - pixel[0]).powi(2) + (projected[1] - pixel[1]).powi(2)
                }
                None => finite = false,
            }
        }
        let error = (sum / 4.0).sqrt();
        if finite && error.is_finite() && center[2] > 0.0 {
            solutions.push((error, rotation, center));
        }
    }
    solutions.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (error, rotation, center) = *solutions.first()?;
    if error > MAX_REPROJECTION_PX {
        return None;
    }
    let alternate = solutions
        .get(1)
        .filter(|solution| solution.0 <= MAX_ALTERNATE_REPROJECTION_PX)
        .map(|solution| solution.1);
    Some(TagPose {
        marker_id,
        rotation,
        center_m: center,
        reprojection_px: error,
        alternate,
    })
}

/// The two rotations of a square of side `side` seen at `ideal` (unit-focal image points of
/// its corners in OpenCV's order): IPPE (Collins and Bartoli 2014) as OpenCV's
/// `SOLVEPNP_IPPE_SQUARE` computes them.
fn ippe_square(ideal: &[[f64; 2]; 4], side: f64) -> Option<[Mat3; 2]> {
    let h = side / 2.0;
    let object = [[-h, h], [h, h], [h, -h], [-h, -h]];
    let homography = homography(&object, ideal)?;
    let p = homography[0][2];
    let q = homography[1][2];
    let j00 = homography[0][0] - homography[2][0] * p;
    let j01 = homography[0][1] - homography[2][1] * p;
    let j10 = homography[1][0] - homography[2][0] * q;
    let j11 = homography[1][1] - homography[2][1] * q;
    ippe_rotations(j00, j01, j10, j11, p, q)
}

/// The plane-to-image homography through four correspondences, normalised to `H[2][2] = 1`.
fn homography(object: &[[f64; 2]; 4], image: &[[f64; 2]; 4]) -> Option<Mat3> {
    // Unknowns h00 h01 h02 h10 h11 h12 h20 h21.
    let mut a = [[0.0f64; 9]; 8];
    for i in 0..4 {
        let [x, y] = object[i];
        let [u, v] = image[i];
        a[2 * i] = [x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y, u];
        a[2 * i + 1] = [0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y, v];
    }
    for col in 0..8 {
        let pivot = (col..8).max_by(|r, s| a[*r][col].abs().total_cmp(&a[*s][col].abs()))?;
        if a[pivot][col].abs() < 1e-15 {
            return None;
        }
        a.swap(col, pivot);
        for row in 0..8 {
            if row != col {
                let f = a[row][col] / a[col][col];
                for k in col..9 {
                    a[row][k] -= f * a[col][k];
                }
            }
        }
    }
    let x: Vec<f64> = (0..8).map(|i| a[i][8] / a[i][i]).collect();
    Some([[x[0], x[1], x[2]], [x[3], x[4], x[5]], [x[6], x[7], 1.0]])
}

#[allow(clippy::many_single_char_names)]
fn ippe_rotations(j00: f64, j01: f64, j10: f64, j11: f64, p: f64, q: f64) -> Option<[Mat3; 2]> {
    let s = (p * p + q * q + 1.0).sqrt();
    let t = (p * p + q * q).sqrt();
    let costh = 1.0 / s;
    let sinth = (1.0 - 1.0 / (s * s)).sqrt();
    let rv: Mat3 = if t < f64::from(f32::EPSILON) {
        geometry::IDENTITY
    } else {
        let k0 = p / t;
        let k1 = q / t;
        [
            [
                (costh - 1.0) * k0 * k0 + 1.0,
                k0 * k1 * (costh - 1.0),
                k0 * sinth,
            ],
            [
                k0 * k1 * (costh - 1.0),
                (costh - 1.0) * k1 * k1 + 1.0,
                k1 * sinth,
            ],
            [
                -k0 * sinth,
                -k1 * sinth,
                (costh - 1.0) * (k0 * k0 + k1 * k1) + 1.0,
            ],
        ]
    };
    let b00 = rv[0][0] - p * rv[2][0];
    let b01 = rv[0][1] - p * rv[2][1];
    let b10 = rv[1][0] - q * rv[2][0];
    let b11 = rv[1][1] - q * rv[2][1];
    let dtm = b00 * b11 - b10 * b01;
    if dtm == 0.0 {
        return None;
    }
    let (bi00, bi01, bi10, bi11) = (b11 / dtm, -b01 / dtm, -b10 / dtm, b00 / dtm);
    let a00 = bi00 * j00 + bi01 * j10;
    let a01 = bi00 * j01 + bi01 * j11;
    let a10 = bi10 * j00 + bi11 * j10;
    let a11 = bi10 * j01 + bi11 * j11;
    let ata00 = a00 * a00 + a01 * a01;
    let ata01 = a00 * a10 + a01 * a11;
    let ata11 = a10 * a10 + a11 * a11;
    let gamma2 =
        0.5 * (ata00 + ata11 + ((ata00 - ata11) * (ata00 - ata11) + 4.0 * ata01 * ata01).sqrt());
    if gamma2 < 0.0 {
        return None;
    }
    let gamma = gamma2.sqrt();
    if gamma.abs() < f64::from(f32::EPSILON) {
        return None;
    }
    let (r00, r01, r10, r11) = (a00 / gamma, a01 / gamma, a10 / gamma, a11 / gamma);
    let b0 = (-r00 * r00 - r10 * r10 + 1.0).max(0.0).sqrt();
    let mut b1 = (-r01 * r01 - r11 * r11 + 1.0).max(0.0).sqrt();
    if -r00 * r01 - r10 * r11 < 0.0 {
        b1 = -b1;
    }
    let rotation = |b0: f64, b1: f64| -> Mat3 {
        let c2 = [
            b1 * r10 - b0 * r11,
            b0 * r01 - b1 * r00,
            r00 * r11 - r01 * r10,
        ];
        let mut out = [[0.0; 3]; 3];
        for (i, row) in out.iter_mut().enumerate() {
            row[0] = r00 * rv[i][0] + r10 * rv[i][1] + b0 * rv[i][2];
            row[1] = r01 * rv[i][0] + r11 * rv[i][1] + b1 * rv[i][2];
            row[2] = c2[0] * rv[i][0] + c2[1] * rv[i][1] + c2[2] * rv[i][2];
        }
        out
    };
    Some([rotation(b0, b1), rotation(-b0, -b1)])
}

/// The translation that best reprojects `object` (z = 0) at `ideal` under `rotation`.
fn translation(rotation: &Mat3, object: &[Vec3; 4], ideal: &[[f64; 2]; 4]) -> Option<Vec3> {
    let mut ata = [[0.0; 3]; 3];
    let mut atb = [0.0; 3];
    for (corner, [u, v]) in object.iter().zip(ideal) {
        let p = apply(rotation, *corner);
        for (row, rhs) in [
            ([1.0, 0.0, -u], u * p[2] - p[0]),
            ([0.0, 1.0, -v], v * p[2] - p[1]),
        ] {
            for i in 0..3 {
                for j in 0..3 {
                    ata[i][j] += row[i] * row[j];
                }
                atb[i] += row[i] * rhs;
            }
        }
    }
    let inverse = geometry::inverse(&ata)?;
    Some(apply(&inverse, atb))
}

/// The finger markers in one frame from their detected corners, each located once.
pub fn sight(markers: &[crate::detect::Marker], lens: &Lens, layout: &Layout) -> Frame {
    let mut out = Vec::new();
    for id in layout.ids() {
        let mut found = markers.iter().filter(|marker| marker.id == id);
        if let (Some(marker), None) = (found.next(), found.next())
            && let Some(pose) = locate_tag(id, &marker.corners, lens, layout)
        {
            out.push(pose);
        }
    }
    out
}

// -- references ---------------------------------------------------------------------------------

/// Where one marker sits on its finger: its pose at `angle_rad` and its hinge axis.
#[derive(Clone, Debug, PartialEq)]
pub struct MarkerReference {
    pub marker_id: u32,
    pub reference: Mat3,
    pub axis: Vec3,
    pub angle_rad: f64,
}

impl MarkerReference {
    pub fn joint_angle(&self, pose: &TagPose) -> f64 {
        let turn = hinge_turn(&self.reference, pose, self.axis);
        self.angle_rad + dot(turn, self.axis)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReferenceOrigin {
    Encoder,
    ClosedStop,
}

impl ReferenceOrigin {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Encoder => "encoder",
            Self::ClosedStop => "closed_stop",
        }
    }
}

/// One side's two marker references, and how they were fixed.
#[derive(Clone, Debug, PartialEq)]
pub struct JawReference {
    pub layout: Layout,
    pub first: MarkerReference,
    pub second: MarkerReference,
    pub origin: ReferenceOrigin,
}

impl JawReference {
    pub fn markers(&self) -> [&MarkerReference; 2] {
        [&self.first, &self.second]
    }
}

fn continuous_family(options: &[Vec<Mat3>], start: usize) -> Vec<usize> {
    let mut cost: Vec<f64> = (0..options[0].len())
        .map(|j| if j == start { 0.0 } else { f64::INFINITY })
        .collect();
    let mut back: Vec<Vec<usize>> = Vec::with_capacity(options.len());
    for pair in options.windows(2) {
        let (previous, current) = (&pair[0], &pair[1]);
        let mut step = Vec::with_capacity(current.len());
        let mut next_cost = Vec::with_capacity(current.len());
        for r in current {
            let mut best = 0;
            let mut best_cost = f64::INFINITY;
            for (j, p) in previous.iter().enumerate() {
                let c = cost[j] + geometry::geodesic(p, r);
                if c < best_cost {
                    best_cost = c;
                    best = j;
                }
            }
            // `min` over an all-infinite range keeps the first.
            step.push(best);
            next_cost.push(cost[best] + geometry::geodesic(&previous[best], r));
        }
        cost = next_cost;
        back.push(step);
    }
    let mut last = 0;
    for (j, c) in cost.iter().enumerate() {
        if *c < cost[last] {
            last = j;
        }
    }
    let mut choice = vec![last];
    for step in back.iter().rev() {
        let previous = step[*choice.last().unwrap_or(&0)];
        choice.push(previous);
    }
    choice.reverse();
    choice
}

fn off_hinge(rotations: &[Mat3]) -> f64 {
    let turns: Vec<Vec3> = rotations
        .iter()
        .map(|r| log(&relative(&rotations[0], r)))
        .collect();
    if turns.len() < 3 {
        return 0.0;
    }
    let axis = principal_axis(turns.iter().copied());
    let off: Vec<f64> = turns
        .iter()
        .map(|t| norm(sub(*t, scale(axis, dot(*t, axis)))))
        .collect();
    median(&off)
}

/// Each sighting with IPPE's tilt chosen consistently across the sequence: the smooth family
/// of choices that agrees most often with IPPE's better-fitting solution.
pub fn consistent_poses(poses: &[Option<TagPose>]) -> Vec<Option<TagPose>> {
    let seen: Vec<usize> = (0..poses.len()).filter(|i| poses[*i].is_some()).collect();
    if seen.is_empty() {
        return poses.to_vec();
    }
    let options: Vec<Vec<Mat3>> = seen
        .iter()
        .map(|i| {
            poses[*i]
                .as_ref()
                .map(TagPose::candidates)
                .unwrap_or_default()
        })
        .collect();
    let families: Vec<Vec<usize>> = (0..options[0].len())
        .map(|start| continuous_family(&options, start))
        .collect();
    let key = |family: &Vec<usize>| {
        let firsts = family.iter().filter(|pick| **pick == 0).count();
        let chosen: Vec<Mat3> = family
            .iter()
            .enumerate()
            .map(|(k, pick)| options[k][*pick])
            .collect();
        (firsts, off_hinge(&chosen))
    };
    let mut best = 0;
    let mut best_key = key(&families[0]);
    for (index, family) in families.iter().enumerate().skip(1) {
        let k = key(family);
        if k.0 > best_key.0 || (k.0 == best_key.0 && k.1 < best_key.1) {
            best = index;
            best_key = k;
        }
    }
    let mut out = poses.to_vec();
    for (i, pick) in seen.iter().zip(&families[best]) {
        if let Some(pose) = &poses[*i] {
            out[*i] = Some(TagPose {
                marker_id: pose.marker_id,
                rotation: pose.candidates()[*pick],
                center_m: pose.center_m,
                reprojection_px: pose.reprojection_px,
                alternate: None,
            });
        }
    }
    out
}

/// [`consistent_poses`] for both finger markers across a recording's frames.
pub fn consistent_frames(layout: &Layout, frames: &[Frame]) -> Vec<Frame> {
    let mut out: Vec<Frame> = vec![Vec::new(); frames.len()];
    for marker in layout.ids() {
        let poses: Vec<Option<TagPose>> =
            frames.iter().map(|f| pose_of(f, marker).cloned()).collect();
        for (i, pose) in consistent_poses(&poses).into_iter().enumerate() {
            if let Some(pose) = pose {
                out[i].push(pose);
            }
        }
    }
    out
}

fn hinge_turn(reference: &Mat3, pose: &TagPose, axis: Vec3) -> Vec3 {
    let mut best = [0.0; 3];
    let mut best_off = f64::INFINITY;
    for rotation in pose.candidates() {
        let turn = log(&relative(reference, &rotation));
        let off = norm(sub(turn, scale(axis, dot(turn, axis))));
        if off < best_off {
            best_off = off;
            best = turn;
        }
    }
    best
}

struct HingeFit {
    median_off: f64,
    axis: Vec3,
    turns: Vec<Vec3>,
    keep: Vec<bool>,
}

fn hinge_fit(reference: &Mat3, poses: &[&TagPose]) -> HingeFit {
    let normal = column(reference, 2);
    let mut turns: Vec<Vec3> = poses
        .iter()
        .map(|pose| log(&relative(reference, &pose.facing(normal))))
        .collect();
    let mut axis = principal_axis(turns.iter().copied());
    let mut keep = vec![true; poses.len()];
    let mut off = vec![0.0; poses.len()];
    for _ in 0..AXIS_ROUNDS {
        turns = poses
            .iter()
            .map(|pose| hinge_turn(reference, pose, axis))
            .collect();
        off = turns
            .iter()
            .map(|t| norm(sub(*t, scale(axis, dot(*t, axis)))))
            .collect();
        let bound = MAX_OFF_AXIS_RAD.max(percentile(&off, 80.0));
        keep = off.iter().map(|o| *o <= bound).collect();
        axis = principal_axis(
            turns
                .iter()
                .zip(&keep)
                .filter(|(_, k)| **k)
                .map(|(t, _)| *t),
        );
    }
    HingeFit {
        median_off: median(&off),
        axis,
        turns,
        keep,
    }
}

fn marker_reference(
    marker_id: u32,
    poses: &[&TagPose],
    angles: &[f64],
    zero_index: usize,
) -> Option<MarkerReference> {
    let mut best: Option<(HingeFit, Mat3)> = None;
    for reference in poses[zero_index].candidates() {
        let fit = hinge_fit(&reference, poses);
        if best
            .as_ref()
            .is_none_or(|(b, _)| fit.median_off < b.median_off)
        {
            best = Some((fit, reference));
        }
    }
    let (fit, reference) = best?;
    let mut axis = fit.axis;
    let mut along: Vec<f64> = fit.turns.iter().map(|t| dot(*t, axis)).collect();
    let kept_along: Vec<f64> = along
        .iter()
        .zip(&fit.keep)
        .filter(|(_, k)| **k)
        .map(|(a, _)| *a)
        .collect();
    if ptp(&kept_along) < MIN_REFERENCE_TRAVEL_RAD {
        return None;
    }
    let kept_angles: Vec<f64> = angles
        .iter()
        .zip(&fit.keep)
        .filter(|(_, k)| **k)
        .map(|(a, _)| *a)
        .collect();
    if correlation(&kept_along, &kept_angles) < 0.0 {
        axis = geometry::neg(axis);
        along = along.iter().map(|a| -a).collect();
    }
    let differences: Vec<f64> = angles
        .iter()
        .zip(&along)
        .zip(&fit.keep)
        .filter(|(_, k)| **k)
        .map(|((angle, a), _)| angle - a)
        .collect();
    Some(MarkerReference {
        marker_id,
        reference,
        axis,
        angle_rad: median(&differences),
    })
}

fn argmin(values: &[f64]) -> usize {
    let mut best = 0;
    for (i, v) in values.iter().enumerate() {
        if *v < values[best] {
            best = i;
        }
    }
    best
}

/// A side's marker references from frames whose encoder angle is known (NaN where not).
pub fn fit_reference(
    layout: &Layout,
    frames: &[Frame],
    encoder_angles_rad: &[f64],
) -> Option<JawReference> {
    let frames = consistent_frames(layout, frames);
    let mut refs = Vec::new();
    for marker in layout.ids() {
        let rows: Vec<usize> = (0..frames.len())
            .filter(|i| {
                pose_of(&frames[*i], marker).is_some() && encoder_angles_rad[*i].is_finite()
            })
            .collect();
        if rows.len() < MIN_REFERENCE_SIGHTINGS {
            return None;
        }
        let known: Vec<f64> = rows.iter().map(|i| encoder_angles_rad[*i]).collect();
        let poses: Vec<&TagPose> = rows
            .iter()
            .filter_map(|i| pose_of(&frames[*i], marker))
            .collect();
        refs.push(marker_reference(marker, &poses, &known, argmin(&known))?);
    }
    let second = refs.pop()?;
    let first = refs.pop()?;
    Some(JawReference {
        layout: *layout,
        first,
        second,
        origin: ReferenceOrigin::Encoder,
    })
}

/// The references without an encoder: the most-closed pose seen is `closed_angle_rad`. Valid
/// only for a recording that closes the jaw fully at least once.
pub fn closed_stop_reference(
    layout: &Layout,
    frames: &[Frame],
    closed_angle_rad: f64,
) -> Option<JawReference> {
    let [first, second] = layout.ids();
    let frames = consistent_frames(layout, frames);
    let both: Vec<usize> = (0..frames.len())
        .filter(|i| pose_of(&frames[*i], first).is_some() && pose_of(&frames[*i], second).is_some())
        .collect();
    if both.len() < MIN_REFERENCE_SIGHTINGS {
        return None;
    }
    let get = |i: usize, id: u32| pose_of(&frames[i], id).expect("both markers seen");
    let normals = (
        column(&get(both[0], first).rotation, 2),
        column(&get(both[0], second).rotation, 2),
    );
    let relative_rotations: Vec<Mat3> = both
        .iter()
        .map(|i| {
            relative(
                &get(*i, first).facing(normals.0),
                &get(*i, second).facing(normals.1),
            )
        })
        .collect();
    let turns: Vec<Vec3> = relative_rotations
        .iter()
        .map(|r| log(&relative(&relative_rotations[0], r)))
        .collect();
    let n = turns.len() as f64;
    let mean = turns.iter().fold([0.0; 3], |acc, t| {
        [acc[0] + t[0], acc[1] + t[1], acc[2] + t[2]]
    });
    let mean = scale(mean, 1.0 / n);
    let axis = principal_axis(turns.iter().map(|t| sub(*t, mean)));
    let mut spread: Vec<f64> = turns.iter().map(|t| dot(*t, axis)).collect();
    let distance: Vec<f64> = both
        .iter()
        .map(|i| norm(sub(get(*i, first).center_m, get(*i, second).center_m)))
        .collect();
    if correlation(&spread, &distance) < 0.0 {
        spread = spread.iter().map(|s| -s).collect();
    }
    let floor = percentile(&spread, 1.0);
    let closed = argmin(&spread.iter().map(|s| (s - floor).abs()).collect::<Vec<_>>());
    let mut refs = Vec::new();
    for marker in layout.ids() {
        let poses: Vec<&TagPose> = both.iter().map(|i| get(*i, marker)).collect();
        let made = marker_reference(marker, &poses, &spread, closed)?;
        refs.push(MarkerReference {
            angle_rad: closed_angle_rad,
            ..made
        });
    }
    let second_ref = refs.pop()?;
    let first_ref = refs.pop()?;
    Some(JawReference {
        layout: *layout,
        first: first_ref,
        second: second_ref,
        origin: ReferenceOrigin::ClosedStop,
    })
}

/// One frame's reading: the joint angle, from how many markers, and their disagreement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JawReading {
    pub angle_rad: f64,
    pub tags: usize,
    pub mirror_disagreement_rad: f64,
}

/// The joint angle one frame shows, or `None` when no marker (or no agreeing pair) is seen.
pub fn read_frame(reference: &JawReference, frame: &Frame) -> Option<JawReading> {
    let angles: Vec<f64> = reference
        .markers()
        .iter()
        .filter_map(|r| pose_of(frame, r.marker_id).map(|pose| r.joint_angle(pose)))
        .collect();
    match angles.as_slice() {
        [] => None,
        [one] => Some(JawReading {
            angle_rad: *one,
            tags: 1,
            mirror_disagreement_rad: f64::NAN,
        }),
        [a, b, ..] => {
            let disagreement = (a - b).abs();
            (disagreement <= MAX_MIRROR_DISAGREEMENT_RAD).then_some(JawReading {
                angle_rad: (a + b) / 2.0,
                tags: 2,
                mirror_disagreement_rad: disagreement,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::exp;

    fn lens() -> Lens {
        Lens::new(
            640,
            480,
            [271.15, 0.0, 310.57, 0.0, 270.43, 221.63, 0.0, 0.0, 1.0],
            "equidistant",
            &[-0.0413, -0.00224, -0.00366, 0.00155],
        )
        .unwrap()
    }

    #[test]
    fn ippe_recovers_a_projected_markers_pose() {
        let layout = Layout::yubi("right").unwrap();
        let lens = lens();
        let rotation = exp([0.5, -0.3, 0.2]);
        let center = [0.012, 0.018, 0.045];
        let pixels = layout.corners_m().map(|c| {
            let p = apply(&rotation, c);
            lens.project([p[0] + center[0], p[1] + center[1], p[2] + center[2]])
                .unwrap()
        });
        let pose = locate_tag(0, &pixels, &lens, &layout).unwrap();
        assert!(
            geometry::geodesic(&pose.rotation, &rotation) < 1e-6,
            "{pose:?}"
        );
        assert!(norm(sub(pose.center_m, center)) < 1e-8);
        assert!(pose.reprojection_px < 1e-6);
    }
}
