//! The jaw solved end to end from rendered finger markers that turn oppositely about their
//! hinges (the scenes `sx_embodiments`' Python tests rendered with OpenCV, 2026-11-25 to 27):
//! the detector, the poses, the references and the smoother, each sensor fault injected into
//! one sensor, the fused track within a degree of the truth at p95.

use std::f64::consts::PI;
use std::sync::OnceLock;

use sx_jaw::detect::{Detector, Luma, Region};
use sx_jaw::fusion::{
    EncoderLaw, EncoderReads, FusionModel, JawSensors, TagReads, camera_lag_ns, fuse_jaw,
    solve_jaw, tag_reads, unfold_encoder,
};
use sx_jaw::geometry::{Mat3, Vec3, apply, exp, matmul, percentile};
use sx_jaw::lens::Lens;
use sx_jaw::solve::{has_jaw_tags, spool_rows};
use sx_jaw::tags::{
    Frame, JawReference, Layout, ReferenceOrigin, closed_stop_reference, fit_reference, sight,
};

const FRAME_NS: i64 = 33_333_333;
const READ_NS: i64 = 10_000_000;
const SECONDS: f64 = 8.0;
const ONE_DEGREE: f64 = PI / 180.0;
const W: usize = 640;
const H: usize = 480;

fn lens() -> Lens {
    Lens::new(
        W as u32,
        H as u32,
        [300.0, 0.0, 320.0, 0.0, 300.0, 240.0, 0.0, 0.0, 1.0],
        "none",
        &[],
    )
    .unwrap()
}

fn layout() -> Layout {
    Layout::yubi("right").unwrap()
}

/// Each finger's hinge, and the marker's lever from it, in the camera frame (metres): the
/// markers sit low in the view, tilted 40 degrees away from the lens, glued a little askew.
const HINGES: [Vec3; 2] = [[-0.025, 0.040, 0.10], [0.025, 0.040, 0.10]];
const LEVER: f64 = 0.03;
const GLUE: [f64; 2] = [4.0 * PI / 180.0, -7.0 * PI / 180.0];

/// The pose of finger `index`'s marker at joint angle `theta`.
fn marker_pose(theta: f64, index: usize) -> (Mat3, Vec3) {
    let tilt = 40f64.to_radians();
    let about_x: Mat3 = [
        [1.0, 0.0, 0.0],
        [0.0, tilt.cos(), -tilt.sin()],
        [0.0, tilt.sin(), tilt.cos()],
    ];
    let facing = matmul(
        &about_x,
        &[[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
    );
    let sign = if index == 0 { -1.0 } else { 1.0 };
    let normal = [facing[0][2], facing[1][2], facing[2][2]];
    let turn = -sign * theta;
    let hinge_turn = exp([normal[0] * turn, normal[1] * turn, normal[2] * turn]);
    let glue = exp([
        normal[0] * GLUE[index],
        normal[1] * GLUE[index],
        normal[2] * GLUE[index],
    ]);
    let lever = apply(
        &hinge_turn,
        apply(&facing, [sign * LEVER * 0.3, LEVER, 0.0]),
    );
    let rotation = matmul(&matmul(&hinge_turn, &glue), &facing);
    let h = HINGES[index];
    (
        rotation,
        [h[0] + lever[0], h[1] + lever[1], h[2] + lever[2]],
    )
}

/// A white frame with the two markers drawn through the lens (4x4 samples per pixel).
fn render(theta: f64, hide: Option<usize>) -> Vec<u8> {
    let lens = lens();
    let layout = layout();
    let mut frame = vec![255u8; W * H];
    for index in 0..2 {
        if hide == Some(index) {
            continue;
        }
        let (rotation, center) = marker_pose(theta, index);
        let code = sx_jaw::dictionary::code(layout.ids()[index]).unwrap();
        let half = layout.marker_side_m / 2.0;
        let corners = layout.corners_m().map(|c| {
            let p = apply(&rotation, c);
            lens.project([p[0] + center[0], p[1] + center[1], p[2] + center[2]])
                .unwrap()
        });
        let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for c in &corners {
            x0 = x0.min(c[0]);
            y0 = y0.min(c[1]);
            x1 = x1.max(c[0]);
            y1 = y1.max(c[1]);
        }
        let normal = [rotation[0][2], rotation[1][2], rotation[2][2]];
        let offset = normal[0] * center[0] + normal[1] * center[1] + normal[2] * center[2];
        for py in (y0.floor() as usize).saturating_sub(1)..=(y1.ceil() as usize + 1).min(H - 1) {
            for px in (x0.floor() as usize).saturating_sub(1)..=(x1.ceil() as usize + 1).min(W - 1)
            {
                let mut covered = 0;
                let mut white = 0;
                for sy in 0..4 {
                    for sx in 0..4 {
                        let u = px as f64 - 0.5 + (f64::from(sx) + 0.5) / 4.0;
                        let v = py as f64 - 0.5 + (f64::from(sy) + 0.5) / 4.0;
                        let ray = [(u - 320.0) / 300.0, (v - 240.0) / 300.0, 1.0];
                        let along =
                            offset / (normal[0] * ray[0] + normal[1] * ray[1] + normal[2] * ray[2]);
                        let hit = [
                            ray[0] * along - center[0],
                            ray[1] * along - center[1],
                            ray[2] * along - center[2],
                        ];
                        // In the marker's own frame: x right, y up, its top-left at (-h, h).
                        let lx = rotation[0][0] * hit[0]
                            + rotation[1][0] * hit[1]
                            + rotation[2][0] * hit[2];
                        let ly = rotation[0][1] * hit[0]
                            + rotation[1][1] * hit[1]
                            + rotation[2][1] * hit[2];
                        if lx.abs() >= half || ly.abs() >= half {
                            continue;
                        }
                        covered += 1;
                        let col = ((lx + half) / (2.0 * half) * 6.0) as usize;
                        let row = ((half - ly) / (2.0 * half) * 6.0) as usize;
                        let bit = (1..5).contains(&row)
                            && (1..5).contains(&col)
                            && (code >> (15 - ((row - 1) * 4 + (col - 1)))) & 1 == 1;
                        if bit {
                            white += 1;
                        }
                    }
                }
                if covered > 0 {
                    let background = 255.0 * f64::from(16 - covered);
                    let value = (background + 255.0 * f64::from(white)) / 16.0;
                    frame[py * W + px] = value.round() as u8;
                }
            }
        }
    }
    frame
}

fn sighted(theta: f64, hide: Option<usize>) -> Frame {
    let image = render(theta, hide);
    let luma = Luma::new(&image, W, H, W).unwrap();
    let markers = Detector::new().detect(&luma, Region::whole(W, H), &layout().ids());
    sight(&markers, &lens(), &layout())
}

fn sweep() -> &'static (Vec<f64>, Vec<Frame>) {
    static SWEEP: OnceLock<(Vec<f64>, Vec<Frame>)> = OnceLock::new();
    SWEEP.get_or_init(|| {
        let mut theta = vec![0.0; 8];
        theta.extend((0..150).map(|i| 0.6 * f64::from(i) / 149.0));
        theta.extend((0..150).map(|i| 0.6 - 0.6 * f64::from(i) / 149.0));
        let frames = theta.iter().map(|a| sighted(*a, None)).collect();
        (theta, frames)
    })
}

fn truth(t_ns: i64) -> f64 {
    let s = t_ns as f64 / 1e9;
    0.30 + 0.25 * (2.0 * PI * s / 4.0).cos() + 0.04 * (2.0 * PI * s / 0.9).sin()
}

fn camera() -> &'static (Vec<i64>, Vec<Frame>) {
    static CAMERA: OnceLock<(Vec<i64>, Vec<Frame>)> = OnceLock::new();
    CAMERA.get_or_init(|| {
        let t: Vec<i64> = (0..)
            .map(|i| i * FRAME_NS)
            .take_while(|t| (*t as f64) < SECONDS * 1e9)
            .collect();
        let frames = t.iter().map(|t| sighted(truth(*t), None)).collect();
        (t, frames)
    })
}

fn reference() -> &'static JawReference {
    static REFERENCE: OnceLock<JawReference> = OnceLock::new();
    REFERENCE.get_or_init(|| {
        let (theta, frames) = sweep();
        fit_reference(&layout(), frames, theta).expect("a reference from the sweep")
    })
}

fn tags(hidden: Option<(f64, f64)>) -> TagReads {
    let (t, frames) = camera();
    let shown: Vec<Frame> = t
        .iter()
        .zip(frames)
        .map(|(t, f)| match hidden {
            Some((a, b)) if (a..b).contains(&(*t as f64 / 1e9)) => Frame::new(),
            _ => f.clone(),
        })
        .collect();
    tag_reads(reference(), t, &shown, &FusionModel::default()).unwrap()
}

/// A deterministic normal draw (xorshift and Box-Muller).
struct Noise(u64);

impl Noise {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    fn normal(&mut self, sigma: f64) -> f64 {
        let (a, b) = (self.uniform().max(1e-12), self.uniform());
        sigma * (-2.0 * a.ln()).sqrt() * (2.0 * PI * b).cos()
    }
}

fn encoder(offset: f64) -> EncoderReads {
    let mut noise = Noise(7);
    let t: Vec<i64> = (0..)
        .map(|i| i * READ_NS)
        .take_while(|t| (*t as f64) < SECONDS * 1e9)
        .collect();
    let a = t
        .iter()
        .map(|t| truth(*t) + offset + noise.normal(0.002))
        .collect();
    EncoderReads::new(t, a).unwrap()
}

fn p95(t: &[i64], angle: &[f64]) -> f64 {
    let errors: Vec<f64> = t
        .iter()
        .zip(angle)
        .map(|(t, a)| (a - truth(*t)).abs())
        .collect();
    percentile(&errors, 95.0)
}

fn model() -> FusionModel {
    FusionModel::default()
}

fn within(reference: &JawReference, tolerance: f64) {
    let theta: Vec<f64> = (0..150)
        .map(|i| 0.57 * f64::from(i) / 149.0)
        .chain((0..150).map(|i| 0.57 - 0.57 * f64::from(i) / 149.0))
        .collect();
    let t: Vec<i64> = (0..theta.len() as i64).map(|i| i * FRAME_NS).collect();
    for hide in [None, Some(0), Some(1)] {
        let frames: Vec<Frame> = theta.iter().map(|a| sighted(*a, hide)).collect();
        let (track, _) = solve_jaw(reference, &t, &frames, None, &model()).unwrap();
        let errors: Vec<f64> = track
            .angle_rad
            .iter()
            .zip(&theta)
            .map(|(a, b)| (a - b).abs())
            .collect();
        assert!(percentile(&errors, 95.0) < tolerance, "{hide:?}");
    }
}

#[test]
fn a_reference_learned_from_a_working_encoder_reads_the_joint() {
    assert_eq!(reference().origin, ReferenceOrigin::Encoder);
    within(reference(), ONE_DEGREE);
}

#[test]
fn without_an_encoder_the_closed_stop_fixes_the_joint() {
    let reference = closed_stop_reference(&layout(), &sweep().1, 0.0).unwrap();
    assert_eq!(reference.origin, ReferenceOrigin::ClosedStop);
    within(&reference, ONE_DEGREE);
}

#[test]
fn every_sample_says_which_sensor_saw_it_and_the_rows_are_the_seals() {
    let (theta, frames) = sweep();
    let mut shown = frames.clone();
    for i in [0, 1, 120, 121, 122] {
        shown[i] = Frame::new();
    }
    shown[180].retain(|p| p.marker_id == layout().first_id);
    let t: Vec<i64> = (0..frames.len() as i64)
        .map(|i| 1_000_000_000 + i * FRAME_NS)
        .collect();
    let (track, tags) = solve_jaw(reference(), &t, &shown, None, &model()).unwrap();
    assert_eq!(track.sensors[121], JawSensors::Neither);
    assert_eq!(track.sensors[180], JawSensors::Tags);
    assert!(track.std_rad[121] > track.std_rad[20]);
    let worst = track
        .angle_rad
        .iter()
        .zip(theta)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);
    assert!(worst < 1.5 * ONE_DEGREE, "{worst}");
    assert!(tags.mirror_p95_rad() < 2.0 * ONE_DEGREE);
    let (rows, _) = spool_rows(&track, (0.0, 0.9599)).unwrap();
    let lines: Vec<&str> = rows.lines().collect();
    assert_eq!(lines.len(), frames.len());
    assert!(lines[121].ends_with(",neither") && lines[20].ends_with(",tags"));
}

#[test]
fn a_recording_without_the_markers_stops_early() {
    let blank = vec![200u8; W * H];
    let luma = Luma::new(&blank, W, H, W).unwrap();
    let none = sight(
        &Detector::new().detect(&luma, Region::whole(W, H), &[0, 1]),
        &lens(),
        &layout(),
    );
    assert!(!has_jaw_tags(&vec![none; 5]));
    assert!(has_jaw_tags(&[sighted(0.2, None)]));
}

#[test]
fn a_dead_encoder_leaves_the_tags_alone() {
    let t = &camera().0;
    let fused = fuse_jaw(t, None, Some(&tags(None)), 0, &model()).unwrap();
    assert!(p95(t, &fused.angle_rad) < ONE_DEGREE);
    assert!(fused.sensors.iter().all(|s| *s == JawSensors::Tags));
}

#[test]
fn an_encoder_dropout_and_hidden_tags_lose_no_good_data() {
    let t = &camera().0;
    let full = encoder(0.0);
    let keep: Vec<usize> = (0..full.len())
        .filter(|i| full.t_ns[*i] < 2_000_000_000 || full.t_ns[*i] >= 3_500_000_000)
        .collect();
    let gone = EncoderReads::new(
        keep.iter().map(|i| full.t_ns[*i]).collect(),
        keep.iter().map(|i| full.angle_rad[*i]).collect(),
    )
    .unwrap();
    let fused = fuse_jaw(t, Some(&gone), Some(&tags(Some((5.0, 6.0)))), 0, &model()).unwrap();
    assert!(p95(t, &fused.angle_rad) < ONE_DEGREE);
    for (i, ti) in t.iter().enumerate() {
        let s = *ti as f64 / 1e9;
        if s > 2.1 && s < 3.4 {
            assert_eq!(fused.sensors[i], JawSensors::Tags);
        }
        if s > 5.1 && s < 5.9 {
            assert_eq!(fused.sensors[i], JawSensors::Encoder);
        }
    }
    assert_eq!(fused.longest_unsupported_s(ONE_DEGREE), 0.0);
}

#[test]
fn impossible_encoder_spikes_get_no_weight() {
    let t = &camera().0;
    let clean = encoder(0.0);
    let mut angle = clean.angle_rad.clone();
    let mut start = 25;
    while start + 10 < angle.len() {
        for (k, sign) in [1.0, -1.0, 1.0, -1.0, 1.0].iter().enumerate() {
            angle[start + k] += 0.3 * sign;
        }
        for a in &mut angle[start + 5..start + 10] {
            *a += 0.2;
        }
        start += 50;
    }
    let spiky = EncoderReads::new(clean.t_ns.clone(), angle).unwrap();
    let fused = fuse_jaw(t, Some(&spiky), Some(&tags(None)), 0, &model()).unwrap();
    assert!(p95(t, &fused.angle_rad) < ONE_DEGREE);
    let worst = t
        .iter()
        .zip(&fused.angle_rad)
        .map(|(t, a)| (a - truth(*t)).abs())
        .fold(0.0, f64::max);
    assert!(worst < 2.0 * ONE_DEGREE);
    let alone = fuse_jaw(t, Some(&spiky), None, 0, &model()).unwrap();
    assert!(p95(t, &alone.angle_rad) < 2.0 * ONE_DEGREE);
}

#[test]
fn a_mis_zeroed_encoder_is_read_against_the_tags_zero() {
    let t = &camera().0;
    let fused = fuse_jaw(t, Some(&encoder(0.05)), Some(&tags(None)), 0, &model()).unwrap();
    assert!(p95(t, &fused.angle_rad) < ONE_DEGREE);
    let mut bias = fused.encoder_bias_rad.clone();
    bias.sort_by(f64::total_cmp);
    assert!((percentile(&bias, 50.0) - 0.05).abs() < 0.005);
    let hidden = fuse_jaw(
        t,
        Some(&encoder(0.05)),
        Some(&tags(Some((3.0, 5.0)))),
        0,
        &model(),
    )
    .unwrap();
    assert!(p95(t, &hidden.angle_rad) < ONE_DEGREE);
}

#[test]
fn neither_sensor_is_what_qc_refuses() {
    let t = &camera().0;
    let fused = fuse_jaw(t, None, Some(&tags(Some((3.0, 4.5)))), 0, &model()).unwrap();
    for (i, ti) in t.iter().enumerate() {
        let s = *ti as f64 / 1e9;
        if s > 3.4 && s < 4.1 {
            assert!(fused.std_rad[i] > 2.0 * ONE_DEGREE);
        }
        if s > 1.0 && s < 2.5 {
            assert!(fused.std_rad[i] < 0.5 * ONE_DEGREE);
        }
    }
    let blind = fused.longest_unsupported_s(2.0 * ONE_DEGREE);
    assert!(0.7 < blind && blind < 1.6, "{blind}");
}

fn late(tags: &TagReads, by: i64) -> TagReads {
    TagReads::new(
        tags.t_ns.iter().map(|t| t + by).collect(),
        tags.angle_rad.clone(),
        tags.std_rad.clone(),
        tags.mirror_rad.clone(),
    )
    .unwrap()
}

#[test]
fn the_camera_lag_is_measured_against_the_encoder_on_any_scale_and_through_blips() {
    let stamped = late(&tags(None), 30_000_000);
    let lag = camera_lag_ns(&stamped, &encoder(0.05)).unwrap();
    assert!((lag - 30_000_000).abs() <= 5_000_000, "{lag}");
    let fused = fuse_jaw(
        &camera().0,
        Some(&encoder(0.05)),
        Some(&stamped),
        lag,
        &model(),
    )
    .unwrap();
    assert!(p95(&camera().0, &fused.angle_rad) < ONE_DEGREE);

    let clean = encoder(0.0);
    let scaled = EncoderReads::new(
        clean.t_ns.clone(),
        clean.angle_rad.iter().map(|a| a / 0.77 + 0.1).collect(),
    )
    .unwrap();
    let lag = camera_lag_ns(&stamped, &scaled).unwrap();
    assert!((lag - 30_000_000).abs() <= 5_000_000, "{lag}");

    let mut angle = clean.angle_rad.clone();
    let mut start = 25;
    while start + 5 < angle.len() {
        for (k, sign) in [1.0, -1.0, 1.0, -1.0, 1.0].iter().enumerate() {
            angle[start + k] += 0.3 * sign;
        }
        start += 50;
    }
    let lag = camera_lag_ns(
        &stamped,
        &EncoderReads::new(clean.t_ns.clone(), angle).unwrap(),
    )
    .unwrap();
    assert!((lag - 30_000_000).abs() <= 5_000_000, "{lag}");
}

#[test]
fn where_neither_sensor_saw_the_jaw_it_is_linear_between_readings_and_held_at_the_ends() {
    let t = &camera().0;
    let base = tags(Some((1.2, 2.2)));
    let angle: Vec<f64> = t
        .iter()
        .zip(&base.angle_rad)
        .map(|(t, a)| if *t as f64 / 1e9 >= 6.5 { f64::NAN } else { *a })
        .collect();
    let tags = TagReads::new(
        base.t_ns.clone(),
        angle,
        base.std_rad.clone(),
        base.mirror_rad.clone(),
    )
    .unwrap();
    let fused = fuse_jaw(t, None, Some(&tags), 0, &model()).unwrap();
    let gap: Vec<usize> = (0..t.len())
        .filter(|i| fused.sensors[*i] == JawSensors::Neither)
        .collect();
    let middle: Vec<usize> = gap
        .iter()
        .copied()
        .filter(|i| (t[*i] as f64) < 3e9)
        .collect();
    let tail: Vec<usize> = gap
        .iter()
        .copied()
        .filter(|i| (t[*i] as f64) > 6e9)
        .collect();
    assert!(middle.len() > 20 && tail.len() > 30);
    let (before, after) = (middle[0] - 1, middle[middle.len() - 1] + 1);
    for i in &middle {
        let f = (t[*i] - t[before]) as f64 / (t[after] - t[before]) as f64;
        let line = fused.angle_rad[before] + f * (fused.angle_rad[after] - fused.angle_rad[before]);
        assert!((fused.angle_rad[*i] - line).abs() < 0.005);
    }
    let held = &fused.angle_rad[tail[0]..=tail[tail.len() - 1]];
    assert!(held.iter().all(|a| (a - held[0]).abs() < 1e-9));
    assert!(fused.std_rad[tail[tail.len() - 1]] > 2.0 * ONE_DEGREE);
    assert!(fused.longest_unsupported_s(2.0 * ONE_DEGREE) > 1.0);
}

fn relative(zero: f64) -> EncoderReads {
    let clean = encoder(0.0);
    let folded = clean
        .angle_rad
        .iter()
        .map(|a| ((a - zero + PI).rem_euclid(2.0 * PI) - PI).abs())
        .collect();
    EncoderReads::new(clean.t_ns, folded).unwrap()
}

#[test]
fn an_encoder_zeroed_mid_travel_is_unfolded_on_the_tags() {
    let unfolded = unfold_encoder(&tags(None), &relative(0.3));
    assert_eq!(unfolded.law, EncoderLaw::Unfolded);
    assert!(unfolded.reflections >= 4);
    let (t, frames) = camera();
    let (fused, _) = solve_jaw(reference(), t, frames, Some(&relative(0.3)), &model()).unwrap();
    assert!(p95(t, &fused.angle_rad) < ONE_DEGREE);
}

#[test]
fn an_encoder_zeroed_far_off_is_carried_from_the_tags_level() {
    let (t, frames) = camera();
    let shifted = encoder(1.0);
    assert_eq!(
        unfold_encoder(&tags(None), &shifted).law,
        EncoderLaw::AsRead
    );
    let (fused, _) = solve_jaw(reference(), t, frames, Some(&shifted), &model()).unwrap();
    assert!(p95(t, &fused.angle_rad) < ONE_DEGREE);
    let bias = percentile(&fused.encoder_bias_rad, 50.0);
    assert!((bias - 1.0).abs() < 0.02, "{bias}");
}

#[test]
fn an_absolute_encoder_is_read_as_is_and_noise_is_refused() {
    let clean = encoder(0.0);
    let unfolded = unfold_encoder(&tags(None), &clean);
    assert_eq!(unfolded.law, EncoderLaw::AsRead);
    assert_eq!(unfolded.reads.as_ref(), Some(&clean));
    let mut noise = Noise(3);
    let random = EncoderReads::new(
        clean.t_ns.clone(),
        clean.t_ns.iter().map(|_| 0.9 * noise.uniform()).collect(),
    )
    .unwrap();
    let refused = unfold_encoder(&tags(None), &random);
    assert_eq!(refused.law, EncoderLaw::Refused);
    assert!(refused.reads.is_none());
}
