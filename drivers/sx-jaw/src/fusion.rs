//! One jaw angle from the YUBI gripper's two sensors, its encoder and its wrist tags
//! (formerly `sx_embodiments.jaw_fusion`, embodiments #326, #327, #332 to #335).
//!
//! One offline estimator over a whole take and one side:
//! - State: the jaw angle, its rate, and the encoder's bias (how far its read, mapped onto the
//!   tags' angle by the take's line, sits from the jaw). The bias steps where the load changes
//!   and is flat while the jaw moves freely.
//! - Process: white acceleration ([`FusionModel::acceleration_rad_s2`] per root second).
//! - Measurements: every mapped encoder read is `angle + bias`; every tag frame is `angle` at
//!   its exposure less the camera's lag, with its own deviation.
//! - Solver: a Rauch-Tung-Striebel smoother, reweighted coarse to fine with Cauchy weights on
//!   the readings and on the bias's steps; nothing is gated away.
//! - Output per sample: the angle, its posterior deviation and which sensors carried it; where
//!   neither did, the angle is linear between the trusted readings either side.

use crate::geometry::{
    Mat3, interp, interp_ns, least_squares_line, median, percentile, ptp, search_sorted,
};
use crate::tags::{Frame, JawReference, consistent_frames, pose_of, read_frame};

pub const STUDENT_NU: f64 = 1.0;
pub const REWEIGHT_SCALES: [f64; 7] = [16.0, 8.0, 4.0, 2.0, 1.0, 1.0, 1.0];
pub const TRUSTED_WEIGHT: f64 = 0.2;
pub const TAG_REACH_NS: i64 = 20_000_000;
pub const ENCODER_REACH_NS: i64 = 15_000_000;
pub const MAX_CAMERA_LAG_NS: i64 = 200_000_000;
pub const CAMERA_LAG_STEP_NS: i64 = 2_500_000;
pub const MIN_LAG_FRAMES: usize = 60;
pub const LAG_KEPT: f64 = 0.9;
pub const FOLD_BAND_RAD: f64 = 0.05;
pub const REFLECT_RATE_RAD_S: f64 = 0.5;
pub const REFLECT_WINDOW: usize = 5;
pub const ALIGNED_RMS_RAD: f64 = 0.04;
pub const ALIGNED_SCALE: (f64, f64) = (0.5, 2.0);
pub const TAG_TRAVEL_RAD: (f64, f64) = (-0.15, 1.1);
pub const LEVEL_MARGIN_RAD: f64 = 0.02;
/// A sample whose deviation exceeds this was seen by neither sensor (sx-seal's
/// `SOLVED_BLIND_STD_RAD`).
pub const BLIND_STD_RAD: f64 = 2.0 * std::f64::consts::PI / 180.0;
pub const MAX_BLIND_S: f64 = 0.5;
pub const TRIMMED_SHARE: f64 = 0.5;
pub const TRIMMED_STARTS: usize = 64;
pub const TRIMMED_STEPS: usize = 20;

/// Which sensors carried one fused sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum JawSensors {
    EncoderAndTags,
    Tags,
    Encoder,
    Neither,
}

impl JawSensors {
    pub const ALL: [Self; 4] = [
        Self::EncoderAndTags,
        Self::Tags,
        Self::Encoder,
        Self::Neither,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            Self::EncoderAndTags => "encoder_and_tags",
            Self::Tags => "tags",
            Self::Encoder => "encoder",
            Self::Neither => "neither",
        }
    }
}

/// The physical and sensor constants of the estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FusionModel {
    pub acceleration_rad_s2: f64,
    pub encoder_std_rad: f64,
    pub bias_walk_rad: f64,
    pub bias_free_rad: f64,
    pub bias_prior_rad: f64,
    pub both_tags_std_rad: f64,
    pub one_tag_std_rad: f64,
    pub reprojection_px: f64,
}

impl Default for FusionModel {
    fn default() -> Self {
        Self {
            acceleration_rad_s2: 4.0,
            encoder_std_rad: 0.003,
            bias_walk_rad: 0.0035,
            bias_free_rad: 0.03,
            bias_prior_rad: 0.25,
            both_tags_std_rad: 0.010,
            one_tag_std_rad: 0.016,
            reprojection_px: 0.5,
        }
    }
}

/// An encoder's reads on the host clock, stamps strictly increasing.
#[derive(Clone, Debug, PartialEq)]
pub struct EncoderReads {
    pub t_ns: Vec<i64>,
    pub angle_rad: Vec<f64>,
}

impl EncoderReads {
    pub fn new(t_ns: Vec<i64>, angle_rad: Vec<f64>) -> Result<Self, String> {
        check_series(&t_ns, angle_rad.len(), "encoder reads")?;
        if angle_rad.iter().any(|a| !a.is_finite()) {
            return Err("encoder reads hold a non-finite angle".to_owned());
        }
        Ok(Self { t_ns, angle_rad })
    }

    pub fn len(&self) -> usize {
        self.t_ns.len()
    }

    pub fn is_empty(&self) -> bool {
        self.t_ns.is_empty()
    }
}

/// Per frame, the joint angle the tags read and its deviation; NaN where unseen.
#[derive(Clone, Debug, PartialEq)]
pub struct TagReads {
    pub t_ns: Vec<i64>,
    pub angle_rad: Vec<f64>,
    pub std_rad: Vec<f64>,
    pub mirror_rad: Vec<f64>,
}

impl TagReads {
    pub fn new(
        t_ns: Vec<i64>,
        angle_rad: Vec<f64>,
        std_rad: Vec<f64>,
        mirror_rad: Vec<f64>,
    ) -> Result<Self, String> {
        check_series(&t_ns, angle_rad.len(), "tag frames")?;
        if std_rad.len() != angle_rad.len() || mirror_rad.len() != angle_rad.len() {
            return Err("tag frames need one deviation and mirror per angle".to_owned());
        }
        Ok(Self {
            t_ns,
            angle_rad,
            std_rad,
            mirror_rad,
        })
    }

    pub fn seen(&self, i: usize) -> bool {
        self.angle_rad[i].is_finite()
    }

    /// The seen frames' stamps and angles.
    fn seen_series(&self) -> (Vec<i64>, Vec<f64>, Vec<f64>) {
        let mut t = Vec::new();
        let mut z = Vec::new();
        let mut s = Vec::new();
        for i in 0..self.t_ns.len() {
            if self.seen(i) {
                t.push(self.t_ns[i]);
                z.push(self.angle_rad[i]);
                s.push(self.std_rad[i]);
            }
        }
        (t, z, s)
    }

    /// The markers' disagreement at p95 over the frames that read both (NaN if none).
    pub fn mirror_p95_rad(&self) -> f64 {
        let both: Vec<f64> = self
            .mirror_rad
            .iter()
            .copied()
            .filter(|m| m.is_finite())
            .collect();
        percentile(&both, 95.0)
    }
}

fn check_series(t_ns: &[i64], values: usize, what: &str) -> Result<(), String> {
    if t_ns.len() != values {
        return Err(format!("{what} need one timestamp per value"));
    }
    if t_ns.windows(2).any(|w| w[1] <= w[0]) {
        return Err(format!("{what} need strictly increasing timestamps"));
    }
    Ok(())
}

/// Each frame's tag reading with its deviation, NaN where unseen.
pub fn tag_reads(
    reference: &JawReference,
    t_ns: &[i64],
    frames: &[Frame],
    model: &FusionModel,
) -> Result<TagReads, String> {
    if t_ns.len() != frames.len() {
        return Err("frames need one timestamp each".to_owned());
    }
    let frames = consistent_frames(&reference.layout, frames);
    let n = t_ns.len();
    let mut angle = vec![f64::NAN; n];
    let mut std = vec![f64::NAN; n];
    let mut mirror = vec![f64::NAN; n];
    for (i, frame) in frames.iter().enumerate() {
        let Some(reading) = read_frame(reference, frame) else {
            continue;
        };
        let used: Vec<f64> = reference
            .layout
            .ids()
            .iter()
            .filter_map(|m| pose_of(frame, *m).map(|p| p.reprojection_px))
            .collect();
        let base = if used.len() == 2 {
            model.both_tags_std_rad
        } else {
            model.one_tag_std_rad
        };
        let mean = used.iter().sum::<f64>() / used.len() as f64;
        angle[i] = reading.angle_rad;
        std[i] = base * (mean / model.reprojection_px).max(1.0);
        mirror[i] = reading.mirror_disagreement_rad;
    }
    TagReads::new(t_ns.to_vec(), angle, std, mirror)
}

fn near(at: &[i64], stamps: &[i64], reach: i64) -> Vec<bool> {
    if stamps.is_empty() {
        return vec![false; at.len()];
    }
    let n = stamps.len();
    at.iter()
        .map(|a| {
            let index = search_sorted(stamps, *a).clamp(1, 1.max(n - 1));
            let lower = stamps[(index - 1).min(n - 1)];
            let upper = stamps[index.min(n - 1)];
            (lower - a).abs().min((upper - a).abs()) <= reach
        })
        .collect()
}

fn argsort_by(values: &[f64]) -> Vec<usize> {
    let mut order: Vec<usize> = (0..values.len()).collect();
    order.sort_by(|a, b| values[*a].total_cmp(&values[*b]));
    order
}

/// The best line through `(read, angle)`: the residuals' RMS over its best nine tenths, and its
/// slope and offset, fitted twice.
pub fn line(read: &[f64], angle: &[f64]) -> (f64, f64, f64) {
    let (slope, offset) = least_squares_line(read, angle);
    let residual: Vec<f64> = read
        .iter()
        .zip(angle)
        .map(|(r, a)| (a - (slope * r + offset)).abs())
        .collect();
    let keep = 2.max((LAG_KEPT * read.len() as f64) as usize);
    let kept: Vec<usize> = argsort_by(&residual).into_iter().take(keep).collect();
    let kx: Vec<f64> = kept.iter().map(|i| read[*i]).collect();
    let ky: Vec<f64> = kept.iter().map(|i| angle[*i]).collect();
    let (slope, offset) = least_squares_line(&kx, &ky);
    let mut residual: Vec<f64> = read
        .iter()
        .zip(angle)
        .map(|(r, a)| (a - (slope * r + offset)).abs())
        .collect();
    residual.sort_by(f64::total_cmp);
    let used = &residual[..kept.len()];
    let rms = (used.iter().map(|r| r * r).sum::<f64>() / used.len() as f64).sqrt();
    (rms, slope, offset)
}

/// How far the camera's stamps trail the encoder's: the lag that best lines the tags up with
/// the line through the encoder; `None` with too few frames beside an encoder read.
pub fn camera_lag_ns(tags: &TagReads, encoder: &EncoderReads) -> Option<i64> {
    let (t, angle, _) = tags.seen_series();
    if encoder.len() < 2 || t.len() < MIN_LAG_FRAMES {
        return None;
    }
    let n = encoder.len();
    let spread = |lag: i64| -> f64 {
        let mut reads = Vec::new();
        let mut angles = Vec::new();
        for (ti, ai) in t.iter().zip(&angle) {
            let at = ti - lag;
            let index = search_sorted(&encoder.t_ns, at).clamp(1, n - 1);
            let gap = (encoder.t_ns[index] - at)
                .abs()
                .min((encoder.t_ns[index - 1] - at).abs());
            if gap <= ENCODER_REACH_NS {
                reads.push(interp_ns(at, &encoder.t_ns, &encoder.angle_rad));
                angles.push(*ai);
            }
        }
        if reads.len() < MIN_LAG_FRAMES {
            return f64::INFINITY;
        }
        line(&reads, &angles).0
    };
    let mut best = -MAX_CAMERA_LAG_NS;
    let mut best_spread = spread(best);
    let mut lag = -MAX_CAMERA_LAG_NS + CAMERA_LAG_STEP_NS;
    while lag <= MAX_CAMERA_LAG_NS {
        let s = spread(lag);
        if s < best_spread {
            best_spread = s;
            best = lag;
        }
        lag += CAMERA_LAG_STEP_NS;
    }
    best_spread.is_finite().then_some(best)
}

/// How a take's encoder reads were taken ([`unfold_encoder`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncoderLaw {
    AsRead,
    Unfolded,
    Refused,
}

#[derive(Clone, Debug, PartialEq)]
pub struct UnfoldedEncoder {
    pub reads: Option<EncoderReads>,
    pub law: EncoderLaw,
    pub reflections: usize,
    pub inferred: usize,
    pub rms_rad: f64,
}

fn aligned(tags: &TagReads, encoder: &EncoderReads) -> (f64, f64) {
    let (t, angle, _) = tags.seen_series();
    if encoder.len() < 2 {
        return (f64::NAN, f64::NAN);
    }
    let close = near(&t, &encoder.t_ns, ENCODER_REACH_NS);
    let first = encoder.t_ns[0];
    let last = encoder.t_ns[encoder.len() - 1];
    let mut reads = Vec::new();
    let mut angles = Vec::new();
    for i in 0..t.len() {
        if close[i] && t[i] >= first && t[i] <= last {
            reads.push(interp_ns(t[i], &encoder.t_ns, &encoder.angle_rad));
            angles.push(angle[i]);
        }
    }
    if reads.len() < MIN_LAG_FRAMES {
        return (f64::NAN, f64::NAN);
    }
    let (rms, slope, _) = line(&reads, &angles);
    (rms, slope)
}

fn in_line(judged: (f64, f64)) -> bool {
    let (rms, slope) = judged;
    rms <= ALIGNED_RMS_RAD && ALIGNED_SCALE.0 <= slope && slope <= ALIGNED_SCALE.1
}

fn as_read_or_refused(encoder: &EncoderReads, as_read: (f64, f64)) -> UnfoldedEncoder {
    if in_line(as_read) {
        UnfoldedEncoder {
            reads: Some(encoder.clone()),
            law: EncoderLaw::AsRead,
            reflections: 0,
            inferred: 0,
            rms_rad: as_read.0,
        }
    } else {
        UnfoldedEncoder {
            reads: None,
            law: EncoderLaw::Refused,
            reflections: 0,
            inferred: 0,
            rms_rad: as_read.0,
        }
    }
}

fn fold_rate(t: &[i64], e: &[f64], before: (usize, usize), after: (usize, usize)) -> f64 {
    let rate = |lo: usize, hi: usize| -> f64 {
        if hi < lo + 2 {
            return 0.0;
        }
        (e[hi - 1] - e[lo]).abs() / 1e-9f64.max((t[hi - 1] - t[lo]) as f64 / 1e9)
    };
    let entering = rate(
        before.0.max(before.1.saturating_sub(REFLECT_WINDOW)),
        before.1,
    );
    let leaving = rate(after.0, after.1.min(after.0 + REFLECT_WINDOW));
    let crossing_s = (t[after.0] - t[before.1 - 1]) as f64 / 1e9;
    if crossing_s > 2.0 * FOLD_BAND_RAD / REFLECT_RATE_RAD_S + 0.05 {
        return 0.0;
    }
    entering.min(leaving)
}

/// The encoder's reads with the parser's folds undone, judged against the tags.
#[allow(clippy::too_many_lines)]
pub fn unfold_encoder(tags: &TagReads, encoder: &EncoderReads) -> UnfoldedEncoder {
    let as_read = aligned(tags, encoder);
    if !as_read.0.is_finite() || in_line(as_read) {
        return UnfoldedEncoder {
            reads: Some(encoder.clone()),
            law: EncoderLaw::AsRead,
            reflections: 0,
            inferred: 0,
            rms_rad: as_read.0,
        };
    }
    let t = &encoder.t_ns;
    let e = &encoder.angle_rad;
    let pi = std::f64::consts::PI;
    let away: Vec<bool> = e
        .iter()
        .map(|v| *v > FOLD_BAND_RAD && *v < pi - FOLD_BAND_RAD)
        .collect();
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut start = None;
    for (i, a) in away.iter().enumerate() {
        match (start, *a) {
            (None, true) => start = Some(i),
            (Some(s), false) => {
                runs.push((s, i));
                start = None;
            }
            _ => {}
        }
    }
    if let Some(s) = start {
        runs.push((s, away.len()));
    }
    if runs.is_empty() {
        return as_read_or_refused(encoder, as_read);
    }
    let (tag_t, tag_z, _) = tags.seen_series();
    let mut tag_at = vec![f64::NAN; t.len()];
    if tag_t.len() >= 2 {
        let close = near(t, &tag_t, TAG_REACH_NS);
        for i in 0..t.len() {
            if close[i] {
                tag_at[i] = interp_ns(t[i], &tag_t, &tag_z);
            }
        }
    }
    let folds: Vec<f64> = (1..runs.len())
        .map(|k| {
            let slice = &e[runs[k - 1].1 - 1..runs[k].0 + 1];
            if median(slice) < pi / 2.0 { 0.0 } else { pi }
        })
        .collect();
    let pi_folds = folds.contains(&pi);
    let tagged = |k: usize| -> Vec<usize> {
        let (a, b) = runs[k];
        (a..b).filter(|i| tag_at[*i].is_finite()).collect()
    };
    // 1. Where the jaw moved under the tags, its slope against the reads is its sign.
    let mut signs: Vec<Option<i32>> = Vec::with_capacity(runs.len());
    for k in 0..runs.len() {
        let both = tagged(k);
        let reads: Vec<f64> = both.iter().map(|i| e[*i]).collect();
        let moved = both.len() >= 10 && ptp(&reads) >= 4.0 * FOLD_BAND_RAD;
        signs.push(if moved {
            let at: Vec<f64> = both.iter().map(|i| tag_at[*i]).collect();
            Some(if line(&reads, &at).1 > 0.0 { 1 } else { -1 })
        } else {
            None
        });
    }
    let first = signs.iter().flatten().next().copied().unwrap_or(1);
    // 2. Held stretches by their level on the sloped line, unseen ones by the travel.
    let mut sloped = vec![false; e.len()];
    for (k, (a, b)) in runs.iter().enumerate() {
        for s in &mut sloped[*a..*b] {
            *s = signs[k].is_some();
        }
    }
    for (i, s) in sloped.iter_mut().enumerate() {
        *s &= tag_at[i].is_finite();
    }
    if sloped.iter().filter(|s| **s).count() >= MIN_LAG_FRAMES && !pi_folds {
        let mut raw_sloped = vec![0.0; e.len()];
        for (k, (a, b)) in runs.iter().enumerate() {
            let sign = f64::from(signs[k].unwrap_or(1));
            for i in *a..*b {
                raw_sloped[i] = sign * e[i];
            }
        }
        let xs: Vec<f64> = (0..e.len())
            .filter(|i| sloped[*i])
            .map(|i| raw_sloped[i])
            .collect();
        let ys: Vec<f64> = (0..e.len())
            .filter(|i| sloped[*i])
            .map(|i| tag_at[i])
            .collect();
        let (_, line_scale, offset) = line(&xs, &ys);
        let (low, high) = TAG_TRAVEL_RAD;
        for k in 0..runs.len() {
            if signs[k].is_some() {
                continue;
            }
            let (a, b) = runs[k];
            let both = tagged(k);
            if both.len() >= 10 {
                let miss: Vec<f64> = [1.0, -1.0]
                    .iter()
                    .map(|sign| {
                        let r: Vec<f64> = both
                            .iter()
                            .map(|i| (line_scale * sign * e[*i] + offset - tag_at[*i]).abs())
                            .collect();
                        median(&r)
                    })
                    .collect();
                if (miss[0] - miss[1]).abs() >= LEVEL_MARGIN_RAD {
                    signs[k] = Some(if miss[0] < miss[1] { 1 } else { -1 });
                }
                continue;
            }
            let inside: Vec<bool> = [1.0, -1.0]
                .iter()
                .map(|sign| {
                    (a..b).all(|i| {
                        let v = line_scale * sign * e[i] + offset;
                        v > low && v < high
                    })
                })
                .collect();
            if inside[0] != inside[1] {
                signs[k] = Some(if inside[0] { 1 } else { -1 });
            }
        }
    }
    // 3. Each fold reflects or not.
    let mut reflect = vec![false; folds.len()];
    let mut inferred_at: Vec<usize> = Vec::new();
    let known: Vec<usize> = (0..signs.len()).filter(|k| signs[*k].is_some()).collect();
    for i in 1..runs.len() {
        if let (Some(a), Some(b)) = (signs[i - 1], signs[i]) {
            reflect[i - 1] = a != b;
        }
    }
    let mut edges: Vec<isize> = vec![-1];
    edges.extend(known.iter().map(|k| *k as isize));
    edges.push(runs.len() as isize);
    for pair in edges.windows(2) {
        let (left, right) = (pair[0], pair[1]);
        let lo = left.max(0) as usize;
        let hi = right.min(runs.len() as isize - 1).max(0) as usize;
        let between: Vec<usize> = (lo..hi)
            .filter(|f| !(signs[*f].is_some() && signs[*f + 1].is_some()))
            .collect();
        if between.is_empty() {
            continue;
        }
        let rates: Vec<f64> = between
            .iter()
            .map(|f| fold_rate(t, e, runs[*f], runs[*f + 1]))
            .collect();
        for (f, rate) in between.iter().zip(&rates) {
            reflect[*f] = *rate >= REFLECT_RATE_RAD_S;
            if !inferred_at.contains(f) {
                inferred_at.push(*f);
            }
        }
        if left >= 0 && (right as usize) < runs.len() {
            let odd = signs[left as usize] != signs[right as usize];
            let count = between.iter().filter(|f| reflect[**f]).count();
            if count % 2 != usize::from(odd) {
                let mut weakest = between[0];
                let mut weakest_key = f64::INFINITY;
                for (f, rate) in between.iter().zip(&rates) {
                    let key = (rate.max(1e-6) / REFLECT_RATE_RAD_S).ln().abs();
                    if key < weakest_key {
                        weakest_key = key;
                        weakest = *f;
                    }
                }
                reflect[weakest] = !reflect[weakest];
            }
        }
    }
    let mut sigma = f64::from(signs[0].unwrap_or(first));
    let mut turns = 0.0;
    let mut raw = vec![0.0; e.len()];
    let mut bounds = vec![0usize];
    for k in 1..runs.len() {
        let (lo, hi) = (runs[k - 1].1, runs[k].0);
        if hi <= lo {
            bounds.push(hi);
            continue;
        }
        let gap = &e[lo..hi];
        let mut at = 0;
        for (j, v) in gap.iter().enumerate() {
            let better = if folds[k - 1] == 0.0 {
                *v < gap[at]
            } else {
                *v > gap[at]
            };
            if better {
                at = j;
            }
        }
        bounds.push(lo + at);
    }
    bounds.push(e.len());
    for k in 0..runs.len() {
        if k > 0 && reflect[k - 1] {
            if folds[k - 1] == pi {
                turns += sigma;
            }
            sigma = -sigma;
        }
        for i in bounds[k]..bounds[k + 1] {
            raw[i] = sigma * e[i] + 2.0 * pi * turns;
        }
    }
    let reflections = reflect.iter().filter(|r| **r).count();
    let inferred = inferred_at.iter().filter(|f| reflect[**f]).count();
    let mut unfolded = EncoderReads {
        t_ns: t.clone(),
        angle_rad: raw.clone(),
    };
    let mut judged = aligned(tags, &unfolded);
    if judged.1 < 0.0 {
        unfolded = EncoderReads {
            t_ns: t.clone(),
            angle_rad: raw.iter().map(|v| -v).collect(),
        };
        judged = aligned(tags, &unfolded);
    }
    if in_line(judged) {
        return UnfoldedEncoder {
            reads: Some(unfolded),
            law: EncoderLaw::Unfolded,
            reflections,
            inferred,
            rms_rad: judged.0,
        };
    }
    as_read_or_refused(encoder, as_read)
}

/// A take's fused jaw angle at the requested times.
#[derive(Clone, Debug, PartialEq)]
pub struct FusedJaw {
    pub t_ns: Vec<i64>,
    pub angle_rad: Vec<f64>,
    pub std_rad: Vec<f64>,
    pub sensors: Vec<JawSensors>,
    pub encoder_bias_rad: Vec<f64>,
    pub encoder_scale: f64,
    pub encoder_offset_rad: f64,
    pub camera_lag_ns: i64,
    pub encoder_trusted: f64,
    pub tags_trusted: f64,
}

impl FusedJaw {
    /// The longest run of samples whose deviation exceeds `bound_rad`, in seconds.
    pub fn longest_unsupported_s(&self, bound_rad: f64) -> f64 {
        let over: Vec<bool> = self.std_rad.iter().map(|s| *s > bound_rad).collect();
        let mut longest = 0.0f64;
        let mut start: Option<usize> = None;
        let n = over.len();
        let spacing = if n > 1 {
            let diffs: Vec<f64> = self.t_ns.windows(2).map(|w| (w[1] - w[0]) as f64).collect();
            median(&diffs) / 1e9
        } else {
            0.0
        };
        for (i, flag) in over.iter().enumerate() {
            if *flag && start.is_none() {
                start = Some(i);
            }
            if let Some(s) = start
                && (!*flag || i == n - 1)
            {
                let end = if *flag { i } else { i - 1 };
                let mut span = (self.t_ns[end] - self.t_ns[s]) as f64 / 1e9;
                if end == s && n > 1 {
                    span = spacing;
                }
                longest = longest.max(span);
                start = None;
            }
        }
        longest
    }
}

const ENCODER: u8 = 0;
const TAG: u8 = 1;
const OUTPUT: u8 = 2;

/// The jaw angle at `at_ns` (on the encoder's clock) from whichever sensors saw it; `lag_ns`
/// is how far the tag stamps trail the encoder's.
#[allow(clippy::too_many_lines)]
pub fn fuse_jaw(
    at_ns: &[i64],
    encoder: Option<&EncoderReads>,
    tags: Option<&TagReads>,
    lag_ns: i64,
    model: &FusionModel,
) -> Result<FusedJaw, String> {
    if at_ns.is_empty() || at_ns.windows(2).any(|w| w[1] <= w[0]) {
        return Err("the fused jaw needs strictly increasing sample times".to_owned());
    }
    let (enc_t, enc_z): (Vec<i64>, Vec<f64>) = match encoder {
        Some(e) => (e.t_ns.clone(), e.angle_rad.clone()),
        None => (Vec::new(), Vec::new()),
    };
    let (tag_t, tag_z, tag_s): (Vec<i64>, Vec<f64>, Vec<f64>) = match tags {
        Some(tags) => {
            let (t, z, s) = tags.seen_series();
            (t.iter().map(|v| v - lag_ns).collect(), z, s)
        }
        None => (Vec::new(), Vec::new(), Vec::new()),
    };
    if enc_t.len() + tag_t.len() == 0 {
        return Err("neither the encoder nor the tags read the jaw".to_owned());
    }
    // The merged timeline, ordered by time then kind (encoder, tag, output), stably.
    let total = enc_t.len() + tag_t.len() + at_ns.len();
    let mut entries: Vec<(i64, u8, usize)> = Vec::with_capacity(total);
    entries.extend(enc_t.iter().enumerate().map(|(i, t)| (*t, ENCODER, i)));
    entries.extend(tag_t.iter().enumerate().map(|(i, t)| (*t, TAG, i)));
    entries.extend(at_ns.iter().enumerate().map(|(i, t)| (*t, OUTPUT, i)));
    entries.sort_by_key(|(t, kind, _)| (*t, *kind));
    let times: Vec<i64> = entries.iter().map(|e| e.0).collect();
    let kinds: Vec<u8> = entries.iter().map(|e| e.1).collect();
    let origin: Vec<usize> = entries.iter().map(|e| e.2).collect();
    let (scale_e, offset_e) = encoder_line(&enc_t, &enc_z, &tag_t, &tag_z);
    let mut values = Vec::with_capacity(total);
    let mut stds = Vec::with_capacity(total);
    for (kind, index) in kinds.iter().zip(&origin) {
        match *kind {
            ENCODER => {
                values.push(scale_e * enc_z[*index] + offset_e);
                stds.push(scale_e * model.encoder_std_rad);
            }
            TAG => {
                values.push(tag_z[*index]);
                stds.push(tag_s[*index]);
            }
            _ => {
                values.push(0.0);
                stds.push(1.0);
            }
        }
    }
    let enc_mapped_t: Vec<i64> = (0..total)
        .filter(|k| kinds[*k] == ENCODER)
        .map(|k| times[k])
        .collect();
    let enc_mapped_z: Vec<f64> = (0..total)
        .filter(|k| kinds[*k] == ENCODER)
        .map(|k| values[k])
        .collect();
    let bias0 = starting_bias(&enc_mapped_t, &enc_mapped_z, &tag_t, &tag_z);
    let first_measured = (0..total).find(|k| kinds[*k] != OUTPUT).unwrap_or(0);
    let first = values[first_measured]
        - if kinds[first_measured] == ENCODER {
            bias0
        } else {
            0.0
        };
    let t0 = times[0];
    let seconds: Vec<f64> = times.iter().map(|t| (t - t0) as f64 / 1e9).collect();
    let mut steps = vec![0.0; total];
    for k in 1..total {
        steps[k] = seconds[k] - seconds[k - 1];
    }
    let mut weights: Vec<f64> = vec![1.0; total];
    let observed = if tag_t.is_empty() { 0.0 } else { 1.0 };
    let mut walk: Vec<f64> = steps
        .iter()
        .map(|s| observed * model.bias_free_rad.powi(2) * s)
        .collect();
    let mut scaled = vec![0.0; total];
    let mut smoother = Smoother::new(total);
    for scale in REWEIGHT_SCALES {
        for k in 0..total {
            scaled[k] = scale * stds[k] / weights[k].sqrt();
        }
        smoother.run(&steps, &kinds, &values, &scaled, first, bias0, &walk, model);
        let state = &smoother.x_s;
        for k in 0..total {
            if kinds[k] == OUTPUT {
                weights[k] = 1.0;
                continue;
            }
            let predicted = state[k][0]
                + if kinds[k] == ENCODER {
                    state[k][2]
                } else {
                    0.0
                };
            let normalized = (values[k] - predicted) / (scale * stds[k]);
            weights[k] = (STUDENT_NU + 1.0) / (STUDENT_NU + normalized * normalized);
        }
        for k in 0..total {
            let step = if k == 0 {
                0.0
            } else {
                state[k][2] - state[k - 1][2]
            };
            walk[k] = observed * ((scale * model.bias_walk_rad).powi(2) * steps[k] + step * step);
        }
    }
    for k in 0..total {
        scaled[k] = stds[k] / weights[k].sqrt();
    }
    smoother.run(&steps, &kinds, &values, &scaled, first, bias0, &walk, model);
    let state = &smoother.x_s;
    let cov = &smoother.p_s;
    let n_out = at_ns.len();
    let mut angle = vec![0.0; n_out];
    let mut std = vec![0.0; n_out];
    let mut bias = vec![0.0; n_out];
    for k in 0..total {
        if kinds[k] == OUTPUT {
            let i = origin[k];
            angle[i] = state[k][0];
            std[i] = cov[k][0][0].max(0.0).sqrt();
            bias[i] = (state[k][0] + state[k][2] - offset_e) / scale_e - state[k][0];
        }
    }
    let trusted: Vec<bool> = (0..total)
        .map(|k| kinds[k] != OUTPUT && weights[k] >= TRUSTED_WEIGHT)
        .collect();
    if !trusted.iter().any(|t| *t) {
        return Err("the fused jaw trusted none of its readings".to_owned());
    }
    let mut enc_ok = vec![false; enc_t.len()];
    let mut tag_ok = vec![false; tag_t.len()];
    let (mut enc_count, mut enc_trusted, mut tag_count, mut tag_trusted) =
        (0usize, 0usize, 0usize, 0usize);
    for k in 0..total {
        match kinds[k] {
            ENCODER => {
                enc_count += 1;
                if trusted[k] {
                    enc_trusted += 1;
                    enc_ok[origin[k]] = true;
                }
            }
            TAG => {
                tag_count += 1;
                if trusted[k] {
                    tag_trusted += 1;
                    tag_ok[origin[k]] = true;
                }
            }
            _ => {}
        }
    }
    let ok_enc_t: Vec<i64> = enc_t
        .iter()
        .zip(&enc_ok)
        .filter(|(_, ok)| **ok)
        .map(|(t, _)| *t)
        .collect();
    let ok_tag_t: Vec<i64> = tag_t
        .iter()
        .zip(&tag_ok)
        .filter(|(_, ok)| **ok)
        .map(|(t, _)| *t)
        .collect();
    let by_encoder = near(at_ns, &ok_enc_t, ENCODER_REACH_NS);
    let by_tags = near(at_ns, &ok_tag_t, TAG_REACH_NS);
    let sensors: Vec<JawSensors> = by_encoder
        .iter()
        .zip(&by_tags)
        .map(|(e, t)| match (*e, *t) {
            (true, true) => JawSensors::EncoderAndTags,
            (true, false) => JawSensors::Encoder,
            (false, true) => JawSensors::Tags,
            (false, false) => JawSensors::Neither,
        })
        .collect();
    if sensors.contains(&JawSensors::Neither) {
        let xp: Vec<f64> = (0..total)
            .filter(|k| trusted[*k])
            .map(|k| times[k] as f64)
            .collect();
        let fp: Vec<f64> = (0..total)
            .filter(|k| trusted[*k])
            .map(|k| state[k][0])
            .collect();
        for i in 0..n_out {
            if sensors[i] == JawSensors::Neither {
                angle[i] = interp(at_ns[i] as f64, &xp, &fp);
            }
        }
    }
    Ok(FusedJaw {
        t_ns: at_ns.to_vec(),
        angle_rad: angle,
        std_rad: std,
        sensors,
        encoder_bias_rad: bias,
        encoder_scale: scale_e,
        encoder_offset_rad: offset_e,
        camera_lag_ns: lag_ns,
        encoder_trusted: if enc_count > 0 {
            enc_trusted as f64 / enc_count as f64
        } else {
            0.0
        },
        tags_trusted: if tag_count > 0 {
            tag_trusted as f64 / tag_count as f64
        } else {
            0.0
        },
    })
}

fn overlap(
    enc_t: &[i64],
    enc_z: &[f64],
    tag_t: &[i64],
    tag_z: &[f64],
) -> Option<(Vec<f64>, Vec<f64>)> {
    if enc_t.len() < 2 || tag_t.is_empty() {
        return None;
    }
    let close = near(tag_t, enc_t, ENCODER_REACH_NS);
    let (first, last) = (enc_t[0], enc_t[enc_t.len() - 1]);
    let mut reads = Vec::new();
    let mut angles = Vec::new();
    for i in 0..tag_t.len() {
        if close[i] && tag_t[i] >= first && tag_t[i] <= last {
            reads.push(interp_ns(tag_t[i], enc_t, enc_z));
            angles.push(tag_z[i]);
        }
    }
    (reads.len() >= MIN_LAG_FRAMES).then_some((reads, angles))
}

/// The take's line from the encoder's reads onto the tags' angle (a reweighted
/// least-trimmed-squares line), or 1 and 0.
fn encoder_line(enc_t: &[i64], enc_z: &[f64], tag_t: &[i64], tag_z: &[f64]) -> (f64, f64) {
    let Some((reads, angles)) = overlap(enc_t, enc_z, tag_t, tag_z) else {
        return (1.0, 0.0);
    };
    let (slope, offset) = trimmed_line(&reads, &angles);
    if !(ALIGNED_SCALE.0..=ALIGNED_SCALE.1).contains(&slope) {
        return (1.0, 0.0);
    }
    (slope, offset)
}

fn fit_rows(read: &[f64], angle: &[f64], rows: &[usize]) -> (f64, f64) {
    let x: Vec<f64> = rows.iter().map(|i| read[*i]).collect();
    let y: Vec<f64> = rows.iter().map(|i| angle[*i]).collect();
    least_squares_line(&x, &y)
}

/// The least-trimmed-squares line through `(read, angle)` over half the points, from 64
/// deterministic starting pairs, then refitted through every point within 2.5 robust
/// deviations of it.
pub fn trimmed_line(read: &[f64], angle: &[f64]) -> (f64, f64) {
    let count = read.len();
    let keep = 2.max((TRIMMED_SHARE * count as f64) as usize);
    let order = argsort_by(read);
    let mut best = (1.0, 0.0);
    let mut best_cost = f64::INFINITY;
    let half = count / 2;
    let residuals = |fit: (f64, f64)| -> Vec<f64> {
        read.iter()
            .zip(angle)
            .map(|(r, a)| a - (fit.0 * r + fit.1))
            .collect()
    };
    for i in 0..TRIMMED_STARTS {
        let lo = order[(i * 7919) % 1.max(half)];
        let hi = order[half + (i * 104_729) % 1.max(count - half)];
        if read[hi] - read[lo] < 1e-6 {
            continue;
        }
        let mut fit = fit_rows(read, angle, &[lo, hi]);
        for _ in 0..TRIMMED_STEPS {
            let absolute: Vec<f64> = residuals(fit).iter().map(|r| r.abs()).collect();
            let kept: Vec<usize> = argsort_by(&absolute).into_iter().take(keep).collect();
            let refit = fit_rows(read, angle, &kept);
            let close = |a: f64, b: f64| (a - b).abs() <= 1e-9 + 1e-5 * b.abs();
            if close(refit.0, fit.0) && close(refit.1, fit.1) {
                break;
            }
            fit = refit;
        }
        let mut squares: Vec<f64> = residuals(fit).iter().map(|r| r * r).collect();
        squares.sort_by(f64::total_cmp);
        let cost: f64 = squares[..keep.min(squares.len())].iter().sum();
        if cost < best_cost {
            best = fit;
            best_cost = cost;
        }
    }
    let residual = residuals(best);
    let absolute: Vec<f64> = residual.iter().map(|r| r.abs()).collect();
    let spread = 1.4826 * median(&absolute) + 1e-9;
    let inside: Vec<usize> = (0..count)
        .filter(|i| absolute[*i] <= 2.5 * spread)
        .collect();
    if inside.len() >= 2 {
        best = fit_rows(read, angle, &inside);
    }
    best
}

fn starting_bias(enc_t: &[i64], enc_z: &[f64], tag_t: &[i64], tag_z: &[f64]) -> f64 {
    match overlap(enc_t, enc_z, tag_t, tag_z) {
        Some((reads, angles)) => {
            let d: Vec<f64> = reads.iter().zip(&angles).map(|(r, a)| r - a).collect();
            median(&d)
        }
        None => 0.0,
    }
}

type State = [f64; 3];

/// The Kalman filter forward and Rauch-Tung-Striebel pass back, one scalar read per epoch;
/// its buffers are kept across the reweighting rounds.
struct Smoother {
    x_pred: Vec<State>,
    p_pred: Vec<Mat3>,
    x_filt: Vec<State>,
    p_filt: Vec<Mat3>,
    x_s: Vec<State>,
    p_s: Vec<Mat3>,
}

impl Smoother {
    fn new(count: usize) -> Self {
        Self {
            x_pred: vec![[0.0; 3]; count],
            p_pred: vec![[[0.0; 3]; 3]; count],
            x_filt: vec![[0.0; 3]; count],
            p_filt: vec![[[0.0; 3]; 3]; count],
            x_s: vec![[0.0; 3]; count],
            p_s: vec![[[0.0; 3]; 3]; count],
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn run(
        &mut self,
        steps: &[f64],
        kinds: &[u8],
        values: &[f64],
        stds: &[f64],
        first_angle: f64,
        first_bias: f64,
        walk: &[f64],
        model: &FusionModel,
    ) {
        use crate::geometry::{inverse, matmul, transpose};
        let count = steps.len();
        let q_acc = model.acceleration_rad_s2 * model.acceleration_rad_s2;
        let mut x = [first_angle, 0.0, first_bias];
        let mut p: Mat3 = [
            [0.25, 0.0, 0.0],
            [0.0, 4.0, 0.0],
            [0.0, 0.0, model.bias_prior_rad * model.bias_prior_rad],
        ];
        for k in 0..count {
            let dt = steps[k];
            if dt > 0.0 || walk[k] > 0.0 {
                x = [x[0] + dt * x[1], x[1], x[2]];
                let f = [[1.0, dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
                let q = [
                    [q_acc * dt.powi(3) / 3.0, q_acc * dt.powi(2) / 2.0, 0.0],
                    [q_acc * dt.powi(2) / 2.0, q_acc * dt, 0.0],
                    [0.0, 0.0, walk[k]],
                ];
                let fp = matmul(&matmul(&f, &p), &transpose(&f));
                for i in 0..3 {
                    for j in 0..3 {
                        p[i][j] = fp[i][j] + q[i][j];
                    }
                }
            }
            self.x_pred[k] = x;
            self.p_pred[k] = p;
            if kinds[k] != OUTPUT {
                let h = [1.0, 0.0, if kinds[k] == ENCODER { 1.0 } else { 0.0 }];
                let ph = [
                    p[0][0] * h[0] + p[0][1] * h[1] + p[0][2] * h[2],
                    p[1][0] * h[0] + p[1][1] * h[1] + p[1][2] * h[2],
                    p[2][0] * h[0] + p[2][1] * h[1] + p[2][2] * h[2],
                ];
                let denominator = h[0] * ph[0] + h[1] * ph[1] + h[2] * ph[2] + stds[k] * stds[k];
                let gain = [
                    ph[0] / denominator,
                    ph[1] / denominator,
                    ph[2] / denominator,
                ];
                let innovation = values[k] - (h[0] * x[0] + h[1] * x[1] + h[2] * x[2]);
                x = [
                    x[0] + gain[0] * innovation,
                    x[1] + gain[1] * innovation,
                    x[2] + gain[2] * innovation,
                ];
                for i in 0..3 {
                    for j in 0..3 {
                        p[i][j] -= gain[i] * ph[j];
                    }
                }
                let symmetric = p;
                for i in 0..3 {
                    for j in 0..3 {
                        p[i][j] = (symmetric[i][j] + symmetric[j][i]) / 2.0;
                    }
                }
            }
            self.x_filt[k] = x;
            self.p_filt[k] = p;
        }
        self.x_s.copy_from_slice(&self.x_filt);
        self.p_s.copy_from_slice(&self.p_filt);
        for k in (0..count.saturating_sub(1)).rev() {
            let dt = steps[k + 1];
            let f = [[1.0, dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
            let inv = inverse(&self.p_pred[k + 1]).unwrap_or([[0.0; 3]; 3]);
            let gain = matmul(&matmul(&self.p_filt[k], &transpose(&f)), &inv);
            let dx = [
                self.x_s[k + 1][0] - self.x_pred[k + 1][0],
                self.x_s[k + 1][1] - self.x_pred[k + 1][1],
                self.x_s[k + 1][2] - self.x_pred[k + 1][2],
            ];
            for i in 0..3 {
                self.x_s[k][i] = self.x_filt[k][i]
                    + gain[i][0] * dx[0]
                    + gain[i][1] * dx[1]
                    + gain[i][2] * dx[2];
            }
            let mut dp = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    dp[i][j] = self.p_s[k + 1][i][j] - self.p_pred[k + 1][i][j];
                }
            }
            let correction = matmul(&matmul(&gain, &dp), &transpose(&gain));
            for i in 0..3 {
                for j in 0..3 {
                    self.p_s[k][i][j] = self.p_filt[k][i][j] + correction[i][j];
                }
            }
        }
    }
}

/// A take's jaw at its frames' stamps, on the encoder's clock: its tags fused with whatever its
/// encoder read, the encoder's folds undone and the camera's lag measured first.
pub fn solve_jaw(
    reference: &JawReference,
    t_ns: &[i64],
    frames: &[Frame],
    encoder: Option<&EncoderReads>,
    model: &FusionModel,
) -> Result<(FusedJaw, TagReads), String> {
    let tags = tag_reads(reference, t_ns, frames, model)?;
    let unfolded = match encoder {
        Some(e) if !e.is_empty() => unfold_encoder(&tags, e).reads,
        _ => None,
    };
    let lag = unfolded.as_ref().and_then(|e| camera_lag_ns(&tags, e));
    let fused = fuse_jaw(
        &tags.t_ns,
        unfolded.as_ref(),
        Some(&tags),
        lag.unwrap_or(0),
        model,
    )?;
    Ok((fused, tags))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_trimmed_line_ignores_a_squeezed_third() {
        let read: Vec<f64> = (0..300).map(|i| f64::from(i) / 300.0).collect();
        let angle: Vec<f64> = read
            .iter()
            .enumerate()
            .map(|(i, r)| {
                if i % 3 == 0 {
                    0.8 * r + 0.06 - 0.1
                } else {
                    0.8 * r + 0.06
                }
            })
            .collect();
        let (slope, offset) = trimmed_line(&read, &angle);
        assert!(
            (slope - 0.8).abs() < 1e-6 && (offset - 0.06).abs() < 1e-6,
            "{slope} {offset}"
        );
    }

    #[test]
    fn the_encoder_alone_follows_its_reads() {
        let t: Vec<i64> = (0..300).map(|i| i * 10_000_000).collect();
        let a: Vec<f64> = t
            .iter()
            .map(|t| 0.3 + 0.2 * (*t as f64 / 1e9).sin())
            .collect();
        let encoder = EncoderReads::new(t.clone(), a.clone()).unwrap();
        let fused = fuse_jaw(&t, Some(&encoder), None, 0, &FusionModel::default()).unwrap();
        for (f, truth) in fused.angle_rad.iter().zip(&a) {
            assert!((f - truth).abs() < 0.003);
        }
        assert!(fused.sensors.iter().all(|s| *s == JawSensors::Encoder));
    }
}
