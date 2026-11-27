//! One side of one take solved from its sightings: the markers' poses through the lens, the
//! reference, the fused track, its QC numbers, and the spool rows the seal reads
//! (`t_monotonic_ns,angle_rad,angle_std_rad,sensors`), formerly `sx._jaw_solve`'s middle.

use crate::detect::Marker;
use crate::fusion::{
    BLIND_STD_RAD, EncoderReads, FusedJaw, FusionModel, JawSensors, TagReads, camera_lag_ns,
    fuse_jaw, solve_jaw, unfold_encoder,
};
use crate::geometry::{interp_ns, median, percentile, search_sorted};
use crate::lens::Lens;
use crate::sightings::Sighting;
use crate::tags::{
    Frame, JawReference, Layout, MAX_MIRROR_DISAGREEMENT_RAD, NO_TAGS_PROBE_FRAMES,
    closed_stop_reference, fit_reference, sight,
};
use crate::unit::JawUnit;

/// The solver as every solved jaw names it (`jaw_solve.json`, the spool's binding and the
/// sealed `/gripper/{side}/jaw_source`). Raise the version whenever a change here can move a
/// solved angle: delivery re-solves what an older version solved.
pub const SOLVER_NAME: &str = "sx-jaw-tags";
pub const SOLVER_VERSION: u32 = 2;

/// An encoder angle counts for a reference frame only within this of a real read.
pub const REFERENCE_ENCODER_REACH_NS: i64 = 25_000_000;

/// Why a side's jaw was not solved from its tags (`sx._jaw_solve.JawSolveRefusal`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    NoTags,
    NoReference,
    OutsideEnvelope,
    MirrorDisagreement,
}

impl Refusal {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::NoTags => "no_tags",
            Self::NoReference => "no_reference",
            Self::OutsideEnvelope => "outside_envelope",
            Self::MirrorDisagreement => "mirror_disagreement",
        }
    }
}

pub type Refused = (Refusal, String);

/// Each kept frame's sighted markers located through the lens; a frame not analysed, or
/// with none, is empty. A sighting whose stamp is not a kept frame's is ignored.
pub fn frames_at(
    frame_t_ns: &[i64],
    sightings: &[Sighting],
    lens: &Lens,
    layout: &Layout,
) -> Vec<Frame> {
    let mut frames = vec![Frame::new(); frame_t_ns.len()];
    for sighting in sightings {
        let index = search_sorted(frame_t_ns, sighting.t_ns);
        if frame_t_ns.get(index) == Some(&sighting.t_ns) {
            frames[index] = sight(&sighting.markers, lens, layout);
        }
    }
    frames
}

/// How many of the kept frames were analysed, and how many showed a finger marker.
pub fn coverage(frame_t_ns: &[i64], sightings: &[Sighting], layout: &Layout) -> (usize, usize) {
    let mut analysed = 0;
    let mut sighted = 0;
    for sighting in sightings {
        let index = search_sorted(frame_t_ns, sighting.t_ns);
        if frame_t_ns.get(index) == Some(&sighting.t_ns) {
            analysed += 1;
            if sighting
                .markers
                .iter()
                .any(|m: &Marker| layout.ids().contains(&m.id))
            {
                sighted += 1;
            }
        }
    }
    (analysed, sighted)
}

/// Whether the take's opening frames (its first 300) show a finger marker at all.
pub fn has_jaw_tags(frames: &[Frame]) -> bool {
    frames
        .iter()
        .take(NO_TAGS_PROBE_FRAMES)
        .any(|frame| !frame.is_empty())
}

/// The encoder angle at each frame, NaN where no read lies within reach.
pub fn encoder_at(frame_t_ns: &[i64], encoder: &EncoderReads) -> Vec<f64> {
    let t = &encoder.t_ns;
    if t.is_empty() {
        return vec![f64::NAN; frame_t_ns.len()];
    }
    let n = t.len();
    frame_t_ns
        .iter()
        .map(|f| {
            let index = search_sorted(t, *f);
            let nearest = (t[index.min(n - 1)] - f).abs();
            let before = (t[index.saturating_sub(1).min(n - 1)] - f).abs();
            if nearest.min(before) <= REFERENCE_ENCODER_REACH_NS {
                interp_ns(*f, t, &encoder.angle_rad)
            } else {
                f64::NAN
            }
        })
        .collect()
}

/// The markers' references fitted on a take of this unit whose encoder was alive.
pub fn history_reference(
    layout: &Layout,
    frames: &[Frame],
    frame_t_ns: &[i64],
    encoder: &EncoderReads,
) -> Option<JawReference> {
    fit_reference(layout, frames, &encoder_at(frame_t_ns, encoder))
}

/// The take's own closed stop, when it reached closed.
pub fn own_reference(layout: &Layout, frames: &[Frame]) -> Option<JawReference> {
    closed_stop_reference(layout, frames, 0.0)
}

/// A side solved, with what `jaw_solve.json` records of it.
#[derive(Clone, Debug)]
pub struct Solved {
    pub fused: FusedJaw,
    pub tags: TagReads,
    pub encoder_reads: usize,
    pub longest_blind_s: f64,
    pub angle_std_p95_rad: f64,
    pub mirror_p95_rad: Option<f64>,
    pub encoder_bias_rad: Option<f64>,
    /// The spool's rows after its header, clamped to the envelope.
    pub rows: String,
    pub clamped: usize,
}

impl Solved {
    pub fn sensors(&self) -> Vec<(JawSensors, usize)> {
        JawSensors::ALL
            .iter()
            .map(|s| (*s, self.fused.sensors.iter().filter(|x| *x == s).count()))
            .filter(|(_, n)| *n > 0)
            .collect()
    }
}

/// The reference the solve uses: a fixed one (the take's closed stop, a history take's
/// fit), or the unit's registered calibration.
pub enum Reference<'a> {
    Markers(&'a JawReference),
    Unit(&'a JawUnit),
}

/// `frames` (one per kept frame at `frame_t_ns`) fused with `encoder` under `reference`, judged
/// and written as the seal reads it within the jaw law's `envelope_rad`.
pub fn solve(
    frame_t_ns: &[i64],
    frames: &[Frame],
    encoder: Option<&EncoderReads>,
    reference: Reference,
    envelope_rad: (f64, f64),
    model: &FusionModel,
) -> Result<Solved, Refused> {
    let fault = |e: String| (Refusal::NoTags, e);
    let (fused, tags) = match reference {
        Reference::Markers(reference) => {
            solve_jaw(reference, frame_t_ns, frames, encoder, model).map_err(fault)?
        }
        Reference::Unit(unit) => {
            let tags = unit.tags(frame_t_ns, frames, model).map_err(fault)?;
            let unfolded = encoder
                .filter(|e| !e.is_empty())
                .and_then(|e| unfold_encoder(&tags, e).reads);
            let reads = unfolded.map(|e| unit.map_encoder(&e));
            let lag = reads.as_ref().and_then(|r| camera_lag_ns(&tags, r));
            let mut fused = fuse_jaw(
                &tags.t_ns,
                reads.as_ref(),
                Some(&tags),
                lag.unwrap_or(0),
                model,
            )
            .map_err(fault)?;
            fused.angle_rad = unit.delivered_angle(&fused);
            (fused, tags)
        }
    };
    let mirror = tags.mirror_p95_rad();
    if mirror.is_finite() && mirror > MAX_MIRROR_DISAGREEMENT_RAD {
        return Err((
            Refusal::MirrorDisagreement,
            format!(
                "the finger markers disagree by {:.1} degrees at p95, past {:.0}: check the gripper",
                mirror.to_degrees(),
                MAX_MIRROR_DISAGREEMENT_RAD.to_degrees()
            ),
        ));
    }
    let (rows, clamped) =
        spool_rows(&fused, envelope_rad).map_err(|e| (Refusal::OutsideEnvelope, e))?;
    let bias = encoder
        .filter(|e| !e.is_empty())
        .map(|_| median(&fused.encoder_bias_rad));
    Ok(Solved {
        longest_blind_s: fused.longest_unsupported_s(BLIND_STD_RAD),
        angle_std_p95_rad: percentile(&fused.std_rad, 95.0),
        mirror_p95_rad: mirror.is_finite().then_some(mirror),
        encoder_bias_rad: bias,
        encoder_reads: encoder.map_or(0, EncoderReads::len),
        fused,
        tags,
        rows,
        clamped,
    })
}

/// The fused track as the seal reads it, clamped to the envelope; refused when it leaves the
/// envelope by more than its own deviation and the mirror bound.
pub fn spool_rows(fused: &FusedJaw, envelope_rad: (f64, f64)) -> Result<(String, usize), String> {
    use std::fmt::Write as _;
    let (low, high) = envelope_rad;
    for (angle, std) in fused.angle_rad.iter().zip(&fused.std_rad) {
        let slack = MAX_MIRROR_DISAGREEMENT_RAD + 3.0 * std;
        if *angle < low - slack || *angle > high + slack {
            return Err(
                "the solved jaw leaves the jaw law's envelope; check the reference".to_owned(),
            );
        }
    }
    let mut rows = String::with_capacity(fused.t_ns.len() * 48);
    let mut clamped = 0;
    for i in 0..fused.t_ns.len() {
        let angle = fused.angle_rad[i].clamp(low, high);
        if angle != fused.angle_rad[i] {
            clamped += 1;
        }
        let _ = writeln!(
            rows,
            "{},{:.6},{:.6},{}",
            fused.t_ns[i],
            angle,
            fused.std_rad[i],
            fused.sensors[i].as_str()
        );
    }
    Ok((rows, clamped))
}
