"""One jaw angle from the YUBI gripper's two independent sensors: its encoder and its wrist tags.

The jaw encoder (AS5601, about 100 Hz) reads the finger joint finely but has its own faults:
it drops reads, its zero can be off or drift, and it can fluctuate faster than any finger or
actuator could move. The wrist camera's two ArUco tags (:mod:`sx_embodiments.jaw_tags`, about
30 fps) read the same joint absolutely, with 1 to 2 mm of gap noise per frame, and are
sometimes hidden. Neither is the truth; each covers the other's faults.

This is one offline estimator over a whole take and one side, with no heuristics:

- State: the jaw angle, its rate, and the encoder's bias (its zero offset, a slow random walk
  so a drifting zero is followed).
- Process: white acceleration of :attr:`FusionModel.acceleration_rad_s2` per root second, the
  physical bound on how fast a finger changes speed. A reading the motion cannot follow (a
  spike train, a wrong frame) leaves a large residual and loses its weight.
- Measurements: every encoder read is ``angle + bias``; every tag frame is ``angle`` at its
  exposure less the camera's lag behind the encoder (:func:`camera_lag_ns`), with its own
  standard deviation from how many tags it saw and how well they reprojected.
- Solver: a Rauch-Tung-Striebel smoother, iteratively reweighted with Student-t weights
  (:data:`STUDENT_NU`). A dropout in either sensor is only missing measurements, and no
  reading is ever gated away outright, so the smoother cannot lock onto its own mistake the
  way a gated causal filter did (2026-11-25: p95 103 mm).
- Output, per sample: the angle, its posterior standard deviation and which sensors carried
  it (:class:`JawSensors`). Where neither did, the angle is linear between the trusted
  readings either side and held at the take's ends: the smoother never invents motion
  there. QC refuses only where the deviation stays above a bound for longer than a duration
  (:meth:`FusedJaw.longest_unsupported_s`): neither sensor saw the jaw.

A dead encoder leaves the tags alone (the bias is then unobservable and irrelevant); a stretch
without tags rides the encoder and the bias learned elsewhere; with tags referenced to the
closed stop, the absolute zero is the tags' and a mis-zeroed encoder is corrected.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from .jaw_tags import (
    MAX_MIRROR_DISAGREEMENT_RAD,
    JawReference,
    JawTagError,
    TagPose,
    consistent_frames,
    read_frame,
)

type Vector = NDArray[np.float64]
type Stamps = NDArray[np.int64]

#: Student-t degrees of freedom of the reweighting (1, a Cauchy loss). With 4, five 0.003 rad
#: encoder reads 0.2 rad off cost less than the motion it takes to reach them, so a 50 ms
#: excursion was followed; under a Cauchy loss each costs about 8 nats, and it is not.
STUDENT_NU = 1.0
#: The reweighting rounds' noise scales, coarse to fine (graduated non-convexity). The first
#: fit is smooth, so a burst of encoder reads the motion cannot follow stands out from it and
#: loses its weight; starting at the readings' own noise instead, the first fit followed a
#: 50 ms, 0.2 rad excursion and every later round kept it.
REWEIGHT_SCALES = (16.0, 8.0, 4.0, 2.0, 1.0, 1.0, 1.0)
#: A reading whose final weight is under this (over three deviations off) did not carry the
#: estimate.
TRUSTED_WEIGHT = 0.2
#: A frame's tag reading supports the estimate within this of its sample.
TAG_REACH_NS = 20_000_000
#: An encoder read supports the estimate within this of its sample (one late 10 ms read).
ENCODER_REACH_NS = 15_000_000
#: The camera lag searched (ns): yubi-0006 measured 20 to 35 ms on a Mac bench.
MAX_CAMERA_LAG_NS = 200_000_000
CAMERA_LAG_STEP_NS = 2_500_000
#: Tag frames needed, each with an encoder read beside it, to estimate the camera's lag.
MIN_LAG_FRAMES = 60
#: The share of those frames, closest to the line through the encoder, that judges a lag.
LAG_KEPT = 0.9
#: QC: a take is refused where the fused deviation stays above :data:`BLIND_STD_RAD` for
#: longer than :data:`MAX_BLIND_S`, i.e. neither sensor saw the jaw for that long (the
#: encoder's own stall bound). sx-seal's ``process::jaw`` holds the same two numbers.
BLIND_STD_RAD = math.radians(2.0)
MAX_BLIND_S = 0.5


class JawSensors(StrEnum):
    """Which sensors carried one fused sample."""

    ENCODER_AND_TAGS = "encoder_and_tags"
    TAGS = "tags"
    ENCODER = "encoder"
    #: Neither: interpolated linearly between the trusted readings either side (held at a
    #: take's ends), and its deviation says how far the jaw could have gone.
    NEITHER = "neither"


@dataclass(frozen=True, slots=True)
class FusionModel:
    """The physical and sensor constants of the estimator."""

    #: White-acceleration density (rad/s^2 per root second), the bound on how fast a finger
    #: changes speed. On yubi-0006's grasp take, 4 to 20 all followed the grasps (fused p95
    #: 1.43 to 1.45 mm against the encoder); 10 rejects a 50 ms encoder spike train.
    acceleration_rad_s2: float = 4.0
    #: The encoder's read noise (12-bit AS5601, 0.0015 rad steps, plus magnet wobble).
    encoder_std_rad: float = 0.003
    #: The encoder bias's random walk (rad per root second). Not only a slow drift: yubi-0006's
    #: encoder read 0.040 rad low on a 34.7 mm grasp and 0.038 rad high when squeezed shut, so
    #: its error moves with the jaw. 0.01 lets the tags set the level within about a second
    #: (the closed hold reads 0.2 mm, not the encoder's 5 mm) and adds 0.4 mm of tag noise.
    bias_walk_rad: float = 0.01
    #: The prior on the bias before any tag sees the jaw: a zero off by up to about 15 degrees.
    bias_prior_rad: float = 0.25
    #: One frame's reading from both tags, and from one; scaled up by poor reprojection.
    #: Measured on the grasp take against the fused track: 0.0087 and 0.0176 rad (robust
    #: deviation), white (frame-to-frame correlation 0.07).
    both_tags_std_rad: float = 0.010
    one_tag_std_rad: float = 0.016
    #: A tag reprojecting worse than this widens its frame's deviation proportionally.
    reprojection_px: float = 0.5


@dataclass(frozen=True, slots=True)
class EncoderReads:
    """An encoder's reads on the host clock (dejittered when available)."""

    t_ns: Stamps
    angle_rad: Vector

    def __post_init__(self) -> None:
        _check_series(self.t_ns, self.angle_rad, "encoder reads")


@dataclass(frozen=True, slots=True)
class TagReads:
    """Per frame, the joint angle the tags read and its standard deviation; NaN where unseen."""

    t_ns: Stamps
    angle_rad: Vector
    std_rad: Vector
    #: The two markers' disagreement per frame (NaN unless both were read).
    mirror_rad: Vector

    def __post_init__(self) -> None:
        _check_series(self.t_ns, self.angle_rad, "tag frames", allow_nan=True)
        if (
            self.std_rad.shape != self.angle_rad.shape
            or self.mirror_rad.shape != self.angle_rad.shape
        ):
            raise JawTagError("tag frames need one deviation and mirror per angle")

    @property
    def seen(self) -> NDArray[np.bool_]:
        return np.isfinite(self.angle_rad)

    @property
    def mirror_p95_rad(self) -> float:
        """The markers' disagreement at p95 over the frames that read both (NaN if none)."""
        both = self.mirror_rad[np.isfinite(self.mirror_rad)]
        return float(np.percentile(both, 95)) if len(both) else math.nan


def _check_series(t_ns: Stamps, values: Vector, what: str, *, allow_nan: bool = False) -> None:
    if t_ns.ndim != 1 or t_ns.shape != values.shape:
        raise JawTagError(f"{what} need one timestamp per value")
    if len(t_ns) and np.any(np.diff(t_ns) <= 0):
        raise JawTagError(f"{what} need strictly increasing timestamps")
    if not allow_nan and not np.all(np.isfinite(values)):
        raise JawTagError(f"{what} hold a non-finite angle")


def tag_reads(
    reference: JawReference,
    t_ns: Sequence[int],
    frames: Sequence[Mapping[int, TagPose]],
    model: FusionModel | None = None,
) -> TagReads:
    """Each frame's tag reading (``jaw_tags.read_frame``) with its deviation, NaN where unseen."""
    model = model or FusionModel()
    times = np.asarray(t_ns, np.int64)
    if len(times) != len(frames):
        raise JawTagError("frames need one timestamp each")
    frames = consistent_frames(reference.layout, frames)
    angle = np.full(len(times), np.nan)
    std = np.full(len(times), np.nan)
    mirror = np.full(len(times), np.nan)
    for i, frame in enumerate(frames):
        reading = read_frame(reference, frame)
        if reading is None:
            continue
        used = [frame[m].reprojection_px for m in reference.layout.ids if m in frame]
        base = model.both_tags_std_rad if len(used) == 2 else model.one_tag_std_rad
        angle[i] = reading.angle_rad
        std[i] = base * max(1.0, float(np.mean(used)) / model.reprojection_px)
        mirror[i] = reading.mirror_disagreement_rad
    return TagReads(times, angle, std, mirror)


def camera_lag_ns(tags: TagReads, encoder: EncoderReads) -> int | None:
    """How far the camera's stamps trail the encoder's: the lag that best lines them up.

    Judged by how far the tags stray from the best line through the encoder (a scale and an
    offset): the root mean square of the residuals, less the largest tenth (a wrong frame or a
    squeeze), over frames with an encoder read beside them. None with too few such frames. The
    line, not a constant: a unit whose tags turn 0.77 of its encoder's angle
    (the encoder reads the trigger, and a squeeze moves it past the fingers) was lined up by
    the difference alone at +45 ms in one take and -17.5 ms in the next on one rig; through a
    line, -2.5 and -5 ms, which the correlation of their changes confirms (2026-11-26, the
    two-hand fusion recordings 214716 and 231306).
    """
    seen = tags.seen
    t, angle = tags.t_ns[seen], tags.angle_rad[seen]
    if len(encoder.t_ns) < 2 or len(t) < MIN_LAG_FRAMES:
        return None

    def spread(lag: int) -> float:
        at = t - lag
        index = np.clip(np.searchsorted(encoder.t_ns, at), 1, len(encoder.t_ns) - 1)
        near = (
            np.minimum(np.abs(encoder.t_ns[index] - at), np.abs(encoder.t_ns[index - 1] - at))
            <= ENCODER_REACH_NS
        )
        if np.count_nonzero(near) < MIN_LAG_FRAMES:
            return math.inf
        read = np.interp(at[near], encoder.t_ns, encoder.angle_rad)
        line = np.vstack([read, np.ones(len(read))]).T
        fit = np.linalg.lstsq(line, angle[near], rcond=None)[0]
        residual = np.sort(np.abs(angle[near] - line @ fit))[: int(LAG_KEPT * len(read))]
        return float(np.sqrt(np.mean(residual**2)))

    lags = range(-MAX_CAMERA_LAG_NS, MAX_CAMERA_LAG_NS + 1, CAMERA_LAG_STEP_NS)
    best = min(lags, key=spread)
    return None if math.isinf(spread(best)) else best


@dataclass(frozen=True, slots=True)
class FusedJaw:
    """A take's fused jaw angle at the requested times."""

    t_ns: Stamps
    angle_rad: Vector
    std_rad: Vector
    sensors: tuple[JawSensors, ...]
    #: The encoder's estimated bias (its reading less the jaw angle) at each sample.
    encoder_bias_rad: Vector
    #: The lag subtracted from the tag frames' stamps.
    camera_lag_ns: int
    #: Of each sensor's readings, the share the estimate trusted (weight at least 0.5).
    encoder_trusted: float
    tags_trusted: float

    def longest_unsupported_s(self, bound_rad: float) -> float:
        """The longest run of samples whose deviation exceeds ``bound_rad``, in seconds."""
        over = self.std_rad > bound_rad
        longest, start = 0.0, None
        for i, flag in enumerate(bool(v) for v in over):
            if flag and start is None:
                start = i
            if start is not None and (not flag or i == len(over) - 1):
                end = i if flag else i - 1
                span = (int(self.t_ns[end]) - int(self.t_ns[start])) / 1e9
                # A single sample over the bound spans at least its own frame interval.
                if end == start and len(self.t_ns) > 1:
                    span = float(np.median(np.diff(self.t_ns))) / 1e9
                longest = max(longest, span)
                start = None
        return longest


# Measurement kinds on the merged timeline.
_ENCODER, _TAG, _OUTPUT = 0, 1, 2


def fuse_jaw(
    at_ns: Sequence[int] | Stamps,
    *,
    encoder: EncoderReads | None,
    tags: TagReads | None,
    lag_ns: int = 0,
    model: FusionModel | None = None,
) -> FusedJaw:
    """The jaw angle at ``at_ns`` (on the encoder's clock) from whichever sensors saw it.

    ``lag_ns`` is how far the tag stamps trail the encoder's (:func:`camera_lag_ns`; 0 without
    an encoder to measure it against).
    """
    model = model or FusionModel()
    out_t = np.asarray(at_ns, np.int64)
    if out_t.ndim != 1 or len(out_t) == 0 or np.any(np.diff(out_t) <= 0):
        raise JawTagError("the fused jaw needs strictly increasing sample times")
    enc_t = encoder.t_ns if encoder is not None else np.zeros(0, np.int64)
    enc_z = encoder.angle_rad if encoder is not None else np.zeros(0)
    if tags is not None:
        seen = tags.seen
        tag_t = tags.t_ns[seen] - lag_ns
        tag_z, tag_s = tags.angle_rad[seen], tags.std_rad[seen]
    else:
        tag_t, tag_z, tag_s = np.zeros(0, np.int64), np.zeros(0), np.zeros(0)
    if len(enc_t) + len(tag_t) == 0:
        raise JawTagError("neither the encoder nor the tags read the jaw")

    times = np.concatenate([enc_t, tag_t, out_t])
    kinds = np.concatenate(
        [
            np.full(len(enc_t), _ENCODER),
            np.full(len(tag_t), _TAG),
            np.full(len(out_t), _OUTPUT),
        ]
    )
    values = np.concatenate([enc_z, tag_z, np.zeros(len(out_t))])
    stds = np.concatenate([np.full(len(enc_t), model.encoder_std_rad), tag_s, np.ones(len(out_t))])
    origin = np.concatenate([np.arange(len(enc_t)), np.arange(len(tag_t)), np.arange(len(out_t))])
    order = np.lexsort((kinds, times))
    times, kinds, values, stds, origin = (
        times[order],
        kinds[order],
        values[order],
        stds[order],
        origin[order],
    )
    measured = kinds != _OUTPUT
    weights = np.ones(len(times))
    first = float(values[measured][0])
    state = np.zeros((len(times), 3))
    cov = np.zeros((len(times), 3, 3))
    for scale in REWEIGHT_SCALES:
        state, cov = _smooth(times, kinds, values, scale * stds / np.sqrt(weights), first, model)
        predicted = state[:, 0] + np.where(kinds == _ENCODER, state[:, 2], 0.0)
        normalized = (values - predicted) / (scale * stds)
        weights = np.where(measured, (STUDENT_NU + 1.0) / (STUDENT_NU + normalized**2), 1.0)
    state, cov = _smooth(times, kinds, values, stds / np.sqrt(weights), first, model)

    outputs = kinds == _OUTPUT
    out_index = origin[outputs]
    angle = np.empty(len(out_t))
    std = np.empty(len(out_t))
    bias = np.empty(len(out_t))
    angle[out_index] = state[outputs, 0]
    std[out_index] = np.sqrt(np.maximum(cov[outputs, 0, 0], 0.0))
    bias[out_index] = state[outputs, 2]
    trusted = measured & (weights >= TRUSTED_WEIGHT)
    if not np.any(trusted):
        raise JawTagError("the fused jaw trusted none of its readings")
    trusted_enc = weights[kinds == _ENCODER] >= TRUSTED_WEIGHT
    trusted_tag = weights[kinds == _TAG] >= TRUSTED_WEIGHT
    enc_ok = np.zeros(len(enc_t), bool)
    enc_ok[origin[kinds == _ENCODER]] = trusted_enc
    tag_ok = np.zeros(len(tag_t), bool)
    tag_ok[origin[kinds == _TAG]] = trusted_tag
    by_encoder = _near(out_t, enc_t[enc_ok], ENCODER_REACH_NS)
    by_tags = _near(out_t, tag_t[tag_ok], TAG_REACH_NS)
    sensors = tuple(
        JawSensors.ENCODER_AND_TAGS
        if e and t
        else JawSensors.ENCODER
        if e
        else JawSensors.TAGS
        if t
        else JawSensors.NEITHER
        for e, t in zip((bool(v) for v in by_encoder), (bool(v) for v in by_tags), strict=True)
    )
    # Where neither sensor saw the jaw, the smoother's mean is a cubic through the edges' rates,
    # and past a take's last reading it runs on at the last rate (2026-11-26: 4 degrees of drift
    # over a 1.5 s tail, 15 degrees at a hidden start). Nothing measured that motion, so the
    # angle there is linear between the trusted readings either side, held at the take's ends;
    # the deviation keeps the smoother's, which says how far the jaw could have gone.
    gap = np.fromiter((s is JawSensors.NEITHER for s in sensors), bool, len(sensors))
    if np.any(gap):
        angle[gap] = np.interp(out_t[gap], times[trusted], state[trusted, 0])
    return FusedJaw(
        t_ns=out_t,
        angle_rad=angle,
        std_rad=std,
        sensors=sensors,
        encoder_bias_rad=bias,
        camera_lag_ns=lag_ns,
        encoder_trusted=float(trusted_enc.mean()) if len(trusted_enc) else 0.0,
        tags_trusted=float(trusted_tag.mean()) if len(trusted_tag) else 0.0,
    )


def _near(at: Stamps, stamps: Stamps, reach: int) -> NDArray[np.bool_]:
    if len(stamps) == 0:
        return np.zeros(len(at), bool)
    index = np.clip(np.searchsorted(stamps, at), 1, max(1, len(stamps) - 1))
    lower = stamps[np.minimum(index - 1, len(stamps) - 1)]
    upper = stamps[np.minimum(index, len(stamps) - 1)]
    return np.asarray(np.minimum(np.abs(lower - at), np.abs(upper - at)) <= reach, np.bool_)


def _smooth(
    times: Stamps,
    kinds: NDArray[np.int_],
    values: Vector,
    stds: Vector,
    first_angle: float,
    model: FusionModel,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """A Kalman filter forward and a Rauch-Tung-Striebel pass back, one scalar read per epoch."""
    count = len(times)
    x_pred = np.zeros((count, 3))
    p_pred = np.zeros((count, 3, 3))
    x_filt = np.zeros((count, 3))
    p_filt = np.zeros((count, 3, 3))
    q_acc = model.acceleration_rad_s2**2
    q_bias = model.bias_walk_rad**2
    x = np.array([first_angle, 0.0, 0.0])
    p = np.diag([0.5**2, 2.0**2, model.bias_prior_rad**2])
    seconds = (times - times[0]).astype(np.float64) / 1e9
    steps = np.diff(seconds, prepend=seconds[0])
    for k in range(count):
        dt = float(steps[k])
        if dt > 0.0:
            x = np.array([x[0] + dt * x[1], x[1], x[2]])
            f = np.array([[1.0, dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
            q = np.array(
                [
                    [q_acc * dt**3 / 3.0, q_acc * dt**2 / 2.0, 0.0],
                    [q_acc * dt**2 / 2.0, q_acc * dt, 0.0],
                    [0.0, 0.0, q_bias * dt],
                ]
            )
            p = f @ p @ f.T + q
        x_pred[k], p_pred[k] = x, p
        kind = int(kinds[k])
        if kind != _OUTPUT:
            h = np.array([1.0, 0.0, 1.0 if kind == _ENCODER else 0.0])
            ph = p @ h
            gain = ph / (float(h @ ph) + float(stds[k]) ** 2)
            x = x + gain * (float(values[k]) - float(h @ x))
            p = p - np.outer(gain, ph)
            p = (p + p.T) / 2.0
        x_filt[k], p_filt[k] = x, p
    x_s = x_filt.copy()
    p_s = p_filt.copy()
    for k in range(count - 2, -1, -1):
        dt = float(steps[k + 1])
        f = np.array([[1.0, dt, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        gain = np.asarray(p_filt[k] @ f.T @ np.linalg.inv(p_pred[k + 1]), np.float64)
        x_s[k] = x_filt[k] + gain @ (x_s[k + 1] - x_pred[k + 1])
        p_s[k] = p_filt[k] + gain @ (p_s[k + 1] - p_pred[k + 1]) @ gain.transpose()
    return x_s, p_s


def solve_jaw(
    reference: JawReference,
    t_ns: Sequence[int],
    frames: Sequence[Mapping[int, TagPose]],
    encoder: EncoderReads | None = None,
    model: FusionModel | None = None,
) -> tuple[FusedJaw, TagReads]:
    """A take's jaw at its frames' stamps, on the encoder's clock: its tags, fused with
    whatever its encoder read.

    The camera's lag is measured against the encoder when there is enough of it to line up
    (:func:`camera_lag_ns`), and is 0 otherwise. The angle at a frame's stamp is the jaw's at
    that instant, as the encoder would have read it, not the one the frame shows ``lag``
    earlier, so it sits on the delivered encoder topic as an encoder read would.
    """
    tags = tag_reads(reference, t_ns, frames, model)
    lag = camera_lag_ns(tags, encoder) if encoder is not None and len(encoder.t_ns) else None
    fused = fuse_jaw(tags.t_ns, encoder=encoder, tags=tags, lag_ns=lag or 0, model=model)
    return fused, tags


#: The seal's solved-jaw spool for one side (sx-seal ``process::jaw``), beside the raw spool.
SOLVED_SPOOL_HEADER = "t_monotonic_ns,angle_rad,angle_std_rad,sensors"


def solved_spool_name(side: str) -> str:
    return f"gripper_{side}.jaw_from_tags.csv"


def write_solved_spool(fused: FusedJaw, path: Path, envelope_rad: tuple[float, float]) -> int:
    """Write ``fused`` as the seal reads it; returns how many angles were clamped to the envelope.

    The jaw law refuses an angle outside its envelope. A reading a hair past a stop (noise at a
    closed jaw) is clamped and counted; a track that leaves it by more than its own deviation
    and the markers' mirror bound is refused.
    """
    low, high = envelope_rad
    slack = MAX_MIRROR_DISAGREEMENT_RAD + 3.0 * fused.std_rad
    if np.any(fused.angle_rad < low - slack) or np.any(fused.angle_rad > high + slack):
        raise JawTagError("the solved jaw leaves the jaw law's envelope; check the reference")
    clamped = np.clip(fused.angle_rad, low, high)
    rows = [SOLVED_SPOOL_HEADER]
    rows += [
        f"{int(t)},{angle:.6f},{std:.6f},{sensors.value}"
        for t, angle, std, sensors in zip(
            fused.t_ns.tolist(),
            clamped.tolist(),
            fused.std_rad.tolist(),
            fused.sensors,
            strict=True,
        )
    ]
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    return int(np.count_nonzero(clamped != fused.angle_rad))
