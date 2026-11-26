"""One jaw angle from the encoder and the wrist tags, each with the faults it really has.

The tags are rendered markers (``test_jaw_tags``) seen at 30 fps; the encoder reads the same
joint at 100 Hz. Each fault is injected into one sensor and the fused track must stay within
one degree of the truth at the 95th percentile.
"""

from __future__ import annotations

import math
from typing import cast

import numpy as np
import pytest
from numpy.typing import NDArray

from sx_embodiments.jaw_fusion import (
    EncoderReads,
    JawSensors,
    TagReads,
    camera_lag_ns,
    fuse_jaw,
    tag_reads,
)
from sx_embodiments.jaw_tags import JawReference, TagPose, fit_reference, sight_tags
from test_jaw_tags import LAYOUT, LENS, render_jaw, rendered_sweep

FRAME_NS = 33_333_333
READ_NS = 10_000_000
SECONDS = 8.0
ONE_DEGREE = math.radians(1.0)


def _truth(t_ns: NDArray[np.int64]) -> NDArray[np.float64]:
    """Two grasps and a hold: open, close fast onto an object, hold, open, close shut."""
    s = t_ns / 1e9
    return np.asarray(
        0.30 + 0.25 * np.cos(2 * math.pi * s / 4.0) + 0.04 * np.sin(2 * math.pi * s / 0.9),
        np.float64,
    )


@pytest.fixture(scope="module")
def reference() -> JawReference:
    theta, frames = rendered_sweep()
    return cast("JawReference", fit_reference(LAYOUT, frames, theta.tolist()))


@pytest.fixture(scope="module")
def camera() -> tuple[NDArray[np.int64], list[dict[int, TagPose]]]:
    t = np.arange(0, int(SECONDS * 1e9), FRAME_NS, dtype=np.int64)
    return t, [sight_tags(render_jaw(float(a)), LENS, LAYOUT) for a in _truth(t).tolist()]


def _tags(
    reference: JawReference,
    camera: tuple[NDArray[np.int64], list[dict[int, TagPose]]],
    hidden: tuple[float, float] | None = None,
) -> TagReads:
    t, frames = camera
    shown = [
        {} if hidden is not None and hidden[0] <= ti / 1e9 < hidden[1] else frame
        for ti, frame in zip(t.tolist(), frames, strict=True)
    ]
    return tag_reads(reference, t.tolist(), shown)


def _encoder(offset: float = 0.0) -> EncoderReads:
    t = np.arange(0, int(SECONDS * 1e9), READ_NS, dtype=np.int64)
    noise = np.random.default_rng(7).normal(0.0, 0.002, len(t))
    return EncoderReads(t, _truth(t) + offset + noise)


def _p95(t: NDArray[np.int64], angle: NDArray[np.float64]) -> float:
    return float(np.percentile(np.abs(angle - _truth(t)), 95))


def test_a_dead_encoder_leaves_the_tags_alone(
    reference: JawReference, camera: tuple[NDArray[np.int64], list[dict[int, TagPose]]]
) -> None:
    t = camera[0]
    fused = fuse_jaw(t, encoder=None, tags=_tags(reference, camera))
    assert _p95(t, fused.angle_rad) < ONE_DEGREE
    assert set(fused.sensors) == {JawSensors.TAGS}


def test_an_encoder_dropout_and_hidden_tags_lose_no_good_data(
    reference: JawReference, camera: tuple[NDArray[np.int64], list[dict[int, TagPose]]]
) -> None:
    t = camera[0]
    full = _encoder()
    gone = (full.t_ns < 2_000_000_000) | (full.t_ns >= 3_500_000_000)  # 1.5 s silent
    encoder = EncoderReads(full.t_ns[gone], full.angle_rad[gone])
    # The tags are hidden for a second elsewhere, while the jaw moves: the encoder carries it.
    fused = fuse_jaw(t, encoder=encoder, tags=_tags(reference, camera, hidden=(5.0, 6.0)))
    assert _p95(t, fused.angle_rad) < ONE_DEGREE
    seconds = t / 1e9
    assert {fused.sensors[i] for i in np.flatnonzero((seconds > 2.1) & (seconds < 3.4))} == {
        JawSensors.TAGS
    }
    assert {fused.sensors[i] for i in np.flatnonzero((seconds > 5.1) & (seconds < 5.9))} == {
        JawSensors.ENCODER
    }
    assert fused.longest_unsupported_s(math.radians(1.0)) == 0.0


def test_impossible_encoder_spikes_get_no_weight(
    reference: JawReference, camera: tuple[NDArray[np.int64], list[dict[int, TagPose]]]
) -> None:
    t = camera[0]
    clean = _encoder()
    angle = clean.angle_rad.copy()
    # Twice a second, a burst no finger could make: five reads alternating 0.3 rad either
    # way, then a 50 ms excursion of 0.2 rad and back (over 1,000 rad/s^2). Bursts that run
    # together into a 150 ms excursion are a motion a hand can make, and then the encoder's
    # many reads outvote the tags' few; that is not what this guards.
    for start in range(25, len(angle) - 10, 50):
        angle[start : start + 5] += 0.3 * np.array([1, -1, 1, -1, 1])
        angle[start + 5 : start + 10] += 0.2
    fused = fuse_jaw(t, encoder=EncoderReads(clean.t_ns, angle), tags=_tags(reference, camera))
    assert _p95(t, fused.angle_rad) < ONE_DEGREE
    assert float(np.max(np.abs(fused.angle_rad - _truth(t)))) < 2 * ONE_DEGREE
    assert 0.75 < fused.encoder_trusted < 0.85  # the bursts are a fifth of the reads
    # Without the tags only the motion bound is left, and it still holds the track to 2 degrees.
    alone = fuse_jaw(t, encoder=EncoderReads(clean.t_ns, angle), tags=None)
    assert _p95(t, alone.angle_rad) < 2 * ONE_DEGREE


def test_a_mis_zeroed_encoder_is_read_against_the_tags_zero(
    reference: JawReference, camera: tuple[NDArray[np.int64], list[dict[int, TagPose]]]
) -> None:
    t = camera[0]
    tags = _tags(reference, camera)
    fused = fuse_jaw(t, encoder=_encoder(offset=0.05), tags=tags)
    assert _p95(t, fused.angle_rad) < ONE_DEGREE
    assert abs(float(np.median(fused.encoder_bias_rad)) - 0.05) < 0.005
    # The bias learned while the tags saw the jaw carries a stretch they do not.
    hidden = fuse_jaw(t, encoder=_encoder(offset=0.05), tags=_tags(reference, camera, (3.0, 5.0)))
    assert _p95(t, hidden.angle_rad) < ONE_DEGREE


def test_neither_sensor_is_what_qc_refuses(
    reference: JawReference, camera: tuple[NDArray[np.int64], list[dict[int, TagPose]]]
) -> None:
    t = camera[0]
    fused = fuse_jaw(t, encoder=None, tags=_tags(reference, camera, hidden=(3.0, 4.5)))
    blind = fused.std_rad[(t / 1e9 > 3.4) & (t / 1e9 < 4.1)]
    assert float(blind.min()) > math.radians(2.0)
    assert 0.7 < fused.longest_unsupported_s(math.radians(2.0)) < 1.6
    seen = fused.std_rad[(t / 1e9 > 1.0) & (t / 1e9 < 2.5)]
    assert float(seen.max()) < math.radians(0.5)


def test_the_camera_lag_is_measured_against_the_encoder(
    reference: JawReference, camera: tuple[NDArray[np.int64], list[dict[int, TagPose]]]
) -> None:
    late = 30_000_000
    tags = _tags(reference, camera)
    stamped = TagReads(tags.t_ns + late, tags.angle_rad, tags.std_rad, tags.mirror_rad)
    lag = camera_lag_ns(stamped, _encoder(offset=0.05))
    assert lag is not None and abs(lag - late) <= 5_000_000
    fused = fuse_jaw(camera[0], encoder=_encoder(offset=0.05), tags=stamped, lag_ns=lag)
    assert _p95(camera[0], fused.angle_rad) < ONE_DEGREE


def test_where_neither_sensor_saw_the_jaw_it_is_linear_between_readings_and_held_at_the_ends(
    reference: JawReference, camera: tuple[NDArray[np.int64], list[dict[int, TagPose]]]
) -> None:
    # The owner, 2026-11-26: a stretch neither sensor saw is a hold or a line, never a motion
    # the smoother made up. Hidden while the jaw moves (1.2 to 2.2 s), and from 6.5 s to the end.
    t = camera[0]
    seconds = t / 1e9
    tags = _tags(reference, camera, hidden=(1.2, 2.2))
    tags = TagReads(
        tags.t_ns,
        np.where(seconds >= 6.5, np.nan, tags.angle_rad),
        tags.std_rad,
        tags.mirror_rad,
    )
    fused = fuse_jaw(t, encoder=None, tags=tags)
    gap = np.array([s is JawSensors.NEITHER for s in fused.sensors])
    middle = np.flatnonzero(gap & (seconds < 3.0))
    tail = np.flatnonzero(gap & (seconds > 6.0))
    assert len(middle) > 20 and len(tail) > 30
    before, after = middle[0] - 1, middle[-1] + 1
    assert fused.sensors[before] is JawSensors.TAGS and fused.sensors[after] is JawSensors.TAGS
    # On the line through the readings either side: no rate carried in, no cubic overshoot.
    line = np.interp(t[middle], t[[before, after]], fused.angle_rad[[before, after]])
    assert float(np.max(np.abs(fused.angle_rad[middle] - line))) < 0.005
    low, high = sorted(fused.angle_rad[[before, after]])
    inside = fused.angle_rad[middle]
    assert np.all((inside >= low - 1e-9) & (inside <= high + 1e-9))
    # Past the last reading it holds, however long the tail: no drift on the last rate.
    assert float(np.ptp(fused.angle_rad[tail])) < 1e-9
    assert abs(float(fused.angle_rad[tail[0]] - fused.angle_rad[tail[0] - 1])) < 0.005
    # The deviation still says nothing saw it, so QC and training can tell.
    assert np.all(np.diff(fused.std_rad[tail]) > 0)
    assert float(fused.std_rad[tail[-1]]) > math.radians(2.0)
    assert fused.longest_unsupported_s(math.radians(2.0)) > 1.0
    # The hidden start of a take holds the first reading the same way.
    head = fuse_jaw(t, encoder=None, tags=_tags(reference, camera, hidden=(0.0, 1.0)))
    first = [s is JawSensors.NEITHER for s in head.sensors].index(False)
    assert first > 20
    assert float(np.ptp(head.angle_rad[:first])) < 1e-9


def test_the_camera_lag_holds_when_the_encoder_reads_on_another_scale(
    reference: JawReference, camera: tuple[NDArray[np.int64], list[dict[int, TagPose]]]
) -> None:
    # A unit whose tags turn 0.77 of its encoder's angle (the two-hand fusion recordings,
    # 2026-11-26): the lag is still the one that lines their motion up.
    late = 30_000_000
    tags = _tags(reference, camera)
    stamped = TagReads(tags.t_ns + late, tags.angle_rad, tags.std_rad, tags.mirror_rad)
    clean = _encoder()
    scaled = EncoderReads(clean.t_ns, clean.angle_rad / 0.77 + 0.1)
    lag = camera_lag_ns(stamped, scaled)
    assert lag is not None and abs(lag - late) <= 5_000_000


def test_the_camera_lag_holds_through_an_encoder_that_blips(
    reference: JawReference, camera: tuple[NDArray[np.int64], list[dict[int, TagPose]]]
) -> None:
    # A tenth of the reads are 0.3 rad blips (five reads either way, twice a second): the line
    # is fitted again through the frames it fits best, so the blips cannot pull it to a lag.
    late = 30_000_000
    tags = _tags(reference, camera)
    stamped = TagReads(tags.t_ns + late, tags.angle_rad, tags.std_rad, tags.mirror_rad)
    clean = _encoder()
    angle = clean.angle_rad.copy()
    step = max(1, round(0.5e9 / float(np.median(np.diff(clean.t_ns)))))
    for start in range(step // 2, len(angle) - 5, step):
        angle[start : start + 5] += 0.3 * np.array([1, -1, 1, -1, 1])
    lag = camera_lag_ns(stamped, EncoderReads(clean.t_ns, angle))
    assert lag is not None and abs(lag - late) <= 5_000_000, lag
