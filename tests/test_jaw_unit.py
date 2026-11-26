"""A YUBI unit's jaw calibration, fitted from a simulated protocol take with the faults the
two-hand fusion recordings showed (2026-11-26): an encoder that reads the trigger 0.11 rad
below the fingers' contact, tags that turn 0.8 of the finger, and pads that compress under a
firm squeeze. The truth is the caliper."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import cast

import numpy as np
import pytest
from numpy.typing import NDArray

from sx_embodiments.jaw_fusion import EncoderReads, TagReads
from sx_embodiments.jaw_tags import JawReference, fit_reference
from sx_embodiments.jaw_unit import (
    PROTOCOL_CHECK_WIDTH_MM,
    PROTOCOL_WIDTHS_MM,
    Evaluation,
    Grip,
    JawUnitCalibration,
    JawUnitError,
    ProtocolHold,
    angle_at_gap,
    fit_unit,
    rigid_gap_m,
    segment_protocol,
)
from test_jaw_tags import LAYOUT, rendered_sweep

#: The simulated unit: finger = 1.25 * tags - 0.02; finger = encoder - 0.11 in free motion;
#: a firm squeeze moves the trigger 0.10 rad past the fingers and compresses the pads 40 mm/rad.
TAG_SCALE, TAG_OFFSET = 1.25, -0.02
ENCODER_OFFSET = -0.11
SQUEEZE_RAD, PAD_MM_PER_RAD = 0.10, 40.0
OPEN_MM = 88.0


def _angle(gap_mm: float) -> float:
    return float(angle_at_gap(np.array([gap_mm / 1000.0]))[0])


def _protocol() -> list[tuple[float, float, float, bool, bool]]:
    """Keyframes: (seconds to reach it, finger angle, squeeze, tags seen, encoder alive)."""
    wide = _angle(OPEN_MM)
    frames: list[tuple[float, float, float, bool, bool]] = [(0.0, wide, 0.0, True, True)]

    def hold(width: float, seen: bool = True, alive: bool = True) -> None:
        light = _angle(width)
        firm = _angle(width - PAD_MM_PER_RAD * SQUEEZE_RAD)
        frames.extend(
            [
                (0.7, light, 0.0, seen, alive),
                (3.0, light, 0.0, seen, alive),
                (0.5, firm, SQUEEZE_RAD, seen, alive),
                (3.0, firm, SQUEEZE_RAD, seen, alive),
                (0.7, wide, 0.0, seen, alive),
                (2.0, wide, 0.0, seen, alive),
            ]
        )

    frames.append((3.0, wide, 0.0, True, True))
    for width in PROTOCOL_WIDTHS_MM:
        hold(width)
    frames.extend([(5.0, 0.0, 0.0, True, True), (5.0, wide, 0.0, True, True)])  # the sweep
    hold(PROTOCOL_CHECK_WIDTH_MM, seen=False)
    hold(PROTOCOL_CHECK_WIDTH_MM, alive=False)
    return frames


def _take() -> tuple[TagReads, EncoderReads]:
    keys = _protocol()
    times = np.cumsum([k[0] for k in keys])
    finger = np.array([k[1] for k in keys])
    squeeze = np.array([k[2] for k in keys])
    rng = np.random.default_rng(11)

    def state(t_s: NDArray[np.floating], column: int) -> NDArray[np.bool_]:
        index = np.clip(np.searchsorted(times, t_s, side="right") - 1, 0, len(keys) - 1)
        return np.array([keys[i][column] for i in index], bool)

    frame_t = np.arange(0.0, times[-1], 1 / 30.0)
    tag_angle = (np.interp(frame_t, times, finger) - TAG_OFFSET) / TAG_SCALE
    tag_angle = tag_angle + rng.normal(0.0, 0.008, len(frame_t))
    tag_angle[~state(frame_t, 3)] = np.nan
    seen = np.isfinite(tag_angle)
    tags = TagReads(
        (frame_t * 1e9).astype(np.int64),
        tag_angle,
        np.where(seen, 0.010, np.nan),
        np.full(len(frame_t), np.nan),
    )
    read_t = np.arange(0.0, times[-1], 0.01)
    alive = state(read_t, 4)
    trigger = np.interp(read_t, times, finger) - np.interp(read_t, times, squeeze)
    reads = trigger - ENCODER_OFFSET + rng.normal(0.0, 0.002, len(read_t))
    encoder = EncoderReads((read_t[alive] * 1e9).astype(np.int64), reads[alive])
    return tags, encoder


@pytest.fixture(scope="module")
def reference() -> JawReference:
    theta, frames = rendered_sweep()
    return cast("JawReference", fit_reference(LAYOUT, frames, theta.tolist()))


@dataclass(frozen=True)
class Fitted:
    holds: list[ProtocolHold]
    unit: JawUnitCalibration
    judged: Evaluation


@pytest.fixture(scope="module")
def fitted(reference: JawReference) -> Fitted:
    tags, encoder = _take()
    holds = segment_protocol(encoder, OPEN_MM)
    unit, judged = fit_unit(
        side="right",
        encoder_id="ec:da:3b:00:11:22",
        camera="flash-uid-0001",
        calibrated_at_utc="2026-11-27T01:00:00Z",
        take="20261127_010000",
        reference=reference,
        raw_tags=tags,
        encoder=encoder,
        holds=holds,
    )
    return Fitted(holds, unit, judged)


def test_the_protocol_holds_are_found_in_order(fitted: Fitted) -> None:
    holds = fitted.holds
    grasped = [(h.width_mm, h.grip) for h in holds if h.grip is not Grip.OPEN]
    assert grasped == [(w, g) for w in PROTOCOL_WIDTHS_MM for g in (Grip.LIGHT, Grip.FIRM)]
    assert all(h.width_mm == OPEN_MM for h in holds if h.grip is Grip.OPEN)


def test_the_three_maps_are_the_units(fitted: Fitted) -> None:
    unit = fitted.unit
    assert abs(unit.tag_map.scale - TAG_SCALE) < 0.03
    assert abs(unit.tag_map.offset_rad - TAG_OFFSET) < 0.01
    assert abs(unit.encoder_map.scale - 1.0) < 0.02
    assert abs(unit.encoder_map.offset_rad - ENCODER_OFFSET) < 0.01
    assert abs(unit.pad_mm_per_rad - PAD_MM_PER_RAD) < 8.0


def test_fused_reads_the_caliper_light_and_firm_and_each_sensor_alone_says_what_it_cannot(
    fitted: Fitted,
) -> None:
    judged = fitted.judged
    grasped = [h for h in judged.holds if h.grip is not Grip.OPEN]
    assert max(abs(h.fused_mm) for h in grasped) < 1.0
    light = [h for h in grasped if h.grip is Grip.LIGHT]
    firm = [h for h in grasped if h.grip is Grip.FIRM]
    # Tags alone see the fingers, not the squeeze: firm holds read short by the pads.
    assert max(abs(h.tags_only_mm) for h in light) < 1.0
    assert all(-5.5 < h.tags_only_mm < -2.5 for h in firm)
    # The encoder alone sees the trigger: right when free, short by the squeeze when firm.
    assert max(abs(h.encoder_only_mm) for h in light) < 1.0
    assert all(h.encoder_only_mm < -5.0 for h in firm if h.width_mm > 0)


def test_the_real_dropouts_are_judged_against_the_check_block(fitted: Fitted) -> None:
    checks = fitted.judged.checks
    covered = [c for c in checks if c.what == "tags_covered"]
    unplugged = [c for c in checks if c.what == "encoder_unplugged"]
    assert len(covered) >= 2 and len(unplugged) >= 1
    # Covered, the encoder carries the light hold; unplugged, the tags see light and firm as
    # one stretch (the pads move them 4 mm) and read it short by up to that compression.
    assert abs(covered[0].fused_mm) < 1.0
    assert -4.5 < unplugged[0].fused_mm < 0.5


def test_the_document_round_trips_and_refuses_a_stray_field(fitted: Fitted) -> None:
    unit = fitted.unit
    again = JawUnitCalibration.from_json(unit.to_json())
    assert again == unit and again.calibration_id == unit.calibration_id
    document = unit.to_dict()
    document["guess"] = 1
    with pytest.raises(JawUnitError, match="unexpected"):
        JawUnitCalibration.from_dict(document)
    with pytest.raises(JawUnitError):
        JawUnitCalibration.from_json("{}")


def test_pressed_shut_reads_zero_not_the_squeeze() -> None:
    # Below contact the law runs on, so an empty firm close's compression brings it to 0 mm.
    closed = rigid_gap_m(np.array([_angle(-PAD_MM_PER_RAD * SQUEEZE_RAD)]))[0] * 1000.0
    assert math.isclose(closed + PAD_MM_PER_RAD * SQUEEZE_RAD, 0.0, abs_tol=1e-6)


def test_a_take_that_is_not_the_protocol_is_refused() -> None:
    t = np.arange(0, 10_000_000_000, 10_000_000, dtype=np.int64)
    with pytest.raises(JawUnitError, match=r"starts fully open|still stretches"):
        segment_protocol(EncoderReads(t, np.full(len(t), 0.2)), OPEN_MM)
