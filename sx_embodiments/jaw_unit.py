"""One YUBI gripper unit's measured jaw: how its encoder, its finger tags and its pads read the
gap, fitted once from a calipered protocol take and used for every later take of that unit.

The truth is the caliper: the gap between the pad surfaces. The canonical jaw law
(:func:`sx_embodiments.yubi.jaw_gap_m`, the yubi-v2 URDF's gap curve) is rigid geometry, so a
finger angle ``theta`` reads ``jaw_gap_m(theta)`` only while nothing presses the pads. Three
things differ per unit (two-hand fusion recordings, 2026-11-26), and each is one fitted map:

- :attr:`JawUnitCalibration.encoder_map`: the encoder reads the trigger, not the fingers. In
  free motion the finger angle is ``scale * encoder + offset``: that unit's trigger stop sat
  0.11 rad below the fingers' contact, so an empty close read 14 mm.
- :attr:`JawUnitCalibration.tag_map`: the finger tags' rotation (a :class:`JawReference`
  fixed on this unit once) is the finger angle up to ``scale * tags + offset``; on that unit
  the tags turned 0.77 to 0.81 of the encoder.
- :attr:`JawUnitCalibration.pad_mm_per_rad`: a firm squeeze moves the trigger past the fingers
  (the *squeeze*, finger angle less the encoder's) and compresses the pads and the linkage,
  so the rigid gap of the fingers reads less than the object. The pad surfaces' gap is the
  rigid gap plus ``pad_mm_per_rad * squeeze``.

With both sensors, the fusion's encoder bias is that squeeze (the solver reads every
mapped encoder read as ``angle + bias``), so the delivered gap is corrected under load. Tags
alone see the fingers but not the squeeze, so a firm hold reads short by the pad compression.
The encoder alone cannot see the object at all under load: at the trigger's stop it reads
the same for every width (0 rad on a 12 mm bar), and no model of the encoder alone recovers
what it never measured; only a sensor on the finger joint (or the tags) does.

The protocol take (:data:`PROTOCOL_WIDTHS_MM`, :func:`segment_protocol`) holds each calipered
width lightly and then firmly, opening fully between them, so every hold is a flat stretch of
the encoder matched to the list in order.
"""

from __future__ import annotations

import json
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from functools import cache
from typing import Final, Literal, Protocol, cast

import numpy as np
from numpy.typing import NDArray
from sx_contracts.identity import JsonValue, content_digest

from .yubi import jaw_gap_m, yubi_jaw

type Vector = NDArray[np.float64]
type Stamps = NDArray[np.int64]

# -- what the solver reads and returns ------------------------------------------------------------
# The jaw solver itself is Rust (sx-embodiments `drivers/sx-jaw`, the one implementation the
# headset runs while it records and the Library runs afterwards; Python reaches it through
# `sx_seal`). This module fits a unit's maps from its readings and judges them with the
# solver's fusion, passed in as `Fuse`.


@dataclass(frozen=True, slots=True)
class EncoderReads:
    """An encoder's reads on the host clock, stamps strictly increasing."""

    t_ns: Stamps
    angle_rad: Vector


@dataclass(frozen=True, slots=True)
class TagReads:
    """Per frame, the joint angle the tags read and its deviation; NaN where unseen."""

    t_ns: Stamps
    angle_rad: Vector
    std_rad: Vector
    mirror_rad: Vector

    @property
    def seen(self) -> NDArray[np.bool_]:
        return np.isfinite(self.angle_rad)


class Fused(Protocol):
    """The solver's fused jaw at the requested stamps."""

    @property
    def angle_rad(self) -> Vector: ...

    @property
    def encoder_bias_rad(self) -> Vector: ...


#: The solver's fusion: the jaw at ``at_ns`` from whichever of the encoder and the tags (moved
#: onto the encoder's clock by the lag) are given.
Fuse = Callable[[Stamps, EncoderReads | None, TagReads | None, int], Fused]

#: OpenCV's 4x4 dictionaries share their first fifty codes; YUBI's markers are IDs 0 to 3.
YUBI_JAW_DICTIONARY: Final = "DICT_4X4_50"


@dataclass(frozen=True, slots=True)
class JawTagLayout:
    """Which markers are on the two fingers, and how big they are printed."""

    first_id: int
    second_id: int
    marker_side_m: float = 0.015
    dictionary: str = YUBI_JAW_DICTIONARY

    def __post_init__(self) -> None:
        if self.first_id == self.second_id or not 0 <= min(self.first_id, self.second_id):
            raise JawUnitError("the two finger markers need distinct non-negative IDs")
        if max(self.first_id, self.second_id) >= 50 or self.dictionary != YUBI_JAW_DICTIONARY:
            raise JawUnitError("the finger markers are DICT_4X4_50 IDs 0 to 49")
        if not (math.isfinite(self.marker_side_m) and 0.002 <= self.marker_side_m <= 0.1):
            raise JawUnitError(f"marker side {self.marker_side_m!r} m is not a printed tag size")

    @property
    def ids(self) -> tuple[int, int]:
        return (self.first_id, self.second_id)


@dataclass(frozen=True, slots=True)
class MarkerReference:
    """Where one marker sits on its finger: its pose at ``angle_rad`` (row-major rotation)
    and its hinge axis."""

    marker_id: int
    reference: tuple[float, ...]
    axis: tuple[float, float, float]
    angle_rad: float


class ReferenceOrigin(StrEnum):
    ENCODER = "encoder"
    CLOSED_STOP = "closed_stop"


@dataclass(frozen=True, slots=True)
class JawReference:
    """One rig side's two marker references, and how they were fixed."""

    layout: JawTagLayout
    first: MarkerReference
    second: MarkerReference
    origin: ReferenceOrigin

    def markers(self) -> tuple[MarkerReference, MarkerReference]:
        return (self.first, self.second)

JAW_UNIT_SCHEMA: Final = "sx.jaw-unit-calibration/v1"
Side = Literal["left", "right"]

#: The protocol's calipered widths in order, mm: an empty close, then five blocks.
PROTOCOL_WIDTHS_MM: Final = (0.0, 5.0, 10.0, 20.0, 40.0, 60.0)
#: The block held while the tags are covered, and again while the encoder is unplugged.
PROTOCOL_CHECK_WIDTH_MM: Final = 20.0
#: A hold is a stretch of the encoder slower than this, rad/s, for at least this long, s.
HOLD_RATE_RAD_S: Final = 0.08
HOLD_MIN_S: Final = 1.0
#: An open hold reads above this share of the take's widest encoder reading.
OPEN_SHARE: Final = 0.85
#: Each hold's edges are dropped by this much (the touch settling), ns.
HOLD_TRIM_NS: Final = 250_000_000
#: A fitted scale outside this band is a wrong segmentation or a broken unit, not a unit.
SCALE_BAND: Final = (0.5, 2.0)


class JawUnitError(ValueError):
    """A jaw unit calibration document, protocol take or fit is not usable."""


class Grip(StrEnum):
    """How a protocol hold was held."""

    #: The pads just touch the object (or each other): no squeeze.
    LIGHT = "light"
    #: Squeezed hard: the trigger moves past the fingers and the pads compress.
    FIRM = "firm"
    #: Fully open, nothing held.
    OPEN = "open"


@dataclass(frozen=True, slots=True)
class LinearMap:
    """``finger angle = scale * reading + offset_rad``."""

    scale: float
    offset_rad: float

    def __post_init__(self) -> None:
        if not (math.isfinite(self.scale) and math.isfinite(self.offset_rad)):
            raise JawUnitError("a jaw map needs a finite scale and offset")
        if not SCALE_BAND[0] <= self.scale <= SCALE_BAND[1]:
            raise JawUnitError(f"a jaw map scale {self.scale:.3f} is outside {SCALE_BAND}")

    def __call__(self, reading: Vector) -> Vector:
        return np.asarray(self.scale * reading + self.offset_rad, np.float64)


@dataclass(frozen=True, slots=True)
class HoldResult:
    """One protocol hold against its caliper: each estimate's median gap less the truth, mm."""

    width_mm: float
    grip: Grip
    tags_only_mm: float
    encoder_only_mm: float
    fused_mm: float


@dataclass(frozen=True, slots=True)
class JawUnitCalibration:
    """One gripper unit's jaw, measured once (:mod:`sx_embodiments.jaw_unit`)."""

    side: Side
    #: The encoder's identity: the mount calibration's ``device_id`` (the rig's
    #: ``{side}_encoder`` component).
    encoder: str
    #: The wrist camera's flash UID (the rig's ``{side}_camera``), which sees the tags.
    camera: str
    calibrated_at_utc: str
    #: The protocol take it was fitted from.
    take: str
    #: The finger markers' poses on this unit, fixed once.
    reference: JawReference
    tag_map: LinearMap
    encoder_map: LinearMap
    pad_mm_per_rad: float
    #: The protocol take's holds, judged by this calibration.
    holds: tuple[HoldResult, ...]

    def __post_init__(self) -> None:
        if self.side not in ("left", "right"):
            raise JawUnitError(f"a YUBI has no {self.side!r} side")
        if not (self.encoder and self.camera and self.take):
            raise JawUnitError("a jaw unit names its encoder, camera and take")
        if not (math.isfinite(self.pad_mm_per_rad) and 0.0 <= self.pad_mm_per_rad <= 500.0):
            raise JawUnitError(f"pad compression {self.pad_mm_per_rad!r} mm/rad is not physical")
        if not self.calibrated_at_utc.endswith("Z"):
            raise JawUnitError("calibrated_at_utc is UTC ISO 8601 ending in Z")

    @property
    def calibration_id(self) -> str:
        return content_digest(self.to_dict())

    # -- applying it -------------------------------------------------------------------------

    def map_tags(self, raw: TagReads) -> TagReads:
        scale = abs(self.tag_map.scale)
        return TagReads(
            raw.t_ns, self.tag_map(raw.angle_rad), np.asarray(raw.std_rad * scale), raw.mirror_rad
        )

    def map_encoder(self, encoder: EncoderReads) -> EncoderReads:
        """The encoder's reads as finger angles in free motion."""
        return EncoderReads(encoder.t_ns, self.encoder_map(encoder.angle_rad))

    def gap_m(self, fused: Fused) -> Vector:
        """The pad surfaces' gap: the fingers' rigid gap plus the pads' compression, which the
        squeeze (the encoder reading past the fingers, ``-bias``) says."""
        squeeze = np.maximum(0.0, -fused.encoder_bias_rad)
        return rigid_gap_m(fused.angle_rad) + self.pad_mm_per_rad * squeeze / 1000.0

    def delivered_angle(self, fused: Fused) -> Vector:
        """The joint angle the jaw law reads as :meth:`gap_m` (never below closed): what the
        encoder topic carries."""
        return angle_at_gap(np.maximum(0.0, self.gap_m(fused)))

    # -- the document ------------------------------------------------------------------------

    def to_dict(self) -> dict[str, JsonValue]:
        layout = self.reference.layout
        return {
            "schema": JAW_UNIT_SCHEMA,
            "side": self.side,
            "encoder": self.encoder,
            "camera": self.camera,
            "calibrated_at_utc": self.calibrated_at_utc,
            "take": self.take,
            "reference": {
                "layout": {
                    "first_id": layout.first_id,
                    "second_id": layout.second_id,
                    "marker_side_m": layout.marker_side_m,
                    "dictionary": layout.dictionary,
                },
                "origin": self.reference.origin.value,
                "markers": [
                    {
                        "marker_id": marker.marker_id,
                        "reference": list(marker.reference),
                        "axis": list(marker.axis),
                        "angle_rad": marker.angle_rad,
                    }
                    for marker in self.reference.markers()
                ],
            },
            "tag_map": {"scale": self.tag_map.scale, "offset_rad": self.tag_map.offset_rad},
            "encoder_map": {
                "scale": self.encoder_map.scale,
                "offset_rad": self.encoder_map.offset_rad,
            },
            "pad_mm_per_rad": self.pad_mm_per_rad,
            "holds": [
                {
                    "width_mm": hold.width_mm,
                    "grip": hold.grip.value,
                    "tags_only_mm": hold.tags_only_mm,
                    "encoder_only_mm": hold.encoder_only_mm,
                    "fused_mm": hold.fused_mm,
                }
                for hold in self.holds
            ],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))

    @classmethod
    def from_json(cls, text: str) -> JawUnitCalibration:
        try:
            document = json.loads(text)
        except json.JSONDecodeError as error:
            raise JawUnitError(f"a jaw unit calibration is not JSON: {error}") from error
        return cls.from_dict(_object(document, "the document"))

    @classmethod
    def from_dict(cls, document: Mapping[str, object]) -> JawUnitCalibration:
        _exact(
            document,
            {
                "schema",
                "side",
                "encoder",
                "camera",
                "calibrated_at_utc",
                "take",
                "reference",
                "tag_map",
                "encoder_map",
                "pad_mm_per_rad",
                "holds",
            },
            "the document",
        )
        if document["schema"] != JAW_UNIT_SCHEMA:
            raise JawUnitError(f"schema is not {JAW_UNIT_SCHEMA}")
        side = _text(document, "side")
        if side not in ("left", "right"):
            raise JawUnitError(f"a YUBI has no {side!r} side")
        reference = _object(document["reference"], "reference")
        _exact(reference, {"layout", "origin", "markers"}, "reference")
        layout_doc = _object(reference["layout"], "reference.layout")
        _exact(layout_doc, {"first_id", "second_id", "marker_side_m", "dictionary"}, "layout")
        try:
            layout = JawTagLayout(
                _integer(layout_doc, "first_id"),
                _integer(layout_doc, "second_id"),
                _number(layout_doc, "marker_side_m"),
                _text(layout_doc, "dictionary"),
            )
            origin = ReferenceOrigin(_text(reference, "origin"))
        except ValueError as error:
            raise JawUnitError(f"reference: {error}") from error
        markers_doc = reference["markers"]
        if not isinstance(markers_doc, list) or len(cast("list[object]", markers_doc)) != 2:
            raise JawUnitError("reference.markers holds the two finger markers")
        markers: list[MarkerReference] = []
        for item in cast("list[object]", markers_doc):
            marker = _object(item, "a marker")
            _exact(marker, {"marker_id", "reference", "axis", "angle_rad"}, "a marker")
            rotation = _numbers(marker, "reference", 9)
            axis = _numbers(marker, "axis", 3)
            markers.append(
                MarkerReference(
                    _integer(marker, "marker_id"),
                    rotation,
                    (axis[0], axis[1], axis[2]),
                    _number(marker, "angle_rad"),
                )
            )
        if (markers[0].marker_id, markers[1].marker_id) != layout.ids:
            raise JawUnitError("reference.markers are not the layout's two markers in order")
        holds: list[HoldResult] = []
        holds_doc = document["holds"]
        if not isinstance(holds_doc, list):
            raise JawUnitError("holds is a list")
        for item in cast("list[object]", holds_doc):
            hold = _object(item, "a hold")
            _exact(
                hold,
                {"width_mm", "grip", "tags_only_mm", "encoder_only_mm", "fused_mm"},
                "a hold",
            )
            try:
                grip = Grip(_text(hold, "grip"))
            except ValueError as error:
                raise JawUnitError(f"a hold: {error}") from error
            holds.append(
                HoldResult(
                    _number(hold, "width_mm"),
                    grip,
                    _number(hold, "tags_only_mm"),
                    _number(hold, "encoder_only_mm"),
                    _number(hold, "fused_mm"),
                )
            )
        return cls(
            side=side,
            encoder=_text(document, "encoder"),
            camera=_text(document, "camera"),
            calibrated_at_utc=_text(document, "calibrated_at_utc"),
            take=_text(document, "take"),
            reference=JawReference(layout, markers[0], markers[1], origin),
            tag_map=_map(document, "tag_map"),
            encoder_map=_map(document, "encoder_map"),
            pad_mm_per_rad=_number(document, "pad_mm_per_rad"),
            holds=tuple(holds),
        )


# -- the rigid jaw law, both ways -------------------------------------------------------------


@cache
def _law() -> tuple[Vector, Vector]:
    jaw = yubi_jaw()
    angles = np.linspace(jaw.joint_lower[0], jaw.joint_upper[0], 4001)
    gaps = np.array([jaw_gap_m(float(a)) for a in angles])
    if np.any(np.diff(gaps) <= 0):
        raise JawUnitError("the canonical jaw's gap does not grow with its angle")
    return np.asarray(angles, np.float64), np.asarray(gaps, np.float64)


def rigid_gap_m(angle_rad: Vector) -> Vector:
    """The canonical jaw law's gap at each angle, clamped at its open stop. Below the closed
    stop it continues on its closing slope (negative): pads pressed together turn the
    fingers past contact, and only the pads' compression brings that back to zero."""
    angles, gaps = _law()
    slope = (gaps[1] - gaps[0]) / (angles[1] - angles[0])
    inside = np.interp(angle_rad, angles, gaps)
    return np.asarray(
        np.where(angle_rad < angles[0], gaps[0] + slope * (angle_rad - angles[0]), inside),
        np.float64,
    )


def angle_at_gap(gap_m: Vector) -> Vector:
    """The joint angle at which :func:`rigid_gap_m` reads each gap (clamped at the open stop)."""
    angles, gaps = _law()
    slope = (gaps[1] - gaps[0]) / (angles[1] - angles[0])
    inside = np.interp(gap_m, gaps, angles)
    return np.asarray(
        np.where(gap_m < gaps[0], angles[0] + (gap_m - gaps[0]) / slope, inside), np.float64
    )


# -- the protocol take ------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class ProtocolHold:
    """One hold of a protocol take, on the encoder's clock."""

    start_ns: int
    end_ns: int
    width_mm: float
    grip: Grip

    def __post_init__(self) -> None:
        if self.end_ns <= self.start_ns:
            raise JawUnitError("a hold ends after it starts")
        if not (math.isfinite(self.width_mm) and self.width_mm >= 0.0):
            raise JawUnitError("a hold's width is a non-negative caliper reading")


def _flat_runs(encoder: EncoderReads) -> list[tuple[int, int, float]]:
    """The encoder's still stretches: (start ns, end ns, median angle)."""
    t, angle = encoder.t_ns, encoder.angle_rad
    if len(t) < 10:
        return []
    grid = np.arange(int(t[0]), int(t[-1]), 10_000_000, dtype=np.int64)
    index = np.clip(np.searchsorted(t, grid), 1, len(t) - 1)
    alive = np.minimum(np.abs(t[index] - grid), np.abs(t[index - 1] - grid)) <= 50_000_000
    value = np.interp(grid, t, angle)
    smooth = np.convolve(value, np.ones(10) / 10.0, mode="same")
    rate = np.abs(np.gradient(smooth, grid / 1e9))
    still = alive & (rate < HOLD_RATE_RAD_S)
    edges = np.flatnonzero(np.diff(np.concatenate([[0], still.astype(np.int8), [0]])))
    runs: list[tuple[int, int, float]] = []
    for a, b in zip(edges[::2].tolist(), edges[1::2].tolist(), strict=True):
        if (grid[b - 1] - grid[a]) / 1e9 >= HOLD_MIN_S:
            runs.append((int(grid[a]), int(grid[b - 1]), float(np.median(value[a:b]))))
    return runs


def segment_protocol(
    encoder: EncoderReads, open_mm: float, widths_mm: Sequence[float] = PROTOCOL_WIDTHS_MM
) -> list[ProtocolHold]:
    """The protocol's holds, matched in order from the encoder's still stretches.

    The protocol opens fully, then for each width holds it lightly and then firmly, opening
    fully again after each. So the still stretches run open, (light, firm, open) per width;
    anything after the last width (the covered-tag and unplugged checks, the sweep) is not a
    protocol hold. ``open_mm`` is the caliper across the fully open pads. A take that does
    not fit is refused: annotate it by hand instead.
    """
    runs = _flat_runs(encoder)
    if not runs:
        raise JawUnitError("the encoder never held still for a second")
    widest = max(value for _, _, value in runs)
    is_open = [value > OPEN_SHARE * widest for _, _, value in runs]
    if not is_open[0]:
        raise JawUnitError("the protocol starts fully open; the first hold is not open")
    holds: list[ProtocolHold] = []
    i = 0
    for width in widths_mm:
        while i < len(runs) and is_open[i]:
            start, end, _ = runs[i]
            holds.append(_trimmed(start, end, open_mm, Grip.OPEN))
            i += 1
        grasp: list[tuple[int, int, float]] = []
        while i < len(runs) and not is_open[i]:
            grasp.append(runs[i])
            i += 1
        if len(grasp) != 2:
            raise JawUnitError(
                f"the {width:g} mm hold shows {len(grasp)} still stretches, not a light then a "
                "firm one: annotate this take's holds by hand (--holds)"
            )
        (s0, e0, light), (s1, e1, firm) = grasp
        if firm > light:
            raise JawUnitError(f"the {width:g} mm firm hold reads more open than its light one")
        holds.append(_trimmed(s0, e0, width, Grip.LIGHT))
        holds.append(_trimmed(s1, e1, width, Grip.FIRM))
    if i < len(runs) and is_open[i]:
        start, end, _ = runs[i]
        holds.append(_trimmed(start, end, open_mm, Grip.OPEN))
    return holds


def _trimmed(start: int, end: int, width: float, grip: Grip) -> ProtocolHold:
    a, b = start + HOLD_TRIM_NS, end - HOLD_TRIM_NS
    return ProtocolHold(a, b, width, grip) if b > a else ProtocolHold(start, end, width, grip)


# -- fitting a unit ---------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class HoldReading:
    """One hold's median readings: the tags' raw angle (from the take's own reference) and the
    encoder's, NaN where that sensor did not read it."""

    hold: ProtocolHold
    tags_rad: float
    encoder_rad: float

    @property
    def truth_rad(self) -> float:
        """The finger angle whose rigid gap is the caliper's width."""
        return float(angle_at_gap(np.array([self.hold.width_mm / 1000.0]))[0])


def hold_readings(
    holds: Sequence[ProtocolHold], raw_tags: TagReads, encoder: EncoderReads, lag_ns: int = 0
) -> list[HoldReading]:
    """Each hold's median raw tag angle (frames moved onto the encoder's clock) and encoder."""
    tag_t = raw_tags.t_ns - lag_ns
    out: list[HoldReading] = []
    for hold in holds:
        seen = (tag_t >= hold.start_ns) & (tag_t <= hold.end_ns) & raw_tags.seen
        read = (encoder.t_ns >= hold.start_ns) & (encoder.t_ns <= hold.end_ns)
        out.append(
            HoldReading(
                hold,
                float(np.median(raw_tags.angle_rad[seen])) if seen.sum() >= 5 else math.nan,
                float(np.median(encoder.angle_rad[read])) if read.sum() >= 5 else math.nan,
            )
        )
    return out


def _line(x: Vector, y: Vector, what: str) -> LinearMap:
    if len(x) < 3 or float(np.ptp(x)) < 0.2:
        raise JawUnitError(f"{what}: fewer than three free holds spanning 0.2 rad")
    scale, offset = np.polyfit(x, y, 1)
    return LinearMap(float(scale), float(offset))


def fit_maps(readings: Sequence[HoldReading]) -> tuple[LinearMap, LinearMap, float]:
    """The unit's tag map and encoder map, each fitted on the free holds (light and open)
    against the caliper, and its pad compression per radian of squeeze from the firm holds."""
    free = [r for r in readings if r.hold.grip is not Grip.FIRM]
    tagged = [r for r in free if math.isfinite(r.tags_rad)]
    read = [r for r in free if math.isfinite(r.encoder_rad)]
    tag_map = _line(
        np.array([r.tags_rad for r in tagged]), np.array([r.truth_rad for r in tagged]), "tags"
    )
    encoder_map = _line(
        np.array([r.encoder_rad for r in read]), np.array([r.truth_rad for r in read]), "encoder"
    )
    firm = [
        r
        for r in readings
        if r.hold.grip is Grip.FIRM
        and r.hold.width_mm > 0.0
        and math.isfinite(r.tags_rad)
        and math.isfinite(r.encoder_rad)
    ]
    finger = tag_map(np.array([r.tags_rad for r in firm]))
    squeeze = np.maximum(0.0, finger - encoder_map(np.array([r.encoder_rad for r in firm])))
    compression = np.array([r.hold.width_mm for r in firm]) - rigid_gap_m(finger) * 1000.0
    energy = float(np.sum(squeeze * squeeze))
    pad = max(0.0, float(np.sum(compression * squeeze)) / energy) if energy > 1e-6 else 0.0
    return tag_map, encoder_map, pad


# -- judging it ---------------------------------------------------------------------------------


class Estimate(StrEnum):
    """Which sensors an evaluation lets the fusion see."""

    TAGS_ONLY = "tags_only"
    ENCODER_ONLY = "encoder_only"
    FUSED = "fused"


@dataclass(frozen=True, slots=True)
class CheckResult:
    """A real dropout of the protocol take holding the check block: the fused gap less 20 mm."""

    what: Literal["tags_covered", "encoder_unplugged"]
    start_ns: int
    end_ns: int
    fused_mm: float


@dataclass(frozen=True, slots=True)
class Evaluation:
    holds: tuple[HoldResult, ...]
    checks: tuple[CheckResult, ...]
    #: Per estimate, the pad-surface gap at every frame (m), for plots.
    gaps_m: Mapping[Estimate, Vector]
    t_ns: Stamps


def evaluate(
    unit: JawUnitCalibration,
    raw_tags: TagReads,
    encoder: EncoderReads,
    holds: Sequence[ProtocolHold],
    fuse: Fuse,
    lag_ns: int = 0,
) -> Evaluation:
    """The protocol take fused three ways from the same data (tags alone, the encoder alone,
    both) through ``unit``, each hold judged against its caliper; and the take's own covered
    and unplugged stretches, fused, against the check block."""
    tags = unit.map_tags(raw_tags)
    reads = unit.map_encoder(encoder)
    t = raw_tags.t_ns
    runs = {
        Estimate.TAGS_ONLY: fuse(t, None, tags, 0),
        Estimate.ENCODER_ONLY: fuse(t, reads, None, 0),
        Estimate.FUSED: fuse(t, reads, tags, lag_ns),
    }
    gaps = {estimate: unit.gap_m(fused) for estimate, fused in runs.items()}

    def error(estimate: Estimate, hold: ProtocolHold) -> float:
        inside = (t >= hold.start_ns) & (t <= hold.end_ns)
        if not inside.any():
            return math.nan
        return float(np.median(gaps[estimate][inside]) * 1000.0 - hold.width_mm)

    results = tuple(
        HoldResult(
            hold.width_mm,
            hold.grip,
            error(Estimate.TAGS_ONLY, hold),
            error(Estimate.ENCODER_ONLY, hold),
            error(Estimate.FUSED, hold),
        )
        for hold in holds
    )
    checks: list[CheckResult] = []
    unseen = ~raw_tags.seen
    # A dropout span also holds the jaw open between its grasps: a still stretch nearer the
    # open stop than the check block is that, not the block.
    opens = [hold.width_mm for hold in holds if hold.grip is Grip.OPEN]
    past_mm = (max(opens) + PROTOCOL_CHECK_WIDTH_MM) / 2.0 if opens else math.inf
    for what, missing in (
        ("tags_covered", unseen),
        ("encoder_unplugged", ~_encoder_alive(t, encoder)),
    ):
        span = _longest(missing, t)
        if span is None or (span[1] - span[0]) < 2_000_000_000:
            continue
        inside = (t >= span[0]) & (t <= span[1])
        for start, end in _still(t[inside], gaps[Estimate.FUSED][inside]):
            window = inside & (t >= start) & (t <= end)
            read_mm = float(np.median(gaps[Estimate.FUSED][window]) * 1000.0)
            if read_mm > past_mm:
                continue
            checks.append(
                CheckResult(
                    cast("Literal['tags_covered', 'encoder_unplugged']", what),
                    start,
                    end,
                    read_mm - PROTOCOL_CHECK_WIDTH_MM,
                )
            )
    return Evaluation(results, tuple(checks), gaps, t)


def fit_unit(
    *,
    side: Side,
    encoder_id: str,
    camera: str,
    calibrated_at_utc: str,
    take: str,
    reference: JawReference,
    raw_tags: TagReads,
    encoder: EncoderReads,
    holds: Sequence[ProtocolHold],
    fuse: Fuse,
    lag_ns: int = 0,
) -> tuple[JawUnitCalibration, Evaluation]:
    """Fit one unit from its protocol take and judge it on the same take.

    ``reference`` is the take's own marker reference (its closed stop: the protocol closes
    empty), and ``raw_tags`` that reference's readings; the fitted tag map makes them finger
    angles, so the reference is kept with the unit and never refitted per take.
    """
    tag_map, encoder_map, pad = fit_maps(hold_readings(holds, raw_tags, encoder, lag_ns))
    unit = JawUnitCalibration(
        side=side,
        encoder=encoder_id,
        camera=camera,
        calibrated_at_utc=calibrated_at_utc,
        take=take,
        reference=reference,
        tag_map=tag_map,
        encoder_map=encoder_map,
        pad_mm_per_rad=pad,
        holds=(),
    )
    judged = evaluate(unit, raw_tags, encoder, holds, fuse, lag_ns)
    return (
        JawUnitCalibration(
            side=side,
            encoder=encoder_id,
            camera=camera,
            calibrated_at_utc=calibrated_at_utc,
            take=take,
            reference=reference,
            tag_map=tag_map,
            encoder_map=encoder_map,
            pad_mm_per_rad=pad,
            holds=judged.holds,
        ),
        judged,
    )


def _encoder_alive(t: Stamps, encoder: EncoderReads) -> NDArray[np.bool_]:
    if len(encoder.t_ns) == 0:
        return np.zeros(len(t), bool)
    index = np.clip(np.searchsorted(encoder.t_ns, t), 1, max(1, len(encoder.t_ns) - 1))
    lower = encoder.t_ns[np.minimum(index - 1, len(encoder.t_ns) - 1)]
    upper = encoder.t_ns[np.minimum(index, len(encoder.t_ns) - 1)]
    return np.asarray(np.minimum(np.abs(lower - t), np.abs(upper - t)) <= 50_000_000, np.bool_)


def _longest(mask: NDArray[np.bool_], t: Stamps) -> tuple[int, int] | None:
    edges = np.flatnonzero(np.diff(np.concatenate([[0], mask.astype(np.int8), [0]])))
    spans = [(int(t[a]), int(t[b - 1])) for a, b in zip(edges[::2], edges[1::2], strict=True)]
    return max(spans, key=lambda s: s[1] - s[0]) if spans else None


def _still(t: Stamps, gap_m: Vector) -> list[tuple[int, int]]:
    """Stretches of at least :data:`HOLD_MIN_S` where the gap moves under 20 mm/s."""
    if len(t) < 3:
        return []
    width = max(1, round(0.2 / float(np.median(np.diff(t)) / 1e9)))
    smooth = np.convolve(gap_m * 1000.0, np.ones(width) / width, mode="same")
    rate = np.abs(np.gradient(smooth, t / 1e9))
    edges = np.flatnonzero(np.diff(np.concatenate([[0], (rate < 20.0).astype(np.int8), [0]])))
    return [
        (int(t[a]) + HOLD_TRIM_NS, int(t[b - 1]) - HOLD_TRIM_NS)
        for a, b in zip(edges[::2].tolist(), edges[1::2].tolist(), strict=True)
        if (t[b - 1] - t[a]) / 1e9 >= HOLD_MIN_S
    ]


# -- reading the document -----------------------------------------------------------------------


def _object(value: object, what: str) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise JawUnitError(f"{what} is not an object")
    return cast("Mapping[str, object]", value)


def _exact(document: Mapping[str, object], fields: set[str], what: str) -> None:
    if set(document) != fields:
        missing, extra = sorted(fields - set(document)), sorted(set(document) - fields)
        raise JawUnitError(f"{what}: missing {missing}, unexpected {extra}")


def _text(document: Mapping[str, object], key: str) -> str:
    value = document.get(key)
    if not isinstance(value, str) or not value:
        raise JawUnitError(f"{key} is not a non-empty string")
    return value


def _integer(document: Mapping[str, object], key: str) -> int:
    value = document.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise JawUnitError(f"{key} is not an integer")
    return value


def _number(document: Mapping[str, object], key: str) -> float:
    value = document.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise JawUnitError(f"{key} is not a finite number")
    return float(value)


def _numbers(document: Mapping[str, object], key: str, count: int) -> tuple[float, ...]:
    value = document.get(key)
    if not isinstance(value, list) or len(cast("list[object]", value)) != count:
        raise JawUnitError(f"{key} is not {count} numbers")
    return tuple(_number({key: item}, key) for item in cast("list[object]", value))


def _map(document: Mapping[str, object], key: str) -> LinearMap:
    value = _object(document.get(key), key)
    _exact(value, {"scale", "offset_rad"}, key)
    return LinearMap(_number(value, "scale"), _number(value, "offset_rad"))
