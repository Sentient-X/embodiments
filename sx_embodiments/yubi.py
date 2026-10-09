"""YUBI encoder interpretation, one jaw law per hardware variant.

Standard YUBI is the existing four-bar hardware: its encoder angle maps to jaw gap
through the canonical embodiment's linkage. Parallel YUBI moves its jaws in a straight
line, so its law is linear in the encoder reading, and the travel, zero and sign come
only from a calibration measured on that unit. The four-bar law is never used for it.
Any other variant has no law until its own measured profile is added here.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from enum import StrEnum
from functools import cache
from typing import Literal

from .parts import GripperSpec


class YubiEncoderError(Exception):
    """The standard YUBI encoder violates its range or per-connection identity."""


@cache
def yubi_jaw() -> GripperSpec:
    """The single canonical YUBI jaw definition shared by both hands."""
    from sx_embodiments import development_embodiments
    from sx_embodiments.parts import GripperSpec

    jaw = next(
        component.part
        for component in development_embodiments["yubi"].components
        if component.instance == "left_jaw"
    )
    if not isinstance(jaw, GripperSpec):
        raise YubiEncoderError("canonical YUBI left_jaw is not a gripper")
    return jaw


# The finger four-bar over-travels the CAD open stop: the left unit's encoder read
# up to 0.876 rad against the URDF box of [0, 0.785398] on 2026-08-18 (the right
# unit tops out at 0.759 rad). A raw sample is refused only beyond the joint box
# plus this envelope; the aperture curve clamps at the CAD open gap for FK.
YUBI_ENCODER_OVERTRAVEL_RAD = math.radians(10.0)


def encoder_envelope_rad() -> tuple[float, float]:
    """The raw-encoder envelope: the canonical jaw's joint box plus the over-travel."""
    jaw = yubi_jaw()
    return (jaw.joint_lower[0], jaw.joint_upper[0] + YUBI_ENCODER_OVERTRAVEL_RAD)


def jaw_gap_m(joint_angle_rad: float) -> float:
    """Jaw opening in metres from the canonical YUBI embodiment."""
    return yubi_jaw().aperture_from_drive(joint_angle_rad)


class YubiVariant(StrEnum):
    STANDARD = "standard"
    PARALLEL = "parallel"


def _fence_device_id(known: str | None, device_id: str) -> str:
    if not device_id.strip():
        raise YubiEncoderError("encoder firmware emitted an empty device ID")
    if known is not None and device_id != known:
        raise YubiEncoderError(
            f"encoder firmware device ID changed from {known!r} to {device_id!r}"
        )
    return device_id


class YubiEncoder:
    """Interpret one standard YUBI UART stream and fence firmware identity changes."""

    def __init__(self) -> None:
        self._firmware_device_id: str | None = None

    def parse(self, line: str) -> float | None:
        """Return a normalized joint angle, or no sample for diagnostic text."""
        try:
            if "," in line:
                device_id, _, value = line.partition(",")
                raw = float(value)
            else:
                device_id, raw = None, float(line)
        except ValueError:
            return None
        # Firmware sends the signed zero difference without modulo. Real units
        # jitter slightly negative at rest; fold one signed revolution to [0, pi].
        if not math.isfinite(raw) or not -2.0 * math.pi <= raw <= 2.0 * math.pi:
            raise YubiEncoderError(f"encoder emitted a value outside the AS5601 range: {raw!r}")
        angle = abs((raw + math.pi) % (2.0 * math.pi) - math.pi)
        maximum = encoder_envelope_rad()[1]
        if angle > maximum:
            raise YubiEncoderError(
                f"encoder joint angle {angle!r} exceeds the YUBI limit {maximum}"
            )
        if device_id is not None:
            self._firmware_device_id = _fence_device_id(self._firmware_device_id, device_id)
        return angle


#: The largest per-point disagreement, in millimetres, between a fitted parallel-jaw
#: law and the apertures measured to fit it. A worse fit is refused, never stored.
PARALLEL_YUBI_MAX_RESIDUAL_MM = 0.5
#: A fit needs this many measured points spread over the jaw's travel.
PARALLEL_YUBI_MIN_SAMPLES = 5
#: Less measured travel than this is not a jaw opening; the points cannot fix a slope.
PARALLEL_YUBI_MIN_TRAVEL_MM = 5.0


class YubiCalibrationError(Exception):
    """A Parallel YUBI calibration is not a usable measured law."""


@dataclass(frozen=True, slots=True)
class ParallelYubiCalibration:
    """One Parallel YUBI unit's measured law: aperture_mm = sign * (count - zero) * mm_per_count.

    ``unit_id`` is the firmware device ID the unit reports, so a calibration can never be
    applied to the unit beside it. ``travel_mm`` is the widest aperture measured;
    ``residual_mm`` the worst disagreement of the fit and ``residual_bound_mm`` the bound
    it was held to, which is also how far a reading may sit outside the travel.
    """

    unit_id: str
    zero_count: float
    mm_per_count: float
    sign: Literal[1, -1]
    travel_mm: float
    residual_mm: float
    residual_bound_mm: float

    def __post_init__(self) -> None:
        if not self.unit_id.strip():
            raise YubiCalibrationError("a Parallel YUBI calibration names its unit")
        numbers = (
            self.zero_count,
            self.mm_per_count,
            self.travel_mm,
            self.residual_mm,
            self.residual_bound_mm,
        )
        if not all(math.isfinite(value) for value in numbers):
            raise YubiCalibrationError("Parallel YUBI calibration values must be finite")
        if self.mm_per_count <= 0.0 or self.travel_mm <= 0.0:
            raise YubiCalibrationError("Parallel YUBI scale and travel must be positive")
        if not 0.0 <= self.residual_mm <= self.residual_bound_mm:
            raise YubiCalibrationError("Parallel YUBI fit residual exceeds its bound")

    def aperture_mm(self, count: float) -> float:
        return self.sign * (count - self.zero_count) * self.mm_per_count

    def count_at(self, aperture_mm: float) -> float:
        return self.zero_count + self.sign * aperture_mm / self.mm_per_count


class ParallelYubiFitRefusalReason(StrEnum):
    TOO_FEW_SAMPLES = "too_few_samples"
    NOT_FINITE = "not_finite"
    NEGATIVE_APERTURE = "negative_aperture"
    NO_CLOSED_STOP = "no_closed_stop"
    NO_TRAVEL = "no_travel"
    RESIDUAL_EXCEEDED = "residual_exceeded"


@dataclass(frozen=True, slots=True)
class ParallelYubiFitRefused:
    reason: ParallelYubiFitRefusalReason
    detail: str


def fit_parallel_yubi_calibration(
    unit_id: str,
    samples: Sequence[tuple[float, float]],
    *,
    max_residual_mm: float = PARALLEL_YUBI_MAX_RESIDUAL_MM,
) -> ParallelYubiCalibration | ParallelYubiFitRefused:
    """Fit one unit's law from measured (encoder count, aperture mm) pairs.

    The points must include the closed stop and span the jaw's travel; the straight
    line through them is accepted only if every point lies within ``max_residual_mm``.
    """

    if len(samples) < PARALLEL_YUBI_MIN_SAMPLES:
        return ParallelYubiFitRefused(
            ParallelYubiFitRefusalReason.TOO_FEW_SAMPLES,
            f"{len(samples)} measured points; at least {PARALLEL_YUBI_MIN_SAMPLES} are needed",
        )
    if not all(math.isfinite(count) and math.isfinite(mm) for count, mm in samples):
        return ParallelYubiFitRefused(
            ParallelYubiFitRefusalReason.NOT_FINITE, "a measured point is not a finite number"
        )
    apertures = [mm for _, mm in samples]
    if min(apertures) < 0.0:
        return ParallelYubiFitRefused(
            ParallelYubiFitRefusalReason.NEGATIVE_APERTURE, "a measured aperture is negative"
        )
    if min(apertures) > max_residual_mm:
        return ParallelYubiFitRefused(
            ParallelYubiFitRefusalReason.NO_CLOSED_STOP,
            "no point was measured with the jaws closed, so the zero is unknown",
        )
    if max(apertures) - min(apertures) < PARALLEL_YUBI_MIN_TRAVEL_MM:
        return ParallelYubiFitRefused(
            ParallelYubiFitRefusalReason.NO_TRAVEL,
            f"the points span under {PARALLEL_YUBI_MIN_TRAVEL_MM} mm of jaw travel",
        )
    n = len(samples)
    mean_count = sum(count for count, _ in samples) / n
    mean_mm = sum(apertures) / n
    spread = sum((count - mean_count) ** 2 for count, _ in samples)
    if spread == 0.0:
        return ParallelYubiFitRefused(
            ParallelYubiFitRefusalReason.NO_TRAVEL, "every point has the same encoder count"
        )
    slope = sum((count - mean_count) * (mm - mean_mm) for count, mm in samples) / spread
    if slope == 0.0:
        return ParallelYubiFitRefused(
            ParallelYubiFitRefusalReason.NO_TRAVEL, "the aperture does not follow the encoder"
        )
    intercept = mean_mm - slope * mean_count
    residual = max(abs(intercept + slope * count - mm) for count, mm in samples)
    if residual > max_residual_mm:
        return ParallelYubiFitRefused(
            ParallelYubiFitRefusalReason.RESIDUAL_EXCEEDED,
            f"a point is {residual:.3f} mm off the fitted line; the bound is {max_residual_mm} mm",
        )
    return ParallelYubiCalibration(
        unit_id=unit_id,
        zero_count=-intercept / slope,
        mm_per_count=abs(slope),
        sign=1 if slope > 0.0 else -1,
        travel_mm=max(apertures),
        residual_mm=residual,
        residual_bound_mm=max_residual_mm,
    )


class ParallelYubiEncoder:
    """Interpret one Parallel YUBI UART stream against its unit's calibration.

    Every line must name its unit, and that unit must be the calibrated one: a reading
    from an unidentified or different unit is refused rather than given this law.
    """

    def __init__(self, calibration: ParallelYubiCalibration) -> None:
        self._law = ParallelYubiJaw(calibration)

    def parse(self, line: str) -> float | None:
        """Return the raw encoder count, or no sample for diagnostic text."""
        device_id, separator, value = line.partition(",")
        try:
            count = float(value if separator else line)
        except ValueError:
            return None
        if not separator:
            raise YubiEncoderError("Parallel YUBI readings must name their unit")
        if not math.isfinite(count):
            raise YubiEncoderError(f"encoder emitted a non-finite count: {count!r}")
        _fence_device_id(self._law.calibration.unit_id, device_id)
        self._law.aperture_m(count)
        return count


@dataclass(frozen=True, slots=True)
class StandardYubiJaw:
    """The four-bar law: encoder joint angle (rad) to jaw gap."""

    variant: Literal[YubiVariant.STANDARD] = YubiVariant.STANDARD

    def encoder(self) -> YubiEncoder:
        return YubiEncoder()

    def envelope(self) -> tuple[float, float]:
        return encoder_envelope_rad()

    def aperture_m(self, reading: float) -> float:
        return jaw_gap_m(reading)


@dataclass(frozen=True, slots=True)
class ParallelYubiJaw:
    """The parallel-jaw law: encoder count to jaw gap, from one unit's measured fit."""

    calibration: ParallelYubiCalibration
    variant: Literal[YubiVariant.PARALLEL] = YubiVariant.PARALLEL

    def encoder(self) -> ParallelYubiEncoder:
        return ParallelYubiEncoder(self.calibration)

    def envelope(self) -> tuple[float, float]:
        tolerance = self.calibration.residual_bound_mm
        ends = (
            self.calibration.count_at(-tolerance),
            self.calibration.count_at(self.calibration.travel_mm + tolerance),
        )
        return (min(ends), max(ends))

    def aperture_m(self, reading: float) -> float:
        mm = self.calibration.aperture_mm(reading)
        tolerance = self.calibration.residual_bound_mm
        if not -tolerance <= mm <= self.calibration.travel_mm + tolerance:
            raise YubiEncoderError(
                f"encoder count {reading!r} is {mm:.3f} mm, outside the measured travel "
                f"[0, {self.calibration.travel_mm}] mm of unit {self.calibration.unit_id!r}"
            )
        return min(max(mm, 0.0), self.calibration.travel_mm) / 1000.0


type YubiJawLaw = StandardYubiJaw | ParallelYubiJaw


class YubiJawUnavailableReason(StrEnum):
    UNSUPPORTED_VARIANT = "unsupported_variant"
    UNCALIBRATED = "uncalibrated"
    CALIBRATION_NOT_APPLICABLE = "calibration_not_applicable"


@dataclass(frozen=True, slots=True)
class YubiJawUnavailable:
    reason: YubiJawUnavailableReason
    detail: str


def yubi_jaw_law(
    variant: str, calibration: ParallelYubiCalibration | None = None
) -> YubiJawLaw | YubiJawUnavailable:
    """Select the jaw law for a device's declared variant, or say why there is none.

    ``variant`` is the device's own declaration; an unknown one has no law. Parallel
    YUBI needs its unit's measured calibration, and Standard YUBI never takes one.
    """

    try:
        known = YubiVariant(variant)
    except ValueError:
        return YubiJawUnavailable(
            YubiJawUnavailableReason.UNSUPPORTED_VARIANT,
            f"YUBI variant {variant!r} has no measured jaw law",
        )
    if known is YubiVariant.STANDARD:
        if calibration is not None:
            return YubiJawUnavailable(
                YubiJawUnavailableReason.CALIBRATION_NOT_APPLICABLE,
                "Standard YUBI uses its four-bar law; a parallel-jaw calibration does not apply",
            )
        return StandardYubiJaw()
    if calibration is None:
        return YubiJawUnavailable(
            YubiJawUnavailableReason.UNCALIBRATED,
            "Parallel YUBI needs this unit's measured travel, zero and sign",
        )
    return ParallelYubiJaw(calibration)
