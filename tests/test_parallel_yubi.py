"""Parallel YUBI: a measured per-unit law, never the standard four-bar law.

No Parallel YUBI hardware was measured for these tests; the points are synthetic and
qualify nothing about a real unit.
"""

import pytest

from sx_embodiments.yubi import (
    ParallelYubiCalibration,
    ParallelYubiFitRefusalReason,
    ParallelYubiFitRefused,
    ParallelYubiJaw,
    StandardYubiJaw,
    YubiEncoderError,
    YubiJawUnavailable,
    YubiJawUnavailableReason,
    fit_parallel_yubi_calibration,
    jaw_gap_m,
    yubi_jaw_law,
)

# A unit whose count falls as the jaws open: 4000 counts closed, 0.02 mm per count.
_CLOSED = 4000.0
_SCALE = 0.02
_POINTS = [(_CLOSED - mm / _SCALE, mm) for mm in (0.0, 10.0, 25.0, 40.0, 55.0, 70.0)]


def _fit(points: list[tuple[float, float]] = _POINTS) -> ParallelYubiCalibration:
    fitted = fit_parallel_yubi_calibration("PY-007", points)
    assert isinstance(fitted, ParallelYubiCalibration)
    return fitted


def test_fit_recovers_travel_zero_and_sign_from_measured_points() -> None:
    noisy = [(count, mm + (0.1 if index % 2 else 0.0)) for index, (count, mm) in enumerate(_POINTS)]
    calibration = _fit(noisy)

    assert calibration.sign == -1
    assert calibration.mm_per_count == pytest.approx(_SCALE, rel=1e-2)
    assert calibration.zero_count == pytest.approx(_CLOSED, abs=10.0)
    assert calibration.travel_mm == pytest.approx(70.1)
    assert 0.0 < calibration.residual_mm <= calibration.residual_bound_mm


@pytest.mark.parametrize(
    ("points", "reason"),
    [
        (_POINTS[:3], ParallelYubiFitRefusalReason.TOO_FEW_SAMPLES),
        ([*_POINTS[:-1], (float("nan"), 70.0)], ParallelYubiFitRefusalReason.NOT_FINITE),
        ([*_POINTS[:-1], (500.0, -1.0)], ParallelYubiFitRefusalReason.NEGATIVE_APERTURE),
        ([*_POINTS[1:], (3000.0, 20.0)], ParallelYubiFitRefusalReason.NO_CLOSED_STOP),
        ([(4000.0 - i, i * 0.5) for i in range(6)], ParallelYubiFitRefusalReason.NO_TRAVEL),
        (
            [*_POINTS[:-1], (_CLOSED - 70.0 / _SCALE, 75.0)],
            ParallelYubiFitRefusalReason.RESIDUAL_EXCEEDED,
        ),
    ],
)
def test_a_bad_fit_is_refused(
    points: list[tuple[float, float]], reason: ParallelYubiFitRefusalReason
) -> None:
    refused = fit_parallel_yubi_calibration("PY-007", points)

    assert isinstance(refused, ParallelYubiFitRefused)
    assert refused.reason is reason


def test_parallel_law_is_linear_bounded_and_not_the_four_bar_law() -> None:
    law = yubi_jaw_law("parallel", _fit())
    assert isinstance(law, ParallelYubiJaw)

    assert law.aperture_m(_CLOSED) == pytest.approx(0.0)
    assert law.aperture_m(_CLOSED - 35.0 / _SCALE) == pytest.approx(0.035)
    assert law.aperture_m(_CLOSED - 35.0 / _SCALE) != pytest.approx(jaw_gap_m(0.035))
    low, high = law.envelope()
    assert low < _CLOSED - 70.0 / _SCALE < _CLOSED < high
    with pytest.raises(YubiEncoderError, match="outside the measured travel"):
        law.aperture_m(_CLOSED - 90.0 / _SCALE)


def test_parallel_encoder_reads_only_its_calibrated_unit() -> None:
    encoder = yubi_jaw_law("parallel", _fit())
    assert isinstance(encoder, ParallelYubiJaw)
    reader = encoder.encoder()

    assert reader.parse("PY-007,3000") == pytest.approx(3000.0)
    assert reader.parse("Magnet not detected") is None
    with pytest.raises(YubiEncoderError, match="name their unit"):
        reader.parse("3000")
    with pytest.raises(YubiEncoderError, match="changed"):
        reader.parse("PY-008,3000")
    with pytest.raises(YubiEncoderError, match="outside the measured travel"):
        reader.parse("PY-007,9000")


def test_law_selection_refuses_unknown_uncalibrated_and_crossed_variants() -> None:
    calibration = _fit()

    assert isinstance(yubi_jaw_law("standard"), StandardYubiJaw)
    unknown = yubi_jaw_law("yubi-mini")
    uncalibrated = yubi_jaw_law("parallel")
    crossed = yubi_jaw_law("standard", calibration)

    assert isinstance(unknown, YubiJawUnavailable)
    assert unknown.reason is YubiJawUnavailableReason.UNSUPPORTED_VARIANT
    assert isinstance(uncalibrated, YubiJawUnavailable)
    assert uncalibrated.reason is YubiJawUnavailableReason.UNCALIBRATED
    assert isinstance(crossed, YubiJawUnavailable)
    assert crossed.reason is YubiJawUnavailableReason.CALIBRATION_NOT_APPLICABLE
