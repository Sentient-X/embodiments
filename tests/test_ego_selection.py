from itertools import product

import numpy as np

from sx_embodiments._ego_selection import choose_postures


def _derivatives(q, t):
    derivatives = []
    for order in range(1, 4):
        q = order * np.diff(q, axis=0) / (t[order:] - t[:-order])[:, None]
        derivatives.append(q)
    return derivatives


def test_mixed_kinematics_match_exhaustive_optimization_on_irregular_source_clock():
    t = np.array([0, 60, 125, 195, 257, 325, 389, 455], dtype=np.int64) * 1_000_000
    seconds = t * 1e-9
    speed = np.array([10.0, 0.08])
    reference = np.array([0, 0, 0.3, 0.02, 0.28, 0, 0, 0])[:, None] * [1, 0.01]
    alternate = np.array([0, 0.02, 0.06, 0.10, 0.09, 0.05, 0.02, 0])[:, None] * [1, 0.01]
    tracking = np.column_stack((np.full(8, 0.1), np.zeros(8)))
    tracking[:2, 1] = 1.0
    eligible = np.array([1, 0, 1, 1, 1, 1, 1, 1], dtype=bool)
    measured = np.ones((len(t), 1), dtype=bool)
    result = choose_postures(t, 15, measured, reference, alternate, speed, tracking, eligible)
    assert result.any() and not result.all()
    options = np.stack((reference, alternate), axis=1)
    base = abs(_derivatives(reference, seconds)[2] / speed).max(1)
    scores = {}
    for bits in product(range(2), repeat=len(t)):
        path = np.array(bits)
        if np.any(path & ~eligible):
            continue
        v, _, jerk = _derivatives(options[np.arange(len(t)), path], seconds)
        largest = np.max(np.abs(jerk / speed), axis=1)
        if np.any(abs(v) > speed + 1.0e-9) or np.any(largest > base + 1.0e-7):
            continue
        scores[bits] = np.sum(tracking[np.arange(len(t)), path]) + np.sum(
            (largest * 0.075**3 / 0.1) ** 2
        )
    assert np.isclose(scores[tuple(result)], min(scores.values()))


def test_seams_rate_bounds_and_missing_observations_are_independent():
    t = np.arange(15, dtype=np.int64) * 66666667
    reference = np.zeros((len(t), 1))
    alternate = np.full_like(reference, 0.5)
    speed = np.array([0.1])
    present = np.ones((len(t), 1), dtype=bool)
    tracking = np.tile([1.0, 0.0], (len(t), 1))
    available = np.ones(len(t), dtype=bool)
    # The whole translated route fits both velocity and jerk, but changing
    # routes at any one exposure would violate the declared velocity bound.
    assert choose_postures(t, 15, present, reference, alternate, speed, tracking, available).all()
    available[4] = False
    assert not choose_postures(
        t, 15, present, reference, alternate, speed, tracking, available
    ).any()
    for break_kind in ("clock", "visibility"):
        clock, visible = t.copy(), present.copy()
        if break_kind == "clock":
            clock[7:] += 3 * 66666667
        else:
            visible[7:] = False
        smooth = np.sin(np.linspace(0, np.pi, 15))[:, None]
        for offset in (0.0, 1.0):
            preference = tracking.copy()
            preference[7:, 1] += offset
            result = choose_postures(
                clock, 15, visible, reference, smooth, speed, preference, available
            )
            assert not result[6:8].any()
            if offset == 0:
                first = result[:7].copy()
            else:
                np.testing.assert_equal(result[:7], first)
    assert not choose_postures(
        t[:3], 15, present[:3], reference[:3], alternate[:3], speed, tracking[:3], available[:3]
    ).any()
