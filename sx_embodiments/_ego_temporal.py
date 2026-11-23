"""Native-clock, two-sided refinement of an offline scalar-joint reference."""

from __future__ import annotations

from bisect import bisect_right
from collections.abc import Callable
from itertools import pairwise
from types import ModuleType
from typing import cast

import numpy as np
import scipy.linalg as linalg  # pyright: ignore[reportMissingTypeStubs] -- SciPy has no stubs.
from numpy.typing import NDArray

from ._ego_collision import CollisionModel

type Floats = NDArray[np.float64]
type Booleans = NDArray[np.bool_]


def _smooth(positions: Floats, intervals: Floats) -> Floats:
    """Fit the seed while minimizing integrated squared angular/linear acceleration.

    Trapezoid integration and divided differences use exposure time in seconds.
    The 75 ms scale sets a tracking bandwidth, not an actuator safety limit.
    """
    n = len(positions)
    if n < 3:
        return positions.copy()
    before, after = intervals[:-1], intervals[1:]
    mass = np.r_[intervals[0], (before + after) / 2, intervals[-1]]
    if intervals.min() < 1e-4:
        # A few near-coincident exposures make normal equations ill-conditioned.
        # Fit on well-separated local knots, then evaluate on every original
        # timestamp. This preserves affine motion including at both endpoints.
        t = np.r_[0.0, np.cumsum(intervals)]
        start = [0]
        for i in range(1, n):
            if t[i] - t[start[-1]] >= 1e-4:
                start.append(i)
        groups = np.diff([*start, n])
        knots: Floats = np.asarray(np.add.reduceat(t, start) / groups, dtype=np.float64)
        positions_at_knots = np.add.reduceat(positions, start, axis=0) / groups[:, None]
        if len(start) == 1:
            design = np.column_stack((np.ones(n), t / t[-1]))
            return np.asarray(design @ np.linalg.lstsq(design, positions, rcond=None)[0])
        smoothed = _smooth(positions_at_knots, np.diff(knots))
        clock = tuple(float(value) for value in knots)
        left = np.array(
            [min(max(bisect_right(clock, float(v)) - 1, 0), len(clock) - 2) for v in t],
            dtype=np.int64,
        )
        fraction = (t - knots[left]) / (knots[left + 1] - knots[left])
        return smoothed[left] + fraction[:, None] * (smoothed[left + 1] - smoothed[left])
    weight = 0.075**4 * mass[1:-1]
    a = 2.0 / (before * (before + after))
    c = 2.0 / (after * (before + after))
    b = -a - c
    # Banded normal equations of the second-difference operator. Each of its
    # rows touches exactly three consecutive exposure timestamps.
    matrix = np.zeros((3, n))
    matrix[0] = mass
    for i, coefficient in enumerate((a, b, c)):
        matrix[0, i : i + n - 2] += weight * coefficient**2
    matrix[1, :-2] += weight * a * b
    matrix[1, 1:-1] += weight * b * c
    matrix[2, :-2] = weight * a * c
    return np.asarray(
        cast("ModuleType", linalg).solveh_banded(matrix, mass[:, None] * positions, lower=True),
        dtype=np.float64,
    )


def native_gaps(intervals: Floats, fps: float | None) -> Booleans:
    if fps is not None and (not np.isfinite(fps) or fps <= 0):
        raise ValueError("fps must be finite and positive")
    if not len(intervals):
        return np.empty(0, dtype=np.bool_)
    period = 1.0 / fps if fps is not None else min(float(intervals.min()), 0.1)
    return intervals > 1.5 * period


def regularize(
    seed: Floats,
    times: NDArray[np.int64],
    present: Booleans,
    influence: Booleans,
    lower: Floats,
    upper: Floats,
    speed: Floats,
    fps: float | None,
    correct: Callable[[int, Floats, Floats], Floats],
    full_q: Callable[[Floats], Floats],
    blocked_controls: Callable[[Booleans], Booleans],
    collision: CollisionModel | None,
) -> Floats:
    if seed.shape != (len(times), len(speed)) or not np.isfinite(seed).all():
        raise ValueError("seed requires a finite native position per joint and exposure")
    if np.any(seed < lower - 1e-9) or np.any(seed > upper + 1e-9):
        raise ValueError("seed exceeds joint position bounds")
    # Subtract int64 timestamps before conversion to preserve small intervals
    # even on an acquisition clock near the current Unix epoch.
    intervals = np.diff(times).astype(np.float64) * 1e-9
    travel = intervals[:, None] * speed
    if np.any(np.abs(np.diff(seed, axis=0)) > travel * (1.0 + 1e-9) + 1e-10):
        raise ValueError("seed exceeds the joint velocity bounds on its source clock")
    gap = native_gaps(intervals, fps)
    if len(times) < 3:
        return seed.copy()
    # Unknown long intervals cannot establish continuity. Known native rates
    # also support deliberately slow acquisition without this fallback ceiling.
    wanted = seed.copy()
    # A hand disappearing must not split a completely independent hand. A
    # shared joint uses the availability of every target affecting that joint.
    for group in np.unique(influence, axis=1).T:
        columns = np.all(influence == group[:, None], axis=0)
        evidence = present[:, group]
        transitions = np.any(evidence[:-1] != evidence[1:], axis=1)
        edges = np.r_[0, np.flatnonzero(gap | transitions) + 1, len(times)]
        for start, stop in pairwise(edges):
            if stop - start >= 3:
                wanted[start:stop, columns] = _smooth(
                    seed[start:stop, columns], intervals[start : stop - 1]
                )
    # A timestamp hole has no evidence for blending the two endpoints.
    isolated = np.r_[False, gap] | np.r_[gap, False]
    for i in range(len(times)):
        if isolated[i]:
            wanted[i] = seed[i]
        elif np.any(np.abs(wanted[i] - seed[i]) > 1e-9):
            wanted[i] = correct(i, seed[i], wanted[i])
    result = seed.copy()
    # Each subpass updates non-adjacent rows with both neighbours fixed.
    # All accepted steps obey each neighbour's native velocity envelope.
    for parity in (0, 1, 0, 1):
        for i in range(parity, len(times), 2):
            lo, hi = lower, upper
            if i:
                lo = np.maximum(lo, result[i - 1] - travel[i - 1])
                hi = np.minimum(hi, result[i - 1] + travel[i - 1])
            if i + 1 < len(times):
                lo = np.maximum(lo, result[i + 1] - travel[i])
                hi = np.minimum(hi, result[i + 1] + travel[i])
            candidate = np.clip(wanted[i], lo, hi)
            if np.max(np.abs(candidate - result[i])) < 1e-10:
                continue
            if collision is not None:
                contact = collision.at(full_q(candidate))
                locked = np.zeros(len(speed), dtype=np.bool_)
                while contact.unallowed > 1e-6:
                    blocked = locked | blocked_controls(contact.blocked_joints)
                    if np.array_equal(blocked, locked):
                        candidate = result[i]
                        break
                    locked = blocked
                    candidate = np.where(locked, result[i], candidate)
                    contact = collision.at(full_q(candidate))
            result[i] = candidate
    return result
