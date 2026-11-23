"""Select a continuous native-time sequence of whole, geometrically evaluated postures."""

from __future__ import annotations

from itertools import pairwise, product

import numpy as np
from numpy.typing import NDArray

from ._ego_temporal import TRACKING_TIME_S, native_gaps

type Floats = NDArray[np.float64]
type Booleans = NDArray[np.bool_]
type Indices = NDArray[np.int64]


def _span(
    poses: Floats, intervals: Floats, speed: Floats, tracking: Floats, available: Booleans
) -> Booleans:
    """Binary Viterbi over eight histories of three poses; the fourth fixes the jerk."""
    count = len(poses)
    if count < 4:
        return np.zeros(count, dtype=np.bool_)
    # Position differences are always in native joint units. Normalize by the
    # joint's declared velocity so a prismatic joint and a rotation can coexist.
    velocities = np.empty((count - 1, 2, 2, len(speed)))
    reachable = np.empty((count - 1, 2, 2), dtype=np.bool_)
    for left, right in product(range(2), repeat=2):
        delta = poses[1:, right] - poses[:-1, left]
        reachable[:, left, right] = np.all(
            abs(delta) <= intervals[:, None] * speed * (1 + 1e-9) + 1e-10, axis=1
        )
        velocities[:, left, right] = delta / (intervals[:, None] * speed)
    accel_dt = (intervals[:-1] + intervals[1:]) / 2
    jerk_dt = (intervals[:-2] + intervals[1:-1] + intervals[2:]) / 3
    candidates = np.empty((count - 3, 16))
    for pattern, (a, b, c, d) in enumerate(product(range(2), repeat=4)):
        first = (velocities[1:-1, b, c] - velocities[:-2, a, b]) / accel_dt[:-1, None]
        second = (velocities[2:, c, d] - velocities[1:-1, b, c]) / accel_dt[1:, None]
        # Dimensionless displacement over the tracking time relative to one
        # tenth second of rated travel, the same coordinate metric as the IK.
        jerk = (second - first) / jerk_dt[:, None]
        largest = np.max(np.abs(jerk), axis=1) * TRACKING_TIME_S**3 / 0.1
        candidates[:, pattern] = largest
    # At all four-frame histories, including where the solution changes branch,
    # the worst normalized jerk may not increase over the original achieved
    # reference. This comparison supplies a data-dependent limit in each span.
    bound = candidates[:, :1] * (1 + 1e-12) + 1e-12
    candidates[candidates > bound] = np.inf
    candidates **= 2

    history = np.full(8, np.inf)
    for state, (a, b, c) in enumerate(product(range(2), repeat=3)):
        if (
            available[0, a]
            and available[1, b]
            and available[2, c]
            and reachable[0, a, b]
            and reachable[1, b, c]
        ):
            history[state] = tracking[0, a] + tracking[1, b] + tracking[2, c]
    predecessors = np.empty((count - 3, 8), dtype=np.uint8)
    tails = np.arange(8)
    before = np.arange(16) >> 1
    last = np.arange(16) & 1
    penultimate = (np.arange(16) >> 1) & 1
    for k in range(count - 3):
        scores = history[before] + candidates[k] + tracking[k + 3, last]
        scores[~(available[k + 3, last] & reachable[k + 2, penultimate, last])] = np.inf
        choices = scores.reshape(2, 8)
        best = np.argmin(choices, axis=0)
        predecessors[k] = before.reshape(2, 8)[best, tails]
        history = choices[best, tails]
    if not np.isfinite(history).any():
        raise ValueError("reference exceeds positional travel on its source clock")
    state = int(np.argmin(history))
    chosen = np.zeros(count, dtype=np.bool_)
    for k in range(count - 1, 2, -1):
        chosen[k] = bool(state & 1)
        state = int(predecessors[k - 3, state])
    chosen[:3] = [bool(state & 4), bool(state & 2), bool(state & 1)]
    return chosen


def choose_postures(
    times: Indices,
    fps: float | None,
    present: Booleans,
    reference: Floats,
    articulated: Floats,
    speed: Floats,
    tracking: Floats,
    eligible: Booleans,
) -> Booleans:
    """Keep whole postures; split at native clock gaps and hand evidence changes."""
    poses = np.stack((reference, articulated), axis=1)
    intervals = np.diff(times).astype(np.float64) * 1e-9
    breaks = native_gaps(intervals, fps) | np.any(present[1:] != present[:-1], axis=1)
    eligible = eligible.copy()
    # Anchor each side of a break at its original feasible pose. This also
    # preserves the native velocity constraint between independently fit spans.
    eligible[1:] &= ~breaks
    eligible[:-1] &= ~breaks
    available = np.column_stack((np.ones(len(times), dtype=np.bool_), eligible))
    chosen = np.zeros(len(times), dtype=np.bool_)
    edges = np.r_[0, np.flatnonzero(breaks) + 1, len(times)]
    for start, stop in pairwise(edges):
        chosen[start:stop] = _span(
            poses[start:stop],
            intervals[start : stop - 1],
            speed,
            tracking[start:stop],
            available[start:stop],
        )
    return chosen
