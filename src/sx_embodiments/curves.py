"""Piecewise-linear scalar curves for measured hardware relationships.

The one current use is a gripper's aperture-vs-joint-angle forward-kinematic table (the
DAS jaw ``gap(q)`` lookup, previously duplicated across seven data-pipeline files). The
record is strictly monotonic in both axes so it is invertible without ambiguity.
"""

from bisect import bisect_left
from dataclasses import dataclass
from itertools import pairwise

from .errors import PartValidationError


def _strictly_monotonic(values: tuple[float, ...]) -> bool:
    increasing = all(a < b for a, b in pairwise(values))
    decreasing = all(a > b for a, b in pairwise(values))
    return increasing or decreasing


@dataclass(frozen=True, slots=True)
class Curve1D:
    """A validated piecewise-linear ``y(x)``; both axes strictly monotonic.

    ``at``/``inverse_at`` clamp outside the table (matching ``np.interp`` semantics so the
    data-pipeline call sites keep byte-identical behavior after adoption).
    """

    x: tuple[float, ...]
    y: tuple[float, ...]

    def __post_init__(self) -> None:
        if len(self.x) != len(self.y):
            raise PartValidationError("curve", "x and y must have equal length")
        if len(self.x) < 2:
            raise PartValidationError("curve", "a curve needs at least two knots")
        if not all(a < b for a, b in pairwise(self.x)):
            raise PartValidationError("curve", "x must be strictly increasing")
        if not _strictly_monotonic(self.y):
            raise PartValidationError("curve", "y must be strictly monotonic")

    def at(self, x: float) -> float:
        """Interpolate ``y(x)``, clamped to the table's ends."""
        return _interp(x, self.x, self.y)

    def inverse_at(self, y: float) -> float:
        """Interpolate ``x(y)`` — valid because ``y`` is strictly monotonic; clamped."""
        if self.y[0] < self.y[-1]:
            return _interp(y, self.y, self.x)
        return _interp(y, tuple(reversed(self.y)), tuple(reversed(self.x)))


def _interp(v: float, xp: tuple[float, ...], fp: tuple[float, ...]) -> float:
    if v <= xp[0]:
        return fp[0]
    if v >= xp[-1]:
        return fp[-1]
    hi = bisect_left(xp, v)
    lo = hi - 1
    span = xp[hi] - xp[lo]
    frac = (v - xp[lo]) / span
    return fp[lo] + frac * (fp[hi] - fp[lo])
