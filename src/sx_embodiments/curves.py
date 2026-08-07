"""Piecewise-linear scalar curves with each knot owning both coordinates."""

import math
from bisect import bisect_left
from dataclasses import dataclass
from itertools import pairwise

from .errors import PartValidationError


@dataclass(frozen=True, slots=True)
class Knot:
    x: float
    y: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.x) or not math.isfinite(self.y):
            raise PartValidationError("curve", "knots must be finite")
        object.__setattr__(self, "x", float(self.x))
        object.__setattr__(self, "y", float(self.y))


@dataclass(frozen=True, slots=True)
class Curve1D:
    """A validated, invertible piecewise-linear curve."""

    knots: tuple[Knot, ...]

    def __post_init__(self) -> None:
        if len(self.knots) < 2:
            raise PartValidationError("curve", "a curve needs at least two knots")
        if not all(left.x < right.x for left, right in pairwise(self.knots)):
            raise PartValidationError("curve", "knot x values must be strictly increasing")
        outputs = tuple(knot.y for knot in self.knots)
        increasing = all(left < right for left, right in pairwise(outputs))
        decreasing = all(left > right for left, right in pairwise(outputs))
        if not (increasing or decreasing):
            raise PartValidationError("curve", "knot y values must be strictly monotonic")

    def at(self, x: float) -> float:
        return _interp(
            x,
            tuple(knot.x for knot in self.knots),
            tuple(knot.y for knot in self.knots),
        )

    def inverse_at(self, y: float) -> float:
        xs = tuple(knot.x for knot in self.knots)
        ys = tuple(knot.y for knot in self.knots)
        if ys[0] < ys[-1]:
            return _interp(y, ys, xs)
        return _interp(y, tuple(reversed(ys)), tuple(reversed(xs)))


def _interp(value: float, inputs: tuple[float, ...], outputs: tuple[float, ...]) -> float:
    if value <= inputs[0]:
        return outputs[0]
    if value >= inputs[-1]:
        return outputs[-1]
    upper = bisect_left(inputs, value)
    lower = upper - 1
    fraction = (value - inputs[lower]) / (inputs[upper] - inputs[lower])
    return outputs[lower] + fraction * (outputs[upper] - outputs[lower])
