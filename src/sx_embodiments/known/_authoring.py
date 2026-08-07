"""Private adapters from manufacturer tables into canonical joint layouts."""

from ..layout import Bounds, CoordinateUnit, JointAxis, JointLayout, Unbounded


def bounded_layout(
    *,
    names: tuple[str, ...],
    units: tuple[CoordinateUnit, ...],
    lower: tuple[float, ...],
    upper: tuple[float, ...],
) -> JointLayout:
    """Zip one source table into axes immediately; no parallel vectors escape."""

    if not (len(names) == len(units) == len(lower) == len(upper)):
        raise ValueError("joint source columns must have equal lengths")
    return JointLayout(
        tuple(
            JointAxis(name=name, unit=unit, bounds=Bounds(lo, hi))
            for name, unit, lo, hi in zip(names, units, lower, upper, strict=True)
        )
    )


def unbounded_layout(
    *, names: tuple[str, ...], units: tuple[CoordinateUnit, ...]
) -> JointLayout:
    """Zip a source table whose limits are genuinely undeclared."""

    if len(names) != len(units):
        raise ValueError("joint source columns must have equal lengths")
    return JointLayout(
        tuple(
            JointAxis(name=name, unit=unit, bounds=Unbounded())
            for name, unit in zip(names, units, strict=True)
        )
    )
