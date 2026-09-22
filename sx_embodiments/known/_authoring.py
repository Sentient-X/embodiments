"""Private adapters from manufacturer tables into canonical joint layouts."""

from ..layout import (
    ActuationBinding,
    ActuatorBinding,
    Bounds,
    CoordinateUnit,
    JointAxis,
    JointLayout,
    ObservationBinding,
    Unbounded,
)


def bounded_layout(
    *,
    names: tuple[str, ...],
    units: tuple[CoordinateUnit, ...],
    lower: tuple[float, ...],
    upper: tuple[float, ...],
    actuators: tuple[ActuatorBinding, ...] | None = None,
    observations: tuple[ObservationBinding, ...] | None = None,
    actuations: tuple[ActuationBinding, ...] | None = None,
) -> JointLayout:
    """Zip one source table into axes immediately; no parallel vectors escape.

    ``actuators`` is the per-axis drive column for bodies whose motors are qualified
    products on a shared bus; integrated vendor arms omit it.
    """

    if not (len(names) == len(units) == len(lower) == len(upper)):
        raise ValueError("joint source columns must have equal lengths")
    if actuators is not None and len(actuators) != len(names):
        raise ValueError("joint source columns must have equal lengths")
    if observations is not None and len(observations) != len(names):
        raise ValueError("joint source columns must have equal lengths")
    if actuations is not None and len(actuations) != len(names):
        raise ValueError("joint source columns must have equal lengths")
    if actuators is not None and actuations is not None:
        raise ValueError("declare actuators or actuation relations, not both")
    bindings = actuators if actuators is not None else (None,) * len(names)
    observation_facts = observations if observations is not None else (None,) * len(names)
    actuation_facts = actuations if actuations is not None else (None,) * len(names)
    return JointLayout(
        tuple(
            JointAxis(
                name=name,
                unit=unit,
                bounds=Bounds(lo, hi),
                actuator=binding,
                observation=observation,
                actuation=actuation,
            )
            for name, unit, lo, hi, binding, observation, actuation in zip(
                names,
                units,
                lower,
                upper,
                bindings,
                observation_facts,
                actuation_facts,
                strict=True,
            )
        )
    )


def unbounded_layout(*, names: tuple[str, ...], units: tuple[CoordinateUnit, ...]) -> JointLayout:
    """Zip a source table whose limits are genuinely undeclared."""

    if len(names) != len(units):
        raise ValueError("joint source columns must have equal lengths")
    return JointLayout(
        tuple(
            JointAxis(name=name, unit=unit, bounds=Unbounded())
            for name, unit in zip(names, units, strict=True)
        )
    )
