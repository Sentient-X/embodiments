"""A superseded body's current successor.

A robot is assigned a body by its content id, and the id changes whenever anything of the
body does: a corrected camera mount or wider joint limits re-identify the static upper body.
An assignment still naming the earlier id is the same robot when the earlier revision (kept
in ``historical/``) drives exactly as the current one: the same joints on the same actuator
models, buses, ports, units, senses and reductions. Its successor is then the current body of
that name, and the station opens it and Fleet renders its directory with no re-assignment.
A revision that rewired anything has no successor: the assignment must name the new body.
"""

from __future__ import annotations

from .embodiment import Embodiment
from .errors import UnknownEmbodimentError
from .identity import EmbodimentName
from .known import development_embodiments, embodiments
from .recorded import historical_document, historical_identities, read_recorded

#: One driven joint as its drive is wired: joint, actuator model, bus kind, port, whether
#: provisional, unit id on the bus, sense and reduction.
Wire = tuple[str, str, str, str, bool, int, int, float]


def wiring(body: Embodiment) -> tuple[Wire, ...]:
    """How ``body``'s drives are wired, in its coordinate order."""
    wires: list[Wire] = []
    for coordinate in body.state.coordinates:
        binding = coordinate.axis.actuator
        if binding is None:
            continue
        wires.append(
            (
                str(coordinate.joint_name),
                str(binding.model.value),
                str(binding.bus.value),
                str(binding.bus_port),
                bool(binding.provisional),
                int(binding.bus_id),
                int(binding.sign),
                float(binding.reduction),
            )
        )
    return tuple(wires)


def current(name: EmbodimentName) -> Embodiment:
    """The body ``name`` registers now, released or in development."""
    try:
        return embodiments[name]
    except UnknownEmbodimentError:
        return development_embodiments[name]


def superseded_ids(name: EmbodimentName) -> tuple[str, ...]:
    """The earlier ids of ``name`` whose revision drives as the current body does, sorted."""
    body = current(name)
    kept = []
    for identity in historical_identities(name):
        earlier = read_recorded(historical_document(name, identity)).embodiment
        if wiring(earlier) == wiring(body):
            kept.append(str(identity))
    return tuple(sorted(kept))


def successor(identity: str) -> Embodiment | None:
    """The current body an earlier id stands for, or ``None`` when no kept revision has that
    id or the revision drives differently."""
    for name in (*embodiments, *development_embodiments):
        if identity in {str(i) for i in historical_identities(name)}:
            return current(name) if identity in superseded_ids(name) else None
    return None


__all__ = ["Wire", "current", "successor", "superseded_ids", "wiring"]
