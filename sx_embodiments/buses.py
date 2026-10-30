"""Which physical bus each driven joint is on, and the joint's name on it.

An :class:`~sx_embodiments.layout.ActuatorBinding` names its bus by kind and port within its
part. A composed body may repeat a part: the bimanual SO-101 is two SO-101 arms, each with
servos 1 to 6 on the Feetech bus's default port. Composition admits that, because separately
rooted chains may reuse addresses (``compose._validate_actuator_addresses``), and two chains
reusing an address cannot share one wire, so each is its own bus. That bus is named here,
from the composition alone, so no id changes and no body spells it: a (kind, port) whose bus
ids repeat across chains is split per chain, each chain's port named by its root component
and the port it repeats (``left_arm_serial``), so two kinds of bus on one chain stay apart.
Every other port is the binding's own, so a body that reuses no address reads as it did.

A driven joint is named as the description names it (``left_shoulder_pan``), which is unique
across the body where a part's own joint name (``shoulder_pan``) is not.
"""

from __future__ import annotations

from dataclasses import dataclass

from .compose import MountedOn
from .embodiment import Embodiment
from .errors import EmbodimentError
from .layout import ActuatorBinding, ActuatorBus, BusPort, StateCoordinate


@dataclass(frozen=True, slots=True)
class DrivenAxis:
    """One driven coordinate: its description joint name, its binding and its bus's port."""

    coordinate: StateCoordinate
    joint: str
    binding: ActuatorBinding
    port: BusPort


def driven_axes(body: Embodiment) -> tuple[DrivenAxis, ...]:
    """``body``'s driven coordinates in native state order, each on its physical bus."""
    roots: dict[str, str] = {}
    for component in body.components:
        mount = component.mount
        instance = str(component.instance)
        roots[instance] = roots[mount.parent] if isinstance(mount, MountedOn) else instance
    driven = [
        (coordinate, joint, coordinate.axis.actuator)
        for coordinate, joint in zip(body.state.coordinates, body.joint_names, strict=True)
        if coordinate.axis.actuator is not None
    ]
    chains: dict[tuple[ActuatorBus, BusPort, int], set[str]] = {}
    for coordinate, _, binding in driven:
        key = (binding.bus, binding.bus_port, binding.bus_id)
        chains.setdefault(key, set()).add(roots[str(coordinate.instance)])
    split = {(bus, port) for (bus, port, _), held in chains.items() if len(held) > 1}
    axes: list[DrivenAxis] = []
    for coordinate, joint, binding in driven:
        port = binding.bus_port
        if (binding.bus, port) in split:
            port = BusPort(f"{roots[str(coordinate.instance)]}_{port}")
        axes.append(DrivenAxis(coordinate, joint, binding, port))
    kept = {axis.port for axis in axes if axis.port == axis.binding.bus_port}
    split_ports = {axis.port for axis in axes if axis.port != axis.binding.bus_port}
    if clash := split_ports & kept:
        raise EmbodimentError(f"{body.name}: a split bus's port {sorted(clash)} is already named")
    return tuple(axes)


__all__ = ["DrivenAxis", "driven_axes"]
