"""Composition: an embodiment is an ordered tuple of part attachments; views are derived.

Composites are built by helper functions returning attachment tuples (``so101_side("left")``),
never by nesting specs — Python composition of tuples IS the compositional model. The
declared attachment order is the wire order (see :mod:`sx_embodiments.layout`).
"""

import math
from dataclasses import dataclass
from enum import StrEnum

from sx_contracts import Capability, ComponentId

from .assets import PackagedAsset
from .errors import ComponentGraphError, CompositionError, LayoutError
from .identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from .layout import ChannelKind, StateCoordinate, StateSpace
from .parts import (
    ArmSpec,
    CameraBinding,
    CameraModality,
    CameraSpec,
    ControlRates,
    DeviceSpec,
    ForceTorqueSpec,
    GripperSpec,
    JointGroupSpec,
    MobileBaseSpec,
    Part,
)


class ComponentRole(StrEnum):
    BODY = "body"  # contributes channels to the physical body-state vector
    LEADER = "leader"  # teleop input device: identity + assets, zero channels
    SENSOR = "sensor"  # cameras/FT: zero channels; cameras define the camera-name set


class ComponentKind(StrEnum):
    MANIPULATOR = "manipulator"
    JOINT_GROUP = "joint_group"
    EFFECTOR = "effector"
    MOBILE_BASE = "mobile_base"
    CAMERA = "camera"
    FORCE_TORQUE_SENSOR = "force_torque_sensor"
    DEVICE = "device"


@dataclass(frozen=True, slots=True)
class RootMount:
    frame: str

    def __post_init__(self) -> None:
        if not self.frame.strip():
            raise ComponentGraphError("root mount frame must not be empty")


@dataclass(frozen=True, slots=True)
class MountedOn:
    parent: str
    frame: str

    def __post_init__(self) -> None:
        if not self.parent.strip() or not self.frame.strip():
            raise ComponentGraphError("mounted components require parent and frame")


ComponentMount = RootMount | MountedOn


class MountKind(StrEnum):
    """How a body meets the world — one side of the stance table placement solves over."""

    BOLT_DOWN = "bolt_down"  # bolted flat onto a support surface (benchtop arms)
    FREE_STANDING = "free_standing"  # rests unanchored on a support (tripods, fixtures)
    MOBILE = "mobile"  # drives on the floor; placement adds a facing constraint
    CLAMP_EDGE = "clamp_edge"  # clamps to a support's edge segment


@dataclass(frozen=True, slots=True)
class BaseMount:
    """How this embodiment's root meets the world: the body's half of a placement.

    ``frame`` names the footprint link in the registered description; the footprint is
    the axis-aligned rectangle ``centre ± half_extents`` in that frame's xy plane
    (measured from the description's collision geometry, not eyeballed);
    ``clearance_m`` is the free margin placement must keep around it.
    """

    kind: MountKind
    frame: str
    half_extents: tuple[float, float]
    centre: tuple[float, float] = (0.0, 0.0)
    clearance_m: float = 0.0

    def __post_init__(self) -> None:
        if not self.frame.strip():
            raise ComponentGraphError("base mount frame must not be empty")
        if (
            len(self.half_extents) != 2
            or any(not math.isfinite(value) or value <= 0.0 for value in self.half_extents)
        ):
            raise ComponentGraphError("base mount half_extents must be two positive values")
        if len(self.centre) != 2 or any(not math.isfinite(value) for value in self.centre):
            raise ComponentGraphError("base mount centre must contain two finite values")
        if not math.isfinite(self.clearance_m) or self.clearance_m < 0.0:
            raise ComponentGraphError("base mount clearance_m must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class BodyAttachment:
    part: Part


@dataclass(frozen=True, slots=True)
class LeaderAttachment:
    part: Part


@dataclass(frozen=True, slots=True)
class SensorAttachment:
    part: CameraSpec | ForceTorqueSpec


ComponentAttachment = BodyAttachment | LeaderAttachment | SensorAttachment


@dataclass(frozen=True, slots=True)
class Component:
    """One named node; attachment kind owns role and part as one fact."""

    instance: str
    attachment: ComponentAttachment
    mount: ComponentMount

    @property
    def part(self) -> Part:
        return self.attachment.part

    @property
    def role(self) -> ComponentRole:
        if isinstance(self.attachment, BodyAttachment):
            return ComponentRole.BODY
        if isinstance(self.attachment, LeaderAttachment):
            return ComponentRole.LEADER
        return ComponentRole.SENSOR

    @property
    def component_id(self) -> ComponentId:
        return ComponentId(self.instance)

    @property
    def part_id(self) -> PartId:
        return self.part.part_id

    @property
    def kind(self) -> ComponentKind:
        return component_kind(self.part)

    @property
    def capabilities(self) -> tuple[Capability, ...]:
        return capabilities_for_part(self.part)


def body_component(
    instance: str,
    part: Part,
    mount: ComponentMount | None = None,
) -> Component:
    return Component(
        instance,
        BodyAttachment(part),
        mount if mount is not None else RootMount(instance),
    )


def leader_component(
    instance: str,
    part: Part,
    mount: ComponentMount | None = None,
) -> Component:
    return Component(
        instance,
        LeaderAttachment(part),
        mount if mount is not None else RootMount(instance),
    )


def sensor_component(
    instance: str,
    part: CameraSpec | ForceTorqueSpec,
    mount: ComponentMount | None = None,
) -> Component:
    return Component(
        instance,
        SensorAttachment(part),
        mount if mount is not None else RootMount(instance),
    )


_SENSOR_PARTS = (CameraSpec, ForceTorqueSpec)
_BODY_PARTS = (ArmSpec, JointGroupSpec, GripperSpec, MobileBaseSpec, DeviceSpec)


@dataclass(frozen=True, slots=True)
class EmbodimentDefinition:
    """Source record used to construct one public ``Embodiment``.

    Package-internal despite the public name: neither it nor the components it holds are
    exported from ``sx_embodiments``, so the only place a definition can be written is the
    ``known/`` registry beside the assets it names.
    """

    name: EmbodimentName
    label: str
    kind: EmbodimentKind
    lineage: Lineage
    attachments: tuple[Component, ...]
    rates: ControlRates | None = None
    extra_assets: tuple[PackagedAsset, ...] = ()
    base_mount: BaseMount | None = None

    def __post_init__(self) -> None:
        name = str(self.name)
        if not name.strip():
            raise CompositionError(name, "embodiment name must not be empty")
        if not self.label.strip():
            raise CompositionError(name, "label must not be empty")
        if not self.lineage.family.strip():
            raise CompositionError(name, "family must not be empty")
        validate_components(name, self.kind, self.attachments)

    def body_attachments(self) -> tuple[Component, ...]:
        return tuple(a for a in self.attachments if a.role is ComponentRole.BODY)

    def layout_declared(self) -> bool:
        """False when any body part's channel contribution is unknown (a DeviceSpec)."""
        return not any(isinstance(a.part, DeviceSpec) for a in self.body_attachments())


def validate_components(name: str, kind: EmbodimentKind, components: tuple[Component, ...]) -> None:
    """Validate one topologically ordered component graph."""
    if not components:
        raise CompositionError(name, "an embodiment needs at least one component")
    seen: set[str] = set()
    for component in components:
        if not component.instance.strip():
            raise CompositionError(name, "component names must not be empty")
        if component.instance in seen:
            raise CompositionError(name, f"duplicate component name {component.instance!r}")
        parent = component.mount.parent if isinstance(component.mount, MountedOn) else None
        if parent is not None and parent not in seen:
            raise CompositionError(
                name,
                f"{component.instance!r} mounts on {parent!r}, which is not declared "
                "before it (components are topologically ordered by declaration)",
            )
        seen.add(component.instance)
        if isinstance(component.part, _SENSOR_PARTS) and not isinstance(
            component.attachment, SensorAttachment
        ):
            raise CompositionError(
                name,
                f"{component.instance!r}: sensor parts need sensor attachment",
            )
        if not isinstance(component.part, _SENSOR_PARTS) and isinstance(
            component.attachment, SensorAttachment
        ):
            raise CompositionError(
                name, f"{component.instance!r}: non-sensor parts cannot use sensor attachment"
            )
    leaders = [component for component in components if component.role is ComponentRole.LEADER]
    if kind is not EmbodimentKind.TELEOP_STATION and leaders:
        raise CompositionError(name, f"{kind.value} embodiments cannot have leaders")
    if kind is EmbodimentKind.TELEOP_STATION:
        if not leaders:
            raise CompositionError(name, "a teleop station needs at least one leader")
        if not any(component.role is ComponentRole.BODY for component in components):
            raise CompositionError(name, "a teleop station needs a follower body")
    if kind is EmbodimentKind.ROBOT and not any(
        component.role is ComponentRole.BODY for component in components
    ):
        raise CompositionError(name, "a robot needs at least one body component")


def state_space(name: str, components: tuple[Component, ...]) -> StateSpace:
    """Derive the ordered native body-state space, or fail closed."""
    body = tuple(component for component in components if component.role is ComponentRole.BODY)
    if any(isinstance(component.part, DeviceSpec) for component in body):
        raise LayoutError(
            name,
            "channel layout is not declared (a body part has no captured description)",
        )
    coordinates: list[StateCoordinate] = []
    for attachment in body:
        part = attachment.part
        if isinstance(part, ArmSpec):
            kind = ChannelKind.ARM_JOINT
        elif isinstance(part, JointGroupSpec):
            kind = ChannelKind.BODY_JOINT
        elif isinstance(part, GripperSpec):
            kind = ChannelKind.GRIPPER
        elif isinstance(part, MobileBaseSpec):
            kind = ChannelKind.BASE
        else:  # pragma: no cover - excluded by layout_declared()
            raise LayoutError(name, f"unlayoutable part {part!r}")
        coordinates.extend(
            StateCoordinate(
                instance=attachment.component_id,
                part_id=part.part_id,
                axis=axis,
                kind=kind,
            )
            for axis in part.layout.axes
        )
    return StateSpace(coordinates=tuple(coordinates))


def camera_bindings(components: tuple[Component, ...]) -> tuple[CameraBinding, ...]:
    """Camera instances in declared order: the embodiment's canonical stream-name set."""
    return tuple(
        CameraBinding(
            name=a.instance,
            camera=a.part,
            mount=a.mount,
        )
        for a in components
        if isinstance(a.part, CameraSpec)
    )


def capabilities_for_part(part: object) -> tuple[Capability, ...]:
    if isinstance(part, ArmSpec):
        return (Capability.SPATIAL_MOTION_SE3,)
    if isinstance(part, GripperSpec):
        return (
            Capability.SPATIAL_MOTION_SE3,
            Capability.GRASP,
            Capability.GRASP_PARALLEL,
        )
    if isinstance(part, MobileBaseSpec):
        return (Capability.PLANAR_MOTION_SE2, Capability.LOCOMOTION_PLANAR)
    if isinstance(part, CameraSpec):
        if part.modality is CameraModality.RGB:
            return (Capability.SENSING_RGB,)
        if part.modality is CameraModality.DEPTH:
            return (Capability.SENSING_DEPTH,)
        return (Capability.SENSING_RGB, Capability.SENSING_DEPTH)
    if isinstance(part, ForceTorqueSpec):
        return (Capability.SENSING_FORCE_TORQUE,)
    return ()


def component_kind(part: object) -> ComponentKind:
    if isinstance(part, ArmSpec):
        return ComponentKind.MANIPULATOR
    if isinstance(part, JointGroupSpec):
        return ComponentKind.JOINT_GROUP
    if isinstance(part, GripperSpec):
        return ComponentKind.EFFECTOR
    if isinstance(part, MobileBaseSpec):
        return ComponentKind.MOBILE_BASE
    if isinstance(part, CameraSpec):
        return ComponentKind.CAMERA
    if isinstance(part, ForceTorqueSpec):
        return ComponentKind.FORCE_TORQUE_SENSOR
    if isinstance(part, DeviceSpec):
        return ComponentKind.DEVICE
    raise ComponentGraphError(f"unsupported embodiment part {part!r}")
