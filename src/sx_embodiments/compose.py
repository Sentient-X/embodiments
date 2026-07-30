"""Composition: an embodiment is an ordered tuple of part attachments; views are derived.

Composites are built by helper functions returning attachment tuples (``so101_side("left")``),
never by nesting specs — Python composition of tuples IS the compositional model. The
declared attachment order is the wire order (see :mod:`sx_embodiments.layout`).
"""

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
class MountFrame:
    """Where a part attaches. Informational (FK/renderers); NEVER affects channel order."""

    parent_instance: str = ""  # "" = the world/root
    frame: str = ""  # frame/link name in the parent's description


@dataclass(frozen=True, slots=True)
class Component:
    """One named node in the embodiment graph, carrying its complete hardware facts."""

    instance: str
    part: Part
    role: ComponentRole
    mount: MountFrame = MountFrame()

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


_SENSOR_PARTS = (CameraSpec, ForceTorqueSpec)
_BODY_PARTS = (ArmSpec, JointGroupSpec, GripperSpec, MobileBaseSpec, DeviceSpec)


@dataclass(frozen=True, slots=True)
class EmbodimentDefinition:
    """Source record used to construct one public ``Embodiment``.

    Package-internal despite the public name: it is not exported from
    ``sx_embodiments``, and ``tools/check_embodiment_ownership.py`` keeps
    construction inside the ``known/`` registry.
    """

    name: EmbodimentName
    label: str
    kind: EmbodimentKind
    lineage: Lineage
    attachments: tuple[Component, ...]
    rates: ControlRates | None = None
    extra_assets: tuple[PackagedAsset, ...] = ()

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
        parent = component.mount.parent_instance
        if parent and parent not in seen:
            raise CompositionError(
                name,
                f"{component.instance!r} mounts on {parent!r}, which is not declared "
                "before it (components are topologically ordered by declaration)",
            )
        seen.add(component.instance)
        if isinstance(component.part, _SENSOR_PARTS):
            if component.role is not ComponentRole.SENSOR:
                raise CompositionError(
                    name, f"{component.instance!r}: sensor parts must use role SENSOR"
                )
        elif component.role is ComponentRole.SENSOR:
            raise CompositionError(
                name, f"{component.instance!r}: body parts cannot use role SENSOR"
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
        lower: tuple[float | None, ...]
        upper: tuple[float | None, ...]
        if isinstance(part, ArmSpec):
            names, units, kind = part.joint_names, part.joint_units, ChannelKind.ARM_JOINT
            lower, upper = part.joint_lower, part.joint_upper
        elif isinstance(part, JointGroupSpec):
            names, units, kind = part.joint_names, part.joint_units, ChannelKind.BODY_JOINT
            lower, upper = part.joint_lower, part.joint_upper
        elif isinstance(part, GripperSpec):
            names, units, kind = part.joint_names, part.joint_units, ChannelKind.GRIPPER
            lower, upper = part.joint_lower, part.joint_upper
        elif isinstance(part, MobileBaseSpec):
            names, units, kind = part.channel_names, part.channel_units, ChannelKind.BASE
            lower = (None,) * len(names)
            upper = (None,) * len(names)
        else:  # pragma: no cover - excluded by layout_declared()
            raise LayoutError(name, f"unlayoutable part {part!r}")
        base = len(coordinates)
        coordinates.extend(
            StateCoordinate(
                index=base + offset,
                instance=attachment.component_id,
                part_id=part.part_id,
                joint_name=name,
                kind=kind,
                unit=unit,
                lower=lo,
                upper=hi,
            )
            for offset, (name, unit, lo, hi) in enumerate(
                zip(names, units, lower, upper, strict=True)
            )
        )
    return StateSpace(coordinates=tuple(coordinates))


def camera_bindings(components: tuple[Component, ...]) -> tuple[CameraBinding, ...]:
    """Camera instances in declared order: the embodiment's canonical stream-name set."""
    return tuple(
        CameraBinding(
            name=a.instance,
            camera=a.part,
            parent_instance=a.mount.parent_instance,
            frame=a.mount.frame,
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
