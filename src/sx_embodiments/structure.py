"""The component and frame graph derived from an embodiment composition."""

from dataclasses import dataclass
from enum import StrEnum

from sx_capabilities import Capability, ComponentId

from .compose import EmbodimentSpec
from .errors import ComponentGraphError
from .identity import PartId
from .parts import (
    ArmSpec,
    CameraModality,
    CameraSpec,
    DeviceSpec,
    ForceTorqueSpec,
    GripperSpec,
    JointGroupSpec,
    MobileBaseSpec,
)


class FrameId(str):
    """Named spatial frame declared by an embodiment descriptor."""

    def __new__(cls, value: str) -> "FrameId":
        if not value.strip() or any(char.isspace() for char in value):
            raise ComponentGraphError(
                f"frame_id must be non-empty and contain no spaces: {value!r}"
            )
        return super().__new__(cls, value)


class ComponentKind(StrEnum):
    MANIPULATOR = "manipulator"
    JOINT_GROUP = "joint_group"
    EFFECTOR = "effector"
    MOBILE_BASE = "mobile_base"
    CAMERA = "camera"
    FORCE_TORQUE_SENSOR = "force_torque_sensor"
    DEVICE = "device"


@dataclass(frozen=True, slots=True)
class RootComponent:
    component_id: ComponentId
    part_id: PartId
    kind: ComponentKind
    frame_id: FrameId
    capabilities: tuple[Capability, ...]


@dataclass(frozen=True, slots=True)
class MountedComponent:
    component_id: ComponentId
    part_id: PartId
    kind: ComponentKind
    frame_id: FrameId
    capabilities: tuple[Capability, ...]
    parent_component_id: ComponentId


Component = RootComponent | MountedComponent


def component_graph(spec: EmbodimentSpec) -> tuple[Component, ...]:
    """Derive a topologically ordered component graph from canonical attachments."""

    result: list[Component] = []
    seen: set[ComponentId] = set()
    for attachment in spec.attachments:
        component_id = ComponentId(attachment.instance)
        frame_id = FrameId(attachment.mount.frame or attachment.instance)
        part_id = attachment.part.part_id
        kind = _component_kind(attachment.part)
        capabilities = _capabilities_for_part(attachment.part)
        if attachment.mount.parent_instance:
            parent = ComponentId(attachment.mount.parent_instance)
            if parent not in seen:
                raise ComponentGraphError(
                    f"component {component_id!r} references undeclared parent {parent!r}"
                )
            result.append(
                MountedComponent(
                    component_id=component_id,
                    part_id=part_id,
                    kind=kind,
                    frame_id=frame_id,
                    capabilities=capabilities,
                    parent_component_id=parent,
                )
            )
        else:
            result.append(
                RootComponent(
                    component_id=component_id,
                    part_id=part_id,
                    kind=kind,
                    frame_id=frame_id,
                    capabilities=capabilities,
                )
            )
        seen.add(component_id)
    return tuple(result)


def _capabilities_for_part(part: object) -> tuple[Capability, ...]:
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


def _component_kind(part: object) -> ComponentKind:
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
