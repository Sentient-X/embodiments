"""Capability projection from the canonical compositional embodiment specification."""

from sx_capabilities import (
    Capability,
    CapabilityProfile,
    ComponentCapabilities,
    ComponentId,
)

from .compose import EmbodimentSpec
from .parts import (
    ArmSpec,
    CameraModality,
    CameraSpec,
    ForceTorqueSpec,
    GripperSpec,
    MobileBaseSpec,
)


def capability_profile(spec: EmbodimentSpec) -> CapabilityProfile:
    """Derive component abilities from physical part facts; never store a second declaration."""

    components: list[ComponentCapabilities] = []
    for attachment in spec.attachments:
        capabilities = capabilities_for_part(attachment.part)
        if capabilities:
            components.append(
                ComponentCapabilities(
                    component_id=ComponentId(attachment.instance),
                    capabilities=capabilities,
                )
            )
    return CapabilityProfile(tuple(components))


def capabilities_for_part(part: object) -> tuple[Capability, ...]:
    """Return the physical capabilities implied by one typed part."""
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
