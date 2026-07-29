"""The fully described PiperX teleoperation station used by the factory."""

from typing import Final

from ..compose import Component, ComponentRole, EmbodimentDefinition, MountFrame
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..parts import DeviceSpec
from .das import UVC_MONO_60
from .piper import PIPER_ARM, PIPER_GRIPPER

PIPERX_STATION_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("piperx-station"),
    label="PiperX single-arm teleop station",
    kind=EmbodimentKind.TELEOP_STATION,
    lineage=Lineage(family="piper", variant="piperx"),
    attachments=(
        Component(
            "leader",
            DeviceSpec(PartId("piperx-leader"), "PiperX leader arm"),
            ComponentRole.LEADER,
        ),
        Component("arm", PIPER_ARM, ComponentRole.BODY),
        Component("gripper", PIPER_GRIPPER, ComponentRole.BODY, MountFrame("arm", "joint6")),
        Component("front", UVC_MONO_60, ComponentRole.SENSOR, MountFrame(frame="world")),
    ),
)
