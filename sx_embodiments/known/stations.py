"""The fully described PiperX teleoperation station used by the factory."""

from typing import Final

from ..compose import (
    EmbodimentDefinition,
    MountedOn,
    RootMount,
    body_component,
    leader_component,
    sensor_component,
)
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
        leader_component(
            "leader",
            DeviceSpec(PartId("piperx-leader"), "PiperX leader arm"),
        ),
        body_component("arm", PIPER_ARM),
        body_component("gripper", PIPER_GRIPPER, MountedOn("arm", "joint6")),
        sensor_component("front", UVC_MONO_60, RootMount("world")),
    ),
)
