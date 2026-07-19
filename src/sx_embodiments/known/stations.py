"""The fully described PiperX teleoperation station used by the factory."""

from typing import Final

from ..compose import Attachment, AttachmentRole, EmbodimentSpec, MountFrame
from ..identity import EmbodimentId, EmbodimentKind, Lineage, PartId
from ..parts import DeviceSpec
from .das import UVC_MONO_60
from .piper import PIPER_ARM, PIPER_GRIPPER

PIPERX_STATION_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("piperx-station"),
    name="PiperX single-arm teleop station",
    kind=EmbodimentKind.TELEOP_STATION,
    lineage=Lineage(family="piper", variant="piperx"),
    attachments=(
        Attachment(
            "leader",
            DeviceSpec(PartId("piperx-leader"), "PiperX leader arm"),
            AttachmentRole.LEADER,
        ),
        Attachment("arm", PIPER_ARM, AttachmentRole.BODY),
        Attachment("gripper", PIPER_GRIPPER, AttachmentRole.BODY, MountFrame("arm", "joint6")),
        Attachment("front", UVC_MONO_60, AttachmentRole.SENSOR, MountFrame(frame="world")),
    ),
)
