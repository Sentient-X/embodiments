"""Teleop stations from the factory rig catalog: Seedstudio B601, PiperX, Sentient A1/A2.

Leader arms are ordinary part attachments with role LEADER (zero channels) — a leader is a
role in a composition, not a part type. Followers whose kinematic descriptions have not
been captured yet are :class:`~sx_embodiments.parts.DeviceSpec` bodies: those stations are
registered (identity, cameras, capabilities) but their channel layouts stay undeclared
until a real description lands. The PiperX follower IS the Piper body, so that station is
fully kinematic today.
"""

from typing import Final

from ..compose import Attachment, AttachmentRole, EmbodimentSpec, MountFrame
from ..identity import EmbodimentId, EmbodimentKind, Lineage, PartId
from ..parts import CameraModality, CameraSpec, DeviceSpec, ForceTorqueSpec, SensorModel
from .das import UVC_MONO_60
from .piper import PIPER_ARM, PIPER_GRIPPER

D435_30: Final = CameraSpec(
    part_id=PartId("realsense-d435"),
    model=SensorModel.REALSENSE_D435,
    modality=CameraModality.RGBD,
    fps=30.0,
)

B601_LEADER: Final = DeviceSpec(PartId("b601-leader"), "Seedstudio B601 leader pair")
B601_DM_FOLLOWER: Final = DeviceSpec(PartId("b601-dm-follower"), "B601-DM bimanual follower")
B601_RS_FOLLOWER: Final = DeviceSpec(PartId("b601-rs-follower"), "B601-RS bimanual follower")

B601_DM_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("b601-dm"),
    name="Seedstudio B601-DM teleop station",
    kind=EmbodimentKind.TELEOP_STATION,
    lineage=Lineage(family="b601", variant="dm"),
    attachments=(
        Attachment("leader", B601_LEADER, AttachmentRole.LEADER),
        Attachment("follower", B601_DM_FOLLOWER, AttachmentRole.BODY),
        Attachment("overhead", D435_30, AttachmentRole.SENSOR),
        Attachment("front", UVC_MONO_60, AttachmentRole.SENSOR),
    ),
)

B601_RS_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("b601-rs"),
    name="Seedstudio B601-RS teleop station",
    kind=EmbodimentKind.TELEOP_STATION,
    lineage=Lineage(family="b601", variant="rs"),
    attachments=(
        Attachment("leader", B601_LEADER, AttachmentRole.LEADER),
        Attachment("follower", B601_RS_FOLLOWER, AttachmentRole.BODY),
        Attachment("front", UVC_MONO_60, AttachmentRole.SENSOR),
    ),
)

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
        Attachment("front", UVC_MONO_60, AttachmentRole.SENSOR),
    ),
)

SENTIENT_A1_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("sentient-a1"),
    name="Sentient A1 single-arm teleop station",
    kind=EmbodimentKind.TELEOP_STATION,
    lineage=Lineage(family="sentient-arm", variant="a1"),
    attachments=(
        Attachment(
            "leader", DeviceSpec(PartId("a1-leader"), "A1 leader arm"), AttachmentRole.LEADER
        ),
        Attachment(
            "follower", DeviceSpec(PartId("a1-arm"), "Sentient A1 arm"), AttachmentRole.BODY
        ),
        Attachment("overhead", D435_30, AttachmentRole.SENSOR),
    ),
)

SENTIENT_A2_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("sentient-a2"),
    name="Sentient A2 bimanual teleop station",
    kind=EmbodimentKind.TELEOP_STATION,
    lineage=Lineage(family="sentient-arm", variant="a2"),
    attachments=(
        Attachment(
            "leader", DeviceSpec(PartId("a2-leader"), "A2 leader pair"), AttachmentRole.LEADER
        ),
        Attachment(
            "follower",
            DeviceSpec(PartId("a2-bimanual"), "Sentient A2 bimanual"),
            AttachmentRole.BODY,
        ),
        Attachment("overhead", D435_30, AttachmentRole.SENSOR),
        Attachment("front", UVC_MONO_60, AttachmentRole.SENSOR),
        Attachment(
            "wrist_ft",
            ForceTorqueSpec(PartId("a2-wrist-ft")),
            AttachmentRole.SENSOR,
            MountFrame("follower", "wrist"),
        ),
    ),
)
