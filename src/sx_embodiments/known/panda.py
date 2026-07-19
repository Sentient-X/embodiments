"""Franka Panda family: the mobile panda_omron sim body, plus the bare-arm variants
(``franka``, ``libero_panda``) already used as ids by the data-catalog and train.

The canonical Menagerie MJCF supplies the portable description asset. RoboCasa/robosuite
scene names such as ``robot0_joint1`` remain consumer-local per the name boundary.
"""

from typing import Final

from ..assets import AssetFormat, AssetRole, PackagedAsset
from ..compose import Attachment, AttachmentRole, EmbodimentSpec, MountFrame
from ..identity import EmbodimentId, EmbodimentKind, Lineage, PartId
from ..parts import (
    ArmSpec,
    ControlRates,
    GripperSpec,
    MimicJoint,
    MobileBaseSpec,
    PhysicalSpec,
)

PANDA_MJCF: Final = PackagedAsset(
    relpath="menagerie/franka_emika_panda/panda.xml",
    sha256="96ad67da03710f17f798c9478fd9e9efdf24a3bf8359f05e456dd9fb158ea273",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    media_type="application/xml",
)

PANDA_ARM: Final = ArmSpec(
    part_id=PartId("panda-arm"),
    joint_names=(
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ),
    joint_lower=(-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973),
    joint_upper=(2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973),
    home_joints=(0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785),
    assets=(PANDA_MJCF,),
    # Franka Emika Panda datasheet (download.franka.de/Datasheet-EN.pdf).
    physical=PhysicalSpec(payload_kg=3.0, reach_m=0.855, mass_kg=18.0),
)

PANDA_GRIPPER: Final = GripperSpec(
    part_id=PartId("panda-gripper"),
    joint_names=("panda_finger_joint1",),
    joint_lower=(0.0,),
    joint_upper=(0.04,),
    travel_m=(0.0, 0.08),  # parallel jaw: aperture = 2 x finger stroke (0.04 m each)
    mimic_joints=(MimicJoint("panda_finger_joint2", of="panda_finger_joint1", multiplier=1.0),),
)

OMRON_BASE: Final = MobileBaseSpec(part_id=PartId("omron-ld"))  # commanded outside joint space

PANDA_OMRON_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("panda_omron"),
    name="Franka Panda on Omron LD mobile base",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="panda", variant="omron"),
    attachments=(
        Attachment("base", OMRON_BASE, AttachmentRole.BODY),
        Attachment("arm", PANDA_ARM, AttachmentRole.BODY, MountFrame("base", "top_plate")),
        Attachment("gripper", PANDA_GRIPPER, AttachmentRole.BODY, MountFrame("arm", "panda_link8")),
    ),
    rates=ControlRates(policy_hz=20.0),
)

FRANKA_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("franka"),
    name="Franka Panda (fixed base)",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="panda"),
    attachments=(
        Attachment("arm", PANDA_ARM, AttachmentRole.BODY),
        Attachment("gripper", PANDA_GRIPPER, AttachmentRole.BODY, MountFrame("arm", "panda_link8")),
    ),
)

LIBERO_PANDA_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("libero_panda"),
    name="Franka Panda (LIBERO benchmark)",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="panda", variant="libero"),
    attachments=(
        Attachment("arm", PANDA_ARM, AttachmentRole.BODY),
        Attachment("gripper", PANDA_GRIPPER, AttachmentRole.BODY, MountFrame("arm", "panda_link8")),
    ),
)
