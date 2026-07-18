"""Agilex Piper: the single-arm fleet body.

The joint box is the deployed canonical data (identical to the pre-recut ``PIPER``
constant enpire safety derives from) — NOT the MuJoCo Menagerie ranges, which differ
slightly (menagerie joint3 lower -2.697 vs deployed -2.967, joint4 ±1.832 vs ±1.745,
joint6 ±3.14 vs ±2.094). The menagerie MJCF is attached as the sim description asset;
deployed limits win on conflict because live safety constraints are derived from them.
"""

from typing import Final

from ..assets import AssetFormat, AssetRole, PackagedAsset
from ..compose import Attachment, AttachmentRole, EmbodimentSpec, MountFrame
from ..identity import EmbodimentId, EmbodimentKind, Lineage, PartId
from ..parts import ArmSpec, ControlRates, GripperSpec, MimicJoint

PIPER_MJCF: Final = PackagedAsset(
    relpath="menagerie/agilex_piper/piper.xml",
    sha256="a7b5b5d3b2a68d5c553b2ee9665d54a422bd8bf1fa6f3251bc11834993d37098",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    media_type="application/xml",
)

PIPER_ARM: Final = ArmSpec(
    part_id=PartId("piper-arm"),
    joint_names=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"),
    joint_lower=(-2.6179, 0.0, -2.967, -1.745, -1.22, -2.09439),
    joint_upper=(2.6179, 3.14, 0.0, 1.745, 1.22, 2.09439),
    home_joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    assets=(PIPER_MJCF,),
)

PIPER_GRIPPER: Final = GripperSpec(
    part_id=PartId("piper-gripper"),
    joint_names=("joint7",),
    joint_lower=(0.0,),
    joint_upper=(0.035,),
    travel_m=(0.0, 0.07),  # parallel jaw: aperture = 2 x finger stroke (0.035 m each)
    mimic_joints=(MimicJoint("joint8", of="joint7", multiplier=-1.0),),
)

PIPER_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("piper"),
    name="Agilex Piper",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="piper"),
    attachments=(
        Attachment("arm", PIPER_ARM, AttachmentRole.BODY),
        Attachment("gripper", PIPER_GRIPPER, AttachmentRole.BODY, MountFrame("arm", "joint6")),
    ),
    rates=ControlRates(policy_hz=30.0),
)
