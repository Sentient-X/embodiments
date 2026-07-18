"""Universal Robots UR5e and UR10e arms from MuJoCo Menagerie."""

from typing import Final

from ..assets import AssetFormat, AssetRole, PackagedAsset
from ..compose import Attachment, AttachmentRole, EmbodimentSpec
from ..identity import EmbodimentId, EmbodimentKind, Lineage, PartId
from ..parts import ArmSpec

UR5E_MJCF: Final = PackagedAsset(
    relpath="menagerie/universal_robots_ur5e/ur5e.xml",
    sha256="ffd0a7b336d3c2d27c35a15cc0d4aaa95196769aca190ac8d6522d3318c345a0",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    media_type="application/xml",
)
UR10E_MJCF: Final = PackagedAsset(
    relpath="menagerie/universal_robots_ur10e/ur10e.xml",
    sha256="7495b8efe33e497ffe892b9279acb010671c2b4b5955f499aa2b1d320dd8c871",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    media_type="application/xml",
)

_JOINT_NAMES: Final = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
_JOINT_LOWER: Final = (-6.28319, -6.28319, -3.1415, -6.28319, -6.28319, -6.28319)
_JOINT_UPPER: Final = (6.28319, 6.28319, 3.1415, 6.28319, 6.28319, 6.28319)
_HOME: Final = (-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0)


def _ur_arm(model: str, asset: PackagedAsset) -> ArmSpec:
    return ArmSpec(
        part_id=PartId(f"{model}-arm"),
        joint_names=_JOINT_NAMES,
        joint_lower=_JOINT_LOWER,
        joint_upper=_JOINT_UPPER,
        home_joints=_HOME,
        assets=(asset,),
    )


UR5E_ARM: Final = _ur_arm("ur5e", UR5E_MJCF)
UR10E_ARM: Final = _ur_arm("ur10e", UR10E_MJCF)

UR5E_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("ur5e"),
    name="Universal Robots UR5e",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="universal-robots", variant="ur5e"),
    attachments=(Attachment("arm", UR5E_ARM, AttachmentRole.BODY),),
)
UR10E_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("ur10e"),
    name="Universal Robots UR10e",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="universal-robots", variant="ur10e"),
    attachments=(Attachment("arm", UR10E_ARM, AttachmentRole.BODY),),
)
