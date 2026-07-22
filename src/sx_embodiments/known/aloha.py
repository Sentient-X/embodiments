"""ALOHA 2 bimanual follower, pinned to the MuJoCo Menagerie model."""

from typing import Final, Literal

from ..assets import AssetFormat, AssetProvenance, AssetRole, PackagedAsset
from ..compose import Component, ComponentRole, MountFrame, _EmbodimentDefinition
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..parts import ArmSpec, GripperSpec, MimicJoint
from .sources import menagerie

ALOHA_MJCF: Final = PackagedAsset(
    relpath="menagerie/aloha/aloha.xml",
    sha256="68430b29719bda1b75e63f540953f81991bc3fd136bdf0a43bbe3e04393b78d3",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=menagerie("aloha/aloha.xml", "BSD-3-Clause"),
    media_type="application/xml",
)
ALOHA_URDF: Final = PackagedAsset(
    relpath="official/aloha/aloha.urdf",
    sha256="16dc0e1a2c84dac010ae629120afce5621e7201c87966ba2a91ff0e069de09a1",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Interbotix/interbotix_ros_manipulators",
        revision="0bb2b0e6d0e619bff02cf74dbd5af5681dcf80c9",
        path="interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf/vx300s.urdf.xacro",
        license_id="BSD-3-Clause",
        generator="xacro 2.1.1 + sx dual-arm composition v1",
    ),
    media_type="application/xml",
)

_ARM_LOWER: Final = (-3.14158, -1.85005, -1.76278, -3.14158, -1.8675, -3.14158)
_ARM_UPPER: Final = (3.14158, 1.25664, 1.6057, 3.14158, 2.23402, 3.14158)
_ARM_HOME: Final = (0.0, -0.96, 1.16, 0.0, -0.3, 0.0)


def _aloha_arm(side: Literal["left", "right"]) -> ArmSpec:
    return ArmSpec(
        part_id=PartId(f"aloha-{side}-arm"),
        joint_names=tuple(
            f"{side}/{name}"
            for name in (
                "waist",
                "shoulder",
                "elbow",
                "forearm_roll",
                "wrist_angle",
                "wrist_rotate",
            )
        ),
        joint_lower=_ARM_LOWER,
        joint_upper=_ARM_UPPER,
        home_joints=_ARM_HOME,
        assets=(ALOHA_URDF, ALOHA_MJCF),
    )


def _aloha_gripper(side: Literal["left", "right"]) -> GripperSpec:
    driven = f"{side}/left_finger"
    return GripperSpec(
        part_id=PartId(f"aloha-{side}-gripper"),
        joint_names=(driven,),
        joint_lower=(0.0,),
        joint_upper=(0.041,),
        travel_m=(0.0, 0.082),
        mimic_joints=(MimicJoint(f"{side}/right_finger", of=driven, multiplier=1.0),),
    )


ALOHA_LEFT_ARM: Final = _aloha_arm("left")
ALOHA_RIGHT_ARM: Final = _aloha_arm("right")
ALOHA_LEFT_GRIPPER: Final = _aloha_gripper("left")
ALOHA_RIGHT_GRIPPER: Final = _aloha_gripper("right")

ALOHA_SPEC: Final = _EmbodimentDefinition(
    embodiment_id=EmbodimentName("aloha"),
    name="ALOHA 2",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="aloha", variant="2"),
    attachments=(
        Component("left_arm", ALOHA_LEFT_ARM, ComponentRole.BODY),
        Component(
            "left_gripper",
            ALOHA_LEFT_GRIPPER,
            ComponentRole.BODY,
            MountFrame("left_arm", "left/gripper_link"),
        ),
        Component("right_arm", ALOHA_RIGHT_ARM, ComponentRole.BODY),
        Component(
            "right_gripper",
            ALOHA_RIGHT_GRIPPER,
            ComponentRole.BODY,
            MountFrame("right_arm", "right/gripper_link"),
        ),
    ),
)
