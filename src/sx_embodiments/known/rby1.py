"""Rainbow Robotics RBY1-M v1.3 with grippers."""

from typing import Final, Literal

from ..assets import AssetFormat, AssetRole, PackagedAsset
from ..compose import Attachment, AttachmentRole, EmbodimentSpec, MountFrame
from ..identity import EmbodimentId, EmbodimentKind, Lineage, PartId
from ..parts import ArmSpec, GripperSpec, JointGroupSpec, MimicJoint, MobileBaseSpec

RBY1_MJCF: Final = PackagedAsset(
    relpath="menagerie/rainbow_robotics_rby1/rby1m_1.3.xml",
    sha256="fa6d736f76e27de5aba22b96e0c98655ae6a0ad36f6ece52a915bf71b460c66c",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    media_type="application/xml",
)

RBY1_BASE: Final = MobileBaseSpec(
    part_id=PartId("rby1m-base"),
    channel_names=("wheel_fr", "wheel_fl", "wheel_rr", "wheel_rl"),
    assets=(RBY1_MJCF,),
)

RBY1_TORSO: Final = JointGroupSpec(
    part_id=PartId("rby1-torso"),
    joint_names=tuple(f"torso_{index}" for index in range(6)),
    joint_lower=(-0.349066, -1.0472, -2.47837, -0.785398, -0.523599, -1.5708),
    joint_upper=(0.349066, 1.52173, 1.5708, 1.5708, 0.523599, 1.5708),
    home_joints=(0.0,) * 6,
)


def _rby1_arm(side: Literal["left", "right"]) -> ArmSpec:
    second = (-0.05, 3.14159) if side == "left" else (-3.14159, 0.05)
    return ArmSpec(
        part_id=PartId(f"rby1-{side}-arm"),
        joint_names=tuple(f"{side}_arm_{index}" for index in range(7)),
        joint_lower=(-2.35619, second[0], -2.0944, -2.61799, -3.14159, -0.872665, -1.5708),
        joint_upper=(2.35619, second[1], 2.0944, 0.01, 3.14159, 0.872665, 1.5708),
        home_joints=(0.0,) * 7,
    )


def _rby1_gripper(side: Literal["left", "right"]) -> GripperSpec:
    suffix = "l" if side == "left" else "r"
    driven = f"gripper_finger_{suffix}1"
    return GripperSpec(
        part_id=PartId(f"rby1-{side}-gripper"),
        joint_names=(driven,),
        joint_lower=(-0.05,),
        joint_upper=(0.0,),
        mimic_joints=(MimicJoint(f"gripper_finger_{suffix}2", of=driven, multiplier=1.0),),
    )


RBY1_RIGHT_ARM: Final = _rby1_arm("right")
RBY1_LEFT_ARM: Final = _rby1_arm("left")
RBY1_HEAD: Final = JointGroupSpec(
    part_id=PartId("rby1-head"),
    joint_names=("head_0", "head_1"),
    joint_lower=(-1.57, -1.57),
    joint_upper=(1.57, 1.57),
    home_joints=(0.0, 0.0),
)
RBY1_RIGHT_GRIPPER: Final = _rby1_gripper("right")
RBY1_LEFT_GRIPPER: Final = _rby1_gripper("left")

RBY1_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("rby1"),
    name="Rainbow Robotics RBY1-M v1.3",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="rby1", variant="m", revision="1.3"),
    attachments=(
        Attachment("base", RBY1_BASE, AttachmentRole.BODY),
        Attachment("torso", RBY1_TORSO, AttachmentRole.BODY, MountFrame("base", "link_torso_0")),
        Attachment(
            "right_arm",
            RBY1_RIGHT_ARM,
            AttachmentRole.BODY,
            MountFrame("torso", "link_right_arm_0"),
        ),
        Attachment(
            "left_arm",
            RBY1_LEFT_ARM,
            AttachmentRole.BODY,
            MountFrame("torso", "link_left_arm_0"),
        ),
        Attachment("head", RBY1_HEAD, AttachmentRole.BODY, MountFrame("torso", "NECK_0")),
        Attachment(
            "right_gripper",
            RBY1_RIGHT_GRIPPER,
            AttachmentRole.BODY,
            MountFrame("right_arm", "link_right_arm_6"),
        ),
        Attachment(
            "left_gripper",
            RBY1_LEFT_GRIPPER,
            AttachmentRole.BODY,
            MountFrame("left_arm", "link_left_arm_6"),
        ),
    ),
)
