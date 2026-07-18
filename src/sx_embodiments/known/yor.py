"""YOR bimanual mobile robot, using its full unwelded model as the description."""

from typing import Final, Literal

from ..assets import AssetFormat, AssetRole, PackagedAsset
from ..compose import Attachment, AttachmentRole, EmbodimentSpec, MountFrame
from ..identity import EmbodimentId, EmbodimentKind, Lineage, PartId
from ..parts import ArmSpec, JointGroupSpec, MobileBaseSpec

YOR_MJCF: Final = PackagedAsset(
    relpath="yor/robot.mjcf",
    sha256="8e9289b712938e1e8c72516ebd28c76c800e2937a4b49d22e60a124708848ee9",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    media_type="application/xml",
)

YOR_BASE: Final = MobileBaseSpec(
    part_id=PartId("yor-swerve-base"),
    channel_names=(
        "front_left_steer",
        "drive_front_left",
        "back_left_steer",
        "drive_back_left",
        "front_right_steer",
        "drive_front_right",
        "back_right_steer",
        "drive_back_right",
    ),
    assets=(YOR_MJCF,),
)
YOR_LIFT: Final = JointGroupSpec(
    part_id=PartId("yor-telescoping-lift"),
    joint_names=("Slider_1", "Slider_2"),
    joint_lower=(0.0, 0.0),
    joint_upper=(0.208, 0.208),
    home_joints=(0.0, 0.0),
)


def _yor_arm(side: Literal["left", "right"]) -> ArmSpec:
    wrist_home = 0.78 if side == "left" else -0.78
    return ArmSpec(
        part_id=PartId(f"yor-{side}-piper-arm"),
        joint_names=tuple(f"{side}_arm_joint{index}" for index in range(1, 7)),
        joint_lower=(-2.61, 0.0, -2.965, -1.74, -1.2, -3.0),
        joint_upper=(2.61, 3.13, 0.0, 1.74, 1.2, 3.0),
        home_joints=(0.0, 1.58065, -0.578175, 0.0, -0.912, wrist_home),
    )


YOR_LEFT_ARM: Final = _yor_arm("left")
YOR_RIGHT_ARM: Final = _yor_arm("right")

YOR_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("yor"),
    name="YOR bimanual mobile robot",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="yor"),
    attachments=(
        Attachment("base", YOR_BASE, AttachmentRole.BODY),
        Attachment(
            "lift",
            YOR_LIFT,
            AttachmentRole.BODY,
            MountFrame("base", "base_profile_short"),
        ),
        Attachment(
            "left_arm",
            YOR_LEFT_ARM,
            AttachmentRole.BODY,
            MountFrame("lift", "Lift_Top"),
        ),
        Attachment(
            "right_arm",
            YOR_RIGHT_ARM,
            AttachmentRole.BODY,
            MountFrame("lift", "Lift_Top"),
        ),
    ),
)
