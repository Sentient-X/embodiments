"""Rainbow Robotics RBY1-M v1.3 with grippers."""

from typing import Final, Literal

from ..assets import AssetFormat, AssetProvenance, AssetRole, PackagedAsset
from ..compose import Component, ComponentRole, EmbodimentDefinition, MountFrame
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import ArmSpec, GripperSpec, JointGroupSpec, MimicJoint, MobileBaseSpec
from .sources import menagerie

RBY1_MJCF: Final = PackagedAsset(
    relpath="menagerie/rainbow_robotics_rby1/rby1m_1.3.xml",
    sha256="fa6d736f76e27de5aba22b96e0c98655ae6a0ad36f6ece52a915bf71b460c66c",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=menagerie("rainbow_robotics_rby1/rby1m_1.3.xml", "Apache-2.0"),
    media_type="application/xml",
)
RBY1_URDF: Final = PackagedAsset(
    relpath="official/rby1/rby1m_v1.3.urdf",
    sha256="66f4ad14779793e94df87b59356321190c9882766b2ed9cbeba6624c0880f11d",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/RainbowRobotics/rby1-sdk",
        revision="38df3267e617d22644f6686e8a7e3c4eac3ce2ee",
        path="models/rby1m/urdf/model_v1.3.urdf",
        license_id="Apache-2.0",
    ),
    media_type="application/xml",
)

RBY1_BASE: Final = MobileBaseSpec(
    part_id=PartId("rby1m-base"),
    channel_names=("wheel_fr", "wheel_fl", "wheel_rr", "wheel_rl"),
    channel_units=(CoordinateUnit.RADIAN,) * 4,
    assets=(RBY1_URDF, RBY1_MJCF),
)

RBY1_TORSO: Final = JointGroupSpec(
    part_id=PartId("rby1-torso"),
    joint_names=tuple(f"torso_{index}" for index in range(6)),
    joint_units=(CoordinateUnit.RADIAN,) * 6,
    joint_lower=(-0.349066, -1.0472, -2.47837, -0.785398, -0.523599, -1.5708),
    joint_upper=(0.349066, 1.52173, 1.5708, 1.5708, 0.523599, 1.5708),
    home_joints=(0.0,) * 6,
)


def _rby1_arm(side: Literal["left", "right"]) -> ArmSpec:
    second = (-0.05, 3.14159) if side == "left" else (-3.14159, 0.05)
    return ArmSpec(
        part_id=PartId(f"rby1-{side}-arm"),
        joint_names=tuple(f"{side}_arm_{index}" for index in range(7)),
        joint_units=(CoordinateUnit.RADIAN,) * 7,
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
        joint_units=(CoordinateUnit.RADIAN,),
        joint_lower=(-0.05,),
        joint_upper=(0.0,),
        mimic_joints=(MimicJoint(f"gripper_finger_{suffix}2", of=driven, multiplier=1.0),),
    )


RBY1_RIGHT_ARM: Final = _rby1_arm("right")
RBY1_LEFT_ARM: Final = _rby1_arm("left")
RBY1_HEAD: Final = JointGroupSpec(
    part_id=PartId("rby1-head"),
    joint_names=("head_0", "head_1"),
    joint_units=(CoordinateUnit.RADIAN,) * 2,
    joint_lower=(-1.57, -1.57),
    joint_upper=(1.57, 1.57),
    home_joints=(0.0, 0.0),
)
RBY1_RIGHT_GRIPPER: Final = _rby1_gripper("right")
RBY1_LEFT_GRIPPER: Final = _rby1_gripper("left")

RBY1_SPEC: Final = EmbodimentDefinition(
    embodiment_id=EmbodimentName("rby1"),
    name="Rainbow Robotics RBY1-M v1.3",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="rby1", variant="m", revision="1.3"),
    attachments=(
        Component("base", RBY1_BASE, ComponentRole.BODY),
        Component("torso", RBY1_TORSO, ComponentRole.BODY, MountFrame("base", "link_torso_0")),
        Component(
            "right_arm",
            RBY1_RIGHT_ARM,
            ComponentRole.BODY,
            MountFrame("torso", "link_right_arm_0"),
        ),
        Component(
            "left_arm",
            RBY1_LEFT_ARM,
            ComponentRole.BODY,
            MountFrame("torso", "link_left_arm_0"),
        ),
        Component("head", RBY1_HEAD, ComponentRole.BODY, MountFrame("torso", "NECK_0")),
        Component(
            "right_gripper",
            RBY1_RIGHT_GRIPPER,
            ComponentRole.BODY,
            MountFrame("right_arm", "link_right_arm_6"),
        ),
        Component(
            "left_gripper",
            RBY1_LEFT_GRIPPER,
            ComponentRole.BODY,
            MountFrame("left_arm", "link_left_arm_6"),
        ),
    ),
)
