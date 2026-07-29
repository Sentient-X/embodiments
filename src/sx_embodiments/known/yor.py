"""YOR bimanual mobile robot, using its full unwelded model as the description."""

from typing import Final, Literal

from ..assets import AssetFormat, AssetProvenance, AssetRole, PackagedAsset
from ..compose import Component, ComponentRole, EmbodimentDefinition, MountFrame
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import ArmSpec, JointGroupSpec, MobileBaseSpec

YOR_MJCF: Final = PackagedAsset(
    relpath="yor/robot.mjcf",
    sha256="8e9289b712938e1e8c72516ebd28c76c800e2937a4b49d22e60a124708848ee9",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/YOR-robot/YOR",
        revision="2cc6dd2edcc7aa6d7f3d58e08bce7d2075d80cb6",
        path="robot/yor-description/yor_description/yor.mjcf",
        license_id="MIT",
    ),
    media_type="application/xml",
)
YOR_URDF: Final = PackagedAsset(
    relpath="official/yor/yor.urdf",
    sha256="3c65774c00643d455f840fc9d7f00e4c6b661d027cdfc1f2b2218230cb5c8124",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/YOR-robot/YOR",
        revision="2cc6dd2edcc7aa6d7f3d58e08bce7d2075d80cb6",
        path="robot/yor-description/robot.mjcf",
        license_id="MIT",
        generator="sx-embodiments mjcf_to_urdf v1",
    ),
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
    channel_units=(CoordinateUnit.RADIAN,) * 8,
    assets=(YOR_URDF, YOR_MJCF),
)
YOR_LIFT: Final = JointGroupSpec(
    part_id=PartId("yor-telescoping-lift"),
    joint_names=("Slider_1", "Slider_2"),
    joint_units=(CoordinateUnit.METER,) * 2,
    joint_lower=(0.0, 0.0),
    joint_upper=(0.208, 0.208),
    home_joints=(0.0, 0.0),
)


def _yor_arm(side: Literal["left", "right"]) -> ArmSpec:
    wrist_home = 0.78 if side == "left" else -0.78
    return ArmSpec(
        part_id=PartId(f"yor-{side}-piper-arm"),
        joint_names=tuple(f"{side}_arm_joint{index}" for index in range(1, 7)),
        joint_units=(CoordinateUnit.RADIAN,) * 6,
        joint_lower=(-2.61, 0.0, -2.965, -1.74, -1.2, -3.0),
        joint_upper=(2.61, 3.13, 0.0, 1.74, 1.2, 3.0),
        home_joints=(0.0, 1.58065, -0.578175, 0.0, -0.912, wrist_home),
    )


YOR_LEFT_ARM: Final = _yor_arm("left")
YOR_RIGHT_ARM: Final = _yor_arm("right")

YOR_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("yor"),
    label="YOR bimanual mobile robot",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="yor"),
    attachments=(
        Component("base", YOR_BASE, ComponentRole.BODY),
        Component(
            "lift",
            YOR_LIFT,
            ComponentRole.BODY,
            MountFrame("base", "base_profile_short"),
        ),
        Component(
            "left_arm",
            YOR_LEFT_ARM,
            ComponentRole.BODY,
            MountFrame("lift", "Lift_Top"),
        ),
        Component(
            "right_arm",
            YOR_RIGHT_ARM,
            ComponentRole.BODY,
            MountFrame("lift", "Lift_Top"),
        ),
    ),
)
