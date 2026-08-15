"""YOR bimanual mobile robot, using its full unwelded model as the description."""

from typing import Final, Literal

from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import EmbodimentDefinition, MountedOn, body_component
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import ArmSpec, JointGroupSpec, MobileBaseSpec
from ._authoring import bounded_layout, unbounded_layout

YOR_MJCF: Final = packaged_asset(
    relpath="yor/robot.mjcf",
    sha256="8e9289b712938e1e8c72516ebd28c76c800e2937a4b49d22e60a124708848ee9",
    size_bytes=28545,
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
YOR_URDF: Final = packaged_asset(
    relpath="official/yor/yor.urdf",
    sha256="b5242242eb182ac80af88c1ccafbf042adc82e72112519867bb4e808cf63ce38",
    size_bytes=24400,
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
    layout=unbounded_layout(
        names=(
            "front_left_steer",
            "drive_front_left",
            "back_left_steer",
            "drive_back_left",
            "front_right_steer",
            "drive_front_right",
            "back_right_steer",
            "drive_back_right",
        ),
        units=(CoordinateUnit.RADIAN,) * 8,
    ),
    assets=(YOR_URDF, YOR_MJCF),
)
YOR_LIFT: Final = JointGroupSpec(
    part_id=PartId("yor-telescoping-lift"),
    layout=bounded_layout(
        names=("Slider_1", "Slider_2"),
        units=(CoordinateUnit.METER,) * 2,
        lower=(0.0, 0.0),
        upper=(0.208, 0.208),
    ),
    home=(0.0, 0.0),
)


def _yor_arm(side: Literal["left", "right"]) -> ArmSpec:
    wrist_home = 0.78 if side == "left" else -0.78
    return ArmSpec(
        part_id=PartId(f"yor-{side}-piper-arm"),
        layout=bounded_layout(
            names=tuple(f"{side}_arm_joint{index}" for index in range(1, 7)),
            units=(CoordinateUnit.RADIAN,) * 6,
            lower=(-2.61, 0.0, -2.965, -1.74, -1.2, -3.0),
            upper=(2.61, 3.13, 0.0, 1.74, 1.2, 3.0),
        ),
        home=(0.0, 1.58065, -0.578175, 0.0, -0.912, wrist_home),
    )


YOR_LEFT_ARM: Final = _yor_arm("left")
YOR_RIGHT_ARM: Final = _yor_arm("right")

YOR_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("yor"),
    label="YOR bimanual mobile robot",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="yor"),
    attachments=(
        body_component("base", YOR_BASE),
        body_component("lift", YOR_LIFT, MountedOn("base", "base_profile_short")),
        body_component("left_arm", YOR_LEFT_ARM, MountedOn("lift", "Lift_Top")),
        body_component("right_arm", YOR_RIGHT_ARM, MountedOn("lift", "Lift_Top")),
    ),
)
