"""YOR bimanual mobile robot, using its full unwelded model as the description."""

from typing import Final, Literal

from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import Direction, EmbodimentDefinition, MountedOn, RootMount, body_component
from ..identity import EmbodimentId, EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..mechanical import Build, Module, ModuleId
from ..parts import ChainSpec, Limb, MobileBaseSpec
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
    sha256="3cfea922617d2c488a9076cc002b80ad551c8922e039cab3c78481c30b603c69",
    size_bytes=24584,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/YOR-robot/YOR",
        revision="2cc6dd2edcc7aa6d7f3d58e08bce7d2075d80cb6",
        path="robot/yor-description/robot.mjcf",
        license_id="MIT",
        generator="sx-embodiments mjcf_to_urdf v2",
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
YOR_LIFT: Final = ChainSpec(
    part_id=PartId("yor-telescoping-lift"),
    layout=bounded_layout(
        names=("Slider_1", "Slider_2"),
        units=(CoordinateUnit.METER,) * 2,
        lower=(0.0, 0.0),
        upper=(0.208, 0.208),
    ),
    home=(0.0, 0.0),
)


def _yor_arm(side: Literal["left", "right"]) -> ChainSpec:
    wrist_home = 0.78 if side == "left" else -0.78
    return ChainSpec(
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

YOR_MODULE: Final = Module(
    module_id=ModuleId("yor"),
    label="YOR",
    description=YOR_URDF,
    components=(
        body_component("base", YOR_BASE, RootMount("base_profile_short")),
        body_component("lift", YOR_LIFT, MountedOn("base", "base_profile_short"), limb=Limb.LIFT),
        body_component("left_arm", YOR_LEFT_ARM, MountedOn("lift", "Lift_Top"), limb=Limb.ARM),
        body_component("right_arm", YOR_RIGHT_ARM, MountedOn("lift", "Lift_Top"), limb=Limb.ARM),
    ),
)

YOR_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("yor"),
    # Its schema-15 publication and the schema-16 body before mjcf_to_urdf v2: v1 copied
    # MJCF's intrinsic xyz euler into URDF's fixed-axis rpy, which turned the arm mounts,
    # grippers and wrist cameras. The joints and their wiring did not change.
    earlier_revisions=(
        EmbodimentId("c76ad4f8a10702d04788450f8795fb3eeb5aba4ed4ed328120664362566493b7"),
        EmbodimentId("6d21b329e04f7fa1af380db2fd97ea5264af6f3d69445cdb9af9c82038f23c06"),
    ),
    label="YOR bimanual mobile robot",
    kind=EmbodimentKind.ROBOT,
    front=Direction.POS_X,
    lineage=Lineage(family="yor"),
    build=Build.of(YOR_MODULE),
)
