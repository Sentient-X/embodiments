"""Universal Robots UR5e and UR10e arms from MuJoCo Menagerie."""

from typing import Final

from ..assets import AssetFormat, AssetProvenance, AssetRole, PackagedAsset, packaged_asset
from ..compose import EmbodimentDefinition, body_component
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import ArmSpec
from ._authoring import bounded_layout
from .sources import menagerie

UR5E_MJCF: Final = packaged_asset(
    relpath="menagerie/universal_robots_ur5e/ur5e.xml",
    sha256="ffd0a7b336d3c2d27c35a15cc0d4aaa95196769aca190ac8d6522d3318c345a0",
    size_bytes=6652,
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=menagerie("universal_robots_ur5e/ur5e.xml", "BSD-3-Clause"),
    media_type="application/xml",
)
UR10E_MJCF: Final = packaged_asset(
    relpath="menagerie/universal_robots_ur10e/ur10e.xml",
    sha256="7495b8efe33e497ffe892b9279acb010671c2b4b5955f499aa2b1d320dd8c871",
    size_bytes=6938,
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=menagerie("universal_robots_ur10e/ur10e.xml", "BSD-3-Clause"),
    media_type="application/xml",
)
UR5E_URDF: Final = packaged_asset(
    relpath="official/universal_robots/ur5e.urdf",
    sha256="83b0eb2e97500cff0b947d37fd26428ebe3ab7d409c602dcd677488285bb3a9a",
    size_bytes=11981,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/UniversalRobots/Universal_Robots_ROS2_Description",
        revision="8c2adeb88e58efa6d5ae376631bd61be7ec49027",
        path="urdf/ur.urdf.xacro",
        license_id="BSD-3-Clause",
        generator="xacro 2.1.1 ur_type=ur5e",
    ),
    media_type="application/xml",
)
UR10E_URDF: Final = packaged_asset(
    relpath="official/universal_robots/ur10e.urdf",
    sha256="3e3b040f4bb12207d6e2ec95078108dbcd58e4d7d2e21a5a7806c9b8839a7a06",
    size_bytes=11877,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/UniversalRobots/Universal_Robots_ROS2_Description",
        revision="8c2adeb88e58efa6d5ae376631bd61be7ec49027",
        path="urdf/ur.urdf.xacro",
        license_id="BSD-3-Clause",
        generator="xacro 2.1.1 ur_type=ur10e",
    ),
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


def _ur_arm(model: str, urdf: PackagedAsset, mjcf: PackagedAsset) -> ArmSpec:
    return ArmSpec(
        part_id=PartId(f"{model}-arm"),
        layout=bounded_layout(
            names=_JOINT_NAMES,
            units=(CoordinateUnit.RADIAN,) * 6,
            lower=_JOINT_LOWER,
            upper=_JOINT_UPPER,
        ),
        home=_HOME,
        assets=(urdf, mjcf),
    )


UR5E_ARM: Final = _ur_arm("ur5e", UR5E_URDF, UR5E_MJCF)
UR10E_ARM: Final = _ur_arm("ur10e", UR10E_URDF, UR10E_MJCF)

UR5E_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("ur5e"),
    label="Universal Robots UR5e",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="universal-robots", variant="ur5e"),
    attachments=(body_component("arm", UR5E_ARM),),
)
UR10E_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("ur10e"),
    label="Universal Robots UR10e",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="universal-robots", variant="ur10e"),
    attachments=(body_component("arm", UR10E_ARM),),
)
