"""Sentient humanoid hardware runtime.

The retained hardware URDF and motor-assignment table define the 25 executed channels.
``waist_rod_joint`` is feedback/passive, while neck/head are not in the motor assignment;
all three remain in the description assets but never enter the physical body layout. The attached
28-joint MJCF is a simulation description whose limits differ from the hardware URDF, so
the hardware URDF limits below win at the execution boundary.
"""

from typing import Final, Literal

from ..assets import AssetFormat, AssetProvenance, AssetRole, PackagedAsset
from ..compose import Component, ComponentRole, EmbodimentDefinition, MountFrame
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import ArmSpec, JointGroupSpec

SENTIENT_HUMANOID_URDF: Final = PackagedAsset(
    relpath="humanoid_pkg/urdf/urdf_robot_acc.urdf",
    sha256="e79678962ede3c9c6def69ac3e40f1ac67e59d41e5205f6356dd294bb88017a6",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Sentient-X/humanoid_pkg",
        revision="666bc6b5e00c76b8388a70fcc40a6987e0f1da7a",
        path="urdf/urdf_robot_acc.urdf",
        license_id="BSD-3-Clause",
    ),
    media_type="application/xml",
)
SENTIENT_HUMANOID_MJCF: Final = PackagedAsset(
    relpath="humanoid_pkg/mjcf/humanoid_28dof.xml",
    sha256="1276bbfd8731401160abe1698c988a6807fdb95f001c2ad9755f18e856b4c6a4",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Sentient-X/humanoid_pkg",
        revision="666bc6b5e00c76b8388a70fcc40a6987e0f1da7a",
        path="mjcf/humanoid_28dof.xml",
        license_id="BSD-3-Clause",
    ),
    media_type="application/xml",
)

SENTIENT_HUMANOID_TORSO: Final = JointGroupSpec(
    part_id=PartId("sentient-humanoid-torso"),
    joint_names=("torso_joint", "waist_roll_joint", "waist_pitch_joint"),
    joint_units=(CoordinateUnit.RADIAN,) * 3,
    joint_lower=(-1.570796326795, -0.270002435284, -0.767944870878),
    joint_upper=(1.570796326795, 0.270002435284, 0.767944870878),
    home_joints=(0.0, 0.0, 0.0),
)


def _humanoid_arm(side: Literal["left", "right"]) -> ArmSpec:
    shoulder_roll = (
        (-0.356919832033, 3.14159265359) if side == "left" else (-3.14159265359, 0.356919832033)
    )
    return ArmSpec(
        part_id=PartId(f"sentient-humanoid-{side}-arm"),
        joint_names=tuple(
            f"{side}_{name}_joint"
            for name in (
                "shoulder_pitch",
                "shoulder_roll",
                "elbow_yaw",
                "elbow_forearm_yaw",
                "wrist_yaw",
            )
        ),
        joint_units=(CoordinateUnit.RADIAN,) * 5,
        joint_lower=(
            -3.14159265359,
            shoulder_roll[0],
            -1.570796326795,
            -1.570796326795,
            -1.570796326795,
        ),
        joint_upper=(
            3.14159265359,
            shoulder_roll[1],
            1.570796326795,
            1.570796326795,
            1.570796326795,
        ),
        home_joints=(0.0,) * 5,
    )


def _humanoid_leg(side: Literal["left", "right"]) -> JointGroupSpec:
    return JointGroupSpec(
        part_id=PartId(f"sentient-humanoid-{side}-leg"),
        joint_names=tuple(
            f"{side}_{name}_joint"
            for name in ("hip_roll", "hip_pitch", "hip_yaw", "knee", "foot1", "foot2")
        ),
        joint_units=(CoordinateUnit.RADIAN,) * 6,
        joint_lower=(
            -0.279252680319,
            -2.094395102393,
            -1.570796326795,
            -1.535889741755,
            -0.698131700798,
            -0.436332312999,
        ),
        joint_upper=(
            1.919862177194,
            2.094395102393,
            1.570796326795,
            1.535889741755,
            0.698131700798,
            0.436332312999,
        ),
        home_joints=(0.0,) * 6,
    )


SENTIENT_HUMANOID_RIGHT_ARM: Final = _humanoid_arm("right")
SENTIENT_HUMANOID_LEFT_ARM: Final = _humanoid_arm("left")
SENTIENT_HUMANOID_RIGHT_LEG: Final = _humanoid_leg("right")
SENTIENT_HUMANOID_LEFT_LEG: Final = _humanoid_leg("left")

SENTIENT_HUMANOID_SPEC: Final = EmbodimentDefinition(
    embodiment_id=EmbodimentName("sentient-humanoid"),
    name="Sentient humanoid",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="sentient-humanoid"),
    attachments=(
        Component("torso", SENTIENT_HUMANOID_TORSO, ComponentRole.BODY),
        Component(
            "right_arm",
            SENTIENT_HUMANOID_RIGHT_ARM,
            ComponentRole.BODY,
            MountFrame("torso", "chest_link"),
        ),
        Component(
            "left_arm",
            SENTIENT_HUMANOID_LEFT_ARM,
            ComponentRole.BODY,
            MountFrame("torso", "chest_link"),
        ),
        Component("right_leg", SENTIENT_HUMANOID_RIGHT_LEG, ComponentRole.BODY),
        Component("left_leg", SENTIENT_HUMANOID_LEFT_LEG, ComponentRole.BODY),
    ),
    extra_assets=(SENTIENT_HUMANOID_URDF, SENTIENT_HUMANOID_MJCF),
)
