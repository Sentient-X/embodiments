"""Unitree G1 29-DOF humanoid without articulated hands."""

from typing import Final, Literal

from ..assets import AssetFormat, AssetProvenance, AssetRole, PackagedAsset
from ..compose import Component, ComponentRole, EmbodimentDefinition, MountFrame
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import ArmSpec, JointGroupSpec
from .sources import menagerie

UNITREE_G1_MJCF: Final = PackagedAsset(
    relpath="menagerie/unitree_g1/g1.xml",
    sha256="3c2616550a31f33e84d3c80b8e913ac5618c8888019b0c9490dae93493e647f3",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=menagerie("unitree_g1/g1.xml", "BSD-3-Clause"),
    media_type="application/xml",
)
UNITREE_G1_URDF: Final = PackagedAsset(
    relpath="official/unitree_g1/g1_29dof_rev_1_0.urdf",
    sha256="f751dbd8a0cdb653dc705cc8aaa36de6658054d0fb98faf18c0462a6707d20e5",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/unitreerobotics/unitree_ros",
        revision="d96d8f63ae17a7108d4f7229c00ef875ba7129c9",
        path="robots/g1_description/g1_29dof_rev_1_0.urdf",
        license_id="BSD-3-Clause",
    ),
    media_type="application/xml",
)


def _g1_leg(side: Literal["left", "right"]) -> JointGroupSpec:
    roll = (-0.5236, 2.9671) if side == "left" else (-2.9671, 0.5236)
    return JointGroupSpec(
        part_id=PartId(f"unitree-g1-{side}-leg"),
        joint_names=tuple(
            f"{side}_{name}_joint"
            for name in ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")
        ),
        joint_units=(CoordinateUnit.RADIAN,) * 6,
        joint_lower=(-2.5307, roll[0], -2.7576, -0.087267, -0.87267, -0.2618),
        joint_upper=(2.8798, roll[1], 2.7576, 2.8798, 0.5236, 0.2618),
        home_joints=(0.0,) * 6,
        assets=(UNITREE_G1_URDF, UNITREE_G1_MJCF) if side == "left" else (),
    )


def _g1_arm(side: Literal["left", "right"]) -> ArmSpec:
    shoulder_roll = (-1.5882, 2.2515) if side == "left" else (-2.2515, 1.5882)
    home_roll = 0.2 if side == "left" else -0.2
    return ArmSpec(
        part_id=PartId(f"unitree-g1-{side}-arm"),
        joint_names=tuple(
            f"{side}_{name}_joint"
            for name in (
                "shoulder_pitch",
                "shoulder_roll",
                "shoulder_yaw",
                "elbow",
                "wrist_roll",
                "wrist_pitch",
                "wrist_yaw",
            )
        ),
        joint_units=(CoordinateUnit.RADIAN,) * 7,
        joint_lower=(
            -3.0892,
            shoulder_roll[0],
            -2.618,
            -1.0472,
            -1.97222,
            -1.61443,
            -1.61443,
        ),
        joint_upper=(
            2.6704,
            shoulder_roll[1],
            2.618,
            2.0944,
            1.97222,
            1.61443,
            1.61443,
        ),
        home_joints=(0.2, home_roll, 0.0, 1.28, 0.0, 0.0, 0.0),
    )


UNITREE_G1_LEFT_LEG: Final = _g1_leg("left")
UNITREE_G1_RIGHT_LEG: Final = _g1_leg("right")
UNITREE_G1_TORSO: Final = JointGroupSpec(
    part_id=PartId("unitree-g1-torso"),
    joint_names=("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"),
    joint_units=(CoordinateUnit.RADIAN,) * 3,
    joint_lower=(-2.618, -0.52, -0.52),
    joint_upper=(2.618, 0.52, 0.52),
    home_joints=(0.0, 0.0, 0.0),
)
UNITREE_G1_LEFT_ARM: Final = _g1_arm("left")
UNITREE_G1_RIGHT_ARM: Final = _g1_arm("right")

UNITREE_G1_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("unitree-g1"),
    label="Unitree G1 (29 DOF)",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="unitree-g1", variant="29dof"),
    attachments=(
        Component("left_leg", UNITREE_G1_LEFT_LEG, ComponentRole.BODY),
        Component("right_leg", UNITREE_G1_RIGHT_LEG, ComponentRole.BODY),
        Component("torso", UNITREE_G1_TORSO, ComponentRole.BODY),
        Component(
            "left_arm",
            UNITREE_G1_LEFT_ARM,
            ComponentRole.BODY,
            MountFrame("torso", "torso_link"),
        ),
        Component(
            "right_arm",
            UNITREE_G1_RIGHT_ARM,
            ComponentRole.BODY,
            MountFrame("torso", "torso_link"),
        ),
    ),
)
