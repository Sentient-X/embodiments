"""Franka Panda family: the mobile panda_omron body and fixed-base Franka.

The canonical Menagerie MJCF supplies the portable description asset. RoboCasa/robosuite
scene names such as ``robot0_joint1`` remain consumer-local per the name boundary.
"""

from typing import Final

from ..assets import AssetFormat, AssetProvenance, AssetRole, PackagedAsset
from ..compose import Component, ComponentRole, MountFrame, _EmbodimentDefinition
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..parts import (
    ArmSpec,
    ControlRates,
    GripperSpec,
    MimicJoint,
    MobileBaseSpec,
    PhysicalSpec,
)
from .sources import menagerie

PANDA_MJCF: Final = PackagedAsset(
    relpath="menagerie/franka_emika_panda/panda.xml",
    sha256="96ad67da03710f17f798c9478fd9e9efdf24a3bf8359f05e456dd9fb158ea273",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=menagerie("franka_emika_panda/panda.xml", "Apache-2.0"),
    media_type="application/xml",
)
PANDA_URDF: Final = PackagedAsset(
    relpath="official/franka_panda/panda.urdf",
    sha256="668d8398e32164587fc2e9886b37d1a17a20d889cafe192f28ba245a3e82c24a",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/frankarobotics/franka_ros",
        revision="ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98",
        path="franka_description/robots/panda/panda.urdf.xacro",
        license_id="Apache-2.0",
        generator="xacro 2.1.1 hand=true",
    ),
    media_type="application/xml",
)

PANDA_ARM: Final = ArmSpec(
    part_id=PartId("panda-arm"),
    joint_names=(
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ),
    joint_lower=(-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973),
    joint_upper=(2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973),
    home_joints=(0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785),
    assets=(PANDA_URDF, PANDA_MJCF),
    # Franka Emika Panda datasheet (download.franka.de/Datasheet-EN.pdf).
    physical=PhysicalSpec(payload_kg=3.0, reach_m=0.855, mass_kg=18.0),
)

PANDA_GRIPPER: Final = GripperSpec(
    part_id=PartId("panda-gripper"),
    joint_names=("panda_finger_joint1",),
    joint_lower=(0.0,),
    joint_upper=(0.04,),
    travel_m=(0.0, 0.08),  # parallel jaw: aperture = 2 x finger stroke (0.04 m each)
    mimic_joints=(MimicJoint("panda_finger_joint2", of="panda_finger_joint1", multiplier=1.0),),
)

OMRON_BASE: Final = MobileBaseSpec(part_id=PartId("omron-ld"))  # commanded outside joint space

PANDA_OMRON_SPEC: Final = _EmbodimentDefinition(
    embodiment_id=EmbodimentName("panda_omron"),
    name="Franka Panda on Omron LD mobile base",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="panda", variant="omron"),
    attachments=(
        Component("base", OMRON_BASE, ComponentRole.BODY),
        Component("arm", PANDA_ARM, ComponentRole.BODY, MountFrame("base", "top_plate")),
        Component("gripper", PANDA_GRIPPER, ComponentRole.BODY, MountFrame("arm", "panda_link8")),
    ),
    rates=ControlRates(policy_hz=20.0),
)

FRANKA_SPEC: Final = _EmbodimentDefinition(
    embodiment_id=EmbodimentName("franka"),
    name="Franka Panda (fixed base)",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="panda"),
    attachments=(
        Component("arm", PANDA_ARM, ComponentRole.BODY),
        Component("gripper", PANDA_GRIPPER, ComponentRole.BODY, MountFrame("arm", "panda_link8")),
    ),
)
