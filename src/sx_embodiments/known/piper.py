"""Agilex Piper: the single-arm fleet body.

The joint box is the deployed canonical data (identical to the pre-recut ``PIPER``
constant enpire safety derives from) — NOT the MuJoCo Menagerie ranges, which differ
slightly (menagerie joint3 lower -2.697 vs deployed -2.967, joint4 ±1.832 vs ±1.745,
joint6 ±3.14 vs ±2.094). The menagerie MJCF is attached as the sim description asset;
deployed limits win on conflict because live safety constraints are derived from them.
"""

from typing import Final

from ..assets import AssetFormat, AssetProvenance, AssetRole, PackagedAsset
from ..compose import Component, ComponentRole, MountFrame, _EmbodimentDefinition
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..parts import ArmSpec, ControlRates, GripperSpec, MimicJoint, PhysicalSpec
from .sources import menagerie

PIPER_MJCF: Final = PackagedAsset(
    relpath="menagerie/agilex_piper/piper.xml",
    sha256="a7b5b5d3b2a68d5c553b2ee9665d54a422bd8bf1fa6f3251bc11834993d37098",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=menagerie("agilex_piper/piper.xml", "MIT"),
    media_type="application/xml",
)
PIPER_URDF: Final = PackagedAsset(
    relpath="official/agilex_piper/piper_description.urdf",
    sha256="884c6536abe861105205cc58681fb069ba408e0673ab6b6222f4f06cdbc9dc9e",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/agilexrobotics/piper_ros",
        revision="ac41fcbcdda598f01b51cf6175ed9a24d0dacadc",
        path="src/piper_description/urdf/piper_description.urdf",
        license_id="MIT",
    ),
    media_type="application/xml",
)

PIPER_ARM: Final = ArmSpec(
    part_id=PartId("piper-arm"),
    joint_names=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"),
    joint_lower=(-2.6179, 0.0, -2.967, -1.745, -1.22, -2.09439),
    joint_upper=(2.6179, 3.14, 0.0, 1.745, 1.22, 2.09439),
    home_joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    assets=(PIPER_URDF, PIPER_MJCF),
    # AgileX PiPER datasheet (global.agilex.ai/products/piper).
    physical=PhysicalSpec(payload_kg=1.5, reach_m=0.626, mass_kg=4.2),
)

PIPER_GRIPPER: Final = GripperSpec(
    part_id=PartId("piper-gripper"),
    joint_names=("joint7",),
    joint_lower=(0.0,),
    joint_upper=(0.035,),
    travel_m=(0.0, 0.07),  # parallel jaw: aperture = 2 x finger stroke (0.035 m each)
    mimic_joints=(MimicJoint("joint8", of="joint7", multiplier=-1.0),),
)

PIPER_SPEC: Final = _EmbodimentDefinition(
    embodiment_id=EmbodimentName("piper"),
    name="Agilex Piper",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="piper"),
    attachments=(
        Component("arm", PIPER_ARM, ComponentRole.BODY),
        Component("gripper", PIPER_GRIPPER, ComponentRole.BODY, MountFrame("arm", "joint6")),
    ),
    rates=ControlRates(policy_hz=30.0),
)
