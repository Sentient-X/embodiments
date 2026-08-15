"""Agilex Piper: the single-arm fleet body.

The joint box is the deployed canonical data (identical to the pre-recut ``PIPER``
constant auto_perfect safety derives from) — NOT the MuJoCo Menagerie ranges, which differ
slightly (menagerie joint3 lower -2.697 vs deployed -2.967, joint4 ±1.832 vs ±1.745,
joint6 ±3.14 vs ±2.094). The menagerie MJCF is attached as the sim description asset;
deployed limits win on conflict because live safety constraints are derived from them.
"""

from typing import Final

from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import (
    BaseMount,
    EmbodimentDefinition,
    MountedOn,
    MountKind,
    RootMount,
    body_component,
)
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import ArmSpec, ControlRates, GripperSpec, MimicJoint, PhysicalSpec
from ._authoring import bounded_layout
from .sources import menagerie

PIPER_MJCF: Final = packaged_asset(
    relpath="menagerie/agilex_piper/piper.xml",
    sha256="a7b5b5d3b2a68d5c553b2ee9665d54a422bd8bf1fa6f3251bc11834993d37098",
    size_bytes=14537,
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=menagerie("agilex_piper/piper.xml", "MIT"),
    media_type="application/xml",
)
PIPER_SOURCE_URDF: Final = packaged_asset(
    relpath="official/agilex_piper/piper_description.urdf",
    sha256="884c6536abe861105205cc58681fb069ba408e0673ab6b6222f4f06cdbc9dc9e",
    size_bytes=12865,
    format=AssetFormat.URDF,
    role=AssetRole.OTHER,
    provenance=AssetProvenance(
        repository="https://github.com/agilexrobotics/piper_ros",
        revision="ac41fcbcdda598f01b51cf6175ed9a24d0dacadc",
        path="src/piper_description/urdf/piper_description.urdf",
        license_id="MIT",
    ),
    media_type="application/xml",
)
PIPER_URDF: Final = packaged_asset(
    relpath="official/agilex_piper/piper_canonical.urdf",
    sha256="e4215e76bfcbd9665b7895ba567c68efc295256915717fd1c5f4a56b4874357b",
    size_bytes=10482,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/agilexrobotics/piper_ros",
        revision="ac41fcbcdda598f01b51cf6175ed9a24d0dacadc",
        path="src/piper_description/urdf/piper_description.urdf",
        license_id="MIT",
        generator="sx-embodiments/tools/compose_registered_urdfs.py: project the "
        "MJCF equality joint8=-joint7 into URDF mimic metadata",
    ),
    media_type="application/xml",
)

PIPER_ARM: Final = ArmSpec(
    part_id=PartId("piper-arm"),
    layout=bounded_layout(
        names=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"),
        units=(CoordinateUnit.RADIAN,) * 6,
        lower=(-2.6179, 0.0, -2.967, -1.745, -1.22, -2.09439),
        upper=(2.6179, 3.14, 0.0, 1.745, 1.22, 2.09439),
    ),
    home=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    # Grasp centre 0.45 m forward, 0.52 m up from the base, pointing straight down, with the
    # wrist roll at zero so its whole travel remains for task yaw on either side.
    ready=(0.0, 1.4547, -0.9060, 0.0, 1.1093, 0.0),
    assets=(PIPER_MJCF,),
    # AgileX PiPER datasheet (global.agilex.ai/products/piper).
    physical=PhysicalSpec(payload_kg=1.5, reach_m=0.626, mass_kg=4.2),
)

PIPER_GRIPPER: Final = GripperSpec(
    part_id=PartId("piper-gripper"),
    layout=bounded_layout(
        names=("joint7",),
        units=(CoordinateUnit.METER,),
        lower=(0.0,),
        upper=(0.035,),
    ),
    travel_m=(0.0, 0.07),  # parallel jaw: aperture = 2 x finger stroke (0.035 m each)
    # Midpoint of the finger pads along the mount frame's approach (+z) axis; the
    # fingertips reach 0.03 m further, the clearance a grasp must leave below the target.
    grasp_centre_m=(0.0, 0.0, 0.10503),
    mimic_joints=(MimicJoint("joint8", of="joint7", multiplier=-1.0),),
)

PIPER_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("piper"),
    label="Agilex Piper",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="piper"),
    attachments=(
        body_component("arm", PIPER_ARM, RootMount("base_link")),
        body_component("gripper", PIPER_GRIPPER, MountedOn("arm", "link6")),
    ),
    extra_assets=(PIPER_URDF,),
    rates=ControlRates(policy_hz=30.0),
    # Footprint measured from the menagerie base_link collision geometry (xy AABB).
    base_mount=BaseMount(
        kind=MountKind.BOLT_DOWN,
        frame="base_link",
        half_extents=(0.065, 0.036),
        centre=(-0.011, 0.0),
    ),
)
