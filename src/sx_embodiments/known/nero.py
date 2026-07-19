"""Agilex NERO: the 7-DoF single-arm body.

The joint box is copied verbatim from the authoritative URDF
(``agilexrobotics/agx_arm_urdf``). The pyAgxArm SDK documentation shows a slightly
different box in its custom-``joint_limits`` *example* (e.g. joint3 ±2.757621 vs the
URDF's ±2.75) — that snippet is an override example, not a datasheet, so the
description asset wins. Arm-only: upstream ships gripper/flange variants only as
xacro compositions with no plain URDF and no gripper datasheet, so no
``GripperSpec`` is claimed (the ur5e/ur10e precedent).
"""

from typing import Final

from ..assets import AssetFormat, AssetProvenance, AssetRole, PackagedAsset
from ..compose import Attachment, AttachmentRole, EmbodimentSpec
from ..identity import EmbodimentId, EmbodimentKind, Lineage, PartId
from ..parts import ArmSpec, PhysicalSpec

NERO_URDF: Final = PackagedAsset(
    relpath="official/agilex_nero/nero_description.urdf",
    sha256="c297c4bd2caeff44c673ae69070fc80f950510c0cb33cfa8b81b5bc774e91278",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/agilexrobotics/agx_arm_urdf",
        revision="f6642ce0d7872c686f29c99e9e10cd23d1d49313",
        path="nero/urdf/nero_description.urdf",
        license_id="MIT",
    ),
    media_type="application/xml",
)

NERO_ARM: Final = ArmSpec(
    part_id=PartId("nero-arm"),
    joint_names=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"),
    joint_lower=(-2.70526, -1.74, -2.75, -1.01, -2.75, -0.73, -1.5707963),
    joint_upper=(2.70526, 1.74, 2.75, 2.14, 2.75, 0.95, 1.5707963),
    home_joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    assets=(NERO_URDF,),
    # AgileX NERO datasheet (global.agilex.ai/products/nero).
    physical=PhysicalSpec(payload_kg=3.0, reach_m=0.58, mass_kg=4.8),
)

NERO_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("nero"),
    name="Agilex NERO",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="nero"),
    attachments=(Attachment("arm", NERO_ARM, AttachmentRole.BODY),),
)
