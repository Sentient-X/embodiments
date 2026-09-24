"""Piper UMI supplied CAD bundle; per-unit tracking/optical calibration is required.

Asset provenance identifies this packaged bundle, not an upstream CAD authority.
The operator authorized the repository and proprietary classification. Original
left/right mesh source URLs have not been supplied; none are fabricated here.
"""

from sx_contracts.assets import AssetFormat, AssetProvenance, AssetRole

from ..assets import packaged_asset
from ..compose import EmbodimentDefinition, RootMount, body_component
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import GripperSpec, MimicJoint
from ._authoring import bounded_layout
from ._piper_umi_assets import ASSET_IDENTITIES

PIPER_UMI_ASSETS = tuple(
    packaged_asset(
        relpath=f"piper_umi_description/{path}",
        sha256=digest,
        size_bytes=size,
        format=AssetFormat.URDF if path.endswith(".urdf") else AssetFormat.MESH,
        role=AssetRole.DESCRIPTION if path.endswith(".urdf") else AssetRole.GEOMETRY,
        provenance=AssetProvenance(
            repository="https://github.com/Sentient-X/embodiments",
            revision=f"sha256:{digest}",
            path=f"assets/piper_umi_description/{path}",
            license_id="LicenseRef-Sentient-Proprietary",
            generator="tools/compose_piper_umi_urdf.py" if path.endswith(".urdf") else None,
        ),
        media_type="application/xml" if path.endswith(".urdf") else "model/stl",
    )
    for path, digest, size in ASSET_IDENTITIES
)
PIPER_UMI_URDF = PIPER_UMI_ASSETS[0]
PIPER_UMI_JAW = GripperSpec(
    part_id=PartId("piper-umi-rack-jaw"),
    layout=bounded_layout(
        names=("slider_1",),
        units=(CoordinateUnit.METER,),
        lower=(0.0,),
        upper=(0.05,),
    ),
    # Native CAD slider coordinates are not encoder radians. The measured
    # encoder→aperture law lives in the episode calibration, not in a CAD curve.
    mimic_joints=(
        MimicJoint("slider_2", of="slider_1", multiplier=-1.0),
        MimicJoint("revolute_1", of="slider_1", multiplier=-100.0),
    ),
    assets=(PIPER_UMI_URDF,),
)
PIPER_UMI_SPEC = EmbodimentDefinition(
    name=EmbodimentName("piper-umi"),
    label="Piper UMI bimanual handheld capture rig",
    kind=EmbodimentKind.CAPTURE_RIG,
    lineage=Lineage(family="piper-umi", revision="supplied-cad-v6-holder-tracking"),
    attachments=(
        body_component("left_jaw", PIPER_UMI_JAW, RootMount("left_hand_root")),
        body_component("right_jaw", PIPER_UMI_JAW, RootMount("right_hand_root")),
    ),
    extra_assets=PIPER_UMI_ASSETS,
)
