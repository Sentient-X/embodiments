"""The upstream YUBI bimanual handheld rig, held out of production pending calibration.

The authoritative kinematic description is the upstream YUBI hardware URDF
(airoa-org/yubi-sw ``yubi_description``, expanded from its xacro by
``tools/expand_yubi_xacro.py``): both hands, fin-ray finger pairs (one encoder-driven
joint per hand, the mirror finger mimics at -1), hand cameras, and the Quest controller
mount frames. Quest is a controller/tracking input in upstream YUBI, not a headset camera
pair. The jaw state is the URDF's driven ``right_finger_joint`` in radians; its mirrored
finger follows at -1 and its aperture curve is derived from the URDF's 109.43 mm finger
length and 1 mm closed half-gap. No DAS part or description asset participates in YUBI.
"""

from typing import Final

from sx_contracts.assets import AssetFormat, AssetProvenance, AssetRole

from ..assets import PackagedAsset, packaged_asset
from ..compose import (
    EmbodimentDefinition,
    OperatorMount,
    OperatorSite,
    RootMount,
    body_component,
)
from ..curves import Curve1D, Knot
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import GripperSpec, MimicJoint
from ._authoring import bounded_layout

YUBI_HANDS_URDF: Final = packaged_asset(
    relpath="yubi_description/urdf/yubi_hands.urdf",
    sha256="d88cb7de516a52fa348d480e5bf52e04976c0d560297d61f3663683022e62dbe",
    size_bytes=8545,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/airoa-org/yubi-sw",
        revision="b7423c31ba6d8ea6d536aca2988e8578751ffe66",
        path="yubi_description/urdf/yubi_hand.urdf.xacro",
        license_id="Apache-2.0",
        generator="sx-embodiments/tools/expand_yubi_xacro.py",
    ),
    media_type="application/xml",
)


def _yubi_mesh(
    relpath: str,
    sha256: str,
    size_bytes: int,
    role: AssetRole,
) -> PackagedAsset:
    return packaged_asset(
        relpath=f"yubi_description/meshes/{relpath}",
        sha256=sha256,
        size_bytes=size_bytes,
        format=AssetFormat.MESH,
        role=role,
        provenance=AssetProvenance(
            repository="https://github.com/airoa-org/yubi-sw",
            revision="b7423c31ba6d8ea6d536aca2988e8578751ffe66",
            path=f"yubi_description/meshes/{relpath}",
            license_id="Apache-2.0",
        ),
        media_type="model/stl",
    )


# Complete mesh closure of YUBI_HANDS_URDF. These are part of the embodiment's
# content identity, not merely files that happen to ship beside the URDF.
YUBI_MESHES: Final[tuple[PackagedAsset, ...]] = (
    _yubi_mesh(
        "hand_left_left_finger_link_col.STL",
        "17b1bae2ff0fec32f9ab1609401f2af32437e1adc0d3b2a9882f443e5709a55e",
        10684,
        AssetRole.COLLISION,
    ),
    _yubi_mesh(
        "hand_left_left_finger_link.STL",
        "df8be6e5d0d1b8de06f1dbf924df3629c26b8bade467d5e68e3218757385ed2d",
        132684,
        AssetRole.GEOMETRY,
    ),
    _yubi_mesh(
        "hand_left_right_finger_link_col.STL",
        "10573f7f04c40dae8e31684f1cc0e1545562ade37286b225ec206c47713c3bb8",
        10384,
        AssetRole.COLLISION,
    ),
    _yubi_mesh(
        "hand_left_right_finger_link.STL",
        "eac43eac280e6e3cfb1d8f21f9775e122d2ec7952b3e50bbacd14027ad5045e4",
        127784,
        AssetRole.GEOMETRY,
    ),
    _yubi_mesh(
        "hand_right_left_finger_link_col.STL",
        "3faa69f644941eebf54c10138b4e6a268c18c2dc1e46b5f1c5730d633fcac2e2",
        12384,
        AssetRole.COLLISION,
    ),
    _yubi_mesh(
        "hand_right_left_finger_link.STL",
        "05346512b5a4e13e7cd01255c61f1f4fc81409688f4d7b9027241eefd3b55811",
        127884,
        AssetRole.GEOMETRY,
    ),
    _yubi_mesh(
        "hand_right_right_finger_link_col.STL",
        "566ac53fbd4ed47696a3dc351a990b1dfe78e2acd264e64b385268eb55706150",
        10784,
        AssetRole.COLLISION,
    ),
    _yubi_mesh(
        "hand_right_right_finger_link.STL",
        "32494b0e2ea9c4019acced883507318af0672e20e3a6a5ce1eca1b2a3d20e55f",
        132884,
        AssetRole.GEOMETRY,
    ),
)

YUBI_JAW: Final = GripperSpec(
    part_id=PartId("yubi-jaw-v1"),
    layout=bounded_layout(
        names=("right_finger_joint",),
        units=(CoordinateUnit.RADIAN,),
        lower=(0.0,),
        upper=(0.94,),
    ),
    travel_m=(0.002, 0.178742166),
    mimic_joints=(MimicJoint("left_finger_joint", of="right_finger_joint", multiplier=-1.0),),
    gap_curve=Curve1D(
        (
            Knot(0.000, 0.002000000),
            Knot(0.094, 0.022542556),
            Knot(0.188, 0.042903732),
            Knot(0.282, 0.062903749),
            Knot(0.376, 0.082366016),
            Knot(0.470, 0.101118692),
            Knot(0.564, 0.118996200),
            Knot(0.658, 0.135840691),
            Knot(0.752, 0.151503435),
            Knot(0.846, 0.165846140),
            Knot(0.940, 0.178742166),
        )
    ),
    assets=(YUBI_HANDS_URDF,),
)

YUBI_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("yubi"),
    label="YUBI bimanual handheld capture rig",
    kind=EmbodimentKind.CAPTURE_RIG,
    lineage=Lineage(family="yubi", revision="b7423c3-sx1"),
    attachments=(
        body_component("left_jaw", YUBI_JAW, RootMount("left_hand_root")),
        body_component("right_jaw", YUBI_JAW, RootMount("right_hand_root")),
    ),
    extra_assets=(YUBI_HANDS_URDF, *YUBI_MESHES),
    operator_mounts=(
        OperatorMount(
            OperatorSite.LEFT_HAND,
            "left_hand_root",
            "quest_left_controller",
        ),
        OperatorMount(
            OperatorSite.RIGHT_HAND,
            "right_hand_root",
            "quest_right_controller",
        ),
    ),
)
