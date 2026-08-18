"""The current YUBI bimanual handheld rig, held out pending per-unit calibration.

The authoritative description is composed from the two supplied Onshape CAD exports.
Each hand preserves the housing, controller, finger, connector, nail, grip, tip, and
camera links. ``left_finger`` is the one encoder-driven joint; the other finger and the
four-bar connector/nail joints mimic it. The explicit Quest tracking frame includes the
measured +90 degree tracking-to-CAD controller basis used by the capture viewer.

The aperture curve is the inner nail-mesh separation across the URDF joint box. The CAD
meshes overlap by 0.106 mm at the mechanical stop, so the recorded closed aperture is
clamped to zero. Per-unit camera calibration remains episode/session evidence.
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
    sha256="dc12644756e03db1efc70cfcbd14a2adca53b5645ce79344d67c5b11250a62bb",
    size_bytes=22504,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Sentient-X/embodiments",
        revision="yubi-description7-6ff03008-8afd1ffe",
        path="assets/yubi_description/urdf/yubi_hands.urdf",
        license_id="LicenseRef-Sentient-Proprietary",
        generator="tools/compose_yubi_urdf.py",
    ),
    media_type="application/xml",
)


def _yubi_mesh(
    relpath: str,
    sha256: str,
    size_bytes: int,
    source_side: str,
    source_name: str,
) -> PackagedAsset:
    repository = {
        "left": (
            "https://cad.onshape.com/documents/b807fc587efa728d9cf496a6/"
            "w/7176242b14d954b35e9a0f8d/e/c659d653cfd1965addfdf1a0"
        ),
        "right": (
            "https://cad.onshape.com/documents/a49048be0a418509ea642b3d/"
            "w/0a6170e4e5b19ae4d8d75a37/e/9cb1639fa99dd40bf32608f7"
        ),
    }[source_side]
    return packaged_asset(
        relpath=f"yubi_description/meshes/{relpath}",
        sha256=sha256,
        size_bytes=size_bytes,
        format=AssetFormat.MESH,
        role=AssetRole.GEOMETRY,
        provenance=AssetProvenance(
            repository=repository,
            revision=f"sha256:{sha256}",
            path=f"meshes/{source_name}",
            license_id="LicenseRef-Sentient-Proprietary",
        ),
        media_type="model/stl",
    )


# Complete mesh closure of YUBI_HANDS_URDF. These are part of the embodiment's
# content identity, not merely files that happen to ship beside the URDF.
YUBI_MESHES: Final[tuple[PackagedAsset, ...]] = (
    _yubi_mesh(
        "shared/link_base_link.stl",
        "685276b224b90ed2e7b1b7697c0c9887874ab13a64ba4225116f32252536a000",
        1951384,
        "left",
        "link_base_link.stl",
    ),
    _yubi_mesh(
        "shared/link_connector.stl",
        "92669d334be292c4a98aad0dfa63bb3e3442ac180ddfc7f2c6d4670c582f0610",
        49084,
        "left",
        "link_connector_left.stl",
    ),
    _yubi_mesh(
        "shared/link_left_finger.stl",
        "bd87c87be45e11b21abcd43afc8d67efb2ff6b6dff968c727cf62d041ec8230f",
        533484,
        "left",
        "link_left_finger.stl",
    ),
    _yubi_mesh(
        "shared/link_left_nail.stl",
        "8ab1f912630be3a0efc1ccd07bf1513e8964259a4109b263745898ec26f39791",
        251484,
        "left",
        "link_left_nail.stl",
    ),
    _yubi_mesh(
        "shared/link_right_finger.stl",
        "be704bf532c29c86e9a5486fa9c53c093a0531ac36f3df4ec9535e9b79ef3936",
        560384,
        "left",
        "link_right_finger.stl",
    ),
    _yubi_mesh(
        "shared/link_right_nail.stl",
        "2f8aff9c5da05b284281ae7e117ad905d44f033cbba872f84542eb9037cd295a",
        251084,
        "left",
        "link_right_nail.stl",
    ),
    _yubi_mesh(
        "left/link_controller.stl",
        "ff2feb49ce9348a289d1b336f9dfc523a88db6e569ea47a7ca78177a453a1206",
        6162384,
        "left",
        "link_controller_left.stl",
    ),
    _yubi_mesh(
        "left/link_housing.stl",
        "1e2e00243d65bd9a624a341f0d4323a2a45d0d4c425fd4c3075dcaf290eb1b31",
        1717384,
        "left",
        "link_housing_left.stl",
    ),
    _yubi_mesh(
        "right/link_controller.stl",
        "f05d4c84aa0aca2ad647b9c3fb8772753bb646a54e4c1054bc8210dc019a47e1",
        6162884,
        "right",
        "link_controller_right.stl",
    ),
    _yubi_mesh(
        "right/link_housing.stl",
        "fec06aa0637d138d20e325714cfc1cc531927706d376bc8b9f1847a7a4196b4e",
        1251234,
        "right",
        "link_housing.stl",
    ),
)

YUBI_JAW: Final = GripperSpec(
    part_id=PartId("yubi-jaw-v2"),
    layout=bounded_layout(
        names=("left_finger",),
        units=(CoordinateUnit.RADIAN,),
        lower=(0.0,),
        upper=(0.785398,),
    ),
    travel_m=(0.0, 0.100083349),
    mimic_joints=(
        MimicJoint("right_finger", of="left_finger", multiplier=-1.0),
        MimicJoint("right_nail", of="left_finger", multiplier=1.0),
        MimicJoint("left_nail", of="left_finger", multiplier=-1.0),
        MimicJoint("right_connector", of="left_finger", multiplier=-1.0),
        MimicJoint("left_connector", of="left_finger", multiplier=1.0),
    ),
    gap_curve=Curve1D(
        (
            Knot(0.000000000, 0.000000000),
            Knot(0.078539816, 0.010457629),
            Knot(0.157079633, 0.021072297),
            Knot(0.235619449, 0.031672278),
            Knot(0.314159265, 0.042192221),
            Knot(0.392699082, 0.052567267),
            Knot(0.471238898, 0.062733450),
            Knot(0.549778714, 0.072628091),
            Knot(0.628318531, 0.082190189),
            Knot(0.706858347, 0.091360788),
            Knot(0.785398000, 0.100083349),
        )
    ),
    assets=(YUBI_HANDS_URDF,),
)

YUBI_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("yubi"),
    label="YUBI bimanual handheld capture rig",
    kind=EmbodimentKind.CAPTURE_RIG,
    lineage=Lineage(family="yubi", revision="description7-sx1"),
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
