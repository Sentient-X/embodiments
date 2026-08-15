"""The upstream YUBI bimanual handheld rig, held out of production pending calibration.

The authoritative kinematic description is the upstream YUBI hardware URDF
(airoa-org/yubi-sw ``yubi_description``, expanded from its xacro by
``tools/expand_yubi_xacro.py``): both hands, fin-ray finger pairs (one encoder-driven
joint per hand, the mirror finger mimics at -1), hand cameras, and the Quest controller
mount frames. Quest is a controller/tracking input in upstream YUBI, not a headset camera
pair. Deliberate divergence, documented here per the abstractions rule: the jaw
*wire records* stay the DAS V4 fin-ray records (they model the recorded magnetic-encoder
aperture channel and its measured gap curve), while the *description asset* is the
yubi-sw URDF — the URDF's own finger joints are not the wire channels.
"""

from typing import Final

from sx_contracts.assets import AssetFormat, AssetProvenance, AssetRole

from ..assets import packaged_asset
from ..compose import (
    EmbodimentDefinition,
    OperatorMount,
    OperatorSite,
    RootMount,
    body_component,
)
from ..identity import EmbodimentKind, EmbodimentName, Lineage
from .das import DAS_JAW_V4

YUBI_HANDS_URDF: Final = packaged_asset(
    relpath="yubi_description/urdf/yubi_hands.urdf",
    sha256="360033943828a8e6822f09a75db92856cb30feacd8131219780326f4c60b5e44",
    size_bytes=7563,
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

YUBI_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("yubi"),
    label="YUBI bimanual handheld capture rig",
    kind=EmbodimentKind.CAPTURE_RIG,
    lineage=Lineage(family="yubi", revision="b7423c3"),
    attachments=(
        body_component("left_jaw", DAS_JAW_V4, RootMount("left_hand_root")),
        body_component("right_jaw", DAS_JAW_V4, RootMount("right_hand_root")),
    ),
    extra_assets=(YUBI_HANDS_URDF,),
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
