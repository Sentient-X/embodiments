"""YUBI: the factory's handheld UMI-style bimanual capture rigs (three variants).

YUBI is the factory SKU of the DAS/UMI handheld family — the standard fin-ray jaw IS the
DAS V4 jaw record. The wide-jaw variant replaces the fingers for a 140 mm span (its
aperture curve is not yet measured, so it carries travel only). Camera instance names are
byte-equal to the factory hardware JSON (``wrist_left``/``wrist_right``); the sxd pipeline's
``left_wrist``-style keys belong to the ``das-umi-v4`` pipeline spec — the two vocabularies
are mapped at adoption, not silently merged.
"""

import dataclasses
from typing import Final

from ..compose import Component, ComponentRole, MountFrame, _EmbodimentDefinition
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..parts import CameraModality, CameraSpec, GripperSpec, SensorModel
from .das import DAS_JAW_V4, DAS_UMI_V4_URDF, UVC_MONO_60

YUBI_JAW_FINRAY: Final = DAS_JAW_V4  # the standard fin-ray jaw is the DAS V4 jaw

YUBI_JAW_WIDE140: Final = dataclasses.replace(
    DAS_JAW_V4,
    part_id=PartId("yubi-jaw-wide140"),
    travel_m=(0.0, 0.140),
    gap_curve=None,  # wide-finger aperture curve not yet measured
)

D405_30: Final = CameraSpec(
    part_id=PartId("realsense-d405"),
    model=SensorModel.REALSENSE_D405,
    modality=CameraModality.RGBD,
    fps=30.0,
    resolution=(1280, 720),  # Intel spec: depth and RGB share the 1280x720 imager pair
)


def _yubi(
    embodiment_id: str,
    name: str,
    variant: str,
    revision: str,
    jaw: GripperSpec,
    wrist_camera: CameraSpec,
) -> _EmbodimentDefinition:
    return _EmbodimentDefinition(
        embodiment_id=EmbodimentName(embodiment_id),
        name=name,
        kind=EmbodimentKind.CAPTURE_RIG,
        lineage=Lineage(family="yubi", variant=variant, revision=revision),
        attachments=(
            Component("left_jaw", jaw, ComponentRole.BODY),
            Component("right_jaw", jaw, ComponentRole.BODY),
            Component(
                "wrist_left",
                wrist_camera,
                ComponentRole.SENSOR,
                MountFrame("left_jaw", "link_ca2"),
            ),
            Component(
                "wrist_right",
                wrist_camera,
                ComponentRole.SENSOR,
                MountFrame("right_jaw", "link_ca2"),
            ),
        ),
        extra_assets=(DAS_UMI_V4_URDF,),
    )


YUBI_MONO_SPEC: Final = _yubi(
    "yubi-mono", "YUBI (UVC monocular wrists)", "mono", "v2-composite", YUBI_JAW_FINRAY, UVC_MONO_60
)
YUBI_DEPTH_SPEC: Final = _yubi(
    "yubi-depth", "YUBI (RealSense D405 wrists)", "depth", "v2-composite", YUBI_JAW_FINRAY, D405_30
)
YUBI_WIDEJAW_SPEC: Final = _yubi(
    "yubi-widejaw", "YUBI (wide 140 mm jaw)", "widejaw", "v3-alu", YUBI_JAW_WIDE140, UVC_MONO_60
)
