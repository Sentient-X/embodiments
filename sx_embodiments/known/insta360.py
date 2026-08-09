"""Insta360-UMI handheld capture rig: X5 dual-fisheye per gripper + AMT212A jaw encoder.

The data-pipeline converter currently emits a resolution-derived provisional fisheye model.
That is a recording-adapter fact, not authoritative X5 optics, so this development definition
deliberately declares no cameras until a complete profile and installation transforms are
pinned. The jaw also has no captured kinematic description (bench aperture widths are
unmeasured), so it is a :class:`DeviceSpec` and the rig's channel layout stays undeclared; the
honest jaw signal is the normalized encoder aperture travelling with the capture.
"""

from typing import Final

from ..compose import EmbodimentDefinition, body_component
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..parts import DeviceSpec

INSTA360_UMI_JAW: Final = DeviceSpec(
    part_id=PartId("insta360-umi-jaw"),
    description="UMI parallel jaw with AMT212A-V 12-bit absolute encoder; kinematics and "
    "bench aperture widths not yet measured",
)

INSTA360_UMI_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("insta360-umi"),
    label="Insta360-UMI handheld gripper pair (X5 dual-fisheye)",
    kind=EmbodimentKind.CAPTURE_RIG,
    lineage=Lineage(family="insta360-umi"),
    attachments=(
        body_component("left_jaw", INSTA360_UMI_JAW),
        body_component("right_jaw", INSTA360_UMI_JAW),
    ),
)
