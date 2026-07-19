"""Insta360-UMI handheld capture rig: X5 dual-fisheye per gripper + AMT212A jaw encoder.

The data-pipeline converter (``sxd/umico/converters/insta360.py``) emits each gripper's
action-facing fisheye lens at 30 fps with a resolution-anchored *equidistant* calibration
(nominal until a proper X5 fisheye calibration is supplied) — the projection family here is
that wire fact. The jaw has no captured kinematic description (bench aperture widths are
unmeasured), so it is a :class:`DeviceSpec` and the rig's channel layout stays undeclared;
the honest jaw signal is the normalized encoder aperture travelling with the capture.

Camera instance names follow the handheld-family convention (``left_wrist``/``right_wrist``,
the das-umi-v4 shape); the converter's MCAP namespace keys (``left``/``right`` in
``/insta360/{side}/camera0``) are mapped at adoption, not silently merged.
"""

from typing import Final

from ..compose import Attachment, AttachmentRole, EmbodimentSpec, MountFrame
from ..identity import EmbodimentId, EmbodimentKind, Lineage, PartId
from ..parts import CameraModality, CameraSpec, DeviceSpec, LensProjection, SensorModel

X5_FISHEYE_30: Final = CameraSpec(
    part_id=PartId("insta360-x5"),
    model=SensorModel.INSTA360_X5,
    modality=CameraModality.RGB,
    fps=30.0,
    projection=LensProjection.EQUIDISTANT,
    # resolution stays None: the rig delivers a per-lens crop downsampled at process time;
    # no single native per-product figure is pinned.
)

INSTA360_UMI_JAW: Final = DeviceSpec(
    part_id=PartId("insta360-umi-jaw"),
    description="UMI parallel jaw with AMT212A-V 12-bit absolute encoder; kinematics and "
    "bench aperture widths not yet measured",
)

INSTA360_UMI_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("insta360-umi"),
    name="Insta360-UMI handheld gripper pair (X5 dual-fisheye)",
    kind=EmbodimentKind.CAPTURE_RIG,
    lineage=Lineage(family="insta360-umi"),
    attachments=(
        Attachment("left_jaw", INSTA360_UMI_JAW, AttachmentRole.BODY),
        Attachment("right_jaw", INSTA360_UMI_JAW, AttachmentRole.BODY),
        Attachment(
            "left_wrist", X5_FISHEYE_30, AttachmentRole.SENSOR, MountFrame("left_jaw", "handle")
        ),
        Attachment(
            "right_wrist", X5_FISHEYE_30, AttachmentRole.SENSOR, MountFrame("right_jaw", "handle")
        ),
    ),
)
