"""DAS/UMI handheld capture family and the Quest ego rig.

The jaw's aperture-vs-angle table (previously copied across seven data-pipeline files) is
the measured ``gap(q)`` forward kinematics of ``assets/das_gripper_with_vr``: full-open
0.105 m at q=0 closing to ~0 at q=0.925 rad. The mimic chain mirrors the URDF exactly
(two levels: joint_4/joint_6 follow joint_2, which follows joint_1) and is pinned by
``tests/test_urdf_parity.py``.

Camera instance names are the sxd pipeline's canonical stream keys for the
``quest-genrobot-umi`` hardware bundle (``left_wrist``/``right_wrist``/``base``) and the
``/quest3s/head_left/camera`` topic for the ego rig.
"""

from typing import Final

from ..assets import AssetFormat, AssetRole, PackagedAsset
from ..compose import Attachment, AttachmentRole, EmbodimentSpec, MountFrame
from ..curves import Curve1D
from ..identity import EmbodimentId, EmbodimentKind, Lineage, PartId
from ..parts import CameraModality, CameraSpec, GripperSpec, MimicJoint, SensorModel

DAS_GRIPPER_URDF: Final = PackagedAsset(
    relpath="das_gripper_with_vr/urdf/DAS_Gripper_urdf.urdf",
    sha256="cea619914abc6539be8f608b3e68f9e70681ee25e583639723f9f285cc6410f9",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    media_type="application/xml",
)

# Baked from the DAS link_3/link_4 mesh FK over q in [0, 0.925] (verified against real
# episodes: observed max aperture ~0.0962 m; 0.105 m is the geometric full-open).
DAS_JAW_GAP_CURVE: Final = Curve1D(
    x=(
        0.0,
        0.0712,
        0.1423,
        0.2135,
        0.2846,
        0.3558,
        0.4269,
        0.4981,
        0.5692,
        0.6404,
        0.7115,
        0.7827,
        0.8538,
        0.925,
    ),
    y=(
        0.105,
        0.0987,
        0.09198,
        0.08484,
        0.07734,
        0.06951,
        0.06139,
        0.05302,
        0.04446,
        0.03571,
        0.02687,
        0.01794,
        0.00899,
        0.00011,
    ),
)

DAS_JAW_V4: Final = GripperSpec(
    part_id=PartId("das-jaw-v4"),
    joint_names=("joint_1",),  # the ONE actuated DOF; everything else mimics
    joint_lower=(0.0,),
    joint_upper=(0.925,),
    travel_m=(0.0, 0.105),
    mimic_joints=(
        MimicJoint("joint_3", of="joint_1", multiplier=-1.0),
        MimicJoint("joint_2", of="joint_1", multiplier=-1.0),
        MimicJoint("joint_4", of="joint_2", multiplier=-1.0),
        MimicJoint("joint_5", of="joint_1", multiplier=1.0),
        MimicJoint("joint_6", of="joint_2", multiplier=1.0),
    ),
    gap_curve=DAS_JAW_GAP_CURVE,
    assets=(DAS_GRIPPER_URDF,),
)

UVC_MONO_60: Final = CameraSpec(
    part_id=PartId("uvc-mono"),
    model=SensorModel.UVC_MONO,
    modality=CameraModality.RGB,
    fps=60.0,
)

QUEST3_HEAD: Final = CameraSpec(
    part_id=PartId("quest3-head"),
    model=SensorModel.QUEST3_RGB,
    modality=CameraModality.RGB,
    fps=30.0,
)

DAS_UMI_V4_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("das-umi-v4"),
    name="DAS/UMI handheld gripper pair V4 (Quest-tracked)",
    kind=EmbodimentKind.CAPTURE_RIG,
    lineage=Lineage(family="das-umi", revision="v4"),
    attachments=(
        Attachment("left_jaw", DAS_JAW_V4, AttachmentRole.BODY),
        Attachment("right_jaw", DAS_JAW_V4, AttachmentRole.BODY),
        Attachment(
            "left_wrist", UVC_MONO_60, AttachmentRole.SENSOR, MountFrame("left_jaw", "handle")
        ),
        Attachment(
            "right_wrist", UVC_MONO_60, AttachmentRole.SENSOR, MountFrame("right_jaw", "handle")
        ),
        Attachment("base", QUEST3_HEAD, AttachmentRole.SENSOR),
    ),
)

QUEST_EGO_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("quest-ego"),
    name="Quest 3 egocentric capture (no gripper)",
    kind=EmbodimentKind.CAPTURE_RIG,
    lineage=Lineage(family="quest-ego"),
    attachments=(Attachment("head_left", QUEST3_HEAD, AttachmentRole.SENSOR),),
)
