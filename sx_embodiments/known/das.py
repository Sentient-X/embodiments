"""DAS/UMI handheld capture family and the Quest ego rig.

The jaw's aperture-vs-angle table (previously copied across seven data-pipeline files) is
the measured ``gap(q)`` forward kinematics of ``assets/das_gripper_with_vr``: full-open
0.105 m at q=0 closing to ~0 at q=0.925 rad. The mimic chain mirrors the URDF exactly
(two levels: joint_4/joint_6 follow joint_2, which follows joint_1) and is pinned by
``tests/test_urdf_parity.py``.

Camera instance names are the sxd pipeline's canonical stream keys for the
``quest-genrobot-umi`` hardware bundle: two wrists and the independently calibrated
left/right Quest passthrough cameras.
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
    sensor_component,
)
from ..curves import Curve1D, Knot
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import (
    CameraModality,
    CameraOptics,
    CameraOpticsAuthority,
    CameraSpec,
    FactSource,
    GripperSpec,
    MimicJoint,
    SensorModel,
)
from ._authoring import bounded_layout

DAS_GRIPPER_URDF: Final = packaged_asset(
    relpath="das_gripper_with_vr/urdf/DAS_Gripper_urdf.urdf",
    sha256="cea619914abc6539be8f608b3e68f9e70681ee25e583639723f9f285cc6410f9",
    size_bytes=22803,
    format=AssetFormat.URDF,
    role=AssetRole.OTHER,
    provenance=AssetProvenance(
        repository="https://github.com/Sentient-X/sxd",
        revision="470b4ba5c3943796fb3840e45f835471eaed96d8",
        path="urdf/DAS_Gripper_with_VR/urdf/DAS_Gripper_urdf.urdf",
        license_id="Apache-2.0",
    ),
    media_type="application/xml",
)

DAS_UMI_V4_URDF: Final = packaged_asset(
    relpath="das_gripper_with_vr/urdf/DAS_UMI_V4.urdf",
    sha256="5d25b310a58b7b737aad45f9bf3cbea266193d3b4b602be1226c9be00a122603",
    size_bytes=34853,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Sentient-X/sxd",
        revision="470b4ba5c3943796fb3840e45f835471eaed96d8",
        path="urdf/DAS_Gripper_with_VR/urdf/DAS_Gripper_urdf.urdf",
        license_id="Apache-2.0",
        generator="sx-embodiments/tools/compose_das_urdf.py",
    ),
    media_type="application/xml",
)

QUEST_EGO_URDF: Final = packaged_asset(
    relpath="quest_ego/urdf/quest_ego.urdf",
    sha256="f41d4a9a2d7e10e0e9da0a0f8c00a98cf95073cf6ed6ae852a8087a2590e1c5d",
    size_bytes=1245,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Sentient-X/sx-embodiments",
        revision="manifest-v5",
        path="assets/quest_ego/urdf/quest_ego.urdf",
        license_id="Apache-2.0",
        generator="controller-free headset description with reference-unit optical frames",
    ),
    media_type="application/xml",
)

QUEST3_HEADSET_MESH: Final = packaged_asset(
    relpath="quest_ego/meshes/quest3mesh.obj",
    sha256="3601bece91279ae9efd85c334274c5689700cef198621a9bf9f122a7a39589ce",
    size_bytes=547739,
    format=AssetFormat.MESH,
    role=AssetRole.GEOMETRY,
    provenance=AssetProvenance(
        repository="https://github.com/Ericcsr/ARCap",
        revision="00ffd461ce7e6e8cd48dd74f6486576ee59d1a0d",
        path="data_processing/assets/quest3mesh.obj",
        license_id="MIT",
    ),
    media_type="model/obj",
)

QUEST3_HEADSET_LICENSE: Final = packaged_asset(
    relpath="quest_ego/LICENSE.ARCap",
    sha256="af471fb8777589b95290ccf8628ce0aa62ef1030d29f3370e047890ee04a3f99",
    size_bytes=1072,
    format=AssetFormat.OTHER,
    role=AssetRole.LICENSE,
    provenance=AssetProvenance(
        repository="https://github.com/Ericcsr/ARCap",
        revision="8fe21e533d2af8549b8c880ff331445dc0a42dbf",
        path="LICENSE",
        license_id="MIT",
    ),
    media_type="text/plain",
)

# Baked from the DAS link_3/link_4 mesh FK over q in [0, 0.925] (verified against real
# episodes: observed max aperture ~0.0962 m; 0.105 m is the geometric full-open).
DAS_JAW_GAP_CURVE: Final = Curve1D(
    knots=(
        Knot(0.0, 0.105),
        Knot(0.0712, 0.0987),
        Knot(0.1423, 0.09198),
        Knot(0.2135, 0.08484),
        Knot(0.2846, 0.07734),
        Knot(0.3558, 0.06951),
        Knot(0.4269, 0.06139),
        Knot(0.4981, 0.05302),
        Knot(0.5692, 0.04446),
        Knot(0.6404, 0.03571),
        Knot(0.7115, 0.02687),
        Knot(0.7827, 0.01794),
        Knot(0.8538, 0.00899),
        Knot(0.925, 0.00011),
    ),
)

DAS_JAW_V4: Final = GripperSpec(
    part_id=PartId("das-jaw-v4"),
    layout=bounded_layout(
        names=("joint_1",),  # the ONE actuated DOF; everything else mimics
        units=(CoordinateUnit.RADIAN,),
        lower=(0.0,),
        upper=(0.925,),
    ),
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

QUEST3_REFERENCE_OPTICS: Final = CameraOptics(
    width=1280,
    height=960,
    image_from_camera=(868.31, 0.0, 640.18, 0.0, 868.31, 482.07, 0.0, 0.0, 1.0),
    distortion_model="none",
    distortion_coefficients=(),
    authority=CameraOpticsAuthority.REFERENCE_UNIT,
    source=FactSource(
        repository="https://gist.github.com/sudotman/652995df8c18c819d095ba0289dbb6f0",
        revision="856868e4f1def6b8bd3d038f922f9b42609fce57",
        path="quest3CameraIntrinsics",
    ),
)

QUEST3_HEAD: Final = CameraSpec(
    part_id=PartId("quest3-head"),
    model=SensorModel.QUEST3_RGB,
    modality=CameraModality.RGB,
    fps=30.0,
    optics=QUEST3_REFERENCE_OPTICS,
)

DAS_UMI_V4_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("das-umi-v4"),
    label="DAS/UMI handheld gripper pair V4 (Quest-tracked)",
    kind=EmbodimentKind.CAPTURE_RIG,
    lineage=Lineage(family="das-umi", revision="v4"),
    attachments=(
        body_component("left_jaw", DAS_JAW_V4, RootMount("left_base_link")),
        body_component("right_jaw", DAS_JAW_V4, RootMount("right_base_link")),
        sensor_component("head_left", QUEST3_HEAD, RootMount("quest3s_camera_optical")),
        sensor_component(
            "head_right",
            QUEST3_HEAD,
            RootMount("quest3s_right_camera_optical"),
        ),
    ),
    extra_assets=(DAS_UMI_V4_URDF, QUEST3_HEADSET_MESH, QUEST3_HEADSET_LICENSE),
    operator_mounts=(
        OperatorMount(OperatorSite.HEAD, "quest3s_head", "quest3s_head"),
        OperatorMount(OperatorSite.LEFT_HAND, "left_base_link", "left_handle"),
        OperatorMount(OperatorSite.RIGHT_HAND, "right_base_link", "right_handle"),
    ),
)

QUEST_EGO_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("quest-ego"),
    label="Quest 3 egocentric headset (stereo passthrough, no controllers)",
    kind=EmbodimentKind.CAPTURE_RIG,
    lineage=Lineage(family="quest-ego"),
    attachments=(
        sensor_component(
            "head_left",
            QUEST3_HEAD,
            RootMount("quest3s_camera_optical"),
        ),
        sensor_component(
            "head_right",
            QUEST3_HEAD,
            RootMount("quest3s_right_camera_optical"),
        ),
    ),
    extra_assets=(QUEST_EGO_URDF, QUEST3_HEADSET_MESH, QUEST3_HEADSET_LICENSE),
    operator_mounts=(OperatorMount(OperatorSite.HEAD, "quest3s_head", "quest3s_head"),),
)
