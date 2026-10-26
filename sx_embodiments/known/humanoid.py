"""Sentient humanoid: the upper body built from parts, and the bodies its parts compose.

The robot in service is the **upper body**: a torso and two five-joint arms on one Damiao CAN
bus, with the vendor head on the chest. The full humanoid is that upper body plus two legs. Both
are assemblies of parts cut from one description, ``humanoid_pkg``'s hardware URDF
(``urdf/urdf_robot_acc.urdf`` at ``666bc6b5``), so the parts and the vendor's robot cannot
drift: :class:`~sx_embodiments.assembly.Subtree` cuts the upper body (the vendor tree without
the legs and the head), the head (below ``neck_joint``) and each leg (below its
``hip_pitch_joint``), and the assembly that rejoins them is the vendor's robot again, every
link where the source put it (``tests/test_humanoid.py`` holds that by forward kinematics).
Each part is a :class:`~sx_embodiments.mechanical.Module`: the upper body offers a neck seat, two
hip seats and two wrists, and the head and legs mate back onto their seats. An SO-101 neck is a
composition, not a registered body, while neck motors are undecided: it mates on the neck seat
in the head's place, or ``compose_embodiments`` sets it on the upper body's ``head_link``. The
parallel grippers are modules whose servo and jaw no source describes, so the upper body with
them is a development body (``sentient-humanoid-upper-body-grippers``).

Drives, from Sentient-X sources at pinned commits:

* ``humanoid_pkg`` ``config/humanoid_joint_motor_assignments.csv`` (same commit): each joint's
  motor and its CAN send id: the right arm on 0x10..0x14, the left on 0x18..0x1C, the torso
  yaw on 0x0F, and the waist's roll and pitch both on the pair 0x0E/0x0D.
* ``sentient_can`` at ``3837d224327d89ed30b6fdee3577769d8dae5f94``,
  ``examples/capture_readings_27motors_mt_direct.cpp``: the motor models (DM8009 shoulders and
  torso, DM4340 elbows, forearms and wrists, DM4340 waist pair) and the answer ids, send id +
  0x20, on ``can0``. The same file drives a sixth DM4310 per arm (0x15, 0x1D) that neither the
  assignment table nor the URDF names; the registry binds only joints the description has.
* ``humanoid_pkg`` ``scripts/motor_state_to_joint_state_acc_retarget.py``: every single-motor
  joint is the motor angle unchanged ("provisional assumption: direct 1:1 mapping"), so each
  binding is identity; a unit's measured zero and direction are its joint calibration.

The waist's roll and pitch are driven together by two motors through a linkage whose mapping
upstream marks provisional, so they stay undocumented drives, outside any driver's chain;
``waist_rod_joint`` is feedback-only and the neck and head joints have no motor, so neither
enters the body state. The legs carry no binding: the robot in service has none.

The head carries an Intel RealSense D435 (``urdf/humanoid_pkg.csv``, ``camera_link`` on
``head_link``), the head module's fitted camera module. Its colour stream is declared at 1280x720
with the datasheet's nominal optics: a pinhole whose focal lengths are the OV2740's published
field of view (69.4 x 42.5 degrees) and whose principal point is the raster centre. The optical
frame sits at the CAD ``camera_link`` origin, which the mesh shows faces +x; a unit's intrinsics
and the camera's true placement on ``head_link`` are its calibrations, measured by
:mod:`sx_embodiments.calibrate`. The head sits on the chest the torso turns, so the camera rides
on the torso.

The hardware URDF's limits win at the execution boundary; the attached 28-joint MJCF is a
simulation description whose limits differ.
"""

import math
from dataclasses import replace
from typing import Final, Literal

from sx_contracts import ComponentId

from ..assembly import Frame, Pose, Subtree
from ..assets import (
    AssetFormat,
    AssetProvenance,
    AssetRole,
    PackagedAsset,
    packaged_asset,
)
from ..compose import (
    EmbodimentDefinition,
    MountedOn,
    RootMount,
    body_component,
    sensor_component,
)
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import (
    UNDOCUMENTED_DRIVE_REASON,
    UNDOCUMENTED_OBSERVATION_REASON,
    ActuatorBinding,
    ActuatorBus,
    ActuatorFeedback,
    ActuatorModel,
    CoordinateUnit,
    DirectDrive,
    UndocumentedDrive,
    Unobserved,
)
from ..mechanical import Build, InterfaceId, Mate, Module, ModuleId, Placed, Socket
from ..media import MediaRig, StereoPair
from ..parts import (
    CameraModality,
    CameraOptics,
    CameraOpticsAuthority,
    CameraSpec,
    ChainSpec,
    DeviceSpec,
    FactSource,
    Limb,
    SensorModel,
)
from ..sites import BodySite
from ._authoring import bounded_layout

SENTIENT_HUMANOID_URDF: Final = packaged_asset(
    relpath="humanoid_pkg/urdf/urdf_robot_acc.urdf",
    sha256="e79678962ede3c9c6def69ac3e40f1ac67e59d41e5205f6356dd294bb88017a6",
    size_bytes=50397,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Sentient-X/humanoid_pkg",
        revision="666bc6b5e00c76b8388a70fcc40a6987e0f1da7a",
        path="urdf/urdf_robot_acc.urdf",
        license_id="BSD-3-Clause",
    ),
    media_type="application/xml",
)
SENTIENT_HUMANOID_MJCF: Final = packaged_asset(
    relpath="humanoid_pkg/mjcf/humanoid_28dof.xml",
    sha256="1276bbfd8731401160abe1698c988a6807fdb95f001c2ad9755f18e856b4c6a4",
    size_bytes=19218,
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Sentient-X/humanoid_pkg",
        revision="666bc6b5e00c76b8388a70fcc40a6987e0f1da7a",
        path="mjcf/humanoid_28dof.xml",
        license_id="BSD-3-Clause",
    ),
    media_type="application/xml",
)

SENTIENT_HUMANOID_SOURCE_URDF: Final = replace(SENTIENT_HUMANOID_URDF, role=AssetRole.OTHER)


def _generated(relpath: str, sha256: str, size_bytes: int) -> PackagedAsset:
    return packaged_asset(
        relpath=relpath,
        sha256=sha256,
        size_bytes=size_bytes,
        format=AssetFormat.URDF,
        role=AssetRole.DESCRIPTION,
        provenance=AssetProvenance(
            repository="https://github.com/Sentient-X/humanoid_pkg",
            revision="666bc6b5e00c76b8388a70fcc40a6987e0f1da7a",
            path="urdf/urdf_robot_acc.urdf",
            license_id="BSD-3-Clause",
            generator="sx-embodiments/tools/compose_registered_urdfs.py",
        ),
        media_type="application/xml",
    )


#: The motors' answer ids are their send ids plus this (``sentient_can``'s ``recv_can_ids``).
SENTIENT_HUMANOID_MASTER_ID_OFFSET: Final = 0x20


def _damiao(model: ActuatorModel, bus_id: int) -> DirectDrive:
    """One motor on the humanoid's one Damiao CAN bus, identity drive map."""
    return DirectDrive(ActuatorBinding(model=model, bus=ActuatorBus.DAMIAO_CAN, bus_id=bus_id))


#: The waist's roll and pitch are driven together by two DM4340 motors (CAN 0x0E, 0x0D) through
#: a linkage whose mapping humanoid_pkg marks provisional: the drive exists but is not documented,
#: so no driver's chain carries it and no unit's reading is a joint observation yet.
_WAIST = UndocumentedDrive(UNDOCUMENTED_DRIVE_REASON)
_WAIST_READ = Unobserved(UNDOCUMENTED_OBSERVATION_REASON)

SENTIENT_HUMANOID_TORSO: Final = ChainSpec(
    part_id=PartId("sentient-humanoid-torso"),
    layout=bounded_layout(
        names=("torso_joint", "waist_roll_joint", "waist_pitch_joint"),
        units=(CoordinateUnit.RADIAN,) * 3,
        lower=(-1.570796326795, -0.270002435284, -0.767944870878),
        upper=(1.570796326795, 0.270002435284, 0.767944870878),
        actuations=(_damiao(ActuatorModel.DAMIAO_DM8009, 0x0F), _WAIST, _WAIST),
        observations=(ActuatorFeedback(), _WAIST_READ, _WAIST_READ),
    ),
    home=(0.0, 0.0, 0.0),
)

#: Each arm's first send id; its five motors take consecutive ids from there.
_ARM_BASE_ID: Final = {"right": 0x10, "left": 0x18}
_ARM_MODELS: Final = (
    ActuatorModel.DAMIAO_DM8009,  # shoulder_pitch
    ActuatorModel.DAMIAO_DM8009,  # shoulder_roll
    ActuatorModel.DAMIAO_DM4340,  # elbow_yaw
    ActuatorModel.DAMIAO_DM4340,  # elbow_forearm_yaw
    ActuatorModel.DAMIAO_DM4340,  # wrist_yaw
)


def _humanoid_arm(side: Literal["left", "right"]) -> ChainSpec:
    shoulder_roll = (
        (-0.356919832033, 3.14159265359) if side == "left" else (-3.14159265359, 0.356919832033)
    )
    return ChainSpec(
        part_id=PartId(f"sentient-humanoid-{side}-arm"),
        layout=bounded_layout(
            names=tuple(
                f"{side}_{name}_joint"
                for name in (
                    "shoulder_pitch",
                    "shoulder_roll",
                    "elbow_yaw",
                    "elbow_forearm_yaw",
                    "wrist_yaw",
                )
            ),
            units=(CoordinateUnit.RADIAN,) * 5,
            lower=(
                -3.14159265359,
                shoulder_roll[0],
                -1.570796326795,
                -1.570796326795,
                -1.570796326795,
            ),
            upper=(
                3.14159265359,
                shoulder_roll[1],
                1.570796326795,
                1.570796326795,
                1.570796326795,
            ),
            actuations=tuple(
                _damiao(model, _ARM_BASE_ID[side] + index)
                for index, model in enumerate(_ARM_MODELS)
            ),
            observations=(ActuatorFeedback(),) * 5,
        ),
        home=(0.0,) * 5,
    )


def _humanoid_leg(side: Literal["left", "right"]) -> ChainSpec:
    return ChainSpec(
        part_id=PartId(f"sentient-humanoid-{side}-leg"),
        layout=bounded_layout(
            names=tuple(
                f"{side}_{name}_joint"
                for name in ("hip_roll", "hip_pitch", "hip_yaw", "knee", "foot1", "foot2")
            ),
            units=(CoordinateUnit.RADIAN,) * 6,
            lower=(
                -0.279252680319,
                -2.094395102393,
                -1.570796326795,
                -1.535889741755,
                -0.698131700798,
                -0.436332312999,
            ),
            upper=(
                1.919862177194,
                2.094395102393,
                1.570796326795,
                1.535889741755,
                0.698131700798,
                0.436332312999,
            ),
        ),
        home=(0.0,) * 6,
    )


SENTIENT_HUMANOID_RIGHT_ARM: Final = _humanoid_arm("right")
SENTIENT_HUMANOID_LEFT_ARM: Final = _humanoid_arm("left")
SENTIENT_HUMANOID_RIGHT_LEG: Final = _humanoid_leg("right")
SENTIENT_HUMANOID_LEFT_LEG: Final = _humanoid_leg("left")


REALSENSE_D435_COLOR: Final = CameraSpec(
    part_id=PartId("realsense-d435-color"),
    model=SensorModel("realsense_d435"),
    modality=CameraModality.RGB,
    fps=30.0,
    optics=CameraOptics(
        width=1280,
        height=720,
        image_from_camera=(924.3, 0.0, 640.0, 0.0, 925.7, 360.0, 0.0, 0.0, 1.0),
        distortion_model="none",
        distortion_coefficients=(),
        authority=CameraOpticsAuthority.MODEL_NOMINAL,
        source=FactSource(
            repository="https://www.realsenseai.com/wp-content/uploads/2020/06/"
            "Intel-RealSense-D400-Series-Datasheet-June-2020.pdf",
            revision="337029-009",
            path="OV2740 RGB sensor properties: horizontal and vertical field of view",
        ),
    ),
)

#: The colour imager's optical frame (z along the view) at the CAD camera origin.
HEAD_CAMERA: Final = Frame(
    "head_camera_color_optical",
    "camera_link",
    Pose(rpy=(-math.pi / 2, 0.0, -math.pi / 2)),
    preview_kind="camera",
    preview_label="Head camera",
    camera_optical=True,
)

#: The vendor tree without its legs and head; it keeps the vendor's names.
UPPER_BODY: Final = Subtree(
    SENTIENT_HUMANOID_URDF,
    without=("right_hip_pitch_joint", "left_hip_pitch_joint", "neck_joint"),
)
#: The vendor head: neck yaw, head pitch and the camera link, below ``neck_joint``.
HEAD: Final = Subtree(SENTIENT_HUMANOID_URDF, below="neck_joint")
LEGS: Final = tuple(
    Subtree(SENTIENT_HUMANOID_URDF, below=f"{side}_hip_pitch_joint") for side in ("right", "left")
)


NECK: Final = InterfaceId("sentient-humanoid-neck")
HIP: Final = InterfaceId("sentient-humanoid-hip")
WRIST: Final = InterfaceId("sentient-humanoid-wrist")

UPPER_BODY_COMPONENTS: Final = (
    body_component("torso", SENTIENT_HUMANOID_TORSO, RootMount("base_link"), limb=Limb.TORSO),
    body_component(
        "right_arm", SENTIENT_HUMANOID_RIGHT_ARM, MountedOn("torso", "chest_link"), limb=Limb.ARM
    ),
    body_component(
        "left_arm", SENTIENT_HUMANOID_LEFT_ARM, MountedOn("torso", "chest_link"), limb=Limb.ARM
    ),
)

#: The torso and arms. The neck and hip seats are the cut joints' parents and origins in the
#: vendor URDF; a gripper meets a wrist at the wrist-yaw link's frame, the last link the CAD
#: models (the flange face's offset from it is not captured).
UPPER_BODY_MODULE: Final = Module(
    module_id=ModuleId("sentient-humanoid-upper-body"),
    label="Sentient humanoid torso and arms",
    description=UPPER_BODY,
    components=UPPER_BODY_COMPONENTS,
    sites=(
        BodySite("left_wrist", ComponentId("left_arm"), "left_wrist_yaw_link"),
        BodySite("right_wrist", ComponentId("right_arm"), "right_wrist_yaw_link"),
        BodySite("left_elbow", ComponentId("left_arm"), "left_elbow_yaw_link"),
        BodySite("right_elbow", ComponentId("right_arm"), "right_elbow_yaw_link"),
        BodySite("chest", ComponentId("torso"), "chest_link"),
        BodySite("pelvis", ComponentId("torso"), "base_link"),
    ),
    sockets=(
        Socket(
            "neck",
            NECK,
            "chest_link",
            Pose((-0.0589999999999757, 0.0, 0.172550670854567)),
            carrier="torso",
        ),
        Socket(
            "right_hip",
            HIP,
            "base_link",
            Pose(
                (4.16270248897038e-05, -0.127999993231209, 0.0),
                (0.0, 3.14159265359, 0.000325211137678403),
            ),
        ),
        Socket("left_hip", HIP, "base_link"),
        Socket("right_wrist", WRIST, "right_wrist_yaw_link", carrier="right_arm"),
        Socket("left_wrist", WRIST, "left_wrist_yaw_link", carrier="left_arm"),
    ),
)

CAMERA_HOLDER: Final = InterfaceId("sentient-humanoid-camera-holder")

#: The D435 as the head fits it: ``camera_link``, below ``camera_joint``, with its colour stream on
#: the optical frame the module adds.
HEAD_CAMERA_MODULE: Final = Module(
    module_id=ModuleId("sentient-humanoid-head-camera"),
    label="Intel RealSense D435 in the humanoid's head",
    description=Subtree(SENTIENT_HUMANOID_URDF, below="camera_joint"),
    plug=CAMERA_HOLDER,
    frames=(HEAD_CAMERA,),
    components=(sensor_component("head", REALSENSE_D435_COLOR, RootMount(HEAD_CAMERA.name)),),
)

#: The vendor's head: a two-servo pan-tilt neck carrying the head, its holder fitted with the D435.
HEAD_MODULE: Final = Module(
    module_id=ModuleId("sentient-humanoid-head"),
    label="Sentient humanoid head on its two-servo neck",
    description=HEAD,
    plug=NECK,
    sockets=(
        Socket(
            "camera",
            CAMERA_HOLDER,
            "head_link",
            Pose((0.0453850000000015, -0.00450000000000009, 0.0774061362008152)),
            fitted=HEAD_CAMERA_MODULE,
        ),
    ),
)


def _leg_module(side: Literal["left", "right"], leg: ChainSpec, part: Subtree) -> Module:
    return Module(
        module_id=ModuleId(f"sentient-humanoid-{side}-leg"),
        label=f"Sentient humanoid {side} leg",
        description=part,
        plug=HIP,
        components=(
            body_component(
                f"{side}_leg", leg, RootMount(f"{side}_hip_pitch_joint_mount"), limb=Limb.LEG
            ),
        ),
    )


LEG_MODULES: Final = (
    _leg_module("right", SENTIENT_HUMANOID_RIGHT_LEG, LEGS[0]),
    _leg_module("left", SENTIENT_HUMANOID_LEFT_LEG, LEGS[1]),
)

#: A parallel gripper on small Dynamixel servos. Its servo model, bus ids and jaw geometry are
#: in no source, so it is a device with no drive the drivers take and a description of one link.
SENTIENT_HUMANOID_GRIPPER_MODULE: Final = Module(
    module_id=ModuleId("sentient-humanoid-parallel-gripper"),
    label="Sentient humanoid parallel gripper",
    description=b'<?xml version="1.0"?>\n<robot name="sentient_humanoid_gripper">\n'
    b'  <link name="mount"/>\n</robot>\n',
    plug=WRIST,
    components=(
        body_component(
            "gripper",
            DeviceSpec(
                PartId("sentient-humanoid-parallel-gripper"),
                "Parallel gripper on small Dynamixel servos; the servo model, its bus ids and "
                "the jaw's geometry are not captured",
            ),
            RootMount("mount"),
        ),
    ),
)


def _rejoin(name: str, module: Module, socket: str) -> Mate:
    """A cut part mated back on the upper body's seat under the vendor's names."""
    return Mate(name, module, host="upper_body", socket=socket, namespaced=False)


_UPPER: Final = Placed("upper_body", UPPER_BODY_MODULE, namespaced=False)
_HEAD: Final = _rejoin("head", HEAD_MODULE, "neck")

SENTIENT_HUMANOID_UPPER_BODY_BUILD: Final = Build(
    robot="sentient_humanoid_upper_body", root="humanoid_origin", placed=(_UPPER,), mates=(_HEAD,)
)
SENTIENT_HUMANOID_BUILD: Final = Build(
    robot="sentient_humanoid",
    root="humanoid_origin",
    placed=(_UPPER,),
    mates=(
        _HEAD,
        _rejoin("right_leg", LEG_MODULES[0], "right_hip"),
        _rejoin("left_leg", LEG_MODULES[1], "left_hip"),
    ),
)
SENTIENT_HUMANOID_UPPER_BODY_GRIPPERS_BUILD: Final = Build(
    robot="sentient_humanoid_upper_body_grippers",
    root="humanoid_origin",
    placed=(_UPPER,),
    mates=(
        _HEAD,
        Mate("right", SENTIENT_HUMANOID_GRIPPER_MODULE, host="upper_body", socket="right_wrist"),
        Mate("left", SENTIENT_HUMANOID_GRIPPER_MODULE, host="upper_body", socket="left_wrist"),
    ),
)


def _infrared(name: str) -> CameraSpec:
    """One D435 infrared imager at 848x480 and 60 fps, its optics from Intel's stated field of
    view (87 x 58 degrees, the depth module's, which the imagers set). The focal lengths are
    rounded to a micro-pixel, so every platform's ``tan`` gives the same digits and digest."""
    return CameraSpec(
        part_id=PartId(f"realsense-d435-{name}"),
        model=SensorModel("realsense_d435"),
        modality=CameraModality.INFRARED,
        fps=60.0,
        optics=CameraOptics(
            width=848,
            height=480,
            image_from_camera=(
                round(424.0 / math.tan(math.radians(87.0) / 2), 6),
                0.0,
                424.0,
                0.0,
                round(240.0 / math.tan(math.radians(58.0) / 2), 6),
                240.0,
                0.0,
                0.0,
                1.0,
            ),
            distortion_model="none",
            distortion_coefficients=(),
            authority=CameraOpticsAuthority.MODEL_NOMINAL,
            source=FactSource(
                repository="https://www.realsenseai.com/wp-content/uploads/2020/06/"
                "Intel-RealSense-D400-Series-Datasheet-June-2020.pdf",
                revision="337029-009",
                path="Depth module properties: depth field of view (HD 16:9)",
            ),
        ),
    )


#: The imagers' optical frames on the CAD camera link, placed from the colour frame by Intel's
#: nominal D435 offsets (``_d435.urdf.xacro``: infrared 1 is 15 mm right of colour, infrared 2 a
#: 50 mm baseline further), seen from behind the camera.
HEAD_INFRARED_FRAMES: Final = tuple(
    Frame(
        f"head_camera_{name}_optical",
        "camera_link",
        Pose((0.0, offset, 0.0), (-math.pi / 2, 0.0, -math.pi / 2)),
        preview_kind="camera",
        preview_label=label,
        camera_optical=True,
    )
    for name, offset, label in (
        ("infrared_1", -0.015, "Head infrared, left"),
        ("infrared_2", -0.065, "Head infrared, right"),
    )
)

#: The head's D435 publishing its infrared pair instead of its colour stream: a body option for
#: stereo viewing, not a registered body. Its pair is one ``head`` track
#: (:data:`HEAD_INFRARED_RIG`).
HEAD_INFRARED_CAMERA_MODULE: Final = Module(
    module_id=ModuleId("sentient-humanoid-head-infrared-camera"),
    label="Intel RealSense D435 in the humanoid's head, infrared pair",
    description=Subtree(SENTIENT_HUMANOID_URDF, below="camera_joint"),
    plug=CAMERA_HOLDER,
    frames=HEAD_INFRARED_FRAMES,
    components=tuple(
        sensor_component(side, _infrared(stream), RootMount(frame.name))
        for side, stream, frame in zip(
            ("head_left", "head_right"),
            ("infrared-1", "infrared-2"),
            HEAD_INFRARED_FRAMES,
            strict=True,
        )
    ),
)

#: The upper body with its head camera publishing the infrared pair.
SENTIENT_HUMANOID_UPPER_BODY_INFRARED_BUILD: Final = Build(
    robot="sentient_humanoid_upper_body",
    root="humanoid_origin",
    placed=(_UPPER,),
    mates=(
        _HEAD,
        Mate(
            "head_camera",
            HEAD_INFRARED_CAMERA_MODULE,
            host="head",
            socket="camera",
            namespaced=False,
        ),
    ),
)

#: The infrared pair as one side-by-side track.
HEAD_INFRARED_RIG: Final = MediaRig(stereo=(StereoPair("head", "head_left", "head_right"),))

SENTIENT_HUMANOID_UPPER_BODY_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("sentient-humanoid-upper-body"),
    label="Sentient humanoid upper body",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="sentient-humanoid", variant="upper-body"),
    build=SENTIENT_HUMANOID_UPPER_BODY_BUILD,
    extra_assets=(
        SENTIENT_HUMANOID_SOURCE_URDF,
        _generated(
            "humanoid_pkg/urdf/sentient_humanoid_upper_body.urdf",
            "9c2901aa4251525cccc082cf8abcc00492f1ceed7cf4a9248d263f66b36d4442",
            23708,
        ),
    ),
)

SENTIENT_HUMANOID_UPPER_BODY_GRIPPERS_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("sentient-humanoid-upper-body-grippers"),
    label="Sentient humanoid upper body with its grippers",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="sentient-humanoid", variant="upper-body-grippers"),
    build=SENTIENT_HUMANOID_UPPER_BODY_GRIPPERS_BUILD,
    extra_assets=(
        SENTIENT_HUMANOID_SOURCE_URDF,
        _generated(
            "humanoid_pkg/urdf/sentient_humanoid_upper_body_grippers.urdf",
            "54d7d9a1f657aa5cf4633498c06687319af4192f1957eca09663f5379ea3342e",
            24144,
        ),
    ),
)

SENTIENT_HUMANOID_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("sentient-humanoid"),
    label="Sentient humanoid",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="sentient-humanoid"),
    build=SENTIENT_HUMANOID_BUILD,
    extra_assets=(
        SENTIENT_HUMANOID_SOURCE_URDF,
        _generated(
            "humanoid_pkg/urdf/sentient_humanoid.urdf",
            "add9520df125459c92e7480ead66e8a4a56d98107379458ae5d7d8742402b3d4",
            41203,
        ),
        SENTIENT_HUMANOID_MJCF,
    ),
)
