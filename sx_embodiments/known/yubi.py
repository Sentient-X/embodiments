"""The YUBI bimanual recording description: parallel-jaw CAD on the published frames.

The kinematic tree is the one composed from the two supplied four-bar Onshape CAD exports:
every link, joint, origin, axis, limit and mimic of ``description7`` is unchanged, so a
recorded controller pose and jaw angle place every frame exactly where they did. Revision
``description8`` replaces only what is drawn on those frames. The meshes are the parallel-jaw
CAD supplied on 2026-11-26 (whether it is the fielded hardware of sx#1354's cohort has not
been checked against a unit), registered to the published frames on the four pivot pins
(finger pins 30 mm apart, connector pins 110 mm apart; under 0.01 mm residual) and written
in each link's own frame. It also adds one fixed marker frame
on each finger, ``{side}_{left,right}_aruco``, at the marker frames of the right hand's CAD
export: on the finger's top face, z out of the face. They are the CAD's nominal places for
the two 15 mm ``DICT_4X4_50`` markers of ``sx_embodiments.jaw_tags`` (right IDs 0/1, left
2/3), not measured on a unit; which ID is on which finger is not recorded here.

The right hand's meshes come from the right parallel-jaw Onshape URDF export (its
``assembly_1.urdf`` is sha256 59f435a3...c166d3); the left hand's mechanism is the same parts
(0.3-0.5 mm against the left STEP export, sha256 a8c2caab...c67c4e) and shares them, with
its own controller and controller holder from the left STEP. The source CAD
documents' URLs are not recorded, so each mesh's provenance names this repository and its
own digest.

What the parallel-jaw CAD says differently from the frames kept here, none of it measured
on a unit:

- its controller body sits 3-5 mm and 8-9 degrees from the ``description7`` controller mesh
  (ICP), and the export's own controller frame 7.4 mm from ``{side}_controller_link``;
- its wrist camera frame is 3.9 mm from ``{side}_hand_cam_optical``, same orientation;
- its jaw stops at 0.66-0.67 rad, short of the 0.785398 rad joint box kept here.

Each hand preserves the housing, controller, finger, connector, nail, grip, tip, and
camera links. ``left_finger`` is the one encoder-driven joint; the other finger and the
connector/nail joints mimic it, which keeps the two nails parallel through the travel. The
explicit Quest tracking frame includes the measured +90 degree tracking-to-CAD controller
basis used by the capture viewer.

``quest_{side}_controller`` is the capture apps' tracked controller pose (the one OQC/Unity
records); per-unit checkerboard mount calibrations against real hand-camera images leave
only 1-3 degree residuals on it. The WebXR/OpenXR grip space that browsers and native OpenXR
apps report is a different pose: fitting the ``description7`` ``{side}_controller_link`` mesh
(a Meta Quest Touch Plus) by ICP to Meta's grip-space asset puts that grip space about 45
degrees and 3 cm from ``quest_{side}_controller``. A WebXR viewer that took grip space for
the capture pose would be off by that much, which plausibly explains the prototypes' ~42
degree live error. :data:`YUBI_TRACKING` is that measured grip space, unchanged because
``{side}_controller_link`` is unchanged; viewers read it from the render pack. It was not
refitted against the ``description8`` controller mesh.

The description bytes changed with this revision, so it has a new content id. Episode seals
and per-unit mount calibrations are keyed to the bytes: the ``description7`` document and its
files stay published (``historical/``, ``yubi_description/_by_digest/``) and existing
recordings keep resolving against them.

The aperture curve is the ``description7`` inner nail-mesh separation across the URDF joint
box, kept. The parallel-jaw nails open from 0 to 99.9 mm over the same box and stay within
0.4 mm of it. Per-unit camera calibration remains episode/session evidence.
"""

from typing import Final

from sx_contracts import ComponentId
from sx_contracts.assets import AssetFormat, AssetProvenance, AssetRole

from ..assets import PackagedAsset, packaged_asset
from ..collection import CollectionDevice, CollectionDeviceKind, CollectionMethod, CollectionSetup
from ..compose import (
    Direction,
    EmbodimentDefinition,
    OperatorMount,
    OperatorSite,
    RootMount,
    body_component,
)
from ..curves import Curve1D, Knot
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit, EncoderReadout, Passive
from ..mechanical import Build, Module, ModuleId
from ..parts import GripperSpec, MimicJoint
from ..recorded_streams import (
    DeviceStream,
    Measurand,
    RecordedCamera,
    RecordedCoordinate,
    RecordedPose,
    RecordedStreams,
)
from ..sites import BodySite
from ..viewing import TrackedDevice, Viewing
from ._authoring import bounded_layout
from .sources import capture_source

YUBI_HANDS_URDF: Final = packaged_asset(
    relpath="yubi_description/urdf/yubi_hands.urdf",
    sha256="827b1b8433e3ce5d7c739466c9b318f66950a21801a56eb81af6e6de018212ea",
    size_bytes=20999,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Sentient-X/embodiments",
        revision="yubi-description8-parallel-jaws",
        path="assets/yubi_description/urdf/yubi_hands.urdf",
        license_id="LicenseRef-Sentient-Proprietary",
        generator="tools/compose_yubi_urdf.py",
    ),
    media_type="application/xml",
)


def _yubi_mesh(relpath: str, sha256: str, size_bytes: int) -> PackagedAsset:
    # The parallel-jaw CAD documents' URLs are not recorded; the bytes are first published
    # here, so the provenance names this repository and the file's own digest.
    return packaged_asset(
        relpath=f"yubi_description/meshes/{relpath}",
        sha256=sha256,
        size_bytes=size_bytes,
        format=AssetFormat.MESH,
        role=AssetRole.GEOMETRY,
        provenance=AssetProvenance(
            repository="https://github.com/Sentient-X/embodiments",
            revision=f"sha256:{sha256}",
            path=f"assets/yubi_description/meshes/{relpath}",
            license_id="LicenseRef-Sentient-Proprietary",
        ),
        media_type="model/stl",
    )


# Complete mesh closure of YUBI_HANDS_URDF. These are part of the embodiment's
# content identity, not merely files that happen to ship beside the URDF.
YUBI_MESHES: Final[tuple[PackagedAsset, ...]] = (
    _yubi_mesh(
        "shared/link_base_link.stl",
        "56619ceeba78e416a2e7c2e21ff6990580e50fccdcf7f682ad75425f08278c96",
        3104384,
    ),
    _yubi_mesh(
        "shared/link_connector_left.stl",
        "d59b2e99088bdbedcb7182ac660541a2381bf81bab0c56626bde9bd3f9509b7a",
        225784,
    ),
    _yubi_mesh(
        "shared/link_connector_right.stl",
        "5285cf025395fd4f50d8379b29cde5b4de27fe18dbdd5b92ce14d3b2dcbc5174",
        225784,
    ),
    _yubi_mesh(
        "shared/link_left_finger.stl",
        "abf0bb4df18085733a3622c4d4f152d69f35bc59461523e8c6327ec72245ff05",
        763384,
    ),
    _yubi_mesh(
        "shared/link_left_nail.stl",
        "b80b6853f9a75f0d62d6af2038646ce03c46953dfbf076901252161dd6b5865c",
        1055984,
    ),
    _yubi_mesh(
        "shared/link_right_finger.stl",
        "b752838cfae466b03a1ea6da11607eaea16280f8c0d43bb976c095202b7fcc8a",
        819984,
    ),
    _yubi_mesh(
        "shared/link_right_nail.stl",
        "fe3ffb302aca9131137e914abab84e3de4c9c985b66ac6b3a93eef4f3af7c590",
        1051884,
    ),
    _yubi_mesh(
        "left/link_controller.stl",
        "526dc0b4840e861bc0a1c42bccfe1fb7d213aca1a8851e70079d1a100b790866",
        278284,
    ),
    _yubi_mesh(
        "left/link_housing.stl",
        "22fc2c87547a175c3e759c65194c0a23781010a26aee884c962df5d8bcdd8fbc",
        4905284,
    ),
    _yubi_mesh(
        "right/link_controller.stl",
        "507e790be417a0f59e16fbd34e6d67245c61fd9924af1317507a8a6642fb274f",
        278284,
    ),
    _yubi_mesh(
        "right/link_housing.stl",
        "c1cfa61e4890e5fb1d78382268faf9f44b0d56ad5d8e85fba1723cc819ad2579",
        5678684,
    ),
)

YUBI_JAW: Final = GripperSpec(
    part_id=PartId("yubi-jaw-v2"),
    layout=bounded_layout(
        names=("left_finger",),
        units=(CoordinateUnit.RADIAN,),
        lower=(0.0,),
        upper=(0.785398,),
        observations=(EncoderReadout("as5601_usb_serial"),),
        actuations=(Passive("handheld jaw has no commanded actuation path"),),
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

YUBI_MODULE: Final = Module(
    module_id=ModuleId("yubi"),
    label="YUBI handheld jaws",
    description=YUBI_HANDS_URDF,
    sites=(
        BodySite("left_hand", ComponentId("left_jaw"), "quest_left_controller"),
        BodySite("right_hand", ComponentId("right_jaw"), "quest_right_controller"),
    ),
    components=(
        body_component("left_jaw", YUBI_JAW, RootMount("left_hand_root")),
        body_component("right_jaw", YUBI_JAW, RootMount("right_hand_root")),
    ),
)


_TOUCH_PLUS_PROVENANCE: Final = (
    "ICP (RMS 0.69 mm) of the description7 {side}_controller_link CAD mesh against Meta's "
    "webxr-input-profiles assets@1.0 meta-quest-touch-plus/{side}.glb, defined in grip space"
)

#: The Touch Plus WebXR/OpenXR grip space in each controller link (``controller_link_from_grip``).
YUBI_TRACKING: Final = (
    TrackedDevice(
        side="left",
        device="meta-quest-touch-plus",
        space="openxr-grip",
        frame="left_controller_link",
        translation_m=(-0.018771, -0.002003, -0.019933),
        rotation_xyzw=(0.664879, -0.635962, -0.281326, 0.272661),
        provenance=_TOUCH_PLUS_PROVENANCE.format(side="left"),
    ),
    TrackedDevice(
        side="right",
        device="meta-quest-touch-plus",
        space="openxr-grip",
        frame="right_controller_link",
        translation_m=(-0.021116, 0.001244, -0.021668),
        rotation_xyzw=(-0.645548, 0.666414, 0.250385, -0.276528),
        provenance=_TOUCH_PLUS_PROVENANCE.format(side="right"),
    ),
)


def _camera(name: str, frame: str, device: str, camera: str) -> RecordedCamera:
    return RecordedCamera(
        name=name,
        frame=frame,
        video=DeviceStream(f"/{device}/{camera}/compressed"),
        calibration=DeviceStream(f"/{device}/{camera}/camera_info"),
    )


#: A pod's cameras as its seal records them (sx-seal: the Quest's head pair, each hand's
#: ``camera0``), named as floor reviewers read them. They are bindings, not cameras of the
#: body: the wrist cameras' model, optics and mount are unmeasured, and the head pair is the
#: Quest's, which neither hand rig's description contains. Each episode carries its own
#: measured calibration on the calibration stream.
POD_CAMERAS: Final = (
    _camera("head_left", "quest3s_left_camera_optical", "quest3s", "head_left/camera"),
    _camera("head_right", "quest3s_right_camera_optical", "quest3s", "head_right/camera"),
    _camera("left_wrist", "left_hand_cam_optical", "gripper", "left/camera0"),
    _camera("right_wrist", "right_hand_cam_optical", "gripper", "right/camera0"),
)


def pod_hands(left_link: str, right_link: str) -> tuple[RecordedPose, ...]:
    """Each Quest controller's pose, world-from-``<side>`` controller link. The headset's own
    pose (``/quest3s/head_pose``) is bound to nothing: neither hand rig's description has a
    head, so it stays evidence until a revision composes the headset (a new id and delivery
    URDF, which waits for the pods' shadow release)."""
    return (
        RecordedPose("left_hand", DeviceStream("/quest3s/left_controller_pose"), left_link),
        RecordedPose("right_hand", DeviceStream("/quest3s/right_controller_pose"), right_link),
    )


YUBI_SPEC: Final = EmbodimentDefinition(
    collection=CollectionSetup(
        method=CollectionMethod.UMI,
        description="Handheld grippers for demonstrations with wrist video and jaw sensing.",
        devices=(
            CollectionDevice(
                CollectionDeviceKind.CAMERA,
                "UVC wrist camera",
                2,
                "Left and right gripper",
                "Wrist video. Exact camera model and lens are not documented; optics and mounts "
                "must be measured for each unit.",
                capture_source("experience/data-factory/pod/src/capture.rs"),
            ),
            CollectionDevice(
                CollectionDeviceKind.ENCODER,
                "AS5601 magnetic encoder",
                2,
                "Left and right jaw",
                "Jaw angle over USB serial. Zero and jaw-aperture calibration are per unit.",
                capture_source("experience/data-factory/pod/src/capture.rs"),
            ),
            CollectionDevice(
                CollectionDeviceKind.TRACKING,
                "Quest headset and controllers",
                1,
                "Operator head and hands",
                "Head and hand tracking. Headset model and available "
                "video streams depend on the installation.",
                capture_source("experience/data-factory/docs/COLLECTING-YUBI-DATA.md"),
            ),
        ),
        notes=(
            "Camera calibration and mounting transforms are incomplete in the hardware registry.",
        ),
    ),
    name=EmbodimentName("yubi"),
    label="YUBI bimanual handheld capture rig",
    kind=EmbodimentKind.CAPTURE_RIG,
    # The root is the recording's room, which the Quest converter writes x right, y ahead, z up
    # (sxd umico quest.py); the floating hands are placed in it.
    front=Direction.POS_Y,
    lineage=Lineage(family="yubi", revision="description8-sx1"),
    build=Build.of(YUBI_MODULE),
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
    # Both hands are in every frame beside the robot, so each is drawn lighter.
    viewing=Viewing(tracking=YUBI_TRACKING, triangle_budget=60_000),
    # A YUBI pod's seal records each Quest controller's pose and each jaw's opening in metres
    # (sx-seal's process/gripper.rs applies the encoder law before it writes).
    recorded_streams=RecordedStreams(
        poses=pod_hands("quest_left_controller", "quest_right_controller"),
        coordinates=(
            RecordedCoordinate(
                "left_jaw", DeviceStream("/gripper/left/magnetic_encoder"), Measurand.JAW_APERTURE_M
            ),
            RecordedCoordinate(
                "right_jaw",
                DeviceStream("/gripper/right/magnetic_encoder"),
                Measurand.JAW_APERTURE_M,
            ),
        ),
        cameras=POD_CAMERAS,
    ),
)
