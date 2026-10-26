"""Sentient static upper body: a torso with two seven-joint arms, clamped to a table.

The robot sits on a table edge facing out from it, where it works a coffee machine: the base
is clamped to the table top, so the description's root ``base_link`` is the table surface. A
yaw torso carries two arms of shoulder pitch, roll and yaw, elbow pitch, forearm yaw, wrist
pitch and roll, each ending in a two-finger gripper with a ``*_gripper_tcp`` frame, and an
SO-101 arm on the chest used as a six-joint neck whose last joint tilts a ZED camera.

The description is ``tools/compose_static_upper_body_urdf.py``'s rendering of
``source/allisa_bot.urdf``, Subraj's ``humanoid_pkg`` ``urdf/allisa_bot.urdf`` of 2026-10-25,
byte for byte, with the package's display meshes. It replaces the ``allisa_feedback``
description this body was first registered from (``source/received.urdf`` until 2026-10-26, in
git history): "The operator rejected the feedback geometry and selected
``urdf/allisa_bot.urdf`` instead. The feedback model is a joint-limit donor only"
(``source/allisa_bot_integration.md``). allisa_bot is that CAD export with the donor's limits
copied onto its 25 movable joints by name (``source/allisa_bot_model_provenance.json``). The
two agree on every link, visual and joint origin; allisa_bot reverses both wrist axes on each
arm, has no collision geometry, and carries a ``zed2i_left_optical_frame`` where the feedback
file had a ``ZED_camera``. Both are Sentient's own CAD (``LicenseRef-Sentient-Proprietary``;
the package ships no licence of its own).

The bindings are humanoid_pkg's hardware profile for it
(``source/humanoid_allisa_bot_hardware.json``): per arm joint the Damiao model, CAN send and
answer ids (answer = send + 0x20, deployed on ``can0``), MIT gains and a sign. It repeats the
feedback file's motor table, whose limits, mapping and directions were flagged confirmed on the
robot, exactly, except that both wrists' motors read with sign -1, because allisa_bot turns
those axes against them. Their confirmed limits are motor coordinates, so the rendering
negates them onto allisa_bot's axes (the package copies them by name unnegated, and says
hardware signs still need evidence). The profile has no torso entry, so the torso's binding
(at 0x1F) is the superseded table's. The torso and the four shoulder motors are DM-J8009P-2EC
(``ActuatorModel.DAMIAO_DM8009P``, as the meshes are), which the tables wrote as "DM8009": the
same wire limits, but a distinct product rated 24-48 V.

* **Elbows** bend both ways about the forearm hanging straight (-pi/2 left, +pi/2 right): about
  90 deg forward to the stop at zero and about 80 deg back, the robot's confirmed stops.
* **Forearm yaw** turns 8.7 rad (left) and 7.4 rad (right), more than once; also confirmed.

The rendering changes the source only where simulation and the registry need it:

* **Mass properties.** No link has an inertial. Each link gets its visual meshes' convex hull
  at one density (600 kg/m^3), 11.5 kg in all; the CAD's mass properties replace them, and the
  estimate is likely light on the arm actuators. This is why it is still a development body.
* **Collision.** No link has collision geometry; each link with geometry gets its visual hull.
* Visual names are made unique, meshes are referenced as ``meshes/<name>.stl``, the four wrist
  limits are negated as above, and the Gazebo ``<sensor>`` element is dropped (the registry
  declares the camera).

Still open with the hardware owner: every arm joint declares 54 Nm and 0.5 rad/s, too slow to
replay human motion, and the superseded driver configuration capped speed at 0.25 rad/s; left
and right do not mirror one sign convention; the right arm's first id is 0x09 where the rest of
that arm runs 0x10..0x15; the gains (kp 20..61, kd 1..5) are that driver's, not a
qualification. The feedback table flagged the right forearm yaw's direction confirmed while its
note said "model pose/direction calibration still pending"; the profile keeps its sign +1.

The neck carries a Stereolabs ZED 2i (:mod:`.zed`): the rendering adds ``ZED_camera`` (x out of
the lenses, y left) on the source's ``zed2i_left_optical_frame`` so that the left eye's optical
frame is the source's own, and the ``head_left`` and ``head_right`` cameras on optical frames
60 mm either side of it, published as one side-by-side ``head`` track
(:data:`STATIC_UPPER_BODY_RIG`). The source puts its left eye at ``camera_link`` +z, where the
feedback file's ``ZED_camera`` was turned half a turn about the lens axis: left and right are
swapped relative to the earlier registration.

**The neck and grippers are provisional, in data.** Each of their bindings is marked
``provisional`` (:class:`~sx_embodiments.layout.ActuatorBinding`): no driver moves them unless
a person bringing the robot up opts in for their port, and the station's normal path drives
the fifteen Damiao axes with them left off.

*The neck.* No profile covers it (humanoid_pkg introduces no torso, SO-101 or camera control).
A read-only probe on 2026-10-25 found it on its own USB serial bus (a WCH CH343, 1,000,000
baud): six Feetech STS3215s (model number 777) at ids 1..6 in SO-101 order, the sixth the
camera tilt where the SO-101's gripper servo was, configured by LeRobot. They are bound here on
a serial bus named ``neck``, identity drive map, each read in LeRobot's degree normalization.

**The neck is not allisa_bot's.** allisa_bot's SO-101 is the CAD export the feedback file had:
arbitrary joint zeros, links 15..29 mm off along their pitch axes, a base mounted facing the
wrong way, and at its own zero pose it penetrates the robot (9 mm into ``base_link``, 41 mm
for the camera); the owner, watching it live, reported it breaking apart. From
``so101_shoulder_pan_joint`` down the neck is TheRobotStudio's ``so101_new_calib`` chain
(SO-ARM100 ``5f6d2b8``, Apache-2.0, vendored as ``source/so101_new_calib.urdf``; its meshes
are the public ``so101`` package's, the same bytes) under our names: joint origins, axes and limits, inertials and meshes. Its
zeros are LeRobot's calibrated mid-range, so the readings are the joint angles with no offset.
allisa_bot's base, its custom holder for the camera servo, the camera bracket and the ZED 2i
meshes ride on it, placed by registering each official part onto allisa_bot's CAD. The base is
turned half a turn, as the robot is built: with it, a recording taken at the side photo of
2026-10-25 reproduces that photo and the front and rear ones with zero offsets. How the bracket
sits on the tilt servo's horn is fitted to the side photo (about 15 deg), so the camera tilt
alone carries a fitted angle, in the description. Its declared range is the free arc it rests
in: the official joint's limits are the jaw's, and the camera meets the wrist between about
+10 and +130 deg. The robot rests folded just outside the other ranges (shoulder lift -102 deg,
wrist flex -105 deg), and the servos' calibration registers are a unit's facts, supplied at
open.

*The grippers.* Each finger has its own Feetech STS3215 (model 777), each gripper's two on a USB
serial bus of their own (a CH343, 1,000,000 baud). humanoid_pkg's Feetech record
(``source/allisa_bot_feetech.json``) confirms the sides by board serial: ``right_gripper`` is
the board answering at ids 1 (left finger) and 2 (right), ``left_gripper`` at 3 (left) and 4
(right). It records each finger's closed and open ticks (operator-confirmed closed,
2026-10-23); each finger's declared range is the description's limit.
"""

from typing import Final

from sx_contracts import ComponentId

from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import (
    BaseMount,
    EmbodimentDefinition,
    MountedOn,
    MountKind,
    RootMount,
    body_component,
    sensor_component,
)
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import (
    ActuatorBinding,
    ActuatorBus,
    ActuatorFeedback,
    ActuatorModel,
    BusPort,
    CoordinateUnit,
    DirectDrive,
)
from ..mechanical import Build, Module, ModuleId
from ..media import MediaRig, StereoPair
from ..parts import ChainSpec, GripperSpec, Limb
from ..sites import BodySite
from ..viewing import VirtualGripper, Workcell
from ._authoring import bounded_layout
from .zed import EYES, ZED_FRAME, optical_frame, zed_2i_eye, zed_2i_imu

_SOURCE: Final = AssetProvenance(
    repository="https://github.com/Sentient-X/humanoid_pkg",
    # The sha256 of humanoid_pkg's urdf/allisa_bot.urdf as vendored; no commit was supplied.
    revision="039b0b7d98893030d5ca80c50eea779ef8fad5b720b8c110685ce010c03b2571",
    path="urdf/allisa_bot.urdf",
    license_id="LicenseRef-Sentient-Proprietary",
)

STATIC_UPPER_BODY_URDF: Final = packaged_asset(
    relpath="sentient_static_upper_body/sentient_static_upper_body.urdf",
    sha256="fd19313e0cbff9d648e2e56fac5ef3eb1e23980a69cbb9ebe6adcf24d395f697",
    size_bytes=221406,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository=_SOURCE.repository,
        revision=_SOURCE.revision,
        path=_SOURCE.path,
        license_id=_SOURCE.license_id,
        generator="sx-embodiments/tools/compose_static_upper_body_urdf.py",
    ),
    media_type="application/xml",
)

_ARM_JOINTS: Final = (
    "shoulder_pitch",
    "shoulder_roll",
    "upper_arm_yaw",
    "elbow_pitch",
    "forearm_yaw",
    "wrist_pitch",
    "wrist_roll",
)
_ARM_MODELS: Final = (
    ActuatorModel.DAMIAO_DM8009P,
    ActuatorModel.DAMIAO_DM8009P,
    ActuatorModel.DAMIAO_DM4340,
    ActuatorModel.DAMIAO_DM4340,
    ActuatorModel.DAMIAO_DM4340,
    ActuatorModel.DAMIAO_DM4310,
    ActuatorModel.DAMIAO_DM4310,
)
#: Each arm's CAN send ids in ``_ARM_JOINTS`` order, as the received motor table lists them.
_ARM_IDS: Final = {
    "left": (0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E),
    "right": (0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15),
}
#: The motors' answer ids are their send ids plus this.
STATIC_UPPER_BODY_MASTER_ID_OFFSET: Final = 0x20


#: Each arm's motor signs in ``_ARM_JOINTS`` order: allisa_bot turns both wrist axes against
#: their motors, which the hardware profile reads with sign -1.
_ARM_SIGNS: Final = (1, 1, 1, 1, 1, -1, -1)


def _damiao(model: ActuatorModel, bus_id: int, sign: int = 1) -> DirectDrive:
    """One motor on the body's one Damiao CAN bus, direct drive."""
    return DirectDrive(
        ActuatorBinding(model=model, bus=ActuatorBus.DAMIAO_CAN, bus_id=bus_id, sign=sign)
    )


STATIC_UPPER_BODY_TORSO: Final = ChainSpec(
    part_id=PartId("sentient-static-upper-body-torso"),
    layout=bounded_layout(
        names=("torso_joint",),
        units=(CoordinateUnit.RADIAN,),
        lower=(-1.221293965056839,),
        upper=(1.1499580376897836,),
        actuations=(_damiao(ActuatorModel.DAMIAO_DM8009P, 0x1F),),
        observations=(ActuatorFeedback(),),
    ),
    home=(0.0,),
)


def _arm(side: str, lower: tuple[float, ...], upper: tuple[float, ...]) -> ChainSpec:
    return ChainSpec(
        part_id=PartId(f"sentient-static-upper-body-{side}-arm"),
        layout=bounded_layout(
            names=tuple(f"{side}_{name}_joint" for name in _ARM_JOINTS),
            units=(CoordinateUnit.RADIAN,) * 7,
            lower=lower,
            upper=upper,
            actuations=tuple(
                _damiao(model, bus_id, sign)
                for model, bus_id, sign in zip(_ARM_MODELS, _ARM_IDS[side], _ARM_SIGNS, strict=True)
            ),
            observations=(ActuatorFeedback(),) * 7,
        ),
        home=(0.0,) * 7,
    )


STATIC_UPPER_BODY_LEFT_ARM: Final = _arm(
    "left",
    lower=(
        -2.642671854734111,
        -0.3873884184023808,
        -5.1230258640421145,
        -2.9753185320820936,
        -6.82669565880827,
        -2.4408712901503016,
        -2.0967803463797967,
    ),
    upper=(
        2.889868009460594,
        1.2018387121385512,
        0.8070115205615327,
        0.000953688868543523,
        1.8839169909208824,
        2.1280613412680243,
        2.096017395284962,
    ),
)

STATIC_UPPER_BODY_RIGHT_ARM: Final = _arm(
    "right",
    lower=(
        -2.9139009689478907,
        -1.2101034267894413,
        -2.4210345616845963,
        -0.029182879377431803,
        -6.009575036240177,
        -2.255855649652858,
        -2.0986877241168838,
    ),
    upper=(
        3.1225680933852136,
        0.35464911018127765,
        2.7597848477912574,
        2.966544594491493,
        1.4261463340199896,
        2.308880750743878,
        2.112039368276493,
    ),
)

#: The neck's serial bus, as commissioning names its bridge. Provisional, as the module says.
STATIC_UPPER_BODY_NECK_PORT: Final = BusPort("neck")


def _sts3215(bus_id: int, port: BusPort) -> DirectDrive:
    """One STS3215 on one of the body's Feetech buses, identity drive map, marked provisional:
    no driver moves it until a supervised bring-up opts in for its port."""
    return DirectDrive(
        ActuatorBinding(
            model=ActuatorModel.FEETECH_STS3215,
            bus=ActuatorBus.FEETECH_SERIAL,
            bus_id=bus_id,
            port=port,
            provisional=True,
        )
    )


#: The SO-101 on the chest, its gripper replaced by a camera tilt joint carrying the ZED.
STATIC_UPPER_BODY_NECK: Final = ChainSpec(
    part_id=PartId("sentient-static-upper-body-neck"),
    layout=bounded_layout(
        names=tuple(
            f"so101_{name}_joint"
            for name in (
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
                "camera_tilt",
            )
        ),
        units=(CoordinateUnit.RADIAN,) * 6,
        # The official new-calibration SO-101's limits; the camera tilt's is the free arc it
        # rests in (the official joint's limits are its jaw's, and the camera meets the wrist).
        lower=(-1.91986, -1.74533, -1.69, -1.65806, -2.74385, -3.14159265),
        upper=(1.91986, 1.74533, 1.69, 1.65806, 2.84121, -0.261799388),
        actuations=tuple(_sts3215(bus_id, STATIC_UPPER_BODY_NECK_PORT) for bus_id in range(1, 7)),
        observations=(ActuatorFeedback(),) * 6,
    ),
    # Zero but the camera tilt, which rests where the robot was recorded holding it.
    home=(0.0, 0.0, 0.0, 0.0, 0.0, -2.2),
)


#: Each gripper's serial bus, as commissioning names its bridge, and its two servos' ids, the
#: left finger's first. Provisional, as the module says: which bus is which side, and which id
#: which finger, is not yet checked on the robot.
STATIC_UPPER_BODY_GRIPPER_BUSES: Final = {
    "right": (BusPort("right_gripper"), (1, 2)),
    "left": (BusPort("left_gripper"), (3, 4)),
}


def _gripper(side: str, left_finger: float, right_finger: float) -> GripperSpec:
    """Two revolute fingers, each on its own STS3215; no source says how their travel couples,
    so neither a mimic nor an aperture is declared."""
    port, ids = STATIC_UPPER_BODY_GRIPPER_BUSES[side]
    return GripperSpec(
        part_id=PartId(f"sentient-static-upper-body-{side}-gripper"),
        layout=bounded_layout(
            names=(f"{side}_gripper_left_finger_joint", f"{side}_gripper_right_finger_joint"),
            units=(CoordinateUnit.RADIAN,) * 2,
            lower=(left_finger, 0.0),
            upper=(0.0, right_finger),
            actuations=tuple(_sts3215(bus_id, port) for bus_id in ids),
            observations=(ActuatorFeedback(),) * 2,
        ),
    )


STATIC_UPPER_BODY_LEFT_GRIPPER: Final = _gripper("left", -0.823747683095, 0.929592357459)
STATIC_UPPER_BODY_RIGHT_GRIPPER: Final = _gripper("right", -0.952602069277, 0.788466124973)

STATIC_UPPER_BODY_MODULE: Final = Module(
    module_id=ModuleId("sentient-static-upper-body"),
    label="Sentient static upper body",
    description=STATIC_UPPER_BODY_URDF,
    components=(
        body_component("torso", STATIC_UPPER_BODY_TORSO, RootMount("base_link"), limb=Limb.TORSO),
        body_component(
            "left_arm", STATIC_UPPER_BODY_LEFT_ARM, MountedOn("torso", "chest_link"), limb=Limb.ARM
        ),
        body_component(
            "right_arm",
            STATIC_UPPER_BODY_RIGHT_ARM,
            MountedOn("torso", "chest_link"),
            limb=Limb.ARM,
        ),
        body_component(
            "left_gripper",
            STATIC_UPPER_BODY_LEFT_GRIPPER,
            MountedOn("left_arm", "left_gripper_base_link"),
        ),
        body_component(
            "right_gripper",
            STATIC_UPPER_BODY_RIGHT_GRIPPER,
            MountedOn("right_arm", "right_gripper_base_link"),
        ),
        body_component(
            "neck", STATIC_UPPER_BODY_NECK, MountedOn("torso", "so101_base_link"), limb=Limb.HEAD
        ),
        *(
            sensor_component(f"head_{eye}", zed_2i_eye(eye), MountedOn("neck", optical_frame(eye)))
            for eye in EYES
        ),
        sensor_component("head_imu", zed_2i_imu(), MountedOn("neck", ZED_FRAME)),
    ),
    # Link origins: the wrist roll axis centre, the elbow pitch axis, the chest, and the
    # grippers' tool points (the description's *_gripper_tcp frames).
    sites=(
        BodySite("left_wrist", ComponentId("left_arm"), "left_wrist_roll_link"),
        BodySite("right_wrist", ComponentId("right_arm"), "right_wrist_roll_link"),
        BodySite("left_hand", ComponentId("left_gripper"), "left_gripper_tcp"),
        BodySite("right_hand", ComponentId("right_gripper"), "right_gripper_tcp"),
        BodySite("left_elbow", ComponentId("left_arm"), "left_elbow_pitch_link"),
        BodySite("right_elbow", ComponentId("right_arm"), "right_elbow_pitch_link"),
        BodySite("chest", ComponentId("torso"), "chest_link"),
    ),
)

#: The ZED's two eyes as one side-by-side track.
STATIC_UPPER_BODY_RIG: Final = MediaRig(stereo=(StereoPair("head", "head_left", "head_right"),))

STATIC_UPPER_BODY_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("sentient-static-upper-body"),
    label="Sentient static upper body",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="sentient-static-upper-body"),
    build=Build.of(STATIC_UPPER_BODY_MODULE),
    # The xy box of base_link's collision hull, the clamp plate on the table top.
    base_mount=BaseMount(
        kind=MountKind.CLAMP_EDGE,
        frame="base_link",
        half_extents=(0.0952, 0.055),
        centre=(0.0402, 0.0),
    ),
    # The received file stays in the asset tree as provenance only: its package:// mesh
    # paths are the received folder's, so it is not a loadable asset of this body.
    extra_assets=(STATIC_UPPER_BODY_URDF,),
)

#: The table top (base_link, the base mount's surface) above the floor, from the received
#: CAD's ``world_to_base`` joint.
STATIC_UPPER_BODY_WORKCELL: Final = Workcell(
    floor_to_frame_m=0.797200208452,
    provenance="humanoid_pkg allisa_feedback.urdf joint world_to_base",
)

#: The operator's empty controllers drive this body's own grippers, 9 cm ahead of the grip.
STATIC_UPPER_BODY_VIRTUAL_GRIPPER: Final = VirtualGripper(ahead_m=0.09)
