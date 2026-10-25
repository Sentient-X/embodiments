"""Sentient static upper body: a torso with two seven-joint arms, clamped to a table.

The robot sits on a table edge facing out from it, where it works a coffee machine: the base
is clamped to the table top, so the description's root ``base_link`` is the table surface
(the received file put it 0.797 m above a floor through a fixed ``world`` joint, which is
that table's height and a placement fact, not part of the body). A yaw torso carries two
arms of shoulder pitch, roll and yaw, elbow pitch, forearm yaw, wrist pitch and roll, each
ending in a two-finger gripper with a ``*_gripper_tcp`` frame, and an SO-101 arm on the chest
used as a six-joint neck whose last joint tilts a ZED camera (``ZED_camera`` frame).

The description is ``tools/compose_static_upper_body_urdf.py``'s rendering of the
description as received on 2026-10-25 (maintainer ``suprajs``, licence ``Proprietary``; its
meshes resolve under ``humanoid_pkg``'s ``models/allisa_current``), kept byte for byte under
``source/``. That file carries the robot's Damiao motor table: per driven joint the model, CAN
send and answer ids (answer = send + 0x20, on ``can1``), MIT gains, limits, and flags that its
mapping, direction and limits are confirmed on the robot. The bindings below are that table;
the joint limits pass through unchanged, and each joint drives its motor with identity sign.

* **Elbows** bend both ways about the forearm hanging straight (-pi/2 left, +pi/2 right): about
  90 deg forward to the stop at zero and about 80 deg back. The first rendering read the
  backward half as impossible and moved the range; the robot's confirmed stops replace that.
* **Forearm yaw** turns 8.7 rad (left) and 7.4 rad (right), more than once; also confirmed.

The rendering still changes the received file where simulation needs it:

* **Mass properties.** No link has an inertial. Each link gets its visual meshes' convex hull
  at one density (600 kg/m^3), 11.6 kg in all; the CAD's mass properties replace them, and the
  estimate is likely light on the arm actuators. This is why it is still a development body.
* **Collision.** The received capsules on the four distal links of each arm and the torso
  box stay; every other link with geometry gets its visual hull as its collision mesh.
* The motor table is removed (the bindings carry it), visual names are made unique, meshes
  are referenced as ``meshes/<name>.stl``, and the ``world`` link and joint are removed.

Still open with the hardware owner: every arm joint declares 54 Nm and 0.5 rad/s, too slow to
replay human motion, and the received driver configuration caps speed at 0.25 rad/s; left and
right do not mirror one sign convention; the right arm's first id is 0x09 where the rest of
that arm runs 0x10..0x15; the motor table's gains (kp 20..61, kd 1..5) are that driver's, not
a qualification. The grippers and the SO-101 neck have no motor entry, so they stay
undocumented drives, and the ZED stays a frame, not a sensor.
"""

from typing import Final

from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import (
    BaseMount,
    EmbodimentDefinition,
    MountedOn,
    MountKind,
    RootMount,
    body_component,
)
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import (
    ActuatorBinding,
    ActuatorBus,
    ActuatorFeedback,
    ActuatorModel,
    CoordinateUnit,
    DirectDrive,
)
from ..mechanical import Build, Module, ModuleId
from ..parts import ChainSpec, GripperSpec, Limb
from ._authoring import bounded_layout

_RECEIVED: Final = AssetProvenance(
    repository="https://github.com/Sentient-X/sx-embodiments",
    revision="788bb9be9911747df56d2bebf6ff45d5da0461b0287e5a881718217735ceb080",
    path="assets/sentient_static_upper_body/source/received.urdf",
    license_id="LicenseRef-Sentient-Proprietary",
)

STATIC_UPPER_BODY_URDF: Final = packaged_asset(
    relpath="sentient_static_upper_body/sentient_static_upper_body.urdf",
    sha256="6c8d85891f0c85fa8a79d5b8a80505d2f4963e0239006525c65e4831ab278fe0",
    size_bytes=234759,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository=_RECEIVED.repository,
        revision=_RECEIVED.revision,
        path=_RECEIVED.path,
        license_id=_RECEIVED.license_id,
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
    ActuatorModel.DAMIAO_DM8009,
    ActuatorModel.DAMIAO_DM8009,
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


def _damiao(model: ActuatorModel, bus_id: int) -> DirectDrive:
    """One motor on the body's one Damiao CAN bus, identity drive map."""
    return DirectDrive(ActuatorBinding(model=model, bus=ActuatorBus.DAMIAO_CAN, bus_id=bus_id))


STATIC_UPPER_BODY_TORSO: Final = ChainSpec(
    part_id=PartId("sentient-static-upper-body-torso"),
    layout=bounded_layout(
        names=("torso_joint",),
        units=(CoordinateUnit.RADIAN,),
        lower=(-1.221293965056839,),
        upper=(1.1499580376897836,),
        actuations=(_damiao(ActuatorModel.DAMIAO_DM8009, 0x1F),),
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
                _damiao(model, bus_id)
                for model, bus_id in zip(_ARM_MODELS, _ARM_IDS[side], strict=True)
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
        -2.1280613412680243,
        -2.096017395284962,
    ),
    upper=(
        2.889868009460594,
        1.2018387121385512,
        0.8070115205615327,
        0.000953688868543523,
        1.8839169909208824,
        2.4408712901503016,
        2.0967803463797967,
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
        -2.308880750743878,
        -2.112039368276493,
    ),
    upper=(
        3.1225680933852136,
        0.35464911018127765,
        2.7597848477912574,
        2.966544594491493,
        1.4261463340199896,
        2.255855649652858,
        2.0986877241168838,
    ),
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
        lower=(
            -2.12930168743,
            -0.0349065850399,
            -3.40339204139,
            -3.28121899375,
            -3.00196631343,
            -0.106465084372,
        ),
        upper=(
            2.12930168743,
            3.63028484415,
            0.0872664625997,
            0.401425727959,
            2.86233997327,
            2.04203522483,
        ),
    ),
    home=(0.0,) * 6,
)


def _gripper(side: str, left_finger: float, right_finger: float) -> GripperSpec:
    """Two revolute fingers the description moves independently; no source says whether one
    motor couples them, so neither a mimic nor an aperture is declared."""
    return GripperSpec(
        part_id=PartId(f"sentient-static-upper-body-{side}-gripper"),
        layout=bounded_layout(
            names=(f"{side}_gripper_left_finger_joint", f"{side}_gripper_right_finger_joint"),
            units=(CoordinateUnit.RADIAN,) * 2,
            lower=(left_finger, 0.0),
            upper=(0.0, right_finger),
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
    ),
)

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
