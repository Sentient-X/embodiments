"""Sentient static upper body: a torso with two seven-joint arms, clamped to a table.

The robot sits on a table edge facing out from it, where it works a coffee machine: the base
is clamped to the table top, so the description's root ``base_link`` is the table surface
(the received file put it 0.797 m above a floor through a fixed ``world`` joint, which is
that table's height and a placement fact, not part of the body). A yaw torso carries two
arms of shoulder pitch, roll and yaw, elbow pitch, forearm yaw, wrist pitch and roll, each
ending in a two-finger gripper with a ``*_gripper_tcp`` frame, and an SO-101 arm on the chest
used as a six-joint neck whose last joint tilts a ZED camera (``ZED_camera`` frame).

The description is ``tools/compose_static_upper_body_urdf.py``'s rendering of the
description package as received on 2026-10-24 (maintainer ``suprajs``,
licence ``Proprietary``), kept byte for byte under ``source/``. The rendering changes the
received file where it cannot be right, and each change is replaced by evidence when it comes:

* **Elbows.** The received range ran from 80.5 deg past straight, the forearm swinging
  behind the body, to 90 deg of flexion. Its span is placed on the flexion side, from the
  straight arm (-pi/2 left, +pi/2 right, where shoulder-to-wrist distance peaks) to 157.6
  deg of flexion, 2.5 deg short of where the wrist meets the upper arm. A measured stop
  from the hardware replaces both ends.
* **Forearm yaw.** The received 8.7 rad (left) and 7.4 rad (right) ranges are cut to one
  turn about zero; they read as multi-turn encoder values.
* **Mass properties.** The received file has no inertial on any link. Each link gets its
  visual meshes' convex hull at one density (600 kg/m^3), 11.6 kg in all; the CAD's mass
  properties replace them, and the estimate is likely light on the arm actuators.
* **Collision.** The received capsules on the four distal links of each arm and the torso
  box stay; every other link with geometry gets its visual hull as its collision mesh.
* Visual names are made unique, meshes are referenced as ``meshes/<name>.stl``, and the
  ``world`` link and joint are removed.

Still as received, and questions for the hardware owner: the left upper-arm yaw spans
-5.12..0.81 rad while the right spans -2.42..2.76; every arm joint declares 54 Nm and
0.5 rad/s, too slow to replay human motion; and left and right do not mirror one sign
convention (the same shoulder-pitch value raises one arm and lowers the other, while upper-
arm yaw turns both the same way). No source names the motors, their bus ids or the ZED
model, so every axis is an undocumented drive and the camera stays a frame, not a sensor.
That makes this a development body.
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
from ..layout import CoordinateUnit
from ..mechanical import Build, Module, ModuleId
from ..parts import ChainSpec, GripperSpec, Limb
from ._authoring import bounded_layout

_RECEIVED: Final = AssetProvenance(
    repository="https://github.com/Sentient-X/sx-embodiments",
    revision="31de0542f8ffa16020e9fbd03081a4a97b4d08cc6f6bd83213c2d2ba724915ee",
    path="assets/sentient_static_upper_body/source/received.urdf",
    license_id="LicenseRef-Sentient-Proprietary",
)

STATIC_UPPER_BODY_URDF: Final = packaged_asset(
    relpath="sentient_static_upper_body/sentient_static_upper_body.urdf",
    sha256="54c53d3d3f8d13f9f4c21e6e19b18b125a6dea809ce6de320da5bc660a85aa49",
    size_bytes=234543,
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

STATIC_UPPER_BODY_TORSO: Final = ChainSpec(
    part_id=PartId("sentient-static-upper-body-torso"),
    layout=bounded_layout(
        names=("torso_joint",),
        units=(CoordinateUnit.RADIAN,),
        lower=(-1.22129396506,),
        upper=(1.14995803769,),
    ),
    home=(0.0,),
)

STATIC_UPPER_BODY_LEFT_ARM: Final = ChainSpec(
    part_id=PartId("sentient-static-upper-body-left-arm"),
    layout=bounded_layout(
        names=tuple(f"left_{name}_joint" for name in _ARM_JOINTS),
        units=(CoordinateUnit.RADIAN,) * 7,
        lower=(
            -2.64267185473,
            -0.387388418402,
            -5.12302586404,
            -1.57079632679,
            -3.14159265359,
            -2.12806134127,
            -2.09601739528,
        ),
        upper=(
            2.88986800946,
            1.20183871214,
            0.807011520562,
            1.17920367321,
            1.88391699092,
            2.44087129015,
            2.09678034638,
        ),
    ),
    home=(0.0,) * 7,
)

STATIC_UPPER_BODY_RIGHT_ARM: Final = ChainSpec(
    part_id=PartId("sentient-static-upper-body-right-arm"),
    layout=bounded_layout(
        names=tuple(f"right_{name}_joint" for name in _ARM_JOINTS),
        units=(CoordinateUnit.RADIAN,) * 7,
        lower=(
            -2.91390096895,
            -1.21010342679,
            -2.42103456168,
            -1.17920367321,
            -3.14159265359,
            -2.30888075074,
            -2.11203936828,
        ),
        upper=(
            3.12256809339,
            0.354649110181,
            2.75978484779,
            1.57079632679,
            1.42614633402,
            2.25585564965,
            2.09868772412,
        ),
    ),
    home=(0.0,) * 7,
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
