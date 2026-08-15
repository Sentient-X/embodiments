"""Seeed Studio reBot B601-DM: the follower arm, the bimanual pair, and the teleop station.

The authoritative description is the upstream SolidWorks export from
``Seeed-Projects/reBotArmController_ROS2`` at commit
``a61efe4fa223ca50cd721ef8ebe4a60e90f28bfd``, vendored under ``assets/b601_dm/`` with its
complete ten-STL mesh closure (``base_link``, ``link1``..``link6``, ``gripper_link``,
``gripper_left``, ``gripper_right``). Two patches were applied when vendoring, and
``AssetProvenance.generator`` declares both so a reader can reproduce this file from
upstream's:

1. the twenty ``package://rebotarm_bringup/description/meshes_b601_gripper/<name>.STL``
   mesh references were rewritten to this repo's relative ``meshes/<name>.STL`` convention
   (the ``assets/so101`` layout), so the closure resolves from the asset's own directory;
2. ``<mimic joint="gripper_joint1" multiplier="1.0"/>`` was added to ``gripper_joint2`` —
   the coupling upstream's SolidWorks export omitted (see the gripper paragraph below).

Nothing else was touched — the untouched upstream file hashes to
``f808f6f0d33274b226c7e59db6d27d1498feb7e0955a94fe30f3f215a7425fa6``, so the digest pinned
below is the *vendored* file's while ``AssetProvenance.path``/``revision`` cite the
upstream original. Upstream ships **no** standalone LICENSE or COPYING file;
``Apache-2.0`` is declared by each ``src/*/package.xml`` ``<license>`` element, and
``rebotarm_bringup/package.xml`` is vendored beside the URDF as that evidence (the
``assets/humanoid_pkg`` and ``assets/yubi_description`` precedent).

Joint names, units, and limits are **the URDF's**, following the Piper precedent
(``known/piper.py`` + ``known/stations.py``): the description asset names the channels, and
``tests/test_urdf_parity.py`` pins the spec against it joint for joint. The deployed
LeRobot driver
(``lerobot.robots.rebot_b601_follower.config_rebot_b601_follower.RebotB601FollowerConfig``)
uses a second vocabulary for the same seven motors, and its soft box disagrees with the
description. Both are recorded here so a reader can see both numbers and know which
governs what:

===== =============== ======================== ============== ==================
index URDF joint      URDF limit (rad)         driver motor   driver soft (deg)
===== =============== ======================== ============== ==================
0     ``joint1``      -2.8 .. 2.8              shoulder_pan   -150 .. 150
1     ``joint2``      -3.14 .. 0               shoulder_lift  -200 .. 1
2     ``joint3``      -3.14 .. 0               elbow_flex     -200 .. 1
3     ``joint4``      -1.87 .. 1.57            wrist_flex     -80 .. 90
4     ``joint5``      -1.57 .. 1.57            wrist_yaw      -90 .. 90
5     ``joint6``      -3.14 .. 3.14            wrist_roll     -90 .. 90
6     ``gripper_j1``  0 .. 0.0715 (meters)     gripper        -270 .. 0
===== =============== ======================== ============== ==================

The index column is load-bearing and holds under either vocabulary: ``motor_can_ids``
orders the motors ``shoulder_pan`` (0x01) .. ``wrist_roll`` (0x06), ``gripper`` (0x07),
which is the kinematic order ``joint1``..``joint6`` then the jaw, so a coordinate index IS
a motor index. Per-joint divergence, stated explicitly rather than averaged away:

* ``joint1``: the driver clips **tighter** (±150 deg vs the described ±2.8 rad =
  ±160.43 deg) — runtime safety margin inside the mechanical stop.
* ``joint2``, ``joint3``: the driver clips **looser** — it admits -200..1 deg where the
  description's hard stop is -3.14..0 rad (-179.91..0 deg), i.e. ~20 deg beyond the
  described range at the lower end and 1 deg past it at the upper. This is an upstream
  discrepancy that the available evidence cannot resolve, so it is NOT silently merged:
  the registry states the described mechanical stop, and any consumer deriving live
  safety limits must take the conservative intersection itself.
* ``joint4``: the driver clips tighter at the lower end (-80 deg vs -1.87 rad =
  -107.14 deg); the upper ends agree to the URDF's rounding (1.57 rad = 89.95 deg).
* ``joint5``: the two agree to that same rounding (±1.57 rad vs ±90 deg).
* ``joint6``: the driver clips **tighter** (±90 deg vs the described ±3.14 rad).
* the gripper is not comparable: see below.

``home_joints`` are zeros because ``RebotB601Follower.calibrate()`` calls
``motor.set_zero_position()`` at the manually posed rest pose — zero *is* the calibrated
home, and the driver's box is expressed against it. Zero lies within every described limit
(``joint2``/``joint3`` reach it exactly: their described upper stop *is* 0).

The gripper channel is the described actuated finger travel: ``gripper_joint1`` is a
prismatic joint with a 0..0.0715 m stroke, and ``gripper_joint2`` is its mirror (identical
stroke, opposite mounting yaw), so ``travel_m`` is the described aperture
``2 x 0.0715 = 0.143`` m — the same derivation as ``PIPER_GRIPPER``. Physically ONE Damiao
motor (CAN ``0x07``) drives both fingers, but upstream's SolidWorks export declares them as
two independent prismatic joints, so the vendored description was patched to declare the
coupling (patch 2 above) and ``tests/test_urdf_parity.py`` now enforces ``mimic_joints``
against that declaration instead of asserting a gap. The multiplier is ``+1.0`` by the
vendored geometry: both joints take ``axis="1 0 0"`` in their own frame, and those frames
sit on the shared ``gripper_link`` yawed ``-1.5708`` and ``+1.5708``, so in the parent
frame the fingers travel along ``(0, -1, 0)`` and ``(0, +1, 0)`` — equal joint values
separate them symmetrically, from touching at ``0`` to the ``0.143`` m aperture at
``0.0715``. The sign is per-robot, not a convention: ``PIPER_GRIPPER``'s
``joint7``/``joint8`` axes resolve to the *same* parent direction, so its mirror is
``-1.0``.

Deliberate divergence, written down per the abstractions rule, and NOT closed by that
patch: the deployed driver's ``gripper`` channel is a Damiao 4310 **motor angle**
(-270..0 deg, ``send_force_pos``/``send_mit`` in radians), and the upstream description
publishes no motor-to-finger transmission, so the two are different quantities with no
verified conversion. A consumer mapping a LeRobot B601 recording onto this channel must
supply that conversion; it is capture-side data, not a registry fact. ``gap_curve``
therefore stays ``None`` (the finger travel is linear in the joint by construction; it is
the *motor* relation that is unknown). No ``<transmission>`` or ``mechanicalReduction`` was
added to the description: the ratio is unknown, and fabricating one would corrupt a file
sim and IK trust.

The bimanual flat convention ``[left joint1..joint6 | left gripper 6 | right joint1..joint6
| right gripper 13]`` is exactly the declared attachment order, and matches
``BiRebotB601Follower``'s feature namespacing (``left_``-prefixed motors in
``motor_can_ids`` order, then ``right_``-prefixed).

Camera instance names are the pod agent's dataset keys from
``data-factory/pod-agent/teleop_camera_ports.py`` (``left_wrist_camera``,
``right_wrist_camera``, ``top_camera``), the same file that pins each role's expected
product: two D405 wrist units and one D435i overhead. Beware when mapping these onto a
recording: ``BiRebotB601Follower`` re-prefixes per-arm camera keys, so the two wrist
streams land in the LeRobot dataset as ``left_left_wrist_camera`` and
``right_right_wrist_camera`` while the top-level ``top_camera`` keeps its name. The
station's canonical stream names are the ones declared here; that doubling is a
consumer-side mapping, not a second vocabulary.

Further deliberate divergences:

* The overhead unit is a **D435i**; the closed ``SensorModel`` vocabulary carries
  ``REALSENSE_D435`` (the same imager pair; the "i" adds an IMU this registry does not
  model). ``SensorModel`` is already projected into another repo's generated types, so a
  new member is a cross-repo wire change and waits for a consumer that needs the IMU.
* ``data-factory/pod-agent/teleop_runner.py`` captures all three cameras RGB-only at
  640x480 and at the session's ``fps`` (default 15, 1..60). That is capture configuration
  travelling with the recording, not a hardware fact, so the specs keep the products'
  nominal facts and ``rates`` stays unbound.
* The leader is a :class:`DeviceSpec`, as on ``piperx-station``. The reBot Arm 102 leader
  has a citable 7-servo joint box (``RebotArm102LeaderConfig.joint_ranges``, deliberately
  equal to the follower's driver box so leader actions drive the follower key-for-key) but
  no captured kinematic description; a ``LEADER`` component contributes zero state
  channels, so nothing is lost by naming it honestly.
"""

import dataclasses
from typing import Final

from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import (
    Component,
    EmbodimentDefinition,
    MountedOn,
    RootMount,
    body_component,
    leader_component,
)
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import (
    ArmSpec,
    DeviceSpec,
    GripperSpec,
    MimicJoint,
)
from ._authoring import bounded_layout

B601_DM_URDF: Final = packaged_asset(
    relpath="b601_dm/reBot_B601_DM_with_gripper.urdf",
    sha256="eb15a091412fa112f11d8bef3d170a40aa3cbb9db335fb145a1b27eb2aa000a0",
    size_bytes=13428,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Seeed-Projects/reBotArmController_ROS2",
        revision="a61efe4fa223ca50cd721ef8ebe4a60e90f28bfd",
        path="src/rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf",
        license_id="Apache-2.0",  # declared by src/*/package.xml; upstream ships no LICENSE
        # Both patches, so the vendored bytes are reproducible from upstream's:
        generator="sx-embodiments vendoring: (1) the 20 package://rebotarm_bringup/"
        "description/meshes_b601_gripper/<name>.STL mesh refs rewritten to meshes/"
        '<name>.STL; (2) <mimic joint="gripper_joint1" multiplier="1.0"/> added to '
        "gripper_joint2 (one Damiao motor at CAN 0x07 drives both fingers; the upstream "
        "SolidWorks export omitted the coupling)",
    ),
    media_type="application/xml",
)
B601_DM_SOURCE_URDF: Final = dataclasses.replace(B601_DM_URDF, role=AssetRole.OTHER)
BIMANUAL_B601_DM_URDF: Final = packaged_asset(
    relpath="b601_dm/bimanual_B601_DM.urdf",
    sha256="21d4f8b3849783e8ca42757298ff4239de68ceceba37d6947e23471aa81b0b84",
    size_bytes=21997,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Seeed-Projects/reBotArmController_ROS2",
        revision="a61efe4fa223ca50cd721ef8ebe4a60e90f28bfd",
        path="src/rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf",
        license_id="Apache-2.0",
        generator="sx-embodiments/tools/compose_registered_urdfs.py",
    ),
    media_type="application/xml",
)
B601_DM_STATION_URDF: Final = packaged_asset(
    relpath="b601_dm/B601_DM_station.urdf",
    sha256="dc55378246f27b5a309089968482b00c281c6185487cc7b37e9a53bb24763d65",
    size_bytes=23455,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Seeed-Projects/reBotArmController_ROS2",
        revision="a61efe4fa223ca50cd721ef8ebe4a60e90f28bfd",
        path="src/rebotarm_bringup/description/urdf/reBot_B601_DM_with_gripper.urdf",
        license_id="Apache-2.0",
        generator="sx-embodiments/tools/compose_registered_urdfs.py",
    ),
    media_type="application/xml",
)

B601_ARM: Final = ArmSpec(
    part_id=PartId("rebot-b601dm-arm"),
    # Names and limits are the vendored URDF's revolute joints, in kinematic order.
    layout=bounded_layout(
        names=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"),
        units=(CoordinateUnit.RADIAN,) * 6,
        lower=(-2.8, -3.14, -3.14, -1.87, -1.57, -3.14),
        upper=(2.8, 0.0, 0.0, 1.57, 1.57, 3.14),
    ),
    home=(0.0,) * 6,  # the driver's calibrated zero pose, inside every URDF limit
    # physical: no manufacturer datasheet captured, so payload/reach/mass stay unstated
)

B601_GRIPPER: Final = GripperSpec(
    part_id=PartId("rebot-b601dm-gripper"),
    layout=bounded_layout(
        names=("gripper_joint1",),  # the described actuated finger; the mirror mimics
        units=(CoordinateUnit.METER,),
        lower=(0.0,),
        upper=(0.0715,),
    ),
    travel_m=(0.0, 0.143),  # parallel jaw: aperture = 2 x finger stroke (0.0715 m each)
    mimic_joints=(MimicJoint("gripper_joint2", of="gripper_joint1", multiplier=1.0),),
)

B601_LEADER: Final = DeviceSpec(
    part_id=PartId("rebot-arm-102-leader"),
    description="Seeed Studio reBot Arm 102 leader, 7 FashionStar UART smart servos "
    "(shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_yaw, wrist_roll, "
    "gripper); kinematic description not captured",
)


def b601_side(side: str) -> tuple[Component, ...]:
    """One B601-DM follower arm + gripper block; two sides ARE the bimanual body."""
    return (
        body_component(f"{side}_arm", B601_ARM, RootMount(f"{side}_base_link")),
        body_component(
            f"{side}_gripper",
            B601_GRIPPER,
            MountedOn(f"{side}_arm", f"{side}_link6"),
        ),
    )


B601_DM_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("b601-dm"),
    label="Seeed Studio reBot B601-DM (single arm)",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="rebot-b601", variant="dm"),
    attachments=(
        body_component("arm", B601_ARM, RootMount("base_link")),
        body_component("gripper", B601_GRIPPER, MountedOn("arm", "link6")),
    ),
    extra_assets=(B601_DM_URDF,),
    # rates unbound: the pod's capture fps is a per-session parameter, not a hardware rate
)

BIMANUAL_B601_DM_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("bimanual-b601-dm"),
    label="Bimanual Seeed Studio reBot B601-DM",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="rebot-b601", variant="dm-bimanual"),
    attachments=(*b601_side("left"), *b601_side("right")),
    extra_assets=(B601_DM_SOURCE_URDF, BIMANUAL_B601_DM_URDF),
)

B601_DM_STATION_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("b601-dm-station"),
    label="Seeed Studio reBot B601-DM bimanual teleop station",
    kind=EmbodimentKind.TELEOP_STATION,
    lineage=Lineage(family="rebot-b601", variant="dm-station"),
    attachments=(
        leader_component("left_leader", B601_LEADER, RootMount("left_leader")),
        leader_component("right_leader", B601_LEADER, RootMount("right_leader")),
        *b601_side("left"),
        *b601_side("right"),
    ),
    extra_assets=(B601_DM_SOURCE_URDF, B601_DM_STATION_URDF),
)
