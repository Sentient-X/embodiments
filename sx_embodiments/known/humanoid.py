"""Sentient humanoid: the upper body built from parts, and the bodies its parts compose.

The robot in service is the **upper body**: a torso and two five-joint arms on one Damiao CAN
bus, with the vendor head on the chest. The full humanoid is that upper body plus two legs. Both
are assemblies of parts cut from one description, ``humanoid_pkg``'s hardware URDF
(``urdf/urdf_robot_acc.urdf`` at ``666bc6b5``), so the parts and the vendor's robot cannot
drift: :class:`~sx_embodiments.assembly.Subtree` cuts the upper body (the vendor tree without
the legs and the head), the head (below ``neck_joint``) and each leg (below its
``hip_pitch_joint``), and the assembly that rejoins them is the vendor's robot again, every
link where the source put it (``tests/test_humanoid.py`` holds that by forward kinematics).
An SO-101 neck is a composition, not a registered body, while neck motors are undecided:
``compose_embodiments`` mounts ``so101`` on the upper body's ``head_link``.

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

The hardware URDF's limits win at the execution boundary; the attached 28-joint MJCF is a
simulation description whose limits differ.
"""

from dataclasses import replace
from typing import Final, Literal

from ..assembly import Assembly, Member, Subtree
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
from ..parts import ChainSpec, Limb
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


def _part(name: str, part: Subtree) -> Member:
    """A cut part rejoining the vendor's tree where it was cut, under the vendor's names."""
    return Member(name, part, seat=part, namespaced=False)


_UPPER_MEMBERS: Final = (Member("upper_body", UPPER_BODY, namespaced=False), _part("head", HEAD))

SENTIENT_HUMANOID_UPPER_BODY_ASSEMBLY: Final = Assembly(
    robot="sentient_humanoid_upper_body", root="humanoid_origin", members=_UPPER_MEMBERS
)
SENTIENT_HUMANOID_ASSEMBLY: Final = Assembly(
    robot="sentient_humanoid",
    root="humanoid_origin",
    members=(
        *_UPPER_MEMBERS,
        *(_part(f"{side}_leg", leg) for side, leg in zip(("right", "left"), LEGS, strict=True)),
    ),
)
UPPER_BODY_COMPONENTS: Final = (
    body_component("torso", SENTIENT_HUMANOID_TORSO, RootMount("base_link"), limb=Limb.TORSO),
    body_component(
        "right_arm", SENTIENT_HUMANOID_RIGHT_ARM, MountedOn("torso", "chest_link"), limb=Limb.ARM
    ),
    body_component(
        "left_arm", SENTIENT_HUMANOID_LEFT_ARM, MountedOn("torso", "chest_link"), limb=Limb.ARM
    ),
)
LEG_COMPONENTS: Final = (
    body_component("right_leg", SENTIENT_HUMANOID_RIGHT_LEG, RootMount("base_link"), limb=Limb.LEG),
    body_component("left_leg", SENTIENT_HUMANOID_LEFT_LEG, RootMount("base_link"), limb=Limb.LEG),
)


SENTIENT_HUMANOID_UPPER_BODY_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("sentient-humanoid-upper-body"),
    label="Sentient humanoid upper body",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="sentient-humanoid", variant="upper-body"),
    attachments=UPPER_BODY_COMPONENTS,
    extra_assets=(
        SENTIENT_HUMANOID_SOURCE_URDF,
        _generated(
            "humanoid_pkg/urdf/sentient_humanoid_upper_body.urdf",
            "dd4e8833e516d8a49c117089614bf1fdab218ef259fa9e319984e96b747da0e5",
            23327,
        ),
    ),
    assembly=SENTIENT_HUMANOID_UPPER_BODY_ASSEMBLY,
)

SENTIENT_HUMANOID_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("sentient-humanoid"),
    label="Sentient humanoid",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="sentient-humanoid"),
    attachments=(*UPPER_BODY_COMPONENTS, *LEG_COMPONENTS),
    extra_assets=(
        SENTIENT_HUMANOID_SOURCE_URDF,
        _generated(
            "humanoid_pkg/urdf/sentient_humanoid.urdf",
            "d39263713f1e85e835ccbe4bbf13c3d0cbf41cf38f3f980817b51fe7c3387c52",
            40822,
        ),
        SENTIENT_HUMANOID_MJCF,
    ),
    assembly=SENTIENT_HUMANOID_ASSEMBLY,
)
