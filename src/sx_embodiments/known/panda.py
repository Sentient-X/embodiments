"""Franka Panda family: the mobile panda_omron body and fixed-base Franka.

The canonical Menagerie MJCF supplies the portable description asset. RoboCasa/robosuite
scene names such as ``robot0_joint1`` remain consumer-local per the name boundary.
"""

from typing import Final

from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import (
    BaseMount,
    EmbodimentDefinition,
    MountedOn,
    MountKind,
    body_component,
)
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import (
    ArmSpec,
    ControlRates,
    GripperSpec,
    MimicJoint,
    MobileBaseSpec,
    PhysicalSpec,
)
from ._authoring import bounded_layout
from .sources import menagerie

PANDA_MJCF: Final = packaged_asset(
    relpath="menagerie/franka_emika_panda/panda.xml",
    sha256="96ad67da03710f17f798c9478fd9e9efdf24a3bf8359f05e456dd9fb158ea273",
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=menagerie("franka_emika_panda/panda.xml", "Apache-2.0"),
    media_type="application/xml",
)
PANDA_URDF: Final = packaged_asset(
    relpath="official/franka_panda/panda.urdf",
    sha256="668d8398e32164587fc2e9886b37d1a17a20d889cafe192f28ba245a3e82c24a",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/frankarobotics/franka_ros",
        revision="ddd2fffd9de44b02ad15b4bbb2bfa2cec4d60d98",
        path="franka_description/robots/panda/panda.urdf.xacro",
        license_id="Apache-2.0",
        generator="xacro 2.1.1 hand=true",
    ),
    media_type="application/xml",
)

PANDA_ARM: Final = ArmSpec(
    part_id=PartId("panda-arm"),
    layout=bounded_layout(
        names=(
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ),
        units=(CoordinateUnit.RADIAN,) * 7,
        lower=(-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973),
        upper=(2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973),
    ),
    home=(0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785),
    # Work-ready: gripper straight down 0.26 m in front of the mount plane, 0.16 m above
    # it, elbow up — the configuration a benchtop task starts from (the classic Franka
    # ready holds the hand ~0.5 m up, staging rather than working height).
    ready=(0.0, -0.153, 0.0, -2.715, 0.0, 1.932, 0.785),
    assets=(PANDA_URDF, PANDA_MJCF),
    # Franka Emika Panda datasheet (download.franka.de/Datasheet-EN.pdf).
    physical=PhysicalSpec(payload_kg=3.0, reach_m=0.855, mass_kg=18.0),
)

PANDA_GRIPPER: Final = GripperSpec(
    part_id=PartId("panda-gripper"),
    layout=bounded_layout(
        names=("panda_finger_joint1",),
        units=(CoordinateUnit.METER,),
        lower=(0.0,),
        upper=(0.04,),
    ),
    travel_m=(0.0, 0.08),  # parallel jaw: aperture = 2 x finger stroke (0.04 m each)
    # Midpoint of the finger pads along the hand frame's approach (+z) axis: fingers mount
    # 0.0584 m from the hand origin and the pads centre 0.045 m further — the official
    # flange-to-TCP 0.1034 m (the flange and hand frames share the z axis).
    grasp_centre_m=(0.0, 0.0, 0.1034),
    mimic_joints=(MimicJoint("panda_finger_joint2", of="panda_finger_joint1", multiplier=1.0),),
)

OMRON_BASE: Final = MobileBaseSpec(part_id=PartId("omron-ld"))  # commanded outside joint space

PANDA_OMRON_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("panda_omron"),
    label="Franka Panda on Omron LD mobile base",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="panda", variant="omron"),
    attachments=(
        body_component("base", OMRON_BASE),
        body_component("arm", PANDA_ARM, MountedOn("base", "top_plate")),
        body_component("gripper", PANDA_GRIPPER, MountedOn("arm", "panda_link8")),
    ),
    rates=ControlRates(policy_hz=20.0),
    # Omron LD-60 chassis footprint (datasheet 699 x 500 mm); it drives on the floor
    # and placement adds the facing constraint toward the work support.
    base_mount=BaseMount(
        kind=MountKind.MOBILE,
        frame="base",
        half_extents=(0.35, 0.25),
    ),
)

FRANKA_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("franka"),
    label="Franka Panda (fixed base)",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="panda"),
    attachments=(
        body_component("arm", PANDA_ARM),
        body_component("gripper", PANDA_GRIPPER, MountedOn("arm", "panda_link8")),
    ),
    # Footprint measured from the menagerie link0 collision geometry (xy AABB).
    base_mount=BaseMount(
        kind=MountKind.BOLT_DOWN,
        frame="link0",
        half_extents=(0.147, 0.099),
        centre=(-0.042, 0.001),
    ),
)
