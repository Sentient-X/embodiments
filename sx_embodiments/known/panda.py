"""Franka Panda family: the mobile panda_omron body and fixed-base Franka.

The canonical Menagerie MJCF supplies the portable description asset. RoboCasa/robosuite
scene names such as ``robot0_joint1`` remain consumer-local per the name boundary.

``panda_omron`` is the Panda mated on an Omron LD base as robosuite builds it
(``robosuite/models/assets/bases/omron_mobile_base.xml``): the base's collision box and its
support column, the arm's ``panda_link0`` on the column's top at the torso's lowest height.
"""

import dataclasses
import xml.etree.ElementTree as ET
from typing import Final

from ..assembly import Pose, Subtree
from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import (
    BaseMount,
    Component,
    EmbodimentDefinition,
    MountKind,
    RootMount,
    body_component,
)
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..mechanical import Box, Build, Cylinder, InterfaceId, Mate, Module, ModuleId, Placed, Socket
from ..parts import (
    ChainSpec,
    ControlRates,
    GripperSpec,
    Limb,
    MimicJoint,
    MobileBaseSpec,
    PhysicalSpec,
)
from ._authoring import bounded_layout
from .sources import menagerie

PANDA_MJCF: Final = packaged_asset(
    relpath="menagerie/franka_emika_panda/panda.xml",
    sha256="96ad67da03710f17f798c9478fd9e9efdf24a3bf8359f05e456dd9fb158ea273",
    size_bytes=14438,
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=menagerie("franka_emika_panda/panda.xml", "Apache-2.0"),
    media_type="application/xml",
)
PANDA_URDF: Final = packaged_asset(
    relpath="official/franka_panda/panda.urdf",
    sha256="668d8398e32164587fc2e9886b37d1a17a20d889cafe192f28ba245a3e82c24a",
    size_bytes=19046,
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

PANDA_ARM: Final = ChainSpec(
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

#: The Franka Hand, fitted on the arm in the description: everything below ``panda_hand_joint``.
PANDA_HAND_MODULE: Final = Module(
    module_id=ModuleId("franka-hand"),
    label="Franka Hand",
    description=Subtree(PANDA_URDF, below="panda_hand_joint"),
    plug=InterfaceId("iso-9409-1-50-4-m6"),
    components=(body_component("gripper", PANDA_GRIPPER, RootMount("panda_hand_joint_mount")),),
)

#: The flange's ISO pattern sits 45 degrees about the approach axis from ``panda_link8``.
_PANDA_FLANGE: Final = Pose(rpy=(0.0, 0.0, -0.7853981633974483))


def _panda(
    module_id: str, label: str, *components: Component, plug: InterfaceId | None = None
) -> Module:
    return Module(
        module_id=ModuleId(module_id),
        label=label,
        description=PANDA_URDF,
        plug=plug,
        components=components,
        sockets=(
            Socket(
                "flange",
                InterfaceId("iso-9409-1-50-4-m6"),
                "panda_link8",
                _PANDA_FLANGE,
                carrier="arm",
                fitted=PANDA_HAND_MODULE,
            ),
        ),
    )


OMRON_BASE: Final = MobileBaseSpec(part_id=PartId("omron-ld"))  # commanded outside joint space

#: The seat robosuite gives a robot on its Omron base: the support column's top.
OMRON_SUPPORT: Final = InterfaceId("robosuite-omron-support")


def _omron_ld_description() -> bytes:
    """The Omron LD base as robosuite models it, in metres from the floor under its origin:
    the chassis box (``pedestal_feet_col``, 0.70 x 0.50 x 0.38 m, its centre at
    (-0.20, 0, 0.192)) and the support column (radius 0.12 m, 0.40 m long, centred at
    (-0.05, 0, 0.50)); robosuite's box density, 10 kg/m3, gives the chassis its mass."""
    robot = ET.Element("robot", {"name": "omron_ld"})
    link = ET.SubElement(robot, "link", {"name": "omron_ld"})
    chassis, column = Box((0.70, 0.50, 0.38)), Cylinder(0.12, 0.40)
    mass = 10.0 * 0.70 * 0.50 * 0.38
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "-0.2 0 0.192", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": repr(mass)})
    ixx, iyy, izz = chassis.inertia(mass)
    ET.SubElement(
        inertial,
        "inertia",
        {"ixx": repr(ixx), "ixy": "0", "ixz": "0", "iyy": repr(iyy), "iyz": "0", "izz": repr(izz)},
    )
    for shape, xyz in ((chassis, "-0.2 0 0.192"), (column, "-0.05 0 0.5")):
        for tag in ("visual", "collision"):
            element = ET.SubElement(link, tag)
            ET.SubElement(element, "origin", {"xyz": xyz, "rpy": "0 0 0"})
            ET.SubElement(element, "geometry").append(shape.element())
    ET.indent(robot, space="  ")
    return ET.tostring(robot, encoding="utf-8", xml_declaration=True) + b"\n"


#: The Omron LD base: its chassis and column, and the drive commanded outside joint space.
OMRON_LD_MODULE: Final = Module(
    module_id=ModuleId("omron-ld"),
    label="Omron LD mobile base",
    description=_omron_ld_description(),
    components=(body_component("base", OMRON_BASE, RootMount("omron_ld")),),
    sockets=(
        # No joint of the base moves its column, so nothing in it carries the arm.
        Socket("support", OMRON_SUPPORT, "omron_ld", Pose((0.0, 0.0, 0.70))),
    ),
)

PANDA_OMRON_URDF: Final = packaged_asset(
    relpath="official/franka_panda/panda_omron.urdf",
    sha256="2a58617bb77711f382a78cad463805b71868de94d615192d122b9ae981feb010",
    size_bytes=17963,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/frankarobotics/franka_ros",
        revision=PANDA_URDF.provenance.revision,
        path=PANDA_URDF.provenance.path,
        license_id="Apache-2.0",
        # The Omron LD's geometry is robosuite's (MIT), read from
        # ARISE-Initiative/robosuite@5ce6643f3092639d08f7b0f90ed1c6a84f50552c
        # robosuite/models/assets/bases/omron_mobile_base.xml.
        generator="sx-embodiments/tools/compose_registered_urdfs.py",
    ),
    media_type="application/xml",
)

PANDA_OMRON_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("panda_omron"),
    label="Franka Panda on Omron LD mobile base",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="panda", variant="omron"),
    build=Build(
        robot="panda_omron",
        root="omron_footprint",
        placed=(Placed("omron", OMRON_LD_MODULE, namespaced=False),),
        mates=(
            Mate(
                "panda",
                _panda(
                    "franka-panda-omron",
                    "Franka Panda on an Omron LD base",
                    body_component(
                        "arm",
                        # On the base, the Panda's file is a source of the body's description.
                        dataclasses.replace(
                            PANDA_ARM,
                            assets=(
                                dataclasses.replace(PANDA_URDF, role=AssetRole.OTHER),
                                PANDA_MJCF,
                            ),
                        ),
                        RootMount("panda_link0"),
                        limb=Limb.ARM,
                    ),
                    plug=OMRON_SUPPORT,
                ),
                "omron",
                "support",
                namespaced=False,
            ),
        ),
    ),
    extra_assets=(PANDA_OMRON_URDF,),
    rates=ControlRates(policy_hz=20.0),
    # Omron LD-60 chassis footprint (datasheet 699 x 500 mm); it drives on the floor
    # and placement adds the facing constraint toward the work support.
    base_mount=BaseMount(
        kind=MountKind.MOBILE,
        frame="base",
        half_extents=(0.35, 0.25),
    ),
)

#: The fixed-base arm with its hand fitted.
PANDA_MODULE: Final = _panda(
    "franka-panda",
    "Franka Panda arm",
    body_component("arm", PANDA_ARM, RootMount("panda_link0"), limb=Limb.ARM),
)

FRANKA_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("franka"),
    label="Franka Panda (fixed base)",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="panda"),
    build=Build.of(PANDA_MODULE),
    # Footprint measured from the menagerie link0 collision geometry (xy AABB).
    base_mount=BaseMount(
        kind=MountKind.BOLT_DOWN,
        frame="link0",
        half_extents=(0.147, 0.099),
        centre=(-0.042, 0.001),
    ),
)
