"""ALOHA 2 bimanual follower, pinned to the MuJoCo Menagerie model."""

from typing import Final, Literal

from ..assembly import Pose, Subtree
from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import EmbodimentDefinition, RootMount, body_component
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..mechanical import Build, InterfaceId, Module, ModuleId, Slot, Socket
from ..parts import ChainSpec, GripperSpec, Limb, MimicJoint
from ._authoring import bounded_layout
from .sources import menagerie

ALOHA_MJCF: Final = packaged_asset(
    relpath="menagerie/aloha/aloha.xml",
    sha256="68430b29719bda1b75e63f540953f81991bc3fd136bdf0a43bbe3e04393b78d3",
    size_bytes=19150,
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=menagerie("aloha/aloha.xml", "BSD-3-Clause"),
    media_type="application/xml",
)
ALOHA_URDF: Final = packaged_asset(
    relpath="official/aloha/aloha.urdf",
    sha256="e463b4f311e97e84a610cd072f74490d359638b64b1f6467b887f8a99a2f1c6c",
    size_bytes=35559,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Interbotix/interbotix_ros_manipulators",
        revision="0bb2b0e6d0e619bff02cf74dbd5af5681dcf80c9",
        path="interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf/vx300s.urdf.xacro",
        license_id="BSD-3-Clause",
        generator="xacro 2.1.1 + sx dual-arm composition v1",
    ),
    media_type="application/xml",
)

_ARM_LOWER: Final = (-3.14158, -1.85005, -1.76278, -3.14158, -1.8675, -3.14158)
_ARM_UPPER: Final = (3.14158, 1.25664, 1.6057, 3.14158, 2.23402, 3.14158)
_ARM_HOME: Final = (0.0, -0.96, 1.16, 0.0, -0.3, 0.0)


def _aloha_arm(side: Literal["left", "right"]) -> ChainSpec:
    return ChainSpec(
        part_id=PartId(f"aloha-{side}-arm"),
        layout=bounded_layout(
            names=tuple(
                f"{side}/{name}"
                for name in (
                    "waist",
                    "shoulder",
                    "elbow",
                    "forearm_roll",
                    "wrist_angle",
                    "wrist_rotate",
                )
            ),
            units=(CoordinateUnit.RADIAN,) * 6,
            lower=_ARM_LOWER,
            upper=_ARM_UPPER,
        ),
        home=_ARM_HOME,
        assets=(ALOHA_URDF, ALOHA_MJCF),
    )


def _aloha_gripper(side: Literal["left", "right"]) -> GripperSpec:
    driven = f"{side}/left_finger"
    return GripperSpec(
        part_id=PartId(f"aloha-{side}-gripper"),
        layout=bounded_layout(
            names=(driven,),
            units=(CoordinateUnit.METER,),
            lower=(0.0,),
            upper=(0.041,),
        ),
        travel_m=(0.0, 0.082),
        mimic_joints=(MimicJoint(f"{side}/right_finger", of=driven, multiplier=1.0),),
    )


ALOHA_LEFT_ARM: Final = _aloha_arm("left")
ALOHA_RIGHT_ARM: Final = _aloha_arm("right")
ALOHA_LEFT_GRIPPER: Final = _aloha_gripper("left")
ALOHA_RIGHT_GRIPPER: Final = _aloha_gripper("right")

_GRIPPER_SEAT: Final = InterfaceId("interbotix-xs-gripper")
_GRIPPERS: Final[tuple[tuple[Literal["left", "right"], GripperSpec], ...]] = (
    ("left", ALOHA_LEFT_GRIPPER),
    ("right", ALOHA_RIGHT_GRIPPER),
)


def _gripper_module(side: Literal["left", "right"], gripper: GripperSpec) -> Module:
    """The X-Series gripper as the description fits it: everything below ``<side>/ee_arm``."""
    return Module(
        module_id=ModuleId(f"aloha-{side}-gripper"),
        label=f"ALOHA {side} gripper",
        description=Subtree(ALOHA_URDF, below=f"{side}/ee_arm"),
        plug=_GRIPPER_SEAT,
        components=(body_component(f"{side}_gripper", gripper, RootMount(f"{side}/ee_arm_mount")),),
    )


#: Both arms as the description ships them, each wrist fitted with its gripper; the state order
#: is each arm followed by its gripper.
ALOHA_MODULE: Final = Module(
    module_id=ModuleId("aloha"),
    label="ALOHA 2 follower pair",
    description=ALOHA_URDF,
    components=(
        body_component("left_arm", ALOHA_LEFT_ARM, RootMount("left/base_link"), limb=Limb.ARM),
        Slot("left_wrist"),
        body_component("right_arm", ALOHA_RIGHT_ARM, RootMount("right/base_link"), limb=Limb.ARM),
        Slot("right_wrist"),
    ),
    sockets=tuple(
        Socket(
            f"{side}_wrist",
            _GRIPPER_SEAT,
            f"{side}/gripper_link",
            Pose((0.042825, 0.0, 0.0)),
            carrier=f"{side}_arm",
            fitted=_gripper_module(side, gripper),
        )
        for side, gripper in _GRIPPERS
    ),
)

ALOHA_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("aloha"),
    label="ALOHA 2",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="aloha", variant="2"),
    build=Build.of(ALOHA_MODULE),
)
