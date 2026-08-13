"""ROBOTIS AI Worker FFW-BG2 (rev4 follower), pinned to the vendored `ai_worker` URDF.

The flagship of ROBOTIS's FFW family: a mobile bimanual manipulator sold as a
physical-AI data-collection platform (leader-follower teleop, LeRobot interchange).
This entry admits the follower's morphology only; the ROS 2 runtime, the LG2 leader
rig, and the Dynamixel driver are deliberately not imported (see
`plans/fuse-ai-worker-2026-08.md` in glued for the adjudication and follow-up
triggers).

State order is the vendored URDF's joint declaration order — lift, head, left arm,
right arm, then the gripper drive joints — which component topology can express
exactly. Upstream's ros2_control xacro declares a second order (right arm first);
neither is more native, and the URDF is the artifact this registry vendors as
authoritative.

The rev4 follower is stationary: its URDF roots at ``world -> world_fixed -> base_link``,
and the swerve mobility in the family belongs to the SG2/F2/SH5 variants (each a distinct
revision with its own steer/drive joints). Each RH-P12-RN(A) gripper is one revolute
drive joint with three
x1.0 mimics; no aperture curve is published at the pin, so `travel_m` stays unset and
aperture kinematics fail closed rather than guess. The head ZED-Mini and the two
wrist RealSense D405s have mount frames in the URDF but no pinnable intrinsics, so
the embodiment stays development-tier (`MISSING_CAMERA_CALIBRATION`) with no camera
bindings yet.
"""

from typing import Final, Literal

from ..assets import AssetFormat, AssetRole, packaged_asset
from ..compose import EmbodimentDefinition, MountedOn, body_component
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import ArmSpec, GripperSpec, JointGroupSpec, MimicJoint
from ._authoring import bounded_layout
from .sources import ai_worker

FFW_BG2_URDF: Final = packaged_asset(
    relpath="ai_worker/ffw_bg2_rev4/ffw_bg2_follower.urdf",
    sha256="86a5d17c23a2fde7e1c2c65ae499386fec4168f007a9526a858106e93d1dd056",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=ai_worker(
        "ffw_description/urdf/ffw_bg2_rev4_follower/ffw_bg2_follower.urdf",
        generator=(
            "mesh URIs rewritten package://ffw_description/meshes/... -> meshes/... and the "
            "leaked-container D405 path (file:///root/ros2_ws/...) -> meshes/realsense_d405/; "
            "upstream file sha256 d811530c3635dc2a1e8515db81edcea58b92872e84387987ef3a6024b2f27355"
        ),
    ),
    media_type="application/xml",
)

# Left/right arm limits mirror on joints 2 and 7 (URDF `<limit>`; joint 4 is shared).
_ARM_L_LOWER: Final = (-3.14, 0.0, -3.14, -2.9361, -3.14, -1.57, -1.8201)
_ARM_L_UPPER: Final = (3.14, 3.14, 3.14, 1.0786, 3.14, 1.57, 1.5804)
_ARM_R_LOWER: Final = (-3.14, -3.14, -3.14, -2.9361, -3.14, -1.57, -1.5804)
_ARM_R_UPPER: Final = (3.14, 0.0, 3.14, 1.0786, 3.14, 1.57, 1.8201)
_ARM_HOME: Final = (0.0,) * 7  # ffw_bringup initial_positions.yaml: all-zero home


def _ffw_arm(side: Literal["l", "r"]) -> ArmSpec:
    return ArmSpec(
        part_id=PartId(f"ffw-bg2-{side}-arm"),
        layout=bounded_layout(
            names=tuple(f"arm_{side}_joint{i}" for i in range(1, 8)),
            units=(CoordinateUnit.RADIAN,) * 7,
            lower=_ARM_L_LOWER if side == "l" else _ARM_R_LOWER,
            upper=_ARM_L_UPPER if side == "l" else _ARM_R_UPPER,
        ),
        home=_ARM_HOME,
        assets=(FFW_BG2_URDF,),
    )


def _ffw_gripper(side: Literal["l", "r"]) -> GripperSpec:
    driven = f"gripper_{side}_joint1"
    return GripperSpec(
        part_id=PartId(f"ffw-bg2-{side}-gripper"),
        layout=bounded_layout(
            names=(driven,),
            units=(CoordinateUnit.RADIAN,),
            lower=(0.0,),
            upper=(1.1,),
        ),
        mimic_joints=tuple(
            MimicJoint(f"gripper_{side}_joint{i}", of=driven, multiplier=1.0) for i in (2, 3, 4)
        ),
    )


FFW_BG2_TORSO: Final = JointGroupSpec(
    part_id=PartId("ffw-bg2-torso"),
    layout=bounded_layout(
        names=("lift_joint",),
        units=(CoordinateUnit.METER,),
        lower=(-0.5,),
        upper=(0.0,),
    ),
    home=(0.0,),
)

FFW_BG2_HEAD: Final = JointGroupSpec(
    part_id=PartId("ffw-bg2-head"),
    layout=bounded_layout(
        names=("head_joint1", "head_joint2"),
        units=(CoordinateUnit.RADIAN,) * 2,
        lower=(-0.2317, -0.35),
        upper=(0.6951, 0.35),
    ),
    home=(0.0, 0.0),
)

FFW_BG2_LEFT_ARM: Final = _ffw_arm("l")
FFW_BG2_RIGHT_ARM: Final = _ffw_arm("r")
FFW_BG2_LEFT_GRIPPER: Final = _ffw_gripper("l")
FFW_BG2_RIGHT_GRIPPER: Final = _ffw_gripper("r")

FFW_BG2_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("ffw-bg2"),
    label="ROBOTIS AI Worker FFW-BG2 (rev4)",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="ffw", variant="bg2-rev4"),
    attachments=(
        body_component("torso", FFW_BG2_TORSO),
        body_component("head", FFW_BG2_HEAD, MountedOn("torso", "arm_base_link")),
        body_component("left_arm", FFW_BG2_LEFT_ARM, MountedOn("torso", "arm_base_link")),
        body_component("right_arm", FFW_BG2_RIGHT_ARM, MountedOn("torso", "arm_base_link")),
        body_component("left_gripper", FFW_BG2_LEFT_GRIPPER, MountedOn("left_arm", "arm_l_link7")),
        body_component(
            "right_gripper", FFW_BG2_RIGHT_GRIPPER, MountedOn("right_arm", "arm_r_link7")
        ),
    ),
)
