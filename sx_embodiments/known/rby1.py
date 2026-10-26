"""Rainbow Robotics RBY1-M v1.3 with grippers."""

from typing import Final, Literal

from sx_contracts import ComponentId

from ..assembly import Pose, Subtree
from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import EmbodimentDefinition, MountedOn, RootMount, body_component
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..mechanical import Build, InterfaceId, Module, ModuleId, Socket
from ..parts import ChainSpec, GripperSpec, Limb, MimicJoint, MobileBaseSpec
from ..sites import BodySite
from ._authoring import bounded_layout, unbounded_layout
from .sources import menagerie

RBY1_MJCF: Final = packaged_asset(
    relpath="menagerie/rainbow_robotics_rby1/rby1m_1.3.xml",
    sha256="fa6d736f76e27de5aba22b96e0c98655ae6a0ad36f6ece52a915bf71b460c66c",
    size_bytes=34775,
    format=AssetFormat.MJCF,
    role=AssetRole.DESCRIPTION,
    provenance=menagerie("rainbow_robotics_rby1/rby1m_1.3.xml", "Apache-2.0"),
    media_type="application/xml",
)
RBY1_URDF: Final = packaged_asset(
    relpath="official/rby1/rby1m_v1.3.urdf",
    sha256="66f4ad14779793e94df87b59356321190c9882766b2ed9cbeba6624c0880f11d",
    size_bytes=38603,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/RainbowRobotics/rby1-sdk",
        revision="38df3267e617d22644f6686e8a7e3c4eac3ce2ee",
        path="models/rby1m/urdf/model_v1.3.urdf",
        license_id="Apache-2.0",
    ),
    media_type="application/xml",
)

RBY1_BASE: Final = MobileBaseSpec(
    part_id=PartId("rby1m-base"),
    layout=unbounded_layout(
        names=("wheel_fr", "wheel_fl", "wheel_rr", "wheel_rl"),
        units=(CoordinateUnit.RADIAN,) * 4,
    ),
    assets=(RBY1_URDF, RBY1_MJCF),
)

RBY1_TORSO: Final = ChainSpec(
    part_id=PartId("rby1-torso"),
    layout=bounded_layout(
        names=tuple(f"torso_{index}" for index in range(6)),
        units=(CoordinateUnit.RADIAN,) * 6,
        lower=(-0.261799388, -0.523598776, -2.617993878, -0.785398163, -0.523598776, -2.35619449),
        upper=(0.261799388, 1.570796327, 1.570796327, 1.570796327, 0.523598776, 2.35619449),
    ),
    home=(0.0,) * 6,
)


def _rby1_arm(side: Literal["left", "right"]) -> ChainSpec:
    # Physical ranges are the pinned official URDF's, not the alternate MJCF's.
    second = (-0.017453293, 3.141592654) if side == "left" else (-3.141592654, 0.017453293)
    return ChainSpec(
        part_id=PartId(f"rby1-{side}-arm"),
        layout=bounded_layout(
            names=tuple(f"{side}_arm_{index}" for index in range(7)),
            units=(CoordinateUnit.RADIAN,) * 7,
            lower=(
                -3.141592654,
                second[0],
                -3.141592654,
                -2.617993878,
                -3.141592654,
                -0.872664626,
                -1.5707963268,
            ),
            upper=(
                3.141592654,
                second[1],
                3.141592654,
                0.017453293,
                3.141592654,
                0.872664626,
                1.5707963268,
            ),
        ),
        home=(0.0,) * 7,
    )


def _rby1_gripper(side: Literal["left", "right"]) -> GripperSpec:
    suffix = "l" if side == "left" else "r"
    driven = f"gripper_finger_{suffix}1"
    return GripperSpec(
        part_id=PartId(f"rby1-{side}-gripper"),
        layout=bounded_layout(
            names=(driven,),
            units=(CoordinateUnit.METER,),
            lower=(0.0,),
            upper=(0.05,),
        ),
        # Authoritative URDF: both axes point +Y in the common ee frame, but the
        # opposed fingers have positive and negative ranges. Symmetric jaws use -q.
        # The alternate Menagerie MJCF uses a different first-finger axis and +q.
        mimic_joints=(MimicJoint(f"gripper_finger_{suffix}2", of=driven, multiplier=-1.0),),
    )


RBY1_RIGHT_ARM: Final = _rby1_arm("right")
RBY1_LEFT_ARM: Final = _rby1_arm("left")
RBY1_HEAD: Final = ChainSpec(
    part_id=PartId("rby1-head"),
    layout=bounded_layout(
        names=("head_0", "head_1"),
        units=(CoordinateUnit.RADIAN,) * 2,
        lower=(-1.57, -1.57),
        upper=(1.57, 1.57),
    ),
    home=(0.0, 0.0),
)
RBY1_RIGHT_GRIPPER: Final = _rby1_gripper("right")
RBY1_LEFT_GRIPPER: Final = _rby1_gripper("left")

_FLANGE: Final = InterfaceId("rainbow-rby1-flange")
_GRIPPERS: Final[tuple[tuple[Literal["left", "right"], GripperSpec], ...]] = (
    ("right", RBY1_RIGHT_GRIPPER),
    ("left", RBY1_LEFT_GRIPPER),
)


def _gripper_module(side: Literal["left", "right"], gripper: GripperSpec) -> Module:
    """A gripper as the description fits it: everything below the arm's ``END`` joint."""
    end = f"END_{side}"
    return Module(
        module_id=ModuleId(f"rby1-{side}-gripper"),
        label=f"RB-Y1 {side} gripper",
        description=Subtree(RBY1_URDF, below=end),
        plug=_FLANGE,
        components=(body_component(f"{side}_gripper", gripper, RootMount(f"{end}_mount")),),
    )


#: The whole robot as the description ships it, each wrist fitted with its gripper.
RBY1_MODULE: Final = Module(
    module_id=ModuleId("rby1"),
    label="Rainbow Robotics RB-Y1",
    description=RBY1_URDF,
    # Named vendor link origins, not inferred joint indices or TCP/tool calibrations.
    sites=(
        BodySite("left_wrist", ComponentId("left_arm"), "link_left_arm_6"),
        BodySite("right_wrist", ComponentId("right_arm"), "link_right_arm_6"),
        BodySite("left_elbow", ComponentId("left_arm"), "link_left_arm_3"),
        BodySite("right_elbow", ComponentId("right_arm"), "link_right_arm_3"),
        BodySite("chest", ComponentId("torso"), "link_torso_5"),
        BodySite("base", ComponentId("base"), "base"),
    ),
    components=(
        body_component("base", RBY1_BASE, RootMount("base")),
        body_component("torso", RBY1_TORSO, MountedOn("base", "base"), limb=Limb.TORSO),
        body_component(
            "right_arm", RBY1_RIGHT_ARM, MountedOn("torso", "link_torso_5"), limb=Limb.ARM
        ),
        body_component(
            "left_arm", RBY1_LEFT_ARM, MountedOn("torso", "link_torso_5"), limb=Limb.ARM
        ),
        body_component("head", RBY1_HEAD, MountedOn("torso", "link_head_0"), limb=Limb.HEAD),
    ),
    sockets=tuple(
        Socket(
            f"{side}_flange",
            _FLANGE,
            f"link_{side}_arm_6",
            Pose((0.0, 0.0, -0.125)),
            carrier=f"{side}_arm",
            fitted=_gripper_module(side, gripper),
        )
        for side, gripper in _GRIPPERS
    ),
)

RBY1_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("rby1"),
    label="Rainbow Robotics RBY1-M v1.3",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="rby1", variant="m", revision="1.3"),
    build=Build.of(RBY1_MODULE),
)
