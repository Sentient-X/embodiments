"""SO-101 family: the 5-DOF hobby arm and the bimanual pair supervisors runs.

Joint names and limits mirror ``assets/so101/so101.urdf`` (pinned by
``tests/test_urdf_parity.py``). The bimanual flat convention ``[left 0..5 | right 0..5]``
with the jaw at index 5 of each block is exactly the declared attachment order — the
supervisors ``ChannelLayout(arms=2, block=6, gripper_index=5)`` falls out of
``embodiments[...].state.coordinates``.
"""

import dataclasses
from typing import Final

from ..assembly import INSPECTION_NOTE, Pose
from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import EmbodimentDefinition, MountedOn, RootMount, body_component
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import ActuatorBinding, ActuatorBus, ActuatorFeedback, ActuatorModel, CoordinateUnit
from ..mechanical import Build, InterfaceId, Module, ModuleId, Placed
from ..parts import ChainSpec, GripperSpec, Limb
from ._authoring import bounded_layout


def _sts3215(bus_id: int) -> ActuatorBinding:
    """One calibrated STS3215 address on the arm's Feetech daisy chain.

    Both bimanual sides reuse the same per-chain addresses: each side is its own
    serial adapter, so the chain — not the body — scopes the id space.
    """

    return ActuatorBinding(
        model=ActuatorModel.FEETECH_STS3215,
        bus=ActuatorBus.FEETECH_SERIAL,
        bus_id=bus_id,
    )


SO101_URDF: Final = packaged_asset(
    relpath="so101/so101.urdf",
    sha256="dd7f789c1aa4b9f82174dd49f6c4d62f5338f0956ec8e59c37576ee161903279",
    size_bytes=16065,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/AbdulazizAlmuzairee/Squint",
        revision="d8ca2fbfb4cef6b6097c71f9ec172c76125a572f",
        path="envs/robot/so101.urdf",
        license_id="MIT",
    ),
    media_type="application/xml",
)
SO101_SOURCE_URDF: Final = dataclasses.replace(SO101_URDF, role=AssetRole.OTHER)
BIMANUAL_SO101_URDF: Final = packaged_asset(
    relpath="so101/bimanual_so101.urdf",
    sha256="df87bae56067a63a5401868c9fd8a97817051e4a2026b14b16085d9bd214c123",
    size_bytes=30715,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/AbdulazizAlmuzairee/Squint",
        revision="d8ca2fbfb4cef6b6097c71f9ec172c76125a572f",
        path="envs/robot/so101.urdf",
        license_id="MIT",
        generator="sx-embodiments/tools/compose_registered_urdfs.py",
    ),
    media_type="application/xml",
)

SO101_ARM: Final = ChainSpec(
    part_id=PartId("so101-arm"),
    layout=bounded_layout(
        names=("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"),
        units=(CoordinateUnit.RADIAN,) * 5,
        lower=(-1.91986, -1.74533, -1.69, -1.65806, -2.74385),
        upper=(1.91986, 1.74533, 1.69, 1.65806, 2.84121),
        actuators=tuple(_sts3215(bus_id) for bus_id in (1, 2, 3, 4, 5)),
        observations=(ActuatorFeedback(),) * 5,
    ),
    home=(0.0, 0.0, 0.0, 0.0, 0.0),
)

SO101_JAW: Final = GripperSpec(
    part_id=PartId("so101-jaw"),
    layout=bounded_layout(
        names=("gripper",),
        units=(CoordinateUnit.RADIAN,),
        lower=(-0.174533,),
        upper=(2.0944,),
        actuators=(_sts3215(6),),
        observations=(ActuatorFeedback(),),
    ),
    # aperture-in-meters not yet measured; episodes carry the joint value
)


#: The arm with its jaw: the static jaw is the wrist-roll link itself, so the jaw is the arm's
#: own, not a module of its own.
SO101_MODULE: Final = Module(
    module_id=ModuleId("so101"),
    label="SO-101 arm with its jaw",
    description=SO101_URDF,
    plug=InterfaceId("so101-base"),
    components=(
        body_component("arm", SO101_ARM, RootMount("base_link"), limb=Limb.ARM),
        body_component("jaw", SO101_JAW, MountedOn("arm", "gripper_link")),
    ),
)

SO101_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("so101"),
    label="SO-101 (single arm)",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="so101"),
    build=Build.of(SO101_MODULE),
    extra_assets=(SO101_URDF,),
    # rates unbound: sim benchmarks and hobby rigs run at their own configured rates
)

#: Two single arms side by side at a nominal inspection layout; two sides ARE the bimanual body.
BIMANUAL_SO101_BUILD: Final = Build(
    robot="bimanual_so101",
    root="bimanual_origin",
    note=INSPECTION_NOTE,
    placed=(
        Placed("left", SO101_MODULE, Pose((0.0, 0.25, 0.0))),
        Placed("right", SO101_MODULE, Pose((0.0, -0.25, 0.0))),
    ),
)

BIMANUAL_SO101_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("bimanual-so101"),
    label="Bimanual SO-101",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="so101", variant="bimanual"),
    build=BIMANUAL_SO101_BUILD,
    extra_assets=(SO101_SOURCE_URDF, BIMANUAL_SO101_URDF),
    # rates unbound: per-episode control_hz travels with the recordings
)
