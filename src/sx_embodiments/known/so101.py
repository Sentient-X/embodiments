"""SO-101 family: the 5-DOF hobby arm and the bimanual pair supervisors runs.

Joint names and limits mirror ``assets/so101/so101.urdf`` (pinned by
``tests/test_urdf_parity.py``). The bimanual flat convention ``[left 0..5 | right 0..5]``
with the jaw at index 5 of each block is exactly the declared attachment order — the
supervisors ``ChannelLayout(arms=2, block=6, gripper_index=5)`` falls out of
``embodiments[...].state.uniform_arm_blocks()``.
"""

from typing import Final

from ..assets import AssetFormat, AssetProvenance, AssetRole, PackagedAsset
from ..compose import Component, ComponentRole, MountFrame, _EmbodimentDefinition
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..parts import ArmSpec, GripperSpec

SO101_URDF: Final = PackagedAsset(
    relpath="so101/so101.urdf",
    sha256="dd7f789c1aa4b9f82174dd49f6c4d62f5338f0956ec8e59c37576ee161903279",
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

SO101_ARM: Final = ArmSpec(
    part_id=PartId("so101-arm"),
    joint_names=("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"),
    joint_lower=(-1.91986, -1.74533, -1.69, -1.65806, -2.74385),
    joint_upper=(1.91986, 1.74533, 1.69, 1.65806, 2.84121),
    home_joints=(0.0, 0.0, 0.0, 0.0, 0.0),
    assets=(SO101_URDF,),
)

SO101_JAW: Final = GripperSpec(
    part_id=PartId("so101-jaw"),
    joint_names=("gripper",),
    joint_lower=(-0.174533,),
    joint_upper=(2.0944,),
    # aperture-in-meters not yet measured; episodes carry the joint value
)


def so101_side(side: str) -> tuple[Component, ...]:
    """One SO-101 arm+jaw block; composing two sides IS the bimanual body."""
    return (
        Component(f"{side}_arm", SO101_ARM, ComponentRole.BODY),
        Component(
            f"{side}_jaw", SO101_JAW, ComponentRole.BODY, MountFrame(f"{side}_arm", "wrist_roll")
        ),
    )


SO101_SPEC: Final = _EmbodimentDefinition(
    embodiment_id=EmbodimentName("so101"),
    name="SO-101 (single arm)",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="so101"),
    attachments=(
        Component("arm", SO101_ARM, ComponentRole.BODY),
        Component("jaw", SO101_JAW, ComponentRole.BODY, MountFrame("arm", "wrist_roll")),
    ),
    # rates unbound: sim benchmarks and hobby rigs run at their own configured rates
)

BIMANUAL_SO101_SPEC: Final = _EmbodimentDefinition(
    embodiment_id=EmbodimentName("bimanual-so101"),
    name="Bimanual SO-101",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="so101", variant="bimanual"),
    attachments=(*so101_side("left"), *so101_side("right")),
    # rates unbound: per-episode control_hz travels with the recordings
)
