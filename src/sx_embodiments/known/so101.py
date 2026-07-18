"""SO-101 family: the 5-DOF hobby arm and the bimanual pair supervisors runs.

Joint names and limits mirror ``assets/so101/so101.urdf`` (pinned by
``tests/test_urdf_parity.py``). The bimanual flat convention ``[left 0..5 | right 0..5]``
with the jaw at index 5 of each block is exactly the declared attachment order — the
supervisors ``ChannelLayout(arms=2, block=6, gripper_index=5)`` falls out of
``flat_layout(...).uniform_arm_blocks()``.
"""

from typing import Final

from ..assets import AssetFormat, AssetRole, PackagedAsset
from ..compose import Attachment, AttachmentRole, EmbodimentSpec, MountFrame
from ..identity import EmbodimentId, EmbodimentKind, Lineage, PartId
from ..parts import ArmSpec, GripperSpec

SO101_URDF: Final = PackagedAsset(
    relpath="so101/so101.urdf",
    sha256="dd7f789c1aa4b9f82174dd49f6c4d62f5338f0956ec8e59c37576ee161903279",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
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


def so101_side(side: str) -> tuple[Attachment, ...]:
    """One SO-101 arm+jaw block; composing two sides IS the bimanual body."""
    return (
        Attachment(f"{side}_arm", SO101_ARM, AttachmentRole.BODY),
        Attachment(
            f"{side}_jaw", SO101_JAW, AttachmentRole.BODY, MountFrame(f"{side}_arm", "wrist_roll")
        ),
    )


BIMANUAL_SO101_SPEC: Final = EmbodimentSpec(
    embodiment_id=EmbodimentId("bimanual-so101"),
    name="Bimanual SO-101",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="so101", variant="bimanual"),
    attachments=(*so101_side("left"), *so101_side("right")),
    # rates unbound: per-episode control_hz travels with the recordings
)
