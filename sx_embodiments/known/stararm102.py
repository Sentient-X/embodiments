"""Development StarArm102-LD model; LD geometry is not yet qualified.

The generic upstream URDF retains its joint frames, limits and inertials, but
has no visual/collision geometry. Its bounds differ from the manufacturer's LD
specification; see docs/STARARM102.md for evidence and the promotion gate.
Encoder-to-model calibration remains recording/session input.
"""

from typing import Final

from sx_contracts.assets import AssetFormat, AssetProvenance, AssetRole

from ..assets import packaged_asset
from ..compose import EmbodimentDefinition, MountedOn, RootMount, body_component
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit, UndocumentedDrive, VendorReadout
from ..parts import ArmSpec, GripperSpec, MimicJoint
from ._authoring import bounded_layout

STARARM102_URDF: Final = packaged_asset(
    relpath="stararm102/kinematics.urdf",
    sha256="2316cbdd6f1ac529ef329888452cd9f8c6e36220dbae88cac3c258e20cf1d280",
    size_bytes=7431,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/servodevelop/Star-Arm-102",
        revision="0896306e40891c3ee4c97228e85dd708d61326de",
        path="ROS2_HUMBLE/src/stararm102_description/urdf/stararm102_description.urdf",
        license_id="MIT",
        generator="remove visual/collision elements; normalize CRLF to LF",
    ),
    media_type="application/xml",
)

# The SDK reads seven UART monitor channels (IDs 0..6). These describe the
# readout interface, not a calibrated encoder-to-URDF transform or a drive map.
STARARM102_READOUTS: Final = tuple(
    VendorReadout(f"fashionstar.uart.servo_monitor[{address}].angle_monitor")
    for address in range(7)
)
STARARM102_DRIVE: Final = UndocumentedDrive(
    "LD SDK unlocks the leader for manual input; nominal drive capability and "
    "encoder-to-model mapping are not qualified. See docs/STARARM102.md."
)

STARARM102_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("stararm102-ld"),
    label="StarArm102-LD leader (development kinematics)",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="stararm102", variant="ld"),
    attachments=(
        body_component(
            "arm",
            ArmSpec(
                part_id=PartId("stararm102-arm"),
                layout=bounded_layout(
                    names=tuple(f"joint{i}" for i in range(1, 7)),
                    units=(CoordinateUnit.RADIAN,) * 6,
                    lower=(-2.27, 0.0, -3.14, -1.57, -1.2, -3.14),
                    upper=(2.27, 3.14, 0.0, 2.2, 1.2, 3.14),
                    observations=STARARM102_READOUTS[:6],
                    actuations=(STARARM102_DRIVE,) * 6,
                ),
                home=(0.0,) * 6,
            ),
            RootMount("world"),
        ),
        body_component(
            "jaw",
            GripperSpec(
                part_id=PartId("stararm102-jaw"),
                layout=bounded_layout(
                    names=("joint7_left",),
                    units=(CoordinateUnit.RADIAN,),
                    lower=(0.0,),
                    upper=(1.57,),
                    observations=STARARM102_READOUTS[6:],
                    actuations=(STARARM102_DRIVE,),
                ),
                mimic_joints=(MimicJoint("joint7_right", of="joint7_left", multiplier=-1.0),),
            ),
            MountedOn("arm", "link6"),
        ),
    ),
    extra_assets=(STARARM102_URDF,),
)
