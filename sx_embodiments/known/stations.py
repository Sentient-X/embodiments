"""The fully described PiperX teleoperation station used by the factory."""

from typing import Final

from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import (
    EmbodimentDefinition,
    MountedOn,
    RootMount,
    body_component,
    leader_component,
    sensor_component,
)
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..parts import DeviceSpec
from .das import UVC_MONO_60
from .piper import PIPER_ARM, PIPER_GRIPPER, PIPER_SOURCE_URDF

PIPER_STATION_URDF: Final = packaged_asset(
    relpath="official/agilex_piper/piper_station.urdf",
    sha256="0285385fbd97d9f0e25cb38556f4af7194de860354f7b7f081fabd3248b2a607",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/agilexrobotics/piper_ros",
        revision="ac41fcbcdda598f01b51cf6175ed9a24d0dacadc",
        path="src/piper_description/urdf/piper_description.urdf",
        license_id="MIT",
        generator="sx-embodiments/tools/compose_registered_urdfs.py",
    ),
    media_type="application/xml",
)

PIPERX_STATION_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("piperx-station"),
    label="PiperX single-arm teleop station",
    kind=EmbodimentKind.TELEOP_STATION,
    lineage=Lineage(family="piper", variant="piperx"),
    attachments=(
        leader_component(
            "leader",
            DeviceSpec(PartId("piperx-leader"), "PiperX leader arm"),
            RootMount("leader_base_link"),
        ),
        body_component("follower_arm", PIPER_ARM, RootMount("follower_base_link")),
        body_component(
            "follower_gripper",
            PIPER_GRIPPER,
            MountedOn("follower_arm", "follower_link6"),
        ),
        sensor_component("front", UVC_MONO_60, RootMount("front_camera")),
    ),
    extra_assets=(PIPER_SOURCE_URDF, PIPER_STATION_URDF),
)
