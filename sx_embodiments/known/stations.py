"""The fully described PiperX teleoperation station used by the factory."""

from typing import Final

from ..assembly import INSPECTION_NOTE, Frame, MountJoint, Pose
from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..collection import CollectionMethod, CollectionSetup
from ..compose import (
    EmbodimentDefinition,
    RootMount,
    leader_component,
)
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..mechanical import Build, Module, ModuleId, Placed
from ..parts import DeviceSpec
from .piper import PIPER_MODULE, PIPER_SOURCE_URDF, PIPER_URDF

PIPER_STATION_URDF: Final = packaged_asset(
    relpath="official/agilex_piper/piper_station.urdf",
    sha256="0285385fbd97d9f0e25cb38556f4af7194de860354f7b7f081fabd3248b2a607",
    size_bytes=22394,
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

#: The leader is a Piper description with no drive declared; the station's leader device sits
#: on its base.
_PIPER_LEADER_MODULE: Final = Module(
    module_id=ModuleId("piperx-leader"), label="PiperX leader arm", description=PIPER_URDF
)

#: A leader and a follower side by side, with the front camera's installation default.
PIPER_STATION_BUILD: Final = Build(
    robot="piper_station",
    root="station_origin",
    note=INSPECTION_NOTE,
    placed=(
        Placed("leader", _PIPER_LEADER_MODULE, Pose((0.0, 0.42, 0.0))),
        Placed("follower", PIPER_MODULE, Pose((0.0, -0.42, 0.0))),
    ),
    frames=(
        Frame(
            "front_camera",
            "station_origin",
            Pose((-0.35, 0.0, 0.85), (0.0, 0.65, 0.0)),
            MountJoint.FLOATING,
            "camera",
            "Front camera",
        ),
    ),
    devices=(
        leader_component(
            "leader",
            DeviceSpec(PartId("piperx-leader"), "PiperX leader arm"),
            RootMount("leader_base_link"),
        ),
    ),
)

PIPERX_STATION_SPEC: Final = EmbodimentDefinition(
    collection=CollectionSetup(
        method=CollectionMethod.TELEOP,
        description="A leader arm guides a Piper follower to collect manipulation demonstrations.",
        devices=(),
        notes=(
            "Camera models, counts, and mounting positions are not verified for this station. "
            "The arm description alone does not establish a camera setup.",
            "Leader and follower joint state can describe motion; encoder part numbers and "
            "the recorded channel configuration are not documented here.",
        ),
    ),
    name=EmbodimentName("piperx-station"),
    label="PiperX leader-follower station",
    kind=EmbodimentKind.TELEOP_STATION,
    lineage=Lineage(family="piper", variant="piperx"),
    build=PIPER_STATION_BUILD,
    extra_assets=(PIPER_SOURCE_URDF, PIPER_STATION_URDF),
)
