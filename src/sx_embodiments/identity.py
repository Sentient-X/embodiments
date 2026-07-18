"""Embodiment identity: the flat join key, part ids, kinds, and structured lineage.

``EmbodimentId`` stays a flat opaque string — it lives in Postgres rows, ``.rrd`` metadata,
wire JSON, and LeRobot exports, and equality is string equality. Structured ancestry lives
INSIDE the record as :class:`Lineage`; it is never parsed out of the id. A hardware revision
that changes kinematics or geometry gets a NEW id; a cosmetic revision keeps the id and notes
itself in ``Lineage.revision``.
"""

from dataclasses import dataclass
from enum import StrEnum
from typing import NewType

from .errors import ManifestSchemaError

EmbodimentId = NewType("EmbodimentId", str)
PartId = NewType("PartId", str)


class EmbodimentManifestDigest(str):
    """Validated lowercase SHA-256 identity of a complete manifest revision."""

    def __new__(cls, value: str) -> "EmbodimentManifestDigest":
        if len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
            raise ManifestSchemaError(
                "embodiment manifest digest must be 64 lowercase hexadecimal characters"
            )
        return super().__new__(cls, value)


@dataclass(frozen=True, slots=True)
class EmbodimentRef:
    """Exact deployable hardware identity: body name plus immutable manifest revision."""

    embodiment_id: EmbodimentId
    manifest_sha256: EmbodimentManifestDigest

    def __post_init__(self) -> None:
        if not str(self.embodiment_id).strip():
            raise ManifestSchemaError("embodiment ref id must not be empty")


class EmbodimentKind(StrEnum):
    """What role a piece of hardware plays in the data lifecycle."""

    ROBOT = "robot"  # a body that executes actions (autonomously or teleoperated)
    CAPTURE_RIG = "capture_rig"  # human-held/worn data-collection hardware
    TELEOP_STATION = "teleop_station"  # a leader/follower pair


@dataclass(frozen=True, slots=True)
class Lineage:
    """Structured ancestry carried inside the record, never encoded in the id string."""

    family: str  # "so101", "das-umi", "yubi", "b601", "piper", "panda"
    variant: str = ""  # "bimanual", "widejaw", "dm"
    revision: str = ""  # hardware revision when it does NOT change data semantics
