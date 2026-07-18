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

EmbodimentId = NewType("EmbodimentId", str)
PartId = NewType("PartId", str)


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
