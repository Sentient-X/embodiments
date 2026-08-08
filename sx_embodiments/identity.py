"""Small identity vocabulary for content-addressed embodiments."""

from dataclasses import dataclass
from enum import StrEnum
from typing import NewType

from sx_contracts.wire import HexDigest

from .errors import EmbodimentSchemaError

EmbodimentName = NewType("EmbodimentName", str)
PartId = NewType("PartId", str)


class EmbodimentId(HexDigest):
    """The lowercase SHA-256 of one complete embodiment document."""

    error = EmbodimentSchemaError
    noun = "embodiment id"


class EmbodimentKind(StrEnum):
    """What role a piece of hardware plays in the data lifecycle."""

    ROBOT = "robot"  # a body that executes actions (autonomously or teleoperated)
    CAPTURE_RIG = "capture_rig"  # human-held/worn data-collection hardware
    TELEOP_STATION = "teleop_station"  # a leader/follower pair


@dataclass(frozen=True, slots=True)
class Lineage:
    """Internal registry metadata preserved in the portable document."""

    family: str
    variant: str = ""
    revision: str = ""
