"""Small identity vocabulary for content-addressed embodiments."""

from dataclasses import dataclass
from enum import StrEnum
from typing import NewType

from .errors import EmbodimentSchemaError

EmbodimentName = NewType("EmbodimentName", str)
PartId = NewType("PartId", str)


class EmbodimentId(str):
    """The lowercase SHA-256 of one complete embodiment document."""

    def __new__(cls, value: str) -> "EmbodimentId":
        if len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
            raise EmbodimentSchemaError("embodiment id must be 64 lowercase hexadecimal characters")
        return super().__new__(cls, value)


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
