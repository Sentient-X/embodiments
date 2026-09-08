"""Collection equipment inventory, independent of calibrated robot geometry.

A device can be identified before its optics or installation are measured. These
records describe that equipment; they never supply CameraSpec, joint limits, or
qualification. Recording configuration and per-unit measurements remain with capture.
"""

from dataclasses import dataclass
from enum import StrEnum

from .errors import PartValidationError
from .parts import FactSource


class CollectionMethod(StrEnum):
    UMI = "umi"
    EGO = "ego"
    TELEOP = "teleop"


class CollectionDeviceKind(StrEnum):
    CAMERA = "camera"
    ENCODER = "encoder"
    TRACKING = "tracking"


@dataclass(frozen=True, slots=True)
class CollectionDevice:
    kind: CollectionDeviceKind
    label: str
    quantity: int
    placement: str
    details: str
    source: FactSource

    def __post_init__(self) -> None:
        if type(self.quantity) is not int or self.quantity < 1:
            raise PartValidationError(self.label, "a listed device must have a positive quantity")
        if not all(value.strip() for value in (self.label, self.placement, self.details)):
            raise PartValidationError(self.label, "device descriptions must not be empty")


@dataclass(frozen=True, slots=True)
class CollectionSetup:
    """Documented collection use; no claim of calibrated geometry or readiness."""

    method: CollectionMethod
    description: str
    devices: tuple[CollectionDevice, ...]
    notes: tuple[str, ...]

    def __post_init__(self) -> None:
        if not self.description.strip() or any(not note.strip() for note in self.notes):
            raise PartValidationError("collection", "collection descriptions must not be empty")
