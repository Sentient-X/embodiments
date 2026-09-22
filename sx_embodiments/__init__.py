"""One content-addressed embodiment object and a friendly-name registry.

Registry lookup, checked assembly and composition return the same complete Embodiment.
Parts stay owned by their authoring source; composition namespaces instances and frames.
"""

from .assemble import admit_part, assemble, composable_parts, part_from_dict, part_to_dict
from .assets import resolve_asset
from .compose import BaseMount, MountKind, OperatorMount, OperatorSite
from .composition import PlacedEmbodiment, compose_embodiments
from .embodiment import Embodiment, EmbodimentMigration, convert_v13_to_v14
from .errors import (
    AssemblyError,
    AssetDigestMismatchError,
    AssetsUnavailableError,
    ComponentGraphError,
    CompositionError,
    EmbodimentError,
    EmbodimentSchemaError,
    GripperKinematicsError,
    InvalidCameraMountError,
    LayoutError,
    MissingUrdfError,
    PartValidationError,
    UnknownEmbodimentError,
)
from .identity import EmbodimentId, EmbodimentKind, EmbodimentName
from .known import development_embodiments, embodiments, preview_asset
from .layout import (
    ActuationBinding,
    ActuatorBinding,
    ActuatorBus,
    ActuatorFeedback,
    ActuatorModel,
    Bounds,
    ChannelKind,
    CoordinateBounds,
    CoordinateUnit,
    DirectDrive,
    EncoderReadout,
    IntegratedDrive,
    ObservationBinding,
    Passive,
    Unbounded,
    UndocumentedDrive,
    Unobserved,
    VendorReadout,
)
from .parts import CameraOptics, CameraOpticsAuthority, FactSource

__all__ = [
    "preview_asset",
    "ActuationBinding",
    "ActuatorBinding",
    "ActuatorBus",
    "ActuatorFeedback",
    "ActuatorModel",
    "AssemblyError",
    "AssetDigestMismatchError",
    "AssetsUnavailableError",
    "BaseMount",
    "Bounds",
    "CameraOptics",
    "CameraOpticsAuthority",
    "ChannelKind",
    "ComponentGraphError",
    "CompositionError",
    "CoordinateBounds",
    "CoordinateUnit",
    "DirectDrive",
    "Embodiment",
    "EmbodimentError",
    "EmbodimentId",
    "EmbodimentKind",
    "EmbodimentMigration",
    "EmbodimentName",
    "EmbodimentSchemaError",
    "EncoderReadout",
    "FactSource",
    "GripperKinematicsError",
    "IntegratedDrive",
    "InvalidCameraMountError",
    "LayoutError",
    "MissingUrdfError",
    "MountKind",
    "ObservationBinding",
    "OperatorMount",
    "OperatorSite",
    "PartValidationError",
    "Passive",
    "PlacedEmbodiment",
    "Unbounded",
    "UndocumentedDrive",
    "UnknownEmbodimentError",
    "Unobserved",
    "VendorReadout",
    "admit_part",
    "assemble",
    "composable_parts",
    "compose_embodiments",
    "convert_v13_to_v14",
    "development_embodiments",
    "embodiments",
    "part_from_dict",
    "part_to_dict",
    "resolve_asset",
]
