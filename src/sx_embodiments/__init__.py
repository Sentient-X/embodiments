"""One content-addressed embodiment object and a friendly-name registry."""

from .assets import (
    AssetFormat,
    AssetProvenance,
    AssetRef,
    AssetRole,
    EmbodiedAsset,
    validate_logical_path,
)
from .compose import Component, ComponentKind, ComponentRole
from .embodiment import Embodiment
from .errors import (
    AssetDigestMismatchError,
    AssetIntegrityError,
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
from .identity import EmbodimentId, EmbodimentKind, EmbodimentName, PartId
from .known import EmbodimentRegistry, embodiments
from .layout import ChannelKind, CoordinateUnit, StateSpace
from .parts import CameraModality, LensProjection, SensorModel

__all__ = [
    "AssetDigestMismatchError",
    "AssetFormat",
    "AssetIntegrityError",
    "AssetProvenance",
    "AssetRef",
    "AssetRole",
    "AssetsUnavailableError",
    "CameraModality",
    "ChannelKind",
    "Component",
    "ComponentGraphError",
    "ComponentKind",
    "ComponentRole",
    "CompositionError",
    "CoordinateUnit",
    "EmbodiedAsset",
    "Embodiment",
    "EmbodimentError",
    "EmbodimentId",
    "EmbodimentKind",
    "EmbodimentName",
    "EmbodimentRegistry",
    "EmbodimentSchemaError",
    "GripperKinematicsError",
    "InvalidCameraMountError",
    "LayoutError",
    "LensProjection",
    "MissingUrdfError",
    "PartId",
    "PartValidationError",
    "SensorModel",
    "StateSpace",
    "UnknownEmbodimentError",
    "embodiments",
    "validate_logical_path",
]
