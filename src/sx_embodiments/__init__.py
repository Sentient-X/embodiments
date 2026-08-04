"""One content-addressed embodiment object and a friendly-name registry."""

from .assets import (
    EmbodiedAsset,
    asset_root,
    resolve_asset,
)
from .compose import BaseMount, Component, ComponentKind, ComponentRole, MountKind
from .embodiment import Embodiment
from .errors import (
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
from .identity import EmbodimentId, EmbodimentKind, EmbodimentName, PartId
from .known import EmbodimentRegistry, embodiments
from .layout import ChannelKind, CoordinateUnit, StateSpace
from .parts import CameraModality, LensProjection, SensorModel

__all__ = [
    "AssetDigestMismatchError",
    "AssetsUnavailableError",
    "BaseMount",
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
    "MountKind",
    "PartId",
    "PartValidationError",
    "SensorModel",
    "StateSpace",
    "UnknownEmbodimentError",
    "asset_root",
    "embodiments",
    "resolve_asset",
]
