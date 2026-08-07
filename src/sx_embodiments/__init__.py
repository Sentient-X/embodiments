"""One content-addressed embodiment object and a friendly-name registry."""

from sx_contracts import ProvenancedAsset

from .assets import asset_root, resolve_asset
from .compose import (
    BaseMount,
    BodyAttachment,
    Component,
    ComponentKind,
    ComponentRole,
    LeaderAttachment,
    MountedOn,
    MountKind,
    RootMount,
    SensorAttachment,
)
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
from .layout import (
    Bounds,
    ChannelKind,
    CoordinateBounds,
    CoordinateUnit,
    JointAxis,
    JointLayout,
    StateSpace,
    Unbounded,
)
from .parts import CameraModality, LensProjection, SensorModel

__all__ = [
    "AssetDigestMismatchError",
    "AssetsUnavailableError",
    "BaseMount",
    "BodyAttachment",
    "Bounds",
    "CameraModality",
    "ChannelKind",
    "Component",
    "ComponentGraphError",
    "ComponentKind",
    "ComponentRole",
    "CompositionError",
    "CoordinateBounds",
    "CoordinateUnit",
    "Embodiment",
    "EmbodimentError",
    "EmbodimentId",
    "EmbodimentKind",
    "EmbodimentName",
    "EmbodimentRegistry",
    "EmbodimentSchemaError",
    "GripperKinematicsError",
    "InvalidCameraMountError",
    "JointAxis",
    "JointLayout",
    "LayoutError",
    "LeaderAttachment",
    "LensProjection",
    "MissingUrdfError",
    "MountKind",
    "MountedOn",
    "PartId",
    "PartValidationError",
    "ProvenancedAsset",
    "RootMount",
    "SensorAttachment",
    "SensorModel",
    "StateSpace",
    "Unbounded",
    "UnknownEmbodimentError",
    "asset_root",
    "embodiments",
    "resolve_asset",
]
