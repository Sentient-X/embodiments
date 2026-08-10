"""One content-addressed embodiment object and a friendly-name registry.

The public surface is read-only on purpose. ``embodiments[...]`` hands out complete
:class:`~sx_embodiments.embodiment.Embodiment` objects; everything else exported here is
vocabulary for *reading* one — state layout, base mounts, typed errors, asset resolution.
The materials an embodiment is assembled from (components, attachments, parts, joint
layouts, lineage, kinds) are not exported, so no caller outside this package can reach the
arguments ``Embodiment(...)`` requires. Registration is a change to ``sx_embodiments.known``.
"""

from .assets import resolve_asset
from .compose import BaseMount, MountKind, OperatorMount, OperatorSite
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
from .identity import EmbodimentId, EmbodimentName
from .known import development_embodiments, embodiments
from .layout import Bounds, ChannelKind, CoordinateBounds, CoordinateUnit, Unbounded
from .parts import CameraOptics, CameraOpticsAuthority, FactSource

__all__ = [
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
    "Embodiment",
    "EmbodimentError",
    "EmbodimentId",
    "EmbodimentName",
    "EmbodimentSchemaError",
    "FactSource",
    "GripperKinematicsError",
    "InvalidCameraMountError",
    "LayoutError",
    "MissingUrdfError",
    "MountKind",
    "OperatorMount",
    "OperatorSite",
    "PartValidationError",
    "Unbounded",
    "UnknownEmbodimentError",
    "development_embodiments",
    "embodiments",
    "resolve_asset",
]
