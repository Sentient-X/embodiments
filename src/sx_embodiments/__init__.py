"""Portable embodiment contracts: identity, parts, composition, assets, and the registry.

The compositional :class:`EmbodimentSpec` is the single source; the kinematic
:class:`Embodiment`, channel :class:`FlatLayout`, camera-name sets, and wire
:class:`EmbodimentManifest` are derived views. FK solving, simulator loading, Rerun
emission, drivers, and asset hosting stay at consumer boundaries — this package states
facts and derives views; it never executes.
"""

from .assets import AssetFormat, AssetRef, AssetRole, PackagedAsset, asset_root
from .compose import (
    Attachment,
    AttachmentRole,
    EmbodimentSpec,
    MountFrame,
    camera_bindings,
    camera_names,
    flat_layout,
    kinematic_view,
    total_dof,
)
from .curves import Curve1D
from .errors import (
    AssetIntegrityError,
    AssetsUnavailableError,
    CompositionError,
    EmbodimentError,
    LayoutError,
    ManifestSchemaError,
    PartValidationError,
    UnknownEmbodimentError,
)
from .identity import (
    EmbodimentId,
    EmbodimentKind,
    EmbodimentManifestDigest,
    EmbodimentRef,
    Lineage,
    PartId,
)
from .kinematic import Embodiment
from .known import (
    EMBODIMENTS,
    PANDA_OMRON,
    PIPER,
    embodiment_spec,
    layout_for,
)
from .layout import ChannelKind, ChannelSlot, FlatLayout
from .manifest import EmbodimentManifest, manifest_for, manifest_from_dict
from .parts import (
    ArmSpec,
    CameraBinding,
    CameraModality,
    CameraSpec,
    ControlRates,
    DeviceSpec,
    ForceTorqueSpec,
    GripperSpec,
    JointGroupSpec,
    MimicJoint,
    MobileBaseSpec,
    Part,
    SensorModel,
)

__all__ = [
    "EMBODIMENTS",
    "PANDA_OMRON",
    "PIPER",
    "ArmSpec",
    "AssetFormat",
    "AssetIntegrityError",
    "AssetRef",
    "AssetRole",
    "AssetsUnavailableError",
    "Attachment",
    "AttachmentRole",
    "CameraBinding",
    "CameraModality",
    "CameraSpec",
    "ChannelKind",
    "ChannelSlot",
    "CompositionError",
    "ControlRates",
    "Curve1D",
    "DeviceSpec",
    "Embodiment",
    "EmbodimentError",
    "EmbodimentId",
    "EmbodimentKind",
    "EmbodimentManifest",
    "EmbodimentManifestDigest",
    "EmbodimentRef",
    "EmbodimentSpec",
    "FlatLayout",
    "ForceTorqueSpec",
    "GripperSpec",
    "JointGroupSpec",
    "LayoutError",
    "Lineage",
    "ManifestSchemaError",
    "MimicJoint",
    "MobileBaseSpec",
    "MountFrame",
    "PackagedAsset",
    "Part",
    "PartId",
    "PartValidationError",
    "SensorModel",
    "UnknownEmbodimentError",
    "asset_root",
    "camera_bindings",
    "camera_names",
    "embodiment_spec",
    "flat_layout",
    "kinematic_view",
    "layout_for",
    "manifest_for",
    "manifest_from_dict",
    "total_dof",
]
