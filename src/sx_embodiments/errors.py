"""Typed failure modes for every sx_embodiments contract violation.

Every rejection in this package raises a subclass of :class:`EmbodimentError` carrying the
identifying field of whatever failed. The base inherits :class:`ValueError` deliberately:
consumer boundaries that funnel construction failures through ``except ValueError`` (the
data-catalog episode-dir ingest, pre-adoption tests) keep failing closed while adopters
migrate to the specific types.
"""


class EmbodimentError(ValueError):
    """Base for all sx_embodiments contract violations."""


class AssetIntegrityError(EmbodimentError):
    """An asset reference is malformed: bad URI, digest, or size."""


class MissingUrdfError(EmbodimentError):
    """An episode-ready embodiment has no single authoritative URDF description."""

    def __init__(self, name: str, count: int) -> None:
        super().__init__(f"{name}: expected exactly one URDF description asset, found {count}")
        self.name = name
        self.count = count


class AssetDigestMismatchError(AssetIntegrityError):
    """Packaged asset bytes do not match their declared content digest."""

    def __init__(self, relpath: str, expected: str, actual: str) -> None:
        super().__init__(f"{relpath}: expected sha256 {expected}, got {actual}")
        self.relpath = relpath
        self.expected = expected
        self.actual = actual


class AssetsUnavailableError(EmbodimentError):
    """The description-asset tree cannot be located on this host.

    Raised by :func:`sx_embodiments.assets.asset_root` when neither the
    ``SX_EMBODIMENTS_ASSETS`` environment variable nor the repo-relative ``assets/``
    directory resolves. Never degraded silently.
    """


class PartValidationError(EmbodimentError):
    """A part record's own invariants fail."""

    def __init__(self, part_id: str, message: str) -> None:
        super().__init__(f"{part_id}: {message}")
        self.part_id = part_id


class GripperKinematicsError(PartValidationError):
    """A gripper's drive↔aperture relation is not derivable from its declared facts."""


class CompositionError(EmbodimentError):
    """An embodiment's attachments violate its kind's composition rules."""

    def __init__(self, name: str, message: str) -> None:
        super().__init__(f"{name}: {message}")
        self.name = name


class ComponentGraphError(EmbodimentError):
    """A component id, frame, parent relation, or capability fact is invalid."""


class LayoutError(EmbodimentError):
    """A native state space cannot be derived or does not match the requested shape."""

    def __init__(self, name: str, message: str) -> None:
        super().__init__(f"{name}: {message}")
        self.name = name


class InvalidCameraMountError(CompositionError):
    """An embodiment camera has no explicit mount frame."""


class UnknownEmbodimentError(EmbodimentError):
    """An embodiment name is not in the canonical registry."""

    def __init__(self, name: str) -> None:
        super().__init__(f"unknown embodiment: {name!r}")
        self.name = name


class EmbodimentSchemaError(EmbodimentError):
    """An embodiment document does not conform to the supported wire schema."""
