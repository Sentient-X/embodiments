"""The lazy built-in embodiment registry, one source module per hardware family."""

from collections.abc import Iterator, Mapping
from dataclasses import dataclass
from enum import StrEnum
from pathlib import PurePosixPath
from typing import Final

from sx_contracts.assets import AssetIntegrityError

from ..assets import AssetAudience, PackagedAsset, audience
from ..compose import EmbodimentDefinition
from ..embodiment import Embodiment, embodiment_from_definition, packaged_assets
from ..errors import CompositionError, UnknownEmbodimentError
from ..identity import EmbodimentId, EmbodimentName
from ..mechanical import Module
from ..recorded_streams import RecordedStreams
from ..viewing import Viewing
from ._previews import PREVIEWS
from .aloha import ALOHA_SPEC
from .b601 import B601_DM_SPEC, B601_DM_STATION_SPEC, BIMANUAL_B601_DM_SPEC
from .das import DAS_UMI_V4_SPEC, QUEST_EGO_SPEC
from .ffw import FFW_BG2_SPEC
from .g1 import UNITREE_G1_SPEC
from .humanoid import (
    SENTIENT_HUMANOID_SPEC,
    SENTIENT_HUMANOID_UPPER_BODY_GRIPPERS_SPEC,
    SENTIENT_HUMANOID_UPPER_BODY_SPEC,
)
from .insta360 import INSTA360_UMI_SPEC
from .panda import FRANKA_SPEC, PANDA_OMRON_SPEC
from .piper import PIPER_SPEC
from .piper_umi import PIPER_UMI_SPEC
from .rby1 import RBY1_SPEC
from .sentient_rwh import SENTIENT_RWH_SPEC
from .so101 import BIMANUAL_SO101_SPEC, SO101_SPEC
from .stararm102 import STARARM102_SPEC
from .static_upper_body import STATIC_UPPER_BODY_SPEC
from .stations import PIPERX_STATION_SPEC
from .universal_robots import UR5E_SPEC, UR10E_SPEC
from .yor import YOR_SPEC
from .yubi import YUBI_SPEC

_ALL_SPECS: Final[tuple[EmbodimentDefinition, ...]] = (
    PIPER_SPEC,
    ALOHA_SPEC,
    RBY1_SPEC,
    UNITREE_G1_SPEC,
    UR10E_SPEC,
    UR5E_SPEC,
    YOR_SPEC,
    SENTIENT_HUMANOID_SPEC,
    SENTIENT_HUMANOID_UPPER_BODY_SPEC,
    PANDA_OMRON_SPEC,
    FRANKA_SPEC,
    SO101_SPEC,
    BIMANUAL_SO101_SPEC,
    QUEST_EGO_SPEC,
    B601_DM_SPEC,
    BIMANUAL_B601_DM_SPEC,
)

_DEFINITIONS: Final[Mapping[EmbodimentName, EmbodimentDefinition]] = {
    spec.name: spec for spec in _ALL_SPECS
}


class DevelopmentReason(StrEnum):
    MISSING_AUTHORITATIVE_DESCRIPTION = "missing_authoritative_description"
    MISSING_CAMERA_CALIBRATION = "missing_camera_calibration"
    MISSING_CAMERA_INSTALLATION = "missing_camera_installation"
    MISSING_JOINT_LIMITS = "missing_joint_limits"


@dataclass(frozen=True, slots=True)
class DevelopmentEmbodiment:
    spec: EmbodimentDefinition
    reason: DevelopmentReason


DEVELOPMENT_EMBODIMENTS: Final[Mapping[EmbodimentName, DevelopmentEmbodiment]] = {
    STARARM102_SPEC.name: DevelopmentEmbodiment(
        spec=STARARM102_SPEC,
        reason=DevelopmentReason.MISSING_AUTHORITATIVE_DESCRIPTION,
    ),
    PIPER_UMI_SPEC.name: DevelopmentEmbodiment(
        spec=PIPER_UMI_SPEC,
        reason=DevelopmentReason.MISSING_CAMERA_CALIBRATION,
    ),
    INSTA360_UMI_SPEC.name: DevelopmentEmbodiment(
        spec=INSTA360_UMI_SPEC,
        reason=DevelopmentReason.MISSING_AUTHORITATIVE_DESCRIPTION,
    ),
    DAS_UMI_V4_SPEC.name: DevelopmentEmbodiment(
        spec=DAS_UMI_V4_SPEC,
        reason=DevelopmentReason.MISSING_CAMERA_CALIBRATION,
    ),
    YUBI_SPEC.name: DevelopmentEmbodiment(
        spec=YUBI_SPEC,
        reason=DevelopmentReason.MISSING_CAMERA_CALIBRATION,
    ),
    PIPERX_STATION_SPEC.name: DevelopmentEmbodiment(
        spec=PIPERX_STATION_SPEC,
        reason=DevelopmentReason.MISSING_CAMERA_INSTALLATION,
    ),
    B601_DM_STATION_SPEC.name: DevelopmentEmbodiment(
        spec=B601_DM_STATION_SPEC,
        reason=DevelopmentReason.MISSING_CAMERA_INSTALLATION,
    ),
    FFW_BG2_SPEC.name: DevelopmentEmbodiment(
        spec=FFW_BG2_SPEC,
        reason=DevelopmentReason.MISSING_CAMERA_CALIBRATION,
    ),
    SENTIENT_RWH_SPEC.name: DevelopmentEmbodiment(
        spec=SENTIENT_RWH_SPEC,
        reason=DevelopmentReason.MISSING_JOINT_LIMITS,
    ),
    SENTIENT_HUMANOID_UPPER_BODY_GRIPPERS_SPEC.name: DevelopmentEmbodiment(
        spec=SENTIENT_HUMANOID_UPPER_BODY_GRIPPERS_SPEC,
        reason=DevelopmentReason.MISSING_AUTHORITATIVE_DESCRIPTION,
    ),
    STATIC_UPPER_BODY_SPEC.name: DevelopmentEmbodiment(
        spec=STATIC_UPPER_BODY_SPEC,
        reason=DevelopmentReason.MISSING_AUTHORITATIVE_DESCRIPTION,
    ),
}


class EmbodimentRegistry(Mapping[str, Embodiment]):
    """Read-only, lazy registry with ordinary mapping semantics."""

    def __init__(self, definitions: Mapping[EmbodimentName, EmbodimentDefinition]) -> None:
        self._definitions = dict(definitions)
        self._cache: dict[EmbodimentName, Embodiment] = {}

    def __getitem__(self, key: str | EmbodimentId | EmbodimentName) -> Embodiment:
        name = EmbodimentName(str(key))
        definition = self._definitions.get(name)
        if definition is None:
            for candidate in self.values():
                if candidate.id == key:
                    return candidate
            # An earlier revision's identity reads as the revision that superseded it.
            for earlier in self._definitions.values():
                if str(key) in earlier.earlier_revisions:
                    return self[earlier.name]
            raise UnknownEmbodimentError(str(key))
        cached = self._cache.get(name)
        if cached is None:
            cached = embodiment_from_definition(definition)
            self._cache[name] = cached
        return cached

    def __contains__(self, key: object) -> bool:
        """Whether ``key`` names, or is the id of, a body here: the lookup's answer, without
        its refusal (an `UnknownEmbodimentError` is not a `KeyError`)."""
        if not isinstance(key, str):
            return False
        if EmbodimentName(key) in self._definitions:
            return True
        try:
            self[key]
        except UnknownEmbodimentError:
            return False
        return True

    def __iter__(self) -> Iterator[str]:
        return (str(name) for name in self._definitions)

    def __len__(self) -> int:
        return len(self._definitions)


def viewing_of(name: str) -> Viewing:
    """The viewing facts ``name``'s registry module declares, the default when it declares
    none; raises `UnknownEmbodimentError` for a name no registry holds."""
    key = EmbodimentName(name)
    definition = _DEFINITIONS.get(key)
    if definition is None and key in DEVELOPMENT_EMBODIMENTS:
        definition = DEVELOPMENT_EMBODIMENTS[key].spec
    if definition is None:
        raise UnknownEmbodimentError(name)
    return definition.viewing


def recorded_streams_of(name: str) -> RecordedStreams:
    """What a capture rig's recording names ``name``'s devices and measured coordinates; none
    for a body no rig records. Raises `UnknownEmbodimentError` for a name no registry holds."""
    key = EmbodimentName(name)
    definition = _DEFINITIONS.get(key)
    if definition is None and key in DEVELOPMENT_EMBODIMENTS:
        definition = DEVELOPMENT_EMBODIMENTS[key].spec
    if definition is None:
        raise UnknownEmbodimentError(name)
    return definition.recorded_streams


def collection_definitions() -> tuple[EmbodimentDefinition, ...]:
    """Collection catalog, including equipment whose physical model is incomplete.

    Reads definitions only: inventory discovery does not load meshes, parse URDFs,
    or promote a development revision to an executable embodiment.
    """
    return tuple(
        definition
        for definition in (
            *_ALL_SPECS,
            *(entry.spec for entry in DEVELOPMENT_EMBODIMENTS.values()),
        )
        if definition.collection is not None
    )


embodiments: Final = EmbodimentRegistry(_DEFINITIONS)


def _described(definition: EmbodimentDefinition) -> bool:
    """Whether any module of ``definition`` has a description: an embodiment is built from
    one, so a definition with none (a rig whose parts are unmeasured) is listed, not built."""
    return any(placed.module.description is not None for placed in definition.build.placed)


development_embodiments: Final = EmbodimentRegistry(
    {name: entry.spec for name, entry in DEVELOPMENT_EMBODIMENTS.items() if _described(entry.spec)}
)
"""The same objects, from the registry that says they are not conformant yet.

`DEVELOPMENT_EMBODIMENTS` records *why* each of these is held back — a missing camera
calibration or installation, or in the RWH's case a description that declares no joint
limits at all — but held back from what? Without this it was held back from
being resolvable at all, and the consumers that already ran on these bodies simply broke:
sxd's UMI episode emitter is production, it emits `das-umi-v4` episodes daily, and the
factory seeds a `piperx-station` pod.

So the demotion is real but it is a *statement*, not a removal. A caller that needs one of
these reads it from here, and the call site then says out loud that the body it is using
has a fact still under construction. Promotion is one line: move the spec into
`_ALL_SPECS`, and every `development_embodiments[...]` that should follow it fails loudly.
"""


def asset_audiences() -> Mapping[str, AssetAudience]:
    """The audience of every top-level directory in the asset tree.

    Publication works in whole directories because only a *declared* asset carries a
    licence: a URDF names its meshes, but those thousands of mesh files have no
    declaration of their own. A robot's directory is the smallest unit whose audience is
    knowable, and it is the natural one — a description and the meshes it references are
    one indivisible thing.

    Two typed refusals rather than a guess. A directory whose declared assets disagree
    about their audience cannot be published either way, and is a registry mistake worth
    failing on. A directory that no declaration claims is unclassifiable, which the
    publication tooling must treat as fatal — the safe reading of "nobody said" is never
    "public".
    """
    audiences: dict[str, AssetAudience] = {}
    for definition in (
        *_ALL_SPECS,
        *(entry.spec for entry in DEVELOPMENT_EMBODIMENTS.values()),
    ):
        for packaged in packaged_assets(definition):
            directory = PurePosixPath(packaged.relpath).parts[0]
            declared = audience(packaged.provenance.license_id)
            established = audiences.setdefault(directory, declared)
            if established is not declared:
                raise AssetIntegrityError(
                    f"{directory}: declared both {established} and {declared}; "
                    "an asset directory has one audience or it cannot be published"
                )
    for preview in PREVIEWS.values():
        directory = PurePosixPath(preview.relpath).parts[0]
        declared = audience(preview.provenance.license_id)
        established = audiences.setdefault(directory, declared)
        if established is not declared:
            raise AssetIntegrityError(f"{directory}: preview audience disagrees with its sources")
    return audiences


def definitions() -> tuple[EmbodimentDefinition, ...]:
    """All known hardware, including incomplete revisions; never a hand-picked subset."""
    return (*_ALL_SPECS, *(entry.spec for entry in DEVELOPMENT_EMBODIMENTS.values()))


def modules() -> tuple[Module, ...]:
    """Every module the registry's bodies are built from, fitted ones included, once each."""
    found: dict[str, Module] = {}

    def visit(module: Module) -> None:
        known = found.setdefault(str(module.module_id), module)
        if known is not module:
            if known != module:
                raise CompositionError(str(module.module_id), "two modules share one id")
            return
        for socket in module.sockets:
            if socket.fitted is not None:
                visit(socket.fitted)

    for definition in definitions():
        for entry in (*definition.build.placed, *definition.build.mates):
            visit(entry.module)
    return tuple(found.values())


def preview_asset(embodiment: Embodiment) -> PackagedAsset | None:
    """Pinned display asset for this exact body; absent for unrendered compositions.

    Display assets are derived publications, separate from hardware identity.
    Consumers receive verified bytes through PackagedAsset.path, not URDF internals.
    """
    return PREVIEWS.get(embodiment.id)
