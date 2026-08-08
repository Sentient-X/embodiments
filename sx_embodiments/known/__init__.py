"""The lazy built-in embodiment registry, one source module per hardware family."""

from collections.abc import Iterator, Mapping
from dataclasses import dataclass
from enum import StrEnum
from typing import Final

from ..compose import EmbodimentDefinition
from ..embodiment import Embodiment, embodiment_from_definition
from ..errors import UnknownEmbodimentError
from ..identity import EmbodimentId, EmbodimentName
from .aloha import ALOHA_SPEC
from .b601 import B601_DM_SPEC, B601_DM_STATION_SPEC, BIMANUAL_B601_DM_SPEC
from .das import DAS_UMI_V4_SPEC, QUEST_EGO_SPEC
from .g1 import UNITREE_G1_SPEC
from .humanoid import SENTIENT_HUMANOID_SPEC
from .insta360 import INSTA360_UMI_SPEC
from .nero import NERO_SPEC
from .panda import FRANKA_SPEC, PANDA_OMRON_SPEC
from .piper import PIPER_SPEC
from .rby1 import RBY1_SPEC
from .so101 import BIMANUAL_SO101_SPEC, SO101_SPEC
from .stations import PIPERX_STATION_SPEC
from .universal_robots import UR5E_SPEC, UR10E_SPEC
from .yor import YOR_SPEC
from .yubi import YUBI_DEPTH_SPEC, YUBI_MONO_SPEC, YUBI_WIDEJAW_SPEC

_ALL_SPECS: Final[tuple[EmbodimentDefinition, ...]] = (
    PIPER_SPEC,
    NERO_SPEC,
    ALOHA_SPEC,
    RBY1_SPEC,
    UNITREE_G1_SPEC,
    UR10E_SPEC,
    UR5E_SPEC,
    YOR_SPEC,
    SENTIENT_HUMANOID_SPEC,
    PANDA_OMRON_SPEC,
    FRANKA_SPEC,
    SO101_SPEC,
    BIMANUAL_SO101_SPEC,
    DAS_UMI_V4_SPEC,
    QUEST_EGO_SPEC,
    YUBI_MONO_SPEC,
    YUBI_DEPTH_SPEC,
    YUBI_WIDEJAW_SPEC,
    PIPERX_STATION_SPEC,
    B601_DM_SPEC,
    BIMANUAL_B601_DM_SPEC,
    B601_DM_STATION_SPEC,
)

_DEFINITIONS: Final[Mapping[EmbodimentName, EmbodimentDefinition]] = {
    spec.name: spec for spec in _ALL_SPECS
}


class DevelopmentReason(StrEnum):
    MISSING_AUTHORITATIVE_DESCRIPTION = "missing_authoritative_description"


@dataclass(frozen=True, slots=True)
class DevelopmentEmbodiment:
    spec: EmbodimentDefinition
    reason: DevelopmentReason


DEVELOPMENT_EMBODIMENTS: Final[Mapping[EmbodimentName, DevelopmentEmbodiment]] = {
    INSTA360_UMI_SPEC.name: DevelopmentEmbodiment(
        spec=INSTA360_UMI_SPEC,
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
            raise UnknownEmbodimentError(str(key))
        cached = self._cache.get(name)
        if cached is None:
            cached = embodiment_from_definition(definition)
            self._cache[name] = cached
        return cached

    def __iter__(self) -> Iterator[str]:
        return (str(name) for name in self._definitions)

    def __len__(self) -> int:
        return len(self._definitions)


embodiments: Final = EmbodimentRegistry(_DEFINITIONS)
