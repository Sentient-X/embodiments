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
from .ffw import FFW_BG2_SPEC
from .g1 import UNITREE_G1_SPEC
from .humanoid import SENTIENT_HUMANOID_SPEC
from .insta360 import INSTA360_UMI_SPEC
from .panda import FRANKA_SPEC, PANDA_OMRON_SPEC
from .piper import PIPER_SPEC
from .rby1 import RBY1_SPEC
from .sentient_rwh import SENTIENT_RWH_SPEC
from .so101 import BIMANUAL_SO101_SPEC, SO101_SPEC
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

development_embodiments: Final = EmbodimentRegistry(
    {name: entry.spec for name, entry in DEVELOPMENT_EMBODIMENTS.items()}
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
