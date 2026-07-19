"""The canonical episode-ready and development registries, one module per family.

Episode-ready enumeration is total: every listed spec produces a complete manifest.
``PIPER`` and ``PANDA_OMRON`` remain the flat kinematic :class:`Embodiment` constants
runtimes bind — now DERIVED from their specs via ``kinematic_view`` and pinned by
``tests/test_known.py`` so the derivation can never drift from the deployed values.
"""

from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from typing import Final

from ..compose import EmbodimentSpec, flat_layout, kinematic_view
from ..errors import UnknownEmbodimentError
from ..identity import EmbodimentId
from ..kinematic import Embodiment
from ..layout import FlatLayout
from .aloha import ALOHA_SPEC
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

_ALL_SPECS: Final[tuple[EmbodimentSpec, ...]] = (
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
)

EPISODE_READY_EMBODIMENTS: Final[Mapping[EmbodimentId, EmbodimentSpec]] = {
    spec.embodiment_id: spec for spec in _ALL_SPECS
}


class DevelopmentReason(StrEnum):
    MISSING_AUTHORITATIVE_DESCRIPTION = "missing_authoritative_description"


@dataclass(frozen=True, slots=True)
class DevelopmentEmbodiment:
    spec: EmbodimentSpec
    reason: DevelopmentReason


DEVELOPMENT_EMBODIMENTS: Final[Mapping[EmbodimentId, DevelopmentEmbodiment]] = {
    INSTA360_UMI_SPEC.embodiment_id: DevelopmentEmbodiment(
        spec=INSTA360_UMI_SPEC,
        reason=DevelopmentReason.MISSING_AUTHORITATIVE_DESCRIPTION,
    )
}


def embodiment_spec(embodiment_id: EmbodimentId) -> EmbodimentSpec:
    """Resolve a registry entry, or fail closed."""
    spec = EPISODE_READY_EMBODIMENTS.get(embodiment_id)
    if spec is None:
        raise UnknownEmbodimentError(str(embodiment_id))
    return spec


def layout_for(embodiment_id: EmbodimentId) -> FlatLayout:
    """The declared flat-vector layout of a registered embodiment, or fail closed."""
    return flat_layout(embodiment_spec(embodiment_id))


PIPER: Final[Embodiment] = kinematic_view(PIPER_SPEC)
PANDA_OMRON: Final[Embodiment] = kinematic_view(PANDA_OMRON_SPEC)
