"""The canonical embodiment registry, one module per hardware family.

``EMBODIMENTS`` maps every canonical id to its :class:`~sx_embodiments.compose.EmbodimentSpec`.
``PIPER`` and ``PANDA_OMRON`` remain the flat kinematic :class:`Embodiment` constants
runtimes bind — now DERIVED from their specs via ``kinematic_view`` and pinned by
``tests/test_known.py`` so the derivation can never drift from the deployed values.
"""

from collections.abc import Mapping
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
from .panda import FRANKA_SPEC, LIBERO_PANDA_SPEC, PANDA_OMRON_SPEC
from .piper import PIPER_SPEC
from .rby1 import RBY1_SPEC
from .so101 import BIMANUAL_SO101_SPEC, SO101_SPEC
from .stations import PIPERX_STATION_SPEC
from .universal_robots import UR5E_SPEC, UR10E_SPEC
from .yor import YOR_SPEC
from .yubi import YUBI_DEPTH_SPEC, YUBI_MONO_SPEC, YUBI_WIDEJAW_SPEC

_ALL_SPECS: Final[tuple[EmbodimentSpec, ...]] = (
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
    LIBERO_PANDA_SPEC,
    SO101_SPEC,
    BIMANUAL_SO101_SPEC,
    DAS_UMI_V4_SPEC,
    QUEST_EGO_SPEC,
    INSTA360_UMI_SPEC,
    YUBI_MONO_SPEC,
    YUBI_DEPTH_SPEC,
    YUBI_WIDEJAW_SPEC,
    PIPERX_STATION_SPEC,
)

EMBODIMENTS: Final[Mapping[EmbodimentId, EmbodimentSpec]] = {
    spec.embodiment_id: spec for spec in _ALL_SPECS
}


def embodiment_spec(embodiment_id: EmbodimentId) -> EmbodimentSpec:
    """Resolve a registry entry, or fail closed."""
    spec = EMBODIMENTS.get(embodiment_id)
    if spec is None:
        raise UnknownEmbodimentError(str(embodiment_id))
    return spec


def layout_for(embodiment_id: EmbodimentId) -> FlatLayout:
    """The declared flat-vector layout of a registered embodiment, or fail closed."""
    return flat_layout(embodiment_spec(embodiment_id))


PIPER: Final[Embodiment] = kinematic_view(PIPER_SPEC)
PANDA_OMRON: Final[Embodiment] = kinematic_view(PANDA_OMRON_SPEC)
