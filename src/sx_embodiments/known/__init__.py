"""The canonical embodiment registry, one module per hardware family.

``EMBODIMENTS`` maps every canonical id to its :class:`~sx_embodiments.compose.EmbodimentSpec`.
``PIPER`` and ``PANDA_OMRON`` remain the flat kinematic :class:`Embodiment` constants
runtimes bind — now DERIVED from their specs via ``kinematic_view`` and pinned by
``tests/test_known.py`` so the derivation can never drift from the deployed values.
"""

from collections.abc import Mapping
from typing import Final

from ..compose import EmbodimentSpec, camera_names, flat_layout, kinematic_view
from ..errors import LayoutError, UnknownEmbodimentError
from ..identity import EmbodimentId
from ..kinematic import Embodiment
from ..layout import FlatLayout
from .das import DAS_UMI_V4_SPEC, QUEST_EGO_SPEC
from .panda import FRANKA_SPEC, LIBERO_PANDA_SPEC, PANDA_OMRON_SPEC
from .piper import PIPER_SPEC
from .so101 import BIMANUAL_SO101_SPEC
from .stations import (
    B601_DM_SPEC,
    B601_RS_SPEC,
    PIPERX_STATION_SPEC,
    SENTIENT_A1_SPEC,
    SENTIENT_A2_SPEC,
)
from .yubi import YUBI_DEPTH_SPEC, YUBI_MONO_SPEC, YUBI_WIDEJAW_SPEC

_ALL_SPECS: Final[tuple[EmbodimentSpec, ...]] = (
    PIPER_SPEC,
    PANDA_OMRON_SPEC,
    FRANKA_SPEC,
    LIBERO_PANDA_SPEC,
    BIMANUAL_SO101_SPEC,
    DAS_UMI_V4_SPEC,
    QUEST_EGO_SPEC,
    YUBI_MONO_SPEC,
    YUBI_DEPTH_SPEC,
    YUBI_WIDEJAW_SPEC,
    B601_DM_SPEC,
    B601_RS_SPEC,
    PIPERX_STATION_SPEC,
    SENTIENT_A1_SPEC,
    SENTIENT_A2_SPEC,
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


def validate_action_widths(
    embodiment_id: EmbodimentId, *, joint_dim: int, gripper_dim: int
) -> None:
    """Enforce the declared action widths where the registry declares them.

    Ids not in the registry — and registry entries whose layout is not yet declared (a
    body part without a captured description) — pass through: the registry cannot enforce
    a law it does not state. Registered, declared layouts enforce strictly (typed
    ``LayoutError``).
    """
    spec = EMBODIMENTS.get(embodiment_id)
    if spec is None or not spec.layout_declared():
        return
    flat_layout(spec).validate_widths(joint_dim=joint_dim, gripper_dim=gripper_dim)


def validate_camera_keys(embodiment_id: EmbodimentId, keys: tuple[str, ...]) -> None:
    """Enforce camera-key SUBSET semantics where the registry declares a camera set.

    Any key present must be in the embodiment's declared set; absent cameras are fine
    (partial capture is normal). Unknown ids and embodiments with no declared cameras
    pass through — enforcement is never fictional.
    """
    spec = EMBODIMENTS.get(embodiment_id)
    if spec is None:
        return
    declared = camera_names(spec)
    if not declared:
        return
    unknown = [key for key in keys if key not in declared]
    if unknown:
        raise LayoutError(
            str(embodiment_id),
            f"camera keys {unknown!r} are not in the declared set {declared!r}",
        )


PIPER: Final[Embodiment] = kinematic_view(PIPER_SPEC)
PANDA_OMRON: Final[Embodiment] = kinematic_view(PANDA_OMRON_SPEC)
