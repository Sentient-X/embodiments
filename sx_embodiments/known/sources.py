"""Pinned upstream origins shared by the packaged embodiment descriptions."""

from collections.abc import Mapping
from typing import Final

from sx_contracts.assets import AssetProvenance

from ..parts import FactSource

MENAGERIE_REVISION = "71f066ad0be9cd271f7ed58c030243ef157af9f4"

AI_WORKER_REVISION = "e02c883f57fed84e06d0be6728334036cb362acf"


def capture_source(path: str) -> FactSource:
    """Capture implementation evidence for the documented equipment inventory."""
    return FactSource(
        repository="https://github.com/Sentient-X/sx",
        revision="1fbcf3b24bffa05b983c958ec28e81a5a9da08ee",
        path=path,
    )


def menagerie(path: str, license_id: str) -> AssetProvenance:
    return AssetProvenance(
        repository="https://github.com/google-deepmind/mujoco_menagerie",
        revision=MENAGERIE_REVISION,
        path=path,
        license_id=license_id,
    )


def ai_worker(path: str, generator: str | None = None) -> AssetProvenance:
    return AssetProvenance(
        repository="https://github.com/ROBOTIS-GIT/ai_worker",
        revision=AI_WORKER_REVISION,
        path=path,
        license_id="Apache-2.0",
        generator=generator,
    )


VENDORED_LICENCES: Final[Mapping[str, str]] = {
    # The upstream i2rt YAM model, vendored with menagerie at MENAGERIE_REVISION; no
    # registered embodiment references it yet. Its own LICENSE file is MIT.
    "menagerie/i2rt_yam": "MIT",
    # This repository's own record of MENAGERIE_REVISION, under the package's licence.
    "menagerie/menagerie.commit": "Apache-2.0",
}
"""Licences of tree files that no embodiment declaration reaches, by file or directory.

The manifest reads a file's licence from its declaration; a vendored file nothing
references has none, and inheriting one from an unrelated sibling would be a guess.
"""
