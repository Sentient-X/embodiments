"""Pinned upstream origins shared by the packaged embodiment descriptions."""

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
