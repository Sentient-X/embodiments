"""Pinned upstream origins shared by the packaged embodiment descriptions."""

from sx_contracts.assets import AssetProvenance

MENAGERIE_REVISION = "71f066ad0be9cd271f7ed58c030243ef157af9f4"


def menagerie(path: str, license_id: str) -> AssetProvenance:
    return AssetProvenance(
        repository="https://github.com/google-deepmind/mujoco_menagerie",
        revision=MENAGERIE_REVISION,
        path=path,
        license_id=license_id,
    )
