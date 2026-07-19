"""The factory's reference 6-DOF arm description (the built-in `emb_sxd_arm` seed).

Asset-only for now: the data-factory seeds its built-in embodiment row from this file and
derives DOF by parsing it. A full `EmbodimentSpec` lands if a second consumer needs the
typed joint box.
"""

from typing import Final

from ..assets import AssetFormat, AssetProvenance, AssetRole, PackagedAsset

SXD_ARM_URDF: Final = PackagedAsset(
    relpath="sxd_arm/sxd_arm.urdf",
    sha256="899d251c3cce682f084afeedb37197f45b610ab7655655bc3e0c8bfbd27a17c2",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Sentient-X/sxd-data-factory",
        revision="dc70b0be0ee8ca4dbe5723f81aa3fc2dcf8f6fe7",
        path="backend/app/assets/sxd_arm.urdf",
        license_id="Apache-2.0",
    ),
    media_type="application/xml",
)
