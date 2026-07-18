"""The factory's reference 6-DOF arm description (the built-in `emb_sxd_arm` seed).

Asset-only for now: the data-factory seeds its built-in embodiment row from this file and
derives DOF by parsing it. A full `EmbodimentSpec` lands if a second consumer needs the
typed joint box.
"""

from typing import Final

from ..assets import AssetFormat, AssetRole, PackagedAsset

SXD_ARM_URDF: Final = PackagedAsset(
    relpath="sxd_arm/sxd_arm.urdf",
    sha256="899d251c3cce682f084afeedb37197f45b610ab7655655bc3e0c8bfbd27a17c2",
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    media_type="application/xml",
)
