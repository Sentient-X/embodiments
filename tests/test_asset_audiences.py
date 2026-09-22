"""Who may receive which asset bytes, derived from the declared licence and nothing else."""

import pytest
from sx_contracts.assets import AssetIntegrityError

from sx_embodiments.assets import AssetAudience, asset_root, audience
from sx_embodiments.known import asset_audiences


def test_licence_ref_prefix_is_the_entitlement_fact() -> None:
    assert audience("LicenseRef-Sentient-Proprietary") is AssetAudience.ENTITLED
    assert audience("LicenseRef-Anything-Else") is AssetAudience.ENTITLED
    for public in ("Apache-2.0", "MIT", "BSD-3-Clause", "Apache-2.0 AND BSD-3-Clause"):
        assert audience(public) is AssetAudience.PUBLIC


def test_empty_licence_is_refused_not_guessed() -> None:
    """An unclassifiable licence must never fall through to the permissive answer."""
    with pytest.raises(AssetIntegrityError):
        audience("   ")


def test_the_companys_own_robots_are_entitled_only() -> None:
    """The RWH and Yubi descriptions are the hardware design; they are never public.

    Pinned by name because this is the fact the publication path depends on. If a robot
    is added whose directory belongs on the entitled side, this test failing is the
    intended prompt to say so out loud.
    """
    entitled = {
        name for name, value in asset_audiences().items() if value is AssetAudience.ENTITLED
    }
    assert entitled == {"sentient_rwh", "yubi_description"}


def test_every_directory_on_disk_is_claimed_by_a_declaration() -> None:
    """ "Nobody declared it" must never be read as "it is public".

    `asset_audiences()` can only see directories some embodiment declares an asset in.
    A directory in the tree that no declaration claims would be invisible to the
    publication split, so it is a failure here rather than an unclassified upload.
    """
    classified = set(asset_audiences())
    on_disk = {entry.name for entry in asset_root().iterdir() if entry.is_dir()}
    assert on_disk - classified == set(), "unclassified asset directories"


def test_compound_licenses_keep_entitled_sources_private():
    assert audience("Apache-2.0 AND LicenseRef-Sentient-Proprietary") is AssetAudience.ENTITLED
