"""Shared physical pivots add identity-bearing sites without rewriting approved geometry."""

import hashlib
from dataclasses import replace

import pytest

from sx_embodiments import Embodiment
from sx_embodiments.embodiment import embodiment_from_definition
from sx_embodiments.kinematics import Kinematics
from sx_embodiments.known.humanoid import LEG_MODULES, SENTIENT_HUMANOID_SPEC
from sx_embodiments.known.static_upper_body import (
    STATIC_UPPER_BODY_LEFT_ARM_MODULE,
    STATIC_UPPER_BODY_RIGHT_ARM_MODULE,
    STATIC_UPPER_BODY_SPEC,
)


@pytest.mark.parametrize(
    ("full", "previous_id", "urdf_sha256"),
    [
        (
            True,
            "ca3fb7c3b69a4df643d21904d64b57c7d954c536dbb70660143d75133e53f317",
            "e7fd42a06794c5d3fce549e55630ea0a9194eee04411b7f06a9d7ed5ada87e67",
        ),
        (
            False,
            "c227a86c973e3e8672c0454d8b713bcf1f62bd50d6c509e8869ac30180f167df",
            "11fa1ead5302c9f99a2c7a9c83e6d4f193ffe6a93de742d093f75431f3153c76",
        ),
    ],
)
def test_only_sites_change_the_prior_body_identity(
    full: bool, previous_id: str, urdf_sha256: str
) -> None:
    body = embodiment_from_definition(SENTIENT_HUMANOID_SPEC if full else STATIC_UPPER_BODY_SPEC)
    added = {
        f"{side}_{pivot}"
        for side in ("left", "right")
        for pivot in (("shoulder", "hip", "knee", "ankle") if full else ("shoulder",))
    }
    prior = replace(body, sites=tuple(site for site in body.sites if site.name not in added))
    # Stronger than equal FK: removing only these sites restores the exact former
    # content identity, including all cameras, limits, assets and drive provenance.
    assert str(prior.id) == previous_id
    assert body.id != prior.id
    assert body.urdf_bytes == prior.urdf_bytes
    assert hashlib.sha256(body.urdf_bytes).hexdigest() == urdf_sha256
    assert Embodiment.from_json(body.to_json()).id == body.id
    assert Embodiment.from_json(prior.to_json()).id == prior.id


def test_module_owned_pivots_follow_shared_assemblies_at_nonzero_positions() -> None:
    arms = (STATIC_UPPER_BODY_LEFT_ARM_MODULE, STATIC_UPPER_BODY_RIGHT_ARM_MODULE)
    for spec in (SENTIENT_HUMANOID_SPEC, STATIC_UPPER_BODY_SPEC):
        body = embodiment_from_definition(spec)
        kin = Kinematics.of(body.urdf_bytes)
        root = next(iter(kin.links - kin.parent_joint.keys()))
        positions = dict.fromkeys(kin.independent, 0.13)
        for side, module in zip(("left", "right"), arms, strict=True):
            shoulder = next(site for site in module.sites if site.name == f"{side}_shoulder")
            assert body.site(shoulder.name) == shoulder
            for role, link in (
                ("shoulder", "shoulder_pitch_link"),
                ("elbow", "elbow_pitch_link"),
                ("wrist", "wrist_roll_link"),
            ):
                site = body.site(f"{side}_{role}")
                assert site.link == f"{side}_{link}"
                assert site.transform(kin, root, positions) == pytest.approx(
                    kin.pose(root, site.link, positions), abs=1e-12
                )
            # A pivot is at its own axis origin: changing shoulder pitch rotates
            # its frame but cannot move the shoulder site along an arm segment.
            before = shoulder.transform(kin, root, positions)
            after = shoulder.transform(
                kin, root, {**positions, f"{side}_shoulder_pitch_joint": 0.37}
            )
            assert tuple(before[i] for i in (3, 7, 11)) == pytest.approx(
                tuple(after[i] for i in (3, 7, 11)), abs=1e-12
            )
    full = embodiment_from_definition(SENTIENT_HUMANOID_SPEC)
    kin = Kinematics.of(full.urdf_bytes)
    root = next(iter(kin.links - kin.parent_joint.keys()))
    positions = dict.fromkeys(kin.independent, 0.13)
    for side, module in zip(("right", "left"), LEG_MODULES, strict=True):
        for role, link in (
            ("hip", "hip_pitch_link"),
            ("knee", "knee_link"),
            ("ankle", "foot2_link"),
        ):
            site = full.site(f"{side}_{role}")
            assert site in module.sites
            assert site.link == f"{side}_{link}"
            assert site.transform(kin, root, positions) == pytest.approx(
                kin.pose(root, site.link, positions), abs=1e-12
            )
