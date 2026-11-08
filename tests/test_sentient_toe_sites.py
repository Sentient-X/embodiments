"""Toe references are literal CAD edge midpoints, not invented joints or sole frames."""

import hashlib
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from sx_embodiments.embodiment import embodiment_from_definition
from sx_embodiments.kinematics import Kinematics, compose, from_xyz_rpy
from sx_embodiments.known.humanoid import SENTIENT_HUMANOID_SPEC


@pytest.mark.parametrize(
    ("side", "mesh_sha256"),
    [
        ("left", "c18c804235e6637500515b134aa0ecd24ee056dba422ebea8c427b72b65e1e22"),
        ("right", "b6f1260b19e0abe97ca4248ad827753e500327357378c369c7d18db5b27e8130"),
    ],
)
def test_toe_reference_is_the_midpoint_of_a_literal_front_bottom_edge(
    side: str, mesh_sha256: str
) -> None:
    body = embodiment_from_definition(SENTIENT_HUMANOID_SPEC)
    site = body.site(f"{side}_toe_edge")
    path = Path(__file__).resolve().parents[1] / f"assets/humanoid_pkg/meshes/{side}_foot2_link.stl"
    raw = path.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == mesh_sha256
    triangles = np.frombuffer(
        raw, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), offset=84
    )["v"].astype(np.float64)
    point = np.asarray(site.pose.xyz)
    # The reviewed front is +X. Find an actual source triangle edge at that
    # forefoot extremum, whose midpoint is the authored point and whose length
    # is 40 mm. This independently checks membership and both lateral endpoints.
    front_x = triangles[:, :, 0].max()
    edges = np.concatenate([triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]])
    front = edges[np.all(np.abs(edges[:, :, 0] - front_x) < 1e-9, axis=1)]
    matching = front[np.linalg.norm(front.mean(axis=1) - point, axis=1) < 1e-12]
    assert len(matching) > 0
    assert np.linalg.norm(matching[0, 1] - matching[0, 0]) == pytest.approx(0.04, abs=1e-8)
    assert point[2] < -0.049  # bottom edge, not the top of the plate
    assert site.link == f"{side}_foot2_link"
    assert site.pose.rpy == (0.0, 0.0, 0.0)
    kin = Kinematics.of(body.urdf_bytes)
    positions = dict.fromkeys(kin.independent, 0.13)
    assert site.transform(kin, "base_link", positions) == pytest.approx(
        compose(
            kin.pose("base_link", site.link, positions), from_xyz_rpy(site.pose.xyz, site.pose.rpy)
        ),
        abs=1e-12,
    )


def test_toe_sites_change_only_body_site_facts_and_preserve_prior_identity() -> None:
    body = embodiment_from_definition(SENTIENT_HUMANOID_SPEC)
    prior = replace(body, sites=tuple(s for s in body.sites if not s.name.endswith("_toe_edge")))
    assert str(prior.id) == "eb249c00dcff12dc212db1e6122df0ea4f13d57f17fc90d4c4a8855051e036f5"
    assert body.id != prior.id
    assert body.urdf_bytes == prior.urdf_bytes
    assert hashlib.sha256(body.urdf_bytes).hexdigest() == (
        "e7fd42a06794c5d3fce549e55630ea0a9194eee04411b7f06a9d7ed5ada87e67"
    )
