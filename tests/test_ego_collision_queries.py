import numpy as np
import pytest


def test_candidate_cad_omits_only_frozen_internal_pairs_and_keeps_full_gradient(tmp_path):
    pytest.importorskip("mujoco")
    from sx_embodiments._ego_collision import CollisionModel, collision_assets
    from sx_embodiments.ego_retarget import _model

    path = tmp_path / "carriage.urdf"
    links = ("base", "carriage", "digit")
    path.write_text(
        '<robot name="carriage">'
        + "".join(
            f'<link name="{name}"><collision><geometry>'
            '<sphere radius=".1"/></geometry></collision></link>'
            for name in links
        )
        + "".join(
            f'<joint name="slide{i}" type="prismatic"><parent link="{links[i]}"/>'
            f'<child link="{links[i+1]}"/><axis xyz="1 0 0"/>'
            '<limit lower="-2" upper="2" velocity="2" effort="5"/></joint>'
            for i in range(2)
        )
        + "</robot>"
    )
    kin = _model(path)
    geometry = collision_assets(path)
    full = CollisionModel.read(geometry, kin.names, kin.neutral, ())
    fresh = CollisionModel.read(geometry, kin.names, kin.neutral, ())
    assert full is not None and fresh is not None
    screening = full.moving_pairs(np.array([0], dtype=np.int64))
    assert screening is not full
    # Frozen 50 mm offset between digit and carriage. Moving carriage can
    # drive them both into the base; those two changing pairs stay enabled.
    clear = np.array([0.4, 0.05])
    near = np.array([0.04, 0.05])
    assert full.at(clear).unallowed == pytest.approx(0.15)
    assert screening.at(clear).unallowed == 0
    assert screening.at(near).unallowed == full.at(near).unallowed == pytest.approx(0.16)
    for row in (near, clear, near, near, clear):
        for only in (True, False):
            # Force the reference through a distinct pose for each evaluation.
            fresh.at(np.array([-0.9, -0.9]))
            fresh.at(np.array([0.9, 0.9]))
            oracle = fresh.at(row, gradient=True, penetration_only=only)
            got = full.at(row, gradient=True, penetration_only=only)
            assert got.worst_pair == oracle.worst_pair
            assert (got.maximum, got.excess, got.unallowed) == (
                oracle.maximum,
                oracle.excess,
                oracle.unallowed,
            )
            for name in ("violation", "jacobian", "blocked_joints"):
                np.testing.assert_array_equal(getattr(got, name), getattr(oracle, name))
    # Custom affinity rules and poses with independent coordinates affecting
    # every link pair always retain the original physical model.
    assert full.moving_pairs(np.array([0, 1], dtype=np.int64)) is full
    full.model.geom_conaffinity[0] = 5
    assert full.moving_pairs(np.array([0], dtype=np.int64)) is full
