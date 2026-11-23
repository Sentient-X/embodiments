"""Exercise whole-segment coupling, native clocks and admission evidence."""

import json

import numpy as np
import pytest

pytest.importorskip("mujoco")

from sx_embodiments._ego_motion import _apply, _derivatives, _Normal
from sx_embodiments.ego_retarget import MappingProfile, optimize_trajectory
from test_ego_retarget import DESCRIPTION


def test_native_divided_differences_and_banded_coupling():
    t = np.array([0, 0.064, 0.132, 0.195, 0.263, 0.331])
    v, mid, acceleration, jerk = _derivatives(t)
    np.testing.assert_allclose(_apply((t**2)[:, None], mid)[:, 0], 2 * t[1:-1])
    np.testing.assert_allclose(_apply((t**2)[:, None], acceleration), 2)
    np.testing.assert_allclose(_apply((t**3)[:, None], jerk), 6)
    n, m = len(t), 2
    gen = np.random.default_rng(19)
    q = gen.normal(size=(n, m))
    j = gen.normal(size=(n, 4, m))
    r = gen.normal(size=(n, 4))
    normal = _Normal(n, m)
    normal.spatial(r, j)
    a = np.zeros((n, 4, n, m))
    for i in range(n):
        a[i, :, i, :] = j[i]
    dense, target = [a.reshape(-1, n * m)], [r.ravel()]
    for coefficients in (v, acceleration, jerk):
        duration = gen.uniform(0.001, 0.005, size=(len(coefficients), m))
        residual = _apply(q, coefficients) * duration
        normal.temporal(residual, coefficients, duration)
        a = np.zeros((len(coefficients), m, n, m))
        for row, coeffs in enumerate(coefficients):
            for k, coeff in enumerate(coeffs):
                a[row, :, row + k, :] = np.diag(coeff * duration[row])
        dense.append(a.reshape(-1, n * m))
        target.append(residual.ravel())
    damping = np.array([0.9, 0.3])
    mat = np.concatenate(dense)
    expected = np.linalg.solve(
        mat.T @ mat + np.diag(np.tile(damping, n) ** 2), -mat.T @ np.concatenate(target)
    )
    step, residual = normal.step(damping)
    np.testing.assert_allclose(step.ravel(), expected, atol=1e-9)
    assert residual < 1e-10


@pytest.mark.parametrize("with_inertia", [True, False])
def test_public_sequence_solver_preserves_digits_and_native_evidence(tmp_path, with_inertia):
    description = DESCRIPTION
    if with_inertia:
        inertial = (
            '<inertial><mass value="0.2"/><inertia ixx="0.001" iyy="0.001" izz="0.001" '
            'ixy="0" ixz="0" iyz="0"/></inertial>'
        )
        for name in ("wrist", "jaw", "opposite"):
            description = description.replace(
                f'<link name="{name}"/>', f'<link name="{name}">{inertial}</link>'
            )
    path = tmp_path / "arm.urdf"
    path.write_text(description)
    mapping = [None] * 21
    mapping[0], mapping[4], mapping[8] = "wrist", "jaw", "opposite"
    profile = MappingProfile({"right": tuple(mapping)}, camera_optical_link="eye")
    n = 32
    times = 100_000_000 * np.arange(n, dtype=np.int64)
    times[16:] += 5_000_000_000
    target = np.repeat([0.0, 0.7], 16)
    seed = np.column_stack((target + np.tile([-0.02, 0.02], 16), np.full(n, 0.02)))
    hands = np.zeros((n, 21, 3))
    hands[:, :, 0] = target[:, None]
    hands[:, 4, 1] = 0.02
    hands[:, 8, 1] = -0.02
    hands[8] = np.nan
    fit, motion = optimize_trajectory(
        path,
        profile,
        time_ns=times,
        hands_world_m={"right": hands},
        world_from_camera=np.tile(np.eye(4), (n, 1, 1)),
        robot_from_world=np.eye(4),
        fps=10,
        joint_names=("rail", "close"),
        q=seed,
    )
    np.testing.assert_array_equal(fit.q[:, 1], seed[:, 1])
    np.testing.assert_array_equal(fit.time_ns, times)
    assert not fit.observed[8, 0]
    assert np.isnan(fit.wrist_error_m[8, 0])
    assert np.median(abs(fit.q[:, 0] - target)) < 0.003
    assert np.max(fit.q[:16, 0]) < 0.02 and np.min(fit.q[16:, 0]) > 0.68
    assert motion.document()["continuous_segments"] == 2
    assert motion.effort_model_available == with_inertia
    if with_inertia:
        has_stencil = np.ones(n, dtype=bool)
        has_stencil[[0, 15, 16, 31]] = False
        np.testing.assert_array_equal(np.isfinite(motion.torque).all(axis=1), has_stencil)
        np.testing.assert_array_equal(motion.training_effort_ok, has_stencil)
        assert np.nanmax(abs(motion.torque)) < np.nanmax(abs(motion.torque_before)) / 10
    else:
        assert np.isnan(motion.torque).all()
        assert not motion.training_effort_ok.any()
        assert "inertia" in motion.document()["effort_model_issue"]


def test_semantic_arm_profile_roundtrip_and_validation(tmp_path):
    file = tmp_path / "profile.json"
    profile = MappingProfile.sentient_wuji()
    file.write_text(json.dumps(profile.document()))
    assert MappingProfile.read(file) == profile
    legacy = MappingProfile({"right": ("wrist",) + (None,) * 20})
    assert "arm_links" not in legacy.document()
    file.write_text(json.dumps(legacy.document()))
    assert MappingProfile.read(file) == legacy
    for malformed in (
        ["shoulder", "elbow", "wrist"],
        {"right": ["shoulder", "elbow"]},
        {"right": ["shoulder", 2, "wrist"]},
        {"right": ["shoulder", "elbow", "elbow"]},
        {"right": ["shoulder", "", "wrist"]},
        {"left": ["shoulder", "elbow", "wrist"]},
    ):
        file.write_text(json.dumps({**legacy.document(), "arm_links": malformed}))
        with pytest.raises(ValueError, match="arm_links"):
            MappingProfile.read(file)
