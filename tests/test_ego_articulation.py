import numpy as np

from sx_embodiments._ego_articulation import ArticulationTargets


def _finger(x, angle):
    """The first joint translates the wrist; the last bends the distal segment."""
    tip = np.array([x + 0.06 + 0.03 * np.cos(angle), 0.03 * np.sin(angle), 0])
    sites = np.array([[x - 0.01, 0, 0], [x, 0, 0], [x + 0.06, 0, 0], tip])
    jac = np.zeros((4, 3, 2))
    jac[:, 0, 0] = 1
    jac[-1, :, 1] = [-0.03 * np.sin(angle), 0.03 * np.cos(angle), 0]
    return sites, jac


def _measurement(angles):
    mapped = np.array([0, 2, 3, 4])
    whole = np.full((len(angles), 21, 3), np.nan)
    for row, angle in enumerate(angles):
        if np.isfinite(angle):
            whole[row, mapped] = _finger(0.11, angle)[0]
    return mapped, whole


def test_distal_bend_and_absolute_tip_have_independent_analytic_jacobians():
    mapped, points = _measurement([0.2])
    ancestor = np.array([[True, False]] * 3 + [[True, True]])
    target = ArticulationTargets.from_landmarks(
        mapped, 1, points, np.array([0]), 15, ancestor, None, None
    )
    np.testing.assert_array_equal(target.controls, [False, True])
    xyz, jac = _finger(0.10, 0.9)
    residual, derivative = target.residuals(0, points[0], xyz, jac, None)
    np.testing.assert_allclose(residual[:3], 16 * (points[0, 4] - xyz[-1]))
    np.testing.assert_allclose(residual[3], 4 * 0.09 * (0.2 - 0.9))
    for dof in range(2):
        h = np.eye(2)[dof] * 1e-6
        samples = []
        for direction in (-1, 1):
            sites, site_jac = _finger(*(np.array([0.10, 0.9]) + direction * h))
            samples.append(target.residuals(0, points[0], sites, site_jac, None)[0])
        np.testing.assert_allclose(derivative[:, dof], -(samples[1] - samples[0]) / 2e-6, atol=1e-8)


def test_mimic_driver_shared_with_camera_is_protected_and_chain_jacobian_projects_once():
    mapped, points = _measurement([0.3])
    sites, jac = _finger(0.10, 0.9)
    # The optical link depends on a follower of the distal joint.
    ancestor = np.array([[1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 1, 0], [0, 0, 1]], dtype=bool)
    coupling = np.array([[1.0, 0.0], [0.0, 1.0], [0.0, -0.5]])
    target = ArticulationTargets.from_landmarks(
        mapped, 1, points, np.array([0]), 15, ancestor, 4, coupling
    )
    assert not target.controls.any()
    full = np.concatenate((jac, jac[..., 1:]), axis=-1)
    residual, independent = target.residuals(0, points[0], sites, full, coupling)
    _, raw = target.residuals(0, points[0], sites, full, None)
    assert residual.shape == (4,)
    np.testing.assert_allclose(independent, raw @ coupling)


def test_unobserved_triplets_and_missing_exposures_break_the_scalar_fit():
    timestamps = np.array([0, 1, 2, 3, 4, 8, 9, 10]) * 66666667
    ancestor = np.array([[True, False]] * 3 + [[True, True]])
    measured = []
    for future in (0.4, 1.7):
        mapped, points = _measurement([0.1, 0.2, 0.4, np.nan, 0.7, future, 0.2, future])
        target = ArticulationTargets.from_landmarks(
            mapped, 1, points, timestamps, 15, ancestor, None, None
        )
        measured.append(target.angles[:, 0])
        assert np.isnan(target.angles[3, 0])
        np.testing.assert_allclose(target.angles[4, 0], 0.7, atol=1e-12, rtol=0)
        sites, jac = _finger(0.1, 0.4)
        residual, derivative = target.residuals(3, points[3], sites, jac, None)
        assert residual.shape == (0,)
        assert derivative.shape == (0, 2)
    np.testing.assert_equal(measured[0][:5], measured[1][:5])

    targets = ArticulationTargets.from_landmarks(
        mapped[[0, 3]], 1, points, timestamps, 15, ancestor[[0, 3]], None, None
    )
    assert targets.sites.shape == (0, 3)
    assert not targets.controls.any()
