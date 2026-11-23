"""Exercise whole-segment coupling, native clocks and admission evidence."""

import hashlib
import json
from dataclasses import replace

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


@pytest.mark.parametrize(("limit", "admitted"), [(1, False), (100, True), (None, False)])
def test_gravity_exceedance_and_unknown_axis_cannot_train(tmp_path, limit, admitted):
    path = tmp_path / "lift.urdf"
    effort = f' effort="{limit}"' if limit is not None else ""
    path.write_text(
        '<robot name="lift"><link name="base"/><link name="wrist"><inertial>'
        '<mass value="0.6"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>'
        '</inertial></link><joint name="lift" type="prismatic"><parent link="base"/>'
        '<child link="wrist"/><axis xyz="0 0 1"/>'
        f'<limit lower="-0.1" upper="0.1" velocity="1"{effort}/></joint></robot>'
    )
    times = np.arange(7, dtype=np.int64) * 100_000_000
    hands = np.zeros((len(times), 21, 3))
    hands[:, 4, 1] = 0.03
    fit, report = optimize_trajectory(
        path,
        MappingProfile({"right": ("wrist",) + (None,) * 20}),
        time_ns=times,
        hands_world_m={"right": hands},
        world_from_camera=np.tile(np.eye(4), (len(times), 1, 1)),
        robot_from_world=np.eye(4),
        joint_names=("lift",),
        q=np.zeros((len(times), 1)),
        fps=10,
    )
    assert report.effort_evaluated
    assert report.effort_model_available == (limit is not None)
    expected = 0.6 * (9.81 + np.diff(fit.q[:, 0], n=2) / 0.1**2)
    np.testing.assert_allclose(report.torque[1:-1, 0], expected, rtol=0.001)
    np.testing.assert_array_equal(report.training_effort_ok, [False] + [admitted] * 5 + [False])


def test_near_coincident_native_exposures_are_pinned_and_break_effort_stencils(tmp_path):
    source = tmp_path / "arm.urdf"
    source.write_text(DESCRIPTION)
    mapping = MappingProfile({"right": ("wrist",) + (None,) * 20})
    times = np.array([0, 100_000_000, 200_000_000, 200_000_001, 300_000_001, 400_000_001])
    hands = np.zeros((len(times), 21, 3))
    hands[:, 4, 1] = 0.02
    seed = np.zeros((len(times), 2))
    seed[:, 0] = [0.03, 0.07, 0.11, 0.11 + 1e-10, 0.08, 0.05]
    fit, _ = optimize_trajectory(
        source,
        mapping,
        time_ns=times,
        hands_world_m={"right": hands},
        world_from_camera=np.tile(np.eye(4), (len(times), 1, 1)),
        robot_from_world=np.eye(4),
        joint_names=("rail", "close"),
        q=seed,
        fps=10,
    )
    np.testing.assert_array_equal(fit.q[2:4].view(np.uint64), seed[2:4].view(np.uint64))
    assert not np.array_equal(fit.q[:2], seed[:2])


def test_head_follows_source_camera_direction_and_turn_rate(tmp_path):
    path = tmp_path / "neck.urdf"
    path.write_text(
        '<robot name="neck"><link name="base"/><link name="eye"/><link name="palm"/>'
        '<joint name="neck" type="revolute"><parent link="base"/><child link="eye"/>'
        '<axis xyz="1 0 0"/><limit effort="3" lower="-1" upper="1" velocity="3"/></joint>'
        '<joint name="palm_mount" type="fixed"><parent link="base"/><child link="palm"/>'
        "</joint></robot>"
    )
    n = 51
    times = 66_666_667 * np.arange(n, dtype=np.int64)
    angle = 0.3 * np.sin(1.8 * times * 1e-9)
    cameras = np.tile(np.eye(4), (n, 1, 1))
    cosine, sine = np.cos(angle), np.sin(angle)
    cameras[:, 1, 1] = cameras[:, 2, 2] = cosine
    cameras[:, 1, 2], cameras[:, 2, 1] = -sine, sine
    hand = np.zeros((n, 21, 3))
    hand[:, 4, 1] = 0.05
    profile = MappingProfile({"right": ("palm",) + (None,) * 20}, camera_optical_link="eye")
    args = {
        "time_ns": times,
        "hands_world_m": {"right": hand},
        "world_from_camera": cameras,
        "robot_from_world": np.eye(4),
        "joint_names": ("neck",),
        "q": np.zeros((n, 1)),
        "fps": 15,
    }
    observed = []
    fit, expected = optimize_trajectory(path, profile, on_checkpoint=observed.append, **args)
    assert len(observed) >= 2
    first, complete = observed[0], observed[-1]
    key = (first.first_index, first.stop_index, first.phase)
    assert first.stop is None and complete.stop is not None
    resumed, report = optimize_trajectory(path, profile, checkpoints={key: first}, **args)
    np.testing.assert_array_equal(resumed.q, fit.q)
    assert report.document() == expected.document()
    final, _ = optimize_trajectory(path, profile, checkpoints={key: complete}, **args)
    np.testing.assert_array_equal(final.q, fit.q)
    invalid = replace(first, q=first.q + 0.1)
    with pytest.raises(ValueError, match="checkpoint"):
        optimize_trajectory(path, profile, checkpoints={key: invalid}, **args)
    assert np.rad2deg(np.max(fit.camera_rotation_error_rad)) < 1
    assert np.rad2deg(np.max(np.abs(np.diff(fit.q[:, 0] - angle)) / (np.diff(times) * 1e-9))) < 3


def test_operating_speed_qualifies_the_actual_edges_and_both_endpoints(tmp_path):
    path = tmp_path / "slider.urdf"
    path.write_text(
        '<robot name="slider"><link name="base"/><link name="palm"/><link name="guide"><inertial>'
        '<mass value="0.1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>'
        '</inertial></link><joint name="palm_mount" type="fixed"><parent link="base"/>'
        '<child link="palm"/></joint><joint name="slide" type="prismatic"><parent link="base"/>'
        '<child link="guide"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" '
        'effort="100" velocity="5"/></joint></robot>'
    )
    limits = tmp_path / "slider.effort.json"
    limits.write_text(
        json.dumps(
            {
                "urdf_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "joints": [
                    {
                        "joint": "slide",
                        "maximum": 100,
                        "basis": "fixture rating",
                        "unit": "N",
                        "reflected_inertia_kg_m2": None,
                        "operating_velocity": {
                            "maximum": 0.1,
                            "unit": "m/s",
                            "basis": "fixture governor",
                        },
                    }
                ],
            }
        )
    )
    times = np.array([0, 100, 200, 300, 400, 1000, 1100], dtype=np.int64) * 1_000_000
    q = np.array([0, 0.005, 0.025, 0.025, 0.03, 0.95, 0.946])[:, None]
    hand = np.zeros((len(q), 21, 3))
    hand[:, 4, 1] = 0.05
    fit, report = optimize_trajectory(
        path,
        MappingProfile({"right": ("palm",) + (None,) * 20}),
        time_ns=times,
        hands_world_m={"right": hand},
        joint_names=("slide",),
        q=q,
        fps=10,
        effort_limits=limits,
        robot_from_world=np.eye(4),
        world_from_camera=np.tile(np.eye(4), (len(q), 1, 1)),
    )
    np.testing.assert_array_equal(fit.q, q)
    np.testing.assert_array_equal(report.speed_limits, [0.1])
    np.testing.assert_array_equal(report.speed_edge_ok, [True, False, True, True, False, True])
    np.testing.assert_array_equal(
        report.training_speed_ok, [True, False, False, True, True, True, True]
    )


def test_governed_alternate_retargets_the_remaining_axis_and_checks_full_objective(tmp_path):
    path = tmp_path / "coaxial.urdf"
    inertial = (
        '<inertial><mass value="0.2"/>'
        '<inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial>'
    )
    path.write_text(
        f'<robot name="redundant"><link name="base"/><link name="car">{inertial}</link>'
        f'<link name="palm">{inertial}</link>'
        '<joint name="slow" type="prismatic"><parent link="base"/><child link="car"/>'
        '<axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="100" velocity="3"/></joint>'
        '<joint name="trim" type="prismatic"><parent link="car"/><child link="palm"/>'
        '<axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="100" velocity="3"/></joint></robot>'
    )
    limits = tmp_path / "coaxial.effort.json"
    limits.write_text(
        json.dumps(
            {
                "urdf_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "joints": [
                    {
                        "joint": "slow",
                        "unit": "N",
                        "maximum": 100,
                        "basis": "fixture",
                        "reflected_inertia_kg_m2": None,
                        "operating_velocity": {
                            "maximum": 0.12,
                            "unit": "m/s",
                            "basis": "documented fixture governor",
                        },
                    }
                ],
            }
        )
    )
    n = 20
    times = 100_000_000 * np.arange(n, dtype=np.int64)
    displacement = 0.10 * np.sin(0.65 * np.arange(n))
    q = np.column_stack((displacement, -displacement))
    hand = np.zeros((n, 21, 3))
    hand[:, 4, 1] = 0.04
    states = []
    args = {
        "time_ns": times,
        "hands_world_m": {"right": hand},
        "joint_names": ("slow", "trim"),
        "q": q,
        "fps": 10,
        "robot_from_world": np.eye(4),
        "world_from_camera": np.tile(np.eye(4), (n, 1, 1)),
        "effort_limits": limits,
    }
    prof = MappingProfile({"right": ("palm",) + (None,) * 20})
    fit, rep = optimize_trajectory(path, prof, **args, on_checkpoint=states.append)
    s = rep.document()["segments"][0]
    assert {c.phase for c in states} == {"whole", "governed_start"}
    assert s["governed_start"]["projected_joint"] == "slow"
    assert s["final_objective"] <= s["objective"][-1] + 1.0e-9
    lattice = s["feasible_lattice"]
    assert lattice["finite_global_objective"] == pytest.approx(
        lattice["direct_objective"], abs=1e-8
    )
    assert rep.speed_edge_ok.all() and np.max(fit.wrist_error_m) < 0.005
    snapshots = {(c.first_index, c.stop_index, c.phase): c for c in states}
    again, second = optimize_trajectory(path, prof, **args, checkpoints=snapshots)
    np.testing.assert_array_equal(fit.q, again.q)
    assert second.document() == rep.document()
