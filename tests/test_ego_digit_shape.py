from itertools import product

import numpy as np
import pytest

from sx_embodiments._ego_digit_shape import _path, bends
from sx_embodiments.ego_retarget import MappingProfile, refine_trajectory
from sx_embodiments.kinematics import Kinematics


def test_complete_digit_traces_measured_bends_with_a_fixed_wrist_and_real_contacts(tmp_path):
    # A non-Wuji, single-finger, planar robot with a translating wrist. Pinning the
    # wrist forces the two measured interior angles to be achieved by the linkage.
    links = ["base", "wrist", "knuckle", "middle", "distal", "tip"]
    joints = ["rail", "knuckle_flex", "middle_flex", "distal_flex", "pad"]
    types = ["prismatic", "revolute", "revolute", "revolute", "fixed"]
    parts = [
        f'<link name="{name}"><collision><geometry><sphere radius=".002"/>'
        "</geometry></collision></link>"
        for name in links
    ]
    for k, (name, kind) in enumerate(zip(joints, types, strict=True)):
        axis = "1 0 0" if kind == "prismatic" else "0 0 1"
        parts.append(
            f'<joint name="{name}" type="{kind}"><parent link="{links[k]}"/>'
            f'<child link="{links[k + 1]}"/><origin xyz=".06 0 0"/><axis xyz="{axis}"/>'
            '<limit lower="-1.4" upper="1.4" effort="1" velocity="4"/></joint>'
        )
    urdf = tmp_path / "one_digit.urdf"
    urdf.write_text('<robot name="one_digit">' + "".join(parts) + "</robot>")
    clock = np.r_[np.arange(6) * 70_000_000, 1_700_000_000 + np.arange(6) * 70_000_000]
    seed = np.tile([0.03, 0.5, 1.1, -0.6], (len(clock), 1))
    seed[:, 0] += np.linspace(0, 0.005, len(clock))
    targets = seed.copy()
    targets[:6, 1:] = [0.7, 0.25, 0.45]
    targets[6:, 1:] = [0.5, 0.65, 0.35]
    kinematics = Kinematics.of(urdf.read_bytes())
    points = np.zeros((len(clock), 21, 3))
    for i, q in enumerate(targets):
        for k, link in enumerate(links[1:]):
            transform = kinematics.pose("base", link, dict(zip(joints[:-1], q, strict=True)))
            points[i, k] = np.asarray(transform).reshape(4, 4)[:3, 3]
    points[4] = np.nan
    profile = MappingProfile({"left": tuple(links[1:] + [None] * 16)})
    options = {
        "time_ns": clock,
        "hands_world_m": {"left": points},
        "world_from_camera": np.tile(np.eye(4), (len(clock), 1, 1)),
        "robot_from_world": np.eye(4),
        "joint_names": tuple(joints[:-1]),
        "q": seed,
        "fps": 15.0,
    }
    result = refine_trajectory(
        urdf, profile, **options, digit_shape_limit_m=0.04, temporal_seed=True
    )
    np.testing.assert_array_equal(result.q[:, 0], seed[:, 0])
    np.testing.assert_array_equal(result.observed[:, 0], np.isfinite(points[:, 0, 0]))
    assert np.all(result.self_collision_checked)
    assert np.all(result.self_collision_unallowed_m <= 1e-6)
    achieved = result.landmark_positions_robot_m[:, 0, 1:5]
    err = np.abs(bends(achieved) - bends(points[:, 1:5]))
    assert np.nanmax(err) < np.deg2rad(3)
    assert np.nanmax(result.fingertip_error_m) < 0.01
    assert np.all(np.abs(np.diff(result.q, axis=0)) <= 4 * np.diff(clock)[:, None] * 1e-9 + 1e-6)
    with pytest.raises(ValueError, match="temporal_seed requires"):
        refine_trajectory(urdf, profile, **options, temporal_seed=True)


@pytest.mark.parametrize("n", [2, 3, 5])
def test_jerk_path_retains_its_velocity_history(n):
    options = 3
    rng = np.random.default_rng(n)
    clock = np.cumsum(np.r_[0.0, rng.uniform(0.06, 0.09, n - 1)])
    dt = np.diff(clock)
    poses = rng.normal(0, 0.04, (n, options, 2))
    poses[:, 0] = 0
    velocities = (poses[1:, None, :, :] - poses[:-1, :, None, :]) / dt[:, None, None, None]
    score = rng.uniform(0, 0.2, (n, options))
    bounds = np.array([1.4, 0.8])

    def cost(path):
        pos = poses[np.arange(n), path]
        velocity = np.diff(pos, axis=0) / dt[:, None]
        if np.any(np.abs(velocity) > bounds + 1e-6):
            return np.inf
        acceleration = np.diff(velocity, axis=0) / ((dt[1:] + dt[:-1])[:, None] / 2)
        jerk = np.diff(acceleration, axis=0) / ((clock[3:] - clock[:-3])[:, None] / 3)
        return (
            score[np.arange(n), path].sum()
            + 0.00005 * (acceleration**2).sum() / 2
            + 0.0000005 * (jerk**2).sum() / 2
        )

    actual = _path(score, velocities, bounds, dt, polish=True)
    optimal = min(cost(path) for path in product(range(options), repeat=n))
    assert cost(actual) == pytest.approx(optimal)
