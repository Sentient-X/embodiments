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
    clock = np.arange(12, dtype=np.int64) * 70_000_000
    seed = np.tile([0.03, 0.5, 1.1, -0.6], (len(clock), 1))
    seed[:, 0] += np.linspace(0, 0.005, len(clock))
    targets = seed.copy()
    targets[:6, 1:] = [0.7, 0.25, 0.45]
    targets[6:, 1:] = [0.7, 0.25, 0.45]
    kinematics = Kinematics.of(urdf.read_bytes())
    points = np.zeros((len(clock), 21, 3))
    for i, q in enumerate(targets):
        for k, link in enumerate(links[1:]):
            transform = kinematics.pose("base", link, dict(zip(joints[:-1], q, strict=True)))
            points[i, k] = np.asarray(transform).reshape(4, 4)[:3, 3]
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

    # Reappearing fingertips must not revise earlier observed or missing targets.
    missing = points.copy()
    missing[4:8] = np.nan

    def occluded(source):
        opts = {
            **options,
            "time_ns": np.arange(len(clock), dtype=np.int64) * 70_000_000,
            "hands_world_m": {"left": source},
        }
        return refine_trajectory(
            urdf, profile, **opts, digit_shape_limit_m=0.04, temporal_seed=True
        )

    first = occluded(missing)
    missing[8:] = points[:4]
    second = occluded(missing)
    np.testing.assert_array_equal(first.q[:8], second.q[:8])
    np.testing.assert_array_equal(first.q[4:8], seed[4:8])
    assert not first.observed[4:8].any()

    cut_clock = clock.copy()
    cut_clock[6:] += 200_000_000
    changed = points.copy()
    changed[:6] = points[-6:] + np.array([0.015, 0.010, 0])

    def separated(source):
        return refine_trajectory(
            urdf,
            profile,
            **{**options, "time_ns": cut_clock, "hands_world_m": {"left": source}},
            digit_shape_limit_m=0.04,
            temporal_seed=True,
        )

    cut, changed_cut = separated(points), separated(changed)
    np.testing.assert_array_equal(cut.q[5:7], seed[5:7])
    np.testing.assert_array_equal(cut.q[6:], changed_cut.q[6:])

    sparse = MappingProfile({"left": tuple(["wrist", None, None, None, "tip"] + [None] * 16)})
    bad = seed.copy()
    bad[1, 0] += 0.5
    with pytest.raises(ValueError, match="velocity limits"):
        refine_trajectory(
            urdf, sparse, **{**options, "q": bad}, digit_shape_limit_m=0.04, temporal_seed=True
        )


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


def test_occluded_left_digit_does_not_disable_a_visible_right_digit_or_drive_auxiliary_mimic(
    tmp_path,
):
    # Disjoint side chains, plus an unobserved auxiliary joint driven by the
    # first left finger hinge. Its leader must stay fixed even with a fingertip target.
    def link(name):
        return (
            f'<link name="{name}"><collision><geometry><sphere radius=".002"/>'
            "</geometry></collision></link>"
        )

    def joint(name, kind, parent, child, origin, mimic=""):
        return (
            f'<joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{origin}"/><axis xyz="0 0 1"/><limit lower="-1.4" upper="1.4" '
            f'effort="1" velocity="8"/>{mimic}</joint>'
        )

    xml = [link("root")]
    hands = {}
    for sign, side in [(-1, "left"), (1, "right")]:
        names = [f"{side}_{k}" for k in range(5)]
        hands[side] = tuple(names + [None] * 16)
        for k, name in enumerate(names):
            xml.append(link(name))
            xml.append(
                joint(
                    name + "_j",
                    "revolute" if 0 < k < 4 else "fixed",
                    names[k - 1] if k else "root",
                    name,
                    ".06 0 0" if k else f"0 {sign * 0.4} 0",
                )
            )
    xml += [
        link("aux"),
        joint(
            "coupled",
            "revolute",
            "root",
            "aux",
            "0 1 0",
            '<mimic joint="left_1_j" multiplier="-.5"/>',
        ),
    ]
    path = tmp_path / "bimanual.urdf"
    path.write_text('<robot name="bimanual">' + "".join(xml) + "</robot>")
    kin = Kinematics.of(path.read_bytes())
    names = tuple(side + f"_{j}_j" for side in hands for j in (1, 2, 3))
    q = np.tile([0.3, 0.9, 0.1, 0.3, 0.9, 0.1], (13, 1))
    t = np.arange(len(q), dtype=np.int64) * 100_000_000
    target = dict(zip(names, [0.45, 0.25, 0.4, 0.45, 0.25, 0.4], strict=True))
    pts = {}
    for side, links in hands.items():
        a = np.zeros((len(t), 21, 3))
        for j, name in enumerate(links[:5]):
            a[:, j] = np.asarray(kin.pose("root", name, target)).reshape(4, 4)[:3, 3]
        pts[side] = a
    pts["left"][3:9] = np.nan
    fit = refine_trajectory(
        path,
        MappingProfile(hands),
        time_ns=t,
        hands_world_m=pts,
        world_from_camera=np.tile(np.eye(4), (len(t), 1, 1)),
        robot_from_world=np.eye(4),
        joint_names=names,
        q=q,
        fps=10.0,
        digit_shape_limit_m=0.04,
        temporal_seed=True,
    )
    np.testing.assert_array_equal(fit.q[3:9, :3], q[3:9, :3])
    np.testing.assert_array_equal(fit.q[:, 0], q[:, 0])
    assert np.all(np.max(abs(fit.q[3:9, 3:] - q[3:9, 3:]), axis=1) > 0.1)
