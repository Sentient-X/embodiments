"""Exercise FK of a different embodiment, sparse observations, and physical joint bounds."""

import json
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")  # optional numerical projection
from sx_embodiments.ego_retarget import MappingProfile, estimate_registration, solve
from sx_embodiments.kinematics import Kinematics

DESCRIPTION = """<robot name="fixture">
  <link name="base"/><link name="wrist"/><link name="jaw"/><link name="opposite"/>
  <link name="eye"/>
  <joint type="fixed" name="optics"><parent link="base"/><child link="eye"/>
    <origin xyz="0 0 0.5"/></joint>
  <joint type="prismatic" name="rail">
    <parent link="base"/><child link="wrist"/><axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="0.5"/></joint>
  <joint type="prismatic" name="close">
    <parent link="wrist"/><child link="jaw"/><axis xyz="0 1 0"/>
    <limit lower="-0.1" upper="0.1" effort="1" velocity="0.3"/></joint>
  <joint type="prismatic" name="follower">
    <parent link="wrist"/><child link="opposite"/><axis xyz="0 1 0"/>
    <limit lower="-0.05" upper="0.05" effort="1" velocity="0.1"/>
    <mimic joint="close" multiplier="-1"/></joint>
</robot>"""


@pytest.fixture
def setup(tmp_path: Path):
    urdf = tmp_path / "arm.urdf"
    urdf.write_text(DESCRIPTION)
    mapping = [None] * 21
    mapping[0], mapping[4], mapping[8] = "wrist", "jaw", "opposite"
    config = tmp_path / "mapping.json"
    config.write_text(json.dumps({"hands": {"right": mapping}, "camera_optical_link": "eye"}))
    return urdf, MappingProfile.read(config)


def test_mimic_limits_missing_samples_and_fk_at_final_solution(setup, tmp_path):
    urdf, profile = setup
    times = np.array([0, 10_000_000, 45_000_000, 60_000_000], dtype=np.int64)
    cameras = np.tile(np.eye(4), (len(times), 1, 1))
    fingers = np.zeros((len(times), 21, 3))
    fingers[:, 4, 1], fingers[:, 8, 1] = 0.02, -0.02
    fingers[1:, 4, 1], fingers[1:, 8, 1] = 0.09, -0.09
    fingers[1:, :, 0] = 0.3
    fingers[2] = np.nan
    registration = np.eye(4)
    registration[1, 3] = -0.2
    fingers[..., 1] += 0.2
    fit = solve(
        urdf,
        profile,
        time_ns=times,
        hands_world_m={"right": fingers},
        world_from_camera=cameras,
        robot_from_world=registration,
    )
    assert fit.joint_names == ("rail", "close")
    assert fit.observed[:, 0].tolist() == [True, True, False, True]
    assert 0.019 < fit.q[0, 1] < 0.021
    assert np.all(fit.q[:, 1] <= 0.05)
    velocity = abs(np.diff(fit.q, axis=0)) / (np.diff(times)[:, None] * 1e-9)
    assert np.all(velocity <= np.array([0.5, 0.1]) + 1e-10)
    assert np.isnan(fit.fingertip_error_m[2]).all()
    assert np.isnan(fit.fingertip_error_m[:, :, 2:]).all()
    independent = Kinematics.of(urdf.read_bytes())
    for i, row in enumerate(fit.q):
        q = dict(zip(fit.joint_names, row, strict=True))
        for landmark, link in [(0, "wrist"), (4, "jaw"), (8, "opposite")]:
            fk = np.array(independent.pose("base", link, q)).reshape(4, 4)[:3, 3]
            np.testing.assert_allclose(fit.landmark_positions_robot_m[i, 0, landmark], fk)
            if fit.observed[i, 0]:
                target = fingers[i, landmark] + registration[:3, 3]
                np.testing.assert_allclose(
                    fit.landmark_error_m[i, 0, landmark], np.linalg.norm(fk - target)
                )
    fit.save(tmp_path / "fit.npz")
    with np.load(tmp_path / "fit.npz") as saved:
        np.testing.assert_array_equal(saved["q"], fit.q)
        np.testing.assert_array_equal(saved["time_ns"], times)


def test_virtual_registration_commutes_with_source_translation(setup):
    urdf, profile = setup
    times = np.array([0, 1_000_000_000], dtype=np.int64)
    cameras = np.tile(np.eye(4), (len(times), 1, 1))
    fingers = np.zeros((len(times), 21, 3))
    fingers[:, 4, 1] = 0.02
    kwargs = {"time_ns": times, "world_from_camera": cameras}
    first = estimate_registration(urdf, profile, hands_world_m={"right": fingers}, **kwargs)
    shift = np.array([2.0, -3.0, 0.5])
    cameras[:, :3, 3] += shift
    second = estimate_registration(
        urdf, profile, hands_world_m={"right": fingers + shift}, **kwargs
    )
    np.testing.assert_allclose(second[:3, :3], first[:3, :3])
    np.testing.assert_allclose(second[:3, 3] + second[:3, :3] @ shift, first[:3, 3], atol=1e-12)
    with pytest.raises(ValueError, match="observed wrist"):
        estimate_registration(urdf, profile, hands_world_m={"right": fingers * np.nan}, **kwargs)
