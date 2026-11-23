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
    assert not np.any(fit.self_collision_checked)
    assert np.isnan(fit.self_collision_unallowed_m).all()
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


def test_checked_clearance_blocks_collision_and_keeps_tracking_error(tmp_path):
    description = """<robot name="slider">
      <link name="base"><collision><origin xyz="0.3 0 0"/>
        <geometry><sphere radius="0.065"/></geometry></collision></link>
      <link name="jaw"><collision><geometry><sphere radius="0.065"/></geometry></collision></link>
      <joint name="rail" type="prismatic">
        <parent link="base"/><child link="jaw"/><axis xyz="1 0 0"/>
        <limit lower="0" upper="0.4" effort="1" velocity="3"/></joint>
    </robot>"""
    path = tmp_path / "slide.urdf"
    path.write_text(description)
    times = np.array([0, 90_000_000, 190_000_000, 300_000_000], dtype=np.int64)
    hands = np.zeros((4, 21, 3))
    hands[:, :, 0] = np.array([0.0, 0.13, 0.23, 0.27])[:, None]
    hands[:, 4, 1] = 0.01
    profile = MappingProfile({"right": ("jaw",) + (None,) * 20})
    fit = solve(
        path,
        profile,
        time_ns=times,
        hands_world_m={"right": hands},
        world_from_camera=np.tile(np.eye(4), (4, 1, 1)),
        robot_from_world=np.eye(4),
    )
    assert fit.self_collision_checked.all()
    assert np.max(fit.self_collision_unallowed_m) <= 1e-6
    assert 0.12 < fit.q[-1, 0] <= 0.171
    assert fit.wrist_error_m[-1, 0] >= 0.099

    permitted = MappingProfile(profile.hands, allowed_collision_pairs=(("base", "jaw"),))
    allowed = solve(
        path,
        permitted,
        time_ns=times,
        hands_world_m={"right": hands},
        world_from_camera=np.tile(np.eye(4), (4, 1, 1)),
        robot_from_world=np.eye(4),
    )
    assert allowed.self_collision_max_m[-1] > 0.09
    assert allowed.self_collision_excess_m[-1] > 0.09
    assert allowed.self_collision_unallowed_m[-1] == 0.0


def test_collision_identity_changes_with_mesh_contents(tmp_path):
    from sx_embodiments.ego_retarget import collision_description_sha256

    path = tmp_path / "arm.urdf"
    path.write_text(
        '<robot name="arm"><link name="p"><collision><geometry>'
        '<mesh filename="pad.stl"/></geometry></collision></link></robot>'
    )
    mesh = tmp_path / "pad.stl"
    mesh.write_bytes(b"first mesh")
    before = collision_description_sha256(path)
    mesh.write_bytes(b"second mesh")
    assert before != collision_description_sha256(path)


def test_moving_parent_and_child_contact_is_visible_and_bearing_depth_is_bounded(tmp_path):
    from sx_embodiments._ego_collision import CollisionModel, collision_assets
    from sx_embodiments.ego_retarget import _model

    path = tmp_path / "parent-collision.urdf"
    description = """<robot name="probe">
      <link name="root"><collision><origin xyz="-3 0 0"/>
        <geometry><sphere radius=".10"/></geometry></collision></link>
      <link name="arm"><collision><origin xyz="0 0 0"/>
        <geometry><sphere radius=".10"/></geometry></collision></link>
      <link name="hand"><collision><origin xyz=".05 0 0"/>
        <geometry><sphere radius=".10"/></geometry></collision></link>
      <joint name="j1" type="prismatic"><parent link="root"/><child link="arm"/>
        <axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
      <joint name="j2" type="prismatic"><parent link="arm"/><child link="hand"/>
        <axis xyz="1 0 0"/><limit lower="-1" upper="1" velocity="1" effort="1"/></joint>
    </robot>"""
    path.write_text(description)
    robot = _model(path)
    default = CollisionModel.read(collision_assets(path), robot.names, robot.neutral, ())
    assert default is not None
    contacts = default.at(robot.neutral)
    assert contacts.unallowed == pytest.approx(0.150)
    assert contacts.maximum == pytest.approx(0.150)
    assert contacts.worst_pair == ("arm", "hand")
    with pytest.raises(ValueError, match="rotary bearing"):
        CollisionModel.read(
            collision_assets(path), robot.names, robot.neutral, (), (("arm", "hand", 0.16),)
        )

    # A bounded bearing allowance can admit the nested neutral housings, but a
    # rotation which drives them deeper must still report the actual absolute depth.
    description = description.replace('name="j2" type="prismatic"', 'name="j2" type="revolute"')
    description = description.replace('<axis xyz="1 0 0"/>', '<axis xyz="0 0 1"/>')
    description = description.replace(
        'name="arm"><collision><origin xyz="0 0 0"', 'name="arm"><collision><origin xyz=".1 0 0"'
    )
    description = description.replace('xyz=".05 0 0"', 'xyz=".08 .06 0"')
    path.write_text(description)
    model = CollisionModel.read(
        collision_assets(path), robot.names, robot.neutral, (), (("arm", "hand", 0.14),)
    )
    assert model is not None
    assert model.at(robot.neutral).unallowed == 0.0
    assert model.at(np.array([0.0, -0.64])).unallowed > 0.19


def test_builtin_bearing_profile_round_trip(tmp_path):
    profile = MappingProfile.sentient_wuji()
    assert not profile.allowed_collision_pairs
    assert len(profile.bearing_depth_limits_m) == 69
    path = tmp_path / "wuji.json"
    path.write_text(json.dumps(profile.document()))
    assert MappingProfile.read(path) == profile


def test_verified_bearings_bind_to_visual_cad_and_hold_only_within_the_arc(tmp_path):
    from sx_embodiments._ego_collision import (
        BearingArc,
        CollisionModel,
        JointInterval,
        cad_sweep_identity,
        certified_arcs,
        collision_assets,
        nominal_bearings,
    )
    from sx_embodiments.ego_retarget import _model

    path = tmp_path / "rotor.urdf"
    path.write_text("""<robot name="rotor"><link name="case">
      <visual><geometry><mesh filename="cad.stl"/></geometry></visual>
      <collision><geometry><sphere radius=".1"/></geometry></collision></link>
      <link name="rotor"><collision><origin xyz=".05 0 0"/>
        <geometry><sphere radius=".1"/></geometry></collision></link>
      <joint name="axle" type="revolute"><parent link="case"/><child link="rotor"/>
        <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="3"/>
      </joint></robot>""")
    cad = tmp_path / "cad.stl"
    cad.write_bytes(b"first visual CAD")
    before = collision_assets(path)[2]
    cad.write_bytes(b"changed visual CAD")
    geometry = collision_assets(path)
    assert geometry[2] != before
    assert not certified_arcs(cad_sweep_identity(), geometry[2])

    robot = _model(path)
    # A serialized v1 profile can still carry the nominal depth rows without
    # any new certificate field. It cannot grant clearances on a different arm.
    with pytest.raises(ValueError, match="exact certified CAD"):
        CollisionModel.read(geometry, robot.names, robot.neutral, (), nominal_bearings())
    arc = BearingArc(("case", "rotor"), (JointInterval("axle", -0.2, 0.2),), 0.151)
    collision = CollisionModel.read(geometry, robot.names, robot.neutral, (), (), (arc,))
    assert collision is not None
    assert np.isclose(collision.at(np.array([0.1])).maximum, 0.15)
    assert collision.at(np.array([0.1])).unallowed == 0
    assert np.isclose(collision.at(np.array([0.25])).unallowed, 0.15)
