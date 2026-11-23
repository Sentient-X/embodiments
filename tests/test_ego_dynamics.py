"""Independent unit, joint order and virtual-work checks for design inverse dynamics."""

from pathlib import Path

import numpy as np
import pytest

from sx_embodiments._ego_dynamics import EffortModel
from sx_embodiments.calibrate.dynamics import STANDARD_GRAVITY, RigidBodyTree

_PENDULUM = """<robot name="load">
<link name="base"/>
<link name="weight"><inertial><origin xyz="0.3 0 0"/><mass value="2"/>
<inertia ixx="0.01" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/></inertial></link>
<joint name="pitch" type="revolute"><parent link="base"/><child link="weight"/>
<axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="11" velocity="2"/></joint>
<link name="optical"/><joint name="frame" type="fixed">
<parent link="weight"/><child link="optical"/></joint>
</robot>"""


def test_offset_pendulum_gravity_and_linearization(tmp_path: Path) -> None:
    path = tmp_path / "robot.urdf"
    path.write_text(_PENDULUM)
    model = EffortModel.of(path, ("pitch",))
    angle, acceleration = 0.4, 0.75
    tau, position, velocity, mass = model.linearize(
        np.array([angle]), np.array([1.2]), np.array([acceleration])
    )
    expected_inertia = 2 * 0.3**2 + 0.02
    assert mass[0, 0] == pytest.approx(expected_inertia)
    assert tau[0] == pytest.approx(
        expected_inertia * acceleration - 2 * 0.3 * STANDARD_GRAVITY * np.cos(angle)
    )
    assert position[0, 0] == pytest.approx(2 * 0.3 * STANDARD_GRAVITY * np.sin(angle), abs=5e-6)
    assert velocity[0, 0] == pytest.approx(0, abs=1e-7)
    assert model.effort_limits.tolist() == [11.0]
    sideways = EffortModel.of(path, ("pitch",), gravity=(STANDARD_GRAVITY, 0.0, 0.0))
    assert abs(sideways.inverse(np.zeros(1), np.zeros(1), np.zeros(1))[0]) < 1e-12


def test_mimic_and_permuted_slider_match_independent_rigid_tree(tmp_path: Path) -> None:
    # The slider and coupled second pole exercise both units and the follower's
    # reflected inertia; fixed-link mass must also load its upstream joints.
    inertial = """<inertial><origin xyz="0.2 0.1 0"/><mass value="1.7"/>
    <inertia ixx="0.02" iyy="0.03" izz="0.04" ixy="0" ixz="0" iyz="0"/></inertial>"""
    extra = f"""<link name="slider">{inertial}</link>
    <joint name="slide" type="prismatic"><parent link="weight"/><child link="slider"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" velocity="1" effort="142"/></joint>
    <link name="coupled">{inertial}</link>
    <joint name="second" type="continuous"><parent link="slider"/><child link="coupled"/>
    <axis xyz="0 1 0"/><mimic joint="pitch" multiplier="-1.5" offset="0.4"/></joint>
    <link name="payload">{inertial}</link>
    <joint name="bolt" type="fixed"><parent link="coupled"/><child link="payload"/></joint>"""
    xml = _PENDULUM.replace("</robot>", extra + "</robot>")
    path = tmp_path / "coupled.urdf"
    path.write_text(xml)
    names = ("slide", "pitch")
    model = EffortModel.of(path, names)
    independent = RigidBodyTree.of(path.read_bytes())
    order = [names.index(name) for name in independent.axes]
    rng = np.random.default_rng(234)
    for _ in range(5):
        q, v, acc = rng.normal(size=(3, 2))
        tau, *jac = model.linearize(q, v, acc, np.array([1]))
        truth = independent.inverse(independent.parameters(), q[order], v[order], acc[order])
        np.testing.assert_allclose(tau[order], truth, atol=1e-8)
        for kind, jacobian in enumerate(jac):
            h = 1e-6
            plus, minus = np.array([q, v, acc]), np.array([q, v, acc])
            plus[kind, 1] += h
            minus[kind, 1] -= h
            derivative = (model.inverse(*plus) - model.inverse(*minus)) / (2 * h)
            np.testing.assert_allclose(jacobian[:, 0], derivative, atol=3e-5)
    assert model.effort_limits.tolist() == [142, 11]


def test_missing_moving_inertia_is_named(tmp_path: Path) -> None:
    path = tmp_path / "massless.urdf"
    path.write_text(_PENDULUM.replace('<mass value="2"/>', ""))
    with pytest.raises(ValueError, match=r"missing physical link inertia.*weight"):
        EffortModel.of(path, ("pitch",))


def test_source_bound_drive_budget_and_reflected_mass(tmp_path: Path) -> None:
    import hashlib
    import json

    from sx_embodiments._ego_effort_limits import load_effort_limits

    urdf = tmp_path / "load.urdf"
    config = tmp_path / "load.effort.json"
    urdf.write_text(_PENDULUM)
    row = {
        "joint": "pitch",
        "unit": "Nm",
        "maximum": 8,
        "basis": "test continuous drive rating",
        "reflected_inertia_kg_m2": 0.12,
    }
    data = {"urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(), "joints": [row]}
    config.write_text(json.dumps(data))
    loaded = load_effort_limits(config, urdf)
    model = EffortModel.of(urdf, ("pitch",), effort_limits=loaded)
    zero = np.zeros(1)
    torque, _, _, mass = model.linearize(zero, zero, np.ones(1))
    assert model.effort_limits[0] == 8
    assert np.isnan(model.operating_velocity_limits).all()  # historical companion
    operating = {"maximum": 0.25, "unit": "rad/s", "basis": "test axis governor"}
    config.write_text(json.dumps(dict(data, joints=[dict(row, operating_velocity=operating)])))
    governed = EffortModel.of(urdf, ("pitch",), effort_limits=load_effort_limits(config, urdf))
    np.testing.assert_array_equal(governed.operating_velocity_limits, [0.25])
    np.testing.assert_array_equal(governed.effort_limits, model.effort_limits)
    assert mass[0, 0] == pytest.approx(0.32)
    assert torque[0] == pytest.approx(0.32 - 0.6 * STANDARD_GRAVITY)
    for invalid, match in [
        (dict(row, maximum=12), "loosen"),
        (dict(row, maximum=True), "number"),
        (dict(row, unit="N"), "Nm"),
        (dict(row, joint="optical"), "independent"),
        (dict(row, reflected_inertia_kg_m2=-1), "inertia"),
        (dict(row, basis=" "), "basis"),
        (dict(row, operating_velocity=dict(operating, maximum=3)), "loosen"),
        (dict(row, operating_velocity=dict(operating, unit="m/s")), "rad/s"),
        (dict(row, operating_velocity=dict(operating, basis=" ")), "basis"),
        (dict(row, operating_velocity=dict(operating, maximum=0)), "positive"),
    ]:
        config.write_text(json.dumps(dict(data, joints=[invalid])))
        with pytest.raises(ValueError, match=match):
            load_effort_limits(config, urdf)
    config.write_text(json.dumps(dict(data, joints=[row, row])))
    with pytest.raises(ValueError, match="unique"):
        load_effort_limits(config, urdf)
    urdf.write_text(_PENDULUM + "\n")
    with pytest.raises(ValueError, match="SHA-256"):
        load_effort_limits(config, urdf)
    with pytest.raises(ValueError, match="SHA-256"):
        EffortModel.of(urdf, ("pitch",), effort_limits=loaded)
