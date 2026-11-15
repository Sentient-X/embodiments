"""The full humanoid carries its real drives: every leg and waist motor, and the limits its
description declares come from those motors' ratings, never from the CAD's placeholders."""

import importlib.util
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from sx_embodiments import development_embodiments
from sx_embodiments.buses import driven_axes
from sx_embodiments.errors import LayoutError
from sx_embodiments.kinematics import Kinematics
from sx_embodiments.known.humanoid import (
    SENTIENT_HUMANOID_RATED_URDF,
    SENTIENT_HUMANOID_URDF,
)
from sx_embodiments.layout import (
    ActuatorBinding,
    ActuatorBus,
    ActuatorModel,
    DirectDrive,
    LinkageDrive,
    joint_rating,
)
from sx_embodiments.nominal_mujoco import NominalExportError, rated_joint

ROOT = Path(__file__).resolve().parents[1]

#: humanoid_pkg config/humanoid_joint_motor_assignments.csv and sentient_can
#: examples/capture_readings_27motors.cpp: each leg joint's send ids and motor model.
LEG_MOTORS = {
    "right_hip_pitch_joint": ((0x01,), ActuatorModel.DAMIAO_DM10010),
    "right_hip_roll_joint": ((0x02,), ActuatorModel.DAMIAO_DM10010),
    "right_hip_yaw_joint": ((0x03,), ActuatorModel.DAMIAO_DM8009),
    "right_knee_joint": ((0x04,), ActuatorModel.DAMIAO_DM10010),
    "right_foot1_joint": ((0x05, 0x06), ActuatorModel.DAMIAO_DM4340),
    "right_foot2_joint": ((0x05, 0x06), ActuatorModel.DAMIAO_DM4340),
    "left_hip_pitch_joint": ((0x07,), ActuatorModel.DAMIAO_DM10010),
    "left_hip_roll_joint": ((0x08,), ActuatorModel.DAMIAO_DM10010),
    "left_hip_yaw_joint": ((0x09,), ActuatorModel.DAMIAO_DM8009),
    "left_knee_joint": ((0x0A,), ActuatorModel.DAMIAO_DM10010),
    "left_foot1_joint": ((0x0B, 0x0C), ActuatorModel.DAMIAO_DM4340),
    "left_foot2_joint": ((0x0B, 0x0C), ActuatorModel.DAMIAO_DM4340),
    "torso_joint": ((0x0F,), ActuatorModel.DAMIAO_DM8009),
    "waist_roll_joint": ((0x0E, 0x0D), ActuatorModel.DAMIAO_DM4340),
    "waist_pitch_joint": ((0x0E, 0x0D), ActuatorModel.DAMIAO_DM4340),
}


def _body():
    return development_embodiments["sentient-humanoid"]


def test_every_leg_and_waist_joint_names_its_motors() -> None:
    axes = {c.axis.name: c.axis for c in _body().state.coordinates}
    for name, (ids, model) in LEG_MOTORS.items():
        actuation = axes[name].actuation
        if isinstance(actuation, DirectDrive):
            bindings = (actuation.actuator,)
        else:
            assert isinstance(actuation, LinkageDrive), name
            bindings = actuation.actuators
        assert tuple(b.bus_id for b in bindings) == ids, name
        assert {b.model for b in bindings} == {model}, name
        assert all(b.bus is ActuatorBus.DAMIAO_CAN for b in bindings), name
        if name != "torso_joint":
            assert all(b.provisional for b in bindings), name


def test_no_driver_chain_carries_a_linkage_and_every_leg_motor_is_unverified() -> None:
    driven = {axis.joint: axis.binding for axis in driven_axes(_body())}
    linkages = {n for n, (ids, _) in LEG_MOTORS.items() if len(ids) > 1}
    assert not set(driven) & linkages
    legs = {n for n, (ids, _) in LEG_MOTORS.items() if len(ids) == 1 and n != "torso_joint"}
    assert legs <= set(driven)
    assert all(driven[n].provisional for n in legs)


def test_a_linkage_motor_must_stay_provisional() -> None:
    motor = ActuatorBinding(ActuatorModel.DAMIAO_DM4340, ActuatorBus.DAMIAO_CAN, 5)
    with pytest.raises(LayoutError):
        LinkageDrive((motor, motor))


def test_the_described_limits_are_the_drives_ratings() -> None:
    joints = {j.get("name"): j for j in ET.fromstring(_body().urdf_bytes).iter("joint")}
    axes = {c.axis.name: c.axis for c in _body().state.coordinates}
    for name in LEG_MOTORS:
        rating = joint_rating(axes[name].actuation)
        assert rating is not None, name
        limit = joints[name].find("limit")
        assert limit is not None, name
        assert float(limit.get("effort", "")) == pytest.approx(rating.effort), name
        assert float(limit.get("velocity", "")) == pytest.approx(rating.velocity, rel=1e-8), name
    knee = joint_rating(axes["right_knee_joint"].actuation)
    assert knee is not None and knee.effort == 150.0
    ankle = joint_rating(axes["right_foot1_joint"].actuation)
    assert ankle is not None and ankle.effort == 54.0


def test_the_rated_source_is_the_generators_rendering() -> None:
    spec = importlib.util.spec_from_file_location(
        "compose_humanoid_rated_urdf", ROOT / "tools/compose_humanoid_rated_urdf.py"
    )
    assert spec is not None and spec.loader is not None
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    source = SENTIENT_HUMANOID_URDF.path().read_bytes().decode("utf-8")
    assert SENTIENT_HUMANOID_RATED_URDF.path().read_bytes() == tool.render(source).encode()


def test_a_nominal_joint_takes_its_effort_and_armature_from_the_drive() -> None:
    coordinates = {c.axis.name: c for c in _body().state.coordinates}
    hip_yaw = rated_joint(coordinates["left_hip_yaw_joint"], kp=100.0, kd=5.0, action_scale=0.25)
    assert (hip_yaw.effort, hip_yaw.armature) == (40.0, 0.02)
    with pytest.raises(NominalExportError):
        rated_joint(coordinates["left_hip_yaw_joint"], kp=1.0, kd=0.0, action_scale=1, armature=1)
    # The DM10010's rotor inertia is unpublished: the profile states the one it assumes.
    with pytest.raises(NominalExportError):
        rated_joint(coordinates["left_knee_joint"], kp=1.0, kd=0.0, action_scale=1.0)
    knee = rated_joint(
        coordinates["left_knee_joint"], kp=1.0, kd=0.0, action_scale=1.0, armature=0.05
    )
    assert (knee.effort, knee.armature) == (150.0, 0.05)


def test_both_hip_rolls_abduct_at_a_positive_angle_so_one_range_serves_both() -> None:
    """The right hip roll's origin (R_x(pi)) sits below the right hip pitch's (R_y(pi)), so its
    +X axis is the pelvis's -X: a positive angle moves each foot outward, and both legs share
    the CAD's [-0.279, 1.920] (16 deg adduction, 110 deg abduction). Mirroring the right range
    from its axis alone (2026-11-16) would have inverted it."""
    body = _body()
    kinematics = Kinematics.of(body.urdf_bytes)
    root = next(iter(kinematics.links - kinematics.parent_joint.keys()))
    zero = dict.fromkeys(kinematics.independent, 0.0)
    for side, outward in (("left", 1.0), ("right", -1.0)):
        foot = kinematics.joints[f"{side}_foot2_joint"].child
        rest = kinematics.pose(root, foot, zero)[7]
        rolled = kinematics.pose(root, foot, {**zero, f"{side}_hip_roll_joint": 0.3})[7]
        assert (rolled - rest) * outward > 0.15, side
    bounds = {c.axis.name: (c.lower, c.upper) for c in body.state.coordinates}
    assert bounds["left_hip_roll_joint"] == bounds["right_hip_roll_joint"]
