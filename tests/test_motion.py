"""Each driven joint's motion comes from its motor's rating and the arm it carries."""

from __future__ import annotations

import math

import pytest

from sx_embodiments import development_embodiments, embodiments
from sx_embodiments.calibrate.motion import (
    SETTLED_TRACKING_RAD,
    JointMotion,
    joint_motion,
    tracking_bound,
)
from sx_embodiments.known.static_upper_body import (
    STATIC_UPPER_BODY_MASS_MARGIN,
    STATIC_UPPER_BODY_SERVO_APPROACH,
    STATIC_UPPER_BODY_SPEED_CAP,
)
from sx_embodiments.layout import ACTUATOR_RATINGS, ActuatorRating, LayoutError


@pytest.fixture(scope="module")
def static_motion() -> dict[str, JointMotion]:
    return joint_motion(
        development_embodiments["sentient-static-upper-body"],
        STATIC_UPPER_BODY_SERVO_APPROACH,
        STATIC_UPPER_BODY_MASS_MARGIN,
        STATIC_UPPER_BODY_SPEED_CAP,
    )


def test_every_damiao_joint_moves_at_its_motors_rated_speed(
    static_motion: dict[str, JointMotion],
) -> None:
    body = development_embodiments["sentient-static-upper-body"]
    for coordinate in body.state.coordinates:
        binding = coordinate.axis.actuator
        if binding is None:
            continue
        motion = static_motion[coordinate.joint_name]
        rated = ACTUATOR_RATINGS[binding.model].rated_speed_rad_s
        assert rated is not None
        servo = STATIC_UPPER_BODY_SERVO_APPROACH.get(binding.bus_port)
        expected = rated / binding.reduction if servo is None else min(rated, servo.speed)
        expected = min(expected, STATIC_UPPER_BODY_SPEED_CAP.get(coordinate.joint_name, expected))
        assert math.isclose(motion.speed, expected), coordinate.joint_name
        if servo is not None:
            assert motion.acceleration <= servo.acceleration


def test_an_arm_joints_acceleration_is_what_its_torque_leaves_over_its_inertia(
    static_motion: dict[str, JointMotion],
) -> None:
    # The J8009P shoulders carry the whole arm on their 20 N·m rated torque: well below the
    # 0.1 s ramp of a free joint.
    shoulder = static_motion["left_shoulder_pitch_joint"]
    assert shoulder.acceleration < shoulder.speed / 0.1
    assert shoulder.acceleration < 20.0 / 0.1
    # The torso keeps its bring-up 0.25 rad/s until it is identified and re-tuned.
    assert math.isclose(static_motion["torso_joint"].speed, 0.25)
    # A wrist moves next to nothing: its ramp, not its torque, bounds it.
    wrist = static_motion["left_wrist_roll_joint"]
    assert math.isclose(wrist.acceleration, wrist.speed / 0.1)


def test_the_tracking_bound_grows_with_speed() -> None:
    assert tracking_bound(0.0) == SETTLED_TRACKING_RAD
    assert math.isclose(tracking_bound(3.77), 0.2 + 3 * 3.77 * 0.02)


def test_a_motion_must_be_positive_and_finite() -> None:
    for speed, acceleration in ((0.0, 1.0), (1.0, math.inf), (-1.0, 1.0)):
        with pytest.raises(LayoutError):
            JointMotion(speed, acceleration)


def test_every_rated_joint_moves_its_rotor_beside_its_links() -> None:
    # The humanoid's wrist yaw links weigh 1.05e-4 kg·m²; its DM4340's rotor, 2e-5 kg·m² turning
    # forty times as fast, is 0.032 at the joint. Without it the joint looked a hundred times too
    # light to the simulator and the gains' ceilings (sx#3004).
    for name in ("sentient-humanoid-upper-body", "b601-dm"):
        body = embodiments[name]
        motion = joint_motion(body)
        for coordinate in body.state.coordinates:
            binding = coordinate.axis.actuator
            if binding is None:
                continue
            armature = ACTUATOR_RATINGS[binding.model].armature_kg_m2
            if armature is None:
                continue
            assert motion[coordinate.joint_name].inertia >= armature * binding.reduction**2, (
                coordinate.joint_name
            )
    wrist = joint_motion(embodiments["sentient-humanoid-upper-body"])["right_wrist_yaw_joint"]
    assert 0.032 < wrist.inertia < 0.0325


def test_an_armature_must_be_positive_and_finite() -> None:
    for armature in (0.0, -1e-3, math.nan):
        with pytest.raises(LayoutError):
            ActuatorRating(7.0, armature_kg_m2=armature)
