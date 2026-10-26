"""Identified dynamics: the regressor is MuJoCo's inverse dynamics, the fit recovers a robot's
parameters from its torque, and the result is a calibration one robot instance selects."""

import json
from datetime import UTC, datetime
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from sx_embodiments.calibrate.dynamics import (  # noqa: E402
    PER_LINK,
    ExcitationLimits,
    Hold,
    RigidBodyTree,
    Trajectory,
    excitation,
    fit,
    hold_poses,
)
from sx_embodiments.calibration import (  # noqa: E402
    CalibrationError,
    DynamicsCalibration,
    JointFriction,
    LinkInertia,
    calibration_from_dict,
    calibration_id,
    calibration_to_dict,
)
from sx_embodiments.embodiment import embodiment_from_definition  # noqa: E402
from sx_embodiments.instance import InstanceError, RobotInstance, resolve  # noqa: E402
from sx_embodiments.known.static_upper_body import STATIC_UPPER_BODY_SPEC  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "assets/sentient_static_upper_body/sentient_static_upper_body.urdf"
PARITY = ROOT / "drivers/sx-embodiment-drivers/tests/fixtures/dynamics-parity.json"
ARM = (
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_upper_arm_yaw_joint",
    "right_elbow_pitch_joint",
)


@pytest.fixture(scope="module")
def tree() -> RigidBodyTree:
    return RigidBodyTree.of(URDF.read_bytes())


def test_the_regressor_is_mujocos_inverse_dynamics(tree: RigidBodyTree) -> None:
    fixture = json.loads(PARITY.read_text())
    assert list(tree.axes) == fixture["axes"]
    calibration = calibration_from_dict(fixture["calibration"])
    assert isinstance(calibration, DynamicsCalibration)
    for theta, key in ((tree.parameters(), "design"), (tree.parameters(calibration), "calibrated")):
        for sample in fixture["samples"]:
            torque = tree.inverse(
                theta,
                sample["position"],
                sample["velocity"],
                sample["acceleration"],
                fixture["gravity"],
                calibration.stiction_velocity,
            )
            assert np.max(np.abs(torque - np.array(sample[key]))) < 1e-6


def _truth(tree: RigidBodyTree) -> object:
    rng = np.random.default_rng(3)
    theta = tree.parameters()
    for link in tree.subtree_links(ARM):
        theta[PER_LINK * link : PER_LINK * (link + 1)] *= rng.uniform(0.7, 1.3)
    base = PER_LINK * len(tree.links)
    for joint in ARM:
        at = base + 3 * tree.axes.index(joint)
        theta[at : at + 3] = (0.4, 0.2, 0.01)
    return theta


def test_holds_and_an_excitation_recover_gravity_and_friction(tree: RigidBodyTree) -> None:
    truth = _truth(tree)
    rest = dict.fromkeys(tree.axes, 0.0)
    poses = hold_poses(tree, ARM, rest, 12, lambda _pose: True, candidates=120)
    assert len({tuple(pose[j] for j in ARM) for pose in poses}) == 12
    zero = np.zeros(len(tree.axes))
    holds = [
        Hold(
            {j: pose[j] for j in ARM},
            dict(
                zip(
                    ARM,
                    (
                        tree.inverse(truth, tree.vector(pose), zero, zero)[
                            [tree.axes.index(j) for j in ARM]
                        ]
                    ).tolist(),
                    strict=True,
                )
            ),
        )
        for pose in poses
    ]
    limits = ExcitationLimits(speed=0.5, acceleration=2.0, torque=dict.fromkeys(ARM, 30.0))
    plan, condition = excitation(
        tree, ARM, rest, limits, lambda _pose: True, candidates=4, rate_hz=5.0
    )
    assert np.isfinite(condition)
    q, qd, qdd = plan.at(np.arange(0.0, plan.period, 0.1))
    assert np.max(np.abs(qd)) <= 0.5 + 1e-9 and np.max(np.abs(qdd)) <= 2.0 + 1e-9
    assert np.allclose(qd[0], 0.0, atol=1e-9) and np.allclose(qdd[0], 0.0, atol=1e-9)
    index = [tree.axes.index(j) for j in ARM]
    torque = np.array(
        [
            tree.inverse(
                truth,
                tree.vector(dict(zip(ARM, q[i], strict=True))),
                tree.vector(dict(zip(ARM, qd[i], strict=True))),
                tree.vector(dict(zip(ARM, qdd[i], strict=True))),
            )[index]
            for i in range(len(q))
        ]
    )
    found = fit(tree, ARM, holds, [Trajectory(ARM, q, qd, qdd, torque)], rest)
    assert found.residual < 0.05
    assert not found.reverted
    for pose in poses[:5]:
        x = tree.vector(pose)
        want = tree.inverse(truth, x, zero, zero)[index]
        got = tree.inverse(found.theta, x, zero, zero)[index]
        assert np.max(np.abs(got - want)) < 0.05
    record = found.calibration("ab" * 32, (), datetime(2026, 10, 26, tzinfo=UTC))
    assert calibration_from_dict(calibration_to_dict(record)) == record
    assert {item.joint for item in record.joints} == set(ARM)
    by_joint = {item.joint: item for item in record.joints}
    assert abs(by_joint["right_elbow_pitch_joint"].coulomb - 0.4) < 0.05


def _record(sha: str, link: str = "right_elbow_pitch_link") -> DynamicsCalibration:
    from sx_embodiments.calibration import CalibrationMethod, Measurement

    return DynamicsCalibration(
        description_sha256=sha,  # pyright: ignore[reportArgumentType]
        stiction_velocity=0.02,
        links=(LinkInertia(link, 0.5, (0.0, 0.0, -0.05), (0.008, 0.0, 0.0, 0.008, 0.0, 0.0005)),),
        joints=(JointFriction("right_elbow_pitch_joint", 0.3, 0.1, 0.01),),
        units=(),
        measurement=Measurement(
            CalibrationMethod.DYNAMICS_IDENTIFICATION, datetime(2026, 10, 26, tzinfo=UTC)
        ),
    )


def test_a_dynamics_calibration_is_selected_for_exactly_its_description() -> None:
    body = embodiment_from_definition(STATIC_UPPER_BODY_SPEC)
    record = _record(body.urdf.sha256)
    resolved = resolve(body, RobotInstance.of(body.id, {}, (record,)), (record,))
    assert resolved.dynamics is not None
    assert resolved.dynamics.calibration == record
    assert resolved.dynamics.measured == calibration_id(record)
    other = _record("0" * 64)
    with pytest.raises(InstanceError):
        resolve(body, RobotInstance.of(body.id, {}, (other,)), (other,))
    invented = _record(body.urdf.sha256, link="no_such_link")
    with pytest.raises(InstanceError):
        resolve(body, RobotInstance.of(body.id, {}, (invented,)), (invented,))


def test_unphysical_inertia_is_refused() -> None:
    with pytest.raises(CalibrationError):
        LinkInertia("l", 1.0, (0.0, 0.0, 0.0), (0.1, 0.0, 0.0, 0.1, 0.0, 0.3))
    with pytest.raises(CalibrationError):
        LinkInertia("l", -1.0, (0.0, 0.0, 0.0), (0.1, 0.0, 0.0, 0.1, 0.0, 0.1))
    with pytest.raises(CalibrationError):
        JointFriction("j", -0.1, 0.0, 0.0)
    document = calibration_to_dict(_record("ab" * 32))
    with pytest.raises(CalibrationError):
        calibration_from_dict({**document, "extra": 1})
