"""In-place friction: sweeps of a known joint recover its friction and gravity, a contact is
rejected rather than fitted, and the model is the Rust one."""

import json
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

from sx_embodiments.calibrate.friction import (  # noqa: E402
    Segment,
    SegmentLimits,
    budget,
    fit_joint,
    moving_friction,
)
from sx_embodiments.calibration import JointFriction  # noqa: E402

PARITY = (
    Path(__file__).resolve().parents[1]
    / "drivers/sx-embodiment-drivers/tests/fixtures/friction-parity.json"
)
TRUTH = JointFriction(
    "elbow",
    coulomb=1.4,
    viscous=0.6,
    armature=0.0,
    stribeck=1.2,
    stribeck_velocity=0.03,
    stribeck_exponent=1.5,
    load_motor=0.04,
    load_external=0.04,
)
GRAVITY = (3.0, -1.0, 0.1)


def _sweeps(rng: object, contact: bool = False) -> list[Segment]:
    generator = np.random.default_rng(0)
    segments: list[Segment] = []
    for pass_index in range(3):
        for speed in (0.05, 0.1, 0.2, 0.4):
            for direction in (1.0, -1.0):
                q = np.linspace(-0.6, 0.6, 120)[:: int(direction)]
                v = np.full_like(q, direction * speed)
                g = GRAVITY[0] * np.sin(q) + GRAVITY[1] * np.cos(q) + GRAVITY[2]
                # The drive delivers g + f with f depending on that same drive torque: iterate.
                tau = g.copy()
                for _ in range(20):
                    tau = g + direction * budget(TRUTH, v, tau, -g) + TRUTH.viscous * v
                tau = tau + generator.normal(0.0, 0.02, tau.shape)
                if contact and pass_index == 1 and speed == 0.1 and direction > 0:
                    tau[60:] += 4.0
                segments.append(
                    Segment(f"{pass_index}:{speed}:{direction}", pass_index, q, v, tau, q)
                )
        for direction in (1.0, -1.0):
            q0 = np.array([0.2])
            g0 = GRAVITY[0] * np.sin(q0) + GRAVITY[1] * np.cos(q0) + GRAVITY[2]
            onset = g0.copy()
            for _ in range(20):
                onset = g0 + direction * budget(TRUTH, np.zeros(1), onset, -g0)
            segments.append(
                Segment(
                    f"{pass_index}:break:{direction}",
                    pass_index,
                    q0,
                    np.zeros(1),
                    onset,
                    q0,
                    int(direction),
                )
            )
    return segments


def test_sweeps_recover_friction_and_gravity_and_pick_a_stribeck_model() -> None:
    found = fit_joint("elbow", _sweeps(None), SegmentLimits(peak_torque=27.0))
    assert found.model in ("m2", "m4", "m5")
    assert found.held_out[found.model] < 0.1
    assert found.held_out["m1"] > found.held_out[found.model]
    assert abs(found.gravity[0] - GRAVITY[0]) < 0.1 and abs(found.gravity[1] - GRAVITY[1]) < 0.1
    v = np.array([0.0, 0.1, 0.4])
    want = budget(TRUTH, v, np.zeros(3), np.zeros(3))
    got = budget(found.friction, v, np.zeros(3), np.zeros(3))
    assert np.max(np.abs(got - want)) < 0.2
    assert not found.rejected


def test_a_contact_is_rejected_not_fitted() -> None:
    found = fit_joint("elbow", _sweeps(None, contact=True), SegmentLimits(peak_torque=27.0))
    assert [r.label for r in found.rejected] == ["1:0.1:1.0"]
    assert "contact" in found.rejected[0].reason


def test_the_python_model_is_the_rust_one() -> None:
    fixture = json.loads(PARITY.read_text())
    for case in fixture["cases"]:
        friction = JointFriction("j", **case["friction"])
        got = moving_friction(
            friction,
            np.array(case["speed"]),
            np.array(case["acceleration"]),
            np.array(case["motor"]),
            np.array(case["external"]),
            fixture["stiction_velocity"],
        )
        assert np.max(np.abs(got - np.array(case["torque"]))) < 1e-12
