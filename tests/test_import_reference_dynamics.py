"""A reference arm model, accepted on the robot, becomes one identified-dynamics calibration
(``tools/import_reference_dynamics.py``). Ported from the compensation trial's importer tests
(embodiments #279, Kripansh Sharma). No device or network access; the reference's files are
read as text and AST, never imported."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import shutil
import sys
from dataclasses import replace
from pathlib import Path
from types import ModuleType
from typing import cast

import pytest

np = pytest.importorskip("numpy")

from sx_embodiments.calibration import (  # noqa: E402
    DynamicsCalibration,
    JointFriction,
    calibration_from_dict,
    calibration_id,
    calibration_to_dict,
)

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "drivers/sx-embodiment-drivers/tests/fixtures/reference_gravity"
REFERENCE = FIXTURES / "reference_bundle/reference_snapshot"
URDF = (ROOT / "render_packs/sentient-static-upper-body/robot.urdf").read_bytes()
BINDINGS = (ROOT / "drivers/sx-damiao-can/bindings/sentient-static-upper-body.json").read_bytes()
DRIVE = (FIXTURES / "drive.json").read_bytes()
COMMISSIONING = json.loads((FIXTURES / "commissioning.json").read_text())
ARM = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_upper_arm_yaw_joint",
    "left_elbow_pitch_joint",
    "left_forearm_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
)
# The Spark's whole-body fit (Fleet revision 2, read 2026-11-14), which the arm's sits beside.
WHOLE_BYTES = (FIXTURES / "static-upper-body.dynamics.json").read_bytes()
WHOLE = calibration_from_dict(json.loads(WHOLE_BYTES))
assert isinstance(WHOLE, DynamicsCalibration)


def _importer() -> ModuleType:
    spec = importlib.util.spec_from_file_location(
        "import_reference_dynamics", ROOT / "tools/import_reference_dynamics.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


IMPORTER = _importer()


def _build(
    reference: Path = REFERENCE,
    drive: bytes = DRIVE,
    commissioning: object = COMMISSIONING,
    max_residual_nm: float = 0.1,
    beside: DynamicsCalibration | None = WHOLE,
) -> tuple[DynamicsCalibration, dict[str, object]]:
    return IMPORTER.build_calibration(
        reference,
        URDF,
        drive,
        BINDINGS,
        json.dumps(commissioning).encode(),
        mass_scale=1.0,
        friction_multiplier=1.0,
        max_residual_nm=max_residual_nm,
        beside=beside,
    )


@pytest.fixture(scope="module")
def nominal() -> tuple[DynamicsCalibration, dict[str, object]]:
    # An unrelated commissioned bus changes nothing: only the arm's buses are bound.
    commissioning = copy.deepcopy(COMMISSIONING)
    commissioning["ports"]["neck"] = "other-neck-bus"
    return _build(commissioning=commissioning)


def test_the_fixture_is_what_the_importer_writes(
    nominal: tuple[DynamicsCalibration, dict[str, object]],
) -> None:
    calibration, receipt = nominal
    fixture = json.loads((FIXTURES / "left-arm.dynamics.json").read_text())
    assert calibration_to_dict(calibration) == fixture
    assert calibration_from_dict(fixture) == calibration
    assert json.loads((FIXTURES / "left-arm.receipt.json").read_text()) == json.loads(
        json.dumps(receipt)
    )
    assert calibration.units == ("fixture-can-serial:1",)
    assert calibration.description_sha256 == hashlib.sha256(URDF).hexdigest()
    friction = {item.joint: item for item in calibration.joints}
    assert friction["left_elbow_pitch_joint"].coulomb == pytest.approx(0.22)
    assert friction["left_elbow_pitch_joint"].velocity_deadband == pytest.approx(0.02)
    assert friction["left_wrist_pitch_joint"].coulomb == 0.0
    assert all(
        item.velocity_deadband == 0.0 for name, item in friction.items() if "elbow" not in name
    )
    assert calibration.stiction_velocity == pytest.approx(0.01)


def test_a_nominal_drive_leaves_a_residual_the_default_bound_refuses(
    nominal: tuple[DynamicsCalibration, dict[str, object]],
) -> None:
    _, receipt = nominal
    residual = receipt["residual_nm"]
    assert isinstance(residual, dict)
    worst = max(float(value) for value in residual.values())
    # The reference's encoder zeros disagree with the nominal drive's (wrist roll by 104°).
    assert 1e-3 < worst < 0.1
    offsets = receipt["offsets_rad"]
    assert isinstance(offsets, dict)
    assert offsets["left_wrist_roll_joint"] == pytest.approx(-1.820974)


def test_a_drive_that_agrees_with_the_reference_zeros_fits_exactly() -> None:
    config = json.loads((REFERENCE / "left_arm_calibrated_config.json").read_text())["joints"]
    drive = json.loads(DRIVE)
    bindings = {axis["joint"]: axis for axis in json.loads(BINDINGS)["axes"]}
    for joint in drive["joints"]:
        if joint["joint"] in config:
            gain = bindings[joint["joint"]]["sign"] * joint["direction"]
            gain *= bindings[joint["joint"]]["reduction"]
            joint["zero"] = -config[joint["joint"]]["q_offset_motor_rad"] / gain
    calibration, receipt = _build(drive=json.dumps(drive).encode(), max_residual_nm=1e-3)
    residual = receipt["residual_nm"]
    assert isinstance(residual, dict)
    assert max(float(value) for value in residual.values()) < 1e-4
    assert calibration.measurement.residual is not None
    assert calibration.measurement.residual.value < 1e-4


def test_a_changed_torque_policy_is_refused(tmp_path: Path) -> None:
    # A source can be integrity-valid yet implement a newly changed control law.
    bundle = tmp_path / "bundle"
    shutil.copytree(FIXTURES / "reference_bundle", bundle)
    controller = bundle / "reference_snapshot/left_arm_freedrive.py"
    controller.write_text(controller.read_text().replace("0.02", "0.025"))
    manifest = json.loads((bundle / "MANIFEST.json").read_text())
    for record in manifest["files"]:
        if record["path"] == "reference_snapshot/left_arm_freedrive.py":
            record["bytes"] = controller.stat().st_size
            record["sha256"] = hashlib.sha256(controller.read_bytes()).hexdigest()
    (bundle / "MANIFEST.json").write_text(json.dumps(manifest))
    with pytest.raises(IMPORTER.ReferenceImportError, match="torque policy changed"):
        _build(reference=bundle / "reference_snapshot")


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("instance", "different-instance"),
        ("ports", {"neck": "other"}),
        ("schema", "unknown"),
        ("pod_revision", True),
    ],
)
def test_foreign_commissioning_is_refused(field: str, value: object) -> None:
    bad = copy.deepcopy(COMMISSIONING)
    bad[field] = value
    with pytest.raises(IMPORTER.ReferenceImportError):
        _build(commissioning=bad)


def test_an_unrecorded_source_is_refused_before_it_is_read(tmp_path: Path) -> None:
    reference = tmp_path / "reference_snapshot"
    reference.mkdir()
    (reference / "left_arm_dynamics.py").write_text("raise RuntimeError('do not execute')")
    (tmp_path / "MANIFEST.json").write_text('{"files":[]}')
    with pytest.raises(IMPORTER.ReferenceImportError, match="manifest"):
        _build(reference=reference)


def test_a_deadband_is_carried_only_when_set() -> None:
    plain = JointFriction("j", coulomb=0.2, viscous=0.0, armature=0.0)
    document = calibration_to_dict(
        DynamicsCalibration(
            description_sha256=hashlib.sha256(b"x").hexdigest(),  # type: ignore[arg-type]
            stiction_velocity=0.01,
            links=(),
            joints=(plain,),
            units=(),
            measurement=calibration_from_dict(
                json.loads((FIXTURES / "left-arm.dynamics.json").read_text())
            ).measurement,
        )
    )
    joints = document["joints"]
    assert isinstance(joints, list)
    entry = joints[0]
    assert isinstance(entry, dict)
    # A calibration written before the deadband existed keeps its identity.
    assert "velocity_deadband" not in entry
    parsed = calibration_from_dict(document)
    assert calibration_id(parsed) == calibration_id(calibration_from_dict(document))
    entry["velocity_deadband"] = 0.02
    banded = calibration_from_dict(document)
    assert isinstance(banded, DynamicsCalibration)
    assert banded.joints[0].velocity_deadband == 0.02
    entry["velocity_deadband"] = -1.0
    with pytest.raises(ValueError, match="friction terms"):
        calibration_from_dict(document)


def test_a_whole_body_fit_gives_up_what_the_reference_takes_over(
    nominal: tuple[DynamicsCalibration, dict[str, object]],
) -> None:
    calibration, _ = nominal
    whole = DynamicsCalibration(
        description_sha256=calibration.description_sha256,
        stiction_velocity=0.02,
        links=tuple(
            sorted(
                (*calibration.links, replace(calibration.links[0], link="right_elbow_pitch_link")),
                key=lambda item: item.link,
            )
        ),
        joints=(
            JointFriction("left_elbow_pitch_joint", coulomb=0.0, viscous=0.0, armature=0.0),
            JointFriction("right_elbow_pitch_joint", coulomb=0.77, viscous=0.26, armature=0.0),
        ),
        units=calibration.units,
        measurement=calibration.measurement,
    )
    trimmed = IMPORTER.superseded(whole, calibration, "ab" * 32)
    assert [item.link for item in trimmed.links] == ["right_elbow_pitch_link"]
    assert [item.joint for item in trimmed.joints] == ["right_elbow_pitch_joint"]
    assert trimmed.stiction_velocity == 0.02
    with pytest.raises(IMPORTER.ReferenceImportError, match="all of the old one"):
        IMPORTER.superseded(calibration, calibration, "ab" * 32)


def test_the_trimmed_fit_is_a_subset_that_claims_no_residual(
    nominal: tuple[DynamicsCalibration, dict[str, object]],
) -> None:
    # It keeps the original's method, time and evidence, names the document it was cut from,
    # and claims none of the whole fit's residual (over joints it no longer carries).
    calibration, _ = nominal
    digest = hashlib.sha256(WHOLE_BYTES).hexdigest()
    trimmed = IMPORTER.superseded(WHOLE, calibration, digest)
    assert trimmed.measurement.method == WHOLE.measurement.method
    assert trimmed.measurement.measured_at == WHOLE.measurement.measured_at
    assert trimmed.measurement.residual is None
    assert [item.reference for item in trimmed.measurement.evidence][-1] == digest
    assert calibration_to_dict(trimmed) == json.loads(
        (FIXTURES / "static-upper-body.trimmed.dynamics.json").read_text()
    )


def _station_theta(*calibrations: DynamicsCalibration) -> object:
    from sx_embodiments.calibrate.dynamics import RigidBodyTree

    tree = RigidBodyTree.of(URDF)
    theta = tree.parameters()
    for calibration in calibrations:
        applied = tree.parameters(calibration)
        for item in calibration.links:
            at = 10 * tree.links.index(item.link)
            theta[at : at + 10] = applied[at : at + 10]
    return theta


def test_the_arm_is_fitted_in_the_model_the_station_runs_beside_the_whole_body_fit(
    nominal: tuple[DynamicsCalibration, dict[str, object]],
) -> None:
    # The refute of 2026-11-14: fitted against the description's gripper, the arm's file put
    # beside the whole-body fit (whose gripper base is 0.086 kg, not 0.048) moved the arm's
    # gravity by up to 0.21 N·m. Fitted beside it, the station's model is what was certified.
    calibration, receipt = nominal
    trimmed = IMPORTER.superseded(WHOLE, calibration, hashlib.sha256(WHOLE_BYTES).hexdigest())
    alone, _ = _build(beside=None)
    oracle = json.loads((FIXTURES / "gravity.json").read_text())
    from sx_embodiments.calibrate.dynamics import RigidBodyTree

    tree = RigidBodyTree.of(URDF)
    rows = [tree.axes.index(name) for name in ARM]
    rest = np.zeros(len(tree.axes))

    def oracle_error(theta: object) -> float:
        worst = 0.0
        for case in oracle["cases"]:
            q = rest.copy()
            q[rows] = case["q"]
            got = 0.95 * (tree.regressor(q, rest, rest) @ theta)[rows]
            worst = max(worst, float(np.abs(got - np.array(case["gravity"])).max()))
        return worst

    assert oracle_error(_station_theta(trimmed, calibration)) < 0.045
    assert oracle_error(_station_theta(trimmed, alone)) > 0.1
    assert oracle_error(_station_theta(alone)) < 0.045
    residual = cast("dict[str, float]", receipt["residual_nm"])
    assert 1e-3 < max(residual.values()) < 0.1
    assert "existing fit" in cast("str", receipt["beside"])


def test_a_drive_rendered_for_an_earlier_revision_of_the_body_is_accepted() -> None:
    # The Spark (Fleet revision 2, read 2026-11-14) still carries the body's earlier id; Fleet
    # re-renders the drive with today's at the next assignment, with the same zeros.
    drive = json.loads(DRIVE)
    drive["embodiment"] = "51db79c80289f76c9f20f2043c114836dfcb80baca88e8f5094762c925a37402"
    calibration, _ = _build(drive=json.dumps(drive).encode())
    assert calibration_to_dict(calibration) == json.loads(
        (FIXTURES / "left-arm.dynamics.json").read_text()
    )
    drive["embodiment"] = "0" * 64
    with pytest.raises(IMPORTER.ReferenceImportError, match="different bodies"):
        _build(drive=json.dumps(drive).encode())
