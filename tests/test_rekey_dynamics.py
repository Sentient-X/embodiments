"""Dynamics documents move to a re-rendered description only when it changed velocity limits."""

import importlib.util
import json
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parents[1] / "tools/rekey_dynamics.py"
_SPEC = importlib.util.spec_from_file_location("rekey_dynamics", TOOL)
assert _SPEC is not None and _SPEC.loader is not None
rekey_dynamics = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(rekey_dynamics)

URDF = """<robot name="r"><link name="a"><inertial><mass value="{mass}"/></inertial></link>
<link name="b"/><joint name="j" type="revolute"><parent link="a"/><child link="b"/>
<limit lower="{lower}" upper="1" effort="{effort}" velocity="{velocity}"/></joint></robot>
"""


def _write(
    tmp_path: Path,
    name: str,
    mass: str,
    velocity: str,
    *,
    effort: str = "54",
    lower: str = "-1",
) -> Path:
    path = tmp_path / name
    path.write_text(URDF.format(mass=mass, velocity=velocity, effort=effort, lower=lower))
    return path


def test_a_velocity_only_rerender_rekeys_every_digest_a_document_carries(tmp_path: Path) -> None:
    old = _write(tmp_path, "old.urdf", "1.0", "0.5")
    new = _write(tmp_path, "new.urdf", "1.0", "3.77", effort="27")
    old_sha, new_sha = rekey_dynamics.digest(old), rekey_dynamics.digest(new)
    document = tmp_path / "fixture.json"
    document.write_text(
        json.dumps({"description_sha256": old_sha, "calibration": {"description_sha256": old_sha}})
    )
    rekey_dynamics.check_only_velocity(old, new)
    assert rekey_dynamics.rekey(document, old_sha, new_sha)
    assert json.loads(document.read_text())["calibration"]["description_sha256"] == new_sha
    assert not rekey_dynamics.rekey(document, old_sha, new_sha)


def test_a_rerender_that_changes_dynamics_or_a_foreign_document_is_refused(
    tmp_path: Path,
) -> None:
    old = _write(tmp_path, "old.urdf", "1.0", "0.5")
    for changed in [
        _write(tmp_path, "mass.urdf", "2.0", "0.5"),
        _write(tmp_path, "lower.urdf", "1.0", "0.5", lower="-2"),
        _write(tmp_path, "nan.urdf", "1.0", "nan"),
        _write(tmp_path, "negative.urdf", "1.0", "-5"),
        _write(tmp_path, "effortless.urdf", "1.0", "0.5", effort="0"),
    ]:
        with pytest.raises(rekey_dynamics.RekeyError):
            rekey_dynamics.check_only_velocity(old, changed)
    foreign = tmp_path / "foreign.json"
    foreign.write_text(json.dumps({"description_sha256": "another"}))
    with pytest.raises(rekey_dynamics.RekeyError):
        rekey_dynamics.rekey(foreign, rekey_dynamics.digest(old), "new")


def test_every_identified_static_upper_body_document_names_the_current_description() -> None:
    from sx_embodiments.known.static_upper_body import STATIC_UPPER_BODY_URDF

    root = TOOL.parents[1]
    documents = [
        *sorted((root / "docs/dynamics").glob("*.json")),
        root / "drivers/sx-embodiment-drivers/tests/fixtures/dynamics-parity.json",
    ]
    for document in documents:
        recorded = rekey_dynamics._recorded(json.loads(document.read_text()))
        assert recorded, document
        assert set(recorded) == {STATIC_UPPER_BODY_URDF.sha256}, document
