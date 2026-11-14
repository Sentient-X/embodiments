"""Artifact rejection and motor/model conversion laws; no device or network access."""

from __future__ import annotations

import copy
import importlib.util
import json
import math
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from sx_embodiments.compensation import (
    CompensationError,
    compensation_from_dict,
    compensation_to_dict,
)

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "drivers/sx-embodiment-drivers/tests/fixtures/accepted_compensation"


class CompensationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.document = json.loads((FIXTURES / "profile.json").read_text())

    def test_codec_preserves_the_bound_artifact(self) -> None:
        self.assertEqual(compensation_to_dict(compensation_from_dict(self.document)), self.document)

    def test_imported_affine_map_preserves_motor_coordinates_and_power(self) -> None:
        profile = compensation_from_dict(self.document)
        # The fixture uses a nominal production drive, independent source encoder offsets.
        reference_signs = [1, 1, 1, 1, 1, -1, 1]
        production_signs = [1, 1, 1, 1, 1, -1, -1]
        reference_offsets = [
            -0.011635,
            0.013924,
            -0.008583,
            0.101925,
            -0.000954,
            0.141337,
            1.820974,
        ]
        for index, mapping in enumerate(profile.joints):
            for q in (-0.7, 0, 0.4):
                motor = production_signs[index] * q
                source = reference_signs[index] * (motor - reference_offsets[index])
                self.assertAlmostEqual(mapping.scale * q + mapping.offset, source)
                source_velocity = mapping.scale * 0.37
                source_torque = -0.61
                self.assertAlmostEqual(
                    source_velocity * source_torque,
                    0.37 * mapping.scale * source_torque,
                )
        self.assertEqual(profile.joints[5].coulomb, 0)
        self.assertEqual(profile.joints[6].scale, -1)
        self.assertEqual(profile.joints[3].velocity_deadband, 0.02)

    def test_elbow_offset_is_geometry_and_not_a_second_encoder_offset(self) -> None:
        root = ET.fromstring(compensation_from_dict(self.document).source_urdf)
        elbow = root.find("joint[@name='left_elbow_pitch_joint']/origin")
        self.assertIsNotNone(elbow)
        assert elbow is not None
        self.assertEqual([float(x) for x in elbow.attrib["rpy"].split()], [0, -math.pi / 2, 0])
        self.assertAlmostEqual(self.document["joints"][3]["offset"], -0.101925)

    def test_refuses_missing_unknown_duplicate_and_nonfinite_fields(self) -> None:
        mutations: list[dict[str, object]] = []
        unknown = copy.deepcopy(self.document)
        unknown["hidden_gain"] = 2
        mutations.append(unknown)
        missing = copy.deepcopy(self.document)
        del missing["drive_sha256"]
        mutations.append(missing)
        duplicate = copy.deepcopy(self.document)
        duplicate["joints"].append(copy.deepcopy(duplicate["joints"][0]))
        mutations.append(duplicate)
        for field, value in (
            ("scale", 0),
            ("scale", float("nan")),
            ("offset", float("inf")),
            ("coulomb", -1),
            ("velocity_deadband", -1),
            ("transition_velocity", 0),
            ("max_torque", 0),
            ("coulomb", True),
        ):
            bad = copy.deepcopy(self.document)
            bad["joints"][0][field] = value
            mutations.append(bad)
        for document in mutations:
            with (
                self.subTest(document=document),
                self.assertRaises((CompensationError, ValueError)),
            ):
                compensation_from_dict(document)

    def test_refuses_unmapped_source_motion_and_malformed_identity(self) -> None:
        for field, value in (
            ("source_urdf", "<robot/>"),
            ("source_urdf", "<broken"),
            ("description_sha256", "not-a-digest"),
            ("drive_sha256", "f" * 63),
            ("gravity_scale", 1.1),
            ("torque_slew_nm_s", 0),
            ("units", []),
            ("units", ["bus", "bus"]),
            ("units", ["bus z"]),
            ("units", ["z", "a"]),
        ):
            bad = copy.deepcopy(self.document)
            bad[field] = value
            with (
                self.subTest(field=field, value=value),
                self.assertRaises((CompensationError, ValueError)),
            ):
                compensation_from_dict(bad)

    def test_importer_reproduces_fixture_and_refuses_changed_torque_policy(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "import_teleop_compensation",
            ROOT / "tools/import_teleop_compensation.py",
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        inputs = FIXTURES / "reference_bundle/reference_snapshot"
        artifact, _ = module.build_compensation(
            inputs,
            (ROOT / "render_packs/sentient-static-upper-body/robot.urdf").read_bytes(),
            (FIXTURES / "drive.json").read_bytes(),
            (ROOT / "drivers/sx-damiao-can/bindings/sentient-static-upper-body.json").read_bytes(),
            (FIXTURES / "commissioning.json").read_bytes(),
            gravity_ratio=0.95,
            mass_scale=1,
            friction_multiplier=1,
            max_torque_fraction=0.4,
        )
        self.assertEqual(compensation_to_dict(artifact), self.document)
        # A source can be integrity-valid yet implement a newly changed control law.
        import hashlib
        import shutil

        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory) / "bundle"
            shutil.copytree(FIXTURES / "reference_bundle", bundle)
            controller = bundle / "reference_snapshot/left_arm_freedrive.py"
            controller.write_text(controller.read_text().replace("0.02", "0.025"))
            manifest = json.loads((bundle / "MANIFEST.json").read_text())
            for record in manifest["files"]:
                if record["path"] == "reference_snapshot/left_arm_freedrive.py":
                    record["bytes"] = controller.stat().st_size
                    record["sha256"] = hashlib.sha256(controller.read_bytes()).hexdigest()
            (bundle / "MANIFEST.json").write_text(json.dumps(manifest))
            with self.assertRaisesRegex(CompensationError, "torque policy changed"):
                module.build_compensation(
                    bundle / "reference_snapshot",
                    b"",
                    b"",
                    b"",
                    b"",
                    gravity_ratio=0.95,
                    mass_scale=1,
                    friction_multiplier=1,
                    max_torque_fraction=0.4,
                )

    def test_importer_binds_selected_buses_and_refuses_foreign_commissioning(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "import_teleop_compensation",
            ROOT / "tools/import_teleop_compensation.py",
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        commissioning = json.loads((FIXTURES / "commissioning.json").read_text())
        commissioning["ports"]["neck"] = "other-neck-bus"
        args = (
            FIXTURES / "reference_bundle/reference_snapshot",
            (ROOT / "render_packs/sentient-static-upper-body/robot.urdf").read_bytes(),
            (FIXTURES / "drive.json").read_bytes(),
            (ROOT / "drivers/sx-damiao-can/bindings/sentient-static-upper-body.json").read_bytes(),
        )
        artifact, _ = module.build_compensation(
            *args,
            json.dumps(commissioning).encode(),
            gravity_ratio=0.95,
            mass_scale=1,
            friction_multiplier=1,
            max_torque_fraction=0.4,
        )
        self.assertEqual(artifact.units, ("fixture-can-serial:1",))
        for field, value in (
            ("instance", "different-instance"),
            ("ports", {"neck": "other"}),
            ("schema", "unknown"),
            ("pod_revision", True),
        ):
            bad = copy.deepcopy(commissioning)
            bad[field] = value
            with self.subTest(field=field), self.assertRaises(CompensationError):
                module.build_compensation(
                    *args,
                    json.dumps(bad).encode(),
                    gravity_ratio=0.95,
                    mass_scale=1,
                    friction_multiplier=1,
                    max_torque_fraction=0.4,
                )

    def test_manifest_verification_rejects_an_unrecorded_source(self) -> None:
        # Import only the offline tool; it AST-parses reference modules and never imports CAN.
        spec = importlib.util.spec_from_file_location(
            "import_teleop_compensation",
            ROOT / "tools/import_teleop_compensation.py",
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as directory:
            bundle = Path(directory)
            reference = bundle / "reference_snapshot"
            reference.mkdir()
            (reference / "left_arm_dynamics.py").write_text("raise RuntimeError('do not execute')")
            (bundle / "MANIFEST.json").write_text('{"files":[]}')
            with self.assertRaisesRegex(CompensationError, "manifest"):
                module.build_compensation(
                    reference,
                    b"",
                    b"",
                    b"",
                    b"",
                    gravity_ratio=0.95,
                    mass_scale=1,
                    friction_multiplier=1,
                    max_torque_fraction=0.4,
                )


if __name__ == "__main__":
    unittest.main()
