"""Fixed CAD parts survive reusable arm composition exactly once, without moving joints."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest

from sx_embodiments import EmbodimentId, EmbodimentName, historical_document, read_recorded
from sx_embodiments.kinematics import Kinematics, from_xyz_rpy
from sx_embodiments.known.humanoid import SENTIENT_HUMANOID_BUILD
from sx_embodiments.known.static_upper_body import STATIC_UPPER_BODY_URDF

PREVIOUS = EmbodimentId("adf5a8a1e6621b0300cf3b316d2caf778e2ceef9c9aab63f9f9f58cb6981ac54")


def _visuals(payload, positions):
    tree = ET.fromstring(payload)
    kin = Kinematics.of(payload)
    poses = kin.poses("chest_link", positions)
    result = {}
    for link in tree.findall("link"):
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            assert mesh is not None
            origin = visual.find("origin")
            xyz = tuple(map(float, origin.get("xyz", "0 0 0").split()))
            rpy = tuple(map(float, origin.get("rpy", "0 0 0").split()))
            key = (mesh.get("filename"), mesh.get("scale", "1 1 1"))
            result.setdefault(key, []).append(
                np.asarray(poses[link.get("name")]).reshape(4, 4)
                @ np.asarray(from_xyz_rpy(xyz, rpy)).reshape(4, 4)
            )
    return result


def test_split_preserves_every_source_visual_once_and_every_original_joint_pose():
    old = read_recorded(
        historical_document(EmbodimentName("sentient-static-upper-body"), PREVIOUS)
    ).embodiment.urdf_bytes
    new = STATIC_UPPER_BODY_URDF.path().read_bytes()
    before, after = Kinematics.of(old), Kinematics.of(new)
    assert before.independent == after.independent
    for angle in (0.0, 0.17):
        positions = dict.fromkeys(before.independent, angle)
        expected, actual = (
            before.poses("chest_link", positions),
            after.poses("chest_link", positions),
        )
        for name, pose in expected.items():
            assert actual[name] == pytest.approx(pose, abs=1e-12), name
        original, split = _visuals(old, positions), _visuals(new, positions)
        assert original.keys() == split.keys()
        for key, poses in original.items():
            remaining = list(split[key])
            assert len(poses) == len(remaining)
            for pose in poses:
                closest = min(
                    range(len(remaining)), key=lambda i: np.max(np.abs(remaining[i] - pose))
                )
                np.testing.assert_allclose(remaining.pop(closest), pose, atol=2e-8, rtol=0)


def test_both_bodies_include_source_mount_geometry_and_estimated_inertia_once():
    static = ET.fromstring(STATIC_UPPER_BODY_URDF.path().read_bytes())
    full = ET.fromstring(SENTIENT_HUMANOID_BUILD.urdf())
    assert len(static.find("link[@name='chest_link']").findall("visual")) == 5
    for side in ("left", "right"):
        name = side + "_shoulder_mount_link"
        expected = static.find(f"link[@name='{name}']")
        actual = full.find(f"link[@name='{name}']")
        assert len(expected.findall("visual")) == len(actual.findall("visual")) == 4
        for element in ("inertial", "collision"):
            assert expected.find(element) is not None
        # Full composition qualifies relative asset references; geometry origins stay identical.
        for a, b in zip(expected.findall("visual"), actual.findall("visual"), strict=True):
            assert a.find("origin").attrib == b.find("origin").attrib
            assert (
                b.find("geometry/mesh")
                .get("filename")
                .endswith(a.find("geometry/mesh").get("filename"))
            )
        joint = full.find(f"joint[@name='{side}_shoulder_pitch_joint']")
        assert joint.find("parent").get("link") == name
        assert joint.find("origin").get("xyz") == "0 0 0"
