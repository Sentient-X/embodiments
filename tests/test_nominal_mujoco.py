"""Canonical owner projection is portable, accounted, and geometrically exact."""

import hashlib
import json
import math
import shutil
from dataclasses import replace

import mujoco
import numpy as np
import pytest
from sx_contracts.assets import AssetFormat, AssetProvenance, AssetRef, AssetRole, ProvenancedAsset

from sx_embodiments import Embodiment
from sx_embodiments.compose import RootMount, body_component
from sx_embodiments.identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from sx_embodiments.layout import Bounds, CoordinateUnit, JointAxis, JointLayout
from sx_embodiments.nominal_mujoco import (
    NominalExportError,
    NominalJoint,
    NominalProfile,
    SupportContact,
    export_nominal_mujoco,
)
from sx_embodiments.parts import ChainSpec, Limb


def fixture_body(tmp_path, monkeypatch):
    inertial = (
        '<inertial><origin xyz="0.1 0 0" rpy="0 0 0.2"/><mass value="1"/>'
        '<inertia ixx="0.01" iyy="0.02" izz="0.025" ixy="0" ixz="0" iyz="0"/></inertial>'
    )
    xml = (
        '<robot name="oracle">'
        + "".join(
            f'<link name="{name}">{inertial}<collision><geometry>'
            '<box size="0.2 0.1 0.1"/></geometry></collision></link>'
            for name in ("base", "arm", "tip")
        )
        + '<joint name="motor" type="revolute"><parent link="base"/><child link="arm"/>'
        '<origin xyz="0 0 1"/><axis xyz="0 0 1"/>'
        '<limit lower="-1.5" upper="1.5" effort="3" velocity="1"/></joint>'
        '<joint name="passive" type="revolute"><parent link="arm"/><child link="tip"/>'
        '<origin xyz="1 0 0"/><axis xyz="0 0 1"/>'
        '<limit lower="-1" upper="1" effort="3" velocity="1"/></joint></robot>'
    ).encode()
    assets = tmp_path / "assets"
    assets.mkdir()
    (assets / "oracle.urdf").write_bytes(xml)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSETS", str(assets))
    ref = ProvenancedAsset(
        AssetRef.from_bytes(
            xml,
            uri="package://sx-embodiments/oracle.urdf",
            format=AssetFormat.URDF,
            role=AssetRole.DESCRIPTION,
        ),
        AssetProvenance("https://example.com/oracle", "v1", "oracle.urdf", "MIT"),
    )
    part = ChainSpec(
        PartId("oracle"),
        JointLayout((JointAxis("motor", CoordinateUnit.RADIAN, Bounds(-1.5, 1.5)),)),
        (0.0,),
    )
    body = Embodiment(
        EmbodimentName("oracle"),
        "Oracle",
        EmbodimentKind.ROBOT,
        Lineage("test"),
        (body_component("arm", part, RootMount("base"), limb=Limb.ARM),),
        (ref,),
    )
    return body


def profile(body):
    return NominalProfile(
        str(body.id),
        (NominalJoint("motor", 30.0, 2.0, 5.0, 0.01, 0.2, 0.3),),
        (("passive", 0.4),),
        ("arm", "tip"),
        (SupportContact("tip", 0.05),),
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0, 0.0),
        0.005,
        (0.0, 0.0, -9.81),
        (1.0, 0.005, 0.0001),
        ("Synthetic simulation parameters; no hardware qualification.",),
    )


def test_portable_model_preserves_root_passive_pose_inertia_and_control(tmp_path, monkeypatch):
    body = fixture_body(tmp_path, monkeypatch)
    before = body.to_json()
    output = tmp_path / "bundle"
    receipt = export_nominal_mujoco(body, profile(body), output)
    moved = tmp_path / "moved"
    shutil.move(output, moved)
    model = mujoco.MjModel.from_xml_path(str(moved / "scene.xml"))
    assert model.njnt == 2 and model.nu == 1 and model.nq == 8
    assert model.joint("floating_root").id == 0
    data = mujoco.MjData(model)
    data.qpos[model.jnt_qposadr[model.joint("motor").id]] = 0.3
    mujoco.mj_forward(model, data)
    tip = model.body("tip").id
    # Independent analytic root + one actuated hinge + fixed nonzero passive hold.
    np.testing.assert_allclose(data.xpos[tip], [math.cos(0.3), math.sin(0.3), 2.0], atol=1e-12)
    np.testing.assert_allclose(
        data.xmat[tip].reshape(3, 3)[:, 0], [math.cos(0.7), math.sin(0.7), 0], atol=1e-12
    )
    np.testing.assert_allclose(model.body_mass[1:], [1, 1, 1])
    assert model.actuator_gainprm[0, 0] == 30
    np.testing.assert_allclose(model.actuator_biasprm[0, :3], [0, -30, -2])
    np.testing.assert_allclose(model.actuator_forcerange[0], [-5, 5])
    assert model.dof_armature[-1] == 0.01
    # MuJoCo diagonalizes the full tensor; rotate it back to the link frame.
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.body_iquat[model.body("arm").id])
    rotation = rotation.reshape(3, 3)
    actual_tensor = rotation @ np.diag(model.body_inertia[model.body("arm").id]) @ rotation.T
    c, s = math.cos(0.2), math.sin(0.2)
    inertial_rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    expected_tensor = inertial_rotation @ np.diag([0.01, 0.02, 0.025]) @ inertial_rotation.T
    np.testing.assert_allclose(actual_tensor, expected_tensor, atol=1e-12)
    assert receipt.scene_sha256 == hashlib.sha256((moved / "scene.xml").read_bytes()).hexdigest()
    document = json.loads((moved / "receipt.json").read_text())
    assert document["profile"]["passive_holds"] == [["passive", 0.4]]
    assert document["embodiment_id"] == str(body.id)
    assert body.to_json() == before


@pytest.mark.parametrize("fault", ["identity", "passive", "outside", "control", "tracked"])
def test_unknown_or_ambiguous_facts_refuse_before_publication(tmp_path, monkeypatch, fault):
    body = fixture_body(tmp_path, monkeypatch)
    p = profile(body)
    if fault == "identity":
        p = replace(p, expected_embodiment_id="0" * 64)
    if fault == "passive":
        p = replace(p, passive_holds=())
    if fault == "outside":
        p = replace(p, passive_holds=(("passive", 2.0),))
    if fault == "control":
        p = replace(p, joints=(replace(p.joints[0], name="unknown"),))
    if fault == "tracked":
        p = replace(p, tracked_links=("arm", "tip", "absent"))
    with pytest.raises(NominalExportError):
        export_nominal_mujoco(body, p, tmp_path / "bad")
    assert not (tmp_path / "bad").exists()


def test_profile_identity_changes_without_mutating_body_and_no_overwrite(tmp_path, monkeypatch):
    body = fixture_body(tmp_path, monkeypatch)
    p = profile(body)
    assert p.sha256 != replace(p, joints=(replace(p.joints[0], kp=31.0),)).sha256
    output = tmp_path / "bundle"
    export_nominal_mujoco(body, p, output)
    with pytest.raises(NominalExportError, match="exists"):
        export_nominal_mujoco(body, p, output)
    assert (output / "scene.xml").is_file()


def test_mesh_bundle_keeps_verified_bytes_and_survives_source_removal(tmp_path, monkeypatch):
    import struct

    body = fixture_body(tmp_path, monkeypatch)
    vertices = ((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.1))
    faces = ((0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3))
    mesh = b"oracle".ljust(80, b"\0") + struct.pack("<I", len(faces))
    for face in faces:
        coords = tuple(x for index in face for x in vertices[index])
        mesh += struct.pack("<12fH", 0.0, 0.0, 0.0, *coords, 0)
    source_path = tmp_path / "assets" / "oracle.stl"
    source_path.write_bytes(mesh)
    urdf = body.urdf_bytes.replace(b'<box size="0.2 0.1 0.1"/>', b'<mesh filename="oracle.stl"/>')
    (tmp_path / "assets" / "oracle.urdf").write_bytes(urdf)
    provenance = body.assets[0].provenance
    body = replace(
        body,
        assets=(
            ProvenancedAsset(
                AssetRef.from_bytes(
                    urdf,
                    uri="package://sx-embodiments/oracle.urdf",
                    format=AssetFormat.URDF,
                    role=AssetRole.DESCRIPTION,
                ),
                provenance,
            ),
            ProvenancedAsset(
                AssetRef.from_bytes(
                    mesh,
                    uri="package://sx-embodiments/oracle.stl",
                    format=AssetFormat.MESH,
                    role=AssetRole.GEOMETRY,
                ),
                replace(provenance, path="oracle.stl"),
            ),
        ),
    )
    output = tmp_path / "bundle"
    receipt = export_nominal_mujoco(body, profile(body), output)
    assert (output / "canonical.urdf").read_bytes() == urdf
    for rel, digest in receipt.files:
        assert hashlib.sha256((output / rel).read_bytes()).hexdigest() == digest
    mesh_files = list((output / "meshes").iterdir())
    assert len(mesh_files) == 1 and mesh_files[0].read_bytes() == mesh
    # A source asset is not required after export; move the temporary fixture tree.
    shutil.move(tmp_path / "assets", tmp_path / "retained_source")
    model = mujoco.MjModel.from_xml_path(str(output / "scene.xml"))
    assert model.nmesh == 1 and model.nu == 1


def test_profile_artifact_roundtrip_and_strict_boundary(tmp_path, monkeypatch):
    from dataclasses import asdict

    from sx_contracts.identity import canonical_json

    from sx_embodiments.nominal_mujoco import nominal_profile_from_json

    original = profile(fixture_body(tmp_path, monkeypatch))
    payload = canonical_json(asdict(original)).encode()
    assert nominal_profile_from_json(payload) == original
    assert nominal_profile_from_json(payload).sha256 == original.sha256
    document = asdict(original)
    document["unknown"] = 1
    with pytest.raises(NominalExportError):
        nominal_profile_from_json(json.dumps(document).encode())
    with pytest.raises(NominalExportError, match="Duplicate"):
        nominal_profile_from_json(b'{"joints":[],"joints":[]}')
    document = asdict(original)
    document["timestep"] = True
    with pytest.raises(NominalExportError):
        nominal_profile_from_json(json.dumps(document).encode())


def test_ground_only_masks_preserve_geometry_and_allow_floor_contacts(tmp_path, monkeypatch):
    from sx_embodiments.nominal_mujoco import CollisionMode

    body = fixture_body(tmp_path, monkeypatch)
    settings = replace(profile(body), collision_mode=CollisionMode.GROUND_ONLY)
    export_nominal_mujoco(body, settings, tmp_path / "ground")
    model = mujoco.MjModel.from_xml_path(str(tmp_path / "ground" / "scene.xml"))
    floor = model.geom("nominal_floor").id
    robot = [i for i in range(model.ngeom) if model.geom_bodyid[i] != 0]
    assert (model.geom_contype[floor], model.geom_conaffinity[floor]) == (1, 2)
    assert all((model.geom_contype[i], model.geom_conaffinity[i]) == (2, 1) for i in robot)
    for a in robot:
        assert (model.geom_contype[a] & model.geom_conaffinity[floor]) != 0
        for b in robot:
            assert (model.geom_contype[a] & model.geom_conaffinity[b]) == 0
    # A real geometric ground intersection still creates contacts.
    data = mujoco.MjData(model)
    data.qpos[2] = 0.02
    mujoco.mj_forward(model, data)
    assert data.ncon > 0
    assert all(floor in (c.geom1, c.geom2) for c in data.contact[: data.ncon])


def test_legacy_collision_profile_keeps_original_semantics(tmp_path, monkeypatch):
    from dataclasses import asdict

    from sx_embodiments.nominal_mujoco import CollisionMode, nominal_profile_from_json

    raw = asdict(profile(fixture_body(tmp_path, monkeypatch)))
    raw.pop("collision_mode")
    assert nominal_profile_from_json(json.dumps(raw).encode()).collision_mode is CollisionMode.FULL
    raw["collision_mode"] = "ignored"
    with pytest.raises(ValueError):
        nominal_profile_from_json(json.dumps(raw).encode())


def test_mixed_rpy_joint_passive_visual_collision_and_inertia_frames(tmp_path, monkeypatch):
    """Independent RzRyRx oracle catches intrinsic/extrinsic errors in every path."""
    import xml.etree.ElementTree as ET

    from sx_embodiments.kinematics import Kinematics

    def rpy(a, b, c):
        ca, sa, cb, sb, cc, sc = (
            math.cos(a),
            math.sin(a),
            math.cos(b),
            math.sin(b),
            math.cos(c),
            math.sin(c),
        )
        rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
        ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
        rz = np.array([[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]])
        return rz @ ry @ rx

    def transform(position, rotation):
        matrix = np.eye(4)
        matrix[:3, :3], matrix[:3, 3] = rotation, position
        return matrix

    body = fixture_body(tmp_path, monkeypatch)
    tree = ET.fromstring(body.urdf_bytes)
    motor_rpy, passive_rpy = (0.4, -0.2, 0.3), (-0.3, 0.1, -0.2)
    inertia_rpy, visual_rpy, collision_rpy = (0.2, -0.3, 0.4), (-0.2, 0.5, 0.1), (0.3, -0.1, 0.2)
    tree.find("joint[@name='motor']/origin").set("rpy", " ".join(map(str, motor_rpy)))
    tree.find("joint[@name='passive']/origin").set("rpy", " ".join(map(str, passive_rpy)))
    for link in tree.findall("link"):
        origin = link.find("inertial/origin")
        origin.set("xyz", ".1 .02 .03")
        origin.set("rpy", " ".join(map(str, inertia_rpy)))
        ET.SubElement(
            link.find("collision"),
            "origin",
            xyz=".02 .03 .04",
            rpy=" ".join(map(str, collision_rpy)),
        )
        visual = ET.SubElement(link, "visual")
        ET.SubElement(visual, "origin", xyz="-.03 .02 .01", rpy=" ".join(map(str, visual_rpy)))
        ET.SubElement(ET.SubElement(visual, "geometry"), "box", size=".08 .06 .04")
    data = ET.tostring(tree)
    (tmp_path / "assets" / "oracle.urdf").write_bytes(data)
    old = body.assets[0]
    new = replace(
        old,
        asset=AssetRef.from_bytes(
            data, uri=old.asset.uri, format=AssetFormat.URDF, role=AssetRole.DESCRIPTION
        ),
    )
    body = replace(body, assets=(new,))
    settings = replace(profile(body), root_quaternion=(math.cos(0.125), 0.0, 0.0, math.sin(0.125)))
    receipt = export_nominal_mujoco(body, settings, tmp_path / "mixed")
    assert receipt.schema == "sx-embodiments.nominal-mujoco/v2"
    assert receipt.rotation_convention == "urdf-rpy-extrinsic-XYZ"
    model = mujoco.MjModel.from_xml_path(str(tmp_path / "mixed" / "scene.xml"))
    state = mujoco.MjData(model)
    q = 0.35
    state.qpos[model.jnt_qposadr[model.joint("motor").id]] = q
    mujoco.mj_forward(model, state)
    base = transform([0, 0, 1], rpy(0, 0, 0.25))
    arm = base @ transform([0, 0, 1], rpy(*motor_rpy)) @ transform([0, 0, 0], rpy(0, 0, q))
    tip = arm @ transform([1, 0, 0], rpy(*passive_rpy)) @ transform([0, 0, 0], rpy(0, 0, 0.4))
    owner = Kinematics.of(data)
    for name, expected in [("base", base), ("arm", arm), ("tip", tip)]:
        body_id = model.body(name).id
        np.testing.assert_allclose(state.xpos[body_id], expected[:3, 3], atol=1e-12)
        np.testing.assert_allclose(state.xmat[body_id].reshape(3, 3), expected[:3, :3], atol=1e-12)
        owner_pose = base @ np.asarray(
            owner.pose("base", name, {"motor": q, "passive": 0.4})
        ).reshape(4, 4)
        np.testing.assert_allclose(expected, owner_pose, atol=1e-12)
        for kind, angles, position in [
            ("visual", visual_rpy, [-0.03, 0.02, 0.01]),
            ("collision", collision_rpy, [0.02, 0.03, 0.04]),
        ]:
            geometry = model.geom(f"{name}/{kind}/0").id
            wanted = expected @ transform(position, rpy(*angles))
            np.testing.assert_allclose(state.geom_xpos[geometry], wanted[:3, 3], atol=1e-12)
            np.testing.assert_allclose(
                state.geom_xmat[geometry].reshape(3, 3), wanted[:3, :3], atol=1e-12
            )
        np.testing.assert_allclose(
            state.xipos[body_id], (expected @ [0.1, 0.02, 0.03, 1])[:3], atol=1e-12
        )
        inertia_rotation = expected[:3, :3] @ rpy(*inertia_rpy)
        wanted_tensor = inertia_rotation @ np.diag([0.01, 0.02, 0.025]) @ inertia_rotation.T
        actual_rotation = state.ximat[body_id].reshape(3, 3)
        actual_tensor = actual_rotation @ np.diag(model.body_inertia[body_id]) @ actual_rotation.T
        # MuJoCo diagonalizes fullinertia numerically; retain a tight absolute
        # tensor tolerance while body/visual/collision frames above use 1e-12.
        np.testing.assert_allclose(actual_tensor, wanted_tensor, atol=5e-10, rtol=0)
