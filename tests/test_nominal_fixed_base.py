"""Fixed-base linear motion and affine source coupling are real model constraints."""

import json
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
    CollisionMode,
    NominalExportError,
    NominalJoint,
    NominalProfile,
    RootMode,
    export_nominal_mujoco,
    nominal_profile_from_json,
)
from sx_embodiments.parts import ChainSpec, Limb


def fixture(tmp_path, monkeypatch):
    inertia = (
        '<inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01" '
        'ixy="0" ixz="0" iyz="0"/></inertial>'
    )
    xml = (
        '<robot name="linear">'
        + "".join(f'<link name="{name}">{inertia}</link>' for name in ("base", "drive", "follower"))
        + '<joint name="motor" type="prismatic"><parent link="base"/><child link="drive"/>'
        '<axis xyz="1 0 0"/><limit lower="-.06" upper=".06" effort="10" velocity="1"/></joint>'
        + '<joint name="follower" type="prismatic"><parent link="base"/><child link="follower"/>'
        '<origin xyz="0 .2 0"/><axis xyz="1 0 0"/>'
        '<limit lower="0" upper=".2" effort="10" velocity="1"/>'
        '<mimic joint="motor" multiplier="-2" offset=".1"/></joint></robot>'
    ).encode()
    assets = tmp_path / "assets"
    assets.mkdir()
    (assets / "linear.urdf").write_bytes(xml)
    monkeypatch.setenv("SX_EMBODIMENTS_ASSETS", str(assets))
    asset = ProvenancedAsset(
        AssetRef.from_bytes(
            xml,
            uri="package://sx-embodiments/linear.urdf",
            format=AssetFormat.URDF,
            role=AssetRole.DESCRIPTION,
        ),
        AssetProvenance("https://example.com/linear", "v1", "linear.urdf", "MIT"),
    )
    part = ChainSpec(
        PartId("linear"),
        JointLayout((JointAxis("motor", CoordinateUnit.METER, Bounds(-0.05, 0.05)),)),
        (0.0,),
    )
    body = Embodiment(
        EmbodimentName("linear"),
        "Linear",
        EmbodimentKind.ROBOT,
        Lineage("test"),
        (body_component("arm", part, RootMount("base"), limb=Limb.ARM),),
        (asset,),
    )
    profile = NominalProfile(
        str(body.id),
        (NominalJoint("motor", 500.0, 80.0, 20.0, 0.01, 0.0, 0.01),),
        (),
        ("drive", "follower"),
        (),
        (0.0, 0.0, 0.5),
        (1.0, 0.0, 0.0, 0.0),
        0.002,
        (0.0, 0.0, 0.0),
        (1.0, 0.005, 0.0001),
        ("Synthetic nominal fixture; no hardware facts.",),
        CollisionMode.GROUND_ONLY,
        RootMode.FIXED,
    )
    return body, profile


def test_fixed_slide_and_affine_mimic_follow_source_under_physics(tmp_path, monkeypatch):
    body, profile = fixture(tmp_path, monkeypatch)
    destination = tmp_path / "bundle"
    export_nominal_mujoco(body, profile, destination)
    model = mujoco.MjModel.from_xml_path(str(destination / "scene.xml"))
    data = mujoco.MjData(model)
    assert model.nq == 2 and model.nu == 1 and model.neq == 1
    assert np.all(model.jnt_type == mujoco.mjtJoint.mjJNT_SLIDE)
    np.testing.assert_allclose(model.jnt_range[model.joint("motor").id], [-0.05, 0.05])
    drive = model.jnt_qposadr[model.joint("motor").id]
    follow = model.jnt_qposadr[model.joint("follower").id]
    data.qpos[drive] = 0.025
    data.qpos[follow] = 0.05
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(data.xpos[model.body("drive").id], [0.025, 0, 0.5], atol=1e-12)
    np.testing.assert_allclose(data.xpos[model.body("follower").id], [0.05, 0.2, 0.5], atol=1e-12)
    data.ctrl[0] = -0.025
    for _ in range(1500):
        mujoco.mj_step(model, data)
    assert abs(data.qpos[drive] + 0.025) < 1e-5
    assert abs(data.qpos[follow] - (0.1 - 2 * data.qpos[drive])) < 1e-5
    assert not any(w.number for w in data.warning)
    document = json.loads((destination / "receipt.json").read_text())
    assert nominal_profile_from_json(json.dumps(document["profile"]).encode()) == profile


def test_native_limits_may_restrict_but_never_extend_source(tmp_path, monkeypatch):
    body, profile = fixture(tmp_path, monkeypatch)
    component = body.components[0]
    part = replace(
        component.part,
        layout=JointLayout((JointAxis("motor", CoordinateUnit.METER, Bounds(-0.07, 0.07)),)),
    )
    changed = replace(
        body, components=(body_component("arm", part, RootMount("base"), limb=Limb.ARM),)
    )
    with pytest.raises(NominalExportError, match="extend outside"):
        export_nominal_mujoco(
            changed, replace(profile, expected_embodiment_id=str(changed.id)), tmp_path / "bad"
        )
    assert not (tmp_path / "bad").exists()


def test_empty_contacts_only_valid_for_fixed_root(tmp_path, monkeypatch):
    _, profile = fixture(tmp_path, monkeypatch)
    with pytest.raises(NominalExportError, match="Contact"):
        replace(profile, root_mode=RootMode.FLOATING)
