"""The pack builder reads every mesh format and collision shape the registry's descriptions
use: STL, OBJ and COLLADA meshes; boxes, cylinders, spheres and capsules. The bodies that need
COLLADA or capsules are built here; the Catalog's conformance test builds every body.
"""

import json

import numpy as np
import pytest

from sx_embodiments import embodiments, viewing_of
from sx_embodiments.render_pack import _read_dae, build, capsule_mesh


#: Bodies drawn from COLLADA meshes (all five) and a capsule collision (rby1).
@pytest.mark.parametrize("name", ["rby1", "ur5e", "franka"])
def test_a_body_drawn_from_collada_builds_its_pack(name: str) -> None:
    body = embodiments[name]
    pack = build(body, name=name, viewing=viewing_of(name))
    document = json.loads(pack.document)
    assert document["embodiment_digest"] == str(body.id)
    assert 0 < document["triangles"] <= viewing_of(name).triangle_budget


COLLADA = b"""<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="millimetre" meter="0.001"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries>
    <geometry id="square"><mesh>
      <source id="p"><float_array id="pa" count="12">0 0 0 10 0 0 10 10 0 0 10 0</float_array>
        <technique_common><accessor source="#pa" count="4" stride="3"/></technique_common>
      </source>
      <vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
      <polylist count="1"><input semantic="VERTEX" source="#v" offset="0"/>
        <vcount>4</vcount><p>0 1 2 3</p></polylist>
    </mesh></geometry>
  </library_geometries>
  <library_visual_scenes><visual_scene id="s">
    <node id="n"><translate>0 0 5</translate><instance_geometry url="#square"/></node>
  </visual_scene></library_visual_scenes>
</COLLADA>"""


def test_a_collada_mesh_is_read_in_metres_placed_by_its_node_with_polygons_fanned() -> None:
    ((rgba, corners),) = _read_dae(COLLADA, "square.dae")
    assert rgba is None  # it binds no material: the URDF's colour or grey draws it
    assert corners.shape == (6, 3)  # one quad, two triangles
    assert np.allclose(corners[:, 2], 0.005)  # translated 5 mm up, in metres
    assert np.isclose(corners[:, 0].max(), 0.01)


def test_a_capsule_is_a_cylinder_with_round_caps() -> None:
    mesh = capsule_mesh(0.05, 0.2)
    assert np.isclose(mesh.vertices[:, 2].max(), 0.15) and np.isclose(
        mesh.vertices[:, 2].min(), -0.15
    )
    assert np.isclose(np.abs(mesh.vertices[:, 0]).max(), 0.05)


def test_a_collada_mesh_keeps_its_material_colour_and_turns_its_up_axis_to_z() -> None:
    document = (
        COLLADA.replace(b"<up_axis>Z_UP</up_axis>", b"<up_axis>X_UP</up_axis>")
        .replace(
            b"<library_geometries>",
            b"""<library_effects><effect id="red-fx"><profile_COMMON><technique sid="c"><lambert>
              <diffuse><color>0.8 0.1 0.1 1</color></diffuse></lambert></technique>
            </profile_COMMON></effect></library_effects>
            <library_materials><material id="red"><instance_effect url="#red-fx"/></material>
            </library_materials><library_geometries>""",
        )
        .replace(b'<polylist count="1">', b'<polylist count="1" material="paint">')
        .replace(
            b'<instance_geometry url="#square"/>',
            b"""<instance_geometry url="#square"><bind_material><technique_common>
              <instance_material symbol="paint" target="#red"/></technique_common></bind_material>
            </instance_geometry>""",
        )
    )
    ((rgba, corners),) = _read_dae(document, "square.dae")
    assert rgba == (0.8, 0.1, 0.1, 1.0)
    # X_UP (COLLADA 1.4): the file's x is up, its y is left, its z toward the viewer; in URDF
    # Z-up those are +z, -x and -y.
    assert np.isclose(corners[:, 2].max(), 0.01) and np.isclose(corners[:, 2].min(), 0.0)
    assert np.isclose(corners[:, 0].min(), -0.01) and np.isclose(corners[:, 0].max(), 0.0)
    assert np.allclose(corners[:, 1], -0.005)
