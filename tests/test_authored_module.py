"""An authored module (a plate from photos and measurements) admits, mates and refuses."""

from __future__ import annotations

from typing import Any, cast

import pytest

from sx_embodiments import admit_module
from sx_embodiments.composition import compose_build
from sx_embodiments.errors import (
    AssemblyError,
    EmbodimentSchemaError,
    InterfaceMismatchError,
    SocketError,
)
from sx_embodiments.known.universal_robots import UR5E_MODULE
from sx_embodiments.mechanical import Build, InterfaceId, Mate, Placed, Socket

ISO = InterfaceId("iso-9409-1-50-4-m6")
CAMERA = InterfaceId("unc-1-4-20")

PLATE = b"""<?xml version="1.0"?>
<robot name="adapter_plate">
  <link name="mount">
    <inertial><origin xyz="0 0 0.003"/><mass value="0.078"/>
      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="9e-5" iyz="0" izz="1e-4"/></inertial>
    <visual><geometry><mesh filename="meshes/adapter_plate.stl"/></geometry></visual>
  </link>
  <link name="top"/>
  <joint name="top_face" type="fixed">
    <parent link="mount"/><child link="top"/><origin xyz="0 0 0.006"/>
  </joint>
</robot>
"""
MESHES = frozenset({"meshes/adapter_plate.stl"})


def _plate(**changes: object):
    arguments: dict[str, object] = {
        "urdf": PLATE,
        "plug": ISO,
        "sockets": (Socket("camera", CAMERA, "top"),),
        "mesh_paths": MESHES,
    }
    arguments.update(changes)
    return admit_module("adapter-plate", "Adapter plate", **cast("dict[str, Any]", arguments))


def test_a_passive_plate_mates_on_the_ur5e_flange() -> None:
    plate = _plate()
    assert plate.plug == ISO and plate.components == ()
    assert plate.socket("camera").carrier is None
    build = Build(
        placed=(Placed("arm", UR5E_MODULE),),
        mates=(Mate("plate", plate, "arm", "flange"),),
        robot="ur5e-plate",
        root="world",
    )
    urdf = build.assembly().urdf().decode()
    assert '<link name="plate_mount"' in urdf and '<child link="plate_mount"' in urdf
    body = compose_build(build, name="ur5e-plate", label="UR5e with plate")
    # The plate rides on the arm and adds no coordinate.
    assert len(body.state.coordinates) == 6


def test_a_plate_with_another_plug_is_refused_on_the_flange() -> None:
    plate = _plate(plug=CAMERA)
    with pytest.raises(InterfaceMismatchError):
        Build(
            placed=(Placed("arm", UR5E_MODULE),),
            mates=(Mate("plate", plate, "arm", "flange"),),
            robot="r",
            root="world",
        )


_REFUSALS: list[tuple[dict[str, object], type[Exception]]] = [
    ({"plug": InterfaceId("tenant-made-up")}, AssemblyError),
    ({"sockets": (Socket("camera", CAMERA, "nowhere"),)}, SocketError),
    ({"mesh_paths": frozenset()}, EmbodimentSchemaError),
    ({"urdf": PLATE.replace(b'type="fixed"', b'type="revolute"')}, AssemblyError),
    ({"urdf": PLATE.replace(b'value="0.078"', b'value="0"')}, EmbodimentSchemaError),
    ({"urdf": PLATE.replace(b'value="0.078"', b'value="nan"')}, EmbodimentSchemaError),
]


@pytest.mark.parametrize(("changes", "error"), _REFUSALS)
def test_an_authored_module_is_refused_by_type(
    changes: dict[str, object], error: type[Exception]
) -> None:
    with pytest.raises(error):
        _plate(**changes)
