"""An authored module (a plate from photos and measurements) admits, mates and refuses."""

from __future__ import annotations

import struct
from pathlib import Path
from typing import Any, cast

import pytest

import sx_embodiments
from sx_embodiments import admit_module, part_to_dict
from sx_embodiments.assemble import admit_base, module_from_dict, module_to_dict
from sx_embodiments.checks import CollisionGeometryError
from sx_embodiments.compose import Component
from sx_embodiments.composition import compose_build
from sx_embodiments.errors import (
    AssemblyError,
    EmbodimentSchemaError,
    InterfaceMismatchError,
    SocketError,
)
from sx_embodiments.known.so101 import SO101_MODULE
from sx_embodiments.known.universal_robots import UR5E_MODULE
from sx_embodiments.layout import UndocumentedDrive
from sx_embodiments.mechanical import (
    MECHANICAL_INTERFACES,
    Box,
    Build,
    InterfaceId,
    Mate,
    Placed,
    Socket,
    rigid_module,
)
from sx_embodiments.parts import MobileBaseSpec

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


def test_an_authored_module_round_trips_through_its_document() -> None:
    plate = _plate()
    document = module_to_dict(plate)

    assert document["plug"] == ISO and document["part"] is None
    again = module_from_dict(document, urdf=PLATE, mesh_paths=MESHES)
    assert again == plate
    with pytest.raises(EmbodimentSchemaError):
        module_from_dict({**document, "schema": "sx.authored-module/v0"}, urdf=PLATE)


def test_the_iso_flange_pattern_is_the_one_its_id_names() -> None:
    """iso-9409-1-<pitch>-<count>-m<thread>: the id and the recorded pattern agree."""
    for interface, record in MECHANICAL_INTERFACES.items():
        if not interface.startswith("iso-9409-1-"):
            continue
        pitch, count, thread = interface.removeprefix("iso-9409-1-").split("-")
        assert record.pattern is not None
        assert record.pattern.pitch_circle_diameter_mm == float(pitch)
        assert record.pattern.count == int(count)
        assert record.pattern.thread.lower() == thread
        assert record.pattern.thread_diameter_mm == float(thread.removeprefix("m"))


def test_an_authored_base_is_admitted_from_its_specs_and_never_qualified() -> None:
    from test_mobile_base import four_arm_base

    base = four_arm_base()
    module = admit_base(base, urdf=cast(bytes, base.module.description))

    assert module == base.module
    (drive,) = [entry for entry in module.components if isinstance(entry, Component)]
    assert all(
        isinstance(axis.actuation, UndocumentedDrive)
        for axis in cast(MobileBaseSpec, drive.part).layout.axes
    )
    document = module_to_dict(module)
    assert document["base"] is not None and document["part"] is None
    assert module_from_dict(document, urdf=cast(bytes, module.description)) == module
    with pytest.raises(EmbodimentSchemaError, match="rendering of its specs"):
        admit_base(base, urdf=PLATE)


def test_a_chain_or_a_base_part_is_not_an_authored_module() -> None:
    from sx_embodiments.known.so101 import SO101_ARM
    from test_mobile_base import four_arm_base

    for part in (SO101_ARM, four_arm_base().spec):
        with pytest.raises(AssemblyError):
            _plate(part=part_to_dict(part))


def test_the_so101_wrist_seat_sits_between_its_holes_on_the_wrist_roll_face() -> None:
    """The socket frame is on the outer face of the mesh the description draws, and a holder
    plugged by so101-wrist mates there riding on the arm."""
    socket = SO101_MODULE.socket("wrist")
    assert (socket.interface, socket.link, socket.carrier) == ("so101-wrist", "gripper_link", "arm")
    vertices = _stl_vertices(
        Path(sx_embodiments.__file__).parent.parent
        / "assets/so101/meshes/wrist_roll_follower_so101_v1.stl"
    )
    # The description places the mesh at (0, -0.000218, 0.00095), rotated pi about x.
    placed = [(x, -y - 0.000218214, -z + 0.000949706) for x, y, z in vertices]
    x0, y0, z0 = socket.pose.xyz
    face = [y for x, y, z in placed if abs(x - x0) < 0.004 and abs(z - z0) < 0.004]
    assert face and abs(max(face) - y0) < 0.0002
    holder = rigid_module(
        "wrist-holder",
        "Wrist holder",
        shape=Box((0.03, 0.02, 0.004)),
        mass_kg=0.01,
        plug=InterfaceId("so101-wrist"),
    )
    build = Build(
        placed=(Placed("arm", SO101_MODULE),),
        mates=(Mate("holder", holder, "arm", "wrist"),),
        robot="so101-holder",
        root="world",
    )
    assert b'<parent link="arm_gripper_link"' in build.assembly().urdf()
    pattern = MECHANICAL_INTERFACES[InterfaceId("so101-wrist")].pattern
    assert pattern is not None and (pattern.count, pattern.pitch_circle_diameter_mm) == (2, 8.1)


def _stl_vertices(path: Path) -> list[tuple[float, float, float]]:
    data = path.read_bytes()
    (count,) = struct.unpack_from("<I", data, 80)
    points: list[tuple[float, float, float]] = []
    for index in range(count):
        offset = 84 + index * 50 + 12
        for corner in range(3):
            x, y, z = struct.unpack_from("<3f", data, offset + corner * 12)
            points.append((x, y, z))
    return points


@pytest.mark.parametrize(
    "shape",
    (
        '<box size="0.1 0.1"/>',
        '<box size="0.1 -0.1 0.1"/>',
        '<cylinder radius="nan" length="0.1"/>',
        '<sphere radius="wide"/>',
        "<mesh/>",
    ),
)
def test_a_collision_shape_the_checks_cannot_read_is_refused(shape: str) -> None:
    broken = PLATE.replace(
        b"</visual>",
        b"</visual><collision><geometry>" + shape.encode() + b"</geometry></collision>",
    )
    with pytest.raises(CollisionGeometryError, match="collision"):
        _plate(urdf=broken)
