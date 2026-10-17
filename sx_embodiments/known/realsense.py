"""Intel RealSense D405 and D435 as camera modules, from Intel's own descriptions.

``IntelRealSense/realsense-ros`` at ``a34e2a9dbc748430692fa2e170598eec256cf7a9``,
``realsense2_description/urdf/_d405.urdf.xacro`` and ``_d435.urdf.xacro`` (Apache-2.0), with
``use_nominal_extrinsics`` and without the mesh: the body is the collision box Intel declares,
rooted at the bottom 1/4-20 screw (the module's plug), with Intel's nominal depth, infrared and
colour frames and their optical frames. The inertial values are Intel's; Intel marks them as not
reliable for modelling. A module carries a camera stream only when its optics are given, since a
unit's intrinsics are its calibration and the registry pins no model-nominal profile for these.
"""

import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Final, Literal

from ..compose import RootMount, sensor_component
from ..mechanical import InterfaceId, Module, ModuleId
from ..parts import CameraSpec

Stream = Literal["depth", "infra1", "infra2", "color"]


@dataclass(frozen=True, slots=True)
class _Geometry:
    """The xacro's properties, in its own names."""

    width: float
    height: float
    depth: float
    mount_from_center_offset: float
    zero_depth_to_glass: float
    depth_py: float
    infra2_offset: float
    color_offset: float
    collision_x: float

    @property
    def mesh_x_offset(self) -> float:
        return self.mount_from_center_offset - 0.1e-3 - self.zero_depth_to_glass


_MODELS: Final[Mapping[str, tuple[str, _Geometry]]] = {
    "d405": (
        "Intel RealSense D405",
        _Geometry(
            width=0.042,
            height=0.042,
            depth=0.023,
            mount_from_center_offset=0.01465,
            zero_depth_to_glass=3.7e-3,
            depth_py=0.009,
            infra2_offset=-0.018,
            color_offset=0.0,
            collision_x=3.7e-3 - 0.023 / 2,
        ),
    ),
    "d435": (
        "Intel RealSense D435",
        _Geometry(
            width=0.090,
            height=0.025,
            depth=0.02505,
            mount_from_center_offset=0.0149,
            zero_depth_to_glass=4.2e-3,
            depth_py=0.0175,
            infra2_offset=-0.050,
            color_offset=0.015,
            collision_x=0.0,
        ),
    ),
}

_OPTICAL: Final = f"{-math.pi / 2!r} 0 {-math.pi / 2!r}"


def _number(value: float) -> str:
    return repr(float(value)).removesuffix(".0")


def _fixed(robot: ET.Element, parent: str, child: str, xyz: str, rpy: str = "0 0 0") -> None:
    ET.SubElement(robot, "link", {"name": child})
    joint = ET.SubElement(robot, "joint", {"name": f"{child}_joint", "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})


def _description(model: str, geometry: _Geometry) -> bytes:
    robot = ET.Element("robot", {"name": f"realsense_{model}"})
    ET.SubElement(robot, "link", {"name": "bottom_screw_frame"})
    _fixed(
        robot,
        "bottom_screw_frame",
        "camera_link",
        " ".join(
            _number(v) for v in (geometry.mesh_x_offset, geometry.depth_py, geometry.height / 2)
        ),
    )
    body = robot.find("link[@name='camera_link']")
    assert body is not None
    collision = ET.SubElement(body, "collision")
    ET.SubElement(
        collision,
        "origin",
        {"xyz": f"{_number(geometry.collision_x)} {_number(-geometry.depth_py)} 0", "rpy": "0 0 0"},
    )
    shape = ET.SubElement(collision, "geometry")
    ET.SubElement(
        shape,
        "box",
        {"size": " ".join(_number(v) for v in (geometry.depth, geometry.width, geometry.height))},
    )
    inertial = ET.SubElement(body, "inertial")
    ET.SubElement(inertial, "mass", {"value": "0.072"})
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0"})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": "0.003881243",
            "ixy": "0.0",
            "ixz": "0.0",
            "iyy": "0.000498940",
            "iyz": "0.0",
            "izz": "0.003879257",
        },
    )
    offsets = {
        "depth": 0.0,
        "infra1": 0.0,
        "infra2": geometry.infra2_offset,
        "color": geometry.color_offset,
    }
    for stream, offset in offsets.items():
        _fixed(robot, "camera_link", f"{stream}_frame", f"0 {_number(offset)} 0")
        _fixed(robot, f"{stream}_frame", f"{stream}_optical_frame", "0 0 0", _OPTICAL)
    ET.indent(robot, space="  ")
    return ET.tostring(robot, encoding="utf-8", xml_declaration=True) + b"\n"


def realsense_module(
    model: Literal["d405", "d435"], streams: Mapping[Stream, CameraSpec] | None = None
) -> Module:
    """A RealSense on its 1/4-20 thread; each given stream is a camera on its optical frame."""
    label, geometry = _MODELS[model]
    return Module(
        module_id=ModuleId(f"realsense-{model}"),
        label=label,
        description=_description(model, geometry),
        plug=InterfaceId("unc-1-4-20"),
        components=tuple(
            sensor_component(stream, camera, RootMount(f"{stream}_optical_frame"))
            for stream, camera in (streams or {}).items()
        ),
    )
