#!/usr/bin/env python3
"""Build the Sentient static upper body URDF from its source description.

The source is ``assets/sentient_static_upper_body/source/allisa_bot.urdf``, byte for byte: the
description the robot's operator selected on 2026-10-25, as Subraj's ``humanoid_pkg`` ships it
(``urdf/allisa_bot.urdf``, sha256 ``039b0b7d…``). That file is allisa_bot's CAD export (source
sha256 ``623305bb…``) with each of its 25 movable joints given the exact limits of the earlier
``allisa_feedback`` description, which "the operator rejected" for its geometry and keeps only
as a joint-limit donor (``source/allisa_bot_integration.md`` and
``source/allisa_bot_model_provenance.json``, copied beside it). Its meshes are the package's
display meshes, decimated from the CAD with original units and frames (the provenance file
records each one's source and display sha256 and bounds change).

The rendering is the source plus only what simulation and the registry need; the changes, and
the evidence that would replace each, are in ``known/static_upper_body.py``:

* every link with geometry gets its visual meshes' convex hull as its collision (the source
  has none) and that hull's mass properties at one density (it has no inertials);
* visual names are made unique and meshes are referenced as ``meshes/<name>.stl``;
* the four wrist pitch and roll joints' limits are negated: allisa_bot reverses those axes
  against the motors, whose confirmed limits the donor carries in motor coordinates, and the
  hardware profile reads each of those motors with sign -1;
* the ``<sensor>`` elements (Gazebo's, not URDF's) are removed; the registry declares the ZED;
* the SO-101 neck below its base is TheRobotStudio's official new-calibration chain, with
  allisa_bot's camera holder, bracket and ZED on it (the comment above ``NECK_LINKS``);
* a ``ZED_camera`` frame (Stereolabs' ROS convention, x out of the lenses, y left) and both
  eyes' optical frames, 60 mm either side of it, are added on the source's
  ``zed2i_left_optical_frame``, so the left eye's frame is the source's own.

Run from the package root:
``uv run --with numpy --with scipy python tools/compose_static_upper_body_urdf.py``.
``tests/test_static_upper_body.py`` re-renders and compares bytes.
"""

from __future__ import annotations

import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import ConvexHull  # pyright: ignore[reportMissingTypeStubs]  # no stubs

from sx_embodiments.assembly import Pose
from sx_embodiments.known.zed import EYES, OPTICAL_RPY, eye_offset, optical_frame

ROOT = Path(__file__).resolve().parents[1]
ASSET = ROOT / "assets/sentient_static_upper_body"
SOURCE = ASSET / "source/allisa_bot.urdf"
#: TheRobotStudio's SO-101 in its new calibration, byte for byte (see the neck, below).
SO101 = ASSET / "source/so101_new_calib.urdf"
SO101_MESHES = "meshes/so101/"
TARGET = ASSET / "sentient_static_upper_body.urdf"
SOURCE_MESH_PREFIX = "package://humanoid_pkg/meshes/allisa_bot/"
#: The source's frame on the ZED's left eye, optical convention (z out of the lens).
SOURCE_ZED_EYE = "zed2i_left_optical_frame"
#: The neck's camera frame, a ZED 2i in Stereolabs' ROS convention (x out of the lenses, y left).
ZED_FRAME = "ZED_camera"
#: The joints allisa_bot turns against their motors; their donor limits are motor coordinates.
REVERSED_JOINTS = (
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
)

# One density for every link's convex hull. The hulls overstate solid volume, so this is
# below aluminium; it is an estimate for simulation until the CAD's mass properties arrive.
HULL_DENSITY_KG_M3 = 600.0

Vec = NDArray[np.float64]

# The neck. allisa_bot's SO-101 (the same CAD export the feedback file had) has arbitrary joint
# zeros and links up to 29 mm apart along the pitch axes where the arm's parts cannot be; the
# robot's servos are calibrated by LeRobot, whose degree reading is zero at each servo's
# calibrated mid-range. TheRobotStudio's ``so101_new_calib`` (SO-ARM100 commit 5f6d2b8,
# Apache-2.0) puts every joint's zero there, so the neck from ``so101_shoulder_pan_joint`` down
# is that file's chain under our names: joint origins, axes and limits, and each link's
# inertial, visual and collision meshes (vendored byte for byte under ``meshes/so101/``). A
# LeRobot degree reading is the joint angle with no offset. Each part of the official chain was
# registered by ICP onto the matching part of allisa_bot's CAD (0.4..0.9 mm rms), which places
# what the official file cannot know:
#
# * Where the base sits on the chest. allisa_bot's ``so101_base_link`` frame is the official
#   base frame translated by ``SO101_BASE_IN_RECEIVED`` (axes parallel within 0.05 deg), and its
#   meshes (base, pan servo, driver board as built) stay, moved into the official frame, on the
#   received footprint and facing the received way. An earlier half turn of the base about its
#   plate centre, fitted to a side photo, put the base facing backwards; the operator at the robot
#   corrected it on 2026-10-26 (the base faces as allisa_bot places it). Turning about the
#   vertical is invisible to gravity, so the IMU check below cannot see this one; eyes can.
# * How the camera sits on the last servo. allisa_bot's ``so101_wrist_roll_link`` frame is the
#   official ``gripper_link`` frame (within 0.4 mm and 0.3 deg), so its custom holder for the
#   camera servo (``RECEIVED_CAMERA_HOLDER``), which replaces the official fixed jaw, keeps its
#   placement there. The official ``gripper`` joint, whose servo now tilts the camera, is
#   ``so101_camera_tilt_joint``; its child ``camera_link`` (the camera bracket, the ZED 2i
#   meshes and the left eye's optical frame) is allisa_bot's, moved onto the official moving
#   jaw's frame (allisa_bot's tilt axis is the official jaw's, 3.4 mm further along it). The
#   bracket is not the jaw, so how it sits on the servo horn is this build's fact, which no
#   upstream file states: ``CAMERA_MOUNT_TILT`` is allisa_bot's tilt angle that the official
#   zero takes: the camera turns only about the tilt axis, and this angle puts the ZED 2i's
#   body upright, facing forward and 7.8 deg nose down at the recorded neck pose of 2026-10-26,
#   as the operator sees it and matching the side photo. At this angle the ``ZED_camera`` frame
#   comes out upside down against the ZED's own meshes (its z points down), so the frame built
#   from allisa_bot's left optical frame, and the IMU mounted on it, carry a convention error
#   that is still open; the IMU's reported pitch sign is wrong for the same reason.
NECK_LINKS = {
    "base_link": "so101_base_link",
    "shoulder_link": "so101_shoulder_pan_link",
    "upper_arm_link": "so101_shoulder_lift_link",
    "lower_arm_link": "so101_elbow_flex_link",
    "wrist_link": "so101_wrist_flex_link",
    "gripper_link": "so101_wrist_roll_link",
    "moving_jaw_so101_v1_link": "camera_link",
}
NECK_JOINTS = {
    "shoulder_pan": "so101_shoulder_pan_joint",
    "shoulder_lift": "so101_shoulder_lift_joint",
    "elbow_flex": "so101_elbow_flex_joint",
    "wrist_flex": "so101_wrist_flex_joint",
    "wrist_roll": "so101_wrist_roll_joint",
    "gripper": "so101_camera_tilt_joint",
}
#: Official links whose received CAD meshes stand in for the official ones: the base as
#: mounted on the chest, and the camera mount in place of the moving jaw.
NECK_KEEPS_RECEIVED_MESHES = ("so101_base_link", "camera_link")
#: The official fixed jaw, which this build replaces with the received holder.
NECK_DROPPED_MESHES = ("wrist_roll_follower_so101_v1.stl",)
#: The received holder that carries the camera tilt servo where the official fixed jaw was.
RECEIVED_CAMERA_HOLDER = "Wrist_Roll_Follower_SO101_5__Fillet1.stl"
SO101_BASE_IN_RECEIVED = (0.013891, -0.031756, -0.005189)
CAMERA_MOUNT_TILT = math.radians(141.71)
#: The camera tilt's range. The official joint's limits are its jaw's; with the bracket on the
#: horn the camera meets the wrist between about +10 and +130 deg (a MuJoCo sweep of the
#: rendered collisions from the recorded rest), so the range is the free arc it rests in, kept
#: 25 deg clear of that contact for the fitted mount angle's uncertainty.
CAMERA_TILT_RANGE = ("-3.14159265", "-0.261799388")


def _floats(text: str | None, default: tuple[float, float, float]) -> Vec:
    return np.array([float(v) for v in text.split()] if text else default, dtype=np.float64)


def _rpy(rpy: Vec) -> Vec:
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _negated(number: str) -> str:
    """``-number``, spelled with the source's digits."""
    return number.removeprefix("-") if number.startswith("-") else f"-{number}"


def _pose(pose: Pose) -> Vec:
    out = np.eye(4)
    out[:3, :3] = _rpy(np.array(pose.rpy, dtype=np.float64))
    out[:3, 3] = pose.xyz
    return out


def _matrix(origin: ET.Element | None) -> Vec:
    xyz = _floats(origin.get("xyz") if origin is not None else None, (0, 0, 0))
    rpy = _floats(origin.get("rpy") if origin is not None else None, (0, 0, 0))
    out = np.eye(4)
    out[:3, :3] = _rpy(rpy)
    out[:3, 3] = xyz
    return out


def _set_origin(element: ET.Element, matrix: Vec) -> None:
    """Write ``matrix`` as ``element``'s origin, URDF's fixed-axis roll, pitch, yaw."""
    r = matrix[:3, :3]
    rpy = (
        math.atan2(r[2, 1], r[2, 2]),
        -math.asin(max(-1.0, min(1.0, r[2, 0]))),
        math.atan2(r[1, 0], r[0, 0]),
    )
    xyz = np.where(np.abs(matrix[:3, 3]) < 1e-12, 0.0, matrix[:3, 3])
    origin = element.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        element.insert(0, origin)
    origin.attrib = {"xyz": _fmt(xyz), "rpy": _fmt(np.array(rpy))}


def _rz(angle: float) -> Vec:
    out = np.eye(4)
    out[:3, :3] = _rpy(np.array([0.0, 0.0, angle]))
    return out


def _graft_neck(robot: ET.Element) -> None:
    """Replace the received SO-101 below its base with the official new-calibration chain."""
    official = ET.parse(SO101).getroot()
    links = {link.get("name"): link for link in robot.findall("link")}
    joints = {joint.get("name"): joint for joint in robot.findall("joint")}

    # The base: the official frame, the received meshes moved into it, the mount moved onto it.
    base_in_received = np.eye(4)
    base_in_received[:3, 3] = SO101_BASE_IN_RECEIVED
    received_in_base = np.linalg.inv(base_in_received)
    for visual in links["so101_base_link"].findall("visual"):
        _set_origin(visual, received_in_base @ _matrix(visual.find("origin")))
    mount = joints["so101_mount_joint"]
    _set_origin(mount, _matrix(mount.find("origin")) @ base_in_received)

    # The camera mount on the official jaw frame, from the received tilt joint (whose parent
    # frame is the official gripper_link's) at the angle the official zero takes.
    jaw = next(j for j in official.findall("joint") if j.get("name") == "gripper")
    received_tilt = joints[NECK_JOINTS["gripper"]]
    camera_in_jaw = (
        np.linalg.inv(_matrix(jaw.find("origin")))
        @ _matrix(received_tilt.find("origin"))
        @ _rz(CAMERA_MOUNT_TILT)
    )
    camera = links["camera_link"]
    for visual in camera.findall("visual"):
        _set_origin(visual, camera_in_jaw @ _matrix(visual.find("origin")))
    zed = joints["zed2i_left_optical_joint"]
    _set_origin(zed, camera_in_jaw @ _matrix(zed.find("origin")))
    holder = links["so101_wrist_roll_link"].findall("visual")

    # Every official link below the base, under its received name, in the received link's place.
    for name in NECK_JOINTS.values():
        robot.remove(joints[name])
    for index, material in enumerate(official.findall("material")):
        robot.insert(index, material)  # the official visuals name these
    for official_name, name in NECK_LINKS.items():
        if name in NECK_KEEPS_RECEIVED_MESHES:
            continue
        source = next(e for e in official.findall("link") if e.get("name") == official_name)
        link = ET.Element("link", {"name": name})
        for child in source:
            mesh = child.find("geometry/mesh")
            if mesh is not None:
                filename = mesh.get("filename", "").removeprefix("assets/")
                if filename in NECK_DROPPED_MESHES:
                    continue
                mesh.set("filename", SO101_MESHES + filename)
            link.append(child)
        if name == "so101_wrist_roll_link":
            # The received holder, where the official fixed jaw was; its own mesh collides.
            for visual in holder:
                mesh, origin = visual.find("geometry/mesh"), visual.find("origin")
                assert mesh is not None and origin is not None
                filename = mesh.get("filename", "").removeprefix(SOURCE_MESH_PREFIX)
                if filename != RECEIVED_CAMERA_HOLDER:
                    continue  # the received servo, which the official one replaces
                link.append(visual)
                collision = ET.SubElement(link, "collision", {"name": f"{name}_holder"})
                ET.SubElement(collision, "origin", origin.attrib)
                geometry = ET.SubElement(collision, "geometry")
                ET.SubElement(geometry, "mesh", {"filename": f"meshes/{filename}"})
        robot.insert(list(robot).index(links[name]), link)
        robot.remove(links[name])
    for joint in official.findall("joint"):
        name = NECK_JOINTS.get(joint.get("name", ""))
        if name is None:
            continue
        joint.set("name", name)
        for end in ("parent", "child"):
            element = joint.find(end)
            assert element is not None
            element.set("link", NECK_LINKS[element.get("link", "")])
        if name == NECK_JOINTS["gripper"]:
            limit = joint.find("limit")
            assert limit is not None
            limit.set("lower", CAMERA_TILT_RANGE[0])
            limit.set("upper", CAMERA_TILT_RANGE[1])
        robot.insert(list(robot).index(mount) + 1, joint)


def _read_stl(path: Path) -> Vec:
    data = path.read_bytes()
    (count,) = struct.unpack_from("<I", data, 80)
    if len(data) != 84 + 50 * count:
        raise ValueError(f"{path.name}: not a binary STL")
    record = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    tris = np.frombuffer(data, dtype=record, offset=84)
    return tris["v"].reshape(-1, 3).astype(np.float64)


def _write_stl(path: Path, vertices: Vec, faces: NDArray[np.int32]) -> None:
    out = bytearray(b"sentient static upper body collision hull".ljust(80, b"\0"))
    out += struct.pack("<I", len(faces))
    for face in faces:
        a, b, c = vertices[face]
        normal: Vec = np.asarray(np.cross(b - a, c - a), dtype=np.float64)
        normal = normal / (float(np.sqrt(normal @ normal)) or 1.0)
        out += struct.pack("<12fH", *normal, *a, *b, *c, 0)
    path.write_bytes(bytes(out))


def _hull(points: Vec) -> tuple[Vec, NDArray[np.int32]]:
    hull = ConvexHull(points)
    vertices = points[hull.vertices]
    index = {old: new for new, old in enumerate(hull.vertices)}
    centre = vertices.mean(axis=0)
    faces: list[list[int]] = []
    for simplex in hull.simplices:
        face = [index[i] for i in simplex]
        a, b, c = vertices[face]
        if np.dot(np.cross(b - a, c - a), a - centre) < 0:
            face = [face[0], face[2], face[1]]
        faces.append(face)
    return np.round(vertices, 7), np.array(sorted(faces), dtype=np.int32)


def _mass_properties(
    vertices: Vec, faces: NDArray[np.int32], density: float
) -> tuple[float, Vec, Vec]:
    """Mass, centre of mass and inertia about it of a closed outward-wound triangle mesh."""
    volume = 0.0
    first = np.zeros(3)
    second = np.zeros((3, 3))
    for face in faces:
        a, b, c = vertices[face]
        v = np.dot(a, np.cross(b, c)) / 6.0
        volume += v
        first += v * (a + b + c) / 4.0
        s = a + b + c
        second += v / 20.0 * (np.outer(a, a) + np.outer(b, b) + np.outer(c, c) + np.outer(s, s))
    com = first / volume
    covariance = second - volume * np.outer(com, com)
    inertia = (np.trace(covariance) * np.eye(3) - covariance) * density
    return volume * density, com, inertia


def _link_points(link: ET.Element) -> Vec | None:
    points: list[Vec] = []
    for visual in link.findall("visual"):
        mesh = visual.find("geometry/mesh")
        if mesh is None:
            continue
        origin = visual.find("origin")
        xyz = _floats(origin.get("xyz") if origin is not None else None, (0, 0, 0))
        rpy = _floats(origin.get("rpy") if origin is not None else None, (0, 0, 0))
        scale = _floats(mesh.get("scale"), (1, 1, 1))
        raw = _read_stl(ASSET / mesh.get("filename", ""))
        points.append((raw * scale) @ _rpy(rpy).T + xyz)
    return np.concatenate(points) if points else None


def _fmt(values: Vec) -> str:
    return " ".join(f"{v:.9g}" for v in values)


def render() -> dict[Path, bytes]:
    robot = ET.parse(SOURCE).getroot()
    robot.set("name", "sentient_static_upper_body")
    outputs: dict[Path, bytes] = {}

    for sensor in robot.findall("sensor"):
        robot.remove(sensor)
    _graft_neck(robot)
    for joint in robot.findall("joint"):
        limit = joint.find("limit")
        if joint.get("name") in REVERSED_JOINTS and limit is not None:
            lower, upper = limit.get("lower", ""), limit.get("upper", "")
            limit.set("lower", _negated(upper))
            limit.set("upper", _negated(lower))

    for link in robot.findall("link"):
        name = link.get("name", "")
        for i, visual in enumerate(link.findall("visual")):
            visual.set("name", f"{name}_visual_{i}")
            mesh = visual.find("geometry/mesh")
            filename = "" if mesh is None else mesh.get("filename", "")
            if mesh is not None and filename.startswith(SOURCE_MESH_PREFIX):
                mesh.set("filename", f"meshes/{filename.removeprefix(SOURCE_MESH_PREFIX)}")
        if link.find("inertial") is not None and name in NECK_LINKS.values():
            continue  # the official SO-101's own mass properties and collision meshes
        points = _link_points(link)
        if points is None:
            continue
        vertices, faces = _hull(points)
        mass, com, inertia = _mass_properties(vertices, faces, HULL_DENSITY_KG_M3)
        if link.find("inertial") is not None:
            raise ValueError(f"{name}: the source now carries an inertial; stop estimating it")
        inertial = ET.Element("inertial")
        ET.SubElement(inertial, "origin", {"xyz": _fmt(com), "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": f"{mass:.6g}"})
        ET.SubElement(
            inertial,
            "inertia",
            {
                f"i{'xyz'[i]}{'xyz'[j]}": f"{inertia[i, j]:.6g}"
                for i, j in ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))
            },
        )
        link.insert(0, inertial)
        if link.find("collision") is None:
            relpath = f"meshes/collision/{name}.stl"
            stl = ET.Element("collision", {"name": f"{name}_hull"})
            ET.SubElement(stl, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            geometry = ET.SubElement(stl, "geometry")
            ET.SubElement(geometry, "mesh", {"filename": relpath})
            link.append(stl)
            path = ASSET / relpath
            _write_stl(path, vertices, faces)
            outputs[path] = path.read_bytes()

    # The ZED's frame, placed so its left eye's optical frame is the source's, and both eyes.
    if robot.find(f"link[@name='{SOURCE_ZED_EYE}']") is None:
        raise ValueError(f"the source no longer carries {SOURCE_ZED_EYE}")
    left = _pose(Pose((0.0, eye_offset("left"), 0.0), OPTICAL_RPY))
    ET.SubElement(robot, "link", {"name": ZED_FRAME})
    joint = ET.SubElement(
        robot, "joint", {"name": f"{SOURCE_ZED_EYE}_to_{ZED_FRAME}", "type": "fixed"}
    )
    _set_origin(joint, np.linalg.inv(left))
    ET.SubElement(joint, "parent", {"link": SOURCE_ZED_EYE})
    ET.SubElement(joint, "child", {"link": ZED_FRAME})
    for eye in EYES:
        name = optical_frame(eye)
        ET.SubElement(
            robot,
            "link",
            {
                "name": name,
                "data-preview-kind": "camera",
                "data-preview-label": f"Head camera, {eye}",
                "data-frame-convention": "camera_optical",
            },
        )
        joint = ET.SubElement(robot, "joint", {"name": f"{ZED_FRAME}_to_{name}", "type": "fixed"})
        ET.SubElement(joint, "origin", Pose((0.0, eye_offset(eye), 0.0), OPTICAL_RPY).origin())
        ET.SubElement(joint, "parent", {"link": ZED_FRAME})
        ET.SubElement(joint, "child", {"link": name})

    ET.indent(robot, space="  ")
    xml = '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(robot, encoding="unicode")
    outputs[TARGET] = (xml + "\n").encode()
    return outputs


def main() -> None:
    (ASSET / "meshes/collision").mkdir(parents=True, exist_ok=True)
    for path, data in render().items():
        path.write_bytes(data)
        sys.stdout.write(f"wrote {path.relative_to(ROOT)}\n")


if __name__ == "__main__":
    main()
