"""Reproduce the offline CAD separation evidence in sentient_wuji_cad_sweeps.csv.

Install trimesh, shapely and python-fcl in addition to the retarget extra. Run
`python tools/verify_ego_bearing_cad.py path/to/assembled.urdf` against the
content-addressed assembly. FCL is an offline evidence generator only.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import fcl
import numpy as np
import trimesh

from sx_embodiments._ego_collision import _CAD_SWEEPS, collision_assets
from sx_embodiments.kinematics import Kinematics, from_xyz_rpy


def exterior(mesh, axis, radius, lower, upper):
    """Cut only the declared rotor interface out of each link's original CAD.

    An inscribed 24-sided prism leaves a superset of the surface outside the
    cylinder. Keep triangles that straddle a cut plane, clipped at the plane.
    """
    tangent = np.cross(axis, np.eye(3)[np.argmin(abs(axis))])
    tangent /= np.linalg.norm(tangent)
    bitangent = np.cross(axis, tangent)
    normals = [
        math.cos(theta) * tangent + math.sin(theta) * bitangent
        for theta in np.arange(24) * (2 * math.pi / 24)
    ]
    planes = [(axis, upper), (-axis, -lower)]
    planes.extend((normal, radius * math.cos(math.pi / 24)) for normal in normals)
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.faces)
    values = vertices @ np.stack([n for n, _ in planes]).T - [v for _, v in planes]
    outside = np.zeros(len(faces), dtype=bool)
    inside = np.ones(len(faces), dtype=bool)
    for index in range(len(planes)):
        corners = values[:, index][faces]
        outside |= corners.min(axis=1) > 1e-9
        inside &= corners.max(axis=1) < -1e-9
    pieces = [mesh.submesh([outside], append=True)] if outside.any() else []
    boundary = ~(outside | inside)
    if boundary.any():
        remainder = mesh.submesh([boundary], append=True)
        for normal, offset in planes:
            if not len(remainder.faces):
                break
            origin = normal * offset
            kept = remainder.slice_plane(origin, normal)
            remainder = remainder.slice_plane(origin, -normal)
            if len(kept.faces):
                pieces.append(kept)
    return pieces


def body_surfaces(link, asset_dir, joint_from_link, axis, row):
    pieces = []
    for visual in link.findall("visual"):
        element = visual.find("geometry/mesh")
        if element is None:
            raise ValueError("certificate expects original CAD mesh visuals for both links")
        mesh = trimesh.load_mesh(asset_dir / element.attrib["filename"])
        mesh.vertices *= np.fromstring(element.get("scale", "1 1 1"), sep=" ")
        origin = visual.find("origin")

        def vector(name, origin=origin):
            text = "0 0 0" if origin is None else origin.get(name, "0 0 0")
            return tuple(float(value) for value in text.split())

        placement = np.array(from_xyz_rpy(vector("xyz"), vector("rpy"))).reshape(4, 4)
        mesh.apply_transform(joint_from_link @ placement)
        if row["joint_2"]:
            pieces.append(mesh)
        else:
            pieces.extend(
                exterior(
                    mesh,
                    axis,
                    float(row["seat_radius_m"]),
                    float(row["seat_axial_min_m"]),
                    float(row["seat_axial_max_m"]),
                )
            )
    if not pieces:
        raise ValueError("rotary interface removes the entire link")
    return trimesh.util.concatenate(pieces)


def collision_object(mesh):
    model = fcl.BVHModel()
    model.beginModel(len(mesh.faces), len(mesh.vertices))
    model.addSubModel(np.asarray(mesh.vertices, dtype="d"), np.asarray(mesh.faces, dtype="i"))
    model.endModel()
    return fcl.CollisionObject(model)


def winding(surface, point):
    corners = surface.triangles - point
    lengths = np.linalg.norm(corners, axis=2)
    numer = np.einsum("ij,ij->i", corners[:, 0], np.cross(corners[:, 1], corners[:, 2]))
    denom = lengths.prod(axis=1)
    for i in range(3):
        denom += (
            np.einsum("ij,ij->i", corners[:, i], corners[:, (i + 1) % 3]) * lengths[:, (i + 2) % 3]
        )
    return np.arctan2(numer, denom).sum() / (2 * math.pi)


def outer_shells(mesh):
    parts = mesh.split(only_watertight=False)
    shells = [part for part in parts if part.is_watertight and part.volume > 1e-12]
    if not shells:
        raise ValueError("no closed exterior CAD solid")
    for part in parts:
        if part.is_watertight:
            continue
        # Three right-palm cavity shells have open seams. Verify that even their
        # entire convex enclosures lie inside a closed exterior and away from
        # its boundary. The filled exterior conservatively covers those cavities.
        hull = collision_object(part.convex_hull)
        if not any(
            abs(winding(shell, part.vertices[0]) - 1.0) < 0.01
            and fcl.distance(
                collision_object(shell), hull, fcl.DistanceRequest(), fcl.DistanceResult()
            )
            > 1e-7
            for shell in shells
        ):
            raise ValueError("unclosed CAD component has no verified closed outer enclosure")
    return parts, shells


def assert_not_nested(a, b):
    """Conservatively fill cavities to rule out complete containment at neutral."""
    parts_a, shells_a = outer_shells(a)
    parts_b, shells_b = outer_shells(b)
    for parts, shells in ((parts_a, shells_b), (parts_b, shells_a)):
        for part in parts:
            for shell in shells:
                value = winding(shell, part.vertices[0])
                if not np.isfinite(value) or abs(value) > 0.01:
                    raise ValueError(f"connected CAD surface is nested at neutral: winding {value}")


def compound_sweep(urdf, root, tree, row):
    first, second = tree.joints[row["joint"]], tree.joints[row["joint_2"]]
    if first.child != second.parent or {first.parent, second.child} != {
        row["link_a"],
        row["link_b"],
    }:
        raise ValueError("declared joints do not connect the two CAD links")
    fixed = body_surfaces(
        root.find(f"link[@name='{first.parent}']"), urdf.parent, np.eye(4), None, row
    )
    moving = body_surfaces(
        root.find(f"link[@name='{second.child}']"), urdf.parent, np.eye(4), None, row
    )
    neutral = np.array(
        tree.pose(first.parent, second.child, {first.name: 0.0, second.name: 0.0})
    ).reshape(4, 4)
    assert_not_nested(fixed, moving.copy().apply_transform(neutral))
    a, b = collision_object(fixed), collision_object(moving)
    second_pose = np.array(tree.pose(first.child, second.child, {second.name: 0.0})).reshape(4, 4)
    radii = np.array(
        [
            np.linalg.norm(second_pose[:3, 3]) + np.linalg.norm(moving.vertices, axis=1).max(),
            np.linalg.norm(np.cross(moving.vertices, second.axis), axis=1).max(),
        ]
    )
    box = np.array([[float(row["lower_rad" + s]), float(row["upper_rad" + s])] for s in ("", "_2")])
    stack = [box]
    queries, minimum = 0, math.inf
    request = fcl.DistanceRequest(enable_nearest_points=True)
    while stack:
        box = stack.pop()
        center = box.mean(axis=1)
        pose = np.array(
            tree.pose(first.parent, second.child, {first.name: center[0], second.name: center[1]})
        ).reshape(4, 4)
        b.setTransform(fcl.Transform(pose[:3, :3], pose[:3, 3]))
        distance = fcl.distance(a, b, request, fcl.DistanceResult())
        queries += 1
        minimum = min(minimum, distance)
        if distance <= 1e-7:
            raise ValueError(f"{first.name}/{second.name} at {center}: clearance {distance} m")
        # The maximum displacement over a joint box is bounded by the sum of
        # the two joint rotations times conservative farthest-point radii.
        displacements = radii * np.diff(box, axis=1)[:, 0] / 2
        if distance <= 1e-7 + displacements.sum():
            axis = displacements.argmax()
            left, right = box.copy(), box.copy()
            left[axis, 1] = right[axis, 0] = center[axis]
            stack.extend((right, left))
    return queries, minimum, radii.max()


def sweep(urdf, root, tree, row):
    if row["joint_2"]:
        return compound_sweep(urdf, root, tree, row)
    name = row["joint"]
    joint = tree.joints[name]
    if {joint.parent, joint.child} != {row["link_a"], row["link_b"]}:
        raise ValueError(f"{name} is not the declared joint between these links")
    axis = np.array(joint.axis)
    neutral = np.array(tree.pose(joint.parent, joint.child, {name: 0.0})).reshape(4, 4)
    parent = root.find(f"link[@name='{joint.parent}']")
    child = root.find(f"link[@name='{joint.child}']")
    fixed = body_surfaces(parent, urdf.parent, np.linalg.inv(neutral), axis, row)
    moving = body_surfaces(child, urdf.parent, np.eye(4), axis, row)
    radius = np.linalg.norm(np.cross(moving.vertices, axis), axis=1).max()
    a, b = collision_object(fixed), collision_object(moving)
    request = fcl.DistanceRequest(enable_nearest_points=True)
    angle, stop = float(row["lower_rad"]), float(row["upper_rad"])
    queries, minimum = 0, math.inf
    while True:
        pose = np.array(tree.pose(joint.parent, joint.child, {name: angle})).reshape(4, 4)
        pose = np.linalg.inv(neutral) @ pose
        b.setTransform(fcl.Transform(pose[:3, :3], pose[:3, 3]))
        distance = fcl.distance(a, b, request, fcl.DistanceResult())
        queries += 1
        minimum = min(minimum, distance)
        if distance <= 1e-7:
            raise ValueError(f"{name} at {angle:.8f}: exterior clearance {distance} m")
        # Every surface point moves by at most radius * abs(delta_angle).
        # The interval proof remains valid when the closest feature changes.
        step = (distance - 1e-7) / radius
        if angle + step >= stop:
            return queries, minimum, radius
        angle += 0.99 * step


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("--joint", help="audit only this joint")
    args = parser.parse_args()
    geometry_id = collision_assets(args.urdf)[2]
    root = ET.parse(args.urdf).getroot()
    tree = Kinematics.of(args.urdf.read_bytes())
    with _CAD_SWEEPS.open(newline="") as stream:
        rows = [r for r in csv.DictReader(stream) if not args.joint or r["joint"] == args.joint]
    if not rows or any(row["geometry_sha256"] != geometry_id for row in rows):
        raise ValueError("no matching certified joint or CAD geometry identity")
    for row in rows:
        queries, minimum, radius = sweep(args.urdf, root, tree, row)
        sys.stdout.write(
            f"{row['joint']} [{row['lower_rad']}, {row['upper_rad']}] radians, "
            f"{queries} queries, minimum sampled distance {minimum:.8g} m, radius {radius:.8g} m\n"
        )


if __name__ == "__main__":
    main()
