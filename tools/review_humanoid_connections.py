#!/usr/bin/env python3
"""Render and audit the development humanoid's literal visual geometry, without dynamics.

Requires numpy, scipy, mujoco and Pillow. Run from the owner package root:
  PYTHONPATH=. python tools/review_humanoid_connections.py --output /absolute/new-review
The output must be new. Optional --previous-urdf preserves a before/after shoulder view.
No actuator commands, nominal mass overrides, training, or geometry corrections occur here.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial import cKDTree

from sx_embodiments import development_embodiments, materialize
from sx_embodiments.assets import PACKAGE_URI_PREFIX, description_asset_uri
from sx_embodiments.kinematics import Kinematics, from_xyz_rpy
from sx_embodiments.known.humanoid import SENTIENT_HUMANOID_URDF
from sx_embodiments.known.static_upper_body import STATIC_UPPER_BODY_URDF
from sx_embodiments.materialize import closure


def numbers(values):
    return " ".join(map(str, values))


def quaternion(matrix):
    value = np.empty(4)
    mujoco.mju_mat2Quat(value, matrix[:3, :3].ravel())
    return numbers(value)


def scene(payload, uri, assets, destination):
    source = ET.fromstring(payload)
    kin = Kinematics.of(payload)
    root = next(iter(kin.links - kin.parent_joint.keys()))
    poses = kin.poses(root, dict.fromkeys(kin.independent, 0.0))
    xml = ET.Element("mujoco")
    ET.SubElement(xml, "compiler", angle="radian", eulerseq="XYZ")
    visual = ET.SubElement(xml, "visual")
    ET.SubElement(visual, "rgba", haze=".055 .07 .09 1")
    asset = ET.SubElement(xml, "asset")
    ET.SubElement(
        asset,
        "texture",
        name="background",
        type="skybox",
        builtin="flat",
        rgb1=".055 .07 .09",
        rgb2=".055 .07 .09",
        width="64",
        height="64",
    )
    world = ET.SubElement(xml, "worldbody")
    meshes = {}
    for link in source.findall("link"):
        matrix = np.asarray(poses[link.get("name")]).reshape(4, 4)
        node = ET.SubElement(
            world,
            "body",
            name=link.get("name"),
            pos=numbers(matrix[:3, 3]),
            quat=quaternion(matrix),
        )
        for v in link.findall("visual"):
            origin = v.find("origin")
            xyz = (
                tuple(map(float, origin.get("xyz", "0 0 0").split()))
                if origin is not None
                else (0.0, 0.0, 0.0)
            )
            rpy = (
                tuple(map(float, origin.get("rpy", "0 0 0").split()))
                if origin is not None
                else (0.0, 0.0, 0.0)
            )
            transform = np.asarray(from_xyz_rpy(xyz, rpy)).reshape(4, 4)
            geom = ET.SubElement(
                node,
                "geom",
                group="1",
                contype="0",
                conaffinity="0",
                pos=numbers(xyz),
                quat=quaternion(transform),
                rgba=".62 .72 .80 1",
            )
            shape = v.find("geometry")[0]
            if shape.tag != "mesh":
                raise ValueError("This CAD review expects mesh visuals")
            path = assets / description_asset_uri(uri, shape.get("filename")).removeprefix(
                PACKAGE_URI_PREFIX
            )
            scale = shape.get("scale", "1 1 1")
            key = (str(path), scale)
            if key not in meshes:
                meshes[key] = "mesh" + str(len(meshes))
                ET.SubElement(
                    asset, "mesh", name=meshes[key], file=str(path), scale=scale, inertia="shell"
                )
            geom.set("type", "mesh")
            geom.set("mesh", meshes[key])
    ET.ElementTree(xml).write(destination)
    model = mujoco.MjModel.from_xml_path(str(destination))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    vertices = {}
    for g in range(model.ngeom):
        mesh = model.geom_dataid[g]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        points = (
            model.mesh_vert[start : start + count] @ data.geom_xmat[g].reshape(3, 3).T
            + data.geom_xpos[g]
        )
        vertices.setdefault(model.body(model.geom_bodyid[g]).name, []).append(points)
    return model, data, {name: np.concatenate(parts) for name, parts in vertices.items()}


def separation(vertices, first, second):
    return float(cKDTree(vertices[first]).query(vertices[second])[0].min())


def font(size):
    for path in (
        "/System/Library/Fonts/Helvetica.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default(size=size)


def panel(model, data, title, target, distance, azimuth, height=720):
    model.vis.global_.offwidth = 600
    model.vis.global_.offheight = height
    model.vis.headlight.ambient[:] = 0.35
    model.vis.headlight.diffuse[:] = 0.65
    model.vis.headlight.specular[:] = 0.08
    camera = mujoco.MjvCamera()
    camera.lookat[:] = target
    camera.distance, camera.azimuth, camera.elevation = distance, azimuth, -5
    with mujoco.Renderer(model, height=height, width=600) as renderer:
        renderer.update_scene(data, camera=camera)
        result = Image.new("RGB", (600, height + 70), (14, 18, 23))
        result.paste(Image.fromarray(renderer.render()), (0, 45))
    draw = ImageDraw.Draw(result)
    draw.text((12, 10), title, font=font(21), fill=(175, 220, 235))
    draw.text(
        (12, height + 48),
        "Visual CAD only · zero display pose · review pending",
        font=font(13),
        fill=(195, 210, 220),
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--previous-urdf", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    candidate = output / "candidate"
    candidate.mkdir()
    body = development_embodiments["sentient-humanoid"]
    assets = materialize(body)
    payload = body.urdf_bytes
    (candidate / "embodiment.json").write_text(body.to_json())
    (candidate / "robot.urdf").write_bytes(payload)
    copied = []
    for relative, blob in closure(body).items():
        source = assets / relative
        destination = candidate / "assets" / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        with destination.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        if digest != str(blob.sha256) or destination.stat().st_size != blob.size_bytes:
            raise ValueError("Portable asset copy failed content verification: " + relative)
        copied.append(
            {
                "path": "assets/" + relative,
                "sha256": digest,
                "size_bytes": destination.stat().st_size,
            }
        )
    view = ET.fromstring(payload)
    for mesh in view.iter("mesh"):
        mesh.set("filename", "assets/" + mesh.get("filename").removeprefix(PACKAGE_URI_PREFIX))
    ET.ElementTree(view).write(candidate / "view-relative.urdf")
    uri = str(body.urdf.asset.uri)
    current, data, vertices = scene(payload, uri, assets, candidate / "static-visual.xml")
    points = np.concatenate(list(vertices.values()))
    center = (points.min(axis=0) + points.max(axis=0)) / 2
    center[:2] = 0
    plate = Image.new("RGB", (1800, 790), (14, 18, 23))
    for i, (name, angle) in enumerate((("Front", 0), ("Side", 90), ("Three-quarter", 40))):
        image = panel(current, data, name, center, 2.15, angle)
        image.save(output / (name.lower() + ".png"))
        plate.paste(image, (600 * i, 0))
    plate.save(output / "assembly-review.png")
    comparisons = [
        (
            "Original full-body source",
            SENTIENT_HUMANOID_URDF.path().read_bytes(),
            str(SENTIENT_HUMANOID_URDF.ref().uri),
        ),
        (
            "Corrected static source",
            STATIC_UPPER_BODY_URDF.path().read_bytes(),
            str(STATIC_UPPER_BODY_URDF.ref().uri),
        ),
    ]
    if args.previous_urdf:
        comparisons.append(("Before: omitted fixed mounts", args.previous_urdf.read_bytes(), uri))
    comparisons.append(("Restored source-owned mounts", payload, uri))
    report = {}
    strip = Image.new("RGB", (600 * len(comparisons), 620), (14, 18, 23))
    for i, (title, source, source_uri) in enumerate(comparisons):
        model, state, samples = scene(
            source, source_uri, assets, output / ("comparison-" + str(i) + ".xml")
        )
        target = (
            state.xpos[model.body("left_shoulder_pitch_link").id]
            + state.xpos[model.body("right_shoulder_pitch_link").id]
        ) / 2
        target[2] -= 0.04
        strip.paste(panel(model, state, title, target, 0.76, 0, height=550), (600 * i, 0))
        seams = {}
        for side in ("left", "right"):
            child = side + (
                "_shoulder_mount_link"
                if side + "_shoulder_mount_link" in samples
                else "_shoulder_pitch_link"
            )
            seams[side + "_shoulder_m"] = separation(samples, "chest_link", child)
            if side + "_ankle1_link" in samples:
                seams[side + "_ankle_m"] = separation(
                    samples, side + "_ankle1_link", side + "_ankle2_link"
                )
        report[title] = seams
    strip.save(output / "shoulder-comparison.png")
    ankle_plate = Image.new("RGB", (1800, 620), (14, 18, 23))
    for index, (model_path, title, angle) in enumerate(
        (
            (output / "comparison-0.xml", "Original source: right ankle", -45),
            (candidate / "static-visual.xml", "Current candidate: right ankle", -45),
            (candidate / "static-visual.xml", "Current candidate: side", 90),
        )
    ):
        model = mujoco.MjModel.from_xml_path(str(model_path))
        state = mujoco.MjData(model)
        mujoco.mj_forward(model, state)
        target = state.xpos[model.body("right_ankle1_link").id] + [0, 0, -0.11]
        ankle_plate.paste(
            panel(model, state, title, target, 0.53, angle, height=550), (600 * index, 0)
        )
    ankle_plate.save(output / "ankle-source-comparison.png")
    tree = ET.fromstring(payload)
    links = [x.get("name") for x in tree.findall("link")]
    joints = tree.findall("joint")
    parents = {j.find("child").get("link"): j.find("parent").get("link") for j in joints}
    roots = set(links) - parents.keys()
    assert len(links) == len(set(links)) and len(parents) == len(joints) and len(roots) == 1
    reached = set(roots)
    while True:
        expanded = reached | {child for child, parent in parents.items() if parent in reached}
        if expanded == reached:
            break
        reached = expanded
    assert reached == set(links) and len(joints) == len(links) - 1
    receipt = {
        "status": "awaiting_user_engineering_review",
        "body_id": str(body.id),
        "urdf_sha256": hashlib.sha256(payload).hexdigest(),
        "links": len(links),
        "joints": len(joints),
        "root": next(iter(roots)),
        "reachable_links": len(reached),
        "native_coordinates": list(body.joint_names),
        "display_pose": "all independent coordinates zero; not a calibrated Ready pose",
        "visual_vertex_separations": report,
        "metric_scope": (
            "Sampled vertex distance is an upper bound on minimum surface separation, "
            "not collision or mechanical fit proof"
        ),
        "assets": copied,
        "training_authorized": False,
        "physical_fit_qualified": False,
    }
    (candidate / "engineering-review.json").write_text(json.dumps(receipt, indent=2))
    (candidate / "OPEN.md").write_text(
        "Open view-relative.urdf with a URDF viewer; keep assets/ beside it. "
        "Only mesh URIs differ from robot.urdf. static-visual.xml is a massless, "
        "jointless display scene, never a physics model. Candidate awaiting user "
        "engineering review; no training or physical qualification.\n"
    )
    print(json.dumps({key: value for key, value in receipt.items() if key != "assets"}, indent=2))  # noqa: T201 -- CLI report


if __name__ == "__main__":
    main()
