"""Package the nominal fixed-base Sentient assembly with left and right Wuji hands."""

from __future__ import annotations

import errno
import hashlib
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from itertools import product
from pathlib import Path

from . import development_embodiments
from .assembly import Assembly, Member, Pose, Subtree
from .assets import PACKAGE_URI_PREFIX, description_asset_uri
from .checks import stl_bounds
from .kinematics import from_xyz_rpy
from .known.static_upper_body import STATIC_UPPER_BODY_URDF
from .materialize import materialize


def _publish(path: Path, payload: bytes) -> None:
    """Atomic, create-only publication; repeat calls verify the existing bytes."""

    def already_published() -> bool:
        if not path.exists():
            return False
        if path.read_bytes() != payload:
            raise ValueError(f"existing file differs from requested assembly: {path}")
        return True

    if already_published():
        return
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as handle:
        tmp = Path(handle.name)
    try:
        tmp.write_bytes(payload)
        try:
            if os.name == "nt":
                tmp.rename(path)  # On Windows rename is atomic and refuses an existing name.
            else:
                os.link(tmp, path)
        except FileExistsError:
            already_published()
        except OSError as error:
            if os.name == "nt" or error.errno not in {errno.EOPNOTSUPP, errno.ENOSYS, errno.EPERM}:
                raise
            # FAT/exFAT lack hard links. Serialize the existence check and rename
            # on the destination directory. The OS releases the lock on process exit.
            import fcntl

            directory = os.open(path.parent, os.O_RDONLY)
            try:
                fcntl.flock(directory, fcntl.LOCK_EX)
                if not already_published():
                    tmp.rename(path)
            finally:
                os.close(directory)
    finally:
        tmp.unlink(missing_ok=True)


def _extent(root: ET.Element, link: str, axis: int, directory: Path) -> tuple[float, float]:
    """Projection of the link's housing on the mounting axis, in link metres."""
    extrema: list[float] = []
    for geometry in root.findall(f"link[@name='{link}']/collision"):
        mesh = geometry.find("geometry/mesh")
        if mesh is None or not mesh.attrib["filename"].lower().endswith(".stl"):
            raise ValueError(f"{link} needs a declared STL housing to place the nominal wrist")
        low, high = stl_bounds(directory / mesh.attrib["filename"])
        origin = geometry.find("origin")

        def triple(
            attribute: str, default: str, origin: ET.Element | None = origin
        ) -> tuple[float, float, float]:
            text = default if origin is None else origin.get(attribute, default)
            a, b, c = (float(x) for x in text.split())
            return (a, b, c)

        transform = from_xyz_rpy(triple("xyz", "0 0 0"), triple("rpy", "0 0 0"))
        scale = tuple(float(x) for x in mesh.get("scale", "1 1 1").split())
        if len(scale) != 3:
            raise ValueError("mesh scale must have three values")
        # Bounding-box corners give the exact projection for these axis-aligned CAD
        # housing frames and a conservative exterior plane for a rotated housing.
        for corner in product(*zip(low, high, strict=True)):
            extrema.append(
                transform[4 * axis + 3]
                + sum(transform[4 * axis + j] * corner[j] * scale[j] for j in range(3))
            )
    if not extrema or max(extrema) <= min(extrema):
        raise ValueError(f"{link} has no housing extent along the mount axis")
    return min(extrema), max(extrema)


def assemble(wuji_urdf_dir: Path, output: Path) -> Path:
    """Assemble vendor left.urdf + right.urdf, packaging meshes beside the output.

    These wrist seats are nominal (palm +z follows the wrist-roll link +x).
    This generates an offline URDF. Physical adapter calibration is separate.
    """
    body_root = materialize(development_embodiments["sentient-static-upper-body"])
    output.parent.mkdir(parents=True, exist_ok=True)
    mesh_dir = output.parent / "meshes"
    mesh_dir.mkdir(exist_ok=True)
    copied: dict[Path, str] = {}

    def portable(path: Path) -> str:
        resolved = path.resolve(strict=True)
        if resolved not in copied:
            data = resolved.read_bytes()
            name = hashlib.sha256(data).hexdigest() + resolved.suffix.lower()
            _publish(mesh_dir / name, data)
            copied[resolved] = f"meshes/{name}"
        return copied[resolved]

    def body_asset(uri: str) -> str:
        absolute = description_asset_uri(PACKAGE_URI_PREFIX + STATIC_UPPER_BODY_URDF.relpath, uri)
        if not absolute.startswith(PACKAGE_URI_PREFIX):
            raise ValueError(f"unresolved body asset {uri}")
        return portable(body_root / absolute.removeprefix(PACKAGE_URI_PREFIX))

    def hand_asset(uri: str) -> str:
        return portable(wuji_urdf_dir / uri)

    upper = Subtree(
        STATIC_UPPER_BODY_URDF, without=("left_gripper_mount_joint", "right_gripper_mount_joint")
    )
    members = [Member("body", upper, namespaced=False, mesh_uri=body_asset)]
    for side in ("left", "right"):
        hand = ET.parse(wuji_urdf_dir / f"{side}.urdf").getroot()
        for compiler in hand.findall("mujoco"):
            hand.remove(compiler)
        members.append(
            Member(
                f"{side}_wuji",
                ET.tostring(hand),
                namespaced=False,
                parent=f"{side}_wrist_roll_link",
                pose=Pose(rpy=(0.0, math.pi / 2.0, 0.0)),
                mesh_uri=hand_asset,
            )
        )
    assembly = Assembly(
        "sentient_static_upper_body_wuji",
        "assembly_base",
        tuple(members),
        note="Nominal Wuji palm +z along wrist +x. Unmeasured adapter geometry.",
    )
    root = ET.fromstring(assembly.urdf())
    for side in ("left", "right"):
        wrist = f"{side}_wrist_roll_link"
        # Position the rear of the Wuji palm housing at the distal face of the
        # wrist housing. The joint origins of these different vendors are not mating planes.
        face = _extent(root, wrist, 0, output.parent)[1]
        rear = _extent(root, f"{side}_palm_link", 2, output.parent)[0]
        joint = root.find(f"joint[@name='{wrist}_to_{side}_wuji']/origin")
        if joint is None:
            raise ValueError(f"assembly lacks {side} Wuji mounting joint")
        joint.set("xyz", f"{face - rear:.12g} 0 0")
    for compiler in root.findall("mujoco"):
        root.remove(compiler)
    ET.SubElement(
        ET.SubElement(root, "mujoco"),
        "compiler",
        strippath="false",
        discardvisual="false",
        fusestatic="false",
        balanceinertia="true",
    )
    for link in root.findall("link"):
        for category in ("visual", "collision"):
            for i, geom in enumerate(link.findall(category)):
                geom.set("name", f"{link.attrib['name']}_{category}_{i}")
    ET.indent(root)
    _publish(output, ET.tostring(root, encoding="utf-8", xml_declaration=True))
    return output
