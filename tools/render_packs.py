#!/usr/bin/env python3
"""Render the committed render packs and the tree-pose fixture the Rust FK is held to.

    python tools/render_packs.py           # rewrite every pack and the fixture
    python tools/render_packs.py --check   # exit 1 when one disagrees

``render_packs/<body>/`` holds :func:`sx_embodiments.render_pack.build`'s pack for each body
below (``robot.urdf``, ``pack.glb``, ``pack.json``), within its triangle budget: the static
upper body at the headset's 150k, the YUBI hands at 60k because the operator sees both in
every frame beside the robot. ``drivers/sx-embodiment-drivers/tests/fixtures/tree-poses.json``
holds, for each pack's ``robot.urdf``, every link's pose in the root frame at seeded joint
positions as :class:`sx_embodiments.kinematics.Kinematics` answers (a floating joint holds its
origin), which ``sx_embodiment_drivers::kinematics::Tree::link_poses`` is held equal to.
``tests/test_render_packs.py`` runs the same check. Needs the ``render`` extra (NumPy).
"""

from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path

from sx_embodiments import development_embodiments, embodiments
from sx_embodiments.embodiment import Embodiment
from sx_embodiments.errors import UnknownEmbodimentError
from sx_embodiments.kinematics import JointType, Kinematics, Transform, compose
from sx_embodiments.known.static_upper_body import (
    STATIC_UPPER_BODY_VIRTUAL_GRIPPER,
    STATIC_UPPER_BODY_WORKCELL,
)
from sx_embodiments.known.yubi import YUBI_TRACKING
from sx_embodiments.render_pack import RenderPack, build
from sx_embodiments.viewing import Viewing

ROOT = Path(__file__).resolve().parents[1]
PACKS = ROOT / "render_packs"
POSES = ROOT / "drivers/sx-embodiment-drivers/tests/fixtures/tree-poses.json"

#: Each body drawn, with its triangle budget.
BODIES: dict[str, int] = {"sentient-static-upper-body": 150_000, "yubi": 60_000}
#: Each body's viewing facts, as its registry module declares them.
VIEWING: dict[str, Viewing] = {
    "sentient-static-upper-body": Viewing(
        mount=STATIC_UPPER_BODY_WORKCELL, virtual_gripper=STATIC_UPPER_BODY_VIRTUAL_GRIPPER
    ),
    "yubi": Viewing(tracking=YUBI_TRACKING),
}
SAMPLES = 4
_HELD = (JointType.FLOATING, JointType.PLANAR)


def body(name: str) -> Embodiment:
    try:
        return embodiments[name]
    except UnknownEmbodimentError:
        return development_embodiments[name]


def tree_poses(kinematics: Kinematics, positions: dict[str, float]) -> dict[str, Transform]:
    """Every link in the root frame; a floating or planar joint holds its origin."""
    poses: dict[str, Transform] = {}

    def pose(link: str) -> Transform:
        if link not in poses:
            name = kinematics.parent_joint.get(link)
            if name is None:
                poses[link] = compose()
            else:
                joint = kinematics.joints[name]
                held = joint.type in _HELD
                local = (
                    joint.origin
                    if held
                    else kinematics.pose(
                        joint.parent, link, _relevant(kinematics, joint.name, positions)
                    )
                )
                poses[link] = compose(pose(joint.parent), local)
        return poses[link]

    return {link: pose(link) for link in sorted(kinematics.links)}


def _relevant(kinematics: Kinematics, joint: str, positions: dict[str, float]) -> dict[str, float]:
    leader = kinematics.joints[joint]
    while leader.mimic is not None:
        leader = kinematics.joints[leader.mimic.leader]
    return {leader.name: positions[leader.name]} if leader.name in positions else {}


def packs() -> dict[str, RenderPack]:
    return {
        name: build(body(name), name=name, triangle_budget=budget, viewing=VIEWING.get(name))
        for name, budget in BODIES.items()
    }


def fixture(rendered: dict[str, RenderPack]) -> str:
    document: dict[str, object] = {}
    for name, pack in rendered.items():
        kinematics = Kinematics.of(pack.urdf)
        generator = random.Random(name)
        samples: list[dict[str, object]] = []
        for _ in range(SAMPLES):
            positions = {
                joint: generator.uniform(-1.5, 1.5)
                for joint in kinematics.independent
                if kinematics.joints[joint].type not in _HELD
            }
            samples.append({"positions": positions, "poses": tree_poses(kinematics, positions)})
        document[name] = samples
    return json.dumps(document, indent=1, sort_keys=True) + "\n"


def rendered_files() -> dict[Path, bytes]:
    rendered = packs()
    files = {
        PACKS / name / file: data
        for name, pack in rendered.items()
        for file, data in pack.files.items()
    }
    files[POSES] = fixture(rendered).encode()
    return files


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="compare instead of writing")
    arguments = parser.parse_args()
    files = rendered_files()
    if arguments.check:
        stale = [
            path for path, data in files.items() if not path.is_file() or path.read_bytes() != data
        ]
        for path in stale:
            print(f"stale: {path.relative_to(ROOT)}", file=sys.stderr)  # noqa: T201
        return 1 if stale else 0
    for path, data in files.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    return 0


if __name__ == "__main__":
    sys.exit(main())
