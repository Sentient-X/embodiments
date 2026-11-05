#!/usr/bin/env python3
"""Render every registry body with a tool frame for ``sx-embodiment-ik``'s registry-wide tests.

    python tools/render_ik_registry.py           # rewrite the fixture
    python tools/render_ik_registry.py --check   # exit 1 when it disagrees

``drivers/sx-embodiment-ik/tests/fixtures/registry.json`` holds, for each body of both
registries (``embodiments`` then ``development_embodiments``) whose ``sx_embodiments.frames.tools``
names a tool frame: its composed description cut to its kinematics (links, joints, limits,
mimics; no geometry), its ``collision.json`` capsules and ignored pairs as
``sx_embodiments.collision_proxies`` fits them (the convex pieces left out: they refine contacts
only), and its tool frames in the registry's order. A declared support whose frame the
description lacks is dropped and listed under ``defects`` (2026-11-04: ``franka`` and
``panda_omron`` mount on ``link0`` and ``base``, which their prefixed descriptions call
``panda_link0``). The solve's tests build each body from
these bytes, so no body is listed by hand in Rust. ``tests/test_ik_registry.py`` runs the check.
Needs the ``render`` extra (NumPy).
"""

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from sx_embodiments import development_embodiments, embodiments
from sx_embodiments.collision_proxies import proxies
from sx_embodiments.frames import tools
from sx_embodiments.rendered import same_rendering

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "drivers/sx-embodiment-ik/tests/fixtures/registry.json"
#: What a description keeps: the tree and its joints' numbers.
_GEOMETRY = ("visual", "collision", "inertial")


def _kinematic(urdf: bytes) -> str:
    """``urdf`` with every link's geometry and inertia dropped."""
    root = ET.fromstring(urdf)  # nosec B314 - registry-owned description bytes
    for link in root.findall("link"):
        for child in list(link):
            if child.tag in _GEOMETRY:
                link.remove(child)
    for child in list(root):
        if child.tag not in ("link", "joint"):
            root.remove(child)
    return ET.tostring(root, encoding="unicode")


def _collision(name: str, document: bytes | None, urdf: str) -> tuple[dict[str, object], list[str]]:
    """The body's proxies without their pieces, and what the registry declares wrong in them."""
    if document is None:
        return {
            "schema": "sx.collision-proxies/v4",
            "embodiment_id": name,
            "urdf_sha256": "",
            "capsules": [],
            "hulls": [],
            "ignored_pairs": {"adjacent": [], "touching_at_zero": []},
            "adjacent_joints": 0,
            "generated_by": "sx_embodiments.collision_proxies/v4",
            "support": None,
        }, []
    proxied = json.loads(document)
    proxied["hulls"] = []
    links = {link.get("name") for link in ET.fromstring(urdf).findall("link")}  # nosec B314
    support = proxied.get("support")
    if support is not None and support["frame"] not in links:
        # The mount names a frame the composed description renamed (a prefixed arm): the solve
        # cannot place the support, so the fixture carries the body without it and says so.
        proxied["support"] = None
        return proxied, [f"support frame {support['frame']!r} is not a link of the description"]
    return proxied, []


def rendered() -> dict[Path, str]:
    bodies = []
    for registry in (embodiments, development_embodiments):
        for name in registry:
            body = registry[name]
            frames = [frame for _, frame in tools(body)]
            if not frames:
                continue
            urdf = _kinematic(body.urdf_bytes)
            collision, defects = _collision(str(name), proxies(body, name=str(name)), urdf)
            bodies.append(
                {
                    "embodiment": str(name),
                    "tools": frames,
                    "urdf": urdf,
                    "collision": collision,
                    "defects": defects,
                }
            )
    return {FIXTURE: json.dumps(bodies, separators=(",", ":"), sort_keys=True) + "\n"}


def stale() -> list[Path]:
    return [path for path, text in rendered().items() if not same_rendering(path, text.encode())]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="compare instead of writing")
    arguments = parser.parse_args()
    if arguments.check:
        for path in stale():
            sys.stderr.write(f"stale: {path.relative_to(ROOT)}\n")
        return 1 if stale() else 0
    for path, text in rendered().items():
        if same_rendering(path, text.encode("utf-8")):
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
