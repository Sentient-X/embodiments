"""Whether a committed rendering is still what its tool renders, across platforms.

Forward kinematics, media plans and other rendered fixtures sum floating-point products in an
order the platform's BLAS chooses, so the last bits of a number differ between macOS and Linux
(CI renders on Linux) and a byte comparison of full-precision JSON fails on one of them.
Rounding at render time cannot fix that: a value near a rounding boundary still lands on
either side. So a JSON rendering is current when it is the same document with every number
equal to `REL` relative (or `ABS` absolute near zero): below the 1e-9 the drivers compare
poses at, and far above the platforms' disagreement (a few units in the last place, which
long kinematic chains compound). Anything else is compared byte for byte.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import cast

REL = 1e-10
ABS = 1e-12


def _same(left: object, right: object) -> bool:
    if isinstance(left, bool) or isinstance(right, bool):
        return left is right
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(left, right, rel_tol=REL, abs_tol=ABS)
    if isinstance(left, dict) and isinstance(right, dict):
        mine, theirs = cast(dict[str, object], left), cast(dict[str, object], right)
        return mine.keys() == theirs.keys() and all(_same(mine[k], theirs[k]) for k in mine)
    if isinstance(left, list) and isinstance(right, list):
        ours, others = cast(list[object], left), cast(list[object], right)
        return len(ours) == len(others) and all(map(_same, ours, others))
    return left == right


def same_rendering(path: Path, rendered: bytes) -> bool:
    """`path` holds `rendered`: the same bytes, or for JSON the same document to `REL`."""
    if not path.is_file():
        return False
    committed = path.read_bytes()
    if committed == rendered:
        return True
    if path.suffix != ".json":
        return False
    try:
        return _same(json.loads(committed), json.loads(rendered))
    except ValueError:
        return False


def _first_difference(left: object, right: object, where: str) -> str | None:
    if isinstance(left, bool) or isinstance(right, bool):
        return None if left is right else f"{where}: {left!r} != {right!r}"
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        if math.isclose(left, right, rel_tol=REL, abs_tol=ABS):
            return None
        gap = abs(left - right) / max(abs(left), abs(right), ABS)
        return f"{where}: {left!r} != {right!r} (relative {gap:.1e}, allowed {REL:.0e})"
    if isinstance(left, dict) and isinstance(right, dict):
        mine, theirs = cast(dict[str, object], left), cast(dict[str, object], right)
        if mine.keys() != theirs.keys():
            return f"{where}: keys differ by {sorted(mine.keys() ^ theirs.keys())}"
        for key in mine:
            if (found := _first_difference(mine[key], theirs[key], f"{where}.{key}")) is not None:
                return found
        return None
    if isinstance(left, list) and isinstance(right, list):
        ours, others = cast(list[object], left), cast(list[object], right)
        if len(ours) != len(others):
            return f"{where}: {len(ours)} items != {len(others)}"
        for index, (a, b) in enumerate(zip(ours, others, strict=True)):
            if (found := _first_difference(a, b, f"{where}[{index}]")) is not None:
                return found
        return None
    if left == right:
        return None
    ours_text, theirs_text = str(cast(object, left))[:120], str(right)[:120]
    return f"{where}: {ours_text!r} != {theirs_text!r}"


def rendering_difference(path: Path, rendered: bytes) -> str | None:
    """Where `path` first stops holding `rendered` (committed value first), or ``None`` when
    :func:`same_rendering` holds: a red fixture check says which number moved and by how
    much, so a platform's last bits are told from a real change without a second machine."""
    if same_rendering(path, rendered):
        return None
    if not path.is_file():
        return f"{path.name} is missing"
    if path.suffix != ".json":
        return f"{path.name}: the bytes differ"
    try:
        return _first_difference(json.loads(path.read_bytes()), json.loads(rendered), "$")
    except ValueError:
        return f"{path.name} is not JSON"
