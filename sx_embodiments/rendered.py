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
