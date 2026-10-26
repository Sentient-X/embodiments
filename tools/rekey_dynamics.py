#!/usr/bin/env python3
"""Re-key dynamics documents identified against one description to a re-rendered one.

A dynamics calibration (and every document that records the description it was identified
against: plans, sweeps, the parity fixture) names its description by ``description_sha256``,
and ``Dynamics::calibrated`` refuses one whose digest is not the description's. A re-render
that changes only what dynamics never read (each joint's ``<limit velocity>`` and ``effort``,
each still a finite positive number) leaves every
identified parameter true, so the documents move to the new digest; anything else is refused,
and the calibration must be identified again.

Run from the package root:
``python tools/rekey_dynamics.py --old <old.urdf> --new <new.urdf> <document.json>...``.
The old description comes from git (``git show <commit>:assets/.../robot.urdf > old.urdf``).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


class RekeyError(ValueError):
    """The two descriptions differ in more than joint velocity limits, or a document is not
    keyed to the old one."""


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


#: The ``<limit>`` attributes dynamics never read.
UNREAD = ("velocity", "effort")


def _without_unread(path: Path) -> bytes:
    root = ET.parse(path).getroot()
    for limit in root.iter("limit"):
        for name in UNREAD:
            limit.attrib.pop(name, None)
    return ET.tostring(root)


def _limits_are_numbers(path: Path) -> None:
    for limit in ET.parse(path).getroot().iter("limit"):
        for name in UNREAD:
            text = limit.get(name)
            if text is None:
                raise RekeyError(f"{path}: a <limit> has no {name}")
            try:
                value = float(text)
            except ValueError:
                raise RekeyError(f"{path}: <limit {name}={text!r}> is not a number") from None
            if not (math.isfinite(value) and value > 0.0):
                raise RekeyError(f"{path}: <limit {name}={text!r}> is not finite and positive")


def check_only_velocity(old: Path, new: Path) -> None:
    """Refuse two descriptions that differ in anything but ``<limit velocity>`` and
    ``effort``, and a new one whose values for those are not finite positive numbers."""
    _limits_are_numbers(new)
    if _without_unread(old) != _without_unread(new):
        raise RekeyError(
            f"{new} differs from {old} beyond joint velocity and effort limits: identify again"
        )


def _recorded(node: object) -> list[object]:
    """Every ``description_sha256`` in a document, however deep (a fixture nests one)."""
    if isinstance(node, dict):
        found: list[object] = [v for k, v in node.items() if k == "description_sha256"]
        for value in node.values():
            found += _recorded(value)
        return found
    if isinstance(node, list):
        return [found for item in node for found in _recorded(item)]
    return []


def rekey(document: Path, old_sha: str, new_sha: str, *, write: bool = True) -> bool:
    """Move every ``description_sha256`` in ``document`` from ``old_sha`` to ``new_sha``;
    returns whether it changed."""
    text = document.read_text(encoding="utf-8")
    recorded = _recorded(json.loads(text))
    if not recorded:
        raise RekeyError(f"{document} names no description")
    if all(value == new_sha for value in recorded):
        return False
    if any(value != old_sha for value in recorded):
        raise RekeyError(
            f"{document} names descriptions {sorted(set(map(str, recorded)))}, not {old_sha}"
        )
    field = f'"description_sha256": "{old_sha}"'
    if text.count(field) != len(recorded):
        raise RekeyError(f"{document} does not carry each digest as {field}")
    if write:
        new = text.replace(field, f'"description_sha256": "{new_sha}"')
        document.write_text(new, encoding="utf-8")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--old", type=Path, required=True, help="the description identified against"
    )
    parser.add_argument("--new", type=Path, required=True, help="the re-rendered description")
    parser.add_argument("documents", type=Path, nargs="+")
    arguments = parser.parse_args()
    try:
        check_only_velocity(arguments.old, arguments.new)
        old_sha, new_sha = digest(arguments.old), digest(arguments.new)
        # Every document is checked before any is written: one refused leaves all as they were.
        for document in arguments.documents:
            rekey(document, old_sha, new_sha, write=False)
        for document in arguments.documents:
            moved = rekey(document, old_sha, new_sha)
            sys.stdout.write(f"{'re-keyed' if moved else 'already current'}: {document}\n")
    except (RekeyError, json.JSONDecodeError, OSError) as error:
        sys.stderr.write(f"rekey_dynamics: {error}\n")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
