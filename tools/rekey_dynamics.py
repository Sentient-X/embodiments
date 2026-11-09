#!/usr/bin/env python3
"""Re-key dynamics documents identified against one description to a re-rendered one.

A dynamics calibration (and every document that records the description it was identified
against: plans, sweeps, the parity fixture) names its description by ``description_sha256``,
and ``Dynamics::calibrated`` refuses one whose digest is not the description's. A re-render
leaves every identified parameter true, so the documents move to the new digest, when it
changes only

* what dynamics never read (each joint's ``<limit velocity>`` and ``effort``, each still a
  finite positive number), or
* links outside every identified joint's reach: no document identifies the link, no joint a
  document identifies lies between the root and it (a changed joint counts as a change to its
  child), and no joint above an identified one moves. A fixed link inserted above an
  identified joint that leaves it where it was is no move: the moving joints above it are the
  same and so is where each sits on the next, fixed joints composed in. The torque at an
  identified joint depends only on the links it carries and the joints above it, so a fit of
  the arms keeps when the neck's camera mount is corrected. ``Dynamics::superseding`` applies
  the same rule at the station, so a calibration keyed to an earlier description the package
  keeps (``_by_digest``) needs no re-key; this tool moves the repository's own documents.

Anything else is refused, and the calibration must be identified again.

Run from the package root:
``python tools/rekey_dynamics.py --old <old.urdf> --new <new.urdf> <document.json>...``.
The old description comes from git (``git show <commit>:assets/.../robot.urdf > old.urdf``).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
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


def _identified(node: object) -> tuple[set[str], set[str]]:
    """The links and joints every calibration in a document identifies, however deep."""
    links: set[str] = set()
    joints: set[str] = set()
    if isinstance(node, dict):
        for key, field in (("links", "link"), ("joints", "joint")):
            value = node.get(key)
            if isinstance(value, list):
                # A calibration's entries name theirs; a plan lists the joints it excites.
                named = {
                    str(item[field]) if isinstance(item, dict) else item
                    for item in value
                    if (isinstance(item, dict) and field in item) or isinstance(item, str)
                }
                (links if key == "links" else joints).update(named)
        for value in node.values():
            more_links, more_joints = _identified(value)
            links |= more_links
            joints |= more_joints
    elif isinstance(node, list):
        for item in node:
            more_links, more_joints = _identified(item)
            links |= more_links
            joints |= more_joints
    return links, joints


#: How far two placements of one joint may differ, per transform entry, and still be one: a
#: fixed joint composed into the next moves the last bits, never a millimetre.
PLACEMENT_EPSILON = 1e-12

Matrix = tuple[tuple[float, ...], ...]


class _Description:
    """One description's links and joints (velocity and effort dropped), by name."""

    def __init__(self, path: Path) -> None:
        root = ET.fromstring(_without_unread(path))
        self.file = path
        self.links = {link.get("name", ""): link for link in root.findall("link")}
        self.joints = {joint.get("name", ""): joint for joint in root.findall("joint")}
        self.tree: dict[str, tuple[str, str]] = {}
        for name, joint in self.joints.items():
            parent, child = joint.find("parent"), joint.find("child")
            if parent is None or child is None:
                raise RekeyError(f"{path}: joint {name} names no parent or child")
            self.tree[name] = (parent.get("link", ""), child.get("link", ""))
        self.above = {child: joint for joint, (_, child) in self.tree.items()}

    def kind(self, joint: str) -> str:
        return self.joints[joint].get("type", "")

    def moves(self, joint: str) -> bytes:
        """The joint as it moves, without where it sits: its element less origin and parent."""
        element = ET.fromstring(ET.tostring(self.joints[joint]))
        for tag in ("origin", "parent"):
            for found in element.findall(tag):
                element.remove(found)
        return ET.tostring(element)

    def placement(self, joint: str) -> Matrix:
        origin = self.joints[joint].find("origin")
        xyz = _floats(origin.get("xyz") if origin is not None else None)
        rpy = _floats(origin.get("rpy") if origin is not None else None)
        return _transform(xyz, rpy)

    def path(self, joint: str) -> list[tuple[str, Matrix]]:
        """The moving joints from the root down to ``joint`` (which ends it, moving or not),
        each with its placement on the moving joint above it, fixed joints composed in."""
        up = [joint]
        while (parent := self.tree[up[-1]][0]) in self.above:
            up.append(self.above[parent])
        out: list[tuple[str, Matrix]] = []
        placed: Matrix | None = None
        for name in reversed(up):
            at = self.placement(name) if placed is None else _compose(placed, self.placement(name))
            if self.kind(name) == "fixed" and name != joint:
                placed = at
            else:
                out.append((name, at))
                placed = None
        return out


def _floats(text: str | None) -> tuple[float, float, float]:
    values = [float(value) for value in (text or "0 0 0").split()]
    if len(values) != 3:
        raise RekeyError(f"an origin {text!r} is not three numbers")
    return values[0], values[1], values[2]


def _transform(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> Matrix:
    """A URDF origin: translation, then fixed-axis roll, pitch, yaw."""
    (cr, sr), (cp, sp), (cy, sy) = ((math.cos(a), math.sin(a)) for a in rpy)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]),
        (-sp, cp * sr, cp * cr, xyz[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def _compose(a: Matrix, b: Matrix) -> Matrix:
    return tuple(
        tuple(sum(a[row][k] * b[k][column] for k in range(4)) for column in range(4))
        for row in range(4)
    )


def _same_place(a: Matrix, b: Matrix) -> bool:
    return all(
        abs(x - y) <= PLACEMENT_EPSILON
        for row_a, row_b in zip(a, b, strict=True)
        for x, y in zip(row_a, row_b, strict=True)
    )


def check_rekeyable(old: Path, new: Path, identified: tuple[set[str], set[str]]) -> None:
    """Refuse two descriptions whose difference reaches an identified link or joint (see the
    module), and a new one whose velocity or effort limits are not finite positive numbers.

    For each identified joint (and the joint above each identified link) the links and joints
    it carries must be the same, by name and element, and so must the moving joints between
    the root and it and where each sits on the one above: a fixed link inserted above it that
    leaves it in place (the static upper body's shoulder mounts) reaches no fit. Anything else
    may differ."""
    _limits_are_numbers(new)
    if _without_unread(old) == _without_unread(new):
        return
    links, joints = identified
    before, after = _Description(old), _Description(new)
    fitted = {joint for joint in joints if joint in after.tree}
    for link in links:
        if link not in after.links:
            raise RekeyError(f"{new} has no identified link {link}: identify again")
        if link not in after.above:
            raise RekeyError(
                f"{new} changes the root link {link}, which carries all: identify again"
            )
        fitted.add(after.above[link])
    if joints - set(after.tree):
        raise RekeyError(f"{new} has no identified joint {sorted(joints - set(after.tree))}")
    for joint in sorted(fitted):
        child = after.tree[joint][1]
        if child not in before.above:
            raise RekeyError(f"{new} has other links or joints than {old}: identify again")
        earlier = before.above[child]
        mine, theirs = after.path(joint), before.path(earlier)
        if len(mine) != len(theirs):
            raise RekeyError(
                f"{new} moves the joints above identified joint {joint}: identify again"
            )
        for (name, placed), (other, was) in zip(mine, theirs, strict=True):
            same = (
                after.tree[name][1] == before.tree[other][1]
                and after.moves(name) == before.moves(other)
                and _same_place(placed, was)
            )
            if same:
                continue
            if name == joint and joint in joints:
                raise RekeyError(f"{new} changes identified joint {joint}: identify again")
            if name == joint:
                raise RekeyError(f"{new} changes identified link {child}: identify again")
            raise RekeyError(f"{new} moves {name}, above identified joint {joint}: identify again")
        # Everything it carries, by name and element.
        below = [child]
        while below:
            link = below.pop()
            if link not in before.links or ET.tostring(after.links[link]) != ET.tostring(
                before.links[link]
            ):
                if link in links:
                    raise RekeyError(f"{new} changes identified link {link}: identify again")
                raise RekeyError(
                    f"{new} changes {link}, which identified joint {joint} carries: identify again"
                )
            mine_below = {n for n, (parent, _) in after.tree.items() if parent == link}
            theirs_below = {n for n, (parent, _) in before.tree.items() if parent == link}
            for name in sorted(mine_below | theirs_below):
                if (
                    name not in mine_below
                    or name not in theirs_below
                    or ET.tostring(after.joints[name]) != ET.tostring(before.joints[name])
                ):
                    if name in joints:
                        raise RekeyError(f"{new} changes identified joint {name}: identify again")
                    raise RekeyError(
                        f"{new} changes {name}, which identified joint {joint} carries: "
                        "identify again"
                    )
                below.append(after.tree[name][1])


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
    # Any JSON spacing: a station writes its calibrations compact.
    field = re.compile(rf'("description_sha256"\s*:\s*")({old_sha})(")')
    new, count = field.subn(rf"\g<1>{new_sha}\g<3>", text)
    if count != len(recorded):
        raise RekeyError(f"{document} does not carry each digest as a plain string field")
    if write:
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
        identified: tuple[set[str], set[str]] = (set(), set())
        for document in arguments.documents:
            links, joints = _identified(json.loads(document.read_text(encoding="utf-8")))
            identified = (identified[0] | links, identified[1] | joints)
        check_rekeyable(arguments.old, arguments.new, identified)
        old_sha, new_sha = digest(arguments.old), digest(arguments.new)
        # Every document is checked before any is written: one refused leaves all as they were.
        for document in arguments.documents:
            rekey(document, old_sha, new_sha, write=False)
        for document in arguments.documents:
            moved = rekey(document, old_sha, new_sha)
            sys.stdout.write(f"{'re-keyed' if moved else 'already current'}: {document}\n")
    except (RekeyError, json.JSONDecodeError, OSError, UnicodeDecodeError, ET.ParseError) as error:
        sys.stderr.write(f"rekey_dynamics: {error}\n")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
