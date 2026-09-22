"""Bind component-local coordinates to unique joints in their description subtree."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections import Counter

from .compose import Component
from .errors import LayoutError
from .layout import StateSpace


def joint_names(
    components: tuple[Component, ...], state: StateSpace, urdf: bytes
) -> tuple[str, ...]:
    """Resolve exact names, or qualified local names within one physical subtree.

    Older packaged assemblies retain local joint names in each repeated part. Their
    mount frame scopes resolution; ambiguous or absent bindings are errors, never a
    left/right naming table or a vector-position guess.
    """
    root = ET.fromstring(urdf)
    joints = {item.attrib["name"]: item for item in root.findall("joint")}
    counts = Counter(coordinate.joint_name for coordinate in state.coordinates)
    mounts = {component.component_id: component.mount.frame for component in components}
    result: list[str] = []
    for coordinate in state.coordinates:
        local = coordinate.joint_name
        if local in joints and counts[local] == 1:
            result.append(local)
            continue
        frame = mounts[coordinate.instance]
        boundaries = set(mounts.values()) - {frame}
        reachable = {frame}
        scoped: set[str] = set()
        changed = True
        while changed:
            changed = False
            for name, joint in joints.items():
                parent, child = joint.find("parent"), joint.find("child")
                if parent is None or child is None:
                    raise LayoutError("description", f"joint {name!r} lacks a parent or child")
                if (
                    parent.attrib["link"] in reachable
                    and parent.attrib["link"] not in boundaries
                    and name not in scoped
                ):
                    scoped.add(name)
                    reachable.add(child.attrib["link"])
                    changed = True
        candidates = [
            name for name in scoped if name == local or name.endswith(("/" + local, "_" + local))
        ]
        if len(candidates) != 1:
            raise LayoutError(
                "description",
                f"{coordinate.instance}/{local} needs one joint in its mount subtree; "
                f"got {sorted(candidates)}",
            )
        result.append(candidates[0])
    if len(set(result)) != len(result):
        raise LayoutError(
            "description", "multiple native coordinates resolve to the same URDF joint"
        )
    return tuple(result)
