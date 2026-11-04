"""Every body declares its frames: a facing, axes on every link, and roles in place of names.

No hand list: the bodies are both registries' (every one the registry can describe), and each
check reads only what the body declares (``sx_embodiments.frames``). A body's declared front is
held to its own geometry wherever the geometry shows it: its two tools hang from branches that
part along its left, and a camera that does not ride a tool looks ahead with image-right to the
body's right. Run with ``uv run pytest packages/sx-embodiments/tests/test_frames.py``.
"""

import math
import xml.etree.ElementTree as ET

import pytest

from sx_embodiments import development_embodiments, embodiments
from sx_embodiments.embodiment import Embodiment
from sx_embodiments.parts import CameraSpec

np = pytest.importorskip("numpy")
frames_kernel = pytest.importorskip("sx_geometry.frames")

from sx_embodiments.assembly import OPTICAL_RPY, Frame, Pose  # noqa: E402
from sx_embodiments.errors import CompositionError  # noqa: E402
from sx_embodiments.frames import (  # noqa: E402
    OPTICAL_AXES,
    FrameRole,
    body_frames,
    root_axes,
    root_link,
    sides,
    tools,
)
from sx_embodiments.kinematics import Kinematics, KinematicsError  # noqa: E402

FLU, RDF, Direction = frames_kernel.FLU, frames_kernel.RDF, frames_kernel.Direction

BODIES = {
    **dict.fromkeys(embodiments, embodiments),
    **dict.fromkeys(development_embodiments, development_embodiments),
}


def _body(name: str) -> Embodiment:
    return BODIES[name][name]


def _rest(kinematics: Kinematics) -> dict[str, float]:
    return dict.fromkeys(kinematics.independent, 0.0)


def _in_root(body: Embodiment, root: str) -> np.ndarray:
    """The body's forward, left and up as rows, in its root link's coordinates."""
    return frames_kernel.basis(root_axes(body), FLU).T


@pytest.mark.parametrize("name", list(BODIES))
def test_every_frame_declares_its_axes_and_every_role_names_a_link(name: str) -> None:
    body = _body(name)
    links = [link.get("name") for link in ET.fromstring(body.urdf_bytes).findall("link")]
    frames = body_frames(body)  # refuses an undeclared front or a role on an absent link
    assert [frame.name for frame in frames] == links
    assert all(frame.frame.axes.handedness is frames_kernel.Handedness.RIGHT for frame in frames)
    root = next(frame for frame in frames if frame.name == root_link(body))
    assert root.frame.axes == root_axes(body)


@pytest.mark.parametrize("name", list(BODIES))
def test_every_camera_is_a_view_on_an_optical_frame(name: str) -> None:
    body = _body(name)
    frames = {frame.name: frame for frame in body_frames(body)}
    cameras = {c.mount.frame for c in body.components if isinstance(c.part, CameraSpec)}
    views = {name for name, frame in frames.items() if FrameRole.VIEW in frame.roles}
    assert views == cameras
    assert all(frames[camera].frame.axes == OPTICAL_AXES for camera in cameras)


@pytest.mark.parametrize("name", list(BODIES))
def test_every_tool_is_a_tool_frame(name: str) -> None:
    body = _body(name)
    tool_frames = {frame for _, frame in tools(body)}
    roles = {frame.name for frame in body_frames(body) if FrameRole.TOOL in frame.roles}
    assert roles == tool_frames


@pytest.mark.parametrize("name", list(BODIES))
def test_the_declared_front_agrees_with_the_body(name: str) -> None:
    """Two tools part along the body's left; a camera not on a tool looks ahead, upright."""
    body = _body(name)
    kinematics, root = Kinematics.of(body.urdf_bytes), root_link(body)
    forward, left, up = _in_root(body, root)
    placed = [frame for _, frame in tools(body)]
    if len(placed) == 2:
        try:
            first, second = (
                np.asarray(kinematics.pose(root, branch, _rest(kinematics)), float).reshape(4, 4)
                for branch in _parting(kinematics, *placed)
            )
        except KinematicsError:  # floating hands: an operator mount, if any, says the side
            pass
        else:
            apart = first[:3, 3] - second[:3, 3]
            assert abs(apart @ left) > 0.9 * np.linalg.norm(apart), (name, apart)
    on_tools = set(placed)
    for component in body.components:
        if not isinstance(component.part, CameraSpec):
            continue
        frame = component.mount.frame
        if on_tools & _above(kinematics, frame):
            continue  # a wrist camera looks where its tool points
        try:
            pose = np.asarray(kinematics.pose(root, frame, _rest(kinematics)), float).reshape(4, 4)
        except KinematicsError:
            continue
        image_right, image_down, view = pose[:3, :3].T
        assert view @ forward > 0.5, (name, frame, view)
        assert image_right @ -left > 0.5, (name, frame, image_right)
        assert image_down @ -up > 0.5, (name, frame, image_down)


@pytest.mark.parametrize("name", list(BODIES))
def test_two_tools_are_on_two_sides(name: str) -> None:
    """Two tools are a left and a right one, wherever the body says so: by an operator mount,
    or by a description that places both at rest. Floating hands with no operator mount have
    no side (piper-umi, 2026-11-04: declaring its mounts moves its id and its sealed bytes)."""
    body = _body(name)
    kinematics = Kinematics.of(body.urdf_bytes)
    placed = [frame for _, frame in tools(body)]
    if len(placed) != 2:
        return
    mounted = {mount.root_frame for mount in body.operator_mounts}
    floating = any(
        _above(kinematics, frame) & mounted == set() and _floats(kinematics, frame)
        for frame in placed
    )
    found = sides(body, placed)
    if floating:
        assert found == {}, found
    else:
        assert sorted(found.values()) == [Direction.LEFT, Direction.RIGHT], found


def test_the_optical_rotation_is_the_kernels() -> None:
    """The one optical rpy the registry writes is the rotation the two axes imply."""
    computed = frames_kernel.URDF_RPY.angles(frames_kernel.basis(FLU, RDF))
    assert all(
        math.isclose(a, b, abs_tol=1e-15) for a, b in zip(OPTICAL_RPY, computed, strict=True)
    )


def test_an_optical_frame_takes_its_rotation_from_its_axes() -> None:
    frame = Frame("eye_optical", "eye", Pose((0.0, 0.01, 0.0)), camera_optical=True)
    assert frame.placement == Pose((0.0, 0.01, 0.0), OPTICAL_RPY)
    with pytest.raises(CompositionError):
        Frame("eye_optical", "eye", Pose(rpy=(0.1, 0.0, 0.0)), camera_optical=True)


def _lineage(kinematics: Kinematics, link: str) -> list[str]:
    found = [link]
    while (joint := kinematics.parent_joint.get(link)) is not None:
        link = kinematics.joints[joint].parent
        found.append(link)
    return found[::-1]


def _above(kinematics: Kinematics, link: str) -> set[str]:
    return set(_lineage(kinematics, link))


def _floats(kinematics: Kinematics, link: str) -> bool:
    """Whether a joint one position cannot place (floating, planar) carries ``link``."""
    try:
        kinematics.pose(_lineage(kinematics, link)[0], link, _rest(kinematics))
    except KinematicsError:
        return True
    return False


def _parting(kinematics: Kinematics, a: str, b: str) -> tuple[str, str]:
    up_a, up_b = _lineage(kinematics, a), _lineage(kinematics, b)
    shared = next(i for i, (x, y) in enumerate(zip(up_a, up_b, strict=False)) if x != y)
    return up_a[shared], up_b[shared]
