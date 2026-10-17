"""Modules mate by typed interfaces, and every body in the registry is built from them."""

import math
import random
import xml.etree.ElementTree as ET
from dataclasses import replace

import pytest

from sx_embodiments.assembly import Pose, Subtree
from sx_embodiments.compose import MountedOn, RootMount
from sx_embodiments.composition import compose_build
from sx_embodiments.errors import CompositionError, InterfaceMismatchError, SocketError
from sx_embodiments.kinematics import Kinematics, Transform, compose, from_xyz_rpy
from sx_embodiments.known import definitions, embodiments, modules
from sx_embodiments.known.b601 import B601_GRIPPER_MODULE
from sx_embodiments.known.humanoid import HEAD_MODULE, UPPER_BODY_MODULE
from sx_embodiments.known.panda import PANDA_HAND_MODULE
from sx_embodiments.known.piper import PIPER_GRIPPER_MODULE, PIPER_MODULE
from sx_embodiments.known.realsense import realsense_module
from sx_embodiments.known.so101 import SO101_MODULE
from sx_embodiments.known.universal_robots import UR5E_MODULE
from sx_embodiments.mechanical import (
    MECHANICAL_INTERFACES,
    Box,
    Build,
    Cylinder,
    InterfaceId,
    Mate,
    Module,
    Placed,
    Socket,
    carrier,
    rigid_module,
)
from sx_embodiments.parts import ChainSpec, GripperSpec, MobileBaseSpec

ISO = InterfaceId("iso-9409-1-50-4-m6")
THREAD = InterfaceId("unc-1-4-20")


def _close(a: Transform, b: Transform) -> bool:
    return all(abs(x - y) < 1e-9 for x, y in zip(a, b, strict=True))


def _pose(pose: Pose) -> Transform:
    return from_xyz_rpy(pose.xyz, pose.rpy)


def _up(z: float, y: float = 0.0) -> Transform:
    return from_xyz_rpy((0.0, y, z), (0.0, 0.0, 0.0))


def link_poses(urdf: bytes, joints: dict[str, float], frame: str) -> dict[str, Transform]:
    """Every link's pose in ``frame``, joints not given held at zero."""
    kinematics = Kinematics.of(urdf)
    return kinematics.poses(frame, {name: joints.get(name, 0.0) for name in kinematics.independent})


def _fitted_sockets() -> list[tuple[Module, Socket]]:
    return [
        (module, socket)
        for module in modules()
        for socket in module.sockets
        if socket.fitted is not None
    ]


def _alone(module: Module) -> Build:
    """The module as it was made, on a root of its own, keeping its names."""
    return Build(robot="alone", root="origin", placed=(Placed("host", module, namespaced=False),))


def _owners(urdf: bytes, module: Module) -> dict[str, str]:
    """Which of a module's components each joint of its description is the motion of."""
    owners: dict[str, str] = {}
    for component in _alone(module).components():
        part = component.part
        if isinstance(part, ChainSpec | GripperSpec | MobileBaseSpec):
            for axis in part.layout.axes:
                owners[axis.name] = component.instance
    for joint in ET.fromstring(urdf).findall("joint"):
        mimic = joint.find("mimic")
        if mimic is not None and mimic.get("joint") in owners:
            owners[joint.attrib["name"]] = owners[mimic.attrib["joint"]]
    return owners


def _parents(urdf: bytes) -> dict[str, tuple[str, str]]:
    out: dict[str, tuple[str, str]] = {}
    for joint in ET.fromstring(urdf).findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        assert parent is not None and child is not None
        out[child.attrib["link"]] = (parent.attrib["link"], joint.attrib["name"])
    return out


def test_the_registry_declares_its_detachable_parts() -> None:
    """Every vendor-fitted gripper is its own module on a typed flange, not part of its arm."""
    fitted = {str(socket.fitted.module_id) for _, socket in _fitted_sockets() if socket.fitted}
    assert {
        "piper-gripper",
        "rebot-b601dm-gripper",
        "franka-hand",
        "rby1-left-gripper",
        "rby1-right-gripper",
        "aloha-left-gripper",
        "aloha-right-gripper",
        "ffw-bg2-l-gripper",
        "ffw-bg2-r-gripper",
    } <= fitted
    for module in modules():
        for socket in module.sockets:
            assert socket.interface in MECHANICAL_INTERFACES, (module.module_id, socket.name)
        if module.plug is not None:
            assert module.plug in MECHANICAL_INTERFACES, module.module_id


def test_every_socket_sits_on_a_link_of_its_module() -> None:
    for module in modules():
        links = module.links()
        for socket in module.sockets:
            assert socket.link in links, (module.module_id, socket.name)


def test_a_fitted_socket_is_where_its_cut_hangs() -> None:
    for module, socket in _fitted_sockets():
        assert socket.fitted is not None
        cut = socket.fitted.description
        assert isinstance(cut, Subtree)
        assert socket.link == cut.parent(), (module.module_id, socket.name)
        assert _close(_pose(socket.pose), _pose(cut.pose())), (module.module_id, socket.name)


def test_a_fitted_module_is_detachable() -> None:
    """Cut out and mated back, a fitted module leaves every vendor link where it was, carries
    the same components, and means the same: its metric facts do not move with its seat."""
    rng = random.Random(16)
    for module, socket in _fitted_sockets():
        assert socket.fitted is not None
        shipped = _alone(module)
        reopened = Build(
            robot="reopened",
            root="origin",
            placed=(Placed("host", module, namespaced=False),),
            mates=(Mate("part", socket.fitted, "host", socket.name, namespaced=False),),
        )
        source, rejoined = shipped.urdf(), reopened.urdf()
        joints = {name: rng.uniform(-0.5, 0.5) for name in Kinematics.of(source).independent}
        expected = link_poses(source, joints, _root(source))
        found = link_poses(rejoined, joints, _root(source))
        for name, pose in expected.items():
            assert _close(pose, found[name]), (module.module_id, socket.name, name)
        assert {c.instance: c for c in shipped.components()} == {
            c.instance: c for c in reopened.components()
        }
        seat = _pose(socket.pose)
        for component in socket.fitted.components:
            part = getattr(component, "part", None)
            centre = getattr(part, "grasp_centre_m", None)
            if centre is None:
                continue
            moved = [
                sum(seat[4 * i + k] * centre[k] for k in range(3)) + seat[4 * i + 3]
                for i in range(3)
            ]
            assert moved == pytest.approx(list(centre), abs=1e-6), module.module_id


def _root(urdf: bytes) -> str:
    robot = ET.fromstring(urdf)
    children = {c.attrib["link"] for c in robot.iter("child")}
    (root,) = {link.attrib["name"] for link in robot.findall("link")} - children
    return root


def test_every_socket_names_the_component_that_moves_it() -> None:
    for module in modules():
        if module.description is None or not module.sockets:
            continue
        urdf = _alone(module).urdf()
        parents, owners = _parents(urdf), _owners(urdf, module)
        for socket in module.sockets:
            assert carrier(socket.link, parents, owners) == socket.carrier, (
                module.module_id,
                socket.name,
            )


def test_another_makers_hand_mates_on_an_iso_flange() -> None:
    """The Franka Hand on a UR5e: both flanges are ISO 9409-1-50-4-M6, so no adapter."""
    build = Build(
        robot="ur5e_franka_hand",
        root="origin",
        placed=(Placed("ur5e", UR5E_MODULE),),
        mates=(Mate("hand", PANDA_HAND_MODULE, "ur5e", "flange"),),
    )
    body = compose_build(build, name="ur5e-franka-hand", label="UR5e with a Franka Hand")
    assert body.state.width == embodiments["ur5e"].state.width + 1
    mounts = {c.instance: c.mount for c in body.components}
    assert mounts["hand_gripper"] == MountedOn("ur5e_arm", "hand_panda_hand_joint_mount")
    poses = link_poses(body.urdf_bytes, {}, "origin")
    assert _close(poses["hand_panda_hand_joint_mount"], poses["ur5e_tool0"])


def _adapter(plug: InterfaceId, socket: InterfaceId, thickness: float = 0.01) -> Module:
    return rigid_module(
        f"adapter-{plug}-{socket}",
        f"{plug} to {socket} adapter plate",
        shape=Cylinder(radius=0.035, length=thickness),
        mass_kg=0.05,
        plug=plug,
        sockets={"top": socket},
    )


def test_a_gripper_swap_cuts_out_what_the_vendor_fitted() -> None:
    """A B601 gripper on a Piper through an adapter: the Piper's own jaw is gone, the B601's
    rides on the Piper's arm, and the body is a normal embodiment."""
    build = Build(
        robot="piper_b601_gripper",
        root="origin",
        placed=(Placed("piper", PIPER_MODULE, namespaced=False),),
        mates=(
            Mate(
                "plate",
                _adapter(InterfaceId("agilex-piper-flange"), InterfaceId("seeed-b601-flange")),
                "piper",
                "flange",
            ),
            Mate("b601", B601_GRIPPER_MODULE, "plate", "top"),
        ),
    )
    body = compose_build(build, name="piper-b601-gripper", label="Piper with a B601 gripper")
    links = {link.attrib["name"] for link in ET.fromstring(body.urdf_bytes).findall("link")}
    assert "link7" not in links and "gripper_base" not in links
    instances = [c.instance for c in body.components]
    assert instances == ["arm", "b601_gripper"]
    assert body.components[1].mount == MountedOn("arm", "b601_gripper_joint_mount")
    poses = link_poses(body.urdf_bytes, {}, "origin")
    plate_top = compose(poses["plate_mount"], _up(0.01))
    assert _close(poses["b601_gripper_joint_mount"], plate_top)
    assert _close(poses["plate_mount"], poses["link6"])


def test_a_chain_of_mates_is_the_product_of_its_poses() -> None:
    """Arm, adapter, gripper, fingertip: each frame is its socket's frame on the one before."""
    tipped = replace(
        PIPER_GRIPPER_MODULE,
        sockets=(Socket("tip", InterfaceId("test-fingertip"), "link7", Pose((0.0, 0.0, 0.02))),),
    )
    pad = rigid_module(
        "pad",
        "Fingertip pad",
        shape=Box((0.01, 0.02, 0.004)),
        mass_kg=0.002,
        plug=InterfaceId("test-fingertip"),
    )
    build = Build(
        robot="tipped",
        root="origin",
        placed=(Placed("ur5e", UR5E_MODULE),),
        mates=(
            Mate(
                "plate", _adapter(ISO, InterfaceId("agilex-piper-flange"), 0.012), "ur5e", "flange"
            ),
            Mate("jaw", tipped, "plate", "top"),
            Mate("pad", pad, "jaw", "tip"),
        ),
    )
    joints = {"jaw_joint7": 0.02, "ur5e_wrist_3_joint": 0.7}
    poses = link_poses(build.urdf(), joints, "origin")
    assert _close(poses["plate_mount"], poses["ur5e_tool0"])
    on_plate = compose(poses["plate_mount"], _up(0.012))
    assert _close(poses["jaw_joint6_to_gripper_base_mount"], on_plate)
    tip = compose(poses["jaw_link7"], _up(0.02))
    assert _close(poses["pad_mount"], tip)
    components = {c.instance: c for c in build.components()}
    assert components["jaw_gripper"].mount == MountedOn(
        "ur5e_arm", "jaw_joint6_to_gripper_base_mount"
    )


def test_a_wrong_pairing_is_a_typed_refusal() -> None:
    piper = Placed("piper", PIPER_MODULE, namespaced=False)
    with pytest.raises(InterfaceMismatchError) as refused:
        Build(
            robot="x",
            root="o",
            placed=(piper,),
            mates=(Mate("g", B601_GRIPPER_MODULE, "piper", "flange"),),
        )
    assert refused.value.interface == "agilex-piper-flange"
    with pytest.raises(SocketError, match="no socket"):
        Build(
            robot="x",
            root="o",
            placed=(piper,),
            mates=(Mate("g", PIPER_GRIPPER_MODULE, "piper", "wrist"),),
        )
    with pytest.raises(SocketError, match="already carries"):
        Build(
            robot="x",
            root="o",
            placed=(piper,),
            mates=(
                Mate("g", PIPER_GRIPPER_MODULE, "piper", "flange"),
                Mate("h", PIPER_GRIPPER_MODULE, "piper", "flange"),
            ),
        )
    with pytest.raises(SocketError, match="no plug"):
        Build(
            robot="x",
            root="o",
            placed=(Placed("ur5e", UR5E_MODULE),),
            mates=(Mate("p", PIPER_MODULE, "ur5e", "flange"),),
        )
    with pytest.raises(SocketError, match="not a module placed or mated before"):
        Build(
            robot="x",
            root="o",
            placed=(piper,),
            mates=(Mate("g", PIPER_GRIPPER_MODULE, "nobody", "flange"),),
        )
    with pytest.raises(InterfaceMismatchError):
        Socket("flange", ISO, "link6", fitted=PIPER_GRIPPER_MODULE)
    with pytest.raises(SocketError, match="does not have"):
        Build(
            robot="x",
            root="o",
            placed=(
                Placed(
                    "ur5e",
                    replace(UR5E_MODULE, sockets=(Socket("flange", ISO, "no_such_link"),)),
                ),
            ),
            mates=(Mate("p", replace(PIPER_GRIPPER_MODULE, plug=ISO), "ur5e", "flange"),),
        ).urdf()
    with pytest.raises(SocketError, match="not one of its components"):
        replace(UR5E_MODULE, sockets=(Socket("flange", ISO, "tool0", carrier="wrist"),))


def test_a_camera_module_composes_on_a_wrist_through_a_bracket() -> None:
    """One D435 module, from Intel's description, on a UR5e flange and a humanoid wrist."""
    d435 = realsense_module("d435")
    on_ur = Build(
        robot="ur_camera",
        root="origin",
        placed=(Placed("ur5e", UR5E_MODULE),),
        mates=(
            Mate("bracket", _adapter(ISO, THREAD), "ur5e", "flange"),
            Mate("camera", d435, "bracket", "top"),
        ),
    )
    poses = link_poses(on_ur.urdf(), {}, "origin")
    screw = compose(poses["bracket_mount"], _up(0.01))
    assert _close(poses["camera_bottom_screw_frame"], screw)
    # Intel's nominal colour frame: 15 mm along +y of the camera body from the depth frame.
    colour = compose(poses["camera_camera_link"], _up(0.0, 0.015))
    assert _close(poses["camera_color_frame"], colour)
    on_wrist = Build(
        robot="humanoid_camera",
        root="humanoid_origin",
        placed=(Placed("upper_body", UPPER_BODY_MODULE, namespaced=False),),
        mates=(
            Mate("head", HEAD_MODULE, "upper_body", "neck", namespaced=False),
            Mate(
                "bracket",
                _adapter(InterfaceId("sentient-humanoid-wrist"), THREAD),
                "upper_body",
                "right_wrist",
            ),
            Mate("camera", d435, "bracket", "top"),
        ),
    )
    assert "camera_color_optical_frame" in {
        link.attrib["name"] for link in ET.fromstring(on_wrist.urdf()).findall("link")
    }


def test_a_camera_with_optics_rides_on_what_carries_its_bracket() -> None:
    quest = embodiments["quest-ego"].components[0].part
    d405 = realsense_module("d405", {"color": quest})  # type: ignore[dict-item]
    build = Build(
        robot="wrist_camera",
        root="origin",
        placed=(Placed("ur5e", UR5E_MODULE),),
        mates=(
            Mate("bracket", _adapter(ISO, THREAD), "ur5e", "flange"),
            Mate("wrist", d405, "bracket", "top"),
        ),
    )
    (camera,) = (c for c in build.components() if c.instance == "wrist_color")
    assert camera.mount == MountedOn("ur5e_arm", "wrist_color_optical_frame")


def test_an_so101_mates_in_the_heads_place_and_rides_with_the_torso() -> None:
    build = Build(
        robot="so101_neck",
        root="humanoid_origin",
        placed=(Placed("upper_body", UPPER_BODY_MODULE, namespaced=False),),
        mates=(
            Mate(
                "seat",
                _adapter(InterfaceId("sentient-humanoid-neck"), InterfaceId("so101-base")),
                "upper_body",
                "neck",
            ),
            Mate("neck", SO101_MODULE, "seat", "top"),
        ),
    )
    body = compose_build(build, name="humanoid-so101-neck", label="Humanoid with an SO-101 neck")
    mounts = {c.instance: c.mount for c in body.components}
    assert mounts["neck_arm"] == MountedOn("torso", "neck_base_link")
    assert mounts["neck_jaw"] == MountedOn("neck_arm", "neck_gripper_link")
    links = {link.attrib["name"] for link in ET.fromstring(body.urdf_bytes).findall("link")}
    assert "head_link" not in links


def test_a_rigid_element_is_its_solid() -> None:
    plate = rigid_module(
        "plate", "Plate", shape=Box((0.1, 0.2, 0.02)), mass_kg=0.3, plug=ISO, sockets={"top": ISO}
    )
    assert isinstance(plate.description, bytes)
    inertia = ET.fromstring(plate.description).find("link/inertial/inertia")
    assert inertia is not None
    assert float(inertia.attrib["ixx"]) == pytest.approx(0.3 * (0.2**2 + 0.02**2) / 12)
    assert float(inertia.attrib["izz"]) == pytest.approx(0.3 * (0.1**2 + 0.2**2) / 12)
    (top,) = plate.sockets
    assert top.pose == Pose((0.0, 0.0, 0.02)) and top.link == "mount"
    rod = Cylinder(radius=0.01, length=0.1)
    assert rod.inertia(1.0)[2] == pytest.approx(0.5 * 0.01**2)
    for mass in (0.0, -1.0, math.inf):
        with pytest.raises(CompositionError):
            rigid_module("bad", "Bad", shape=rod, mass_kg=mass, plug=None)


def test_every_definition_is_built_and_a_build_without_a_root_is_one_module() -> None:
    for definition in definitions():
        build = definition.build
        assert build.placed, definition.name
        if build.root is None:
            assert len(build.placed) == 1 and not build.mates
    with pytest.raises(CompositionError):
        Build(placed=(Placed("a", SO101_MODULE), Placed("b", SO101_MODULE)))
    assert Build.of(SO101_MODULE).components() == embodiments["so101"].components


def test_a_root_level_mate_on_a_humanoid_keeps_the_registry_body() -> None:
    """The registry's upper body is exactly its build: placed torso and arms, the head mated."""
    upper = next(d for d in definitions() if d.name == "sentient-humanoid-upper-body")
    assert [m.socket for m in upper.build.mates] == ["neck"]
    assert all(isinstance(c.mount, RootMount | MountedOn) for c in upper.attachments)
