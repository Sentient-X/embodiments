"""Part specs mirror the description assets they claim to mirror (stdlib xml only).

Scoped to the parts whose docstrings assert URDF parity: SO-101, the DAS jaw, the Sentient
humanoid, and the reBot B601-DM. The Piper spec deliberately diverges from its menagerie
MJCF (deployed limits win; see ``known/piper.py``), so it is NOT pinned here.
"""

import xml.etree.ElementTree as ET
from math import radians
from pathlib import Path

from sx_embodiments import embodiments, resolve_asset
from sx_embodiments.known.b601 import B601_ARM, B601_DM_URDF, B601_GRIPPER
from sx_embodiments.known.das import DAS_JAW_V4
from sx_embodiments.known.humanoid import (
    SENTIENT_HUMANOID_LEFT_ARM,
    SENTIENT_HUMANOID_LEFT_LEG,
    SENTIENT_HUMANOID_RIGHT_ARM,
    SENTIENT_HUMANOID_RIGHT_LEG,
    SENTIENT_HUMANOID_TORSO,
    SENTIENT_HUMANOID_URDF,
)
from sx_embodiments.known.so101 import SO101_ARM, SO101_JAW, SO101_URDF


def _movable_joints(path: Path) -> dict[str, tuple[float, float, ET.Element]]:
    joints: dict[str, tuple[float, float, ET.Element]] = {}
    for joint in ET.parse(path).getroot().iter("joint"):
        if joint.get("type") in (None, "fixed"):
            continue
        limit = joint.find("limit")
        assert limit is not None
        name = joint.get("name")
        assert name is not None
        joints[name] = (float(limit.get("lower", "0")), float(limit.get("upper", "0")), joint)
    return joints


def test_so101_spec_matches_urdf() -> None:
    joints = _movable_joints(SO101_URDF.path())
    assert set(joints) == {*SO101_ARM.joint_names, *SO101_JAW.joint_names}
    for i, name in enumerate(SO101_ARM.joint_names):
        lower, upper, _ = joints[name]
        assert (lower, upper) == (SO101_ARM.joint_lower[i], SO101_ARM.joint_upper[i])
    lower, upper, _ = joints[SO101_JAW.joint_names[0]]
    assert (lower, upper) == (SO101_JAW.joint_lower[0], SO101_JAW.joint_upper[0])


def test_b601_spec_matches_urdf_and_records_the_driver_divergence() -> None:
    """The B601-DM channels ARE the vendored URDF's movable joints (the Piper precedent).

    Every name, unit, and limit comes from the description; the deployed driver's second
    vocabulary and its disagreeing soft box are pinned here too, so the divergence the
    module docstring describes cannot drift silently.
    """
    joints = _movable_joints(B601_DM_URDF.path())
    assert set(joints) == {
        *B601_ARM.joint_names,
        *B601_GRIPPER.joint_names,
        *(mimic.joint_name for mimic in B601_GRIPPER.mimic_joints),
    }
    for index, name in enumerate(B601_ARM.joint_names):
        lower, upper, element = joints[name]
        assert (lower, upper) == (B601_ARM.joint_lower[index], B601_ARM.joint_upper[index])
        assert element.get("type") == "revolute"

    finger, mirror = "gripper_joint1", "gripper_joint2"
    lower, upper, element = joints[finger]
    assert (lower, upper) == (B601_GRIPPER.joint_lower[0], B601_GRIPPER.joint_upper[0])
    assert element.get("type") == "prismatic"
    assert element.find("mimic") is None  # the driven finger mimics nothing
    # The mirror's coupling is a declared URDF fact (the vendoring patch: upstream's
    # SolidWorks export omitted it), so the seven-channel layout is DERIVED from the
    # description rather than asserted around a gap — the DAS jaw's shape.
    mirror_lower, mirror_upper, mirror_element = joints[mirror]
    assert (mirror_lower, mirror_upper) == (lower, upper)  # identical stroke
    declared = mirror_element.find("mimic")
    assert declared is not None
    (recorded,) = B601_GRIPPER.mimic_joints
    assert recorded.joint_name == mirror
    assert declared.get("joint") == recorded.of == finger
    assert float(declared.get("multiplier", "1")) == recorded.multiplier
    assert B601_GRIPPER.travel_m == (0.0, 2 * upper)  # aperture = 2 x finger stroke

    # The deployed driver's box (degrees) against the described box (radians), per joint.
    driver_deg = {
        "joint1": (-150.0, 150.0),  # shoulder_pan
        "joint2": (-200.0, 1.0),  # shoulder_lift
        "joint3": (-200.0, 1.0),  # elbow_flex
        "joint4": (-80.0, 90.0),  # wrist_flex
        "joint5": (-90.0, 90.0),  # wrist_yaw
        "joint6": (-90.0, 90.0),  # wrist_roll
    }
    tighter = {"joint1", "joint6"}  # driver clips inside the described stop
    looser = {"joint2", "joint3"}  # driver admits ~20 deg beyond it (upstream discrepancy)
    for name, (driver_lo, driver_hi) in driver_deg.items():
        described_lo, described_hi, _ = joints[name]
        if name in tighter:
            assert described_lo < radians(driver_lo) and radians(driver_hi) < described_hi
        elif name in looser:
            assert radians(driver_lo) < described_lo and described_hi < radians(driver_hi)
        else:  # joint4/joint5: agree to the URDF's 1.57-for-90-deg rounding
            assert abs(radians(driver_hi) - described_hi) < 1e-3
    assert B601_ARM.home_joints == (0.0,) * 6  # the driver's calibrated zero pose
    assert all(
        lo <= home <= hi  # joint2/joint3 close AT zero: the described upper limit is 0.0
        for lo, home, hi in zip(
            B601_ARM.joint_lower, B601_ARM.home_joints, B601_ARM.joint_upper, strict=True
        )
    )


def test_das_jaw_matches_urdf_mimic_chain() -> None:
    from sx_embodiments.known.das import DAS_GRIPPER_URDF

    joints = _movable_joints(DAS_GRIPPER_URDF.path())
    assert set(joints) == {
        *DAS_JAW_V4.joint_names,
        *(m.joint_name for m in DAS_JAW_V4.mimic_joints),
    }
    lower, upper, _ = joints["joint_1"]
    assert (lower, upper) == (0.0, 0.925)
    for mimic in DAS_JAW_V4.mimic_joints:
        _, _, element = joints[mimic.joint_name]
        declared = element.find("mimic")
        assert declared is not None, mimic.joint_name
        assert declared.get("joint") == mimic.of
        assert float(declared.get("multiplier", "1")) == mimic.multiplier


def test_das_gap_curve_shape() -> None:
    curve = DAS_JAW_V4.gap_curve
    assert curve is not None
    assert curve.at(0.0) == 0.105  # geometric full-open
    assert curve.at(0.925) == 0.00011  # closed
    # inverse round-trips through the measured region
    for q in (0.1, 0.42, 0.8):
        assert abs(curve.inverse_at(curve.at(q)) - q) < 1e-9


def test_sentient_humanoid_executed_groups_match_hardware_urdf() -> None:
    joints = _movable_joints(SENTIENT_HUMANOID_URDF.path())
    parts = (
        SENTIENT_HUMANOID_TORSO,
        SENTIENT_HUMANOID_RIGHT_ARM,
        SENTIENT_HUMANOID_LEFT_ARM,
        SENTIENT_HUMANOID_RIGHT_LEG,
        SENTIENT_HUMANOID_LEFT_LEG,
    )
    for part in parts:
        for index, name in enumerate(part.joint_names):
            lower, upper, _ = joints[name]
            assert (lower, upper) == (part.joint_lower[index], part.joint_upper[index])

    executed = {name for part in parts for name in part.joint_names}
    assert "waist_rod_joint" not in executed  # feedback/passive, never an action row
    assert {"neck_yaw_joint", "head_joint"}.isdisjoint(executed)  # no motor assignment


def test_sentient_rwh_layout_mirrors_the_export_and_pins_why_it_is_development_only() -> None:
    """The RWH's held-back facts are asserted, not just described in its docstring.

    Each assertion here is a tripwire on the *next* CAD export: when the limit table
    lands, or the ``RA_7`` mate is repaired, this test fails and forces the registry to
    catch up rather than silently keeping a stale shape.
    """
    from sx_embodiments.identity import EmbodimentName
    from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS, DevelopmentReason
    from sx_embodiments.known.sentient_rwh import (
        SENTIENT_RWH_BASE,
        SENTIENT_RWH_LEFT_ARM,
        SENTIENT_RWH_RIGHT_ARM,
        SENTIENT_RWH_TORSO,
        SENTIENT_RWH_URDF,
    )

    entry = DEVELOPMENT_EMBODIMENTS[EmbodimentName("sentient-rwh")]
    assert entry.reason is DevelopmentReason.MISSING_JOINT_LIMITS
    assert "sentient-rwh" not in set(embodiments)  # never production while limits are absent

    root = ET.parse(SENTIENT_RWH_URDF.path()).getroot()
    joints = {
        name: joint for joint in root.iter("joint") if (name := joint.get("name")) is not None
    }
    movable = {name: j for name, j in joints.items() if j.get("type") not in (None, "fixed")}

    # 1. NO joint declares a limit — the fact that holds this body in development.
    assert len(movable) == 56
    assert all(j.get("type") == "continuous" for j in movable.values())
    articulated = [j for n, j in movable.items() if not n.endswith("_tyre")]
    assert all(j.find("limit") is None for j in articulated)

    # 2. Every declared channel is a movable joint of the description, in export order.
    declared = (
        *SENTIENT_RWH_BASE.channel_names,
        *SENTIENT_RWH_TORSO.joint_names,
        *SENTIENT_RWH_RIGHT_ARM.joint_names,
        *SENTIENT_RWH_LEFT_ARM.joint_names,
    )
    assert set(declared) <= set(movable)
    assert len(declared) == 24
    assert [name for name in movable if name in set(declared)] == list(declared)

    # 3. The arm asymmetry: RA_7 is fixed, its mirror LA_7 is not.
    assert joints["RA_7"].get("type") == "fixed"
    assert joints["LA_7"].get("type") == "continuous"
    assert SENTIENT_RWH_RIGHT_ARM.dof == 7 and SENTIENT_RWH_LEFT_ARM.dof == 8
    assert "RA_7" not in SENTIENT_RWH_RIGHT_ARM.joint_names

    # 4. The tilted left shoulder against the plain right one.
    assert joints["RA_1"].find("axis").get("xyz") == "-1 0 0"  # type: ignore[union-attr]
    left_axis = [float(v) for v in joints["LA_1"].find("axis").get("xyz", "").split()]  # type: ignore[union-attr]
    assert abs(left_axis[0] - 0.965925826289068) < 1e-12
    assert abs(left_axis[2] + 0.258819045102521) < 1e-12

    # 5. Both 16-DOF hands are complete in the description and absent from the layout.
    for side, wrist in (("RA", "RA_8"), ("LA", "LA_8_wrist")):
        prefixes = (f"{side}_Finger", f"{side}_THUMB", f"{side}_Thumb")
        hand = {n for n in movable if n.startswith(prefixes)}
        assert len(hand) == 16, side
        assert hand.isdisjoint(declared)  # a GripperSpec needs bounds these joints lack
        assert wrist in declared  # but the wrist carrying them is an executed channel

    # 6. The four drive wheels are unbounded because `continuous` is complete for them.
    assert SENTIENT_RWH_BASE.channel_names == ("RL_tyre", "FL_tyre", "FR_tyre", "RR_tyre")
    assert all(joints[name].get("type") == "continuous" for name in SENTIENT_RWH_BASE.channel_names)


def test_every_episode_ready_layout_is_declared_by_its_authoritative_urdf() -> None:
    for embodiment in embodiments.values():
        urdf = embodiment.urdf
        movable = {
            joint.get("name")
            for joint in ET.parse(resolve_asset(urdf.asset)).getroot().iter("joint")
            if joint.get("type") not in (None, "fixed")
        }
        for coordinate in embodiment.state.coordinates:
            side = str(coordinate.instance).split("_", 1)[0]
            assert coordinate.joint_name in movable or (
                f"{side}_{coordinate.joint_name}" in movable
            ), f"{embodiment.name}: {coordinate.instance}/{coordinate.joint_name}"
        assert all(asset.provenance is not None for asset in embodiment.assets)


def test_yubi_hands_urdf_matches_upstream_hand_model() -> None:
    """The composed description preserves both complete CAD mechanisms and frames."""
    from sx_embodiments.assets import asset_root
    from sx_embodiments.known.yubi import YUBI_HANDS_URDF, YUBI_JAW

    root = ET.parse(YUBI_HANDS_URDF.path()).getroot()
    links = {link.get("name") for link in root.iter("link")}
    assert {
        "quest_origin",
        "quest_left_controller",
        "quest_right_controller",
        "left_controller_link",
        "right_controller_link",
        "left_hand_cam_optical",
        "right_hand_cam_optical",
    } <= links

    revolute = {
        joint.get("name"): joint for joint in root.iter("joint") if joint.get("type") == "revolute"
    }
    for hand in ("left", "right"):
        expected = {
            f"{hand}_left_finger",
            *(f"{hand}_{mimic.joint_name}" for mimic in YUBI_JAW.mimic_joints),
        }
        assert expected <= set(revolute)
        driven = revolute[f"{hand}_left_finger"]
        limit = driven.find("limit")
        assert limit is not None and (limit.get("lower"), limit.get("upper")) == (
            "0",
            "0.785398",
        )
        assert driven.find("mimic") is None
        for mimic in YUBI_JAW.mimic_joints:
            declared = revolute[f"{hand}_{mimic.joint_name}"].find("mimic")
            assert declared is not None
            assert declared.get("joint") == f"{hand}_{mimic.of}"
            assert float(declared.get("multiplier", "1")) == mimic.multiplier

        fixed = {joint.get("name"): joint for joint in root.iter("joint")}
        camera = fixed[f"{hand}_camera_link_frame"]
        assert camera.find("parent").get("link") == f"{hand}_base_link"  # type: ignore[union-attr]
        assert camera.find("child").get("link") == f"{hand}_hand_cam_optical"  # type: ignore[union-attr]
        tracking = fixed[f"{hand}_controller_tracking_frame"]
        assert tracking.find("parent").get("link") == f"{hand}_controller_link"  # type: ignore[union-attr]
        assert tracking.find("child").get("link") == f"quest_{hand}_controller"  # type: ignore[union-attr]
        tracking_origin = tracking.find("origin")
        assert tracking_origin is not None
        assert [float(value) for value in tracking_origin.get("rpy", "").split()] == [
            0.0,
            0.0,
            -1.5707963267948966,
        ]

    parents = {
        child.get("link"): parent.get("link")
        for joint in root.iter("joint")
        if (parent := joint.find("parent")) is not None
        and (child := joint.find("child")) is not None
    }
    assert set(links) - set(parents) == {"quest_origin"}
    reached = {"quest_origin"}
    while discovered := {child for child, parent in parents.items() if parent in reached} - reached:
        reached.update(discovered)
    assert reached == links

    package_root = asset_root()
    meshes = {
        mesh.get("filename") for mesh in root.iter("mesh") if mesh.get("filename") is not None
    }
    assert meshes, "yubi hands URDF references no meshes"
    assert "package://sx-embodiments/quest_ego/meshes/quest3mesh.obj" not in meshes
    for filename in meshes:
        assert filename is not None and filename.startswith("package://")
        package, relative = filename.removeprefix("package://").split("/", 1)
        if package != "sx-embodiments":
            relative = f"{package}/{relative}"
        assert (package_root / relative).is_file(), f"unresolved mesh {filename}"


def test_yubi_composed_urdf_is_regenerated_from_the_vendored_sources() -> None:
    import runpy

    from sx_embodiments.known.yubi import YUBI_HANDS_URDF

    tool = Path(__file__).parents[1] / "tools/compose_yubi_urdf.py"
    render = runpy.run_path(str(tool))["render"]
    assert render() == YUBI_HANDS_URDF.path().read_bytes()


def test_yubi_camera_housings_are_not_promoted_as_calibrated_optics() -> None:
    from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS

    entry = DEVELOPMENT_EMBODIMENTS["yubi"]
    assert entry.reason.value == "missing_camera_calibration"
    assert not any(component.role.value == "sensor" for component in entry.spec.attachments)
