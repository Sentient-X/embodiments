"""A nominal full-body upgrade shares corrected parts without inventing lower-body facts."""

import xml.etree.ElementTree as ET

import pytest

from sx_embodiments import (
    EmbodimentId,
    EmbodimentName,
    development_embodiments,
    embodiments,
    historical_document,
    materialize,
    read_recorded,
)
from sx_embodiments.assembly import Subtree
from sx_embodiments.assets import AssetAudience, AssetFormat, AssetRole, audience
from sx_embodiments.kinematics import Kinematics
from sx_embodiments.known.humanoid import (
    FULL_BODY_CORE,
    HEAD_MODULE,
    LEG_MODULES,
    SENTIENT_HUMANOID_BUILD,
    SENTIENT_HUMANOID_TORSO,
    SENTIENT_HUMANOID_URDF,
    UPPER_BODY_MODULE,
)
from sx_embodiments.known.static_upper_body import (
    STATIC_UPPER_BODY_LEFT_ARM_MODULE,
    STATIC_UPPER_BODY_RIGHT_ARM_MODULE,
)
from sx_embodiments.layout import UndocumentedDrive
from sx_embodiments.mechanical import Build, Placed

OLD = EmbodimentId("d5f2aad80bcb7bb4ff1eabfdf401b7f4910a89f4d12a731a8c8e24c177090bc5")


def _joint_origin(source, name):
    joint = ET.fromstring(source.path().read_bytes()).find(f"joint[@name='{name}']")
    assert joint is not None
    origin = joint.find("origin")
    assert origin is not None
    return tuple(float(x) for x in origin.get("xyz", "0 0 0").split())


def test_full_body_shares_corrected_modules_and_keeps_unqualified_lower_body() -> None:
    body = development_embodiments["sentient-humanoid"]
    assert "sentient-humanoid" not in embodiments
    assert len(body.joint_names) == 33
    assert [mate.module for mate in SENTIENT_HUMANOID_BUILD.mates] == [
        STATIC_UPPER_BODY_LEFT_ARM_MODULE,
        STATIC_UPPER_BODY_RIGHT_ARM_MODULE,
        HEAD_MODULE,
        *LEG_MODULES,
    ]
    by_name = {c.instance: c for c in SENTIENT_HUMANOID_BUILD.components()}
    assert by_name["torso"].part is SENTIENT_HUMANOID_TORSO
    for part in (
        STATIC_UPPER_BODY_LEFT_ARM_MODULE,
        STATIC_UPPER_BODY_RIGHT_ARM_MODULE,
    ):
        for component in Build.of(part).components():
            assert by_name[component.instance].part is component.part
    for coordinate in body.state.coordinates:
        if coordinate.instance in {"left_leg", "right_leg"} or coordinate.joint_name.startswith(
            "waist_"
        ):
            assert isinstance(coordinate.axis.actuation, UndocumentedDrive)
    assert not any(a.asset.format is AssetFormat.MJCF for a in body.assets)
    assert body.urdf.asset.role is AssetRole.DESCRIPTION
    assert "BSD-3-Clause" in body.urdf.provenance.license_id
    assert "LicenseRef-Sentient-Proprietary" in body.urdf.provenance.license_id
    assert audience(body.urdf.provenance.license_id) is AssetAudience.ENTITLED
    assert materialize(body).is_dir()


def test_original_head_mount_and_motion_frames_are_preserved() -> None:
    source = Kinematics.of(SENTIENT_HUMANOID_URDF.path().read_bytes())
    full = Kinematics.of(SENTIENT_HUMANOID_BUILD.urdf())
    assert FULL_BODY_CORE.socket("neck") == UPPER_BODY_MODULE.socket("neck")
    assert not any("so101" in name for name in full.links | full.joints.keys())
    assert not any(
        "so101" in name for name in development_embodiments["sentient-humanoid"].joint_names
    )
    for yaw, pitch in ((0.0, 0.0), (0.4, -0.3), (-0.6, 0.2)):
        positions = {"neck_yaw_joint": yaw, "head_joint": pitch}
        for name in ("neck_link", "neck_yaw_link", "head_link", "camera_link"):
            assert full.pose("chest_link", name, positions) == pytest.approx(
                source.pose("chest_link", name, positions), abs=1e-12
            )
    for side in ("left", "right"):
        assert FULL_BODY_CORE.socket(f"{side}_shoulder").pose.xyz == _joint_origin(
            SENTIENT_HUMANOID_URDF, f"{side}_shoulder_pitch_joint"
        )


def test_old_pelvis_waist_and_legs_keep_their_source_fk_under_nonzero_waist() -> None:
    source = Kinematics.of(SENTIENT_HUMANOID_URDF.path().read_bytes())
    full = Kinematics.of(SENTIENT_HUMANOID_BUILD.urdf())
    kept = FULL_BODY_CORE.links() | frozenset().union(*(part.links() for part in LEG_MODULES))
    positions = dict.fromkeys(set(source.independent) | set(full.independent), 0.12)
    positions.update(waist_roll_joint=0.2, waist_pitch_joint=-0.3, torso_joint=0.4)
    expected = source.poses("pelvis", {k: v for k, v in positions.items() if k in source.joints})
    actual = full.poses("pelvis", {k: v for k, v in positions.items() if k in full.joints})
    for link in kept & expected.keys():
        assert actual[link] == pytest.approx(expected[link], abs=1e-12), link


@pytest.mark.parametrize(
    "socket,module",
    [
        ("left_shoulder", STATIC_UPPER_BODY_LEFT_ARM_MODULE),
        ("right_shoulder", STATIC_UPPER_BODY_RIGHT_ARM_MODULE),
        ("neck", HEAD_MODULE),
    ],
)
def test_corrected_parts_retain_their_fk_and_ride_the_waist(socket, module) -> None:
    source = module.description
    assert isinstance(source, Subtree) and source.mount_link is not None
    seat = FULL_BODY_CORE.socket(socket)
    local = Kinematics.of(
        Build(
            robot="part",
            root="chest_link",
            placed=(Placed("part", module, pose=seat.pose, namespaced=False),),
        ).urdf()
    )
    full = Kinematics.of(SENTIENT_HUMANOID_BUILD.urdf())
    positions = dict.fromkeys(full.independent, 0.1)
    for link, pose in local.poses(
        "chest_link", {k: v for k, v in positions.items() if k in local.joints}
    ).items():
        if link in full.links:
            assert full.pose("chest_link", link, positions) == pytest.approx(pose, abs=1e-12)
    tip = (
        "head_camera_color_optical"
        if socket == "neck"
        else socket.replace("shoulder", "gripper_tcp")
    )
    before = full.pose("pelvis", tip, positions)
    positions.update(waist_roll_joint=0.2, waist_pitch_joint=-0.3)
    after = full.pose("pelvis", tip, positions)
    assert after != pytest.approx(before)
    assert full.pose("chest_link", tip, positions) == pytest.approx(
        local.pose("chest_link", tip, {k: v for k, v in positions.items() if k in local.joints})
    )


def test_old_full_recording_still_resolves_its_25_coordinate_description() -> None:
    old = read_recorded(historical_document(EmbodimentName("sentient-humanoid"), OLD))
    assert old.recorded_id == old.embodiment.id == OLD
    assert len(old.embodiment.joint_names) == 25
    assert "left_wrist_yaw_joint" in old.embodiment.joint_names
    assert "left_wrist_roll_joint" not in old.embodiment.joint_names
    assert old.embodiment.urdf_bytes != development_embodiments["sentient-humanoid"].urdf_bytes
    assert materialize(old.embodiment).is_dir()
    assert str(OLD) not in development_embodiments
