"""Real Quest capture-app builds resolve to their pose conventions; one captures."""

import pytest

from sx_embodiments.quest import (
    ACTIVE_REAL_QUEST_APK_BUILD_ID,
    HEADSET_QUEST_APK_BUILDS,
    QUEST_APK_BUILD_1_4_5,
    QUEST_APK_BUILD_1_4_6,
    REAL_QUEST_POSE_CONVENTION_FACTS,
    InactiveQuestApkBuildError,
    QuestApkBuildId,
    QuestCaptureApp,
    require_active_real_quest_build,
    resolve_real_quest_pose_convention,
)


def test_1_4_7_captures_and_older_recordings_still_resolve() -> None:
    """The body-status APK captures; old 1.4.5/1.4.6 recordings retain their conventions."""
    build = QuestApkBuildId(
        "sha256:c1964dfcc4f8634b80d4bfd0b4b64e1ac79bbd2713a50cd74e338ce7bc6f9c4b"
    )
    assert build == ACTIVE_REAL_QUEST_APK_BUILD_ID
    app = QuestCaptureApp(build, "1.4.7", "quest/test/os")
    assert resolve_real_quest_pose_convention(app).capture_app == app
    require_active_real_quest_build(app)
    for old_build, version in [(QUEST_APK_BUILD_1_4_5, "1.4.5"), (QUEST_APK_BUILD_1_4_6, "1.4.6")]:
        assert (
            REAL_QUEST_POSE_CONVENTION_FACTS[build] == REAL_QUEST_POSE_CONVENTION_FACTS[old_build]
        )
        old = QuestCaptureApp(old_build, version, "quest/test/os")
        assert resolve_real_quest_pose_convention(old).capture_app == old
        with pytest.raises(InactiveQuestApkBuildError):
            require_active_real_quest_build(old)


def test_headset_builds_resolve_with_1_4_6_conventions_and_none_is_active() -> None:
    """A headset-pod build is registered so a headset seal resolves it; the laptop pod's
    active build does not move with it."""
    assert HEADSET_QUEST_APK_BUILDS
    for version, build in HEADSET_QUEST_APK_BUILDS.items():
        assert build != ACTIVE_REAL_QUEST_APK_BUILD_ID
        assert (
            REAL_QUEST_POSE_CONVENTION_FACTS[build]
            == REAL_QUEST_POSE_CONVENTION_FACTS[QUEST_APK_BUILD_1_4_6]
        )
        app = QuestCaptureApp(build, version, "quest/test/os")
        assert resolve_real_quest_pose_convention(app).capture_app == app
        with pytest.raises(InactiveQuestApkBuildError):
            require_active_real_quest_build(app)
