"""Real Quest capture-app builds resolve to their pose conventions; one captures."""

import pytest

from sx_embodiments.quest import (
    ACTIVE_REAL_QUEST_APK_BUILD_ID,
    QUEST_APK_BUILD_1_4_5,
    QUEST_APK_BUILD_1_4_6,
    REAL_QUEST_POSE_CONVENTION_FACTS,
    InactiveQuestApkBuildError,
    QuestCaptureApp,
    require_active_real_quest_build,
    resolve_real_quest_pose_convention,
)


def test_1_4_6_resolves_with_unchanged_conventions_but_does_not_capture_until_pinned() -> None:
    """OpenQuestCapture 1.4.6 keeps 1.4.5's pose producers; new capture stays on the pod's
    pinned build until both move together (sx#1307)."""
    assert ACTIVE_REAL_QUEST_APK_BUILD_ID == QUEST_APK_BUILD_1_4_5
    assert (
        REAL_QUEST_POSE_CONVENTION_FACTS[QUEST_APK_BUILD_1_4_6]
        == (REAL_QUEST_POSE_CONVENTION_FACTS[QUEST_APK_BUILD_1_4_5])
    )
    app = QuestCaptureApp(QUEST_APK_BUILD_1_4_6, "1.4.6", "quest/test/os")
    assert resolve_real_quest_pose_convention(app).capture_app == app
    with pytest.raises(InactiveQuestApkBuildError):
        require_active_real_quest_build(app)
