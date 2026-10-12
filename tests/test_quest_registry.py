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


def test_1_4_6_captures_and_1_4_5_recordings_still_resolve() -> None:
    """OpenQuestCapture 1.4.6 keeps 1.4.5's pose producers and is the build new capture
    is pinned to, with the pod's APK pin (sx#1307); 1.4.5's recordings still resolve."""
    assert ACTIVE_REAL_QUEST_APK_BUILD_ID == QUEST_APK_BUILD_1_4_6
    assert (
        REAL_QUEST_POSE_CONVENTION_FACTS[QUEST_APK_BUILD_1_4_6]
        == (REAL_QUEST_POSE_CONVENTION_FACTS[QUEST_APK_BUILD_1_4_5])
    )
    app = QuestCaptureApp(QUEST_APK_BUILD_1_4_6, "1.4.6", "quest/test/os")
    assert resolve_real_quest_pose_convention(app).capture_app == app
    require_active_real_quest_build(app)
    old = QuestCaptureApp(QUEST_APK_BUILD_1_4_5, "1.4.5", "quest/test/os")
    assert resolve_real_quest_pose_convention(old).capture_app == old
    with pytest.raises(InactiveQuestApkBuildError):
        require_active_real_quest_build(old)
