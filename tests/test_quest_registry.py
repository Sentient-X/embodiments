"""Real Quest capture-app builds are registered once; exactly one captures."""

import pytest

from sx_embodiments.quest import (
    ACTIVE_REAL_QUEST_APK_BUILD_ID,
    HEADSET_QUEST_APK_BUILDS,
    QUEST_APK_BUILD_1_4_5,
    QUEST_APK_BUILD_1_4_6,
    REAL_QUEST_APK_BUILDS,
    InactiveQuestApkBuildError,
    QuestCaptureApp,
    require_active_real_quest_build,
)


def test_1_4_6_captures_and_1_4_5_recordings_stay_registered() -> None:
    """OpenQuestCapture 1.4.6 is the build new capture is pinned to, with the pod's APK pin
    (sx#1307); 1.4.5's recordings stay registered for replay."""
    assert ACTIVE_REAL_QUEST_APK_BUILD_ID == QUEST_APK_BUILD_1_4_6
    assert {QUEST_APK_BUILD_1_4_5, QUEST_APK_BUILD_1_4_6} <= set(REAL_QUEST_APK_BUILDS)
    require_active_real_quest_build(QuestCaptureApp(QUEST_APK_BUILD_1_4_6, "1.4.6", "quest/os"))
    old = QuestCaptureApp(QUEST_APK_BUILD_1_4_5, "1.4.5", "quest/test/os")
    with pytest.raises(InactiveQuestApkBuildError):
        require_active_real_quest_build(old)


def test_headset_builds_are_registered_and_none_is_active() -> None:
    """A headset-pod build is registered so a headset seal admits it; the laptop pod's
    active build does not move with it."""
    assert HEADSET_QUEST_APK_BUILDS
    assert len(set(REAL_QUEST_APK_BUILDS)) == len(REAL_QUEST_APK_BUILDS)
    for version, build in HEADSET_QUEST_APK_BUILDS.items():
        assert build != ACTIVE_REAL_QUEST_APK_BUILD_ID
        assert build in REAL_QUEST_APK_BUILDS
        with pytest.raises(InactiveQuestApkBuildError):
            require_active_real_quest_build(QuestCaptureApp(build, version, "quest/test/os"))
