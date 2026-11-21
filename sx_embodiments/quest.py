"""OpenQuestCapture builds: their content identities, and which one new capture is pinned to.

What a build's pose streams were recorded in is not here: a recording declares it
(``recorded_sources.json``), and for recordings made before declarations the one stored-data
derivation (``sx_delivery.recorded_sources``) states each build's sources.
"""

from __future__ import annotations

import re
from collections.abc import Mapping
from dataclasses import dataclass
from enum import StrEnum
from types import MappingProxyType

_SHA256_BUILD_ID = re.compile(r"sha256:[0-9a-f]{64}")


class QuestApkBuildIdError(Exception):
    """An APK content identity is not a canonical SHA-256 value."""

    def __init__(self, value: str, reason: str) -> None:
        self.value = value
        self.reason = reason
        super().__init__(f"invalid Quest APK build id {value!r}: {reason}")


@dataclass(frozen=True, slots=True, order=True)
class QuestApkBuildId:
    """Content identity of one exact installed Quest APK set."""

    value: str

    def __post_init__(self) -> None:
        if _SHA256_BUILD_ID.fullmatch(self.value) is None:
            raise QuestApkBuildIdError(
                self.value,
                "expected sha256 followed by 64 lowercase hexadecimal digits",
            )

    @classmethod
    def from_digest(cls, digest: str) -> QuestApkBuildId:
        return cls(f"sha256:{digest}")

    def __str__(self) -> str:
        return self.value


class QuestCaptureAppError(Exception):
    """Installed capture-app facts are incomplete or contradictory."""

    def __init__(self, field: str, reason: str) -> None:
        self.field = field
        self.reason = reason
        super().__init__(f"Quest capture app {field} {reason}")


@dataclass(frozen=True, slots=True)
class QuestCaptureApp:
    """Exact app bytes plus the independent human-readable runtime facts."""

    apk_build_id: QuestApkBuildId
    app_version: str
    os_build: str

    def __post_init__(self) -> None:
        for field, value in (
            ("app_version", self.app_version),
            ("os_build", self.os_build),
        ):
            if not value.strip():
                raise QuestCaptureAppError(field, "must be non-empty")


class QuestCaptureContract(StrEnum):
    """The OpenQuestCapture 1.4 metadata/sidecar contract a session was recorded under.

    ``1.4.0``: OpenQuestCapture 1.4.0-1.4.2. ``1.4.1``: 1.4.3, which may write ``-1``
    for an exposure duration or capture frame number the HAL omitted and reports
    ``capture_error``. ``1.4.2``: 1.4.4, which requests one AE/fps range for both
    eyes, selects encoded frames on an absolute sensor-clock grid, and reports that
    selection in ``capture_report`` — so both eyes deliver the same exposure slots
    and conversion gates on their pairing. ``1.4.3``: 1.4.5+, which asks Camera2 for
    the native listed pixel-array stream, resamples it isotropically into the
    encoder, and states ``requested_stream_size``, ``source_stream_size``,
    ``output_size`` and the SurfaceTexture ``texture_transform`` in the metadata —
    so the MP4's intrinsics are derived from recorded facts. Contracts before it
    asked for 720x720, a size the device does not list, and are converted under
    the pinned crop rule (:mod:`sxd.umico.converters.quest_camera_geometry`).
    """

    V1_4_0 = "1.4.0"
    V1_4_1 = "1.4.1"
    V1_4_2 = "1.4.2"
    V1_4_3 = "1.4.3"

    @property
    def stereo_locked(self) -> bool:
        """Both eyes share one sensor-clock frame grid; their pairing is a gate."""
        return self in (QuestCaptureContract.V1_4_2, QuestCaptureContract.V1_4_3)

    @property
    def records_stream_geometry(self) -> bool:
        """The metadata states the requested/source/output stream sizes."""
        return self is QuestCaptureContract.V1_4_3


class QuestBuildError(Exception):
    """An installed capture build is not the one new capture is pinned to."""


class QuestBuildNotConfiguredError(QuestBuildError):
    def __init__(self) -> None:
        super().__init__("no active real Quest APK is registered for managed capture")


class InactiveQuestApkBuildError(QuestBuildError):
    def __init__(self, installed: QuestApkBuildId, active: QuestApkBuildId) -> None:
        self.installed = installed
        self.active = active
        super().__init__(
            f"Quest APK {installed} is registered but inactive; active build is {active}"
        )


# The simulator has no APK bytes; this reserved stable identity keeps its
# convention explicit without being admissible as a real device build.
SIMULATED_QUEST_APK_BUILD_ID = QuestApkBuildId(
    "sha256:91b59f5c4c4a1631328457a5bb7fa5353c1b6f9f20041e2636c6b17a1a95d9fe"
)
SIMULATED_QUEST_CAPTURE_APP = QuestCaptureApp(
    apk_build_id=SIMULATED_QUEST_APK_BUILD_ID,
    app_version="simulated",
    os_build="simulated",
)


# Real APK identities are deliberately source-controlled and append-only. A digest
# is measured from the exact signed release APK bytes; the managed-capture
# preflight independently hashes the installed base.apk and requires that same
# identity. The simulator identity never appears here.
# Verified 2026-08-06 from the locally pulled APK set on a Quest 3S running
# capture app 1.2.0; the preflight recording retained all mono-time streams.
QUEST_APK_BUILD_1_2_0 = QuestApkBuildId(
    "sha256:c5fa65dc67e74bfeee30a63b8e665bfcd66101a434a295b9f7941e0e843e8b3e"
)
QUEST_APK_BUILD_1_2_0_STEREO = QuestApkBuildId(
    "sha256:17836ff99cb34e2df42201d58114995d509f87b4703971722f1567c68d6a8d93"
)
# Built 2026-08-14 from OpenQuestCapture 1.3.0 after the exact-30-Hz,
# per-exposure-timestamp, tracking-origin, and stereo-evidence contract landed.
# Those changes do not touch the HMD/controller/body pose producers.
QUEST_APK_BUILD_1_3_0 = QuestApkBuildId(
    "sha256:18878d2e44b225ae81bcccd8db53875ec93b490d0143417d3a46d7ab7f0e2f2d"
)
# Built 2026-08-15 from OpenQuestCapture 1.4.0 after fresh-exposure selection,
# complete Camera2 result joining, stereo two-phase stop, and the production
# 720 px dual-camera encoder profile landed. Those changes leave the
# HMD/controller/body pose producers unchanged.
QUEST_APK_BUILD_1_4_0 = QuestApkBuildId(
    "sha256:d95f67f9e45289135cb11db3efc0062e2a745d3705650ab581a49531e969b1c7"
)
# Rebuilt 2026-08-15 with recording-lifecycle ownership fixed: recycling a
# Camera2 session no longer disposes its active encoder behind RecordingManager,
# and any real provider teardown finalizes per-eye metadata.
# Pose-producing code remains byte-for-byte unchanged from the registered 1.4 build.
QUEST_APK_BUILD_1_4_0_LIFECYCLE = QuestApkBuildId(
    "sha256:d75021362320b2a5bb27d6d2158cef99bcbb20ee73dcee3a95ecadbd124b9999"
)
# Built 2026-08-15 from OpenQuestCapture 1.4.1. MediaCodec's configured
# one-second interval is now the sole IDR scheduler; removing the competing
# manual sync-frame requests prevents intermittent per-eye GOP aborts.
# Diagnostic logging changed, but the pose producers remain unchanged.
QUEST_APK_BUILD_1_4_1 = QuestApkBuildId(
    "sha256:e3b1e72f780581a5efbc7dd351663663f0e73760f8996a9a1d5c9663f9e5ed21"
)
# Built 2026-08-15 from OpenQuestCapture 1.4.2. The shared Quest timestamp
# source now calls bionic clock_gettime(CLOCK_MONOTONIC) directly instead of
# Unity/IL2CPP Stopwatch, eliminating the measured multi-thousand-ppm rate
# error. Pose coordinate producers and conventions are unchanged.
QUEST_APK_BUILD_1_4_2 = QuestApkBuildId(
    "sha256:0a0b497b1d04d9c609d0fb312e5a770ed1a9fe506b47aa48460e8dc2d01005d4"
)
# Built 2026-08-16 from OpenQuestCapture 1.4.3 (capture contract 1.4.1). The
# recorder no longer stops an eye for sidecar bookkeeping: a Camera2 result
# without SENSOR_EXPOSURE_TIME (which the Quest HAL omits sporadically) or a
# missing result is written as -1 and counted in capture_report; capture
# results are retained by age, not count, and never cleared at start. Pose
# producers and conventions are unchanged. Rebuilt from the same committed tree
# (OpenQuestCapture 0081c2e / QuestCameraLib 059a9b8) after a discarded probe
# build overwrote the first artifact; Unity builds are not byte-reproducible.
QUEST_APK_BUILD_1_4_3 = QuestApkBuildId(
    "sha256:7a8098c48d30b1bff6fb94a36b85d4c43ad87961601542450e9986dadd0c4b31"
)
# Built 2026-08-16 from OpenQuestCapture 1.4.4 (5237f14, QuestCameraLib da6d740;
# capture contract 1.4.2). The camera sessions request AE range [60,60], on
# which the Quest 3S HAL delivers one shared 50 Hz lattice with left/right
# SENSOR_TIMESTAMPs identical to the nanosecond, and the recorder selects the
# first frame of each absolute 33.33 ms bin so both eyes deliver the same
# instants (20/40/40 ms, mean 30 Hz). Pose producers and conventions are
# unchanged.
QUEST_APK_BUILD_1_4_4 = QuestApkBuildId(
    "sha256:f9ce2b720d861a3c6dac9e761fbbfff82d4e8f6009860d7314dff498edbe7bfb"
)
# Built 2026-08-18 from OpenQuestCapture 1.4.5 (capture contract 1.4.3). The
# recorder asks Camera2 for the native listed 1280x1280 stream instead of an
# unlisted 720x720 the camera service rounded to 720x576 (a 5:4 centre crop
# stretched square), refuses to start when the request is not among the
# camera's listed output sizes, resamples the whole array isotropically into
# the 720x720 encoder, and states requested/source/output sizes and the
# SurfaceTexture transform in the metadata sidecar. Pose producers and
# conventions are unchanged (docs/plans/partial/quest-camera-geometry-2026-08.md).
QUEST_APK_BUILD_1_4_5 = QuestApkBuildId(
    "sha256:ff6e924b7f45955b86c74a50e0a0e2c0726f45070bdb82a95084444caa76e7d9"
)
# Released 2026-10-08 as OpenQuestCapture 1.4.6 (version code 13, capture contract
# 1.4.3, OpenQuestCapture#9): refuses to start with frozen head tracking, reports stop,
# pause and controller state to the pod, flushes CSVs every second, writes
# stream_manifest.json, and starts body tracking at launch. body_tracking.csv gains four
# columns after the joints. Pose producers and conventions are unchanged. Active since the
# pod's APK pin moved to it (sx#1307).
QUEST_APK_BUILD_1_4_6 = QuestApkBuildId(
    "sha256:71ee51b9367b5152e830348c2f859c0e481f5b59c66f522cfaa544594fd9b222"
)
# Headset-pod builds: OpenQuestCapture with the wrist rig in the one app (quest-only pod,
# sx docs/plans/quest-only-pod-2026-10.md), each recorded on the headset's one clock. They are
# registered so a headset seal resolves them (a headset seal needs a registered build, not the
# active one); none is the active laptop-pod build. Pose producers are those of 1.4.6, so each
# takes its convention. Adding a build is one line here, then re-rendering sx-seal's contract
# (`uv run python packages/sx-seal/tools/render_contract.py`).
HEADSET_QUEST_APK_BUILDS: Mapping[str, QuestApkBuildId] = MappingProxyType(
    {
        # 2026-10-17, OpenQuestCapture 1.5.0 build 20 with the test libsx_rig e5c36ce5 swapped
        # in and debug-re-signed (capture contract 1.4.3); recorded the yubi-06 rig's
        # 20261017_132259 and 20261017_132435.
        "1.5.0-20-rig-e5c36ce5": QuestApkBuildId(
            "sha256:975ca9a810bb3c8b1038a605ad8881e563afcdc5e72292a3f8a85cba00ce9430"
        ),
        # 2026-10-17, OpenQuestCapture 1.5.0 build 23 (quest-pod/rig 5e6f61b, capture contract
        # 1.4.3); recorded 20261017_153607, 20261017_153640 and 20261017_153708.
        "1.5.0-23": QuestApkBuildId(
            "sha256:7e81d67a7a5491201d84b65d61e3ec1825f7883280b0d92778bac72d52aabcac"
        ),
        # 2026-10-17, OpenQuestCapture 1.5.0 build 24 (quest-pod/rig d85e322), installed on
        # the bench Quest 3S; no episode recorded with it yet.
        "1.5.0-24": QuestApkBuildId(
            "sha256:d5b51230bc6d502ce1f0361e0bb27807681b67a9e9e9e083221faa37e1bbac44"
        ),
        # 2026-10-17, OpenQuestCapture 1.5.0 build 26 (OQC 5d34fcd, QuestCameraLib 51a4868),
        # the final headset build.
        "1.5.0-26": QuestApkBuildId(
            "sha256:6aab166a88c48ea68f525b3598e762fe18ae2c2519aa0dd11af606a6d9d5787a"
        ),
        # 2026-10-19, OpenQuestCapture 1.5.0 build 28 (OQC 3240834, QuestCameraLib 51a4868,
        # libsx_rig.so sha256 b88d099c from sx quest-rig 957b728a0e): pod_config.json written
        # by rename, the flat enrollment request, and the rig type a laptop pod takes.
        "1.5.0-28": QuestApkBuildId(
            "sha256:2b5b21f03cefe7547588876375c3a32d2f9e1748114f2fe1d7f27c33b367f1a5"
        ),
        # 2026-10-19, OpenQuestCapture 1.5.0 build 31 (OQC 46de4ca, QuestCameraLib 51a4868,
        # libsx_rig.so sha256 e7b6eef1 from sx quest-rig ac29588cb6): Pod setup knows an
        # enrolled headset, every check on a Factory resolves the rig from it (bounded 5 s).
        "1.5.0-31": QuestApkBuildId(
            "sha256:9d523e8e4c6d09b7adb345a899fe5edbe5f55799033e5a9db2bad06ed4cb5a32"
        ),
        # 2026-10-19, OpenQuestCapture 1.5.0 build 32 (OQC f7747bf, QuestCameraLib 51a4868,
        # libsx_rig.so e7b6eef1 as build 31): a collector signs in from Pod setup, and the
        # left menu button has one handler.
        "1.5.0-32": QuestApkBuildId(
            "sha256:3fca797ab8b1edc4e17338ab23b672e84298bfbeb32b0c1798b864d7dc701b61"
        ),
        # 2026-10-19, OpenQuestCapture 1.5.0 build 33 (OQC 69bc756, QuestCameraLib 51a4868,
        # libsx_rig.so e7b6eef1 as build 31): a Pod status block in the recordings menu.
        "1.5.0-33": QuestApkBuildId(
            "sha256:40992ce4dc0f8626c90f24d38c4da8c7935d4089bf9e6e9bd427a2f866e3b093"
        ),
        # 2026-10-19, OpenQuestCapture 1.5.0 build 35 (OQC f7c07ee, QuestCameraLib 51a4868,
        # libsx_rig.so e7b6eef1 as build 31): the recordings menu's Pod card in the app's style.
        "1.5.0-35": QuestApkBuildId(
            "sha256:20c3c9ccd6c630b28fea440d6dda320c488ae624020101407ccc7994f5f8cc74"
        ),
        # Builds 36-67 went to pods as HIL trials, unregistered (sx#4210). The entries below
        # are the builds pod5's staged episodes carry as episode.json `quest_apk_build_id`
        # (read on the headset 2026-11-15: 228 seals pending "not on the accepted build
        # list"), plus 1.5.0-72, the build now installed on pod5.
        #
        # 2026-10-22, OpenQuestCapture 1.5.0 build 58 (OQC cfb4546, QuestCameraLib 51a4868,
        # libsx_rig.so c6678356), the archived release: pod4 and pod5 recorded on it; on pod5
        # 196 episodes, 20261102_180514 to 20261105_195648.
        "1.5.0-58": QuestApkBuildId(
            "sha256:799ad8f2baa39e4d07335f6dc4499f404aa809931e5cc1d1841d95809de25783"
        ),
        # 2026-11-05, build 58 with only libsx_rig.so rebuilt (sx 9c60a11b6a, packaged
        # 1ffc1159): pod5's storage/backlog HIL trial, bounded per-camera and per-encoder
        # writer queues. Recorded pod5 20261105_212529 to 20261105_215449 (4).
        "1.5.0-58-storage-2a761d61": QuestApkBuildId(
            "sha256:2a761d6117c310aaffd24adbf02a969d3cd7492137657ad80141329b850a9178"
        ),
        # 2026-11-05, build 58 with only libsx_rig.so rebuilt (sx 9c60a11b6a): pod5's
        # temporary fixed-exposure diagnostic probe. Recorded pod5 20261105_221448 (1).
        "1.5.0-58-exposure-probe-c07ce691": QuestApkBuildId(
            "sha256:c07ce69120c8664ce703ad1087bf831ac9cceee589ce0d1d69fce00c7a47ab45"
        ),
        # 2026-11-05, build 58 with only libsx_rig.so rebuilt (sx 9c60a11b6a): pod5's
        # automatic brightness and backlog correction HIL trial. Recorded pod5
        # 20261105_223222 to 20261106_005423 (6).
        "1.5.0-58-auto-brightness-85af646c": QuestApkBuildId(
            "sha256:85af646c5c2810a379dd5e6ff4a0da24f6cbe15f0a6e589ead35ef364fcd7c02"
        ),
        # 2026-10-29, OpenQuestCapture 1.5.0 build 67 (OQC 5b31ccf, QuestCameraLib f66ab1e,
        # libsx_rig.so 967144db), the archived release. Recorded pod5 20261106_012936 to
        # 20261106_125125 (4).
        "1.5.0-67": QuestApkBuildId(
            "sha256:0b0694f11a5949bd0f1e5f887800741c673ed698736871055fa35890e626f4b4"
        ),
        # 2026-11-06, build 67 with only libsx_rig.so rebuilt (sx 9c0b8959dd, embodiments
        # b15c4b5, packaged bd88fc8d): pod5's local exposure patch, metering frames older than
        # the last control readback are rejected. Recorded pod5 20261106_182352 to
        # 20261109_132228 (83).
        "1.5.0-67-exposure-313d65ab": QuestApkBuildId(
            "sha256:313d65abf450320b6ed1ca23634d8d3e7126713796ad66f07c889f58fc681305"
        ),
        # 2026-11-10, build 67 with libsx_rig.so d84cf8fe (the 11-09 quality candidate's) and
        # the C# chain rebuilt from OQC 5b31ccf plus a patch: pod5's USB-repair HIL trial.
        # Recorded pod5 20261110_171816 to 20261112_174440 (88).
        "1.5.0-67-usb-repair-ea6de61f": QuestApkBuildId(
            "sha256:ea6de61f6b8e099e4c565e79955670d1db562528c93c9aac8d28ac9f9bfc3fcf"
        ),
        # 2026-11-12, the USB-repair build with libsx_rig.so 58293113 and the C# chain rebuilt:
        # pod5's seal/status fix (status parsed off Unity's thread, fresh tracking at start,
        # pending-episode review UI). Recorded pod5 20261112_181140 to 20261113_145058 (5).
        "1.5.0-67-sealing-0e57c237": QuestApkBuildId(
            "sha256:0e57c2379711f3bad5b75f5f4fc2c40dde92c654a3aabf4a2e762bf196c922b3"
        ),
        # 2026-11-15, OpenQuestCapture 1.5.0 build 72 (OQC 59e8fd6, QuestCameraLib 93d2e74,
        # libsx_rig.so f238959a, embodiment assets from sx 310d653e70): the sx#4210 integrity
        # release (shared tracking gate, controller tracked, seal I/O yield), installed on pod5.
        "1.5.0-72": QuestApkBuildId(
            "sha256:7c0ae2c96bc1dc68bd20effd96316b5da59f1e376505304ea22ff44470eea095"
        ),
        # 2026-11-15, OpenQuestCapture 1.5.0 build 73 (OQC 3c6c3e8, QuestCameraLib 93d2e74,
        # libsx_rig.so 16884ba0, embodiment assets from sx 9a455d177c): 72 with the collector's
        # auto-review restored in libsx_rig; replaces 72 on pod5.
        "1.5.0-73": QuestApkBuildId(
            "sha256:3e704e58bfb4a554302ad364e9de2b70b212003ae15c1de819ae8fe54c94b1ad"
        ),
        # 2026-11-15, OpenQuestCapture 1.5.0 build 74 (OQC 0f7c5d0, QuestCameraLib 3e0566c,
        # libsx_rig.so e2f431d8, embodiment assets from sx eaa315c810): replaces 73 on pod5.
        "1.5.0-74": QuestApkBuildId(
            "sha256:854a8aaf5134d83aab4f1dde384274f68d0d3d20683603d60f50041de9500059"
        ),
        # 2026-11-16, OpenQuestCapture 1.5.0 build 75 (OQC 36ca07c, QuestCameraLib a6e6bd3,
        # libsx_rig.so 90b3d29b from sx def516aa36, embodiment assets from sx 07fed1dc87): the
        # right gripper's start/stop hold, mid-episode camera failure stop; replaces 74 on pod5.
        "1.5.0-75": QuestApkBuildId(
            "sha256:adf3575470a2056d695a33c9e2f871b8aa8dd2fdf170e087dc51b60fb32a6366"
        ),
        # 2026-11-16, OpenQuestCapture 1.5.0 build 76 (OQC f560511, QuestCameraLib 0964035,
        # libsx_rig.so 90b3d29b from sx def516aa36, embodiment assets from sx 07fed1dc87): the stop
        # finds stopRecording again (75's AAR compiled it to ()Lkotlin/Unit; and both eyes failed
        # at the first stop), JNI bound on the main thread; passed the on-device gate on pod5
        # (both eyes 436 frames, finished MP4s); replaces 75 on pod5.
        "1.5.0-76": QuestApkBuildId(
            "sha256:47038bdf0efd2039a1411c89c264bbee5119ee8455a39215d187d52e41e3cc26"
        ),
        # 2026-11-21, OpenQuestCapture 1.5.0 build 77 (OQC c548eb1, QuestCameraLib 0964035,
        # libsx_rig.so e02c61b9 from sx bbe8641a30, embodiment assets from sx bbe8641a30): every
        # seal stage streams (a 139 s take 660 MB -> 22 MB peak) and a seal that dies three times
        # the same way under one build is held (seal_held) instead of rerun at every launch.
        "1.5.0-77": QuestApkBuildId(
            "sha256:a107cacc10f4174a6d2792a77a0fccf1323b43ca984127787e768aef97a63eb5"
        ),
        # 2026-11-21, OpenQuestCapture 1.5.0 build 78 (OQC 09eef1f, QuestCameraLib 0964035,
        # libsx_rig.so 57bb9006 from sx cc1eac8a5d, embodiment assets from sx cc1eac8a5d): a take
        # stops itself at 15 minutes (warning cue and notice at 14) and is sealed as usual.
        "1.5.0-78": QuestApkBuildId(
            "sha256:336f2b5498664bb4d23044f3187726afe820125333060817077d964e068baaad"
        ),
    }
)
#: Every real build that has recorded, laptop pod and headset alike (append-only).
REAL_QUEST_APK_BUILDS: tuple[QuestApkBuildId, ...] = (
    QUEST_APK_BUILD_1_2_0,
    QUEST_APK_BUILD_1_2_0_STEREO,
    QUEST_APK_BUILD_1_3_0,
    QUEST_APK_BUILD_1_4_0,
    QUEST_APK_BUILD_1_4_0_LIFECYCLE,
    QUEST_APK_BUILD_1_4_1,
    QUEST_APK_BUILD_1_4_2,
    QUEST_APK_BUILD_1_4_3,
    QUEST_APK_BUILD_1_4_4,
    QUEST_APK_BUILD_1_4_5,
    QUEST_APK_BUILD_1_4_6,
    *HEADSET_QUEST_APK_BUILDS.values(),
)

# New capture is additionally pinned to one registered build. Historical
# registry entries remain resolvable after this active identity advances.
ACTIVE_REAL_QUEST_APK_BUILD_ID: QuestApkBuildId | None = QUEST_APK_BUILD_1_4_6


def require_active_real_quest_build(
    capture_app: QuestCaptureApp,
    *,
    active_build_id: QuestApkBuildId | None = ACTIVE_REAL_QUEST_APK_BUILD_ID,
) -> None:
    """Gate new capture separately from historical convention resolution."""
    if active_build_id is None:
        raise QuestBuildNotConfiguredError()
    if capture_app.apk_build_id != active_build_id:
        raise InactiveQuestApkBuildError(capture_app.apk_build_id, active_build_id)


__all__ = [
    "ACTIVE_REAL_QUEST_APK_BUILD_ID",
    "HEADSET_QUEST_APK_BUILDS",
    "QUEST_APK_BUILD_1_2_0",
    "QUEST_APK_BUILD_1_2_0_STEREO",
    "QUEST_APK_BUILD_1_3_0",
    "QUEST_APK_BUILD_1_4_0",
    "QUEST_APK_BUILD_1_4_0_LIFECYCLE",
    "QUEST_APK_BUILD_1_4_1",
    "QUEST_APK_BUILD_1_4_2",
    "QUEST_APK_BUILD_1_4_3",
    "QUEST_APK_BUILD_1_4_4",
    "QUEST_APK_BUILD_1_4_5",
    "QUEST_APK_BUILD_1_4_6",
    "REAL_QUEST_APK_BUILDS",
    "SIMULATED_QUEST_APK_BUILD_ID",
    "SIMULATED_QUEST_CAPTURE_APP",
    "InactiveQuestApkBuildError",
    "QuestApkBuildId",
    "QuestApkBuildIdError",
    "QuestBuildError",
    "QuestBuildNotConfiguredError",
    "QuestCaptureApp",
    "QuestCaptureAppError",
    "QuestCaptureContract",
    "require_active_real_quest_build",
]
