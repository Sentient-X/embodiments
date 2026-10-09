"""Build-keyed hardware and raw pose facts for OpenQuestCapture."""

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


class QuestRawPoseSource(StrEnum):
    """Coordinate convention serialized by one raw Quest pose stream."""

    UNITY_LH = "unity_lh"
    OVR_NATIVE_RH = "ovr_native_rh"


@dataclass(frozen=True, slots=True)
class QuestPoseConvention:
    """The complete build-bound source convention used by the converter."""

    capture_app: QuestCaptureApp
    controller_hmd_source: QuestRawPoseSource
    body_source: QuestRawPoseSource

    @property
    def apk_build_id(self) -> QuestApkBuildId:
        return self.capture_app.apk_build_id

    @property
    def needs_unmirror(self) -> bool:
        """Existing controller/HMD control equivalent for this raw source."""
        match self.controller_hmd_source:
            case QuestRawPoseSource.UNITY_LH:
                return True
            case QuestRawPoseSource.OVR_NATIVE_RH:
                return False

    @property
    def body_flip(self) -> bool:
        """Existing body-writer control equivalent for this raw source."""
        match self.body_source:
            case QuestRawPoseSource.UNITY_LH:
                return False
            case QuestRawPoseSource.OVR_NATIVE_RH:
                return True


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


class QuestPoseConventionError(Exception):
    """An exact managed APK cannot resolve to a registered convention."""


class QuestPoseConventionNotConfiguredError(QuestPoseConventionError):
    def __init__(self) -> None:
        super().__init__("no active real Quest APK is registered for managed capture")


class UnknownQuestApkBuildError(QuestPoseConventionError):
    def __init__(
        self,
        build_id: QuestApkBuildId,
        registered_build_ids: tuple[QuestApkBuildId, ...],
    ) -> None:
        self.build_id = build_id
        self.registered_build_ids = registered_build_ids
        registered = ", ".join(str(item) for item in registered_build_ids)
        super().__init__(
            f"Quest APK {build_id} has no registered pose convention; "
            f"registered builds: {registered}"
        )


class InactiveQuestApkBuildError(QuestPoseConventionError):
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


def _simulated_app_convention(capture_app: QuestCaptureApp) -> QuestPoseConvention:
    return QuestPoseConvention(
        capture_app=capture_app,
        controller_hmd_source=QuestRawPoseSource.UNITY_LH,
        body_source=QuestRawPoseSource.OVR_NATIVE_RH,
    )


SIMULATED_QUEST_POSE_CONVENTION = _simulated_app_convention(SIMULATED_QUEST_CAPTURE_APP)


@dataclass(frozen=True, slots=True)
class QuestPoseConventionFacts:
    """Append-only calibrated facts for one exact real APK identity."""

    controller_hmd_source: QuestRawPoseSource
    body_source: QuestRawPoseSource


# Real APK facts are deliberately source-controlled and append-only. A digest
# is measured from the exact signed release APK bytes; the managed-capture
# preflight independently hashes the installed base.apk and requires that same
# identity. Pose facts must be calibrated, or carried forward only when the
# pose-producing code is unchanged. The simulator identity never appears here.
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
REAL_QUEST_POSE_CONVENTION_FACTS: Mapping[QuestApkBuildId, QuestPoseConventionFacts] = (
    MappingProxyType(
        {
            QUEST_APK_BUILD_1_2_0: QuestPoseConventionFacts(
                controller_hmd_source=QuestRawPoseSource.UNITY_LH,
                body_source=QuestRawPoseSource.OVR_NATIVE_RH,
            ),
            # The stereo-camera build changes capture output only; its controller,
            # HMD, and body pose sources retain the calibrated 1.2.0 convention.
            QUEST_APK_BUILD_1_2_0_STEREO: QuestPoseConventionFacts(
                controller_hmd_source=QuestRawPoseSource.UNITY_LH,
                body_source=QuestRawPoseSource.OVR_NATIVE_RH,
            ),
            # Capture contract 1.3 changes video cadence/timestamps and records
            # tracking-origin/stereo evidence; raw pose producers are unchanged.
            QUEST_APK_BUILD_1_3_0: QuestPoseConventionFacts(
                controller_hmd_source=QuestRawPoseSource.UNITY_LH,
                body_source=QuestRawPoseSource.OVR_NATIVE_RH,
            ),
            QUEST_APK_BUILD_1_4_0: QuestPoseConventionFacts(
                controller_hmd_source=QuestRawPoseSource.UNITY_LH,
                body_source=QuestRawPoseSource.OVR_NATIVE_RH,
            ),
            QUEST_APK_BUILD_1_4_0_LIFECYCLE: QuestPoseConventionFacts(
                controller_hmd_source=QuestRawPoseSource.UNITY_LH,
                body_source=QuestRawPoseSource.OVR_NATIVE_RH,
            ),
            QUEST_APK_BUILD_1_4_1: QuestPoseConventionFacts(
                controller_hmd_source=QuestRawPoseSource.UNITY_LH,
                body_source=QuestRawPoseSource.OVR_NATIVE_RH,
            ),
            QUEST_APK_BUILD_1_4_2: QuestPoseConventionFacts(
                controller_hmd_source=QuestRawPoseSource.UNITY_LH,
                body_source=QuestRawPoseSource.OVR_NATIVE_RH,
            ),
            QUEST_APK_BUILD_1_4_3: QuestPoseConventionFacts(
                controller_hmd_source=QuestRawPoseSource.UNITY_LH,
                body_source=QuestRawPoseSource.OVR_NATIVE_RH,
            ),
            QUEST_APK_BUILD_1_4_4: QuestPoseConventionFacts(
                controller_hmd_source=QuestRawPoseSource.UNITY_LH,
                body_source=QuestRawPoseSource.OVR_NATIVE_RH,
            ),
            QUEST_APK_BUILD_1_4_5: QuestPoseConventionFacts(
                controller_hmd_source=QuestRawPoseSource.UNITY_LH,
                body_source=QuestRawPoseSource.OVR_NATIVE_RH,
            ),
        }
    )
)

# New capture is additionally pinned to one registered build. Historical
# registry entries remain resolvable after this active identity advances.
ACTIVE_REAL_QUEST_APK_BUILD_ID: QuestApkBuildId | None = QUEST_APK_BUILD_1_4_5


def resolve_real_quest_pose_convention(
    capture_app: QuestCaptureApp,
) -> QuestPoseConvention:
    """Resolve only source-controlled real-device facts, never simulation."""
    facts = REAL_QUEST_POSE_CONVENTION_FACTS.get(capture_app.apk_build_id)
    if facts is None:
        raise UnknownQuestApkBuildError(
            capture_app.apk_build_id,
            tuple(sorted(REAL_QUEST_POSE_CONVENTION_FACTS)),
        )
    return QuestPoseConvention(
        capture_app=capture_app,
        controller_hmd_source=facts.controller_hmd_source,
        body_source=facts.body_source,
    )


def require_active_real_quest_build(
    capture_app: QuestCaptureApp,
    *,
    active_build_id: QuestApkBuildId | None = ACTIVE_REAL_QUEST_APK_BUILD_ID,
) -> None:
    """Gate new capture separately from historical convention resolution."""
    if active_build_id is None:
        raise QuestPoseConventionNotConfiguredError()
    if capture_app.apk_build_id != active_build_id:
        raise InactiveQuestApkBuildError(capture_app.apk_build_id, active_build_id)


__all__ = [
    "ACTIVE_REAL_QUEST_APK_BUILD_ID",
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
    "REAL_QUEST_POSE_CONVENTION_FACTS",
    "SIMULATED_QUEST_APK_BUILD_ID",
    "SIMULATED_QUEST_CAPTURE_APP",
    "SIMULATED_QUEST_POSE_CONVENTION",
    "InactiveQuestApkBuildError",
    "QuestApkBuildId",
    "QuestApkBuildIdError",
    "QuestCaptureApp",
    "QuestCaptureAppError",
    "QuestPoseConvention",
    "QuestPoseConventionError",
    "QuestPoseConventionFacts",
    "QuestPoseConventionNotConfiguredError",
    "QuestRawPoseSource",
    "UnknownQuestApkBuildError",
    "require_active_real_quest_build",
    "resolve_real_quest_pose_convention",
]
